import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

// Use the project's pinned Three.js in Node, without a package install or DOM.
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const cache = new Map();
function url(file) {
  if (cache.has(file)) return cache.get(file);
  const source = fs.readFileSync(path.join(root, file), 'utf8')
    .replace("import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';",
      'class GLTFLoader { loadAsync() { throw new Error("GLB loading is covered by browser checks"); } }')
    .replace(/from '([^']+)'/g, (_, name) => {
      const target = name === 'three' ? 'js/vendor/three/build/three.core.js'
        : name.startsWith('three/addons/') ? name.replace('three/addons/', 'js/vendor/three/addons/')
          : path.posix.normalize(path.posix.join(path.posix.dirname(file), name));
      return `from '${url(target)}'`;
    });
  const value = `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`;
  cache.set(file, value);
  return value;
}

const THREE = await import(url('js/vendor/three/build/three.core.js'));
const { Workstation } = await import(url('js/Workstation.js'));
const { planThreeStageGrasp, simulateEffortGrip } = await import(url('js/grasp-planner.js'));
const { createCSpaceChecker } = await import(url('js/cspace-collision.js'));
const close = (actual, expected, tolerance = 1e-8) => assert(Math.abs(actual - expected) < tolerance, `${actual} != ${expected}`);
const closeVector = (actual, expected) => assert(actual.distanceTo(expected) < 1e-6, `${actual.toArray()} != ${expected.toArray()}`);
const world = (node) => { node.updateWorldMatrix(true, false); return node.matrixWorld.clone(); };
const closeMatrix = (actual, expected) => actual.elements.forEach((value, i) => close(value, expected.elements[i]));

function fixture({ width = .6, options = {} } = {}) {
  const scene = new THREE.Scene(), flange = new THREE.Group();
  scene.add(flange);
  const station = new Workstation(scene, flange, options);
  station.blockSize = .6;
  station.tableHeight = 1;
  const tableMesh = new THREE.Mesh(new THREE.BoxGeometry(6, 1, 4));
  tableMesh.position.y = .5;
  station.table.add(tableMesh);
  station.table.position.set(2, 0, -3);
  scene.add(station.table);
  station.workpiece = new THREE.Mesh(new THREE.BoxGeometry(width, .6, .6));
  station.workpiece.name = 'workpiece';
  station.placementMatrix = new THREE.Matrix4().makeTranslation(0, 1.3, 1.5);
  station.createGripper();
  station.resetWorkpiece();
  station.createObstacleLayout();
  return { scene, flange, station, wp: station.workpiece };
}

function alignGrip({ flange, wp }) {
  wp.updateWorldMatrix(true, false);
  flange.position.copy(new THREE.Vector3(0, .3, 0).applyMatrix4(wp.matrixWorld));
  flange.quaternion.copy(wp.getWorldQuaternion(new THREE.Quaternion()));
  flange.updateWorldMatrix(true, true);
}

test('obstacle specs describe the two rendered pillars and table in world coordinates', () => {
  const { station } = fixture();
  const specs = station.getObstacleSpecs();
  assert.deepEqual(specs.map((spec) => spec.id).sort(), ['pillar_1', 'pillar_2', 'table']);
  for (const spec of specs) {
    const node = spec.id === 'table' ? station.table : station.obstacles.getObjectByName(spec.id);
    const bounds = new THREE.Box3().setFromObject(node);
    closeVector(new THREE.Vector3(...spec.position), bounds.getCenter(new THREE.Vector3()));
    const size = bounds.getSize(new THREE.Vector3());
    close(spec.margin, .03);
    if (spec.type === 'cylinder') {
      close(spec.radius, size.x / 2); close(spec.height, size.y);
      close(bounds.min.y, 1, 1e-6); // Float32 geometry vertices.
    } else {
      assert.equal(spec.type, 'box');
      closeVector(new THREE.Vector3(...spec.size), size);
    }
  }
  specs[0].position[0] = 999;
  assert.notEqual(station.getObstacleSpecs()[0].position[0], 999);
  station.createObstacleLayout();
  assert.equal(station.obstacles.children.length, 2, 'layout initialization is idempotent');
});

test('inflation shells match collision margins without enlarging the real table bounds', () => {
  const { station } = fixture();
  const before = new THREE.Box3().setFromObject(station.table);
  station.setObstacleInflationVisible(true, .15);
  assert.equal(station.obstacleInflation.visible, true);
  assert.equal(station.obstacleInflation.children.length, 3);
  for (const spec of station.getObstacleSpecs()) {
    const shell = station.obstacleInflation.getObjectByName(`${spec.id}-inflation`);
    assert(shell.material.transparent); assert(shell.material.opacity > 0 && shell.material.opacity < 1);
    assert.equal(shell.material.depthWrite, false);
    const bounds = new THREE.Box3().setFromObject(shell);
    closeVector(bounds.getCenter(new THREE.Vector3()), new THREE.Vector3(...spec.position));
    const physicalSize = spec.type === 'cylinder' ? [2 * spec.radius, spec.height, 2 * spec.radius] : spec.size;
    closeVector(bounds.getSize(new THREE.Vector3()), new THREE.Vector3(...physicalSize.map((v) => v + .3)));
    close(spec.margin, .15);
  }
  const after = new THREE.Box3().setFromObject(station.table);
  closeVector(after.min, before.min); closeVector(after.max, before.max);
  station.setObstacleInflationVisible(false);
  assert.equal(station.obstacleInflation.visible, false);
  assert(station.getObstacleSpecs().every((spec) => spec.margin === .15), 'hiding shells does not disable safety margins');
  assert.throws(() => station.setObstacleInflationVisible(true, -.1));
});

test('zone B returns a world surface pose with independent Vector3 and Quaternion values', () => {
  const { station } = fixture();
  const pose = station.getZoneBPose();
  assert(pose.position.isVector3); assert(pose.quaternion.isQuaternion);
  close(pose.position.y, 1);
  const tableBounds = new THREE.Box3().setFromObject(station.table);
  assert(tableBounds.containsPoint(pose.position));
  assert.equal(station.getObstacleSpecs().some((spec) => spec.id === 'zone_B'), false);
  const original = pose.position.clone();
  pose.position.set(999, 999, 999); pose.quaternion.set(1, 0, 0, 0);
  closeVector(station.getZoneBPose().position, original);
  close(station.getZoneBPose().quaternion.w, 1);
});

test('three-stage grasp uses top-face TCP contact and independent pre-grasp/lift poses', () => {
  const target = { position: [2, 1.3, -1.5], size: [.6, .6, .6] };
  const saved = structuredClone(target);
  const plan = planThreeStageGrasp(target);
  closeVector(plan.grasp.position, new THREE.Vector3(2, 1.6, -1.5));
  closeVector(plan.preGrasp.position, new THREE.Vector3(2, 1.68, -1.5));
  closeVector(plan.lift.position, new THREE.Vector3(2, 1.72, -1.5));
  for (const pose of Object.values(plan)) {
    assert(pose.position.isVector3); assert(pose.quaternion.isQuaternion);
    close(pose.quaternion.length(), 1);
    closeVector(new THREE.Vector3(0, 1, 0).applyQuaternion(pose.quaternion), new THREE.Vector3(0, 1, 0));
  }
  plan.preGrasp.position.y = 999; plan.preGrasp.quaternion.set(1, 0, 0, 0);
  close(plan.grasp.position.y, 1.6); close(plan.grasp.quaternion.w, 1);
  assert.deepEqual(target, saved);
});

test('principal axis controls horizontal jaw orientation and distances are configurable', () => {
  const plan = planThreeStageGrasp({ position: [1, 2, 3], size: [.4, .8, .3], axis: [0, 0, 2] },
    { approachDist: .25, liftHeight: .5 });
  closeVector(new THREE.Vector3(1, 0, 0).applyQuaternion(plan.grasp.quaternion), new THREE.Vector3(0, 0, 1));
  close(plan.grasp.position.y, 2.4); close(plan.preGrasp.position.y, 2.65); close(plan.lift.position.y, 2.9);
  assert.throws(() => planThreeStageGrasp({ position: [0, NaN, 0], size: [1, 1, 1] }));
  assert.throws(() => planThreeStageGrasp({ position: [0, 0, 0], size: [0, 1, 1] }));
  assert.throws(() => planThreeStageGrasp({ position: [0, 0, 0], size: [1, 1, 1] }, { approachDist: -1 }));
});

test('50 N effort grip stops on bilateral contact and preserves the workpiece world pose', () => {
  const f = fixture(); alignGrip(f);
  const before = world(f.wp);
  const result = simulateEffortGrip(f.station);
  assert.equal(result.gripped, true); assert.equal(result.contactForceN, 50);
  assert.equal(typeof result.reason, 'string'); assert.equal(f.wp.parent, f.flange);
  closeMatrix(world(f.wp), before);
  close(f.station.gripStatus.opening, .6, .012);
  close(f.station.contactForceN, 50);
  const again = simulateEffortGrip(f.station);
  assert.equal(again.gripped, true); closeMatrix(world(f.wp), before);
});

test('narrow rotated workpiece stops at its own width and keeps that aperture on updates', () => {
  const notifications = [];
  const f = fixture({ width: .3, options: { onGripChange: (status) => notifications.push(status) } });
  f.wp.rotation.y = .6; alignGrip(f);
  // Calibration markers are decorations, not the contact body's geometry.
  const marker = new THREE.Mesh(new THREE.SphereGeometry(1)); marker.position.y = 2; f.wp.add(marker);
  const result = simulateEffortGrip(f.station, 35);
  assert.equal(result.gripped, true); assert.equal(result.contactForceN, 35);
  close(f.station.gripStatus.opening, .3, .012);
  const opening = f.station.gripStatus.opening;
  f.station.updateGripper(); close(f.station.gripStatus.opening, opening);
  assert.equal(notifications.at(-1).attached, true);
  const before = world(f.wp);
  f.flange.position.y += .2;
  close(world(f.wp).elements[13], before.elements[13] + .2);
  f.station.setGrip('OPEN');
  const released = world(f.wp);
  assert.equal(f.wp.parent, f.scene); close(f.station.contactForceN, 0);
  close(f.station.gripStatus.opening, .9);
  f.flange.position.y += .3; closeMatrix(world(f.wp), released);
  f.station.resetWorkpiece(); close(f.station.contactForceN, 0); close(f.station.gripStatus.opening, .9);
});

test('effort grip rejects empty, one-sided and too-wide grasps without moving the target', () => {
  for (const scenario of ['far', 'one-sided', 'too-wide']) {
    const f = fixture({ width: scenario === 'too-wide' ? 1.2 : .6 });
    alignGrip(f);
    if (scenario === 'far') f.flange.position.z += 2;
    if (scenario === 'one-sided') f.flange.position.x += .12;
    const before = world(f.wp), result = simulateEffortGrip(f.station);
    assert.equal(result.gripped, false, scenario);
    assert.equal(result.contactForceN, scenario === 'one-sided' ? 50 : 0, scenario);
    assert.equal(f.wp.parent, f.scene); closeMatrix(world(f.wp), before);
  }
  const f = fixture(); alignGrip(f);
  assert.throws(() => simulateEffortGrip(f.station, 0));
  assert.throws(() => simulateEffortGrip(f.station, NaN));
});

// A small movable link exercises the actual C-space engine with the exported
// workstation obstacles, avoiding assumptions about this asset's IK reachability.
function probeAt(position) {
  const robot = new THREE.Group(), node = new THREE.Group(), tcp = new THREE.Group();
  node.position.fromArray(position); robot.add(node); tcp.position.y = .05; node.add(tcp);
  const joint = { name: 'probe', type: 'prismatic', node, axis: new THREE.Vector3(1, 0, 0),
    restPosition: node.position.clone(), restQuaternion: node.quaternion.clone(), lower: -10, upper: 10, value: 0 };
  return { robot, tcp, joints: [joint] };
}

test('C-space rejects both rendered pillars and catches a crossing with free endpoints', () => {
  const { station } = fixture();
  const specs = station.getObstacleSpecs();
  for (const pillar of specs.filter((spec) => spec.type === 'cylinder')) {
    const probe = probeAt(pillar.position);
    const checker = createCSpaceChecker(probe, specs, { checkFloor: false, linkRadius: 0 });
    const blocked = checker.isConfigFree([0]);
    assert.equal(blocked.free, false); assert.equal(blocked.obstacleId, pillar.id);
    assert.equal(checker.isConfigFree([-.5]).free, true);
    assert.equal(checker.isConfigFree([.5]).free, true);
    assert.equal(checker.isEdgeFree([-.5], [.5], .02).free, false);
    assert.equal(checker.isEdgeFree([.5], [.8], .02).free, true);
    assert.equal(probe.joints[0].value, 0, 'collision checks restore the current configuration');
  }
});

test('collision margin rejects a near miss even while the inflation visualization is hidden', () => {
  const { station } = fixture();
  station.setObstacleInflationVisible(false, 0);
  const pillar = station.getObstacleSpecs()[0];
  const checker = createCSpaceChecker(probeAt(pillar.position), station.getObstacleSpecs(), { checkFloor: false, linkRadius: 0 });
  const nearMiss = [pillar.radius + .015];
  assert.equal(checker.isConfigFree(nearMiss).free, true);
  station.setObstacleInflationVisible(false, .03);
  checker.setObstacles(station.getObstacleSpecs());
  const result = checker.isConfigFree(nearMiss);
  assert.equal(result.free, false); assert.equal(result.obstacleId, 'pillar_1');
  assert.equal(station.obstacleInflation.visible, false);
});
