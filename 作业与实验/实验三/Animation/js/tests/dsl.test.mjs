import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const cache = new Map();
function url(file) {
  if (cache.has(file)) return cache.get(file);
  const source = fs.readFileSync(path.join(root, file), 'utf8').replace(
    /from '([^']+)'/g,
    (_, name) => `from '${url(name === 'three' ? 'js/vendor/three/build/three.core.js' : path.posix.normalize(path.posix.join(path.posix.dirname(file), name)))}'`,
  );
  const value = `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`;
  cache.set(file, value);
  return value;
}

const THREE = await import(url('js/vendor/three/build/three.core.js'));
const { createMotion } = await import(url('js/pendant-motion.js'));
const { createDSLRunner, parseDSLScript, DSL_STATUS } = await import(url('js/dsl-interpreter.js'));

function makeFixture() {
  const robot = new THREE.Object3D();
  const joints = [120, 120, 90, 120, 180].map((limit, i) => {
    const node = new THREE.Object3D();
    robot.add(node);
    return {
      name: `J${i + 1}`, node, type: 'revolute', axis: new THREE.Vector3(0, 1, 0),
      angle: 0, restQuaternion: node.quaternion.clone(),
      lower: -limit * Math.PI / 180, upper: limit * Math.PI / 180,
    };
  });
  const tcp = new THREE.Object3D();
  robot.add(tcp);
  const api = { robot, joints, tcp };
  const motion = createMotion(api, { joints: [], root: new THREE.Object3D() });
  motion.state.enabled = true;
  const runner = createDSLRunner(api, motion, {
    grip: () => ({ state: 'CLOSE', attached: false }),
  });
  const poses = [
    { name: 'P1', angles: [0, 0, 0, 0, 0] },
    { name: 'P2', angles: [0.3, 0, 0, 0, 0] },
  ];
  return { api, motion, runner, poses };
}

test('PAUSED 状态下若外部调用 motion.stop() 或失能急停，状态机立即解除运行锁', () => {
  const { motion, runner, poses } = makeFixture();
  runner.start('MOVJ P2\nWAIT 1\nMOVJ P1', poses);
  assert.equal(runner.pause(), true);
  assert.equal(runner.state.status, DSL_STATUS.PAUSED);

  // 模拟切换页签或点击停止运动
  motion.stop();
  runner.update();
  assert.equal(runner.state.running, false);
  assert.equal(runner.state.status, DSL_STATUS.IDLE);
  assert.equal(motion.state.programActive, false);

  // 模拟 PAUSED 时触发急停
  runner.start('MOVJ P2\nMOVJ P1', poses);
  runner.pause();
  motion.state.estop = true;
  runner.update();
  assert.equal(runner.state.running, false);
  assert.equal(runner.state.status, DSL_STATUS.ESTOP);
});

test('SET SPEED 下限与滑块 10% 一致，且 GRIP 空夹记录持久告警 lastAlert', () => {
  const { motion, runner, poses } = makeFixture();
  assert.equal(parseDSLScript('SET SPEED 5', poses).ok, false, '小于 10% 的倍率应被语法检查拦截');
  assert.equal(parseDSLScript('SET SPEED 40', poses).ok, true);

  runner.start('SET SPEED 40\nGRIP 1\nMOVJ P2', poses);
  assert.equal(motion.state.speed, 40);
  runner.update(); // 执行 GRIP 1
  assert.match(runner.state.lastAlert, /空夹/);
  for (let i = 0; i < 500 && runner.state.running; i += 1) {
    motion.update(0.05);
    runner.update();
  }
  assert.equal(runner.state.status, DSL_STATUS.DONE);
  assert.match(runner.state.lastAlert, /空夹/, '后续指令完成后仍保留空夹告警供界面展示');
});

test('空 DSL 或仅含注释的脚本在检查与执行时均统一拦截为程序为空', () => {
  const { runner, poses } = makeFixture();
  for (const blank of ['', '   \n# 只有注释\n  // 注释']) {
    const parsed = parseDSLScript(blank, poses);
    assert.equal(parsed.ok, false);
    assert.match(parsed.errors[0].message, /程序为空/);
    assert.throws(() => runner.start(blank, poses), /程序为空/);
  }
});

test('引用超限位姿在语法检查与启动时均前置拦截，且防碰撞中断保留具体原因', () => {
  const { api, motion, runner, poses } = makeFixture();
  const outPoses = [
    ...poses,
    { name: 'P3', angles: [0, 0, Math.PI, 0, 0] }, // J3 限位 ±90°，180° 超限
  ];
  const parsed = parseDSLScript('MOVJ P1\nMOVJ P3', outPoses, api.joints);
  assert.equal(parsed.ok, false);
  assert.match(parsed.errors[0].message, /J3.*超出关节限位/);
  assert.throws(() => runner.start('MOVJ P1\nMOVJ P3', outPoses), /J3.*超出关节限位/);

  // 模拟运动期间触发防碰撞保护导致 stop()
  runner.start('MOVJ P2\nMOVJ P1', poses);
  motion.state.collisionCount += 1;
  motion.state.collisionReason = '夹爪与「工作台」发生穿透';
  motion.stop();
  runner.update();
  assert.equal(runner.state.running, false);
  assert.match(runner.state.message, /防碰撞保护中断.*夹爪与「工作台」发生穿透/);
});
