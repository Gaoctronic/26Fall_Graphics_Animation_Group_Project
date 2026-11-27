import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
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
const { createCSpaceChecker } = await import(url('js/cspace-collision.js'));
const { planAStar, planRRT, planRRTStar, computePathLength, createSeededRNG } = await import(url('js/path-planner.js'));

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const jointMapPath = path.resolve(__dirname, '../../images/joint_map.json');
const jointMap = JSON.parse(fs.readFileSync(jointMapPath, 'utf-8'));

// 辅助函数：构建一个与 joint_map.json 同构的轻量 5-DOF 运动链（无需 WebGL / GLB）
function buildSynthetic5DOFRobot() {
  const root = new THREE.Group();
  root.name = 'robot-arm';
  const joints = [];
  let parent = root;
  const offsets = [
    [0, 0.3, 0],
    [0, 0.5, 0],
    [0, 1.2, 0],
    [0, 1.0, 0],
    [0, 0.4, 0],
  ];
  const axes = [
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 1, 0),
  ];
  jointMap.joints.forEach((entry, idx) => {
    const node = new THREE.Group();
    node.name = entry.name;
    node.position.set(...offsets[idx]);
    parent.add(node);
    joints.push({
      name: entry.name,
      type: 'revolute',
      node,
      axis: axes[idx],
      restQuaternion: node.quaternion.clone(),
      restPosition: node.position.clone(),
      lower: THREE.MathUtils.degToRad(entry.limit.lower),
      upper: THREE.MathUtils.degToRad(entry.limit.upper),
      value: 0,
      angle: 0,
    });
    parent = node;
  });
  const tcp = new THREE.Group();
  tcp.name = 'TCP';
  tcp.position.set(0, 0.25, 0);
  parent.add(tcp);
  root.updateMatrixWorld(true);
  return { robot: root, joints, tcp, floorY: 0 };
}

console.log('--- [1] C-space 碰撞检测：正例、反例、关节限位与细杆沿段离散拦截 ---');
{
  const limits2D = [
    { lower: -2.0, upper: 2.0, name: 'J1' },
    { lower: -2.0, upper: 2.0, name: 'J2' },
  ];
  // 在 (0, 0) 放置一道薄墙禁区 x ∈ [-0.06, 0.06], y ∈ [-1.2, 1.2]
  const checker = createCSpaceChecker(null, [
    { id: 'thin-wall', type: 'cspace-box', min: [-0.06, -1.2], max: [0.06, 1.2] },
  ], { limits: limits2D, edgeStepRad: 0.04 });

  // 正例：自由位形
  assert.equal(checker.isConfigFree([-1.0, 0.0]).free, true);
  assert.equal(checker.isConfigFree([1.0, 0.0]).free, true);
  // 反例：障碍内与超限
  assert.equal(checker.isConfigFree([0.0, 0.0]).free, false);
  assert.equal(checker.isConfigFree([2.5, 0.0]).free, false);
  // 细杆穿模拦截：起终点都自由，但直线穿过薄墙，必须被沿段离散打点拦截
  const edgeRes = checker.isEdgeFree([-1.0, 0.0], [1.0, 0.0], 0.04);
  assert.equal(edgeRes.free, false, '细墙应被沿段离散检测拦截');
  assert.equal(edgeRes.obstacleId, 'thin-wall');
  // 绕过墙顶的边应自由
  assert.equal(checker.isEdgeFree([-1.0, 1.5], [1.0, 1.5], 0.04).free, true);
  assert.ok(checker.getStats().configChecks > 10);
  console.log('PASS: C-space 2D 正反例与薄墙离散防穿模验证通过');
}

console.log('--- [2] 3D 机械臂全连杆胶囊体与工作空间圆柱障碍检测 ---');
{
  const sim = buildSynthetic5DOFRobot();
  // 在机械臂前伸必经路径上放一根竖直柱子
  const pillar = { id: 'pillar_1', type: 'cylinder', position: [0, 1.5, 1.0], radius: 0.25, height: 3.0 };
  const checker = createCSpaceChecker(sim, [pillar], { checkFloor: true, safetyMargin: 0.03, linkRadius: 0.05 });

  // 零位竖直向上应无碰
  assert.equal(checker.isConfigFree([0, 0, 0, 0, 0]).free, true, '竖直零位应无碰撞');
  // J2 前倾 ~45° 朝 +Z 方向伸展，连杆应撞到 pillar_1
  const bentRes = checker.isConfigFree([0, 0.7, 0, 0, 0]);
  assert.equal(bentRes.free, false, '前倾撞柱工况应被拦截');
  assert.equal(bentRes.obstacleId, 'pillar_1');
  // 确认碰撞查询后原始关节角已无副作用还原
  assert.deepEqual(sim.joints.map((j) => j.value), [0, 0, 0, 0, 0]);
  console.log('PASS: 3D 机械臂全连杆胶囊体与圆柱障碍拦截验证通过');
}

console.log('--- [3] A* 网格搜索：绕障可行性与全段无碰撞校验 ---');
{
  const limits2D = [
    { lower: -2.0, upper: 2.0, name: 'J1' },
    { lower: -2.0, upper: 2.0, name: 'J2' },
  ];
  const checker = createCSpaceChecker(null, [
    { id: 'center-block', type: 'cspace-box', min: [-0.35, -1.1], max: [0.35, 1.1] },
  ], { limits: limits2D, edgeStepRad: 0.04 });

  const startQ = [-1.2, 0.0];
  const goalQ = [1.2, 0.0];
  const res = planAStar(startQ, goalQ, checker, {
    resolution: 0.15,
    neighborMode: 'full',
    heuristicWeight: 1.0,
  });

  assert.equal(res.ok, true, `A* 应找到绕障路径，实际原因: ${res.reason}`);
  assert.ok(res.path.length >= 3);
  assert.ok(res.rawLength > 2.4, '绕障路径长度应大于直线距离 2.4');
  // 校验路径每一段均严格无碰撞
  for (let i = 1; i < res.path.length; i += 1) {
    assert.equal(checker.isEdgeFree(res.path[i - 1], res.path[i], 0.03).free, true, `A* 第 ${i} 段应无碰撞`);
  }
  console.log(`PASS: A* 绕障成功 | 路点数=${res.path.length} | 弧长=${res.rawLength.toFixed(3)} | 耗时=${res.planningTimeMs.toFixed(2)}ms | 碰撞查询=${res.collisionChecks}`);
}

console.log('--- [4] RRT vs RRT*：固定种子可复现性与渐近最优重连收益 ---');
{
  const limits2D = [
    { lower: -2.0, upper: 2.0, name: 'J1' },
    { lower: -2.0, upper: 2.0, name: 'J2' },
  ];
  const checker = createCSpaceChecker(null, [
    { id: 'mid-obs', type: 'cspace-sphere', center: [0, 0], radius: 0.65 },
  ], { limits: limits2D, edgeStepRad: 0.04 });

  const startQ = [-1.4, -0.2];
  const goalQ = [1.4, 0.2];

  // 同种子可复现性验证
  const rrtA = planRRT(startQ, goalQ, checker, { seed: 2026, stepSize: 0.25, maxIterations: 2000 });
  const rrtB = planRRT(startQ, goalQ, checker, { seed: 2026, stepSize: 0.25, maxIterations: 2000 });
  assert.equal(rrtA.ok, true);
  assert.equal(rrtA.rawLength, rrtB.rawLength, '相同随机种子下 RRT 结果必须完全可复现');

  // RRT* 重连优化验证（多颗种子平均路径长度应明显优于普通 RRT）
  let rrtSum = 0;
  let rrtStarSum = 0;
  let validCount = 0;
  for (let s = 101; s <= 110; s += 1) {
    const r1 = planRRT(startQ, goalQ, checker, { seed: s, stepSize: 0.25, maxIterations: 1500 });
    const r2 = planRRTStar(startQ, goalQ, checker, { seed: s, stepSize: 0.25, maxIterations: 600 });
    if (r1.ok && r2.ok) {
      for (let i = 1; i < r2.path.length; i += 1) {
        assert.equal(checker.isEdgeFree(r2.path[i - 1], r2.path[i], 0.03).free, true, 'RRT* 路径每段必须无碰撞');
      }
      rrtSum += r1.rawLength;
      rrtStarSum += r2.rawLength;
      validCount += 1;
    }
  }
  assert.ok(validCount >= 8, `10 组种子应至少 8 组成功，实际 ${validCount}`);
  const rrtAvg = rrtSum / validCount;
  const rrtStarAvg = rrtStarSum / validCount;
  assert.ok(rrtStarAvg < rrtAvg, `RRT* 平均路径长度 (${rrtStarAvg.toFixed(3)}) 应短于 RRT (${rrtAvg.toFixed(3)})`);
  console.log(`PASS: RRT vs RRT*（${validCount}/10 组对比）| RRT 均长=${rrtAvg.toFixed(3)} → RRT* 均长=${rrtStarAvg.toFixed(3)}（缩短 ${((1 - rrtStarAvg / rrtAvg) * 100).toFixed(1)}%）`);
}

console.log('--- [5] 5-DOF 真实关节限位空间 RRT* 避障规划与不可达超时兜底 ---');
{
  const sim = buildSynthetic5DOFRobot();
  const pillar = { id: 'pillar_1', type: 'cylinder', position: [0.0, 1.5, 1.1], radius: 0.28, height: 3.0 };
  const checker = createCSpaceChecker(sim, [pillar], { checkFloor: true, safetyMargin: 0.03, linkRadius: 0.05 });

  // 起点与终点分别在柱子左侧与右侧
  const startQ = [-0.6, 0.55, 0.2, 0.0, 0.0];
  const goalQ = [0.6, 0.55, 0.2, 0.0, 0.0];
  assert.equal(checker.isConfigFree(startQ).free, true);
  assert.equal(checker.isConfigFree(goalQ).free, true);

  const res5D = planRRTStar(startQ, goalQ, checker, {
    seed: 77,
    stepSize: 0.35,
    maxIterations: 800,
    goalBias: 0.12,
    stopOnFirstSolution: true,
  });
  assert.equal(res5D.ok, true, `5-DOF 避柱规划应成功，实际原因: ${res5D.reason}`);
  assert.ok(res5D.costHistory.length >= 1);
  for (let i = 1; i < res5D.path.length; i += 1) {
    assert.equal(checker.isEdgeFree(res5D.path[i - 1], res5D.path[i], 0.05).free, true);
  }
  console.log(`PASS: 5-DOF 真实限位避柱规划成功 | 路点数=${res5D.path.length} | 弧长=${computePathLength(res5D.path).toFixed(3)} | 耗时=${res5D.planningTimeMs.toFixed(1)}ms`);

  // 不可达超时/迭代上限兜底测试（目标直接放在柱子内）
  const blockedRes = planRRTStar(startQ, [0, 0.7, 0, 0, 0], checker, { seed: 1, maxIterations: 50 });
  assert.equal(blockedRes.ok, false);
  assert.match(blockedRes.reason, /终点位形发生碰撞/);
  console.log('PASS: 不可达目标安全返回失败原因（不卡死）');
}

// 验证 createSeededRNG 确定性
const rng1 = createSeededRNG(12345);
const rng2 = createSeededRNG(12345);
for (let i = 0; i < 5; i += 1) assert.equal(rng1(), rng2());
console.log('ALL C-SPACE & PLANNER TESTS PASSED.');
