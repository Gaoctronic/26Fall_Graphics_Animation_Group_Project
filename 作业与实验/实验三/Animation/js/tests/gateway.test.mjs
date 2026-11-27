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

const { createCSpaceChecker } = await import(url('js/cspace-collision.js'));
const { createDSLRunner } = await import(url('js/dsl-interpreter.js'));
const { createSmartGateway, createDSLDriver, buildDSLFromJointPath, parseNaturalLanguageTask, GATEWAY_STATUS } = await import(url('js/smart-gateway.js'));

console.log('--- [1] 自然语言与 JSON 任务解析入口 ---');
{
  const nl1 = parseNaturalLanguageTask('抓取红色方块并放置到 B 区，避开柱子障碍');
  assert.equal(nl1.ok, true);
  assert.equal(nl1.taskSpec.task, 'pick_and_place');
  assert.equal(nl1.taskSpec.target.color, 'red');
  assert.equal(nl1.taskSpec.destination, 'zone_B');
  assert.deepEqual(nl1.taskSpec.constraints.avoid, ['pillar_1', 'pillar_2']);

  const nl2 = parseNaturalLanguageTask('对工件执行360度环绕拍摄');
  assert.equal(nl2.ok, true);
  assert.equal(nl2.taskSpec.task, 'orbit_scan');

  const jsonRes = parseNaturalLanguageTask('{"task":"avoid_move","goalQ":[0.5,0.2]}');
  assert.equal(jsonRes.ok, true);
  assert.equal(jsonRes.taskSpec.task, 'avoid_move');

  const bad = parseNaturalLanguageTask('随便说一句话');
  assert.equal(bad.ok, false);
  console.log('PASS: 自然语言与结构化 JSON 解析验证通过');
}

console.log('--- [2] 智能网关状态机全生命周期：IDLE → PLANNING → EXECUTING → DONE ---');
{
  const checker = createCSpaceChecker(null, [
    { id: 'obs-1', type: 'cspace-box', min: [-0.2, -0.8], max: [0.2, 0.8] },
  ], {
    limits: [
      { lower: -2, upper: 2, name: 'J1' },
      { lower: -2, upper: 2, name: 'J2' },
    ],
  });

  const events = [];
  const gw = createSmartGateway(null, {
    checker,
    driverExecute: async ({ planResult }) => ({
      ok: true,
      executedWaypoints: planResult.path.length,
    }),
  });
  gw.subscribe((ev) => events.push(ev.status));

  assert.equal(gw.getState().status, GATEWAY_STATUS.IDLE);

  const res = await gw.submitTask({
    task: 'avoid_move',
    startQ: [-1.0, 0.0],
    goalQ: [1.0, 0.0],
    algorithm: 'RRT*',
  }, { seed: 42, maxIterations: 600, stopOnFirstSolution: true });

  assert.equal(res.ok, true);
  assert.equal(gw.getState().status, GATEWAY_STATUS.DONE);
  assert.deepEqual(events, [GATEWAY_STATUS.PLANNING, GATEWAY_STATUS.EXECUTING, GATEWAY_STATUS.DONE]);

  // 复位回到 IDLE
  gw.reset();
  assert.equal(gw.getState().status, GATEWAY_STATUS.IDLE);
  console.log('PASS: 正常生命周期流转 IDLE → PLANNING → EXECUTING → DONE → IDLE 验证通过');
}

console.log('--- [3] 失败分支兜底：规划失败转 ERROR 并携带原因，可恢复 IDLE ---');
{
  const checker = createCSpaceChecker(null, [
    { id: 'block-goal', type: 'cspace-box', min: [0.8, -0.5], max: [1.2, 0.5] },
  ], {
    limits: [
      { lower: -2, upper: 2, name: 'J1' },
      { lower: -2, upper: 2, name: 'J2' },
    ],
  });

  const events = [];
  const gw = createSmartGateway(null, { checker });
  gw.subscribe((ev) => events.push(ev));

  const failRes = await gw.submitTask({
    task: 'avoid_move',
    startQ: [-1.0, 0.0],
    goalQ: [1.0, 0.0], // 落在障碍物内
  });

  assert.equal(failRes.ok, false);
  assert.equal(failRes.status, GATEWAY_STATUS.ERROR);
  assert.equal(gw.getState().status, GATEWAY_STATUS.ERROR);
  assert.match(gw.getState().lastError, /终点位形发生碰撞/);
  assert.deepEqual(events.map((e) => e.status), [GATEWAY_STATUS.PLANNING, GATEWAY_STATUS.ERROR]);

  gw.reset();
  assert.equal(gw.getState().status, GATEWAY_STATUS.IDLE);
  console.log('PASS: 规划失败安全转 ERROR 并携带诊断原因，复位后恢复 IDLE');
}

console.log('--- [4] 可插拔后处理器钩子（组员 C 接口对接验证） ---');
{
  const checker = createCSpaceChecker(null, [], {
    limits: [
      { lower: -2, upper: 2, name: 'J1' },
      { lower: -2, upper: 2, name: 'J2' },
    ],
  });
  const gw = createSmartGateway(null, { checker, driverExecute: async () => ({ ok: true }) });
  // 注册一个模拟后处理器：将路径直接剪枝为首尾两点
  gw.registerHook('postProcessor', (pathQ) => ({
    ok: true,
    path: [pathQ[0], pathQ[pathQ.length - 1]],
  }));
  const res = await gw.submitTask({
    task: 'avoid_move',
    startQ: [-1.0, -1.0],
    goalQ: [1.0, 1.0],
    algorithm: 'RRT',
  }, { seed: 7, maxIterations: 400 });

  assert.equal(res.ok, true);
  assert.equal(res.planResult.postProcessed, true);
  assert.equal(res.planResult.path.length, 2);
  console.log('PASS: 后处理器钩子接入与二次碰撞复验通过');
}

console.log('--- [5] 网关默认驱动钩子对接实验二 DSL 执行器：未到位不 DONE，到位才 DONE，异常转 ERROR ---');
{
  const limits5 = [
    { lower: -2, upper: 2, name: 'J1' },
    { lower: -2, upper: 2, name: 'J2' },
    { lower: -1.5, upper: 1.5, name: 'J3' },
    { lower: -2, upper: 2, name: 'J4' },
    { lower: -3, upper: 3, name: 'J5' },
  ];
  const makeMockSim = () => {
    const joints = limits5.map((l) => ({ ...l, angle: 0, value: 0 }));
    let target = null;
    let allowedFlag = true;
    const motion = {
      state: { speed: 100, programActive: false, stopVersion: 0, collisionCount: 0, collisionReason: '' },
      allowed: () => allowedFlag,
      setAllowed: (v) => { allowedFlag = v; },
      busy: () => target !== null,
      moveJ: (angles) => { target = angles.slice(); },
      stop: () => { target = null; motion.state.stopVersion += 1; },
      stepToTarget: (fraction = 0.5) => {
        if (!target) return;
        let done = true;
        for (let i = 0; i < joints.length; i += 1) {
          const diff = target[i] - joints[i].angle;
          if (Math.abs(diff) <= 0.01) {
            joints[i].angle = target[i];
            joints[i].value = target[i];
          } else {
            joints[i].angle += diff * fraction;
            joints[i].value = joints[i].angle;
            done = false;
          }
        }
        if (done) target = null;
      },
    };
    const api = { robot: { visible: true }, joints, motion };
    const runner = createDSLRunner(api, motion);
    api.program = { runner };
    return { api, motion, runner };
  };

  // 5a: 正常执行 —— 必须逐帧到位后才转 DONE
  const { api, motion, runner } = makeMockSim();
  const checker = createCSpaceChecker(null, [], { limits: limits5 });
  const gw = createSmartGateway(api, { checker, driverExecute: createDSLDriver(api) });
  const goalQ = [0.4, 0.2, -0.1, 0.15, 0.3];

  const taskPromise = gw.submitTask({
    task: 'avoid_move',
    startQ: [0, 0, 0, 0, 0],
    goalQ,
    algorithm: 'RRT*',
  }, { seed: 42, maxIterations: 300, stopOnFirstSolution: true });

  // 提交后立刻检查：必须停留在 EXECUTING，绝不能未运动就 DONE
  await new Promise((r) => setTimeout(r, 10));
  assert.equal(gw.getState().status, GATEWAY_STATUS.EXECUTING, '机械臂尚未到位时网关必须处于 EXECUTING');
  assert.equal(runner.state.running, true);

  // 逐步推进直到真正到位
  for (let i = 0; i < 60 && runner.state.running; i += 1) {
    motion.stepToTarget(0.5);
    runner.update();
    gw.tick();
  }
  const doneRes = await taskPromise;
  assert.equal(doneRes.ok, true);
  assert.equal(gw.getState().status, GATEWAY_STATUS.DONE);
  assert.ok(doneRes.execResult.maxJointError < 0.01);

  // 5b: 执行中途触发急停（allowed -> false）→ 立即转 ERROR 且带明确原因
  gw.reset();
  const estopPromise = gw.submitTask({
    task: 'avoid_move',
    goalQ: [-0.4, -0.2, 0.1, 0, 0],
    algorithm: 'RRT*',
  }, { seed: 42, maxIterations: 300, stopOnFirstSolution: true });
  await new Promise((r) => setTimeout(r, 10));
  assert.equal(gw.getState().status, GATEWAY_STATUS.EXECUTING);
  motion.setAllowed(false);
  runner.update();
  gw.tick();
  const estopRes = await estopPromise;
  assert.equal(estopRes.ok, false);
  assert.equal(gw.getState().status, GATEWAY_STATUS.ERROR);
  assert.match(gw.getState().lastError, /急停|未使能/);

  // 5c: 未使能状态下直接提交 → 启动阶段即拦截至 ERROR 且带明确原因
  gw.reset();
  const disabledRes = await gw.submitTask({
    task: 'avoid_move',
    goalQ: [0.1, 0.1, 0, 0, 0],
    algorithm: 'A*',
  });
  assert.equal(disabledRes.ok, false);
  assert.equal(gw.getState().status, GATEWAY_STATUS.ERROR);
  assert.match(gw.getState().lastError, /未使能|急停/);

  // 校验 buildDSLFromJointPath 输出格式
  const built = buildDSLFromJointPath([[0, 0, 0, 0, 0], goalQ], { currentAngles: [0, 0, 0, 0, 0], speedPercent: 80 });
  assert.match(built.dslScript, /SET SPEED 80/);
  assert.match(built.dslScript, /MOVJ P1/);
  console.log('PASS: 网关对接实验二 DSL 执行器（到位才 DONE + 急停/未使能转 ERROR）验证通过');
}

console.log('--- [5] 规划路点完整保留，超过 DSL 容量时返回错误 ---');
{
  const path = [[0, 0], [0, 0.5], [0.5, 0.5], [0.5, 0]];
  const options = { currentAngles: path[0] };
  const program = buildDSLFromJointPath(path, options);
  assert.deepEqual(program.poses.map(({ angles }) => angles), path.slice(1));
  assert.throws(() => buildDSLFromJointPath(path, { ...options, maxWaypoints: 2 }), /路点数超过/);
}

console.log('--- [6] 未接入驱动时返回执行错误 ---');
{
  const checker = createCSpaceChecker(null, [], {
    limits: [{ lower: -2, upper: 2 }, { lower: -2, upper: 2 }],
  });
  const gw = createSmartGateway(null, { checker });
  const result = await gw.submitTask({
    task: 'avoid_move', startQ: [0, 0], goalQ: [0.2, 0], algorithm: 'A*',
  });
  assert.equal(result.status, GATEWAY_STATUS.ERROR);
  assert.equal(result.stage, 'EXECUTING');
  assert.match(result.reason, /未接入机械臂指令执行器/);
}

console.log('ALL SMART GATEWAY TESTS PASSED.');
