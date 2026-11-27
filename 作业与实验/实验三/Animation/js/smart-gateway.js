import { planAStar, planRRT, planRRTStar, computePathLength } from './path-planner.js';
import { DSL_STATUS, DSL_LIMITS } from './dsl-interpreter.js';

/**
 * 实验三智能网关任务状态机（严格对齐讲义 Slide 3）：
 * IDLE → PLANNING → EXECUTING → DONE
 * 任意阶段失败 → ERROR（携带失败原因返回用户层），处理后可 reset() 回到 IDLE。
 */
export const GATEWAY_STATUS = Object.freeze({
  IDLE: 'IDLE',
  PLANNING: 'PLANNING',
  EXECUTING: 'EXECUTING',
  DONE: 'DONE',
  ERROR: 'ERROR',
});

const PATH_ENDPOINT_TOLERANCE_RAD = 1e-6;

function matchesConfiguration(actual, expected, tolerance = PATH_ENDPOINT_TOLERANCE_RAD) {
  return Array.isArray(actual) && Array.isArray(expected) && actual.length === expected.length
    && expected.every((value, index) => Math.abs(actual[index] - value) <= tolerance);
}

/** 将完整的关节路径（弧度）编译为实验二指令和位姿表。 */
export function buildDSLFromJointPath(pathQ, options = {}) {
  if (!Array.isArray(pathQ) || pathQ.length === 0) {
    throw new Error('关节路径为空，无法生成 DSL 指令');
  }
  const hasSpeed = Number.isFinite(options.speedPercent);
  const instructionBudget = DSL_LIMITS.maxSteps - Number(hasSpeed);
  const maxWaypoints = Math.max(1, Math.min(options.maxWaypoints ?? instructionBudget, instructionBudget));
  const waypoints = pathQ.length > 1 && matchesConfiguration(pathQ[0], options.currentAngles)
    ? pathQ.slice(1) : pathQ;
  if (waypoints.length > maxWaypoints) {
    throw new Error('路径路点数超过指令执行器上限（' + maxWaypoints + '），请重新规划或使用经过碰撞验证的简化路径');
  }

  const poses = waypoints.map((angles, idx) => ({
    name: `P${idx + 1}`,
    angles: angles.slice(),
  }));

  const lines = [];
  if (hasSpeed) {
    const speed = Math.min(Math.max(Math.round(options.speedPercent), DSL_LIMITS.minSpeed), DSL_LIMITS.maxSpeed);
    lines.push(`SET SPEED ${speed}`);
  }
  for (const pose of poses) {
    lines.push(`MOVJ ${pose.name}`);
  }
  return {
    dslScript: lines.join('\n'),
    poses,
    goalAngles: waypoints[waypoints.length - 1].slice(),
  };
}

/**
 * 创建对接实验二指令执行器（api.program.runner）的默认驱动钩子。
 * 核心验收契约：
 * 1) 真实调用 runner.start() 驱动机械臂运动，绝不在下发瞬间就返回 DONE；
 * 2) 等 runner 进入 DONE 且逐轴核验机械臂实际关节角到达目标容差内，才返回 { ok: true }；
 * 3) 执行期发生未使能、急停、防碰撞拦截、DSL 语法/规划失败、超时或未到位时，返回 { ok: false, reason }。
 */
export function createDSLDriver(api, driverOptions = {}) {
  const defaultGoalTol = driverOptions.goalToleranceRad ?? PATH_ENDPOINT_TOLERANCE_RAD;
  const defaultTimeoutMs = driverOptions.execTimeoutMs ?? 60000;
  let activeMonitor = null;

  function driverExecute({ taskSpec, planResult, dslBundle, reportProgress, runOptions = {} }) {
    const runner = runOptions.runner ?? api?.program?.runner;
    const motion = runOptions.motion ?? api?.motion;
    if (!runner || typeof runner.start !== 'function') {
      return Promise.resolve({ ok: false, reason: '实验二指令执行器（api.program.runner）未就绪' });
    }
    if (motion && typeof motion.allowed === 'function' && !motion.allowed()) {
      return Promise.resolve({ ok: false, reason: '机械臂未使能或处于急停状态，请先解除急停并勾选允许运动' });
    }
    if (activeMonitor || runner.state.running || motion?.busy?.()) {
      return Promise.resolve({ ok: false, reason: '指令执行器正忙，请先停止当前运动' });
    }

    const currentAngles = Array.isArray(api?.joints)
      ? api.joints.map((joint) => joint.angle ?? joint.value)
      : null;

    const goalTol = runOptions.goalToleranceRad ?? defaultGoalTol;
    const timeoutMs = runOptions.execTimeoutMs ?? defaultTimeoutMs;
    if (!Number.isFinite(goalTol) || goalTol < 0 || !Number.isFinite(timeoutMs) || timeoutMs <= 0) {
      return Promise.resolve({ ok: false, reason: '到位容差或执行超时参数无效' });
    }

    let dslScript;
    let poses;
    let expectedGoal;
    try {
      if (dslBundle) {
        dslScript = dslBundle.dslScript;
        poses = dslBundle.poseMap ?? dslBundle.poses;
        expectedGoal = dslBundle.goalAngles ?? planResult?.path?.at(-1) ?? null;
      } else {
        const path = planResult?.path;
        if (!matchesConfiguration(currentAngles, path?.[0])) {
          throw new Error('机械臂当前位置与规划起点不一致，请从当前关节角重新规划');
        }
        if (!matchesConfiguration(path.at(-1), taskSpec?.goalQ)) {
          throw new Error('规划终点与目标关节角不一致');
        }
        const compiled = buildDSLFromJointPath(path, {
          currentAngles,
          speedPercent: taskSpec?.speedPercent ?? runOptions.speedPercent,
        });
        dslScript = compiled.dslScript;
        poses = compiled.poses;
        expectedGoal = taskSpec.goalQ.slice();
      }
      runner.start(dslScript, poses);
    } catch (err) {
      return Promise.resolve({ ok: false, reason: `指令执行器启动失败：${err?.message ?? String(err)}` });
    }

    const instructions = runner.state.instructions;
    const startedAt = Date.now();

    return new Promise((resolve) => {
      let settled = false;
      let timerId = null;

      const finish = (result) => {
        if (settled) return;
        settled = true;
        if (timerId !== null) clearInterval(timerId);
        if (activeMonitor === check) activeMonitor = null;
        resolve(result);
      };

      const check = () => {
        if (settled) return;
        const st = runner.state;
        if (st.instructions !== instructions) {
          finish({ ok: false, reason: '网关下发的指令程序已被替换' });
          return;
        }
        if (st.running) {
          if (typeof reportProgress === 'function') {
            const pct = 60 + Math.round((st.progress ?? 0) * 38);
            reportProgress(pct, st.message);
          }
          if (Date.now() - startedAt > timeoutMs) {
            runner.stop?.();
            finish({ ok: false, reason: `指令执行超时（>${timeoutMs} ms），已强制停止` });
          }
          return;
        }

        // 执行器已退出运行态：严格区分正常 DONE 与异常中断（E-STOP / 碰撞 / 手动停止 / 执行报错）
        if (st.status === DSL_STATUS.DONE) {
          const finalAngles = Array.isArray(api?.joints)
            ? api.joints.map((joint) => joint.angle ?? joint.value)
            : [];
          let maxJointError = 0;
          if (expectedGoal !== null) {
            if (!Array.isArray(expectedGoal) || !expectedGoal.every(Number.isFinite)
              || !finalAngles.every(Number.isFinite) || finalAngles.length !== expectedGoal.length) {
              finish({ ok: false, reason: '无法核对实际关节角与目标关节角' });
              return;
            }
            for (let i = 0; i < expectedGoal.length; i += 1) {
              const err = Math.abs(finalAngles[i] - expectedGoal[i]);
              if (err > maxJointError) maxJointError = err;
            }
            if (maxJointError > goalTol) {
              finish({
                ok: false,
                reason: `执行器结束但机械臂未到达目标关节角（最大误差 ${maxJointError.toPrecision(4)} rad > 容差 ${goalTol} rad）`,
                finalAngles,
                maxJointError,
              });
              return;
            }
          }
          finish({
            ok: true,
            dslScript,
            executedInstructions: st.instructions.length,
            finalAngles,
            maxJointError,
            runnerMessage: st.message,
          });
          return;
        }

        if (st.status === DSL_STATUS.ESTOP) {
          finish({ ok: false, reason: st.message || '执行期间触发急停或未使能，任务已中断' });
          return;
        }

        finish({ ok: false, reason: st.message || `指令执行器异常中断（状态：${st.status}）` });
      };

      activeMonitor = check;
      // 同步检查一次（针对极短指令或同步测试），再用 25ms 兜底轮询确保即使不在 rAF 下也能收敛
      check();
      if (!settled) {
        timerId = setInterval(check, 25);
      }
    });
  }

  driverExecute.tick = () => {
    if (activeMonitor) activeMonitor();
  };

  return driverExecute;
}

/**
 * 自然语言任务简易解析入口（可选入口，规划层永远只认结构化 JSON 任务）。
 * 支持中文/英文常见关键词，将其映射为标准 { task, target, destination, constraints }。
 */
export function parseNaturalLanguageTask(rawText) {
  const text = String(rawText ?? '').trim();
  if (!text) {
    return { ok: false, error: '任务描述不能为空', taskSpec: null };
  }

  // 若输入本身已是合法 JSON 对象，直接规范化
  if (text.startsWith('{')) {
    try {
      const parsed = JSON.parse(text);
      if (!parsed || typeof parsed !== 'object' || !parsed.task) {
        return { ok: false, error: 'JSON 任务缺少必填字段 "task"', taskSpec: null };
      }
      return { ok: true, error: null, taskSpec: parsed };
    } catch (err) {
      return { ok: false, error: `JSON 语法解析失败：${err.message}`, taskSpec: null };
    }
  }

  const lower = text.toLowerCase();
  const isScan = /环绕|扫描|拍摄|360|orbit|scan/.test(lower);
  const isPick = /抓取|搬运|放置|放到|pick|place|grab/.test(lower);
  const isMove = /移动|规划|避障|move|plan/.test(lower);

  const color = /红|red/.test(lower) ? 'red'
    : /蓝|blue/.test(lower) ? 'blue'
      : /绿|green/.test(lower) ? 'green' : 'orange';
  const shape = /圆柱|cylinder/.test(lower) ? 'cylinder' : 'cube';
  const destination = /b\s*区|zone_b|区域\s*b/i.test(text) ? 'zone_B'
    : /a\s*区|zone_a|区域\s*a/i.test(text) ? 'zone_A' : 'zone_B';

  const avoid = [];
  if (/柱|障碍|避障|pillar|obstacle/.test(lower) || isPick) {
    avoid.push('pillar_1', 'pillar_2');
  }

  let task = null;
  if (isScan && isPick) task = 'full_pipeline';
  else if (isScan) task = 'orbit_scan';
  else if (isPick) task = 'pick_and_place';
  else if (isMove) task = 'avoid_move';

  if (!task) {
    return {
      ok: false,
      error: `无法识别自然语言任务意图：“${text}”，请包含“抓取/放置/环绕扫描/避障移动”或直接输入结构化 JSON`,
      taskSpec: null,
    };
  }

  return {
    ok: true,
    error: null,
    taskSpec: {
      task,
      target: { color, shape },
      destination,
      constraints: { avoid },
      rawText: text,
    },
  };
}

/**
 * 创建三层智能网关控制器（用户需求 API → 路径规划服务 → 机械臂驱动 API）。
 * 浏览器内先用模块内异步调用 + 事件发布订阅模拟 WebSocket 实时推送，保持接口契约不变。
 */
export function createSmartGateway(api = null, options = {}) {
  let status = GATEWAY_STATUS.IDLE;
  let currentTaskId = 0;
  let lastError = null;
  let lastResult = null;
  let progress = 0;
  const listeners = new Set();
  const history = [];

  // 可插拔服务层钩子（供组员 B/C 零冲突注入；传入真实 api 时默认自动对接实验二 DSL 执行器）
  const hooks = {
    checker: options.checker ?? null,
    postProcessor: options.postProcessor ?? null, // 组员 C：path-smoother
    taskDecomposer: options.taskDecomposer ?? null, // 组员 C：task-decomposer
    driverExecute: options.driverExecute !== undefined
      ? options.driverExecute
      : (api ? createDSLDriver(api, options.driverOptions) : null),
  };

  function emit(event) {
    const payload = {
      taskId: currentTaskId,
      status,
      progress,
      timestamp: Date.now(),
      ...event,
    };
    history.push(payload);
    if (history.length > 200) history.shift();
    for (const fn of listeners) {
      try {
        fn(payload);
      } catch {
        // 隔离单个 UI 订阅者异常，不破坏网关主状态机
      }
    }
    return payload;
  }

  function transition(nextStatus, message, extra = {}) {
    status = nextStatus;
    if (typeof extra.progress === 'number') progress = extra.progress;
    return emit({ message, ...extra });
  }

  function failTask(reason, stage = 'PLANNING') {
    lastError = String(reason || '未知错误');
    const res = {
      ok: false,
      taskId: currentTaskId,
      status: GATEWAY_STATUS.ERROR,
      stage,
      reason: lastError,
    };
    lastResult = res;
    transition(GATEWAY_STATUS.ERROR, `任务失败（${stage}）：${lastError}`, { progress: 100, reason: lastError });
    return res;
  }

  function getChecker() {
    return hooks.checker ?? api?.cspace ?? null;
  }

  function getCurrentJoints() {
    if (api?.joints?.length) return api.joints.map((j) => j.angle ?? j.value ?? 0);
    return [0, 0, 0, 0, 0];
  }

  /**
   * 路径规划服务（中间层）：按 algorithm 调用 A* / RRT / RRT*，可选调用组员 C 后处理器并强制复验碰撞。
   */
  function planJointPath(startQ, goalQ, planOptions = {}) {
    const checker = planOptions.checker ?? getChecker();
    if (!checker) {
      return { ok: false, reason: '未配置 C-space 碰撞查询器（checker）', path: [] };
    }
    const algo = String(planOptions.algorithm ?? 'RRT*').toUpperCase();
    let planRes;
    if (algo === 'A*' || algo === 'ASTAR') {
      planRes = planAStar(startQ, goalQ, checker, planOptions);
    } else if (algo === 'RRT') {
      planRes = planRRT(startQ, goalQ, checker, planOptions);
    } else {
      planRes = planRRTStar(startQ, goalQ, checker, planOptions);
    }

    if (!planRes.ok) return planRes;

    // 若组员 C 注入了路径后处理器（捷径裁剪 + B 样条平滑），则自动执行并强制重验碰撞
    const postProcessor = planOptions.postProcessor ?? hooks.postProcessor;
    if (typeof postProcessor === 'function') {
      const smoothed = postProcessor(planRes.path, checker, planOptions);
      if (smoothed?.ok && Array.isArray(smoothed.path) && smoothed.path.length >= 2) {
        const edgeStep = planOptions.edgeStepRad ?? checker.defaultEdgeStep ?? 0.05;
        let allFree = true;
        for (let i = 1; i < smoothed.path.length; i += 1) {
          if (!checker.isEdgeFree(smoothed.path[i - 1], smoothed.path[i], edgeStep).free) {
            allFree = false;
            break;
          }
        }
        if (allFree) {
          return {
            ...planRes,
            rawPath: planRes.path,
            path: smoothed.path,
            smoothedLength: computePathLength(smoothed.path),
            postProcessed: true,
          };
        }
      }
    }

    return {
      ...planRes,
      rawPath: planRes.path,
      smoothedLength: planRes.rawLength,
      postProcessed: false,
    };
  }

  /**
   * 用户需求 API：提交自然语言或结构化 JSON 任务，驱动完整状态机：
   * IDLE → PLANNING → EXECUTING → DONE（任意阶段失败转 ERROR）。
   */
  async function submitTask(input, runOptions = {}) {
    if (status === GATEWAY_STATUS.PLANNING || status === GATEWAY_STATUS.EXECUTING) {
      return {
        ok: false,
        status,
        reason: `当前已有任务正在执行（状态：${status}），请等待完成`,
      };
    }

    currentTaskId += 1;
    lastError = null;
    progress = 0;

    // 1. 解析任务描述
    let taskSpec;
    if (typeof input === 'string') {
      const parsed = parseNaturalLanguageTask(input);
      if (!parsed.ok) return failTask(parsed.error, 'INPUT');
      taskSpec = parsed.taskSpec;
    } else if (input && typeof input === 'object' && input.task) {
      taskSpec = { ...input };
    } else {
      return failTask('任务输入必须为非空字符串或含 task 字段的 JSON 对象', 'INPUT');
    }

    // IDLE → PLANNING
    transition(GATEWAY_STATUS.PLANNING, `收到合法任务 [${taskSpec.task}]，开始规划…`, {
      progress: 15,
      taskSpec,
    });

    // 2. 规划阶段（优先支持外部注入的 taskDecomposer；否则执行标准点到点避障规划）
    let planResult = null;
    let dslBundle = null;
    try {
      if (typeof hooks.taskDecomposer === 'function' && !taskSpec.goalQ) {
        dslBundle = await hooks.taskDecomposer(taskSpec, { api, checker: getChecker(), planJointPath, runOptions });
        if (!dslBundle?.ok) {
          return failTask(dslBundle?.reason ?? '任务分解失败', 'PLANNING');
        }
        planResult = dslBundle.planResult ?? { ok: true, path: dslBundle.path ?? [] };
      } else {
        const startQ = taskSpec.startQ ?? getCurrentJoints();
        const goalQ = taskSpec.goalQ;
        if (!Array.isArray(goalQ)) {
          return failTask('缺少目标位形 goalQ，或尚未注册任务分解器（taskDecomposer）', 'PLANNING');
        }
        planResult = planJointPath(startQ, goalQ, {
          algorithm: taskSpec.algorithm ?? runOptions.algorithm ?? 'RRT*',
          ...runOptions,
        });
        if (!planResult.ok) {
          return failTask(planResult.reason ?? '路径规划无解', 'PLANNING');
        }
      }
    } catch (err) {
      return failTask(err?.message ?? String(err), 'PLANNING');
    }

    // 3. PLANNING → EXECUTING（二次验证路径合法性后进入执行态）
    transition(GATEWAY_STATUS.EXECUTING, '路径验证通过，下发驱动层执行…', {
      progress: 60,
      planResult,
    });

    try {
      const driver = runOptions.driverExecute ?? hooks.driverExecute;
      if (typeof driver !== 'function') {
        return failTask('未接入机械臂指令执行器', 'EXECUTING');
      }
      let lastReportedPct = 60;
      const reportProgress = (pct, msg) => {
        const clamped = Math.max(60, Math.min(99, Number(pct) || 60));
        if (clamped !== lastReportedPct) {
          lastReportedPct = clamped;
          progress = clamped;
          emit({ message: msg || `执行中（${clamped}%）` });
        }
      };
      const execResult = await driver({ taskSpec, planResult, dslBundle, api, reportProgress, runOptions });
      if (execResult?.ok !== true) {
        return failTask(execResult?.reason ?? '驱动层执行失败', 'EXECUTING');
      }
      const finalResult = {
        ok: true,
        taskId: currentTaskId,
        status: GATEWAY_STATUS.DONE,
        taskSpec,
        planResult,
        dslBundle,
        execResult,
      };
      lastResult = finalResult;
      transition(GATEWAY_STATUS.DONE, `任务 [${taskSpec.task}] 执行完成`, {
        progress: 100,
        result: finalResult,
      });
      return finalResult;
    } catch (err) {
      return failTask(err?.message ?? String(err), 'EXECUTING');
    }
  }

  function reset() {
    if (status === GATEWAY_STATUS.PLANNING || status === GATEWAY_STATUS.EXECUTING) return false;
    lastError = null;
    progress = 0;
    transition(GATEWAY_STATUS.IDLE, '网关已复位，等待新任务', { progress: 0 });
  }

  return {
    getState() {
      return {
        status,
        taskId: currentTaskId,
        progress,
        lastError,
        lastResult,
      };
    },
    getHistory() {
      return history.slice();
    },
    subscribe(listener) {
      if (typeof listener !== 'function') return () => {};
      listeners.add(listener);
      return () => listeners.delete(listener);
    },
    registerHook(name, fnOrObj) {
      if (!(name in hooks)) throw new Error(`未知网关钩子：${name}`);
      hooks[name] = fnOrObj;
    },
    planJointPath,
    submitTask,
    reset,
    tick() {
      hooks.driverExecute?.tick?.();
    },
  };
}
