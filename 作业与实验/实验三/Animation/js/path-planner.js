/**
 * 实验三核心路径规划引擎（组长底座）：
 * - 确定性伪随机数生成器（Mulberry32，保证对比实验可复现）
 * - A* 配置空间网格搜索（支持乐观启发、加权 A*、轴向/全邻接、懒碰撞缓存）
 * - RRT 采样规划（支持目标偏置、固定步长、沿段离散防穿模）
 * - RRT* 渐近最优规划（含邻域选最优父节点 ChooseParent + Rewire 重连 + 收敛曲线记录）
 */

/**
 * 确定性伪随机数生成器（Mulberry32），返回 [0, 1) 均匀分布随机数函数。
 */
export function createSeededRNG(seed = 42) {
  let a = (Number(seed) >>> 0) || 1;
  return function nextRandom() {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/**
 * 计算两个关节位形间的加权欧氏距离。
 */
export function jointDistance(a, b, weights = null) {
  let sum = 0;
  for (let i = 0; i < a.length; i += 1) {
    const w = weights ? (weights[i] ?? 1) : 1;
    const d = (a[i] - b[i]) * w;
    sum += d * d;
  }
  return Math.sqrt(sum);
}

/**
 * 计算关节空间折线路径的总弧长。
 */
export function computePathLength(path, weights = null) {
  if (!Array.isArray(path) || path.length < 2) return 0;
  let total = 0;
  for (let i = 1; i < path.length; i += 1) {
    total += jointDistance(path[i - 1], path[i], weights);
  }
  return total;
}

/**
 * 从 qNear 朝 qSamp 直线延伸，最大步长为 stepSize。
 */
export function steerConfig(qNear, qSamp, stepSize) {
  const dist = jointDistance(qNear, qSamp);
  if (dist <= stepSize || dist < 1e-12) return qSamp.slice();
  const ratio = stepSize / dist;
  return qNear.map((v, i) => v + (qSamp[i] - v) * ratio);
}

/**
 * 轻量二叉最小堆（按 priority 升序），供 A* OPEN 表使用。
 */
class MinHeap {
  constructor() {
    this.items = [];
  }

  get size() {
    return this.items.length;
  }

  push(node) {
    this.items.push(node);
    this._bubbleUp(this.items.length - 1);
  }

  pop() {
    if (this.items.length === 0) return null;
    const top = this.items[0];
    const last = this.items.pop();
    if (this.items.length > 0) {
      this.items[0] = last;
      this._sinkDown(0);
    }
    return top;
  }

  _bubbleUp(index) {
    let i = index;
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (this.items[parent].f <= this.items[i].f) break;
      [this.items[parent], this.items[i]] = [this.items[i], this.items[parent]];
      i = parent;
    }
  }

  _sinkDown(index) {
    let i = index;
    const n = this.items.length;
    while (true) {
      let smallest = i;
      const left = (i << 1) + 1;
      const right = left + 1;
      if (left < n && this.items[left].f < this.items[smallest].f) smallest = left;
      if (right < n && this.items[right].f < this.items[smallest].f) smallest = right;
      if (smallest === i) break;
      [this.items[smallest], this.items[i]] = [this.items[i], this.items[smallest]];
      i = smallest;
    }
  }
}

function nowMs() {
  return typeof performance !== 'undefined' && typeof performance.now === 'function'
    ? performance.now()
    : Date.now();
}

function getEffectiveLimits(checker, dim, customLimits) {
  if (Array.isArray(customLimits) && customLimits.length === dim) return customLimits;
  if (Array.isArray(checker?.limits) && checker.limits.length >= dim) return checker.limits.slice(0, dim);
  return Array.from({ length: dim }, () => ({ lower: -Math.PI, upper: Math.PI }));
}

function validateEndpoints(startQ, goalQ, checker) {
  if (!Array.isArray(startQ) || !Array.isArray(goalQ) || startQ.length !== goalQ.length || startQ.length === 0) {
    return { ok: false, reason: '起点与终点关节角维度不一致或为空' };
  }
  const startRes = checker.isConfigFree(startQ);
  if (!startRes.free) {
    return { ok: false, reason: `起点位形发生碰撞或超限（${startRes.reason}）` };
  }
  const goalRes = checker.isConfigFree(goalQ);
  if (!goalRes.free) {
    return { ok: false, reason: `终点位形发生碰撞或超限（${goalRes.reason}）` };
  }
  return { ok: true, reason: null };
}

/**
 * A* 配置空间网格搜索规划器。
 * - 默认使用乐观欧氏距离启发函数 h(n) = ||n - goal||_2（保证网格最优性）
 * - 支持加权 A*（heuristicWeight > 1）与懒碰撞缓存
 * - 支持通过 activeIndices 仅在指定主动关节子空间做网格搜索（避免高维组合爆炸）
 */
export function planAStar(startQ, goalQ, checker, options = {}) {
  const t0 = nowMs();
  const checksBefore = checker.getStats().configChecks;
  const dim = startQ.length;
  const endpointCheck = validateEndpoints(startQ, goalQ, checker);
  if (!endpointCheck.ok) {
    return {
      ok: false,
      algorithm: 'A*',
      path: [],
      rawLength: 0,
      planningTimeMs: nowMs() - t0,
      collisionChecks: checker.getStats().configChecks - checksBefore,
      expandedNodes: 0,
      exploredNodes: [],
      reason: endpointCheck.reason,
    };
  }

  const limits = getEffectiveLimits(checker, dim, options.limits);
  const resolution = options.resolution ?? 0.2;
  const heuristicWeight = Math.max(1.0, options.heuristicWeight ?? 1.0);
  const maxExpanded = options.maxExpanded ?? 15000;
  const timeoutMs = options.timeoutMs ?? 3000;
  const edgeStepRad = options.edgeStepRad ?? checker.defaultEdgeStep ?? 0.05;
  const neighborMode = options.neighborMode ?? (dim <= 3 ? 'full' : 'axial');
  const activeIndices = Array.isArray(options.activeIndices) && options.activeIndices.length > 0
    ? options.activeIndices
    : Array.from({ length: dim }, (_, i) => i);
  const goalTolerance = options.goalTolerance ?? resolution * Math.sqrt(activeIndices.length) * 0.85;

  // 若起终点直连即无碰撞，直接返回（节省网格开销）
  if (options.tryDirectConnect !== false && checker.isEdgeFree(startQ, goalQ, edgeStepRad).free) {
    const directPath = [startQ.slice(), goalQ.slice()];
    return {
      ok: true,
      algorithm: 'A*',
      path: directPath,
      rawLength: computePathLength(directPath),
      planningTimeMs: nowMs() - t0,
      collisionChecks: checker.getStats().configChecks - checksBefore,
      expandedNodes: 1,
      exploredNodes: [startQ.slice(), goalQ.slice()],
      reason: 'direct-connect',
    };
  }

  // 生成邻居偏移表
  const neighborOffsets = [];
  if (neighborMode === 'axial') {
    for (let k = 0; k < activeIndices.length; k += 1) {
      const plus = new Array(activeIndices.length).fill(0);
      plus[k] = 1;
      const minus = new Array(activeIndices.length).fill(0);
      minus[k] = -1;
      neighborOffsets.push(plus, minus);
    }
  } else {
    const buildOffsets = (depth, current) => {
      if (depth === activeIndices.length) {
        if (current.some((v) => v !== 0)) neighborOffsets.push(current.slice());
        return;
      }
      for (const delta of [-1, 0, 1]) {
        current[depth] = delta;
        buildOffsets(depth + 1, current);
      }
    };
    buildOffsets(0, new Array(activeIndices.length).fill(0));
  }

  const gridToConfig = (idxArr) => {
    const q = startQ.slice();
    for (let k = 0; k < activeIndices.length; k += 1) {
      const axis = activeIndices[k];
      q[axis] = startQ[axis] + idxArr[k] * resolution;
    }
    return q;
  };

  const keyOf = (idxArr) => idxArr.join(',');
  const startIdx = new Array(activeIndices.length).fill(0);
  const startKey = keyOf(startIdx);

  const open = new MinHeap();
  const closed = new Set();
  const pastCost = new Map();
  const parentMap = new Map();
  const coordMap = new Map();
  const freeCache = new Map();
  const exploredNodes = [];

  pastCost.set(startKey, 0);
  coordMap.set(startKey, startIdx);
  freeCache.set(startKey, true);
  open.push({ key: startKey, f: heuristicWeight * jointDistance(startQ, goalQ) });

  let expandedNodes = 0;
  let failureReason = 'OPEN 表耗尽，当前网格分辨率下未找到无碰撞路径';

  while (open.size > 0) {
    if (expandedNodes >= maxExpanded) {
      failureReason = `超过最大扩展节点数上限（${maxExpanded}）`;
      break;
    }
    if ((expandedNodes & 63) === 0 && nowMs() - t0 > timeoutMs) {
      failureReason = `A* 规划超时（>${timeoutMs} ms）`;
      break;
    }

    const current = open.pop();
    if (closed.has(current.key)) continue;
    closed.add(current.key);
    expandedNodes += 1;

    const curIdx = coordMap.get(current.key);
    const curQ = gridToConfig(curIdx);
    if (exploredNodes.length < 2000) exploredNodes.push(curQ);

    // 判断是否到达目标容差范围并可无碰直连至精确 goalQ
    if (jointDistance(curQ, goalQ) <= goalTolerance) {
      if (checker.isEdgeFree(curQ, goalQ, edgeStepRad).free) {
        const revPath = [goalQ.slice()];
        let traceKey = current.key;
        while (traceKey !== undefined) {
          const qNode = gridToConfig(coordMap.get(traceKey));
          if (jointDistance(qNode, revPath[revPath.length - 1]) > 1e-9) {
            revPath.push(qNode);
          }
          traceKey = parentMap.get(traceKey);
        }
        revPath.reverse();
        return {
          ok: true,
          algorithm: 'A*',
          path: revPath,
          rawLength: computePathLength(revPath),
          planningTimeMs: nowMs() - t0,
          collisionChecks: checker.getStats().configChecks - checksBefore,
          expandedNodes,
          exploredNodes,
          reason: 'ok',
        };
      }
    }

    const curG = pastCost.get(current.key);
    for (const offset of neighborOffsets) {
      const nbrIdx = curIdx.map((v, k) => v + offset[k]);
      const nbrKey = keyOf(nbrIdx);
      if (closed.has(nbrKey)) continue;

      const nbrQ = gridToConfig(nbrIdx);
      // 先快速检查关节限位
      let inBounds = true;
      for (let k = 0; k < activeIndices.length; k += 1) {
        const axis = activeIndices[k];
        if (nbrQ[axis] < limits[axis].lower - 1e-9 || nbrQ[axis] > limits[axis].upper + 1e-9) {
          inBounds = false;
          break;
        }
      }
      if (!inBounds) continue;

      const stepCost = jointDistance(curQ, nbrQ);
      const tentativeG = curG + stepCost;
      const knownG = pastCost.get(nbrKey);
      if (knownG !== undefined && tentativeG >= knownG - 1e-12) continue;

      // 懒碰撞检测 + 缓存
      let isFree = freeCache.get(nbrKey);
      if (isFree === undefined) {
        isFree = checker.isConfigFree(nbrQ).free;
        freeCache.set(nbrKey, isFree);
      }
      if (!isFree) continue;

      // 沿边离散校验，防止对角线穿越细杆
      if (!checker.isEdgeFree(curQ, nbrQ, edgeStepRad).free) continue;

      pastCost.set(nbrKey, tentativeG);
      parentMap.set(nbrKey, current.key);
      coordMap.set(nbrKey, nbrIdx);
      const h = jointDistance(nbrQ, goalQ);
      open.push({ key: nbrKey, f: tentativeG + heuristicWeight * h });
    }
  }

  return {
    ok: false,
    algorithm: 'A*',
    path: [],
    rawLength: 0,
    planningTimeMs: nowMs() - t0,
    collisionChecks: checker.getStats().configChecks - checksBefore,
    expandedNodes,
    exploredNodes,
    reason: failureReason,
  };
}

/**
 * RRT 随机采样路径规划器（含目标偏置、固定步长与沿段离散防穿模）。
 */
export function planRRT(startQ, goalQ, checker, options = {}) {
  const t0 = nowMs();
  const checksBefore = checker.getStats().configChecks;
  const dim = startQ.length;
  const endpointCheck = validateEndpoints(startQ, goalQ, checker);
  if (!endpointCheck.ok) {
    return {
      ok: false,
      algorithm: 'RRT',
      path: [],
      rawLength: 0,
      planningTimeMs: nowMs() - t0,
      collisionChecks: checker.getStats().configChecks - checksBefore,
      iterations: 0,
      tree: [],
      reason: endpointCheck.reason,
    };
  }

  const limits = getEffectiveLimits(checker, dim, options.limits);
  const maxIterations = options.maxIterations ?? 2500;
  const stepSize = options.stepSize ?? 0.25;
  const goalBias = options.goalBias ?? 0.08;
  const goalTolerance = options.goalTolerance ?? stepSize;
  const edgeStepRad = options.edgeStepRad ?? checker.defaultEdgeStep ?? 0.05;
  const timeoutMs = options.timeoutMs ?? 3000;
  const rng = createSeededRNG(options.seed ?? 42);

  const tree = [{ q: startQ.slice(), parent: -1, cost: 0 }];
  let failureReason = `达到最大采样次数（${maxIterations}），未找到无碰撞路径`;

  for (let iter = 1; iter <= maxIterations; iter += 1) {
    if ((iter & 63) === 0 && nowMs() - t0 > timeoutMs) {
      failureReason = `RRT 规划超时（>${timeoutMs} ms）`;
      break;
    }

    // 1. 目标偏置采样
    const qSamp = rng() < goalBias
      ? goalQ.slice()
      : limits.map((lim) => lim.lower + rng() * (lim.upper - lim.lower));

    // 2. 最近邻搜索
    let nearIdx = 0;
    let minDist = Infinity;
    for (let i = 0; i < tree.length; i += 1) {
      const d = jointDistance(tree[i].q, qSamp);
      if (d < minDist) {
        minDist = d;
        nearIdx = i;
      }
    }

    // 3. 直线延伸 + 沿段离散防穿模检测
    const qNear = tree[nearIdx].q;
    const qNew = steerConfig(qNear, qSamp, stepSize);
    if (jointDistance(qNear, qNew) < 1e-9) continue;
    if (!checker.isEdgeFree(qNear, qNew, edgeStepRad).free) continue;

    const newIdx = tree.length;
    const newCost = tree[nearIdx].cost + jointDistance(qNear, qNew);
    tree.push({ q: qNew, parent: nearIdx, cost: newCost });

    // 4. 目标连接判定
    const distToGoal = jointDistance(qNew, goalQ);
    if (distToGoal <= goalTolerance && checker.isEdgeFree(qNew, goalQ, edgeStepRad).free) {
      const goalIdx = tree.length;
      tree.push({ q: goalQ.slice(), parent: newIdx, cost: newCost + distToGoal });

      const path = [];
      let cur = goalIdx;
      while (cur !== -1) {
        path.push(tree[cur].q.slice());
        cur = tree[cur].parent;
      }
      path.reverse();

      return {
        ok: true,
        algorithm: 'RRT',
        path,
        rawLength: computePathLength(path),
        planningTimeMs: nowMs() - t0,
        collisionChecks: checker.getStats().configChecks - checksBefore,
        iterations: iter,
        tree,
        reason: 'ok',
      };
    }
  }

  return {
    ok: false,
    algorithm: 'RRT',
    path: [],
    rawLength: 0,
    planningTimeMs: nowMs() - t0,
    collisionChecks: checker.getStats().configChecks - checksBefore,
    iterations: maxIterations,
    tree,
    reason: failureReason,
  };
}

/**
 * RRT* 渐近最优路径规划器：
 * - 在 RRT 基础上实现 ChooseParent（邻域选最优父节点）与 Rewire（邻域重连优化）
 * - 支持在找到首个可行解后继续迭代收敛，记录 costHistory 供 RRT vs RRT* 收敛曲线对比
 */
export function planRRTStar(startQ, goalQ, checker, options = {}) {
  const t0 = nowMs();
  const checksBefore = checker.getStats().configChecks;
  const dim = startQ.length;
  const endpointCheck = validateEndpoints(startQ, goalQ, checker);
  if (!endpointCheck.ok) {
    return {
      ok: false,
      algorithm: 'RRT*',
      path: [],
      rawLength: 0,
      planningTimeMs: nowMs() - t0,
      collisionChecks: checker.getStats().configChecks - checksBefore,
      iterations: 0,
      tree: [],
      costHistory: [],
      reason: endpointCheck.reason,
    };
  }

  const limits = getEffectiveLimits(checker, dim, options.limits);
  const maxIterations = options.maxIterations ?? 1200;
  const stepSize = options.stepSize ?? 0.25;
  const goalBias = options.goalBias ?? 0.08;
  const goalTolerance = options.goalTolerance ?? stepSize;
  const edgeStepRad = options.edgeStepRad ?? checker.defaultEdgeStep ?? 0.05;
  const timeoutMs = options.timeoutMs ?? 4000;
  const stopOnFirstSolution = options.stopOnFirstSolution ?? false;
  const gamma = options.gamma ?? 2.2;
  const maxRewireRadius = options.rewireRadius ?? stepSize * 2.5;
  const rng = createSeededRNG(options.seed ?? 42);

  const tree = [{ q: startQ.slice(), parent: -1, cost: 0, children: [] }];
  const costHistory = [];
  let bestGoalParent = -1;
  let bestGoalCost = Infinity;
  let actualIterations = 0;
  let failureReason = `达到最大采样次数（${maxIterations}），未找到无碰撞路径`;

  const propagateCostDelta = (nodeIdx, delta) => {
    const stack = [nodeIdx];
    while (stack.length > 0) {
      const idx = stack.pop();
      for (const childIdx of tree[idx].children) {
        tree[childIdx].cost += delta;
        stack.push(childIdx);
      }
    }
  };

  const refreshBestGoalCost = (iter) => {
    if (bestGoalParent === -1) return;
    const total = tree[bestGoalParent].cost + jointDistance(tree[bestGoalParent].q, goalQ);
    if (total < bestGoalCost - 1e-9) {
      bestGoalCost = total;
      costHistory.push({ iteration: iter, bestCost: bestGoalCost, elapsedMs: nowMs() - t0 });
    }
  };

  for (let iter = 1; iter <= maxIterations; iter += 1) {
    actualIterations = iter;
    if ((iter & 31) === 0 && nowMs() - t0 > timeoutMs) {
      failureReason = `RRT* 规划超时（>${timeoutMs} ms）`;
      break;
    }

    // 1. 采样
    const qSamp = rng() < goalBias
      ? goalQ.slice()
      : limits.map((lim) => lim.lower + rng() * (lim.upper - lim.lower));

    // 2. 最近邻
    let nearIdx = 0;
    let minDist = Infinity;
    for (let i = 0; i < tree.length; i += 1) {
      const d = jointDistance(tree[i].q, qSamp);
      if (d < minDist) {
        minDist = d;
        nearIdx = i;
      }
    }

    const qNear = tree[nearIdx].q;
    const qNew = steerConfig(qNear, qSamp, stepSize);
    if (jointDistance(qNear, qNew) < 1e-9) continue;
    if (!checker.isEdgeFree(qNear, qNew, edgeStepRad).free) continue;

    // 3. 搜索邻域集合 Q_near
    const n = tree.length + 1;
    const dynamicRadius = Math.min(maxRewireRadius, gamma * Math.pow(Math.log(n) / n, 1 / dim));
    const searchRadius = Math.max(stepSize * 1.25, dynamicRadius);
    const neighbors = [];
    for (let i = 0; i < tree.length; i += 1) {
      const d = jointDistance(tree[i].q, qNew);
      if (d <= searchRadius) neighbors.push({ idx: i, dist: d });
    }

    // 4. ChooseParent：在邻域内选择使起点到 qNew 总代价最小的父节点
    let bestParent = nearIdx;
    let bestCost = tree[nearIdx].cost + jointDistance(qNear, qNew);
    for (const nbr of neighbors) {
      const candCost = tree[nbr.idx].cost + nbr.dist;
      if (candCost < bestCost - 1e-9 && checker.isEdgeFree(tree[nbr.idx].q, qNew, edgeStepRad).free) {
        bestParent = nbr.idx;
        bestCost = candCost;
      }
    }

    const newIdx = tree.length;
    tree.push({ q: qNew, parent: bestParent, cost: bestCost, children: [] });
    tree[bestParent].children.push(newIdx);

    // 5. Rewire：检查邻域节点经 qNew 中转是否更短
    for (const nbr of neighbors) {
      if (nbr.idx === bestParent || nbr.idx === 0) continue;
      const newNbrCost = bestCost + nbr.dist;
      if (newNbrCost < tree[nbr.idx].cost - 1e-9 && checker.isEdgeFree(qNew, tree[nbr.idx].q, edgeStepRad).free) {
        const oldParent = tree[nbr.idx].parent;
        if (oldParent !== -1) {
          const idxInParent = tree[oldParent].children.indexOf(nbr.idx);
          if (idxInParent >= 0) tree[oldParent].children.splice(idxInParent, 1);
        }
        const delta = newNbrCost - tree[nbr.idx].cost;
        tree[nbr.idx].parent = newIdx;
        tree[nbr.idx].cost = newNbrCost;
        tree[newIdx].children.push(nbr.idx);
        propagateCostDelta(nbr.idx, delta);
      }
    }

    // 6. 检查新节点能否直连目标或更新已有目标父节点
    const distToGoal = jointDistance(qNew, goalQ);
    if (distToGoal <= goalTolerance && checker.isEdgeFree(qNew, goalQ, edgeStepRad).free) {
      const candGoalCost = bestCost + distToGoal;
      if (candGoalCost < bestGoalCost - 1e-9) {
        bestGoalParent = newIdx;
        bestGoalCost = candGoalCost;
        costHistory.push({ iteration: iter, bestCost: bestGoalCost, elapsedMs: nowMs() - t0 });
        if (stopOnFirstSolution) break;
      }
    } else {
      refreshBestGoalCost(iter);
    }
  }

  if (bestGoalParent !== -1) {
    const path = [goalQ.slice()];
    let cur = bestGoalParent;
    while (cur !== -1) {
      path.push(tree[cur].q.slice());
      cur = tree[cur].parent;
    }
    path.reverse();
    return {
      ok: true,
      algorithm: 'RRT*',
      path,
      rawLength: computePathLength(path),
      planningTimeMs: nowMs() - t0,
      collisionChecks: checker.getStats().configChecks - checksBefore,
      iterations: actualIterations,
      tree: tree.map(({ q, parent, cost }) => ({ q, parent, cost })),
      costHistory,
      reason: 'ok',
    };
  }

  return {
    ok: false,
    algorithm: 'RRT*',
    path: [],
    rawLength: 0,
    planningTimeMs: nowMs() - t0,
    collisionChecks: checker.getStats().configChecks - checksBefore,
    iterations: actualIterations,
    tree: tree.map(({ q, parent, cost }) => ({ q, parent, cost })),
    costHistory,
    reason: failureReason,
  };
}
