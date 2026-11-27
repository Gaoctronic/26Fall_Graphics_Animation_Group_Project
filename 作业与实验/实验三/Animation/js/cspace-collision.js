import * as THREE from 'three';
import { setJointValue } from './urdf-model.js';

const DEFAULT_SAFETY_MARGIN = 0.03;
const DEFAULT_LINK_RADIUS = 0.04;
const DEFAULT_EDGE_STEP_RAD = 0.05;

function toVec3(input) {
  if (!input) return new THREE.Vector3();
  if (input.isVector3) return input.clone();
  if (Array.isArray(input)) return new THREE.Vector3(input[0] ?? 0, input[1] ?? 0, input[2] ?? 0);
  return new THREE.Vector3(input.x ?? 0, input.y ?? 0, input.z ?? 0);
}

/**
 * 计算空间点 P 到线段 AB 的最短距离平方与最近点参数 t ∈ [0, 1]。
 */
export function pointToSegmentDistSq(p, a, b) {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const abz = b.z - a.z;
  const lenSq = abx * abx + aby * aby + abz * abz;
  if (lenSq < 1e-12) {
    const dx = p.x - a.x;
    const dy = p.y - a.y;
    const dz = p.z - a.z;
    return { distSq: dx * dx + dy * dy + dz * dz, t: 0 };
  }
  const apx = p.x - a.x;
  const apy = p.y - a.y;
  const apz = p.z - a.z;
  const t = Math.max(0, Math.min(1, (apx * abx + apy * aby + apz * abz) / lenSq));
  const cx = a.x + abx * t;
  const cy = a.y + aby * t;
  const cz = a.z + abz * t;
  const dx = p.x - cx;
  const dy = p.y - cy;
  const dz = p.z - cz;
  return { distSq: dx * dx + dy * dy + dz * dz, t };
}

/**
 * 线段 AB 与竖直圆柱体（沿 Y 轴）的胶囊体碰撞检测（支持安全膨胀半径）。
 */
function segmentIntersectsCylinder(a, b, obs, inflateRadius) {
  const center = toVec3(obs.position);
  const halfH = (obs.height ?? 1.0) / 2 + inflateRadius;
  const yMin = center.y - halfH;
  const yMax = center.y + halfH;
  const effR = (obs.radius ?? 0.1) + inflateRadius;
  const effRSq = effR * effR;

  // 沿线段采样（足够细以捕捉倾斜线段与圆柱交点）
  const segLen = a.distanceTo(b);
  const steps = Math.max(2, Math.ceil(segLen / Math.max(effR * 0.4, 0.03)));
  for (let i = 0; i <= steps; i += 1) {
    const t = i / steps;
    const x = a.x + (b.x - a.x) * t;
    const y = a.y + (b.y - a.y) * t;
    const z = a.z + (b.z - a.z) * t;
    if (y >= yMin && y <= yMax) {
      const dx = x - center.x;
      const dz = z - center.z;
      if (dx * dx + dz * dz <= effRSq) {
        return { hit: true, point: new THREE.Vector3(x, y, z) };
      }
    }
  }
  return { hit: false, point: null };
}

/**
 * 线段 AB 与轴对齐包围盒（AABB）的膨胀相交检测。
 */
function segmentIntersectsBox(a, b, obs, inflateRadius) {
  const center = toVec3(obs.position);
  const size = Array.isArray(obs.size)
    ? { x: obs.size[0], y: obs.size[1], z: obs.size[2] }
    : (obs.size ?? { x: 0.2, y: 0.2, z: 0.2 });
  const minX = center.x - size.x / 2 - inflateRadius;
  const maxX = center.x + size.x / 2 + inflateRadius;
  const minY = center.y - size.y / 2 - inflateRadius;
  const maxY = center.y + size.y / 2 + inflateRadius;
  const minZ = center.z - size.z / 2 - inflateRadius;
  const maxZ = center.z + size.z / 2 + inflateRadius;

  // Slab 法求线段与膨胀 AABB 交点
  const dir = [b.x - a.x, b.y - a.y, b.z - a.z];
  const orig = [a.x, a.y, a.z];
  const mins = [minX, minY, minZ];
  const maxs = [maxX, maxY, maxZ];
  let tMin = 0;
  let tMax = 1;
  for (let axis = 0; axis < 3; axis += 1) {
    if (Math.abs(dir[axis]) < 1e-12) {
      if (orig[axis] < mins[axis] || orig[axis] > maxs[axis]) return { hit: false, point: null };
    } else {
      let t1 = (mins[axis] - orig[axis]) / dir[axis];
      let t2 = (maxs[axis] - orig[axis]) / dir[axis];
      if (t1 > t2) [t1, t2] = [t2, t1];
      tMin = Math.max(tMin, t1);
      tMax = Math.min(tMax, t2);
      if (tMin > tMax) return { hit: false, point: null };
    }
  }
  const tHit = Math.max(0, Math.min(1, (tMin + tMax) * 0.5));
  return {
    hit: true,
    point: new THREE.Vector3(
      a.x + dir[0] * tHit,
      a.y + dir[1] * tHit,
      a.z + dir[2] * tHit,
    ),
  };
}

/**
 * 线段 AB 与球体障碍物的膨胀相交检测。
 */
function segmentIntersectsSphere(a, b, obs, inflateRadius) {
  const center = toVec3(obs.position);
  const effR = (obs.radius ?? 0.1) + inflateRadius;
  const { distSq, t } = pointToSegmentDistSq(center, a, b);
  if (distSq <= effR * effR) {
    return {
      hit: true,
      point: new THREE.Vector3(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
      ),
    };
  }
  return { hit: false, point: null };
}

/**
 * 纯配置空间（C-space）抽象禁区检测：支持球体禁区与轴对齐超矩形禁区。
 */
function checkCSpaceObstacle(q, obs) {
  if (obs.type === 'cspace-box') {
    const min = obs.min;
    const max = obs.max;
    for (let i = 0; i < q.length; i += 1) {
      if (q[i] < min[i] || q[i] > max[i]) return false;
    }
    return true;
  }
  if (obs.type === 'cspace-sphere') {
    let distSq = 0;
    for (let i = 0; i < q.length; i += 1) {
      const d = q[i] - (obs.center[i] ?? 0);
      distSq += d * d;
    }
    const r = obs.radius ?? 0.1;
    return distSq <= r * r;
  }
  return false;
}

/**
 * 创建 C-space 配置空间碰撞查询器。
 * 支持：
 * 1) 真实 Three.js 机械臂全连杆骨架 + 障碍物（cylinder/box/sphere）+ 地面防穿模；
 * 2) 抽象 C-space 禁区（cspace-box / cspace-sphere）或自定义回调，用于低维算法验证与单元测试。
 *
 * @param {object|null} api - window.robotSimulation（含 joints, tcp, floorY），可为 null（纯 C-space 模式）
 * @param {Array<object>} [initialObstacles=[]] - 障碍物列表
 * @param {object} [options={}]
 */
export function createCSpaceChecker(api = null, initialObstacles = [], options = {}) {
  const safetyMargin = options.safetyMargin ?? DEFAULT_SAFETY_MARGIN;
  const linkRadius = options.linkRadius ?? DEFAULT_LINK_RADIUS;
  const defaultEdgeStep = options.edgeStepRad ?? DEFAULT_EDGE_STEP_RAD;
  const extraChecker = typeof options.extraChecker === 'function' ? options.extraChecker : null;
  const checkFloor = options.checkFloor ?? Boolean(api && Number.isFinite(api.floorY));

  const limits = options.limits
    ?? (api?.joints
      ? api.joints.map((j) => ({ lower: j.lower, upper: j.upper, name: j.name }))
      : []);

  let obstacles = Array.isArray(initialObstacles) ? initialObstacles.map((o) => ({ ...o })) : [];
  const stats = {
    configChecks: 0,
    edgeChecks: 0,
    collisions: 0,
  };

  function getJointSegmentsForConfig(q) {
    if (!api?.joints?.length) return [];
    const saved = api.joints.map((j) => j.value ?? 0);
    const segments = [];
    try {
      for (let i = 0; i < api.joints.length; i += 1) {
        setJointValue(api.joints[i], q[i] ?? 0);
      }
      if (api.robot) api.robot.updateMatrixWorld(true);
      else if (api.tcp) api.tcp.updateWorldMatrix(true, false);

      const pts = [];
      for (let i = 0; i < api.joints.length; i += 1) {
        const node = api.joints[i].node;
        if (node) {
          const p = new THREE.Vector3();
          node.getWorldPosition(p);
          pts.push({ name: api.joints[i].name ?? `J${i + 1}`, pos: p });
        }
      }
      if (api.tcp) {
        const tcpPos = new THREE.Vector3();
        api.tcp.getWorldPosition(tcpPos);
        pts.push({ name: 'TCP', pos: tcpPos });
      }
      for (let i = 0; i < pts.length - 1; i += 1) {
        segments.push({
          fromName: pts[i].name,
          toName: pts[i + 1].name,
          a: pts[i].pos,
          b: pts[i + 1].pos,
        });
      }
      return segments;
    } finally {
      for (let i = 0; i < api.joints.length; i += 1) {
        setJointValue(api.joints[i], saved[i]);
      }
      if (api.robot) api.robot.updateMatrixWorld(true);
      else if (api.tcp) api.tcp.updateWorldMatrix(true, false);
    }
  }

  /**
   * 查询单个位形 q 是否处于自由配置空间 C_free。
   * @param {number[]} q - 关节角数组（弧度）
   * @returns {{ free: boolean, reason: string|null, obstacleId: string|null, hitPoint: THREE.Vector3|null }}
   */
  function isConfigFree(q) {
    stats.configChecks += 1;
    if (!Array.isArray(q) || q.length === 0 || q.some((v) => !Number.isFinite(v))) {
      stats.collisions += 1;
      return { free: false, reason: 'invalid-config', obstacleId: null, hitPoint: null };
    }

    // 1. 关节限位校验
    for (let i = 0; i < limits.length && i < q.length; i += 1) {
      if (q[i] < limits[i].lower - 1e-9 || q[i] > limits[i].upper + 1e-9) {
        stats.collisions += 1;
        return {
          free: false,
          reason: `joint-limit:${limits[i].name ?? `J${i + 1}`}`,
          obstacleId: null,
          hitPoint: null,
        };
      }
    }

    // 2. 纯 C-space 抽象障碍物检测
    for (const obs of obstacles) {
      if (obs.type === 'cspace-box' || obs.type === 'cspace-sphere') {
        if (checkCSpaceObstacle(q, obs)) {
          stats.collisions += 1;
          return {
            free: false,
            reason: `cspace-obstacle:${obs.id ?? obs.type}`,
            obstacleId: obs.id ?? obs.type,
            hitPoint: null,
          };
        }
      }
    }

    // 3. 3D 工作空间全连杆胶囊体与障碍物/地面检测
    const workspaceObs = obstacles.filter((o) => o.type === 'cylinder' || o.type === 'box' || o.type === 'sphere');
    const needsSegments = Boolean(api?.joints?.length) && (workspaceObs.length > 0 || checkFloor || extraChecker);
    const segments = needsSegments ? getJointSegmentsForConfig(q) : [];

    if (checkFloor && Number.isFinite(api?.floorY)) {
      const floorLimit = api.floorY + safetyMargin;
      // 跳过基座根部第 0 段（允许固定在底座上），检查 J2 及以上连杆与末端
      for (let i = 1; i < segments.length; i += 1) {
        if (segments[i].b.y < floorLimit) {
          stats.collisions += 1;
          return {
            free: false,
            reason: `floor-collision:${segments[i].toName}`,
            obstacleId: 'floor',
            hitPoint: segments[i].b.clone(),
          };
        }
      }
    }

    for (const seg of segments) {
      for (const obs of workspaceObs) {
        const inflate = (obs.margin ?? safetyMargin) + linkRadius;
        let hitRes = { hit: false, point: null };
        if (obs.type === 'cylinder') hitRes = segmentIntersectsCylinder(seg.a, seg.b, obs, inflate);
        else if (obs.type === 'box') hitRes = segmentIntersectsBox(seg.a, seg.b, obs, inflate);
        else if (obs.type === 'sphere') hitRes = segmentIntersectsSphere(seg.a, seg.b, obs, inflate);

        if (hitRes.hit) {
          stats.collisions += 1;
          return {
            free: false,
            reason: `workspace-obstacle:${obs.id ?? obs.type}(${seg.fromName}-${seg.toName})`,
            obstacleId: obs.id ?? obs.type,
            hitPoint: hitRes.point,
          };
        }
      }
    }

    // 4. 外部自定义碰撞扩展（例如自碰撞检测器）
    if (extraChecker) {
      const ext = extraChecker(q, segments);
      if (ext === false || (ext && ext.free === false)) {
        stats.collisions += 1;
        return {
          free: false,
          reason: ext?.reason ?? 'extra-collision',
          obstacleId: ext?.obstacleId ?? null,
          hitPoint: ext?.hitPoint ?? null,
        };
      }
    }

    return { free: true, reason: null, obstacleId: null, hitPoint: null };
  }

  /**
   * 沿关节空间线段 qA → qB 按步长 stepRad 离散打点检测，防止大步长穿过细杆障碍。
   * @param {number[]} qA
   * @param {number[]} qB
   * @param {number} [stepRad]
   * @returns {{ free: boolean, samples: number, hitAt: number|null, hitConfig: number[]|null, reason: string|null, obstacleId: string|null, hitPoint: THREE.Vector3|null }}
   */
  function isEdgeFree(qA, qB, stepRad = defaultEdgeStep) {
    stats.edgeChecks += 1;
    const step = Math.max(1e-4, Number(stepRad) || defaultEdgeStep);
    let maxDelta = 0;
    for (let i = 0; i < qA.length; i += 1) {
      const d = Math.abs(qB[i] - qA[i]);
      if (d > maxDelta) maxDelta = d;
    }
    const divisions = Math.max(1, Math.ceil(maxDelta / step));
    for (let k = 0; k <= divisions; k += 1) {
      const t = k / divisions;
      const q = new Array(qA.length);
      for (let i = 0; i < qA.length; i += 1) {
        q[i] = qA[i] + (qB[i] - qA[i]) * t;
      }
      const res = isConfigFree(q);
      if (!res.free) {
        return {
          free: false,
          samples: k + 1,
          hitAt: t,
          hitConfig: q,
          reason: res.reason,
          obstacleId: res.obstacleId,
          hitPoint: res.hitPoint,
        };
      }
    }
    return {
      free: true,
      samples: divisions + 1,
      hitAt: null,
      hitConfig: null,
      reason: null,
      obstacleId: null,
      hitPoint: null,
    };
  }

  return {
    limits,
    safetyMargin,
    linkRadius,
    defaultEdgeStep,
    isConfigFree,
    isEdgeFree,
    getJointSegmentsForConfig,
    setObstacles(nextObstacles) {
      obstacles = Array.isArray(nextObstacles) ? nextObstacles.map((o) => ({ ...o })) : [];
    },
    getObstacles() {
      return obstacles.map((o) => ({ ...o }));
    },
    getStats() {
      return { ...stats };
    },
    resetStats() {
      stats.configChecks = 0;
      stats.edgeChecks = 0;
      stats.collisions = 0;
    },
  };
}
