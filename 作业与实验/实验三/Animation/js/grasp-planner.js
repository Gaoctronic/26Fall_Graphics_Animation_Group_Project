import * as THREE from 'three';
import { attachWorkpiece } from './Workstation.js';

function vector3(value, name) {
  const values = value?.isVector3 ? value.toArray() : value;
  if (!Array.isArray(values) || values.length !== 3 || !values.every(Number.isFinite)) {
    throw new TypeError(`${name} 必须是三个有限数值`);
  }
  return new THREE.Vector3().fromArray(values);
}

/**
 * 顶抓几何规划，输出世界坐标 TCP 位姿；长度沿用工程资产单位。
 * position 是物体中心，size 是直立物体的尺寸；axis 是夹持主轴，
 * 其 XZ 投影决定夹爪局部 X（闭爪方向）。竖直主轴不限制偏航，取 +X。
 * 夹爪局部 +Y 朝上，手指向下；grasp 位于物体顶面中心。
 * 本函数只生成几何位姿，IK、路径避障和运动执行由调用方完成。
 * @param {{position: number[], size: number[], axis?: number[]}} targetSpec
 * @param {{approachDist?: number, liftHeight?: number}} options
 * @returns {{preGrasp: object, grasp: object, lift: object}}
 */
export function planThreeStageGrasp(targetSpec, options = {}) {
  const center = vector3(targetSpec?.position, 'position');
  const size = vector3(targetSpec?.size, 'size');
  if (Math.min(size.x, size.y, size.z) <= 0) throw new RangeError('size 必须大于 0');
  const approachDist = options.approachDist ?? 0.08;
  const liftHeight = options.liftHeight ?? 0.12;
  if (![approachDist, liftHeight].every((n) => Number.isFinite(n) && n >= 0)) {
    throw new RangeError('approachDist 和 liftHeight 必须是非负有限数值');
  }
  const closingAxis = vector3(targetSpec.axis ?? [1, 0, 0], 'axis');
  if (closingAxis.lengthSq() === 0) throw new RangeError('axis 不能是零向量');
  closingAxis.y = 0;
  if (closingAxis.lengthSq() < 1e-20) closingAxis.set(1, 0, 0);
  closingAxis.normalize();
  const up = new THREE.Vector3(0, 1, 0);
  const forward = new THREE.Vector3().crossVectors(closingAxis, up);
  const quaternion = new THREE.Quaternion().setFromRotationMatrix(
    new THREE.Matrix4().makeBasis(closingAxis, up, forward),
  );
  const graspPosition = center.clone().addScaledVector(up, size.y / 2);
  const pose = (height) => ({
    position: graspPosition.clone().addScaledVector(up, height),
    quaternion: quaternion.clone(),
  });
  return { preGrasp: pose(approachDist), grasp: pose(0), lift: pose(liftHeight) };
}

// 只取工件/手指自身的几何，不把标定标记、文字 Sprite 等子节点算作接触面。
function meshBounds(mesh, inverseFrame) {
  mesh.geometry.computeBoundingBox();
  mesh.updateWorldMatrix(true, false);
  return mesh.geometry.boundingBox.clone().applyMatrix4(
    new THREE.Matrix4().multiplyMatrices(inverseFrame, mesh.matrixWorld),
  );
}

function overlapsYZ(a, b, epsilon) {
  return Math.min(a.max.y, b.max.y) - Math.max(a.min.y, b.min.y) > epsilon &&
    Math.min(a.max.z, b.max.z) - Math.max(a.min.z, b.min.z) > epsilon;
}

/**
 * 同步执行教学用 EFFORT 接触反馈循环；调用前须停止机械臂运动。
 * 每步缩小开口并查询夹爪坐标系中的包围盒；首次接触即停止闭合。
 * 按讲义简化模型，接触力从 0 跳到 effortLimitN（默认 50N），
 * 不是材料刚度/动力学求解。双指接触才挂载；单侧接触停止但不抓取。
 * 返回 { gripped, contactForceN, reason }，开度和力保存在 workstation 中，
 * OPEN/resetWorkpiece 会清零。重复调用已夹持工件不会重开夹爪或移动工件。
 */
export function simulateEffortGrip(workstation, effortLimitN = 50) {
  if (!Number.isFinite(effortLimitN) || effortLimitN <= 0) {
    throw new RangeError('effortLimitN 必须是大于 0 的有限数值');
  }
  const s = workstation?.blockSize;
  if (!workstation?.workpiece?.geometry || !workstation.gripper?.isObject3D ||
      workstation.fingers?.length !== 2 || typeof workstation.setGripperOpening !== 'function' ||
      !Number.isFinite(s) || s <= 0) {
    return { gripped: false, contactForceN: 0, reason: '工位或夹爪尚未初始化' };
  }
  const finish = (gripped, contactForceN, reason) => {
    workstation.contactForceN = contactForceN;
    workstation.options.onGripChange?.(workstation.gripStatus);
    return { gripped, contactForceN, reason };
  };
  if (workstation.attached && workstation.contactForceN > 0) {
    return finish(true, effortLimitN, '工件已双指夹持，保持当前开度');
  }

  workstation.updateGripper();
  const inverse = workstation.gripper.matrixWorld.clone().invert();
  const target = meshBounds(workstation.workpiece, inverse);
  const epsilon = s * 1e-8;
  let opening = workstation.gripperOpening ?? s * (workstation.gripperState === 'OPEN' ? 1.5 : 1);
  // 不允许从已穿入工件的初始开口继续闭合，也不自动张爪或搬动物体。
  if (target.min.x < -opening / 2 - epsilon || target.max.x > opening / 2 + epsilon) {
    return finish(false, 0, '工件超出当前开口，请先张开夹爪并对准工件');
  }
  const left = workstation.fingers.find((finger) => finger.userData.side === -1);
  const right = workstation.fingers.find((finger) => finger.userData.side === 1);
  if (!left || !right) return finish(false, 0, '夹爪缺少左右手指');
  const leftBox = meshBounds(left, inverse);
  const rightBox = meshBounds(right, inverse);
  const leftAligned = overlapsYZ(leftBox, target, epsilon);
  const rightAligned = overlapsYZ(rightBox, target, epsilon);
  const leftContactOpening = -2 * target.min.x;
  const rightContactOpening = 2 * target.max.x;
  const contactOpenings = [
    ...(leftAligned ? [leftContactOpening] : []),
    ...(rightAligned ? [rightContactOpening] : []),
  ].filter((gap) => gap >= 0 && gap <= opening + epsilon);
  // 最后一步截断到几何接触位置，避免离散步长造成穿透。
  const firstContact = Math.max(0, ...contactOpenings);
  const step = s * 0.025;
  workstation.gripperState = 'CLOSE';
  for (;;) {
    workstation.setGripperOpening(opening);
    const currentLeft = meshBounds(left, inverse);
    const currentRight = meshBounds(right, inverse);
    const leftContact = leftAligned && currentLeft.max.x >= target.min.x - epsilon &&
      currentLeft.min.x <= target.max.x + epsilon;
    const rightContact = rightAligned && currentRight.min.x <= target.max.x + epsilon &&
      currentRight.max.x >= target.min.x - epsilon;
    if (leftContact || rightContact) {
      if (leftContact && rightContact) {
        attachWorkpiece(workstation.flange, workstation.workpiece);
        return finish(true, effortLimitN, '双指接触达到力阈值，停止闭合并夹持工件');
      }
      return finish(false, effortLimitN, '仅单侧接触，已停止闭合；请重新对准工件');
    }
    if (opening <= epsilon) return finish(false, 0, '夹爪已闭合，但没有接触到工件');
    const next = Math.max(0, firstContact, opening - step);
    // 数值精度导致候选接触点未相交时，以无接触结果结束，避免停在同一开度。
    if (next >= opening) return finish(false, 0, '未形成有效双指接触');
    opening = next;
  }
}
