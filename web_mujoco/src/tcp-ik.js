import { ARM_JOINTS } from './kinematics.js';

const EPS = 0.004;
const GAIN = 12;
const MIN_DAMPING = 0.018;
const MAX_DAMPING = 0.075;
const SINGULARITY_THRESHOLD = 0.08;
const MAX_JOINT_SPEED = 2.8;
const REACH_ERROR = 0.0015;
export const NOMINAL_REACH = 0.56;

function clip(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function copyQpos(data) {
  const values = new Float64Array(data.qpos.length);
  for (let i = 0; i < data.qpos.length; i += 1) values[i] = data.qpos[i];
  return values;
}

function restoreQpos(data, values) {
  for (let i = 0; i < values.length; i += 1) data.qpos[i] = values[i];
}

function dotRows(a, b) {
  return a.reduce((sum, value, index) => sum + value * (b[index] || 0), 0);
}

function determinant3x3(a) {
  return (
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
    a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
    a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
  );
}

function solve3x3(a, b) {
  const det = determinant3x3(a);
  if (Math.abs(det) < 1e-9) return null;
  const inv = [
    [
      (a[1][1] * a[2][2] - a[1][2] * a[2][1]) / det,
      (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det,
      (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det
    ],
    [
      (a[1][2] * a[2][0] - a[1][0] * a[2][2]) / det,
      (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det,
      (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / det
    ],
    [
      (a[1][0] * a[2][1] - a[1][1] * a[2][0]) / det,
      (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / det,
      (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / det
    ]
  ];
  return [
    inv[0][0] * b[0] + inv[0][1] * b[1] + inv[0][2] * b[2],
    inv[1][0] * b[0] + inv[1][1] * b[1] + inv[1][2] * b[2],
    inv[2][0] * b[0] + inv[2][1] * b[1] + inv[2][2] * b[2]
  ];
}

export function createTcpIk(mujoco, model, data, joints) {
  const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
  let bodyId = mujoco.mj_name2id(model, bodyType, 'gripper_end');
  if (bodyId < 0) bodyId = mujoco.mj_name2id(model, bodyType, 'link6');
  if (bodyId < 0) throw new Error('找不到末端 body gripper_end / link6');

  const arm = ARM_JOINTS.map((joint) => ({
    ...joint,
    qposadr: joints.byName[joint.name].qposadr
  }));

  function tcpPosition() {
    return {
      x: data.xpos[bodyId * 3],
      y: data.xpos[bodyId * 3 + 1],
      z: data.xpos[bodyId * 3 + 2]
    };
  }

  function applyArmAngles(angles) {
    arm.forEach((joint) => {
      data.qpos[joint.qposadr] = clip(angles[joint.name], joint.min, joint.max);
    });
    mujoco.mj_forward(model, data);
  }

  function computeJacobian(baseAngles) {
    const rows = [[], [], []];
    arm.forEach((joint) => {
      const plus = { ...baseAngles, [joint.name]: baseAngles[joint.name] + EPS };
      const minus = { ...baseAngles, [joint.name]: baseAngles[joint.name] - EPS };
      applyArmAngles(plus);
      const plusPos = tcpPosition();
      applyArmAngles(minus);
      const minusPos = tcpPosition();
      rows[0].push((plusPos.x - minusPos.x) / (2 * EPS));
      rows[1].push((plusPos.y - minusPos.y) / (2 * EPS));
      rows[2].push((plusPos.z - minusPos.z) / (2 * EPS));
    });
    applyArmAngles(baseAngles);
    return rows;
  }

  function solveDampedLeastSquares(jacobian, error) {
    const gram = [
      [dotRows(jacobian[0], jacobian[0]), dotRows(jacobian[0], jacobian[1]), dotRows(jacobian[0], jacobian[2])],
      [dotRows(jacobian[1], jacobian[0]), dotRows(jacobian[1], jacobian[1]), dotRows(jacobian[1], jacobian[2])],
      [dotRows(jacobian[2], jacobian[0]), dotRows(jacobian[2], jacobian[1]), dotRows(jacobian[2], jacobian[2])]
    ];
    const scale = Math.max((gram[0][0] + gram[1][1] + gram[2][2]) / 3, 1e-9);
    const normalizedDeterminant =
      Math.max(0, determinant3x3(gram)) / Math.max(scale * scale * scale, 1e-12);
    const singularity = clip(1 - normalizedDeterminant / SINGULARITY_THRESHOLD, 0, 1);
    const damping = MIN_DAMPING + (MAX_DAMPING - MIN_DAMPING) * singularity * singularity;
    const lambda2 = damping * damping;
    const y = solve3x3(
      [
        [gram[0][0] + lambda2, gram[0][1], gram[0][2]],
        [gram[1][0], gram[1][1] + lambda2, gram[1][2]],
        [gram[2][0], gram[2][1], gram[2][2] + lambda2]
      ],
      [error.x, error.y, error.z]
    );
    if (!y) return null;
    return arm.map((_, index) => jacobian[0][index] * y[0] + jacobian[1][index] * y[1] + jacobian[2][index] * y[2]);
  }

  function servoStep(target, dt, ikAngles) {
    if (dt <= 0) return null;
    const saved = copyQpos(data);
    applyArmAngles(ikAngles);
    const current = tcpPosition();
    const error = {
      x: target.x - current.x,
      y: target.y - current.y,
      z: target.z - current.z
    };
    const errorNorm = Math.hypot(error.x, error.y, error.z);
    if (errorNorm < REACH_ERROR) {
      restoreQpos(data, saved);
      mujoco.mj_forward(model, data);
      return { error: errorNorm, reached: true, angles: { ...ikAngles } };
    }

    const scale = Math.min(0.65, Math.max(0.08, GAIN * dt));
    const stepError = { x: error.x * scale, y: error.y * scale, z: error.z * scale };
    const jacobian = computeJacobian(ikAngles);
    const delta = solveDampedLeastSquares(jacobian, stepError);
    if (!delta) {
      restoreQpos(data, saved);
      mujoco.mj_forward(model, data);
      return { error: errorNorm, reached: false, angles: { ...ikAngles } };
    }

    arm.forEach((joint, index) => {
      const limited = clip(delta[index] || 0, -MAX_JOINT_SPEED * dt, MAX_JOINT_SPEED * dt);
      ikAngles[joint.name] = clip(ikAngles[joint.name] + limited, joint.min, joint.max);
    });
    applyArmAngles(ikAngles);
    const after = tcpPosition();
    const afterError = Math.hypot(target.x - after.x, target.y - after.y, target.z - after.z);
    restoreQpos(data, saved);
    mujoco.mj_forward(model, data);
    return { error: afterError, reached: afterError < REACH_ERROR, angles: { ...ikAngles } };
  }

  function clampToWorkspace(point) {
    const next = { x: point.x, y: point.y, z: point.z };
    let clamped = false;
    if (next.z < 0.02) {
      next.z = 0.02;
      clamped = true;
    } else if (next.z > NOMINAL_REACH) {
      next.z = NOMINAL_REACH;
      clamped = true;
    }
    const verticalRatio = clip(next.z / NOMINAL_REACH, 0, 1);
    const planarLimit = Math.max(0.03, NOMINAL_REACH * Math.sqrt(Math.max(0, 1 - verticalRatio * verticalRatio)));
    const planar = Math.hypot(next.x, next.y);
    if (planar > planarLimit) {
      const scale = planarLimit / planar;
      next.x *= scale;
      next.y *= scale;
      clamped = true;
    }
    return { point: next, clamped };
  }

  return {
    bodyId,
    tcpPosition,
    servoStep,
    clampToWorkspace,
    armNames: arm.map((joint) => joint.name)
  };
}
