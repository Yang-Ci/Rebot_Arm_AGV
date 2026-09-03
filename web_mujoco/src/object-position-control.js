export const OBJECT_MOVE_STEP = 0.015;

const TABLE_BOUNDS = { minX: 0.13, maxX: 0.63, minY: -0.45, maxY: 0.45 };
const TABLE_SURFACE_Z = 0.10;
const EDGE_CLEARANCE = 0.008;
const OBJECT_CLEARANCE = 0.004;

export const MOVABLE_OBJECTS = [
  { id: 'red', body: 'red_cube', shape: 'box', halfX: 0.0225, halfY: 0.0225, halfZ: 0.0225 },
  { id: 'blue', body: 'blue_block', shape: 'box', halfX: 0.026, halfY: 0.016, halfZ: 0.019 },
  {
    id: 'yellow', body: 'yellow_cylinder', shape: 'cylinder', radius: 0.020,
    halfX: 0.020, halfY: 0.020, halfZ: 0.026
  }
];

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function normalizeQuaternion(value) {
  const w = Number(value?.w);
  const x = Number(value?.x);
  const y = Number(value?.y);
  const z = Number(value?.z);
  const length = Math.hypot(w, x, y, z);
  if (!Number.isFinite(length) || length < 1e-9) return null;
  return { w: w / length, x: x / length, y: y / length, z: z / length };
}

function yawQuaternion(yaw) {
  return { w: Math.cos(yaw * 0.5), x: 0, y: 0, z: Math.sin(yaw * 0.5) };
}

function yawOf(quaternion) {
  const { w, x, y, z } = quaternion;
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

function rotationMatrix(quaternion) {
  const { w, x, y, z } = quaternion;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)
  ];
}

function footprint(object, quaternion) {
  const matrix = rotationMatrix(quaternion);
  if (object.shape === 'cylinder') {
    const extent = (axis) =>
      object.radius * Math.sqrt(Math.max(0, 1 - axis * axis)) +
      object.halfZ * Math.abs(axis);
    return {
      halfX: extent(matrix[2]),
      halfY: extent(matrix[5]),
      halfZ: extent(matrix[8])
    };
  }
  return {
    halfX: Math.abs(matrix[0]) * object.halfX + Math.abs(matrix[1]) * object.halfY + Math.abs(matrix[2]) * object.halfZ,
    halfY: Math.abs(matrix[3]) * object.halfX + Math.abs(matrix[4]) * object.halfY + Math.abs(matrix[5]) * object.halfZ,
    halfZ: Math.abs(matrix[6]) * object.halfX + Math.abs(matrix[7]) * object.halfY + Math.abs(matrix[8]) * object.halfZ
  };
}

export function createObjectPositionController({ mujoco, model, data, onChange }) {
  const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
  const objects = MOVABLE_OBJECTS.map((spec) => {
    const bodyId = mujoco.mj_name2id(model, bodyType, spec.body);
    if (bodyId < 0) throw new Error(`物体位置控制缺少 body：${spec.body}`);
    const jointAddress = model.body_jntadr[bodyId];
    return {
      ...spec,
      bodyId,
      qposAddress: model.jnt_qposadr[jointAddress],
      dofAddress: model.jnt_dofadr[jointAddress]
    };
  });

  let enabled = false;
  let selectedId = objects[0].id;
  let lastResult = null;
  let heldPose = null;

  function selected() {
    return objects.find((object) => object.id === selectedId) || objects[0];
  }

  function positionOf(object) {
    return {
      x: data.xpos[object.bodyId * 3],
      y: data.xpos[object.bodyId * 3 + 1],
      z: data.xpos[object.bodyId * 3 + 2]
    };
  }

  function quaternionOf(object) {
    const address = object.qposAddress;
    return normalizeQuaternion({
      w: data.qpos[address + 3], x: data.qpos[address + 4],
      y: data.qpos[address + 5], z: data.qpos[address + 6]
    }) || { w: 1, x: 0, y: 0, z: 0 };
  }

  function poseOf(object) {
    const quaternion = quaternionOf(object);
    return { ...positionOf(object), quaternion, yaw: yawOf(quaternion) };
  }

  function captureHeldPose() {
    heldPose = enabled ? poseOf(selected()) : null;
  }

  function snapshot() {
    const pose = poseOf(selected());
    return {
      enabled,
      selectedId,
      position: { x: pose.x, y: pose.y, z: pose.z },
      quaternion: { ...pose.quaternion },
      yaw: pose.yaw,
      lastResult
    };
  }

  function notify() {
    onChange?.(snapshot());
  }

  function overlapsAnother(selectedObject, x, y, quaternion) {
    const selectedFootprint = footprint(selectedObject, quaternion);
    return objects.some((other) => {
      if (other.id === selectedObject.id) return false;
      const pose = poseOf(other);
      const otherFootprint = footprint(other, pose.quaternion);
      return (
        Math.abs(x - pose.x) < selectedFootprint.halfX + otherFootprint.halfX + OBJECT_CLEARANCE &&
        Math.abs(y - pose.y) < selectedFootprint.halfY + otherFootprint.halfY + OBJECT_CLEARANCE
      );
    });
  }

  function writePose(object, pose, forward = true) {
    const address = object.qposAddress;
    data.qpos[address] = pose.x;
    data.qpos[address + 1] = pose.y;
    data.qpos[address + 2] = pose.z;
    data.qpos[address + 3] = pose.quaternion.w;
    data.qpos[address + 4] = pose.quaternion.x;
    data.qpos[address + 5] = pose.quaternion.y;
    data.qpos[address + 6] = pose.quaternion.z;
    for (let index = 0; index < 6; index += 1) data.qvel[object.dofAddress + index] = 0;
    if (forward) mujoco.mj_forward(model, data);
  }

  function setEnabled(next) {
    enabled = Boolean(next);
    lastResult = null;
    captureHeldPose();
    notify();
    return enabled;
  }

  function setSelected(next) {
    if (!objects.some((object) => object.id === next)) return false;
    selectedId = next;
    lastResult = null;
    captureHeldPose();
    notify();
    return true;
  }

  function updatePose(nextX, nextY, nextQuaternion) {
    const quaternion = normalizeQuaternion(nextQuaternion);
    if (!enabled || !Number.isFinite(nextX) || !Number.isFinite(nextY) || !quaternion) {
      lastResult = { ok: false, reason: 'disabled' };
      notify();
      return lastResult;
    }

    const object = selected();
    const current = poseOf(object);
    const rotated = footprint(object, quaternion);
    const minX = TABLE_BOUNDS.minX + rotated.halfX + EDGE_CLEARANCE;
    const maxX = TABLE_BOUNDS.maxX - rotated.halfX - EDGE_CLEARANCE;
    const minY = TABLE_BOUNDS.minY + rotated.halfY + EDGE_CLEARANCE;
    const maxY = TABLE_BOUNDS.maxY - rotated.halfY - EDGE_CLEARANCE;
    const x = clamp(nextX, minX, maxX);
    const y = clamp(nextY, minY, maxY);
    const dot = Math.abs(
      quaternion.w * current.quaternion.w + quaternion.x * current.quaternion.x +
      quaternion.y * current.quaternion.y + quaternion.z * current.quaternion.z
    );

    if (Math.hypot(x - current.x, y - current.y) < 1e-8 && 1 - dot < 1e-9) {
      lastResult = { ok: false, reason: 'boundary', position: current };
      notify();
      return lastResult;
    }
    if (overlapsAnother(object, x, y, quaternion)) {
      lastResult = { ok: false, reason: 'overlap', position: current };
      notify();
      return lastResult;
    }

    const pose = {
      x,
      y,
      z: TABLE_SURFACE_Z + rotated.halfZ,
      quaternion
    };
    writePose(object, pose);
    heldPose = pose;
    lastResult = {
      ok: true,
      clamped: Math.abs(x - nextX) > 1e-8 || Math.abs(y - nextY) > 1e-8,
      position: positionOf(object),
      quaternion: { ...quaternion },
      yaw: yawOf(quaternion)
    };
    notify();
    return lastResult;
  }

  function move(dx, dy) {
    const current = poseOf(selected());
    return updatePose(current.x + dx, current.y + dy, current.quaternion);
  }

  function moveTo(x, y) {
    const current = poseOf(selected());
    return updatePose(x, y, current.quaternion);
  }

  function rotateTo(yaw) {
    const current = poseOf(selected());
    return updatePose(current.x, current.y, yawQuaternion(yaw));
  }

  function setQuaternion(quaternion) {
    const current = poseOf(selected());
    return updatePose(current.x, current.y, quaternion);
  }

  function enforcePose() {
    if (!enabled || !heldPose) return false;
    writePose(selected(), heldPose, false);
    return true;
  }

  function reset() {
    enabled = false;
    heldPose = null;
    lastResult = null;
    notify();
  }

  return {
    objects: objects.map(({ bodyId, qposAddress, dofAddress, ...object }) => ({ ...object })),
    setEnabled,
    setSelected,
    move,
    moveTo,
    rotateTo,
    setQuaternion,
    enforcePose,
    reset,
    state: snapshot,
    isEnabled() {
      return enabled;
    }
  };
}
