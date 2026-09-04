import { ARM_JOINTS, homePose } from './kinematics.js';

export const VISION_TARGETS = [
  { id: 'red', body: 'red_cube', labelKey: 'vision.red', color: '#ef5260' },
  { id: 'blue', body: 'blue_block', labelKey: 'vision.blue', color: '#45aef2' },
  { id: 'yellow', body: 'yellow_cylinder', labelKey: 'vision.yellow', color: '#e6d957' }
];

export const STORAGE_ZONES = {
  red: { x: 0.49, y: -0.16, z: 0.1225 },
  blue: { x: 0.49, y: 0.00, z: 0.119 },
  yellow: { x: 0.49, y: 0.16, z: 0.126 }
};

export const STACK_TARGETS = {
  blue: { x: 0.30, y: 0.30, z: 0.119 },
  red: { x: 0.30, y: 0.30, z: 0.1605 },
  yellow: { x: 0.30, y: 0.294, z: 0.209 }
};

const OPEN_WIDTH = 0.046;
const MOVE_TIMEOUT = 4.5;
const PLACE_PRESS_DEPTH = 0.0005;
const SUPPORT_SETTLE_TIME = 0.10;
const TOWER_SETTLE_TIME = 0.35;
const DROP_XY_TOLERANCE = 0.006;
const MAX_PLACEMENT_SPEED = 0.035;
const STACK_RETREAT_CLEARANCE = 0.12;
const STACK_DEPART_OFFSET = { x: -0.05, y: -0.08 };
const STACK_READY_POSE = {
  joint1: 0,
  joint2: 0.5515,
  joint3: 0.2688,
  joint4: 0.3229,
  joint5: 0.0471,
  joint6: 0
};
const PREGRASP_PREPARE_TIMEOUT = 2.4;
const PREGRASP_ALIGN_TIMEOUT = 2.2;
const PREGRASP_JOINT_TOLERANCE = 0.055;
const CAMERA_FACING_POSE = {
  ...STACK_READY_POSE,
  joint4: 0.80
};
const REACTION_CENTER_TIMEOUT = 1.8;
const REACTION_JOINT_TOLERANCE = 0.07;
const REACTION_GRIP_TOLERANCE = 0.006;
const STACK_CENTER_TIMEOUT = 1.4;
const HAPPY_REACTION = [
  { hold: 0.48, timeout: 1.00, offset: { joint6: -0.55 } },
  { hold: 0.48, timeout: 1.00, offset: { joint6: 0.55 } },
  { hold: 0.35, timeout: 0.85, offset: {} },
  { hold: 0.65, timeout: 1.15, offset: { joint5: 0.36 } },
  { hold: 0.40, timeout: 1.00, offset: {}, grip: 0 },
  { hold: 0.48, timeout: 1.10, offset: {}, grip: OPEN_WIDTH }
];
const SHY_REACTION = [
  { hold: 0.75, timeout: 1.25, offset: { joint2: -0.10, joint3: 0.06, joint5: 0.34 } },
  { hold: 0.75, timeout: 1.25, offset: { joint2: -0.14, joint3: 0.10, joint5: 0.48 } },
  { hold: 0.50, timeout: 1.00, offset: {} }
];
const STAGE_PROGRESS = {
  idle: 0,
  preparing: 0.025,
  aligning: 0.055,
  centering: 0.04,
  opening: 0.08,
  approach: 0.22,
  descend: 0.38,
  closing: 0.50,
  lift: 0.64,
  transfer: 0.76,
  place: 0.86,
  release: 0.94,
  retreat: 0.97,
  depart: 0.985,
  settling: 0.99,
  'celebrate-centering': 0.992,
  celebrate: 0.996,
  'shy-centering': 0.992,
  shy: 0.996,
  complete: 1,
  failed: 1
};

function distance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

export function createGraspDemo({
  mujoco,
  model,
  data,
  joints,
  physics,
  ik,
  workspaceOffset = { x: 0, y: 0 },
  getObserverPosition,
  onChange
}) {
  const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
  const tableBodyId = mujoco.mj_name2id(model, bodyType, 'task_table');
  const fingerBodies = new Set([
    mujoco.mj_name2id(model, bodyType, 'gripper_left'),
    mujoco.mj_name2id(model, bodyType, 'gripper_right')
  ]);
  const targets = VISION_TARGETS.map((target) => {
    const bodyId = mujoco.mj_name2id(model, bodyType, target.body);
    const jointAddress = model.body_jntadr[bodyId];
    return {
      ...target,
      bodyId,
      dofAddress: model.jnt_dofadr[jointAddress]
    };
  });
  if (targets.some((target) => target.bodyId < 0)) {
    throw new Error('抓取演示缺少颜色目标 body');
  }

  let selectedId = targets[0].id;
  let running = false;
  let stage = 'idle';
  let stageStartedAt = 0;
  let lastUpdateAt = 0;
  let ikAngles = {};
  let objectStart = null;
  let moveTarget = null;
  let dropTarget = null;
  let mode = 'put-away';
  let stackQueue = [];
  let pendingStackId = null;
  let expectedSupportBodyId = tableBodyId;
  let carryOffset = null;
  let supportContactSince = null;
  let reaction = null;
  let reactionSequence = [];
  let reactionStep = -1;
  let reactionCenterPose = null;
  let pendingFailureReason = '';
  let pregraspPose = null;
  let pregraspAlignedPose = null;
  let alignmentJoint1 = null;
  let lastError = 0;
  let message = '';

  function selected() {
    return targets.find((target) => target.id === selectedId) || targets[0];
  }

  function bodyPosition(bodyId) {
    return {
      x: data.xpos[bodyId * 3],
      y: data.xpos[bodyId * 3 + 1],
      z: data.xpos[bodyId * 3 + 2]
    };
  }

  function workspacePoint(point) {
    return {
      ...point,
      x: point.x + (workspaceOffset.x || 0),
      y: point.y + (workspaceOffset.y || 0),
      z: point.z + (workspaceOffset.z || 0)
    };
  }

  function selectedLinearSpeed() {
    const start = selected().dofAddress;
    return Math.hypot(data.qvel[start], data.qvel[start + 1], data.qvel[start + 2]);
  }

  function snapshot() {
    return {
      running,
      stage,
      progress: STAGE_PROGRESS[stage] ?? 0,
      selectedId,
      error: lastError,
      message,
      mode,
      objectPosition: bodyPosition(selected().bodyId),
      objectStart,
      carryOffset: carryOffset ? { ...carryOffset } : null,
      reaction,
      reactionStep,
      reactionCenterPose: reactionCenterPose ? { ...reactionCenterPose } : null,
      alignmentJoint1
    };
  }

  function notify() {
    onChange?.(snapshot());
  }

  function enter(next, target = null) {
    stage = next;
    stageStartedAt = data.time;
    moveTarget = target;
    lastError = target ? distance(ik.tcpPosition(), target) : 0;
    notify();
  }

  function setSelected(next) {
    if (running || !targets.some((target) => target.id === next)) return false;
    selectedId = next;
    notify();
    return true;
  }

  function currentArmPose() {
    return Object.fromEntries(
      ARM_JOINTS.map((joint) => [joint.name, data.qpos[joints.byName[joint.name].qposadr]])
    );
  }

  function closestJointAngle(desired, current, joint) {
    const candidates = [-1, 0, 1]
      .map((turn) => desired + turn * Math.PI * 2)
      .filter((angle) => angle >= joint.min && angle <= joint.max);
    if (candidates.length) {
      return candidates.reduce((best, angle) =>
        Math.abs(angle - current) < Math.abs(best - current) ? angle : best
      );
    }
    return Math.min(joint.max, Math.max(joint.min, desired));
  }

  function targetJoint1For(point, current) {
    const relativeX = point.x - (workspaceOffset.x || 0);
    const relativeY = point.y - (workspaceOffset.y || 0);
    const desired = -Math.atan2(relativeY, relativeX);
    return closestJointAngle(desired, current, joints.byName.joint1);
  }

  function armPoseReached(pose, tolerance = PREGRASP_JOINT_TOLERANCE) {
    return ARM_JOINTS.every((joint) => {
      const actual = data.qpos[joints.byName[joint.name].qposadr];
      return Math.abs((pose[joint.name] ?? actual) - actual) < tolerance;
    });
  }

  function begin(nextId, nextDropTarget, nextMode = 'put-away') {
    selectedId = nextId;
    pendingStackId = null;
    mode = nextMode;
    objectStart = bodyPosition(selected().bodyId);
    dropTarget = nextDropTarget;
    expectedSupportBodyId =
      nextMode === 'stack' && nextId === 'red'
        ? targets.find((target) => target.id === 'blue').bodyId
        : nextMode === 'stack' && nextId === 'yellow'
          ? targets.find((target) => target.id === 'red').bodyId
          : tableBodyId;
    carryOffset = null;
    supportContactSince = null;
    reaction = null;
    reactionSequence = [];
    reactionStep = -1;
    reactionCenterPose = null;
    pendingFailureReason = '';
    ikAngles = currentArmPose();
    pregraspPose = null;
    pregraspAlignedPose = null;
    alignmentJoint1 = null;
    // The stack path uses the original IK entry; pre-aligning J1 for red
    // sweeps the wrist through the yellow cylinder.
    if (nextMode === 'stack') {
      message = '';
      lastUpdateAt = data.time;
      physics.setTarget('joint7', OPEN_WIDTH);
      enter('opening');
      return;
    }
    pregraspPose = { ...ikAngles };
    alignmentJoint1 = targetJoint1For(objectStart, ikAngles.joint1);
    pregraspAlignedPose = { ...pregraspPose, joint1: alignmentJoint1 };
    message = '';
    lastUpdateAt = data.time;
    physics.setTarget('joint7', OPEN_WIDTH);
    physics.setTargets(pregraspPose);
    enter('preparing');
  }

  function start(next = selectedId) {
    if (running) return false;
    setSelected(next);
    running = true;
    stackQueue = [];
    begin(next, workspacePoint(STORAGE_ZONES[next]));
    return true;
  }

  function startStack() {
    if (running) return false;
    running = true;
    stackQueue = ['red', 'yellow'];
    begin('blue', workspacePoint(STACK_TARGETS.blue), 'stack');
    return true;
  }

  function cancel(reason = 'cancelled') {
    if (!running && stage === 'idle') return false;
    running = false;
    stage = 'idle';
    message = reason;
    lastError = 0;
    moveTarget = null;
    stackQueue = [];
    pendingStackId = null;
    mode = 'put-away';
    carryOffset = null;
    supportContactSince = null;
    reaction = null;
    reactionSequence = [];
    reactionStep = -1;
    reactionCenterPose = null;
    pendingFailureReason = '';
    pregraspPose = null;
    pregraspAlignedPose = null;
    alignmentJoint1 = null;
    notify();
    return true;
  }

  function fail(reason) {
    if (mode === 'stack') {
      pendingFailureReason = reason;
      startReaction('shy');
      return;
    }
    running = false;
    message = reason;
    physics.setTarget('joint7', OPEN_WIDTH);
    enter('failed');
  }

  function moveStep(dt) {
    const result = ik.servoStep(moveTarget, Math.max(0.001, Math.min(0.035, dt)), ikAngles);
    if (!result) return false;
    ikAngles = result.angles;
    physics.setTargets(ikAngles);
    lastError = distance(ik.tcpPosition(), moveTarget);
    return lastError < 0.006;
  }

  function timedOut() {
    return data.time - stageStartedAt > MOVE_TIMEOUT;
  }

  function objectHasSupportContact() {
    const selectedBodyId = selected().bodyId;
    const count = Math.min(data.ncon, data.contact.size());
    for (let index = 0; index < count; index += 1) {
      const contact = data.contact.get(index);
      if (!contact) continue;
      const body1 = model.geom_bodyid[contact.geom1];
      const body2 = model.geom_bodyid[contact.geom2];
      if (
        (body1 === selectedBodyId && body2 === expectedSupportBodyId) ||
        (body2 === selectedBodyId && body1 === expectedSupportBodyId)
      ) return true;
    }
    return false;
  }

  function objectHasGripContact() {
    const selectedBodyId = selected().bodyId;
    const count = Math.min(data.ncon, data.contact.size());
    for (let index = 0; index < count; index += 1) {
      const contact = data.contact.get(index);
      if (!contact) continue;
      const body1 = model.geom_bodyid[contact.geom1];
      const body2 = model.geom_bodyid[contact.geom2];
      const touchesSelected = body1 === selectedBodyId || body2 === selectedBodyId;
      if (touchesSelected && (fingerBodies.has(body1) || fingerBodies.has(body2))) return true;
    }
    return false;
  }

  function objectIsAtDropTarget() {
    const object = bodyPosition(selected().bodyId);
    const horizontalError = Math.hypot(object.x - dropTarget.x, object.y - dropTarget.y);
    const verticalError = object.z - dropTarget.z;
    return horizontalError < DROP_XY_TOLERANCE && verticalError > -0.018 && verticalError < 0.030;
  }

  function rememberCarryOffset() {
    const object = bodyPosition(selected().bodyId);
    const tcp = ik.tcpPosition();
    carryOffset = {
      x: object.x - tcp.x,
      y: object.y - tcp.y,
      z: object.z - tcp.z
    };
  }

  function gripAlignedDropTarget(z) {
    const object = bodyPosition(selected().bodyId);
    const tcp = ik.tcpPosition();
    return {
      x: tcp.x + dropTarget.x - object.x,
      y: tcp.y + dropTarget.y - object.y,
      z
    };
  }

  function trackObjectOverDropTarget() {
    const object = bodyPosition(selected().bodyId);
    const tcp = ik.tcpPosition();
    moveTarget.x = tcp.x + dropTarget.x - object.x;
    moveTarget.y = tcp.y + dropTarget.y - object.y;
  }

  function verticalRetreatTarget() {
    const tcp = ik.tcpPosition();
    return {
      x: tcp.x,
      y: tcp.y,
      z: dropTarget.z + (mode === 'stack' ? STACK_RETREAT_CLEARANCE : 0.11)
    };
  }

  function reactionPose(overrides = {}, relative = false) {
    const base = reactionCenterPose || homePose(joints);
    const pose = {
      ...base,
      joint7: OPEN_WIDTH,
      joint_left: OPEN_WIDTH,
      joint_right: OPEN_WIDTH
    };
    Object.entries(overrides).forEach(([name, value]) => {
      if (!relative) {
        pose[name] = value;
        return;
      }
      const joint = joints.byName[name];
      pose[name] = Math.min(joint.max, Math.max(joint.min, base[name] + value));
    });
    return pose;
  }

  function reactionStepPose(step) {
    const pose = reactionPose(step.offset, true);
    if (Number.isFinite(step.grip)) {
      pose.joint7 = step.grip;
      pose.joint_left = step.grip;
      pose.joint_right = step.grip;
    }
    return pose;
  }

  function cameraFacingPose() {
    const pose = { ...CAMERA_FACING_POSE };
    const observer = getObserverPosition?.();
    if (!Number.isFinite(observer?.x) || !Number.isFinite(observer?.y)) return pose;
    const azimuth = Math.atan2(
      observer.y - (workspaceOffset.y || 0),
      observer.x - (workspaceOffset.x || 0)
    );
    const joint = joints.byName.joint1;
    pose.joint1 = Math.min(joint.max, Math.max(joint.min, -azimuth));
    return pose;
  }

  function reactionPoseReached(pose) {
    const armReached = armPoseReached(pose, REACTION_JOINT_TOLERANCE);
    const gripReached =
      !Number.isFinite(pose.joint7) ||
      Math.abs(data.qpos[joints.byName.joint7.qposadr] - pose.joint7) < REACTION_GRIP_TOLERANCE;
    return armReached && gripReached;
  }

  function startReaction(nextReaction) {
    reaction = nextReaction;
    reactionSequence = nextReaction === 'happy' ? HAPPY_REACTION : SHY_REACTION;
    reactionStep = -1;
    reactionCenterPose = cameraFacingPose();
    physics.setTargets(reactionPose());
    enter(nextReaction === 'happy' ? 'celebrate-centering' : 'shy-centering');
  }

  function startReactionStep(index) {
    reactionStep = index;
    const step = reactionSequence[index];
    physics.setTargets(reactionStepPose(step));
    enter(reaction === 'happy' ? 'celebrate' : 'shy');
  }

  function finishReaction() {
    running = false;
    if (reaction === 'happy') {
      message = 'stack-complete';
      enter('complete');
    } else {
      message = pendingFailureReason || 'stack-failed';
      enter('failed');
    }
  }

  function updateReaction(now, elapsed) {
    if (reactionStep < 0) {
      if (reactionPoseReached(reactionPose()) || elapsed > REACTION_CENTER_TIMEOUT) {
        startReactionStep(0);
      }
      return;
    }
    const step = reactionSequence[reactionStep];
    const targetPose = reactionStepPose(step);
    if ((elapsed >= step.hold && reactionPoseReached(targetPose)) || elapsed > step.timeout) {
      const next = reactionStep + 1;
      if (next < reactionSequence.length) startReactionStep(next);
      else finishReaction();
    }
  }

  function finishPlacement() {
    if (mode === 'stack' && stackQueue.length > 0) {
      pendingStackId = stackQueue.shift();
      physics.setTargets({
        ...STACK_READY_POSE,
        joint7: OPEN_WIDTH,
        joint_left: OPEN_WIDTH,
        joint_right: OPEN_WIDTH
      });
      enter('centering');
    } else {
      if (mode === 'stack') startReaction('happy');
      else {
        running = false;
        message = 'complete';
        enter('complete');
      }
    }
  }

  function update() {
    if (!running) return snapshot();
    const now = data.time;
    const dt = Math.max(0.001, now - lastUpdateAt);
    lastUpdateAt = now;
    const elapsed = now - stageStartedAt;

    if (stage === 'preparing') {
      if (armPoseReached(pregraspPose) || elapsed > PREGRASP_PREPARE_TIMEOUT) {
        physics.setTargets(pregraspAlignedPose);
        enter('aligning');
      }
    } else if (stage === 'aligning') {
      if (armPoseReached(pregraspAlignedPose) || elapsed > PREGRASP_ALIGN_TIMEOUT) {
        ikAngles = currentArmPose();
        enter('opening');
      }
    } else if (stage === 'centering') {
      if (reactionPoseReached(STACK_READY_POSE) || elapsed > STACK_CENTER_TIMEOUT) {
        begin(pendingStackId, workspacePoint(STACK_TARGETS[pendingStackId]), 'stack');
      }
    } else if (stage === 'opening') {
      const width = data.qpos[joints.byName.joint7.qposadr];
      if ((width > OPEN_WIDTH - 0.006 && elapsed > 0.18) || elapsed > 1.1) {
        enter('approach', { x: objectStart.x, y: objectStart.y, z: objectStart.z + 0.10 });
      }
    } else if (stage === 'approach') {
      if (moveStep(dt)) enter('descend', { x: objectStart.x, y: objectStart.y, z: objectStart.z + 0.003 });
      else if (timedOut()) fail('approach-timeout');
    } else if (stage === 'descend') {
      const gripReady =
        elapsed > 0.15 &&
        distance(ik.tcpPosition(), moveTarget) < 0.018 &&
        objectHasGripContact();
      if (moveStep(dt) || gripReady) {
        physics.setTarget('joint7', 0);
        enter('closing');
      } else if (timedOut()) fail('descend-timeout');
    } else if (stage === 'closing') {
      if (elapsed > 0.8) enter('lift', { x: objectStart.x, y: objectStart.y, z: objectStart.z + 0.13 });
    } else if (stage === 'lift') {
      if (moveStep(dt)) {
        const object = bodyPosition(selected().bodyId);
        if (object.z < objectStart.z + 0.035) fail('grasp-missed');
        else {
          rememberCarryOffset();
          enter('transfer', gripAlignedDropTarget(objectStart.z + 0.13));
        }
      } else if (timedOut()) fail('lift-timeout');
    } else if (stage === 'transfer') {
      if (moveStep(dt)) {
        const offset = carryOffset || { z: -0.006 };
        enter('place', gripAlignedDropTarget(dropTarget.z - offset.z - PLACE_PRESS_DEPTH));
      }
      else if (timedOut()) fail('transfer-timeout');
    } else if (stage === 'place') {
      trackObjectOverDropTarget();
      moveStep(dt);
      const supported =
        objectIsAtDropTarget() &&
        objectHasSupportContact() &&
        selectedLinearSpeed() < MAX_PLACEMENT_SPEED;
      if (supported && supportContactSince === null) supportContactSince = now;
      else if (!supported) supportContactSince = null;
      if (supportContactSince !== null && now - supportContactSince >= SUPPORT_SETTLE_TIME) {
        physics.setTarget('joint7', OPEN_WIDTH);
        enter('release');
      } else if (timedOut()) fail('place-timeout');
    } else if (stage === 'release') {
      const width = data.qpos[joints.byName.joint7.qposadr];
      const gripperIsClear = width > OPEN_WIDTH - 0.008;
      if ((elapsed > 0.35 && gripperIsClear) || elapsed > 1.1) {
        enter('retreat', verticalRetreatTarget());
      }
    } else if (stage === 'retreat') {
      if (moveStep(dt)) {
        if (mode === 'stack') {
          enter('depart', {
            x: dropTarget.x + STACK_DEPART_OFFSET.x,
            y: dropTarget.y + STACK_DEPART_OFFSET.y,
            z: moveTarget.z
          });
        } else enter('settling');
      } else if (timedOut()) fail('retreat-timeout');
    } else if (stage === 'depart') {
      if (moveStep(dt)) enter('settling');
      else if (timedOut()) fail('depart-timeout');
    } else if (stage === 'settling') {
      if (
        (elapsed >= TOWER_SETTLE_TIME && selectedLinearSpeed() < MAX_PLACEMENT_SPEED * 0.5) ||
        elapsed > 1.2
      ) finishPlacement();
    } else if (
      stage === 'celebrate-centering' ||
      stage === 'celebrate' ||
      stage === 'shy-centering' ||
      stage === 'shy'
    ) {
      updateReaction(now, elapsed);
    }
    notify();
    return snapshot();
  }

  function reset() {
    running = false;
    stage = 'idle';
    stageStartedAt = data.time;
    lastUpdateAt = data.time;
    objectStart = null;
    moveTarget = null;
    dropTarget = null;
    pendingStackId = null;
    expectedSupportBodyId = tableBodyId;
    carryOffset = null;
    supportContactSince = null;
    reaction = null;
    reactionSequence = [];
    reactionStep = -1;
    reactionCenterPose = null;
    pendingFailureReason = '';
    pregraspPose = null;
    pregraspAlignedPose = null;
    alignmentJoint1 = null;
    lastError = 0;
    message = '';
    notify();
  }

  return {
    targets: targets.map(({ bodyId, ...target }) => ({ ...target })),
    setSelected,
    start,
    startStack,
    cancel,
    update,
    reset,
    state: snapshot,
    isRunning() {
      return running;
    }
  };
}
