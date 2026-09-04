import {
  AGV_FOOTPRINT_CLEARANCE,
  AGV_NAV_BOUNDS,
  LAB_NAV_OBSTACLES
} from './agv-path-planner.js';

const WHEEL_JOINTS = [
  'wheel_front_left_joint',
  'wheel_front_right_joint',
  'wheel_rear_left_joint',
  'wheel_rear_right_joint'
];

const WHEEL_ACTUATORS = [
  'wheel_front_left_motor',
  'wheel_front_right_motor',
  'wheel_rear_left_motor',
  'wheel_rear_right_motor'
];

export const AGV_MAX_LINEAR_SPEED = 0.8;
export const AGV_MAX_ANGULAR_SPEED = 1.8;
export const AGV_FORWARD_SPEED = 0.65;
export const AGV_TURN_SPEED = 1.4;
export const AGV_STEPS_PER_FRAME = 8;

function clip(value, limit) {
  return Math.min(limit, Math.max(-limit, value));
}

function namedId(mujoco, model, type, name) {
  const id = mujoco.mj_name2id(model, type, name);
  if (id < 0) throw new Error(`找不到 AGV 模型元素 ${name}`);
  return id;
}

/** Collision-aware velocity controller for the browser simulation. */
export function createAgvController(mujoco, model, data) {
  const jointType = mujoco.mjtObj.mjOBJ_JOINT.value;
  const actuatorType = mujoco.mjtObj.mjOBJ_ACTUATOR.value;
  const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
  const equalityType = mujoco.mjtObj.mjOBJ_EQUALITY.value;
  const baseJointId = namedId(mujoco, model, jointType, 'agv_freejoint');
  const baseQposAddress = model.jnt_qposadr[baseJointId];
  const baseDofAddress = model.jnt_dofadr[baseJointId];
  const baseHeight = data.qpos[baseQposAddress + 2];
  const baseBodyId = namedId(mujoco, model, bodyType, 'base_link');
  const dynamicPalletBodyId = mujoco.mj_name2id(model, bodyType, 'dynamic_pallet');
  const dockAnchorBodyId = mujoco.mj_name2id(model, bodyType, 'agv_dock_anchor');
  const dockAnchorMocapId =
    dockAnchorBodyId >= 0 ? model.body_mocapid[dockAnchorBodyId] : -1;
  const dockWeldId = mujoco.mj_name2id(model, equalityType, 'agv_dock_weld');
  const wheelDofAddresses = WHEEL_JOINTS.map((name) => {
    const id = namedId(mujoco, model, jointType, name);
    return model.jnt_dofadr[id];
  });
  const wheelActuatorIds = WHEEL_ACTUATORS.map((name) =>
    namedId(mujoco, model, actuatorType, name)
  );

  const wheelRadius = 0.05;
  const wheelTrack = 0.46;
  const kp = 2.5;
  const kd = 0.12;
  const torqueLimit = 8;
  const command = { linear: 0, angular: 0 };
  let heldPose = null;

  function setDockConstraintTimeConstant(value) {
    if (dockWeldId < 0) return;
    model.eq_solref[dockWeldId * 2] = value;
    model.eq_solref[dockWeldId * 2 + 1] = 1;
  }

  function writeDockAnchor(x, y, z, qw, qx, qy, qz) {
    if (dockAnchorMocapId < 0) return;
    const positionOffset = dockAnchorMocapId * 3;
    const quaternionOffset = dockAnchorMocapId * 4;
    data.mocap_pos[positionOffset] = x;
    data.mocap_pos[positionOffset + 1] = y;
    data.mocap_pos[positionOffset + 2] = z;
    data.mocap_quat[quaternionOffset] = qw;
    data.mocap_quat[quaternionOffset + 1] = qx;
    data.mocap_quat[quaternionOffset + 2] = qy;
    data.mocap_quat[quaternionOffset + 3] = qz;
  }

  function syncReleasedDockAnchor() {
    if (heldPose) return;
    writeDockAnchor(
      data.qpos[baseQposAddress],
      data.qpos[baseQposAddress + 1],
      data.qpos[baseQposAddress + 2],
      data.qpos[baseQposAddress + 3],
      data.qpos[baseQposAddress + 4],
      data.qpos[baseQposAddress + 5],
      data.qpos[baseQposAddress + 6]
    );
  }

  function setCommand(linear = 0, angular = 0) {
    command.linear = clip(Number(linear) || 0, AGV_MAX_LINEAR_SPEED);
    command.angular = clip(Number(angular) || 0, AGV_MAX_ANGULAR_SPEED);
  }

  function stop() {
    setCommand(0, 0);
  }

  function collisionConstrainedVelocity(forwardX, forwardY) {
    let velocityX = forwardX * command.linear;
    let velocityY = forwardY * command.linear;
    let blocked = false;
    const x = data.qpos[baseQposAddress];
    const y = data.qpos[baseQposAddress + 1];
    const lookahead = Math.max(model.opt.timestep, 0.002) * 2;
    const nextX = x + velocityX * lookahead;
    const nextY = y + velocityY * lookahead;

    if (nextX < AGV_NAV_BOUNDS.minX && velocityX < 0) {
      velocityX = 0;
      blocked = true;
    } else if (nextX > AGV_NAV_BOUNDS.maxX && velocityX > 0) {
      velocityX = 0;
      blocked = true;
    }
    if (nextY < AGV_NAV_BOUNDS.minY && velocityY < 0) {
      velocityY = 0;
      blocked = true;
    } else if (nextY > AGV_NAV_BOUNDS.maxY && velocityY > 0) {
      velocityY = 0;
      blocked = true;
    }

    for (const configuredObstacle of LAB_NAV_OBSTACLES) {
      let obstacleX = configuredObstacle.x;
      let obstacleY = configuredObstacle.y;
      if (configuredObstacle.id === 'dynamic-pallet' && dynamicPalletBodyId >= 0) {
        obstacleX = data.xpos[dynamicPalletBodyId * 3];
        obstacleY = data.xpos[dynamicPalletBodyId * 3 + 1];
      }
      const closestX = Math.max(
        obstacleX - configuredObstacle.halfX,
        Math.min(obstacleX + configuredObstacle.halfX, nextX)
      );
      const closestY = Math.max(
        obstacleY - configuredObstacle.halfY,
        Math.min(obstacleY + configuredObstacle.halfY, nextY)
      );
      let normalX = nextX - closestX;
      let normalY = nextY - closestY;
      let distance = Math.hypot(normalX, normalY);
      if (distance >= AGV_FOOTPRINT_CLEARANCE) continue;
      if (distance < 1e-9) {
        const relativeX = nextX - obstacleX;
        const relativeY = nextY - obstacleY;
        const exitX = configuredObstacle.halfX - Math.abs(relativeX);
        const exitY = configuredObstacle.halfY - Math.abs(relativeY);
        if (exitX < exitY) {
          normalX = Math.sign(relativeX) || 1;
          normalY = 0;
        } else {
          normalX = 0;
          normalY = Math.sign(relativeY) || 1;
        }
        distance = 1;
      }
      const unitX = normalX / distance;
      const unitY = normalY / distance;
      const outwardSpeed = velocityX * unitX + velocityY * unitY;
      if (outwardSpeed >= 0) continue;
      velocityX -= outwardSpeed * unitX;
      velocityY -= outwardSpeed * unitY;
      blocked = true;
    }
    return { velocityX, velocityY, angular: blocked ? 0 : command.angular };
  }

  function apply() {
    syncReleasedDockAnchor();
    const xmatOffset = baseBodyId * 9;
    const constrained = collisionConstrainedVelocity(
      data.xmat[xmatOffset],
      data.xmat[xmatOffset + 3]
    );
    const effectiveLinear =
      constrained.velocityX * data.xmat[xmatOffset] +
      constrained.velocityY * data.xmat[xmatOffset + 3];
    const leftTarget = (effectiveLinear - 0.5 * wheelTrack * constrained.angular) / wheelRadius;
    const rightTarget = (effectiveLinear + 0.5 * wheelTrack * constrained.angular) / wheelRadius;
    const targets = [leftTarget, rightTarget, leftTarget, rightTarget];

    wheelDofAddresses.forEach((dofAddress, index) => {
      const velocity = data.qvel[dofAddress];
      data.ctrl[wheelActuatorIds[index]] = clip(
        kp * (targets[index] - velocity) - kd * velocity,
        torqueLimit
      );
    });

    data.qvel[baseDofAddress] = constrained.velocityX;
    data.qvel[baseDofAddress + 1] = constrained.velocityY;
    data.qvel[baseDofAddress + 3] = 0;
    data.qvel[baseDofAddress + 4] = 0;
    data.qvel[baseDofAddress + 5] = constrained.angular;
  }

  function state() {
    return { ...command };
  }

  function pose() {
    const xmatOffset = baseBodyId * 9;
    return {
      x: data.qpos[baseQposAddress],
      y: data.qpos[baseQposAddress + 1],
      yaw: Math.atan2(data.xmat[xmatOffset + 3], data.xmat[xmatOffset])
    };
  }

  function writePlanarPose(x, y, yaw = 0) {
    data.qpos[baseQposAddress] = x;
    data.qpos[baseQposAddress + 1] = y;
    data.qpos[baseQposAddress + 2] = baseHeight;
    data.qpos[baseQposAddress + 3] = Math.cos(yaw * 0.5);
    data.qpos[baseQposAddress + 4] = 0;
    data.qpos[baseQposAddress + 5] = 0;
    data.qpos[baseQposAddress + 6] = Math.sin(yaw * 0.5);
    for (let index = 0; index < 6; index += 1) data.qvel[baseDofAddress + index] = 0;
  }

  function setPlanarPose(x, y, yaw = 0) {
    writePlanarPose(x, y, yaw);
    stop();
    mujoco.mj_forward(model, data);
  }

  function holdPlanarPose(next) {
    heldPose = { x: next.x, y: next.y, yaw: next.yaw || 0 };
    setPlanarPose(heldPose.x, heldPose.y, heldPose.yaw);
    if (dockWeldId >= 0 && dockAnchorMocapId >= 0) {
      writeDockAnchor(
        heldPose.x,
        heldPose.y,
        baseHeight,
        Math.cos(heldPose.yaw * 0.5),
        0,
        0,
        Math.sin(heldPose.yaw * 0.5)
      );
      setDockConstraintTimeConstant(0.001);
      mujoco.mj_forward(model, data);
    }
  }

  function releasePlanarPose() {
    heldPose = null;
    setDockConstraintTimeConstant(100);
    syncReleasedDockAnchor();
    mujoco.mj_forward(model, data);
  }

  function enforceHeldPose() {
    if (!heldPose) return false;
    // New AGV models use a real weld constraint so arm reaction forces are
    // transferred into the dock. Keep the old kinematic fallback for scenes
    // generated before the dock constraint was introduced.
    if (dockWeldId < 0 || dockAnchorMocapId < 0) {
      writePlanarPose(heldPose.x, heldPose.y, heldPose.yaw);
      mujoco.mj_forward(model, data);
    }
    return true;
  }

  return {
    setCommand,
    stop,
    apply,
    state,
    pose,
    setPlanarPose,
    holdPlanarPose,
    releasePlanarPose,
    enforceHeldPose
  };
}
