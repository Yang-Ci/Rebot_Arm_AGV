import { ARM_JOINTS, homePose } from './kinematics.js';

/** Gains and limits copied from rebotarm_mujoco_rs/mujoco_sync.py. */
export const ARM_KP = [80, 100, 100, 35, 25, 18];
export const ARM_KD = [8, 10, 10, 4, 3, 2.5];
export const ARM_TAU_LIMIT = [36, 36, 36, 14, 14, 14];
export const GRIPPER_KP = 300;
export const GRIPPER_KD = 40;
export const GRIPPER_TAU_LIMIT = 150;

/** Display ~60 Hz with 4 physics steps (timestep 0.001 → ~240 Hz). */
export const STEPS_PER_FRAME = 4;

function clip(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function namedId(mujoco, model, type, name) {
  const id = mujoco.mj_name2id(model, type, name);
  if (id < 0) throw new Error(`找不到 ${name}`);
  return id;
}

export function createPhysicsController(
  mujoco,
  model,
  data,
  joints,
  { beforeStep, afterStep } = {}
) {
  const jointType = mujoco.mjtObj.mjOBJ_JOINT.value;
  const actuatorType = mujoco.mjtObj.mjOBJ_ACTUATOR.value;

  const arm = ARM_JOINTS.map((joint, index) => {
    const id = namedId(mujoco, model, jointType, joint.name);
    return {
      name: joint.name,
      qposadr: model.jnt_qposadr[id],
      dofadr: model.jnt_dofadr[id],
      actuatorId: namedId(mujoco, model, actuatorType, `${joint.name}_motor`),
      kp: ARM_KP[index],
      kd: ARM_KD[index],
      tauLimit: ARM_TAU_LIMIT[index]
    };
  });

  const gripperJointId = namedId(mujoco, model, jointType, 'joint7');
  const gripper = {
    qposadr: model.jnt_qposadr[gripperJointId],
    dofadr: model.jnt_dofadr[gripperJointId],
    actuatorId: namedId(mujoco, model, actuatorType, 'joint7_motor')
  };

  const targets = homePose(joints);

  function setTarget(name, value) {
    const joint = joints.byName[name];
    if (!joint || !Number.isFinite(value)) return;
    const next = clip(value, joint.min, joint.max);
    targets[name] = next;
    if (name === 'joint7') {
      targets.joint_left = next;
      targets.joint_right = next;
    }
  }

  function setTargets(pose) {
    Object.entries(pose).forEach(([name, value]) => setTarget(name, value));
  }

  function applyCtrl() {
    for (const joint of arm) {
      const q = data.qpos[joint.qposadr];
      const qd = data.qvel[joint.dofadr];
      const bias = data.qfrc_bias[joint.dofadr];
      const tau = bias + joint.kp * (targets[joint.name] - q) - joint.kd * qd;
      data.ctrl[joint.actuatorId] = clip(tau, -joint.tauLimit, joint.tauLimit);
    }
    const gripperTau =
      GRIPPER_KP * (targets.joint7 - data.qpos[gripper.qposadr]) -
      GRIPPER_KD * data.qvel[gripper.dofadr];
    data.ctrl[gripper.actuatorId] = clip(
      gripperTau,
      -GRIPPER_TAU_LIMIT,
      GRIPPER_TAU_LIMIT
    );
  }

  function step(steps = STEPS_PER_FRAME) {
    for (let i = 0; i < steps; i += 1) {
      applyCtrl();
      beforeStep?.();
      mujoco.mj_step(model, data);
      afterStep?.();
    }
  }

  function reset() {
    mujoco.mj_resetData(model, data);
    Object.assign(targets, homePose(joints));
    mujoco.mj_forward(model, data);
  }

  function telemetry() {
    const samples = arm.map((joint) => ({
      name: joint.name,
      target: targets[joint.name],
      position: data.qpos[joint.qposadr],
      error: targets[joint.name] - data.qpos[joint.qposadr],
      torque: data.qfrc_actuator?.[joint.dofadr] ?? data.ctrl[joint.actuatorId],
      torqueLimit: joint.tauLimit
    }));
    samples.push({
      name: 'joint7',
      target: targets.joint7,
      position: data.qpos[gripper.qposadr],
      error: targets.joint7 - data.qpos[gripper.qposadr],
      torque: data.qfrc_actuator?.[gripper.dofadr] ?? data.ctrl[gripper.actuatorId],
      torqueLimit: GRIPPER_TAU_LIMIT
    });
    const rmsError = Math.sqrt(
      samples.reduce((sum, sample) => sum + sample.error * sample.error, 0) / samples.length
    );
    return { joints: samples, rmsError };
  }

  return { targets, setTarget, setTargets, step, reset, telemetry };
}
