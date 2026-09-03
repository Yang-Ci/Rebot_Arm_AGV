import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import loadMujoco from '@mujoco/mujoco';
import { ARM_JOINTS, bindJoints } from '../src/kinematics.js';
import { createPhysicsController } from '../src/pd-control.js';
import { createTcpIk } from '../src/tcp-ik.js';
import { createGraspDemo, STORAGE_ZONES, STACK_TARGETS } from '../src/grasp-demo.js';
import { OBJECT_MOVE_STEP, createObjectPositionController } from '../src/object-position-control.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const modelsDir = path.resolve(here, '../../rebotarm_mujoco_rs/models');
const graspPhysicsSteps = Number.parseInt(process.env.GRASP_PHYSICS_STEPS || '12', 10);

function namedId(mujoco, model, type, name) {
  const id = mujoco.mj_name2id(model, type, name);
  if (id < 0) throw new Error(`找不到 ${name}`);
  return id;
}

function bodyPos(mujoco, model, data, name) {
  const id = namedId(mujoco, model, mujoco.mjtObj.mjOBJ_BODY.value, name);
  return [data.xpos[id * 3], data.xpos[id * 3 + 1], data.xpos[id * 3 + 2]];
}

function bodyUpZ(mujoco, model, data, name) {
  const id = namedId(mujoco, model, mujoco.mjtObj.mjOBJ_BODY.value, name);
  return data.xmat[id * 9 + 8];
}

function meshExtent(model, meshId) {
  const start = model.mesh_vertadr[meshId] * 3;
  const count = model.mesh_vertnum[meshId];
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let index = 0; index < count; index += 1) {
    for (let axis = 0; axis < 3; axis += 1) {
      const value = model.mesh_vert[start + index * 3 + axis];
      min[axis] = Math.min(min[axis], value);
      max[axis] = Math.max(max[axis], value);
    }
  }
  return max.map((value, axis) => value - min[axis]);
}

function geomPosition(mujoco, model, data, name) {
  const id = namedId(mujoco, model, mujoco.mjtObj.mjOBJ_GEOM.value, name);
  return Array.from(data.geom_xpos.subarray(id * 3, id * 3 + 3));
}

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

async function loadScene(mujoco) {
  const vfs = new mujoco.MjVFS();
  const sceneXml = await readFile(path.join(modelsDir, 'rs_grasp_scene.xml'));
  const armXml = await readFile(path.join(modelsDir, 'rs_arm.xml'));
  vfs.addBuffer('rs_grasp_scene.xml', sceneXml);
  vfs.addBuffer('rs_arm.xml', armXml);

  const meshRe = /\sfile="([^"]+\.(?:stl|obj|msh))"/gi;
  const armText = armXml.toString('utf8');
  let match;
  while ((match = meshRe.exec(armText))) {
    const name = `meshes/${match[1]}`;
    vfs.addBuffer(name, await readFile(path.join(modelsDir, name)));
  }

  const model = mujoco.MjModel.from_xml_string(sceneXml.toString('utf8'), vfs);
  const data = new mujoco.MjData(model);
  mujoco.mj_forward(model, data);
  return { model, data, vfs };
}

async function main() {
  assert(
    Number.isInteger(graspPhysicsSteps) && graspPhysicsSteps >= 4 && graspPhysicsSteps <= 24,
    `GRASP_PHYSICS_STEPS 超出测试范围：${graspPhysicsSteps}`
  );
  const mujoco = await loadMujoco();
  const { model, data, vfs } = await loadScene(mujoco);
  const joints = bindJoints(mujoco, model);
  const physics = createPhysicsController(mujoco, model, data, joints);
  const overheadCameraId = namedId(
    mujoco,
    model,
    mujoco.mjtObj.mjOBJ_CAMERA.value,
    'overhead_rgb'
  );
  const overheadCameraPosition = Array.from(
    data.cam_xpos.subarray(overheadCameraId * 3, overheadCameraId * 3 + 3)
  );
  const overheadCameraFovy = model.cam_fovy[overheadCameraId];
  const wristCameraId = namedId(
    mujoco,
    model,
    mujoco.mjtObj.mjOBJ_CAMERA.value,
    'wrist_rgb'
  );
  const wristCameraPosition = Array.from(
    data.cam_xpos.subarray(wristCameraId * 3, wristCameraId * 3 + 3)
  );
  const mountGeomId = namedId(
    mujoco,
    model,
    mujoco.mjtObj.mjOBJ_GEOM.value,
    'd405_wrist_mount'
  );
  const mountExtent = meshExtent(model, model.geom_dataid[mountGeomId]);
  const gripperBodyId = namedId(
    mujoco,
    model,
    mujoco.mjtObj.mjOBJ_BODY.value,
    'gripper_end'
  );
  assert(model.ncam >= 2, `应包含俯视和腕部相机，ncam=${model.ncam}`);
  const overheadCoveragePoints = [
    { x: 0.13, y: -0.45, z: 0.10 },
    { x: 0.63, y: 0.45, z: 0.10 },
    {
      x: STACK_TARGETS.yellow.x,
      y: STACK_TARGETS.yellow.y + 0.026,
      z: STACK_TARGETS.yellow.z + 0.026
    }
  ];
  const overheadHalfAngle = overheadCameraFovy * Math.PI / 360;
  assert(
    overheadCoveragePoints.every((point) => {
      const distance = overheadCameraPosition[2] - point.z;
      const halfHeight = distance * Math.tan(overheadHalfAngle);
      const halfWidth = halfHeight * 4 / 3;
      return Math.abs(point.x - overheadCameraPosition[0]) <= halfWidth &&
        Math.abs(point.y - overheadCameraPosition[1]) <= halfHeight;
    }),
    `俯视相机无法同时覆盖工作台和叠叠乐塔顶：fovy=${overheadCameraFovy}`
  );
  assert(model.geom_bodyid[mountGeomId] === gripperBodyId, 'D405 支架没有安装在 gripper_end');
  assert(model.geom_group[mountGeomId] < 3, 'D405 支架被放入隐藏渲染组');
  assert(
    Math.max(...mountExtent) > 0.04 && Math.max(...mountExtent) < 0.20,
    `D405 支架尺寸异常：${mountExtent.join(',')}`
  );
  assert(
    wristCameraPosition.every(Number.isFinite),
    `腕部相机位姿无效：${wristCameraPosition.join(',')}`
  );
  assert(Number.isInteger(joints.byName.joint2.id), '关节 id 无效');
  assert(Number.isFinite(data.xanchor[joints.byName.joint2.id * 3]), 'xanchor 无法读取');

  const ik = createTcpIk(mujoco, model, data, joints);
  const tableGeomId = namedId(
    mujoco,
    model,
    mujoco.mjtObj.mjOBJ_GEOM.value,
    'task_table_geom'
  );
  assert(
    Math.abs(model.geom_size[tableGeomId * 3] - 0.25) < 0.001 &&
      Math.abs(model.geom_size[tableGeomId * 3 + 1] - 0.45) < 0.001,
    '桌面尺寸应为约 50 x 90 cm'
  );

  const activePoints = [
    ...Object.values(STORAGE_ZONES),
    ...Object.values(STACK_TARGETS)
  ];
  assert(
    activePoints.every((point) => Math.hypot(point.x, point.y) <= 0.535),
    `抓取/叠放目标超出可达包络：${JSON.stringify(activePoints)}`
  );

  const tcp0 = ik.tcpPosition();
  const tcpMatrix = Array.from(data.xmat.subarray(ik.bodyId * 9, ik.bodyId * 9 + 9));
  const ikAngles = Object.fromEntries(
    ARM_JOINTS.map((joint) => [joint.name, data.qpos[joints.byName[joint.name].qposadr]])
  );
  const ikResult = ik.servoStep(
    { x: tcp0.x + 0.03, y: tcp0.y, z: tcp0.z + 0.02 },
    0.016,
    ikAngles
  );
  assert(ikResult && Number.isFinite(ikResult.error), 'TCP IK servo 失败');

  data.ctrl[0] = 1.25;
  assert(Math.abs(data.ctrl[0] - 1.25) < 1e-9, 'data.ctrl 无法写入');

  const cubeBefore = bodyPos(mujoco, model, data, 'red_cube');
  const initialPositions = {
    red: cubeBefore,
    blue: bodyPos(mujoco, model, data, 'blue_block'),
    yellow: bodyPos(mujoco, model, data, 'yellow_cylinder')
  };
  assert(
    Math.abs(initialPositions.blue[0] - 0.34) < 0.002 &&
      Math.abs(initialPositions.blue[1] - 0.06) < 0.002,
    `蓝色块应位于便于俯抓的中前工作区：${initialPositions.blue.join(',')}`
  );
  const objectPositionController = createObjectPositionController({ mujoco, model, data });
  assert(
    objectPositionController.move(OBJECT_MOVE_STEP, 0).reason === 'disabled',
    '物体移动模式关闭时不应修改物体位置'
  );
  objectPositionController.setEnabled(true);
  const objectMoveResults = {};
  for (const [id, dx, dy] of [
    ['red', OBJECT_MOVE_STEP, 0],
    ['blue', 0, OBJECT_MOVE_STEP],
    ['yellow', -OBJECT_MOVE_STEP, 0]
  ]) {
    objectPositionController.setSelected(id);
    const before = objectPositionController.state().position;
    const result = objectPositionController.move(dx, dy);
    assert(result.ok, `${id} 方向键移动失败：${JSON.stringify(result)}`);
    assert(
      Math.abs(result.position.x - before.x - dx) < 0.001 &&
        Math.abs(result.position.y - before.y - dy) < 0.001,
      `${id} 方向键步长错误：${JSON.stringify(result.position)}`
    );
    objectMoveResults[id] = result.position;
  }
  objectPositionController.setSelected('red');
  const tiltAngle = Math.PI / 6;
  const tiltedQuaternion = {
    w: Math.cos(tiltAngle / 2),
    x: Math.sin(tiltAngle / 2),
    y: 0,
    z: 0
  };
  const rotationResult = objectPositionController.setQuaternion(tiltedQuaternion);
  assert(rotationResult.ok, `红色块三维旋转失败：${JSON.stringify(rotationResult)}`);
  const rotatedState = objectPositionController.state();
  assert(
    Math.abs(rotatedState.quaternion.x - tiltedQuaternion.x) < 1e-6 &&
      rotatedState.position.z > initialPositions.red[2] + 0.004,
    `红色块三维姿态或贴桌高度错误：${JSON.stringify(rotatedState)}`
  );
  const dragResult = objectPositionController.moveTo(0.28, -0.27);
  assert(dragResult.ok, `红色块拖动位姿失败：${JSON.stringify(dragResult)}`);
  assert(
    Math.abs(objectPositionController.state().quaternion.x - tiltedQuaternion.x) < 1e-6,
    '拖动物体时不应丢失三维旋转姿态'
  );
  const redBodyId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, 'red_cube');
  const redJointAddress = model.body_jntadr[redBodyId];
  const redQposAddress = model.jnt_qposadr[redJointAddress];
  data.qpos[redQposAddress] += 0.08;
  data.qpos[redQposAddress + 4] = 0;
  assert(objectPositionController.enforcePose(), '编辑状态应保持当前三维位姿');
  mujoco.mj_forward(model, data);
  assert(
    Math.abs(objectPositionController.state().position.x - dragResult.position.x) < 1e-6 &&
      Math.abs(objectPositionController.state().quaternion.x - tiltedQuaternion.x) < 1e-6,
    '物理步进后未恢复编辑中的三维位姿'
  );
  const redPosition = objectPositionController.state().position;
  const yellowPosition = objectMoveResults.yellow;
  const overlapResult = objectPositionController.move(
    yellowPosition.x - redPosition.x,
    yellowPosition.y - redPosition.y
  );
  assert(overlapResult.reason === 'overlap', '物体移动应阻止两个物体重叠');
  objectPositionController.setSelected('blue');
  const boundaryResult = objectPositionController.move(10, 0);
  assert(boundaryResult.ok && boundaryResult.clamped, '物体移动应限制在桌面安全边界内');
  assert(boundaryResult.position.x < 0.61, `蓝色块越过桌面边界：${boundaryResult.position.x}`);
  objectPositionController.reset();
  physics.reset();
  Object.entries(STORAGE_ZONES).forEach(([id, zone]) => {
    const initial = initialPositions[id];
    assert(
      Math.hypot(initial[0] - zone.x, initial[1] - zone.y) > 0.15,
      `${id} 初始位置距离收纳区过近：${initial.join(',')} vs ${zone.x},${zone.y}`
    );
  });
  physics.step(200);
  const cubeSettled = bodyPos(mujoco, model, data, 'red_cube');
  assert(data.ncon > 0, `色块落地后应有接触，ncon=${data.ncon}`);
  console.log('memory exports', Object.keys(mujoco).filter((key) => /malloc|free|heap|memory/i.test(key)));
  const contactForceBuffer = new mujoco.DoubleBuffer(6);
  const contactForce = contactForceBuffer.GetView();
  let maxContactForce = 0;
  for (let index = 0; index < data.ncon; index += 1) {
    contactForce.fill(0);
    mujoco.mj_contactForce(model, data, index, contactForceBuffer);
    assert(contactForce.every(Number.isFinite), `接触力读取失败：${Array.from(contactForce).join(',')}`);
    maxContactForce = Math.max(maxContactForce, Math.abs(contactForce[0]));
  }
  contactForceBuffer.delete();
  assert(maxContactForce > 0.01, `接触力应为正值：${maxContactForce}`);
  assert(
    cubeSettled[2] > 0.09 && cubeSettled[2] < 0.16,
    `红块应停在桌面上，z=${cubeSettled[2]}`
  );

  const qposadr = joints.byName.joint2.qposadr;
  physics.setTarget('joint2', 0.3);
  physics.step(1);
  const afterOne = data.qpos[qposadr];
  assert(Math.abs(afterOne - 0.3) > 0.15, `joint2 不应瞬移，一步后 q=${afterOne}`);

  physics.step(500);
  const afterHold = data.qpos[qposadr];
  assert(Math.abs(afterHold - 0.3) < 0.05, `joint2 应变到目标附近，q=${afterHold}`);
  const wristCameraAfterJointMotion = Array.from(
    data.cam_xpos.subarray(wristCameraId * 3, wristCameraId * 3 + 3)
  );
  const wristCameraTravel = Math.hypot(
    ...wristCameraAfterJointMotion.map((value, index) => value - wristCameraPosition[index])
  );
  assert(wristCameraTravel > 0.01, `腕部相机没有随机械臂运动：${wristCameraTravel}`);

  physics.reset();
  physics.setTarget('joint7', 0.03);
  physics.step(400);
  const gripper = data.qpos[joints.byName.joint7.qposadr];
  assert(Math.abs(gripper - 0.03) < 0.008, `夹爪应变到 30 mm 附近，q=${gripper}`);

  physics.reset();
  const cubeReset = bodyPos(mujoco, model, data, 'red_cube');
  assert(Math.abs(data.qpos[qposadr]) < 1e-6, `复位后 joint2 应为 0，q=${data.qpos[qposadr]}`);
  assert(Math.abs(data.qpos[joints.byName.joint7.qposadr]) < 1e-6, '复位后夹爪应为 0');
  assert(
    Math.abs(cubeReset[0] - cubeBefore[0]) < 0.01 &&
      Math.abs(cubeReset[1] - cubeBefore[1]) < 0.01 &&
      Math.abs(cubeReset[2] - cubeBefore[2]) < 0.01,
    `复位后红块应回到初始位，got=${cubeReset.join(',')}`
  );

  const graspResults = {};
  for (const [target, body] of [
    ['red', 'red_cube'],
    ['blue', 'blue_block'],
    ['yellow', 'yellow_cylinder']
  ]) {
    physics.reset();
    let graspState = null;
    const pregraspStages = new Set();
    let alignmentStart = null;
    let maximumNonJ1AlignmentTravel = 0;
    let joint1AtApproach = null;
    const graspDemo = createGraspDemo({
      mujoco, model, data, joints, physics, ik,
      onChange: (state) => {
        graspState = state;
        pregraspStages.add(state.stage);
        if (state.stage === 'aligning') {
          const pose = Object.fromEntries(
            ARM_JOINTS.map((joint) => [joint.name, data.qpos[joints.byName[joint.name].qposadr]])
          );
          alignmentStart ||= pose;
          maximumNonJ1AlignmentTravel = Math.max(
            maximumNonJ1AlignmentTravel,
            ...ARM_JOINTS
              .filter((joint) => joint.name !== 'joint1')
              .map((joint) => Math.abs(pose[joint.name] - alignmentStart[joint.name]))
          );
        } else if (state.stage === 'approach' && joint1AtApproach === null) {
          joint1AtApproach = data.qpos[joints.byName.joint1.qposadr];
        }
      }
    });
    const zone = STORAGE_ZONES[target];
    graspDemo.start(target);
    for (let frame = 0; frame < 3200 && graspDemo.isRunning(); frame += 1) {
      graspDemo.update();
      physics.step(graspPhysicsSteps);
    }
    graspState = graspDemo.state();
    const finalPosition = bodyPos(mujoco, model, data, body);
    assert(
      graspState.stage === 'complete',
      `${target} 一键抓取未完成：${graspState.stage}/${graspState.message}`
    );
    assert(
      pregraspStages.has('preparing') && pregraspStages.has('aligning'),
      `${target} 抓取前没有执行安全收拢和 J1 对准：${Array.from(pregraspStages).join(',')}`
    );
    const expectedJoint1 = -Math.atan2(initialPositions[target][1], initialPositions[target][0]);
    assert(
      Number.isFinite(joint1AtApproach) && Math.abs(joint1AtApproach - expectedJoint1) < 0.08,
      `${target} 进入接近阶段前 J1 未对准目标：${joint1AtApproach}/${expectedJoint1}`
    );
    assert(
      maximumNonJ1AlignmentTravel < 0.08,
      `${target} 在 J1 对准阶段仍有其他关节大幅运动：${maximumNonJ1AlignmentTravel}`
    );
    assert(
      Math.abs(finalPosition[0] - zone.x) < 0.025 &&
        Math.abs(finalPosition[1] - zone.y) < 0.025 &&
        Math.abs(finalPosition[2] - zone.z) < 0.02,
      `${target} 没有放回对应收纳区：${finalPosition.join(',')}, expected=${zone.x},${zone.y},${zone.z}`
    );
    assert(
      finalPosition[0] > 0.13 && finalPosition[0] < 0.63 &&
        Math.abs(finalPosition[1]) < 0.45,
      `${target} 放置后超出桌面：${finalPosition.join(',')}`
    );
    graspResults[target] = {
      stage: graspState.stage,
      position: finalPosition,
      carryOffset: graspState.carryOffset
    };
  }

  physics.reset();
  const browserObserver = { x: 0.85, y: -0.95, z: 0.62 };
  const stackOffsets = {};
  const stackStages = new Set();
  const happyJoint6Offsets = [];
  const happyJoint5Offsets = [];
  const happyGripWidths = [];
  let minimumHappyTcpZ = Infinity;
  const stackDemo = createGraspDemo({
    mujoco, model, data, joints, physics, ik,
    getObserverPosition: () => browserObserver,
    onChange: (state) => {
      stackStages.add(state.stage);
      if (state.carryOffset) stackOffsets[state.selectedId] = state.carryOffset;
    }
  });
  stackDemo.startStack();
  for (let frame = 0; frame < 12000 && stackDemo.isRunning(); frame += 1) {
    stackDemo.update();
    const reactionState = stackDemo.state();
    if (reactionState.stage === 'celebrate' && reactionState.reactionCenterPose) {
      minimumHappyTcpZ = Math.min(minimumHappyTcpZ, ik.tcpPosition().z);
      happyJoint6Offsets.push(
        data.qpos[joints.byName.joint6.qposadr] - reactionState.reactionCenterPose.joint6
      );
      happyJoint5Offsets.push(
        data.qpos[joints.byName.joint5.qposadr] - reactionState.reactionCenterPose.joint5
      );
      happyGripWidths.push(data.qpos[joints.byName.joint7.qposadr]);
    }
    physics.step(graspPhysicsSteps);
  }
  const stackState = stackDemo.state();
  const stackAtCompletion = {
    blue: bodyPos(mujoco, model, data, 'blue_block'),
    red: bodyPos(mujoco, model, data, 'red_cube'),
    yellow: bodyPos(mujoco, model, data, 'yellow_cylinder')
  };
  physics.step(1000);
  const stackedBlue = bodyPos(mujoco, model, data, 'blue_block');
  const stackedRed = bodyPos(mujoco, model, data, 'red_cube');
  const stackedYellow = bodyPos(mujoco, model, data, 'yellow_cylinder');
  assert(
    stackState.stage === 'complete',
    `叠叠乐未完成：${stackState.selectedId}/${stackState.stage}/${stackState.message}; ` +
      `objects=${JSON.stringify(stackAtCompletion)}; tcp=${JSON.stringify(ik.tcpPosition())}; ` +
      `carryOffsets=${JSON.stringify(stackOffsets)}; alignmentJoint1=${stackState.alignmentJoint1}`
  );
  assert(
    stackStages.has('centering') &&
      stackStages.has('celebrate-centering') &&
      stackStages.has('celebrate') &&
      !stackStages.has('celebrate-homing') &&
      !stackStages.has('homing'),
    `叠叠乐成功后没有执行庆祝动作：${Array.from(stackStages).join(',')}`
  );
  assert(stackState.reactionCenterPose, '叠叠乐庆祝没有保存面向镜头的中位姿态');
  const expectedCameraFacingPose = {
    joint1: -Math.atan2(browserObserver.y, browserObserver.x),
    joint2: 0.5515,
    joint3: 0.2688,
    joint4: 0.80,
    joint5: 0.0471,
    joint6: 0
  };
  Object.entries(expectedCameraFacingPose).forEach(([name, expected]) => {
    assert(
      Math.abs(stackState.reactionCenterPose[name] - expected) < 1e-6,
      `镜头中位的 ${name} 不正确：${stackState.reactionCenterPose[name]}/${expected}`
    );
  });
  assert(
    Math.min(...happyJoint6Offsets) < -0.20 && Math.max(...happyJoint6Offsets) > 0.20,
    `庆祝动作没有用 J6 完成左右扭头：${Math.min(...happyJoint6Offsets)},${Math.max(...happyJoint6Offsets)}`
  );
  assert(
    Math.max(...happyJoint5Offsets) > 0.14,
    `庆祝动作没有完成歪头：${Math.max(...happyJoint5Offsets)}`
  );
  assert(
    Math.min(...happyGripWidths) < 0.008 && Math.max(...happyGripWidths) > 0.040,
    `庆祝动作没有完成夹爪闭合再张开：${Math.min(...happyGripWidths)},${Math.max(...happyGripWidths)}`
  );
  assert(
    data.qpos[joints.byName.joint7.qposadr] > 0.040,
    `庆祝结束后夹爪没有保持张开：${data.qpos[joints.byName.joint7.qposadr]}`
  );
  assert(minimumHappyTcpZ > 0.20, `庆祝动作末端过低，TCP z=${minimumHappyTcpZ}`);
  const happyCenterError = Math.max(...ARM_JOINTS.map((joint) =>
    Math.abs(
      data.qpos[joints.byName[joint.name].qposadr] - stackState.reactionCenterPose[joint.name]
    )
  ));
  assert(happyCenterError < 0.08, `庆祝结束后没有停在镜头中位：${happyCenterError}`);
  const wristCameraMatrix = Array.from(
    data.cam_xmat.subarray(wristCameraId * 9, wristCameraId * 9 + 9)
  );
  const wristCameraUpZ = wristCameraMatrix[7];
  const wristCameraForward = [-wristCameraMatrix[2], -wristCameraMatrix[5]];
  const observerDirection = [
    browserObserver.x - data.cam_xpos[wristCameraId * 3],
    browserObserver.y - data.cam_xpos[wristCameraId * 3 + 1]
  ];
  const facingDot =
    (wristCameraForward[0] * observerDirection[0] +
      wristCameraForward[1] * observerDirection[1]) /
    (Math.hypot(...wristCameraForward) * Math.hypot(...observerDirection));
  assert(wristCameraUpZ > 0.99, `情绪中位的腕部相机没有保持竖直：${wristCameraUpZ}`);
  assert(facingDot > 0.98, `情绪中位没有面向浏览器用户视角：${facingDot}`);
  [
    ['blue', stackedBlue],
    ['red', stackedRed],
    ['yellow', stackedYellow]
  ].forEach(([id, position]) => {
    const target = STACK_TARGETS[id];
    assert(
      Math.abs(position[0] - target.x) < 0.035 &&
        Math.abs(position[1] - target.y) < 0.035,
      `叠叠乐 ${id} 没有对准堆叠中心：${position.join(',')}, expected=${target.x},${target.y}`
    );
  });
  assert(
    stackedBlue[2] < stackedRed[2] && stackedRed[2] < stackedYellow[2],
    `叠叠乐顺序错误：blue=${stackedBlue[2]}, red=${stackedRed[2]}, yellow=${stackedYellow[2]}`
  );
  const adjacentOffsets = [
    Math.hypot(stackedBlue[0] - stackedRed[0], stackedBlue[1] - stackedRed[1]),
    Math.hypot(stackedRed[0] - stackedYellow[0], stackedRed[1] - stackedYellow[1])
  ];
  assert(
    adjacentOffsets.every((offset) => offset < 0.008),
    `叠放层中心偏差过大：${adjacentOffsets.join(',')}; ` +
      `blue=${stackedBlue.join(',')}; red=${stackedRed.join(',')}; yellow=${stackedYellow.join(',')}`
  );
  const settleDrift = Math.max(
    ...Object.entries(stackAtCompletion).map(([id, position]) => {
      const settled = { blue: stackedBlue, red: stackedRed, yellow: stackedYellow }[id];
      return Math.hypot(...settled.map((value, axis) => value - position[axis]));
    })
  );
  assert(settleDrift < 0.003, `叠叠乐完成后仍在明显移动：${settleDrift}`);
  const upright = [
    bodyUpZ(mujoco, model, data, 'blue_block'),
    bodyUpZ(mujoco, model, data, 'red_cube'),
    bodyUpZ(mujoco, model, data, 'yellow_cylinder')
  ];
  assert(upright.every((upZ) => upZ > 0.995), `叠叠乐存在明显倾斜：${upright.join(',')}`);

  // Reproduce the UI workflow that used to knock the finished tower over:
  // put all objects into their zones first, then run the stack demo without a reset.
  physics.reset();
  const storedStackStages = new Set();
  const storedThenStackedDemo = createGraspDemo({
    mujoco, model, data, joints, physics, ik,
    onChange: (state) => storedStackStages.add(state.stage)
  });
  for (const target of ['red', 'blue', 'yellow']) {
    storedThenStackedDemo.start(target);
    for (let frame = 0; frame < 4000 && storedThenStackedDemo.isRunning(); frame += 1) {
      storedThenStackedDemo.update();
      physics.step(graspPhysicsSteps);
    }
    const storedState = storedThenStackedDemo.state();
    assert(
      storedState.stage === 'complete',
      `收纳后叠放场景的 ${target} 收纳失败：${storedState.stage}/${storedState.message}`
    );
  }
  storedThenStackedDemo.startStack();
  for (let frame = 0; frame < 14000 && storedThenStackedDemo.isRunning(); frame += 1) {
    storedThenStackedDemo.update();
    physics.step(graspPhysicsSteps);
  }
  const storedThenStackedState = storedThenStackedDemo.state();
  assert(
    storedThenStackedState.stage === 'complete',
    `先收纳再叠放未完成：${storedThenStackedState.stage}/${storedThenStackedState.message}`
  );
  assert(
    storedStackStages.has('centering') &&
      storedStackStages.has('celebrate-centering') &&
      storedStackStages.has('celebrate') &&
      !storedStackStages.has('celebrate-homing') &&
      !storedStackStages.has('homing'),
    `先收纳再叠放后没有执行庆祝动作：${Array.from(storedStackStages).join(',')}`
  );
  const storedStackAtCompletion = {
    blue: bodyPos(mujoco, model, data, 'blue_block'),
    red: bodyPos(mujoco, model, data, 'red_cube'),
    yellow: bodyPos(mujoco, model, data, 'yellow_cylinder')
  };
  physics.step(1000);
  const storedStackSettled = {
    blue: bodyPos(mujoco, model, data, 'blue_block'),
    red: bodyPos(mujoco, model, data, 'red_cube'),
    yellow: bodyPos(mujoco, model, data, 'yellow_cylinder')
  };
  const storedStackOffsets = [
    Math.hypot(
      storedStackSettled.blue[0] - storedStackSettled.red[0],
      storedStackSettled.blue[1] - storedStackSettled.red[1]
    ),
    Math.hypot(
      storedStackSettled.red[0] - storedStackSettled.yellow[0],
      storedStackSettled.red[1] - storedStackSettled.yellow[1]
    )
  ];
  const storedStackDrift = Math.max(
    ...Object.keys(storedStackAtCompletion).map((id) => Math.hypot(
      ...storedStackSettled[id].map((value, axis) => value - storedStackAtCompletion[id][axis])
    ))
  );
  const storedStackUpright = [
    bodyUpZ(mujoco, model, data, 'blue_block'),
    bodyUpZ(mujoco, model, data, 'red_cube'),
    bodyUpZ(mujoco, model, data, 'yellow_cylinder')
  ];
  assert(
    storedStackOffsets.every((offset) => offset < 0.010),
    `先收纳再叠放的层中心偏差过大：${storedStackOffsets.join(',')}`
  );
  assert(storedStackDrift < 0.003, `机械臂撤离后叠塔仍在移动：${storedStackDrift}`);
  assert(
    storedStackUpright.every((upZ) => upZ > 0.995),
    `机械臂撤离后叠塔被碰斜：${storedStackUpright.join(',')}`
  );

  physics.reset();
  const failureStages = new Set();
  let minimumShyTcpZ = Infinity;
  let sabotaged = false;
  const failureDemo = createGraspDemo({
    mujoco, model, data, joints, physics, ik,
    onChange: (state) => failureStages.add(state.stage)
  });
  objectPositionController.setEnabled(true);
  objectPositionController.setSelected('blue');
  failureDemo.startStack();
  for (let frame = 0; frame < 6000 && failureDemo.isRunning(); frame += 1) {
    const state = failureDemo.state();
    if (!sabotaged && state.selectedId === 'blue' && state.stage === 'closing') {
      const moved = objectPositionController.move(0.20, -0.10);
      assert(moved.ok, `失败动作测试无法移开蓝色块：${JSON.stringify(moved)}`);
      sabotaged = true;
    }
    failureDemo.update();
    if (failureDemo.state().stage === 'shy') {
      minimumShyTcpZ = Math.min(minimumShyTcpZ, ik.tcpPosition().z);
    }
    physics.step(graspPhysicsSteps);
  }
  objectPositionController.setEnabled(false);
  const failureState = failureDemo.state();
  assert(sabotaged, '失败动作测试没有触发抓取破坏');
  assert(
    failureState.stage === 'failed' && failureState.reaction === 'shy',
    `叠叠乐失败后没有进入害羞终态：${failureState.stage}/${failureState.reaction}`
  );
  assert(
    failureStages.has('shy-centering') && failureStages.has('shy'),
    `叠叠乐失败后没有执行低头动作：${Array.from(failureStages).join(',')}`
  );
  assert(minimumShyTcpZ > 0.13, `低头动作过低，TCP z=${minimumShyTcpZ}`);
  const failureArmError = Math.max(...ARM_JOINTS.map((joint) =>
    Math.abs(
      data.qpos[joints.byName[joint.name].qposadr] - failureState.reactionCenterPose[joint.name]
    )
  ));
  assert(failureArmError < 0.08, `害羞动作结束后没有回到镜头中位：${failureArmError}`);

  console.log(JSON.stringify({
    ngeom: model.ngeom,
    ncam: model.ncam,
    graspPhysicsSteps,
    wristCameraPosition,
    overheadCameraFovy,
    wristCameraTravel,
    d405MountExtent: mountExtent,
    d405MountPosition: geomPosition(mujoco, model, data, 'd405_wrist_mount'),
    d405BodyPosition: geomPosition(mujoco, model, data, 'd405_camera_body'),
    d405FrontPosition: geomPosition(mujoco, model, data, 'd405_camera_front'),
    tcpHome: tcp0,
    tcpMatrix,
    ncon: data.ncon,
    maxContactForce,
    timestep: model.opt.timestep,
    joint2: { afterOne, afterHold },
    gripper,
    cube: { settledZ: cubeSettled[2], reset: cubeReset },
    objectPositionControl: {
      moved: objectMoveResults,
      overlapProtected: overlapResult.reason === 'overlap',
      boundaryX: boundaryResult.position.x
    },
    graspDemo: graspResults,
    stackDemo: {
      stage: stackState.stage,
      blue: stackedBlue,
      red: stackedRed,
      yellow: stackedYellow,
      carryOffsets: stackOffsets,
      reactionStages: Array.from(stackStages).filter((stage) => stage.includes('celebrate')),
      joint6OffsetRange: [Math.min(...happyJoint6Offsets), Math.max(...happyJoint6Offsets)],
      maximumHeadTilt: Math.max(...happyJoint5Offsets),
      gripWidthRange: [Math.min(...happyGripWidths), Math.max(...happyGripWidths)],
      minimumTcpZ: minimumHappyTcpZ,
      finalCenterError: happyCenterError,
      wristCameraUpZ,
      observerFacingDot: facingDot,
      adjacentOffsets,
      settleDrift,
      upright
    },
    storedThenStackedDemo: {
      stage: storedThenStackedState.stage,
      blue: storedStackSettled.blue,
      red: storedStackSettled.red,
      yellow: storedStackSettled.yellow,
      adjacentOffsets: storedStackOffsets,
      settleDrift: storedStackDrift,
      upright: storedStackUpright
    },
    failedStackReaction: {
      stage: failureState.stage,
      reaction: failureState.reaction,
      message: failureState.message,
      stages: Array.from(failureStages).filter((stage) => stage.includes('shy')),
      minimumTcpZ: minimumShyTcpZ,
      finalArmError: failureArmError
    }
  }, null, 2));

  data.delete();
  model.delete();
  vfs.delete();
}

main().catch((error) => {
  console.error(error);
  process.exit(1);
});
