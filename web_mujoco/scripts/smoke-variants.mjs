import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import loadMujoco from '@mujoco/mujoco';
import { createAgvController } from '../src/agv-control.js';
import { planAgvPath } from '../src/agv-path-planner.js';
import {
  GRASP_DOCK,
  GRASP_WORKSPACE_OFFSET,
  createAgvNavigation
} from '../src/agv-navigation.js';
import { createGraspDemo, STORAGE_ZONES } from '../src/grasp-demo.js';
import { bindJoints } from '../src/kinematics.js';
import { MODEL_VARIANTS } from '../src/model-variants.js';
import { createPhysicsController } from '../src/pd-control.js';
import { createTcpIk } from '../src/tcp-ik.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const modelsDir = path.resolve(here, '../../rebotarm_mujoco_rs/models');

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

async function addXmlTree(vfs, relative, seen = new Set()) {
  if (seen.has(relative)) return seen;
  seen.add(relative);
  const bytes = await readFile(path.join(modelsDir, relative));
  vfs.addBuffer(relative, bytes);
  const text = bytes.toString('utf8');
  const includeRe = /<include\s+file="([^"]+)"/gi;
  const meshRe = /\sfile="([^"]+\.(?:stl|obj|msh))"/gi;
  let match;
  while ((match = includeRe.exec(text))) await addXmlTree(vfs, match[1], seen);
  while ((match = meshRe.exec(text))) {
    const file = match[1].replaceAll('\\', '/').replace(/^\.\//, '');
    const meshPath = file.startsWith('meshes/') ? file : `meshes/${file}`;
    if (seen.has(meshPath)) continue;
    seen.add(meshPath);
    vfs.addBuffer(meshPath, await readFile(path.join(modelsDir, meshPath)));
  }
  return seen;
}

function hasName(mujoco, model, type, name) {
  return mujoco.mj_name2id(model, type, name) >= 0;
}

function contactNames(mujoco, model, data) {
  const geomType = mujoco.mjtObj.mjOBJ_GEOM.value;
  const names = [];
  const count = Math.min(data.ncon, data.contact.size());
  for (let index = 0; index < count; index += 1) {
    const contact = data.contact.get(index);
    if (!contact) continue;
    const geom1 = mujoco.mj_id2name(model, geomType, contact.geom1) || `geom#${contact.geom1}`;
    const geom2 = mujoco.mj_id2name(model, geomType, contact.geom2) || `geom#${contact.geom2}`;
    names.push(`${geom1}<->${geom2}`);
  }
  return [...new Set(names)];
}

function armYellowContactNames(mujoco, model, data, armBodyIds) {
  const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
  const yellowBodyId = mujoco.mj_name2id(model, bodyType, 'yellow_cylinder');
  const geomType = mujoco.mjtObj.mjOBJ_GEOM.value;
  const names = [];
  const count = Math.min(data.ncon, data.contact.size());
  for (let index = 0; index < count; index += 1) {
    const contact = data.contact.get(index);
    if (!contact) continue;
    const body1 = model.geom_bodyid[contact.geom1];
    const body2 = model.geom_bodyid[contact.geom2];
    const yellowInvolved = body1 === yellowBodyId || body2 === yellowBodyId;
    const otherBody = body1 === yellowBodyId ? body2 : body1;
    if (!yellowInvolved || !armBodyIds.has(otherBody)) continue;
    const geom1 = mujoco.mj_id2name(model, geomType, contact.geom1) || `geom#${contact.geom1}`;
    const geom2 = mujoco.mj_id2name(model, geomType, contact.geom2) || `geom#${contact.geom2}`;
    names.push(`${geom1}<->${geom2}`);
  }
  return [...new Set(names)];
}

async function compileVariant(mujoco, variant) {
  const vfs = new mujoco.MjVFS();
  const files = await addXmlTree(vfs, variant.sceneXml);
  const model = mujoco.MjModel.from_xml_path(variant.sceneXml, vfs);
  const data = new mujoco.MjData(model);
  mujoco.mj_forward(model, data);
  return { model, data, vfs, files };
}

async function main() {
  const mujoco = await loadMujoco();
  const jointType = mujoco.mjtObj.mjOBJ_JOINT.value;
  const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;

  const arm = await compileVariant(mujoco, MODEL_VARIANTS.arm);
  assert(hasName(mujoco, arm.model, jointType, 'joint1'), '标准款缺少 joint1');
  assert(!hasName(mujoco, arm.model, jointType, 'agv_freejoint'), '标准款不应包含 AGV 自由关节');
  const armGeomCount = arm.model.ngeom;
  arm.data.delete();
  arm.model.delete();
  arm.vfs.delete();

  const agv = await compileVariant(mujoco, MODEL_VARIANTS.agv);
  assert(hasName(mujoco, agv.model, jointType, 'joint1'), 'AGV 款缺少机械臂');
  assert(hasName(mujoco, agv.model, jointType, 'agv_freejoint'), 'AGV 款缺少底盘自由关节');
  assert(hasName(mujoco, agv.model, bodyType, 'agv_chassis'), 'AGV 款缺少底盘 body');
  assert(
    ['front_left', 'front_right', 'rear_left', 'rear_right'].every((wheel) =>
      hasName(mujoco, agv.model, jointType, `wheel_${wheel}_joint`)
    ),
    'AGV 款缺少车轮关节'
  );
  assert([...agv.files].some((file) => file.startsWith('meshes/agv/')), 'AGV 子目录网格未装载');

  const controller = createAgvController(mujoco, agv.model, agv.data);
  controller.setCommand(0.3, 0.4);
  controller.apply();
  const freeJointId = mujoco.mj_name2id(agv.model, jointType, 'agv_freejoint');
  const dof = agv.model.jnt_dofadr[freeJointId];
  assert(Math.hypot(agv.data.qvel[dof], agv.data.qvel[dof + 1]) > 0.29, 'AGV 线速度指令未写入');
  assert(Math.abs(agv.data.qvel[dof + 5] - 0.4) < 1e-9, 'AGV 角速度指令未写入');

  const joints = bindJoints(mujoco, agv.model);
  const physics = createPhysicsController(mujoco, agv.model, agv.data, joints, {
    beforeStep: () => controller.apply(),
    afterStep: () => controller.enforceHeldPose()
  });
  physics.reset();
  const navigation = createAgvNavigation({
    drive: controller,
    maxLinear: () => 0.65,
    maxAngular: () => 1.4,
    planPath: (start, goal) => planAgvPath(start, goal)
  });
  navigation.start(GRASP_DOCK, 'grasp');
  assert(
    navigation.state().path.length > 1,
    'AGV 开始导航时丢失了规划路径'
  );
  for (let step = 0; step < 30000 && navigation.state().active; step += 1) {
    navigation.update();
    physics.step(1);
  }
  assert(
    navigation.state().result === 'reached',
    `AGV 未能自动到达抓取区：${JSON.stringify(navigation.state())}`
  );
  controller.holdPlanarPose(GRASP_DOCK);
  const ik = createTcpIk(mujoco, agv.model, agv.data, joints);
  const grasp = createGraspDemo({
    mujoco,
    model: agv.model,
    data: agv.data,
    joints,
    physics,
    ik,
    workspaceOffset: GRASP_WORKSPACE_OFFSET
  });
  grasp.start('red');
  for (let frame = 0; frame < 3600 && grasp.isRunning(); frame += 1) {
    grasp.update();
    physics.step(12);
  }
  const graspState = grasp.state();
  assert(
    graspState.stage === 'complete',
    `AGV 停靠后的抓取未完成：${graspState.stage}/${graspState.message}; ` +
      `base=${JSON.stringify(controller.pose())}; tcp=${JSON.stringify(ik.tcpPosition())}; ` +
      `object=${JSON.stringify(graspState.objectPosition)}; start=${JSON.stringify(graspState.objectStart)}; ` +
      `joints=${JSON.stringify(Object.fromEntries(['joint1','joint2','joint3','joint4','joint5','joint6'].map((name) => [name, agv.data.qpos[joints.byName[name].qposadr]])))}; ` +
      `targets=${JSON.stringify(physics.targets)}; telemetry=${JSON.stringify(physics.telemetry())}; ` +
      `contacts=${JSON.stringify(contactNames(mujoco, agv.model, agv.data))}`
  );
  const redId = mujoco.mj_name2id(agv.model, bodyType, 'red_cube');
  const red = Array.from(agv.data.xpos.subarray(redId * 3, redId * 3 + 3));
  const redZone = {
    x: STORAGE_ZONES.red.x + GRASP_WORKSPACE_OFFSET.x,
    y: STORAGE_ZONES.red.y + GRASP_WORKSPACE_OFFSET.y,
    z: STORAGE_ZONES.red.z + GRASP_WORKSPACE_OFFSET.z
  };
  assert(
    Math.hypot(red[0] - redZone.x, red[1] - redZone.y) < 0.03 &&
      Math.abs(red[2] - redZone.z) < 0.025,
    `AGV 抓取没有放入平移后的收纳区：${red.join(',')}`
  );

  physics.reset();
  controller.setPlanarPose(GRASP_DOCK.x, GRASP_DOCK.y, GRASP_DOCK.yaw);
  controller.holdPlanarPose(GRASP_DOCK);
  const stack = createGraspDemo({
    mujoco,
    model: agv.model,
    data: agv.data,
    joints,
    physics,
    ik,
    workspaceOffset: GRASP_WORKSPACE_OFFSET,
    getObserverPosition: () => ({ x: 5.8, y: -6.8, z: 4.8 })
  });
  const armBodyIds = new Set([
    'base_link',
    'link1',
    'link2',
    'link3',
    'link4',
    'link5',
    'link6',
    'gripper_end',
    'gripper_coupler',
    'gripper_left',
    'gripper_right'
  ].map((name) => mujoco.mj_name2id(agv.model, bodyType, name)));
  const armYellowContacts = [];
  stack.startStack();
  for (let frame = 0; frame < 12000 && stack.isRunning(); frame += 1) {
    stack.update();
    physics.step(12);
    const contacts = armYellowContactNames(mujoco, agv.model, agv.data, armBodyIds);
    if (contacts.length && stack.state().selectedId === 'red') {
      armYellowContacts.push({
        stage: stack.state().stage,
        selectedId: stack.state().selectedId,
        contacts
      });
    }
  }
  assert(
    stack.state().stage === 'complete',
    `AGV 停靠后的叠叠乐未完成：${stack.state().stage}/${stack.state().message}`
  );
  assert(
    armYellowContacts.length === 0,
    `AGV 叠叠乐时机械臂撞到黄色圆柱：${JSON.stringify(armYellowContacts.slice(0, 12))}`
  );

  console.log(`variants ok: arm=${armGeomCount} geoms, agv=${agv.model.ngeom} geoms, navigation/grasp/stack=ok`);
  agv.data.delete();
  agv.model.delete();
  agv.vfs.delete();
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
