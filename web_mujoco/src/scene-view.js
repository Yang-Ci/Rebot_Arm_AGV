import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RectAreaLightUniformsLib } from 'three/addons/lights/RectAreaLightUniformsLib.js';
import { resolveExplosionLayout } from './explosion-layout.js';
import {
  EXPLOSION_TIMELINE_DURATION_MS,
  evaluateExplosionTimeline
} from './explosion-timeline.js';

const GEOM = {
  PLANE: 0,
  HFIELD: 1,
  SPHERE: 2,
  CAPSULE: 3,
  ELLIPSOID: 4,
  CYLINDER: 5,
  BOX: 6,
  MESH: 7
};

const EXPLOSION_DISTANCE = 0.115;
const EXPLOSION_PART_GAP = 0.044;
const EXPLOSION_GRIPPER_GAP = 0.066;
const EXPLOSION_GROUND_CLEARANCE = 0.003;
const EXPLOSION_CLEARANCE = 0.014;
const MAIN_VIEW_HELPER_LAYER = 1;
const NAV_ARRIVAL_FILL_MS = 340;
const NAV_ARRIVAL_CHECK_MS = 360;
const NAV_ARRIVAL_HOLD_MS = 620;
const NAV_ARRIVAL_FADE_MS = 260;
const MAX_CONTACT_VISUALS = 40;
const MOVABLE_OBJECT_BODY_NAMES = new Set(['red_cube', 'blue_block', 'yellow_cylinder']);
const PRESENTATION_OFFSET = { x: 0.055, y: -0.035, z: 0.072 };
const EXPLOSION_STAGE_KEYS = [
  'gripper', 'joint6', 'joint5', 'joint4', 'joint3', 'joint2', 'joint1', 'base'
];

function explosionStageForBody(bodyName) {
  if (bodyName.startsWith('gripper')) return 0;
  const link = bodyName.match(/^link([1-6])$/)?.[1];
  if (link) return 7 - Number(link);
  return 7;
}

function partKind(name) {
  const value = name.toLowerCase();
  if (value.includes('d405') || value.includes('camera') || value.includes('imager')) return 'camera';
  if (value.includes('wordmark') || value.includes('badge')) return 'badge';
  if (value.includes('gripper') || value.includes('finger')) return 'gripper';
  if (value.includes('motor')) return 'motor';
  if (value.includes('cnc')) return 'structure';
  if (value.includes('pla')) return 'cover';
  if (value.includes('base')) return 'base';
  if (value.includes('link')) return 'link';
  return 'component';
}

function namedObject(mujoco, model, type, id, fallback = '') {
  if (!Number.isInteger(id) || id < 0 || type == null) return fallback;
  return mujoco.mj_id2name(model, type, id) || fallback;
}

function nearestJointName(mujoco, model, bodyId) {
  let current = bodyId;
  while (current > 0) {
    const count = model.body_jntnum ? model.body_jntnum[current] : 0;
    if (count > 0) {
      const jointId = model.body_jntadr[current];
      return namedObject(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT.value, jointId, `joint-${jointId}`);
    }
    current = model.body_parentid ? model.body_parentid[current] : 0;
  }
  return '';
}

function enumValue(mujoco, name, fallback) {
  const value = mujoco.mjtGeom?.[name]?.value;
  return Number.isInteger(value) ? value : fallback;
}

function geomTypes(mujoco) {
  return {
    plane: enumValue(mujoco, 'mjGEOM_PLANE', GEOM.PLANE),
    sphere: enumValue(mujoco, 'mjGEOM_SPHERE', GEOM.SPHERE),
    capsule: enumValue(mujoco, 'mjGEOM_CAPSULE', GEOM.CAPSULE),
    ellipsoid: enumValue(mujoco, 'mjGEOM_ELLIPSOID', GEOM.ELLIPSOID),
    cylinder: enumValue(mujoco, 'mjGEOM_CYLINDER', GEOM.CYLINDER),
    box: enumValue(mujoco, 'mjGEOM_BOX', GEOM.BOX),
    mesh: enumValue(mujoco, 'mjGEOM_MESH', GEOM.MESH)
  };
}

function copyFloats(source, start, count) {
  return Float32Array.from(source.subarray(start, start + count));
}

function createMeshGeometry(model, meshId) {
  const vertadr = model.mesh_vertadr[meshId];
  const vertnum = model.mesh_vertnum[meshId];
  const faceadr = model.mesh_faceadr[meshId];
  const facenum = model.mesh_facenum[meshId];
  if (!vertnum || !facenum) return new THREE.BufferGeometry();

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    'position',
    new THREE.BufferAttribute(copyFloats(model.mesh_vert, vertadr * 3, vertnum * 3), 3)
  );
  const indices = Array.from(model.mesh_face.subarray(faceadr * 3, (faceadr + facenum) * 3));
  geometry.setIndex(indices);

  if (model.mesh_normal && model.mesh_normaladr && model.mesh_normalnum) {
    const normaladr = model.mesh_normaladr[meshId];
    const normalnum = model.mesh_normalnum[meshId];
    if (normalnum === vertnum) {
      geometry.setAttribute(
        'normal',
        new THREE.BufferAttribute(copyFloats(model.mesh_normal, normaladr * 3, normalnum * 3), 3)
      );
    }
  }
  if (!geometry.getAttribute('normal')) geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  return geometry;
}

function primitiveGeometry(type, size, types) {
  if (type === types.plane) {
    return new THREE.PlaneGeometry(2 * (size[0] || 5), 2 * (size[1] || 5));
  }
  if (type === types.sphere) {
    return new THREE.SphereGeometry(size[0], 24, 16);
  }
  if (type === types.capsule) {
    const geometry = new THREE.CapsuleGeometry(size[0], 2 * size[1], 8, 16);
    geometry.rotateX(Math.PI / 2);
    return geometry;
  }
  if (type === types.ellipsoid) {
    const geometry = new THREE.SphereGeometry(1, 24, 16);
    geometry.scale(size[0], size[1], size[2]);
    return geometry;
  }
  if (type === types.cylinder) {
    const geometry = new THREE.CylinderGeometry(size[0], size[0], 2 * size[1], 24);
    geometry.rotateX(Math.PI / 2);
    return geometry;
  }
  if (type === types.box) {
    return new THREE.BoxGeometry(2 * size[0], 2 * size[1], 2 * size[2]);
  }
  return null;
}

function geomMaterial(model, index, materialProps) {
  const rgba = model.geom_rgba.subarray(index * 4, index * 4 + 4);
  let color = new THREE.Color(rgba[0], rgba[1], rgba[2]);
  let opacity = rgba[3];
  let metalness = 0.12;
  let roughness = 0.55;
  const matid = model.geom_matid ? model.geom_matid[index] : -1;
  if (matid >= 0) {
    if (model.mat_rgba) {
      const matRgba = model.mat_rgba.subarray(matid * 4, matid * 4 + 4);
      color = new THREE.Color(matRgba[0], matRgba[1], matRgba[2]);
      opacity = matRgba[3];
    }
    const mp = materialProps && materialProps[matid];
    if (mp && (mp.metallic != null || mp.roughness != null)) {
      if (mp.metallic != null) metalness = mp.metallic;
      if (mp.roughness != null) roughness = mp.roughness;
    } else {
      if (model.mat_metallic) {
        metalness = model.mat_metallic[matid];
      } else if (model.mat_specular) {
        const spec = model.mat_specular.subarray(matid * 4, matid * 4 + 4);
        metalness = (spec[0] + spec[1] + spec[2]) / 3;
      }
      if (model.mat_roughness) {
        roughness = model.mat_roughness[matid];
      } else if (model.mat_shininess) {
        roughness = 1 - model.mat_shininess[matid];
      }
    }
  }
  const finish = materialProps && materialProps[matid];
  const isCnc = finish?.name === 'rs_anodized_silver_mat';
  const isTable = finish?.name === 'rs_table';
  const isD405 = finish?.name?.startsWith('d405_');
  const isD405Lens = finish?.name === 'd405_lens_mat';
  const isMetal = metalness >= 0.45;
  if (isCnc) color.multiplyScalar(0.72);
  if (isTable) color.multiplyScalar(0.58);
  return new THREE.MeshPhysicalMaterial({
    color,
    metalness: Number.isFinite(metalness) ? Math.max(0, Math.min(1, metalness)) : 0.12,
    roughness: Number.isFinite(roughness) ? Math.max(0, Math.min(1, roughness)) : 0.55,
    envMapIntensity: isCnc ? 1.9 : isMetal ? 1.2 : 0.72,
    clearcoat: isCnc ? 0.25 : 0,
    clearcoatRoughness: isCnc ? 0.14 : 0.4,
    emissive: isD405 ? color.clone().multiplyScalar(isD405Lens ? 0.36 : 0.10) : 0x000000,
    emissiveIntensity: isD405Lens ? 0.55 : isD405 ? 0.18 : 0,
    transparent: opacity < 0.999,
    opacity,
    side: THREE.DoubleSide
  });
}

function shouldDrawGeom(model, index) {
  const group = model.geom_group ? model.geom_group[index] : 0;
  if (group >= 3) return false;
  const alpha = model.geom_rgba[index * 4 + 3];
  if (alpha < 0.02) return false;
  return true;
}

function setPose(mesh, xpos, xmat, index) {
  mesh.matrixAutoUpdate = false;
  mesh.matrix.set(
    xmat[index * 9 + 0], xmat[index * 9 + 1], xmat[index * 9 + 2], xpos[index * 3 + 0],
    xmat[index * 9 + 3], xmat[index * 9 + 4], xmat[index * 9 + 5], xpos[index * 3 + 1],
    xmat[index * 9 + 6], xmat[index * 9 + 7], xmat[index * 9 + 8], xpos[index * 3 + 2],
    0, 0, 0, 1
  );
  mesh.matrixWorldNeedsUpdate = true;
}

function createGridTexture() {
  const size = 1024;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext('2d');

  ctx.fillStyle = '#020617';
  ctx.fillRect(0, 0, size, size);

  const glow = ctx.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size * 0.7);
  glow.addColorStop(0, 'rgba(8, 47, 105, 0.48)');
  glow.addColorStop(0.5, 'rgba(3, 21, 56, 0.18)');
  glow.addColorStop(1, 'rgba(2, 6, 23, 0)');
  ctx.fillStyle = glow;
  ctx.fillRect(0, 0, size, size);

  const sub = 10;
  ctx.strokeStyle = 'rgba(0, 126, 255, 0.34)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let i = 1; i < sub; i += 1) {
    const p = (i / sub) * size;
    ctx.moveTo(p, 0); ctx.lineTo(p, size);
    ctx.moveTo(0, p); ctx.lineTo(size, p);
  }
  ctx.stroke();

  ctx.shadowColor = 'rgba(0, 214, 255, 0.9)';
  ctx.shadowBlur = 10;
  ctx.strokeStyle = 'rgba(0, 207, 255, 0.96)';
  ctx.lineWidth = 3;
  ctx.strokeRect(0, 0, size, size);
  ctx.shadowBlur = 0;

  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(5, 5);
  texture.anisotropy = 8;
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function createBackdropTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 1600;
  canvas.height = 900;
  const ctx = canvas.getContext('2d');
  const { width, height } = canvas;

  const sky = ctx.createLinearGradient(0, 0, 0, height);
  sky.addColorStop(0, '#010319');
  sky.addColorStop(0.56, '#030629');
  sky.addColorStop(0.72, '#06154b');
  sky.addColorStop(0.76, '#03153d');
  sky.addColorStop(1, '#01030f');
  ctx.fillStyle = sky;
  ctx.fillRect(0, 0, width, height);

  const horizonY = height * 0.73;
  const horizon = ctx.createLinearGradient(0, horizonY - 60, 0, horizonY + 60);
  horizon.addColorStop(0, 'rgba(0, 87, 255, 0)');
  horizon.addColorStop(0.42, 'rgba(0, 118, 255, 0.24)');
  horizon.addColorStop(0.5, 'rgba(0, 229, 255, 0.98)');
  horizon.addColorStop(0.58, 'rgba(0, 94, 255, 0.30)');
  horizon.addColorStop(1, 'rgba(0, 30, 110, 0)');
  ctx.fillStyle = horizon;
  ctx.fillRect(0, horizonY - 60, width, 120);

  ctx.globalCompositeOperation = 'lighter';
  [0.035, 0.18, 0.50, 0.82, 0.965].forEach((ratio, index) => {
    const x = width * ratio;
    const beam = ctx.createLinearGradient(x - 22, 0, x + 22, 0);
    beam.addColorStop(0, 'rgba(0, 55, 255, 0)');
    beam.addColorStop(0.42, 'rgba(0, 84, 255, 0.18)');
    beam.addColorStop(0.5, index % 2 ? 'rgba(0, 121, 255, 0.78)' : 'rgba(0, 199, 255, 0.92)');
    beam.addColorStop(0.58, 'rgba(0, 84, 255, 0.18)');
    beam.addColorStop(1, 'rgba(0, 55, 255, 0)');
    ctx.fillStyle = beam;
    ctx.fillRect(x - 22, 0, 44, horizonY + 20);
    ctx.fillStyle = 'rgba(92, 220, 255, 0.64)';
    ctx.fillRect(x - 1, 0, 2, horizonY + 20);
  });
  ctx.globalCompositeOperation = 'source-over';

  ctx.strokeStyle = 'rgba(16, 78, 178, 0.18)';
  ctx.lineWidth = 1;
  for (let y = 120; y < horizonY; y += 8) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  const vignette = ctx.createRadialGradient(width / 2, height * 0.55, height * 0.18, width / 2, height * 0.55, width * 0.62);
  vignette.addColorStop(0, 'rgba(0, 0, 0, 0)');
  vignette.addColorStop(1, 'rgba(0, 0, 14, 0.64)');
  ctx.fillStyle = vignette;
  ctx.fillRect(0, 0, width, height);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function createEnvironmentTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 1024;
  canvas.height = 512;
  const ctx = canvas.getContext('2d');
  const { width, height } = canvas;

  const base = ctx.createLinearGradient(0, 0, 0, height);
  base.addColorStop(0, '#d8f6ff');
  base.addColorStop(0.14, '#3977b8');
  base.addColorStop(0.5, '#081b45');
  base.addColorStop(0.76, '#0b3c75');
  base.addColorStop(1, '#d8f7ff');
  ctx.fillStyle = base;
  ctx.fillRect(0, 0, width, height);

  [0.08, 0.30, 0.57, 0.82].forEach((ratio, index) => {
    const x = width * ratio;
    const panel = ctx.createLinearGradient(x - 55, 0, x + 55, 0);
    panel.addColorStop(0, 'rgba(37, 99, 235, 0)');
    panel.addColorStop(0.35, 'rgba(61, 193, 255, 0.28)');
    panel.addColorStop(0.5, index % 2 ? 'rgba(235, 252, 255, 0.98)' : 'rgba(74, 222, 255, 0.92)');
    panel.addColorStop(0.65, 'rgba(61, 193, 255, 0.28)');
    panel.addColorStop(1, 'rgba(37, 99, 235, 0)');
    ctx.fillStyle = panel;
    ctx.fillRect(x - 55, 0, 110, height);
  });

  const horizon = ctx.createLinearGradient(0, height * 0.62, 0, height * 0.83);
  horizon.addColorStop(0, 'rgba(0, 41, 120, 0)');
  horizon.addColorStop(0.48, 'rgba(60, 220, 255, 0.9)');
  horizon.addColorStop(0.55, 'rgba(230, 252, 255, 0.98)');
  horizon.addColorStop(1, 'rgba(0, 41, 120, 0)');
  ctx.fillStyle = horizon;
  ctx.fillRect(0, height * 0.62, width, height * 0.21);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.mapping = THREE.EquirectangularReflectionMapping;
  return texture;
}

export function createSceneView(host, options = {}) {
  const scene = new THREE.Scene();
  const reducedMotionQuery = window.matchMedia('(prefers-reduced-motion: reduce)');
  const backdropTexture = createBackdropTexture();
  scene.background = backdropTexture;

  const camera = new THREE.PerspectiveCamera(45, 1, 0.02, options.far || 20);
  camera.up.set(0, 0, 1);
  camera.position.set(...(options.position || [0.85, -0.95, 0.62]));
  camera.layers.enable(MAIN_VIEW_HELPER_LAYER);
  const overheadCamera = new THREE.PerspectiveCamera(48, 4 / 3, 0.02, 4);
  overheadCamera.matrixAutoUpdate = false;
  const wristCamera = new THREE.PerspectiveCamera(62.82, 4 / 3, 0.02, 4);
  wristCamera.matrixAutoUpdate = false;
  const previewDocument = host.ownerDocument;
  const overheadCanvas = previewDocument.getElementById('overhead-camera-canvas');
  const wristCanvas = previewDocument.getElementById('wrist-camera-canvas');

  const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.02;
  if ('outputColorSpace' in renderer) renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.domElement.style.zIndex = '0';
  host.appendChild(renderer.domElement);
  ['callouts', 'drag-cluster'].forEach((id) => {
    const node = host.querySelector(`#${id}`);
    if (node) host.appendChild(node);
  });

  const ndc = new THREE.Vector3();
  const CAMERA_PREVIEW_INTERVAL_MS = 100;
  const overheadRenderer = overheadCanvas
    ? new THREE.WebGLRenderer({ canvas: overheadCanvas, antialias: true, powerPreference: 'low-power' })
    : null;
  const wristRenderer = wristCanvas
    ? new THREE.WebGLRenderer({ canvas: wristCanvas, antialias: true, powerPreference: 'low-power' })
    : null;
  [overheadRenderer, wristRenderer].forEach((previewRenderer) => {
    if (!previewRenderer) return;
    previewRenderer.setPixelRatio(1);
    previewRenderer.setSize(320, 240, false);
    previewRenderer.shadowMap.enabled = false;
    previewRenderer.toneMapping = THREE.ACESFilmicToneMapping;
    previewRenderer.toneMappingExposure = 1.02;
    if ('outputColorSpace' in previewRenderer) previewRenderer.outputColorSpace = THREE.SRGBColorSpace;
  });

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(...(options.target || [0.28, 0, 0.16]));
  controls.enableDamping = true;

  const pmrem = new THREE.PMREMGenerator(renderer);
  const environmentSource = createEnvironmentTexture();
  const environmentTexture = pmrem.fromEquirectangular(environmentSource).texture;
  scene.environment = environmentTexture;
  environmentSource.dispose();
  pmrem.dispose();

  RectAreaLightUniformsLib.init();
  scene.add(new THREE.AmbientLight(0x6f9bd4, 0.26));
  scene.add(new THREE.HemisphereLight(0xd9f6ff, 0x021026, 0.48));

  const softbox = new THREE.RectAreaLight(0xeaf8ff, 2.0, 1.6, 1.15);
  softbox.position.set(0.28, -0.58, 1.28);
  softbox.lookAt(0.25, 0, 0.16);
  scene.add(softbox);

  const cyanRim = new THREE.RectAreaLight(0x10cfff, 2.8, 0.75, 1.05);
  cyanRim.position.set(-0.58, 0.2, 0.68);
  cyanRim.lookAt(0.23, 0, 0.18);
  scene.add(cyanRim);

  const blueRim = new THREE.RectAreaLight(0x245cff, 1.9, 0.7, 0.9);
  blueRim.position.set(0.86, 0.32, 0.62);
  blueRim.lookAt(0.25, 0, 0.18);
  scene.add(blueRim);

  const key = new THREE.DirectionalLight(0xf2fbff, 1.0);
  key.position.set(1.2, 0.8, 2.0);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.near = 0.1;
  key.shadow.camera.far = 6;
  key.shadow.camera.left = -1.4;
  key.shadow.camera.right = 1.4;
  key.shadow.camera.top = 1.4;
  key.shadow.camera.bottom = -1.4;
  key.shadow.bias = -0.00012;
  key.shadow.normalBias = 0.02;
  scene.add(key);

  const side = new THREE.DirectionalLight(0x8bdcff, 0.38);
  side.position.set(-1, -0.5, 0.8);
  scene.add(side);

  const rim = new THREE.DirectionalLight(0x286fff, 0.62);
  rim.position.set(-0.5, -1, 1.5);
  scene.add(rim);

  const tcpMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.024, 16, 12),
    new THREE.MeshBasicMaterial({
      color: 0x33d6b0,
      wireframe: true,
      transparent: true,
      opacity: 0.4
    })
  );
  tcpMarker.visible = false;
  tcpMarker.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(tcpMarker);

  const targetGhost = new THREE.Mesh(
    new THREE.SphereGeometry(0.018, 28, 18),
    new THREE.MeshBasicMaterial({ color: 0xf2a541, transparent: true, opacity: 0.85 })
  );
  targetGhost.visible = false;
  targetGhost.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(targetGhost);

  const dragErrorLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
    new THREE.LineBasicMaterial({ color: 0xff6b5f, transparent: true, opacity: 0.82 })
  );
  dragErrorLine.visible = false;
  dragErrorLine.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(dragErrorLine);

  const raycaster = new THREE.Raycaster();
  raycaster.layers.enable(MAIN_VIEW_HELPER_LAYER);
  const ndcMouse = new THREE.Vector2();
  const planeHit = new THREE.Vector3();
  const navigationGroundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  const navigationGoalMarker = new THREE.Mesh(
    new THREE.RingGeometry(0.10, 0.135, 32),
    new THREE.MeshBasicMaterial({
      color: 0x35d9ff,
      transparent: true,
      opacity: 0.92,
      side: THREE.DoubleSide,
      depthTest: false
    })
  );
  navigationGoalMarker.visible = false;
  navigationGoalMarker.position.z = 0.008;
  navigationGoalMarker.renderOrder = 20;
  navigationGoalMarker.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(navigationGoalMarker);
  const navigationGoalFill = new THREE.Mesh(
    new THREE.CircleGeometry(0.125, 32),
    new THREE.MeshBasicMaterial({
      color: 0x41e28b,
      transparent: true,
      opacity: 0.94,
      side: THREE.DoubleSide,
      depthTest: false
    })
  );
  navigationGoalFill.visible = false;
  navigationGoalFill.renderOrder = 21;
  navigationGoalFill.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(navigationGoalFill);
  const navigationCheckPoints = [];
  const navigationCheckAnchors = [
    new THREE.Vector3(-0.060, 0.002, 0),
    new THREE.Vector3(-0.018, -0.042, 0),
    new THREE.Vector3(0.070, 0.052, 0)
  ];
  for (let segment = 0; segment < navigationCheckAnchors.length - 1; segment += 1) {
    for (let step = segment ? 1 : 0; step <= 14; step += 1) {
      navigationCheckPoints.push(
        navigationCheckAnchors[segment].clone().lerp(
          navigationCheckAnchors[segment + 1],
          step / 14
        )
      );
    }
  }
  const navigationCheckGeometry = new THREE.BufferGeometry().setFromPoints(navigationCheckPoints);
  navigationCheckGeometry.setDrawRange(0, 0);
  const navigationGoalCheck = new THREE.Line(
    navigationCheckGeometry,
    new THREE.LineBasicMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: 1,
      depthTest: false
    })
  );
  navigationGoalCheck.visible = false;
  navigationGoalCheck.renderOrder = 22;
  navigationGoalCheck.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(navigationGoalCheck);
  let navigationArrivalStartedAt = null;
  const navigationPathGeometry = new THREE.BufferGeometry();
  const navigationPath = new THREE.Line(
    navigationPathGeometry,
    new THREE.LineBasicMaterial({
      color: 0x35d9ff,
      transparent: true,
      opacity: 0.9,
      depthTest: false
    })
  );
  navigationPath.visible = false;
  navigationPath.renderOrder = 19;
  navigationPath.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(navigationPath);
  const selectionBounds = new THREE.Box3();
  const selectionHelper = new THREE.Box3Helper(selectionBounds, 0x4defff);
  selectionHelper.visible = false;
  selectionHelper.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(selectionHelper);

  const contactPointPositions = new Float32Array(MAX_CONTACT_VISUALS * 3);
  const contactPointColors = new Float32Array(MAX_CONTACT_VISUALS * 3);
  const contactLinePositions = new Float32Array(MAX_CONTACT_VISUALS * 6);
  const contactLineColors = new Float32Array(MAX_CONTACT_VISUALS * 6);
  const contactPointPositionAttribute = new THREE.BufferAttribute(contactPointPositions, 3);
  const contactPointColorAttribute = new THREE.BufferAttribute(contactPointColors, 3);
  const contactLinePositionAttribute = new THREE.BufferAttribute(contactLinePositions, 3);
  const contactLineColorAttribute = new THREE.BufferAttribute(contactLineColors, 3);
  [
    contactPointPositionAttribute,
    contactPointColorAttribute,
    contactLinePositionAttribute,
    contactLineColorAttribute
  ].forEach((attribute) => attribute.setUsage(THREE.DynamicDrawUsage));
  const contactPointGeometry = new THREE.BufferGeometry();
  contactPointGeometry.setAttribute('position', contactPointPositionAttribute);
  contactPointGeometry.setAttribute('color', contactPointColorAttribute);
  contactPointGeometry.setDrawRange(0, 0);
  contactPointGeometry.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 10);
  const contactPoints = new THREE.Points(
    contactPointGeometry,
    new THREE.PointsMaterial({ size: 0.008, sizeAttenuation: true, vertexColors: true })
  );
  contactPoints.visible = false;
  contactPoints.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(contactPoints);

  const contactNormalGeometry = new THREE.BufferGeometry();
  contactNormalGeometry.setAttribute('position', contactLinePositionAttribute);
  contactNormalGeometry.setAttribute('color', contactLineColorAttribute);
  contactNormalGeometry.setDrawRange(0, 0);
  contactNormalGeometry.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 10);
  const contactNormals = new THREE.LineSegments(
    contactNormalGeometry,
    new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.92 })
  );
  contactNormals.visible = false;
  contactNormals.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(contactNormals);

  const meshes = [];
  const movableObjectMeshes = [];
  const explodableMeshes = [];
  const cameraModelMeshes = [];
  const partMeshes = new Map();
  const parts = [];
  const explosionGroups = new Map();
  const groundMeshes = [];
  const meshGeometries = new Map();
  const explosionCenter = new THREE.Vector3();
  const groupCenter = new THREE.Vector3();
  const partCenter = new THREE.Vector3();
  const explosionDirection = new THREE.Vector3();
  const explosionTangent = new THREE.Vector3();
  const presentationEuler = new THREE.Euler();
  const presentationRotation = new THREE.Matrix4();
  const presentationTransform = new THREE.Matrix4();
  const presentationBack = new THREE.Matrix4();
  let gridTexture = null;
  let types = geomTypes({});
  let timelineProgress = 0;
  let playbackDirection = 0;
  let explosionStageCount = EXPLOSION_STAGE_KEYS.length;
  let timelineListener = null;
  let groundPickListener = null;
  let lastTimelineSignature = '';
  let explosionAmount = 0;
  let presentationAmount = 0;
  let explosionLayoutReady = false;
  let lastSyncAt = 0;
  let selectedMesh = null;
  let partIsolated = false;
  let selectionListener = null;
  let pickStart = null;
  let lastCameraPreviewAt = -Infinity;
  let overheadCameraId = -1;
  let wristCameraId = -1;
  let overheadCameraEnabled = false;
  let wristCameraEnabled = false;
  let cameraModelVisible = true;
  let objectInteraction = {
    enabled: false,
    selectedBodyName: '',
    pose: null,
    onSelect: null,
    onMove: null,
    onRotate: null
  };
  let objectGesture = null;
  const objectGesturePlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  const objectGestureHit = new THREE.Vector3();
  const arcballCurrent = new THREE.Vector3();
  const arcballDelta = new THREE.Quaternion();
  const arcballWorldDelta = new THREE.Quaternion();
  const arcballCameraInverse = new THREE.Quaternion();
  const arcballResult = new THREE.Quaternion();
  const objectPulseColor = new THREE.Color(0x6feaff);

  function resize() {
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
  }

  function build(mujoco, model, materialProps) {
    clear();
    types = geomTypes(mujoco);
    for (let i = 0; i < model.ngeom; i += 1) {
      if (!shouldDrawGeom(model, i)) continue;
      const type = model.geom_type[i];
      const size = model.geom_size.subarray(i * 3, i * 3 + 3);
      let geometry = null;
      let meshId = -1;
      if (type === types.mesh) {
        meshId = model.geom_dataid[i];
        if (!meshGeometries.has(meshId)) {
          meshGeometries.set(meshId, createMeshGeometry(model, meshId));
        }
        geometry = meshGeometries.get(meshId);
      } else {
        geometry = primitiveGeometry(type, size, types);
      }
      if (!geometry) continue;
      let material;
      if (type === types.plane) {
        if (!gridTexture) gridTexture = createGridTexture();
        material = new THREE.MeshStandardMaterial({
          map: gridTexture,
          color: 0x41658f,
          metalness: 0.1,
          roughness: 0.82,
          envMapIntensity: 0.4
        });
      } else {
        material = geomMaterial(model, i, materialProps);
      }
      const mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = type !== types.plane;
      mesh.receiveShadow = true;
      mesh.userData.geomIndex = i;
      mesh.userData.explodable = (model.geom_group ? model.geom_group[i] : 0) === 1;
      const bodyId = model.geom_bodyid ? model.geom_bodyid[i] : -1;
      mesh.userData.bodyName = bodyId >= 0
        ? (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY.value, bodyId) || `body-${bodyId}`)
        : `geom-${i}`;
      mesh.userData.explosionStage = explosionStageForBody(mesh.userData.bodyName);
      if (!geometry.boundingSphere) geometry.computeBoundingSphere();
      if (!geometry.boundingBox) geometry.computeBoundingBox();
      mesh.userData.localCenter = geometry.boundingSphere?.center.clone() || new THREE.Vector3();
      mesh.userData.localBounds = geometry.boundingBox?.clone() || new THREE.Box3();
      scene.add(mesh);
      meshes.push(mesh);
      if (MOVABLE_OBJECT_BODY_NAMES.has(mesh.userData.bodyName)) {
        mesh.userData.objectBaseEmissive = mesh.material.emissive?.clone() || new THREE.Color(0);
        mesh.userData.objectBaseEmissiveIntensity = mesh.material.emissiveIntensity || 0;
        movableObjectMeshes.push(mesh);
      }
      if (type === types.plane) groundMeshes.push(mesh);
      if (mesh.userData.explodable) {
        explodableMeshes.push(mesh);
        const group = explosionGroups.get(mesh.userData.bodyName) || [];
        mesh.userData.explosionPartIndex = group.length;
        group.push(mesh);
        explosionGroups.set(mesh.userData.bodyName, group);
        const meshName = namedObject(
          mujoco,
          model,
          mujoco.mjtObj.mjOBJ_MESH?.value,
          meshId,
          ''
        );
        const geomName = namedObject(
          mujoco,
          model,
          mujoco.mjtObj.mjOBJ_GEOM.value,
          i,
          ''
        );
        const materialId = model.geom_matid ? model.geom_matid[i] : -1;
        const label = meshName || geomName || `${mesh.userData.bodyName}-${group.length}`;
        const part = {
          id: String(i),
          label,
          body: mesh.userData.bodyName,
          joint: nearestJointName(mujoco, model, bodyId),
          material: namedObject(
            mujoco,
            model,
            mujoco.mjtObj.mjOBJ_MATERIAL.value,
            materialId,
            ''
          ),
          kind: partKind(label),
          stage: mesh.userData.explosionStage
        };
        mesh.userData.part = part;
        mesh.userData.cameraModel = part.kind === 'camera';
        if (mesh.userData.cameraModel) cameraModelMeshes.push(mesh);
        mesh.userData.baseEmissive = mesh.material.emissive?.clone() || null;
        mesh.userData.baseEmissiveIntensity = mesh.material.emissiveIntensity || 0;
        parts.push(part);
        partMeshes.set(part.id, mesh);
      }
    }
    parts.sort((a, b) => a.stage - b.stage || a.label.localeCompare(b.label));
    explosionStageCount = Math.max(
      1,
      ...explodableMeshes.map((mesh) => mesh.userData.explosionStage + 1)
    );
    const cameraType = mujoco.mjtObj.mjOBJ_CAMERA?.value;
    overheadCameraId = cameraType == null
      ? -1
      : mujoco.mj_name2id(model, cameraType, 'overhead_rgb');
    wristCameraId = cameraType == null
      ? -1
      : mujoco.mj_name2id(model, cameraType, 'wrist_rgb');
    for (const [previewCamera, cameraId] of [
      [overheadCamera, overheadCameraId],
      [wristCamera, wristCameraId]
    ]) {
      const modelFov = Number(model.cam_fovy?.[cameraId]);
      if (cameraId >= 0 && Number.isFinite(modelFov) && modelFov > 0) {
        previewCamera.fov = modelFov;
        previewCamera.updateProjectionMatrix();
      }
    }
  }

  function sync(data) {
    const xpos = data.geom_xpos;
    const xmat = data.geom_xmat;
    meshes.forEach((mesh) => setPose(mesh, xpos, xmat, mesh.userData.geomIndex));

    const now = performance.now();
    const dt = lastSyncAt ? Math.min(0.05, (now - lastSyncAt) / 1000) : 1 / 60;
    lastSyncAt = now;
    if (playbackDirection !== 0) {
      if (reducedMotionQuery.matches) {
        timelineProgress = playbackDirection > 0 ? 1 : 0;
      } else {
        timelineProgress = THREE.MathUtils.clamp(
          timelineProgress + playbackDirection * (dt * 1000 / EXPLOSION_TIMELINE_DURATION_MS),
          0,
          1
        );
      }
      if (timelineProgress === 0 || timelineProgress === 1) playbackDirection = 0;
    }
    const timeline = evaluateExplosionTimeline(timelineProgress, explosionStageCount);
    presentationAmount = timeline.presentationAmount;
    explosionAmount = timeline.explosionAmount;

    if ((presentationAmount > 0 || explosionAmount > 0) && explodableMeshes.length) {
      // Use one center per articulated body. Averaging every render mesh would
      // bias the assembly center toward bodies with many trim pieces (notably
      // the gripper) and send all of those pieces down almost the same ray.
      explosionCenter.set(0, 0, 0);
      explosionGroups.forEach((group) => {
        groupCenter.set(0, 0, 0);
        group.forEach((mesh) => {
          partCenter.copy(mesh.userData.localCenter).applyMatrix4(mesh.matrix);
          groupCenter.add(partCenter);
        });
        groupCenter.multiplyScalar(1 / group.length);
        explosionCenter.add(groupCenter);
      });
      explosionCenter.multiplyScalar(1 / explosionGroups.size);

      if (presentationAmount > 0) {
        // Move the intact assembly into a display pose before separating it.
        // Applying one world-space transform to every arm mesh preserves the
        // articulated pose from MuJoCo while moving it away from the origin.
        presentationEuler.set(
          -0.07 * presentationAmount,
          0.1 * presentationAmount,
          0.22 * presentationAmount
        );
        presentationRotation.makeRotationFromEuler(presentationEuler);
        presentationTransform.makeTranslation(
          explosionCenter.x + PRESENTATION_OFFSET.x * presentationAmount,
          explosionCenter.y + PRESENTATION_OFFSET.y * presentationAmount,
          explosionCenter.z + PRESENTATION_OFFSET.z * presentationAmount
        );
        presentationBack.makeTranslation(
          -explosionCenter.x,
          -explosionCenter.y,
          -explosionCenter.z
        );
        presentationTransform.multiply(presentationRotation).multiply(presentationBack);
        explodableMeshes.forEach((mesh) => {
          mesh.matrix.premultiply(presentationTransform);
          mesh.matrixWorldNeedsUpdate = true;
        });
        explosionCenter.applyMatrix4(presentationTransform);
      }

      let groundZ = 0;
      if (groundMeshes.length) {
        groundZ = Math.max(...groundMeshes.map((mesh) => mesh.matrix.elements[14]));
      }

      if (!explosionLayoutReady && explosionAmount > 0) {
        const entries = [];
        explosionGroups.forEach((group, bodyName) => {
          groupCenter.set(0, 0, 0);
          group.forEach((mesh) => {
            partCenter.copy(mesh.userData.localCenter).applyMatrix4(mesh.matrix);
            groupCenter.add(partCenter);
          });
          groupCenter.multiplyScalar(1 / group.length);
          explosionDirection.copy(groupCenter).sub(explosionCenter);
          if (explosionDirection.lengthSq() < 1e-8) explosionDirection.set(1, 0, 0.18);
          explosionDirection.normalize();
          explosionTangent.set(-explosionDirection.y, explosionDirection.x, 0);
          if (explosionTangent.lengthSq() < 1e-8) explosionTangent.set(0, 1, 0);
          explosionTangent.normalize();

          const isGripper = bodyName.startsWith('gripper');
          const partGap = isGripper ? EXPLOSION_GRIPPER_GAP : EXPLOSION_PART_GAP;
          const groupDistance = EXPLOSION_DISTANCE * (isGripper ? 1.14 : 1);
          group.forEach((mesh, index) => {
            const lane = index - (group.length - 1) / 2;
            const offset = new THREE.Vector3(
              explosionDirection.x * groupDistance + explosionTangent.x * lane * partGap,
              explosionDirection.y * groupDistance + explosionTangent.y * lane * partGap,
              explosionDirection.z * groupDistance + Math.abs(lane) * partGap * 0.22
            );
            const box = mesh.userData.localBounds.clone().applyMatrix4(mesh.matrix).translate(offset);
            entries.push({
              id: mesh.userData.geomIndex,
              mesh,
              box,
              offset,
              escapeDirection: offset.clone().normalize()
            });
          });
        });

        resolveExplosionLayout(entries, {
          clearance: EXPLOSION_CLEARANCE,
          groundZ: groundZ + EXPLOSION_GROUND_CLEARANCE
        });
        entries.forEach((entry) => {
          entry.mesh.userData.explosionOffset = entry.offset;
        });
        explosionLayoutReady = true;
      }

      if (explosionLayoutReady) {
        explodableMeshes.forEach((mesh) => {
          const offset = mesh.userData.explosionOffset;
          if (!offset) return;
          const stageAmount = timeline.stageAmount(mesh.userData.explosionStage);
          mesh.matrix.elements[12] += offset.x * stageAmount;
          mesh.matrix.elements[13] += offset.y * stageAmount;
          mesh.matrix.elements[14] += offset.z * stageAmount;
          mesh.matrixWorldNeedsUpdate = true;
        });
      }
    }

    if (timelineProgress === 0) {
      host.classList.remove('exploded');
      explosionLayoutReady = false;
    } else {
      host.classList.add('exploded');
    }

    if (selectedMesh) {
      selectionBounds.copy(selectedMesh.userData.localBounds).applyMatrix4(selectedMesh.matrix);
      selectionHelper.visible = selectedMesh.visible;
      selectionHelper.updateMatrixWorld(true);
    }
    if (data.cam_xpos && data.cam_xmat) {
      if (overheadCameraId >= 0) setPose(overheadCamera, data.cam_xpos, data.cam_xmat, overheadCameraId);
      if (wristCameraId >= 0) setPose(wristCamera, data.cam_xpos, data.cam_xmat, wristCameraId);
    }
    notifyTimeline();
  }

  function render() {
    const now = performance.now();
    updateNavigationArrival(now);
    const objectPulse = 0.5 + 0.5 * Math.sin(now * 0.006);
    movableObjectMeshes.forEach((mesh) => {
      if (!mesh.material.emissive) return;
      mesh.material.emissive.copy(mesh.userData.objectBaseEmissive);
      mesh.material.emissiveIntensity = mesh.userData.objectBaseEmissiveIntensity;
      if (
        objectInteraction.enabled &&
        mesh.userData.bodyName === objectInteraction.selectedBodyName
      ) {
        mesh.material.emissive.lerp(objectPulseColor, 0.38 + objectPulse * 0.28);
        mesh.material.emissiveIntensity = 0.42 + objectPulse * 0.72;
      }
    });
    controls.update();
    renderer.render(scene, camera);
    if (now - lastCameraPreviewAt >= CAMERA_PREVIEW_INTERVAL_MS &&
        (overheadCameraEnabled || wristCameraEnabled)) {
      if (overheadCameraEnabled && overheadRenderer) {
        overheadRenderer.render(scene, overheadCamera);
      }
      if (wristCameraEnabled && wristRenderer) {
        wristRenderer.render(scene, wristCamera);
      }
      lastCameraPreviewAt = now;
    }
  }

  function projectWorld(x, y, z) {
    ndc.set(x, y, z).project(camera);
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    return {
      x: (ndc.x * 0.5 + 0.5) * width,
      y: (-ndc.y * 0.5 + 0.5) * height,
      visible: ndc.z > -1 && ndc.z < 1 && ndc.x > -1.35 && ndc.x < 1.35
    };
  }

  function intersectPlane(clientX, clientY, plane) {
    const rect = host.getBoundingClientRect();
    ndcMouse.set(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1
    );
    raycaster.setFromCamera(ndcMouse, camera);
    return raycaster.ray.intersectPlane(plane, planeHit) ? planeHit.clone() : null;
  }

  function setOrbitEnabled(enabled) {
    controls.enabled = enabled;
  }

  function setGroundPickHandler(listener) {
    groundPickListener = typeof listener === 'function' ? listener : null;
  }

  function setObjectInteraction(options = {}) {
    objectInteraction = {
      enabled: Boolean(options.enabled),
      selectedBodyName: options.selectedBodyName || '',
      pose: options.pose ? { ...options.pose } : null,
      onSelect: options.onSelect || null,
      onMove: options.onMove || null,
      onRotate: options.onRotate || null
    };
    host.classList.toggle('object-edit-mode', objectInteraction.enabled);
    if (!objectInteraction.enabled) {
      finishObjectGesture();
    }
  }

  function setNavigationGoalMarker(point, state = 'selected') {
    navigationGoalMarker.visible = Boolean(point);
    if (!point) {
      navigationArrivalStartedAt = null;
      navigationGoalFill.visible = false;
      navigationGoalCheck.visible = false;
      return;
    }
    navigationGoalMarker.position.set(point.x, point.y, 0.008);
    navigationGoalFill.position.set(point.x, point.y, 0.009);
    navigationGoalCheck.position.set(point.x, point.y, 0.011);
    const colors = { selected: 0x35d9ff, active: 0xffbd4a, reached: 0x41e28b };
    navigationGoalMarker.material.color.setHex(colors[state] || colors.selected);
    navigationGoalMarker.material.opacity = 0.92;
    navigationPath.material.opacity = 0.9;
    if (state === 'reached') {
      navigationArrivalStartedAt = performance.now();
      navigationGoalFill.visible = true;
      navigationGoalFill.scale.setScalar(0.001);
      navigationGoalFill.material.opacity = 0.94;
      navigationGoalCheck.visible = false;
      navigationGoalCheck.material.opacity = 1;
      navigationCheckGeometry.setDrawRange(0, 0);
    } else {
      navigationArrivalStartedAt = null;
      navigationGoalFill.visible = false;
      navigationGoalCheck.visible = false;
    }
  }

  function updateNavigationArrival(now) {
    if (navigationArrivalStartedAt == null) return;
    const elapsed = now - navigationArrivalStartedAt;
    const fillProgress = reducedMotionQuery.matches
      ? 1
      : THREE.MathUtils.clamp(elapsed / NAV_ARRIVAL_FILL_MS, 0, 1);
    const fillEased = 1 - (1 - fillProgress) ** 3;
    navigationGoalFill.scale.setScalar(Math.max(0.001, fillEased));

    const checkStartedAt = reducedMotionQuery.matches ? 0 : NAV_ARRIVAL_FILL_MS * 0.72;
    const checkProgress = reducedMotionQuery.matches
      ? 1
      : THREE.MathUtils.clamp(
          (elapsed - checkStartedAt) / NAV_ARRIVAL_CHECK_MS,
          0,
          1
        );
    navigationGoalCheck.visible = checkProgress > 0;
    navigationCheckGeometry.setDrawRange(
      0,
      Math.ceil(navigationCheckPoints.length * checkProgress)
    );

    const fadeStartedAt = reducedMotionQuery.matches
      ? NAV_ARRIVAL_HOLD_MS
      : NAV_ARRIVAL_FILL_MS + NAV_ARRIVAL_CHECK_MS + NAV_ARRIVAL_HOLD_MS;
    const fade = THREE.MathUtils.clamp((elapsed - fadeStartedAt) / NAV_ARRIVAL_FADE_MS, 0, 1);
    const opacity = 1 - fade;
    navigationGoalMarker.material.opacity = 0.92 * opacity;
    navigationGoalFill.material.opacity = 0.94 * opacity;
    navigationGoalCheck.material.opacity = opacity;
    navigationPath.material.opacity = 0.9 * opacity;
    if (fade < 1) return;
    navigationArrivalStartedAt = null;
    navigationGoalMarker.visible = false;
    navigationGoalFill.visible = false;
    navigationGoalCheck.visible = false;
    navigationPath.visible = false;
  }

  function setNavigationPath(points, state = 'selected') {
    const path = Array.isArray(points) ? points : [];
    navigationPath.visible = path.length >= 2;
    if (!navigationPath.visible) return;
    navigationPathGeometry.setFromPoints(
      path.map((point) => new THREE.Vector3(point.x, point.y, 0.012))
    );
    const colors = { selected: 0x35d9ff, active: 0xffbd4a, reached: 0x41e28b };
    navigationPath.material.color.setHex(colors[state] || colors.selected);
  }

  function setOverheadCameraEnabled(enabled) {
    overheadCameraEnabled = Boolean(enabled && overheadCameraId >= 0 && overheadRenderer);
    if (overheadCameraEnabled) lastCameraPreviewAt = -Infinity;
    return overheadCameraEnabled;
  }

  function setWristCameraEnabled(enabled) {
    wristCameraEnabled = Boolean(enabled && wristCameraId >= 0 && wristRenderer);
    if (wristCameraEnabled) lastCameraPreviewAt = -Infinity;
    return wristCameraEnabled;
  }

  function timelineState() {
    const state = evaluateExplosionTimeline(timelineProgress, explosionStageCount);
    let stageKey = EXPLOSION_STAGE_KEYS[state.activeStage] || 'base';
    if (timelineProgress === 0) stageKey = 'assembled';
    else if (state.explosionAmount === 0) stageKey = 'presentation';
    else if (timelineProgress === 1) stageKey = 'complete';
    return {
      progress: timelineProgress,
      direction: playbackDirection,
      stageIndex: state.activeStage,
      stageKey
    };
  }

  function notifyTimeline(force = false) {
    if (!timelineListener) return;
    const state = timelineState();
    const signature = `${state.progress.toFixed(4)}:${state.direction}:${state.stageKey}`;
    if (!force && signature === lastTimelineSignature) return;
    lastTimelineSignature = signature;
    timelineListener(state);
  }

  function playExplosion(direction = 1) {
    playbackDirection = direction < 0 ? -1 : 1;
    if ((playbackDirection > 0 && timelineProgress >= 1) ||
        (playbackDirection < 0 && timelineProgress <= 0)) {
      playbackDirection = 0;
    }
    if (playbackDirection > 0 && timelineProgress === 0) explosionLayoutReady = false;
    if (timelineProgress > 0 || playbackDirection > 0) host.classList.add('exploded');
    notifyTimeline(true);
    const destination = playbackDirection >= 0 ? 1 : 0;
    return Math.abs(destination - timelineProgress) * EXPLOSION_TIMELINE_DURATION_MS;
  }

  function pauseExplosion() {
    playbackDirection = 0;
    notifyTimeline(true);
  }

  function setExplosionProgress(value) {
    playbackDirection = 0;
    timelineProgress = THREE.MathUtils.clamp(Number(value) || 0, 0, 1);
    if (timelineProgress === 0) {
      explosionLayoutReady = false;
      host.classList.remove('exploded');
    } else {
      host.classList.add('exploded');
    }
    notifyTimeline(true);
  }

  function resetExplosion() {
    playbackDirection = 0;
    timelineProgress = 0;
    explosionAmount = 0;
    presentationAmount = 0;
    explosionLayoutReady = false;
    host.classList.remove('exploded');
    notifyTimeline(true);
  }

  function setExplosion(enabled) {
    return playExplosion(enabled ? 1 : -1);
  }

  function restoreSelectionMaterial(mesh) {
    if (!mesh?.material?.emissive) return;
    if (mesh.userData.baseEmissive) mesh.material.emissive.copy(mesh.userData.baseEmissive);
    mesh.material.emissiveIntensity = mesh.userData.baseEmissiveIntensity;
  }

  function applySelectionMaterial(mesh) {
    if (!mesh?.material?.emissive) return;
    mesh.material.emissive.set(0x18d9ff);
    mesh.material.emissiveIntensity = 0.72;
  }

  function applyIsolation() {
    explodableMeshes.forEach((mesh) => {
      const allowedByCameraToggle = cameraModelVisible || !mesh.userData.cameraModel;
      mesh.visible = allowedByCameraToggle && (!partIsolated || mesh === selectedMesh);
    });
    host.classList.toggle('part-isolated', partIsolated);
    selectionHelper.visible = Boolean(selectedMesh?.visible);
  }

  function notifySelection() {
    selectionListener?.({
      part: selectedMesh?.userData.part || null,
      isolated: partIsolated
    });
  }

  function selectPart(id) {
    const next = partMeshes.get(String(id));
    if (!next) return false;
    if (selectedMesh !== next) {
      restoreSelectionMaterial(selectedMesh);
      selectedMesh = next;
      applySelectionMaterial(selectedMesh);
    }
    applyIsolation();
    notifySelection();
    return true;
  }

  function setPartIsolated(enabled) {
    partIsolated = Boolean(enabled && selectedMesh);
    applyIsolation();
    notifySelection();
  }

  function clearPartSelection() {
    restoreSelectionMaterial(selectedMesh);
    selectedMesh = null;
    partIsolated = false;
    applyIsolation();
    selectionHelper.visible = false;
    notifySelection();
  }

  function setCameraModelVisible(enabled) {
    cameraModelVisible = Boolean(enabled);
    applyIsolation();
    return cameraModelVisible;
  }

  function pickPart(clientX, clientY) {
    const rect = renderer.domElement.getBoundingClientRect();
    ndcMouse.set(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1
    );
    raycaster.setFromCamera(ndcMouse, camera);
    const hit = raycaster.intersectObjects(
      explodableMeshes.filter((mesh) => mesh.visible),
      false
    )[0];
    if (hit) selectPart(hit.object.userData.part.id);
    else if (!partIsolated) clearPartSelection();
  }

  function setDragVisuals({ tcp, target, dragMode, dragging }) {
    tcpMarker.visible = Boolean(dragMode && tcp);
    if (tcp) tcpMarker.position.set(tcp.x, tcp.y, tcp.z);
    targetGhost.visible = Boolean(dragMode && dragging && target);
    if (target) targetGhost.position.set(target.x, target.y, target.z);
    const showLine = Boolean(dragMode && dragging && tcp && target && tcp.distanceTo(target) > 0.001);
    dragErrorLine.visible = showLine;
    if (showLine) {
      dragErrorLine.geometry.setFromPoints([tcp, target]);
    }
  }

  function setContactVisuals(contacts = [], enabled = true) {
    const count = enabled ? Math.min(contacts.length, MAX_CONTACT_VISUALS) : 0;
    if (!count) {
      contactPointGeometry.setDrawRange(0, 0);
      contactNormalGeometry.setDrawRange(0, 0);
      contactPoints.visible = false;
      contactNormals.visible = false;
      return;
    }
    for (let index = 0; index < count; index += 1) {
      const contact = contacts[index];
      const [x, y, z] = contact.position;
      const [nx, ny, nz] = contact.normal;
      const length = 0.012 + Math.min(0.038, Math.log1p(contact.force) * 0.007);
      const pointOffset = index * 3;
      contactPointPositions[pointOffset] = x;
      contactPointPositions[pointOffset + 1] = y;
      contactPointPositions[pointOffset + 2] = z;
      contactPointColors[pointOffset] = 1;
      contactPointColors[pointOffset + 1] = contact.selectedGrip ? 0.72 : 0.20;
      contactPointColors[pointOffset + 2] = contact.selectedGrip ? 0.10 : 0.18;

      const lineOffset = index * 6;
      contactLinePositions[lineOffset] = x;
      contactLinePositions[lineOffset + 1] = y;
      contactLinePositions[lineOffset + 2] = z;
      contactLinePositions[lineOffset + 3] = x + nx * length;
      contactLinePositions[lineOffset + 4] = y + ny * length;
      contactLinePositions[lineOffset + 5] = z + nz * length;
      const red = contact.selectedGrip ? 1 : 0.12;
      const green = contact.selectedGrip ? 0.78 : 0.92;
      const blue = contact.selectedGrip ? 0.16 : 1;
      contactLineColors[lineOffset] = red;
      contactLineColors[lineOffset + 1] = green;
      contactLineColors[lineOffset + 2] = blue;
      contactLineColors[lineOffset + 3] = red;
      contactLineColors[lineOffset + 4] = green;
      contactLineColors[lineOffset + 5] = blue;
    }
    contactPointPositionAttribute.needsUpdate = true;
    contactPointColorAttribute.needsUpdate = true;
    contactLinePositionAttribute.needsUpdate = true;
    contactLineColorAttribute.needsUpdate = true;
    contactPointGeometry.setDrawRange(0, count);
    contactNormalGeometry.setDrawRange(0, count * 2);
    contactPoints.visible = true;
    contactNormals.visible = true;
  }

  function clear() {
    restoreSelectionMaterial(selectedMesh);
    selectedMesh = null;
    partIsolated = false;
    selectionHelper.visible = false;
    host.classList.remove('part-isolated');
    meshes.forEach((mesh) => {
      scene.remove(mesh);
      mesh.material.dispose();
    });
    meshes.length = 0;
    movableObjectMeshes.length = 0;
    explodableMeshes.length = 0;
    cameraModelMeshes.length = 0;
    explosionGroups.clear();
    groundMeshes.length = 0;
    parts.length = 0;
    partMeshes.clear();
    meshGeometries.forEach((geometry) => geometry.dispose());
    meshGeometries.clear();
  }

  function updatePointerRay(clientX, clientY) {
    const rect = renderer.domElement.getBoundingClientRect();
    ndcMouse.set(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1
    );
    raycaster.setFromCamera(ndcMouse, camera);
  }

  function projectToArcball(clientX, clientY, gesture, target) {
    let x = (clientX - gesture.centerClientX) / gesture.radius;
    let y = (gesture.centerClientY - clientY) / gesture.radius;
    const lengthSquared = x * x + y * y;
    if (lengthSquared > 1) {
      const inverseLength = 1 / Math.sqrt(lengthSquared);
      x *= inverseLength;
      y *= inverseLength;
      target.set(x, y, 0);
    } else {
      target.set(x, y, Math.sqrt(1 - lengthSquared));
    }
    return target;
  }

  function beginObjectGesture(event) {
    const rotate = event.button === 2 || (event.button === 0 && (event.shiftKey || event.altKey));
    if (!objectInteraction.enabled || (event.button !== 0 && !rotate)) return false;
    updatePointerRay(event.clientX, event.clientY);
    const objectHit = raycaster.intersectObjects(
      movableObjectMeshes.filter((mesh) => mesh.visible),
      false
    )[0];
    if (!objectHit) return false;
    const bodyName = objectHit.object.userData.bodyName;
    objectInteraction.onSelect?.(bodyName);
    const pose = objectInteraction.pose;
    if (!pose) return true;
    if (rotate) {
      const screen = projectWorld(pose.x, pose.y, pose.z);
      const rect = renderer.domElement.getBoundingClientRect();
      const gesture = {
        mode: 'rotate',
        pointerId: event.pointerId,
        centerClientX: rect.left + screen.x,
        centerClientY: rect.top + screen.y,
        radius: Math.max(55, Math.min(110, rect.height * 0.14)),
        startVector: new THREE.Vector3(),
        cameraQuaternion: camera.getWorldQuaternion(new THREE.Quaternion()),
        startQuaternion: new THREE.Quaternion(
          pose.quaternion?.x || 0,
          pose.quaternion?.y || 0,
          pose.quaternion?.z || 0,
          pose.quaternion?.w ?? 1
        )
      };
      projectToArcball(event.clientX, event.clientY, gesture, gesture.startVector);
      objectGesture = gesture;
    } else {
      objectGesturePlane.constant = -pose.z;
      if (!raycaster.ray.intersectPlane(objectGesturePlane, objectGestureHit)) return true;
      objectGesture = {
        mode: 'move',
        pointerId: event.pointerId,
        offsetX: pose.x - objectGestureHit.x,
        offsetY: pose.y - objectGestureHit.y
      };
    }
    controls.enabled = false;
    renderer.domElement.setPointerCapture?.(event.pointerId);
    host.classList.add(objectGesture.mode === 'move' ? 'object-dragging' : 'object-rotating');
    event.preventDefault();
    return true;
  }

  function moveObjectGesture(event) {
    if (!objectGesture || event.pointerId !== objectGesture.pointerId) return;
    if (objectGesture.mode === 'move') {
      updatePointerRay(event.clientX, event.clientY);
      if (!raycaster.ray.intersectPlane(objectGesturePlane, objectGestureHit)) return;
      objectInteraction.onMove?.({
        x: objectGestureHit.x + objectGesture.offsetX,
        y: objectGestureHit.y + objectGesture.offsetY
      });
    } else {
      projectToArcball(event.clientX, event.clientY, objectGesture, arcballCurrent);
      arcballDelta.setFromUnitVectors(objectGesture.startVector, arcballCurrent);
      arcballCameraInverse.copy(objectGesture.cameraQuaternion).invert();
      arcballWorldDelta
        .copy(objectGesture.cameraQuaternion)
        .multiply(arcballDelta)
        .multiply(arcballCameraInverse);
      arcballResult
        .copy(arcballWorldDelta)
        .multiply(objectGesture.startQuaternion)
        .normalize();
      objectInteraction.onRotate?.({
        w: arcballResult.w,
        x: arcballResult.x,
        y: arcballResult.y,
        z: arcballResult.z
      });
    }
    event.preventDefault();
  }

  function finishObjectGesture(event) {
    if (event && objectGesture && event.pointerId !== objectGesture.pointerId) return false;
    const handled = Boolean(objectGesture);
    if (!handled) return false;
    objectGesture = null;
    controls.enabled = true;
    host.classList.remove('object-dragging', 'object-rotating');
    return true;
  }

  function onPickPointerDown(event) {
    if (beginObjectGesture(event)) {
      pickStart = null;
      return;
    }
    if (event.button !== 0) return;
    pickStart = { x: event.clientX, y: event.clientY, id: event.pointerId };
  }

  function onPickPointerUp(event) {
    if (finishObjectGesture(event)) return;
    if (!pickStart || event.pointerId !== pickStart.id) return;
    const distance = Math.hypot(event.clientX - pickStart.x, event.clientY - pickStart.y);
    pickStart = null;
    if (distance <= 5 && !host.classList.contains('tcp-dragging')) {
      if (groundPickListener) {
        const hit = intersectPlane(event.clientX, event.clientY, navigationGroundPlane);
        if (hit) groundPickListener({ x: hit.x, y: hit.y, z: 0 });
      } else if (!objectInteraction.enabled) {
        pickPart(event.clientX, event.clientY);
      }
    }
  }

  function onSelectionKeyDown(event) {
    if (event.key === 'Escape' && selectedMesh) clearPartSelection();
  }

  function onPickPointerCancel() {
    finishObjectGesture();
    pickStart = null;
  }

  function onContextMenu(event) {
    if (objectInteraction.enabled) event.preventDefault();
  }

  renderer.domElement.addEventListener('pointerdown', onPickPointerDown);
  renderer.domElement.addEventListener('pointermove', moveObjectGesture);
  renderer.domElement.addEventListener('pointerup', onPickPointerUp);
  renderer.domElement.addEventListener('pointercancel', onPickPointerCancel);
  renderer.domElement.addEventListener('contextmenu', onContextMenu);
  window.addEventListener('keydown', onSelectionKeyDown);

  const observer = new ResizeObserver(resize);
  observer.observe(host);
  resize();

  return {
    build,
    sync,
    render,
    resize,
    projectWorld,
    intersectPlane,
    setGroundPickHandler,
    setObjectInteraction,
    setNavigationGoalMarker,
    setNavigationPath,
    setOrbitEnabled,
    setOverheadCameraEnabled,
    isOverheadCameraEnabled() {
      return overheadCameraEnabled;
    },
    hasOverheadCamera() {
      return overheadCameraId >= 0;
    },
    setWristCameraEnabled,
    isWristCameraEnabled() {
      return wristCameraEnabled;
    },
    hasWristCamera() {
      return wristCameraId >= 0;
    },
    setCameraModelVisible,
    isCameraModelVisible() {
      return cameraModelVisible;
    },
    hasCameraModel() {
      return cameraModelMeshes.length > 0;
    },
    setExplosion,
    playExplosion,
    pauseExplosion,
    setExplosionProgress,
    resetExplosion,
    getExplosionState: timelineState,
    onExplosionChange(listener) {
      timelineListener = listener;
      lastTimelineSignature = '';
      notifyTimeline(true);
    },
    getParts() {
      return parts.map((part) => ({ ...part }));
    },
    selectPart,
    setPartIsolated,
    clearPartSelection,
    onPartSelectionChange(listener) {
      selectionListener = listener;
      notifySelection();
    },
    setDragVisuals,
    setContactVisuals,
    camera,
    dispose() {
      observer.disconnect();
      renderer.domElement.removeEventListener('pointerdown', onPickPointerDown);
      renderer.domElement.removeEventListener('pointermove', moveObjectGesture);
      renderer.domElement.removeEventListener('pointerup', onPickPointerUp);
      renderer.domElement.removeEventListener('pointercancel', onPickPointerCancel);
      renderer.domElement.removeEventListener('contextmenu', onContextMenu);
      window.removeEventListener('keydown', onSelectionKeyDown);
      clear();
      selectionHelper.geometry.dispose();
      selectionHelper.material.dispose();
      contactPointGeometry.dispose();
      contactPoints.material.dispose();
      contactNormalGeometry.dispose();
      contactNormals.material.dispose();
      navigationGoalMarker.geometry.dispose();
      navigationGoalMarker.material.dispose();
      navigationGoalFill.geometry.dispose();
      navigationGoalFill.material.dispose();
      navigationCheckGeometry.dispose();
      navigationGoalCheck.material.dispose();
      navigationPathGeometry.dispose();
      navigationPath.material.dispose();
      overheadRenderer?.dispose();
      wristRenderer?.dispose();
      if (gridTexture) { gridTexture.dispose(); gridTexture = null; }
      backdropTexture.dispose();
      environmentTexture.dispose();
      controls.dispose();
      renderer.dispose();
      renderer.domElement.remove();
    }
  };
}
