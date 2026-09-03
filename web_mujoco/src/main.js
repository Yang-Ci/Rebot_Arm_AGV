import { loadMujocoModule, loadRsScene } from './load-model.js';
import { bindJoints, homePose, readAngles } from './kinematics.js';
import { STEPS_PER_FRAME, createPhysicsController } from './pd-control.js';
import { createSceneView } from './scene-view.js';
import { createJointCallouts } from './ui.js';
import { createTcpIk } from './tcp-ik.js';
import { createTcpDrag } from './tcp-drag.js';
import { createPartExplorer } from './part-explorer.js';
import { createGraspDemo } from './grasp-demo.js';
import { createContactTelemetry } from './contact-telemetry.js';
import { createTelemetryPanel } from './telemetry-panel.js';
import { OBJECT_MOVE_STEP, createObjectPositionController } from './object-position-control.js';
import { t, bindLangSwitch, applyStaticI18n, onLangChange } from './i18n.js';
import { persistAppShell } from './register-service-worker.js';
import {
  AGV_FORWARD_SPEED,
  AGV_STEPS_PER_FRAME,
  AGV_TURN_SPEED,
  createAgvController
} from './agv-control.js';
import { selectedModelVariant, switchModelVariant } from './model-variants.js';
import {
  GRASP_DOCK,
  GRASP_WORKSPACE_OFFSET,
  createAgvNavigation
} from './agv-navigation.js';
import { LAB_NAV_OBSTACLES, planAgvPath } from './agv-path-planner.js';

const statusEl = document.getElementById('status');
const calloutsEl = document.getElementById('callouts');
const resetEl = document.getElementById('reset');
const modelVariantEl = document.getElementById('model-variant');
const panelHintEl = document.getElementById('panel-hint');
const toggleExplosionEl = document.getElementById('toggle-explosion');
const pauseExplosionEl = document.getElementById('pause-explosion');
const reverseExplosionEl = document.getElementById('reverse-explosion');
const explosionProgressEl = document.getElementById('explosion-progress');
const explosionProgressValueEl = document.getElementById('explosion-progress-value');
const explosionStageEl = document.getElementById('explosion-stage');
const partExplorerEl = document.getElementById('part-explorer');
const toggleDragEl = document.getElementById('toggle-drag');
const toggleGuidesEl = document.getElementById('toggle-guides');
const toggleCameraViewsEl = document.getElementById('toggle-camera-views');
const cameraFeedGridEl = document.getElementById('camera-feed-grid');
const cameraLayoutRowEl = document.getElementById('camera-layout-row');
const cameraLayoutColumnEl = document.getElementById('camera-layout-column');
const toggleCameraModelEl = document.getElementById('toggle-camera-model');
const overheadCameraFeedEl = document.getElementById('overhead-camera-feed');
const wristCameraFeedEl = document.getElementById('wrist-camera-feed');
const collapseOverheadCameraEl = document.getElementById('collapse-overhead-camera');
const collapseWristCameraEl = document.getElementById('collapse-wrist-camera');
const graspControlsEl = document.getElementById('grasp-controls');
const graspHintEl = document.getElementById('grasp-hint');
const visionTargetsEl = document.getElementById('vision-targets');
const startGraspEl = document.getElementById('start-grasp');
const startStackEl = document.getElementById('start-stack');
const cancelGraspEl = document.getElementById('cancel-grasp');
const graspProgressEl = document.getElementById('grasp-progress');
const graspProgressValueEl = document.getElementById('grasp-progress-value');
const graspStageEl = document.getElementById('grasp-stage');
const objectEditHudEl = document.getElementById('object-edit-hud');
const objectEditSelectionEl = document.getElementById('object-edit-selection');
const agvControlsEl = document.getElementById('agv-controls');
const agvLinearSpeedEl = document.getElementById('agv-linear-speed');
const agvLinearSpeedValueEl = document.getElementById('agv-linear-speed-value');
const agvTurnSpeedEl = document.getElementById('agv-turn-speed');
const agvTurnSpeedValueEl = document.getElementById('agv-turn-speed-value');
const setNavGoalEl = document.getElementById('set-nav-goal');
const goNavGoalEl = document.getElementById('go-nav-goal');
const goGraspZoneEl = document.getElementById('go-grasp-zone');
const cancelNavigationEl = document.getElementById('cancel-navigation');
const agvNavigationStateEl = document.getElementById('agv-navigation-state');
const explosionControlsEl = document.getElementById('explosion-controls');
const telemetryControlsEl = document.getElementById('telemetry-controls');
const toggleContactVisualsEl = document.getElementById('toggle-contact-visuals');
const dragMarkerEl = document.getElementById('drag-marker');
const dragClusterEl = document.getElementById('drag-cluster');
const gripperOpenEl = document.getElementById('gripper-open');
const gripperCloseEl = document.getElementById('gripper-close');
const viewportEl = document.getElementById('viewport');
const langSwitchEl = document.getElementById('lang-select');
const loadingOverlayEl = document.getElementById('loading-overlay');
const loadingTextEl = document.getElementById('loading-text');
const loadingProgressEl = document.getElementById('loading-progress');
const loadingProgressFillEl = document.getElementById('loading-progress-fill');
const loadingProgressValueEl = document.getElementById('loading-progress-value');

const LOAD_STAGE_PROGRESS = {
  'status.booting': 12,
  'status.loadingWasm': 49,
  'status.download': 69,
  'status.downloadProgress': 69,
  'status.loadingAssets': 69,
  'status.preparingRuntime': 89,
  'status.compiling': 94,
  'status.compiled': 99
};

const modelVariant = selectedModelVariant();
document.documentElement.dataset.modelVariant = modelVariant.id;
if (modelVariantEl) {
  modelVariantEl.value = modelVariant.id;
  modelVariantEl.addEventListener('change', () => switchModelVariant(modelVariantEl.value));
}
if (agvControlsEl) agvControlsEl.hidden = !modelVariant.hasAgv;
if (objectEditHudEl) objectEditHudEl.hidden = modelVariant.hasAgv;
if (explosionControlsEl) explosionControlsEl.hidden = modelVariant.hasAgv;
if (toggleDragEl) toggleDragEl.hidden = modelVariant.hasAgv;

function renderVariantUi() {
  if (panelHintEl) panelHintEl.textContent = t(modelVariant.hasAgv ? 'panel.hintAgv' : 'panel.hint');
  if (graspHintEl) graspHintEl.textContent = t(modelVariant.hasAgv ? 'grasp.hintAgv' : 'grasp.hint');
}

bindLangSwitch(langSwitchEl);
applyStaticI18n();
renderVariantUi();

function setStatus(text) {
  statusEl.textContent = text;
}

let loadProgress = { key: 'status.booting', vars: {} };
let loadingComplete = false;
let visualProgress = 0;
let visualProgressTarget = 0;
let visualProgressFrame = 0;
let visualProgressQueue = Promise.resolve();
let visualProgressCreepFrame = 0;
let visualProgressCreepActive = false;

function renderVisualProgress(value) {
  const rounded = Math.round(value);
  if (loadingProgressFillEl) loadingProgressFillEl.style.width = `${value}%`;
  if (loadingProgressValueEl) loadingProgressValueEl.textContent = `${rounded}%`;
  loadingProgressEl?.setAttribute('aria-valuenow', String(rounded));
}

function animateVisualProgress(next) {
  return new Promise((resolve) => {
    const start = visualProgress;
    const startedAt = performance.now();
    const duration = next >= 100 ? 120 : 240;

    const animate = (now) => {
      const ratio = Math.max(0, Math.min(1, (now - startedAt) / duration));
      const eased = 1 - (1 - ratio) ** 3;
      visualProgress = start + (next - start) * eased;
      renderVisualProgress(visualProgress);
      if (ratio < 1) {
        visualProgressFrame = requestAnimationFrame(animate);
      } else {
        visualProgress = next;
        renderVisualProgress(next);
        resolve();
      }
    };
    visualProgressFrame = requestAnimationFrame(animate);
  });
}

function setVisualProgress(next) {
  if (next <= visualProgressTarget) return visualProgressQueue;
  visualProgressTarget = next;
  visualProgressQueue = visualProgressQueue.then(() => animateVisualProgress(next));
  return visualProgressQueue;
}

function stopVisualProgressCreep() {
  visualProgressCreepActive = false;
  cancelAnimationFrame(visualProgressCreepFrame);
}

function startVisualProgressCreep() {
  if (visualProgressCreepActive) return;
  visualProgressCreepActive = true;

  void visualProgressQueue.then(() => {
    if (!visualProgressCreepActive || visualProgressTarget > 69) return;
    const start = visualProgress;
    const startedAt = performance.now();

    const creep = (now) => {
      if (!visualProgressCreepActive || visualProgressTarget > 69) return;
      const elapsedSeconds = Math.max(0, now - startedAt) / 1000;
      visualProgress = Math.min(88, Math.max(visualProgress, start + elapsedSeconds * 0.7));
      renderVisualProgress(visualProgress);
      visualProgressCreepFrame = requestAnimationFrame(creep);
    };

    visualProgressCreepFrame = requestAnimationFrame(creep);
  });
}

function renderLoadProgress() {
  const text = t(loadProgress.key, loadProgress.vars);
  setStatus(text);
  if (loadingTextEl) loadingTextEl.textContent = text;
}

function setLoadProgress(progress) {
  loadProgress = typeof progress === 'string' ? { key: progress, vars: {} } : progress;
  const nextProgress = LOAD_STAGE_PROGRESS[loadProgress.key];
  if (nextProgress === 69) {
    void setVisualProgress(nextProgress);
    startVisualProgressCreep();
  } else {
    stopVisualProgressCreep();
    if (nextProgress != null) void setVisualProgress(nextProgress);
  }
  renderLoadProgress();
}

function finishLoading() {
  loadingComplete = true;
  stopVisualProgressCreep();
  void setVisualProgress(100).then(() => {
    window.setTimeout(() => loadingOverlayEl?.classList.add('is-hidden'), 80);
  });
  void persistAppShell();
}

renderLoadProgress();
setVisualProgress(LOAD_STAGE_PROGRESS['status.booting']);

let panel = null;
let tcpDrag = null;
let partExplorer = null;
let graspDemo = null;
let objectPositionController = null;
let telemetryPanel = null;
let agvNavigation = null;
let readyCount = null;
let guidesVisible = false;
let overheadCameraVisible = false;
let wristCameraVisible = false;
let cameraViewsVisible = false;
let cameraViewsAvailable = false;
let cameraModelVisible = true;
let cameraModelAvailable = false;
let cameraLayout = 'row';
let overheadCameraCollapsed = false;
let wristCameraCollapsed = false;
let selectedVisionTarget = 'red';
let contactVisualsEnabled = true;
let graspState = { running: false, stage: 'idle', progress: 0, message: '' };
let agvDocked = !modelVariant.hasAgv;
let navGoalPickMode = false;
let agvNavigationState = { active: false, kind: 'point', goal: null, distance: 0 };
let agvNavigationMessage = { key: 'agv.noGoal', vars: {} };
let objectControlState = {
  enabled: false,
  selectedId: 'red',
  position: { x: 0.24, y: -0.30, z: 0.1225 },
  quaternion: { w: 1, x: 0, y: 0, z: 0 },
  yaw: 0,
  lastResult: null
};
let explosionState = { progress: 0, direction: 0, stageKey: 'assembled' };

function renderGuidesToggle() {
  if (!toggleGuidesEl) return;
  toggleGuidesEl.textContent = t(guidesVisible ? 'btn.guidesOff' : 'btn.guidesOn');
  toggleGuidesEl.classList.toggle('active', guidesVisible);
  toggleGuidesEl.setAttribute('aria-pressed', String(guidesVisible));
}

function renderCameraViewsToggle() {
  if (toggleCameraViewsEl) {
    toggleCameraViewsEl.textContent = t(cameraViewsVisible ? 'camera.hideAll' : 'camera.showAll');
    toggleCameraViewsEl.classList.toggle('active', cameraViewsVisible);
    toggleCameraViewsEl.setAttribute('aria-checked', String(cameraViewsVisible));
    toggleCameraViewsEl.disabled = !cameraViewsAvailable || explosionState.progress > 0;
  }
  if (cameraFeedGridEl) cameraFeedGridEl.hidden = !cameraViewsVisible;
  overheadCameraFeedEl?.classList.toggle('is-live', overheadCameraVisible);
  wristCameraFeedEl?.classList.toggle('is-live', wristCameraVisible);
}

function renderCameraModelToggle() {
  if (!toggleCameraModelEl) return;
  toggleCameraModelEl.textContent = t(cameraModelVisible ? 'camera.modelHide' : 'camera.modelShow');
  toggleCameraModelEl.setAttribute('aria-checked', String(cameraModelVisible));
  toggleCameraModelEl.classList.toggle('active', cameraModelVisible);
  toggleCameraModelEl.disabled = !cameraModelAvailable;
}

function renderCameraLayout() {
  const column = cameraLayout === 'column';
  cameraFeedGridEl?.classList.toggle('is-column', column);
  cameraLayoutRowEl?.classList.toggle('active', !column);
  cameraLayoutColumnEl?.classList.toggle('active', column);
  cameraLayoutRowEl?.setAttribute('aria-pressed', String(!column));
  cameraLayoutColumnEl?.setAttribute('aria-pressed', String(column));

  const renderCollapse = (feed, button, collapsed) => {
    feed?.classList.toggle('is-collapsed', collapsed);
    if (!button) return;
    button.textContent = collapsed ? '⌄' : '⌃';
    button.setAttribute('aria-expanded', String(!collapsed));
    button.setAttribute('aria-label', t(collapsed ? 'camera.expand' : 'camera.collapse'));
  };
  renderCollapse(overheadCameraFeedEl, collapseOverheadCameraEl, overheadCameraCollapsed);
  renderCollapse(wristCameraFeedEl, collapseWristCameraEl, wristCameraCollapsed);
}

function setAgvNavigationMessage(key, vars = {}) {
  agvNavigationMessage = { key, vars };
  if (agvNavigationStateEl) agvNavigationStateEl.textContent = t(key, vars);
}

function renderAgvNavigationControls() {
  if (!modelVariant.hasAgv) return;
  const busy = agvNavigationState.active || graspState.running;
  if (setNavGoalEl) {
    setNavGoalEl.disabled = busy;
    setNavGoalEl.classList.toggle('active', navGoalPickMode);
    setNavGoalEl.setAttribute('aria-pressed', String(navGoalPickMode));
  }
  if (goNavGoalEl) goNavGoalEl.disabled = busy || !agvNavigationState.goal;
  if (goGraspZoneEl) goGraspZoneEl.disabled = busy;
  if (cancelNavigationEl) cancelNavigationEl.disabled = !agvNavigationState.active;
  if (agvNavigationStateEl) {
    agvNavigationStateEl.textContent = t(agvNavigationMessage.key, agvNavigationMessage.vars);
  }
}

function renderGraspControls() {
  const dockingRequired = modelVariant.hasAgv && !agvDocked;
  visionTargetsEl?.querySelectorAll('[data-target]').forEach((button) => {
    const active = button.dataset.target === selectedVisionTarget;
    button.classList.toggle('active', active);
    button.setAttribute('aria-pressed', String(active));
    button.disabled = graspState.running;
  });
  if (startGraspEl) startGraspEl.disabled = dockingRequired || graspState.running || explosionState.progress > 0;
  if (startStackEl) startStackEl.disabled = dockingRequired || graspState.running || explosionState.progress > 0;
  if (cancelGraspEl) cancelGraspEl.disabled = !graspState.running;
  if (graspProgressEl) graspProgressEl.value = graspState.progress || 0;
  if (graspProgressValueEl) graspProgressValueEl.textContent = `${Math.round((graspState.progress || 0) * 100)}%`;
  if (graspStageEl) graspStageEl.textContent = t(`grasp.stage.${graspState.stage}`);
  document.body.classList.toggle('grasp-running', graspState.running);
  if (toggleDragEl) toggleDragEl.disabled = graspState.running;
  if (gripperOpenEl) gripperOpenEl.disabled = graspState.running;
  if (gripperCloseEl) gripperCloseEl.disabled = graspState.running;
  renderAgvNavigationControls();
}

function renderObjectEditHud() {
  if (!objectEditHudEl || modelVariant.hasAgv) return;
  objectEditHudEl.hidden = false;
  objectEditHudEl.classList.toggle('is-locked', !objectControlState.enabled);
  if (objectEditSelectionEl) {
    objectEditSelectionEl.textContent = t('object.hudSelected', {
      name: t(`vision.${objectControlState.selectedId}`)
    });
  }
}

function renderContactVisualsToggle() {
  if (!toggleContactVisualsEl) return;
  toggleContactVisualsEl.textContent = t(contactVisualsEnabled ? 'telemetry.hideContacts' : 'telemetry.showContacts');
  toggleContactVisualsEl.setAttribute('aria-checked', String(contactVisualsEnabled));
  toggleContactVisualsEl.classList.toggle('active', contactVisualsEnabled);
}

function renderExplosionControls() {
  const percent = Math.round(explosionState.progress * 100);
  explosionProgressEl.value = String(explosionState.progress * 100);
  explosionProgressValueEl.textContent = `${percent}%`;
  explosionProgressEl.setAttribute('aria-valuenow', String(percent));
  explosionStageEl.textContent = t(`timeline.stage.${explosionState.stageKey}`);
  toggleExplosionEl.disabled = graspState.running || (explosionState.progress >= 1 && explosionState.direction === 0);
  explosionProgressEl.disabled = graspState.running;
  pauseExplosionEl.disabled = explosionState.direction === 0;
  reverseExplosionEl.disabled = explosionState.progress <= 0 && explosionState.direction === 0;
  toggleExplosionEl.classList.toggle('active', explosionState.direction > 0);
  pauseExplosionEl.classList.toggle('active', explosionState.direction === 0 && explosionState.progress > 0 && explosionState.progress < 1);
  reverseExplosionEl.classList.toggle('active', explosionState.direction < 0);
  toggleExplosionEl.setAttribute('aria-pressed', String(explosionState.direction > 0));
  reverseExplosionEl.setAttribute('aria-pressed', String(explosionState.direction < 0));
}

onLangChange(() => {
  applyStaticI18n();
  renderVariantUi();
  panel?.applyLang();
  tcpDrag?.applyLang();
  partExplorer?.applyLang();
  renderGuidesToggle();
  renderCameraViewsToggle();
  renderCameraModelToggle();
  renderCameraLayout();
  renderGraspControls();
  renderAgvNavigationControls();
  renderObjectEditHud();
  renderContactVisualsToggle();
  renderExplosionControls();
  if (!loadingComplete) {
    renderLoadProgress();
  } else if (explosionState.direction > 0) {
    setStatus(t('status.exploding'));
  } else if (explosionState.direction < 0) {
    setStatus(t('status.assembling'));
  } else if (explosionState.progress >= 1) {
    setStatus(t('status.exploded'));
  } else if (explosionState.progress > 0) {
    setStatus(t('status.explosionPaused', { percent: Math.round(explosionState.progress * 100) }));
  } else if (!tcpDrag?.isEnabled() && readyCount != null) {
    setStatus(t(modelVariant.hasAgv ? 'status.agvReady' : 'status.ready', { n: readyCount }));
  }
});

async function main() {
  setLoadProgress('status.loadingWasm');
  const mujocoPromise = loadMujocoModule();
  const { mujoco, model, data, files, materialProps } = await loadRsScene(
    mujocoPromise,
    setLoadProgress,
    { sceneXml: modelVariant.sceneXml }
  );
  // Renderer, PMREM and lighting initialization are deliberately deferred so
  // they do not block the WASM and model requests on the first load.
  const view = createSceneView(viewportEl, modelVariant.view);
  const joints = bindJoints(mujoco, model);
  const agvController = modelVariant.hasAgv ? createAgvController(mujoco, model, data) : null;
  if (agvController) {
    const dynamicPalletBodyId = mujoco.mj_name2id(
      model,
      mujoco.mjtObj.mjOBJ_BODY.value,
      'dynamic_pallet'
    );
    const planWebPath = (start, goal) => {
      const obstacles = LAB_NAV_OBSTACLES.map((obstacle) => {
        if (obstacle.id !== 'dynamic-pallet' || dynamicPalletBodyId < 0) return obstacle;
        return {
          ...obstacle,
          x: data.xpos[dynamicPalletBodyId * 3],
          y: data.xpos[dynamicPalletBodyId * 3 + 1]
        };
      });
      return planAgvPath(start, goal, { obstacles });
    };
    agvNavigation = createAgvNavigation({
      drive: agvController,
      maxLinear: () => agvDriveSpeed,
      maxAngular: () => agvTurnSpeed,
      planPath: planWebPath,
      onChange: (state) => {
        agvNavigationState = state;
        const pathState = state.active
          ? 'active'
          : state.result === 'reached'
            ? 'reached'
            : 'selected';
        view.setNavigationPath(state.path, pathState);
        if (state.active && state.result === 'settling') {
          agvDocked = false;
          view.setNavigationGoalMarker(state.goal, 'active');
          setAgvNavigationMessage(state.kind === 'grasp' ? 'agv.graspSettling' : 'agv.navSettling');
        } else if (state.active) {
          agvDocked = false;
          view.setNavigationGoalMarker(state.goal, 'active');
          setAgvNavigationMessage(
            state.kind === 'grasp' ? 'agv.graspNavigating' : 'agv.navigating',
            { distance: state.distance.toFixed(2) }
          );
        } else if (state.result === 'reached') {
          agvDocked = state.kind === 'grasp';
          if (agvDocked) agvController.holdPlanarPose(GRASP_DOCK);
          view.setNavigationGoalMarker(state.goal, 'reached');
          setAgvNavigationMessage(state.kind === 'grasp' ? 'agv.graspReached' : 'agv.goalReached');
          setStatus(t(state.kind === 'grasp' ? 'agv.graspReached' : 'agv.goalReached'));
        } else if (['goal-blocked', 'start-blocked', 'unreachable'].includes(state.result)) {
          agvDocked = false;
          view.setNavigationGoalMarker(state.goal, 'selected');
          setAgvNavigationMessage('agv.routeBlocked');
          setStatus(t('agv.routeBlocked'));
        }
        renderGraspControls();
        renderAgvNavigationControls();
      }
    });
  }
  const physics = createPhysicsController(mujoco, model, data, joints, {
    beforeStep: () => {
      agvNavigation?.update();
      agvController?.apply();
      objectPositionController?.enforcePose();
    },
    afterStep: () => agvController?.enforceHeldPose()
  });
  const ik = createTcpIk(mujoco, model, data, joints);
  const contactTelemetry = createContactTelemetry(mujoco, model, data);

  let syncObjectInteractionState = null;
  objectPositionController = createObjectPositionController({
    mujoco,
    model,
    data,
    onChange: (state) => {
      objectControlState = state;
      syncObjectInteractionState?.(state);
      renderObjectEditHud();
    }
  });
  objectControlState = objectPositionController.state();
  const objectByBodyName = new Map(
    objectPositionController.objects.map((object) => [object.body, object])
  );
  syncObjectInteractionState = (state = objectPositionController.state()) => {
    const selectedObject = objectPositionController.objects.find(
      (object) => object.id === state.selectedId
    );
    view.setObjectInteraction({
      enabled: state.enabled,
      selectedBodyName: selectedObject?.body,
      pose: { ...state.position, quaternion: state.quaternion },
      onSelect: (bodyName) => {
        const object = objectByBodyName.get(bodyName);
        if (!object || object.id === objectControlState.selectedId) return;
        selectedVisionTarget = object.id;
        objectPositionController.setSelected(object.id);
        graspDemo?.setSelected(object.id);
      },
      onMove: ({ x, y }) => {
        reportObjectMove(objectPositionController.moveTo(x, y));
      },
      onRotate: (quaternion) => {
        reportObjectRotation(objectPositionController.setQuaternion(quaternion));
      }
    });
  };
  syncObjectInteractionState(objectControlState);
  objectPositionController.setEnabled(!modelVariant.hasAgv);

  setLoadProgress({ key: 'status.compiled', vars: { ngeom: model.ngeom } });
  view.build(mujoco, model, materialProps);
  view.sync(data);
  cameraViewsAvailable = view.hasOverheadCamera() || view.hasWristCamera();
  overheadCameraVisible = view.setOverheadCameraEnabled(true);
  wristCameraVisible = view.setWristCameraEnabled(true);
  cameraViewsVisible = overheadCameraVisible || wristCameraVisible;
  cameraModelAvailable = view.hasCameraModel();
  cameraModelVisible = view.setCameraModelVisible(true);

  function setCameraViewsVisible(enabled) {
    overheadCameraVisible = view.setOverheadCameraEnabled(enabled);
    wristCameraVisible = view.setWristCameraEnabled(enabled);
    cameraViewsVisible = overheadCameraVisible || wristCameraVisible;
    renderCameraViewsToggle();
    return cameraViewsVisible;
  }

  panel = createJointCallouts(calloutsEl, joints, (name, value) => {
    physics.setTarget(name, value);
  });
  panel.setGuidesVisible(guidesVisible);
  toggleGuidesEl?.addEventListener('click', () => {
    guidesVisible = !guidesVisible;
    panel.setGuidesVisible(guidesVisible);
    renderGuidesToggle();
  });
  renderGuidesToggle();
  toggleCameraViewsEl?.addEventListener('click', () => {
    const visible = setCameraViewsVisible(!cameraViewsVisible);
    setStatus(t(visible ? 'status.cameraViewsOn' : 'status.cameraViewsOff'));
  });
  renderCameraViewsToggle();
  toggleCameraModelEl?.addEventListener('click', () => {
    cameraModelVisible = view.setCameraModelVisible(!cameraModelVisible);
    renderCameraModelToggle();
    setStatus(t(cameraModelVisible ? 'status.cameraModelOn' : 'status.cameraModelOff'));
  });
  renderCameraModelToggle();
  cameraLayoutRowEl?.addEventListener('click', () => {
    cameraLayout = 'row';
    renderCameraLayout();
  });
  cameraLayoutColumnEl?.addEventListener('click', () => {
    cameraLayout = 'column';
    renderCameraLayout();
  });
  collapseOverheadCameraEl?.addEventListener('click', () => {
    overheadCameraCollapsed = !overheadCameraCollapsed;
    renderCameraLayout();
  });
  collapseWristCameraEl?.addEventListener('click', () => {
    wristCameraCollapsed = !wristCameraCollapsed;
    renderCameraLayout();
  });
  renderCameraLayout();
  let previousGraspStage = 'idle';
  graspDemo = createGraspDemo({
    mujoco,
    model,
    data,
    joints,
    physics,
    ik,
    workspaceOffset: modelVariant.hasAgv ? GRASP_WORKSPACE_OFFSET : { x: 0, y: 0, z: 0 },
    getObserverPosition: () => ({
      x: view.camera.position.x,
      y: view.camera.position.y,
      z: view.camera.position.z
    }),
    onChange: (state) => {
      graspState = state;
      selectedVisionTarget = state.selectedId;
      if (objectControlState.selectedId !== state.selectedId) {
        objectPositionController.setSelected(state.selectedId);
      }
      renderGraspControls();
      renderObjectEditHud();
      renderExplosionControls();
      if (state.stage !== previousGraspStage) {
        previousGraspStage = state.stage;
        if (state.stage === 'complete') {
          setStatus(t(state.mode === 'stack' ? 'status.stackComplete' : 'status.graspComplete'));
        }
        else if (state.stage === 'failed') setStatus(t('status.graspFailed'));
        else if (state.running) setStatus(t(`status.grasp.${state.stage}`));
      }
      const editingAvailable = !modelVariant.hasAgv && !state.running && explosionState.progress <= 0;
      if (objectPositionController.isEnabled() !== editingAvailable) {
        objectPositionController.setEnabled(editingAvailable);
      }
    }
  });
  telemetryPanel = createTelemetryPanel(telemetryControlsEl);
  visionTargetsEl?.addEventListener('click', (event) => {
    const button = event.target.closest('[data-target]');
    if (!button || graspState.running) return;
    selectedVisionTarget = button.dataset.target;
    graspDemo.setSelected(selectedVisionTarget);
    objectPositionController.setSelected(selectedVisionTarget);
    renderGraspControls();
  });
  function reportObjectMove(result) {
    if (result.ok) {
      const position = result.position;
      setStatus(t('status.objectMoved', {
        name: t(`vision.${objectControlState.selectedId}`),
        x: Math.round(position.x * 1000),
        y: Math.round(position.y * 1000)
      }));
    } else if (result.reason === 'boundary') setStatus(t('status.objectBoundary'));
    else if (result.reason === 'overlap') setStatus(t('status.objectOverlap'));
  }

  function reportObjectRotation(result) {
    if (!result.ok) {
      reportObjectMove(result);
      return;
    }
    setStatus(t('status.objectRotated', {
      name: t(`vision.${objectControlState.selectedId}`)
    }));
  }

  const objectDirections = {
    forward: [OBJECT_MOVE_STEP, 0],
    backward: [-OBJECT_MOVE_STEP, 0],
    left: [0, OBJECT_MOVE_STEP],
    right: [0, -OBJECT_MOVE_STEP]
  };

  function moveObject(direction) {
    const delta = objectDirections[direction];
    if (!delta || !objectPositionController.isEnabled()) return;
    reportObjectMove(objectPositionController.move(delta[0], delta[1]));
  }

  window.addEventListener('keydown', (event) => {
    if (!objectPositionController.isEnabled() || graspState.running || explosionState.progress > 0) return;
    if (event.target.closest?.('input, select, textarea, [contenteditable="true"]')) return;
    const direction = {
      KeyW: 'forward',
      KeyS: 'backward',
      KeyA: 'left',
      KeyD: 'right'
    }[event.code];
    if (!direction) return;
    event.preventDefault();
    moveObject(direction);
  });

  const agvKeys = new Set();
  let agvDriveSpeed = AGV_FORWARD_SPEED;
  let agvTurnSpeed = AGV_TURN_SPEED;

  function renderAgvSpeedControls() {
    if (agvLinearSpeedValueEl) agvLinearSpeedValueEl.textContent = `${agvDriveSpeed.toFixed(2)} m/s`;
    if (agvTurnSpeedValueEl) agvTurnSpeedValueEl.textContent = `${agvTurnSpeed.toFixed(2)} rad/s`;
  }

  function stopAgv() {
    if (!agvController) return;
    agvKeys.clear();
    agvController.stop();
  }

  function setGoalPickMode(enabled) {
    navGoalPickMode = Boolean(enabled && agvNavigation);
    viewportEl.classList.toggle('nav-goal-mode', navGoalPickMode);
    view.setGroundPickHandler(navGoalPickMode ? (point) => {
      const goal = {
        x: Math.max(-3.65, Math.min(3.65, point.x)),
        y: Math.max(-2.65, Math.min(2.65, point.y))
      };
      const planned = agvNavigation.setGoal(goal, 'point');
      view.setNavigationGoalMarker(goal, 'selected');
      setGoalPickMode(false);
      if (planned) {
        setAgvNavigationMessage('agv.goalSet', {
          x: goal.x.toFixed(2),
          y: goal.y.toFixed(2)
        });
      } else {
        setAgvNavigationMessage('agv.routeBlocked');
      }
      renderAgvNavigationControls();
    } : null);
    if (navGoalPickMode) setAgvNavigationMessage('agv.pickGoal');
    renderAgvNavigationControls();
  }

  function cancelAgvNavigation(showMessage = true, clearInactiveRoute = false) {
    if (!agvNavigationState.active && !clearInactiveRoute) return false;
    const wasActive = agvNavigation.cancel();
    if (showMessage) setAgvNavigationMessage('agv.navCancelled');
    renderAgvNavigationControls();
    return wasActive;
  }

  function updateAgvKeyboardCommand() {
    if (!agvController) return;
    const linear =
      (agvKeys.has('forward') ? agvDriveSpeed : 0) +
      (agvKeys.has('reverse') ? -agvDriveSpeed : 0);
    const angular =
      (agvKeys.has('left') ? agvTurnSpeed : 0) +
      (agvKeys.has('right') ? -agvTurnSpeed : 0);
    agvController.setCommand(linear, angular);
  }

  agvLinearSpeedEl?.addEventListener('input', () => {
    agvDriveSpeed = Number(agvLinearSpeedEl.value);
    renderAgvSpeedControls();
    updateAgvKeyboardCommand();
  });
  agvTurnSpeedEl?.addEventListener('input', () => {
    agvTurnSpeed = Number(agvTurnSpeedEl.value);
    renderAgvSpeedControls();
    updateAgvKeyboardCommand();
  });

  setNavGoalEl?.addEventListener('click', () => {
    setGoalPickMode(!navGoalPickMode);
  });
  goNavGoalEl?.addEventListener('click', () => {
    setGoalPickMode(false);
    stopAgv();
    agvDocked = false;
    agvController?.releasePlanarPose();
    if (!agvNavigation?.start()) setAgvNavigationMessage('agv.routeBlocked');
    renderGraspControls();
  });
  goGraspZoneEl?.addEventListener('click', () => {
    setGoalPickMode(false);
    stopAgv();
    agvDocked = false;
    agvController?.releasePlanarPose();
    if (!agvNavigation?.start(GRASP_DOCK, 'grasp')) {
      setAgvNavigationMessage('agv.routeBlocked');
    }
    renderGraspControls();
  });
  cancelNavigationEl?.addEventListener('click', () => cancelAgvNavigation());

  const agvKeyCommands = {
    ArrowUp: 'forward', KeyW: 'forward',
    ArrowDown: 'reverse', KeyS: 'reverse',
    ArrowLeft: 'left', KeyA: 'left',
    ArrowRight: 'right', KeyD: 'right',
    Space: 'stop'
  };
  window.addEventListener('keydown', (event) => {
    if (!agvController || event.target.closest?.('input, select, textarea, button, [contenteditable="true"]')) return;
    const command = agvKeyCommands[event.code];
    if (!command) return;
    event.preventDefault();
    if (graspState.running) return;
    setGoalPickMode(false);
    if (command === 'stop') stopAgv();
    else {
      cancelAgvNavigation(false, true);
      agvDocked = false;
      agvController.releasePlanarPose();
      agvKeys.add(command);
      updateAgvKeyboardCommand();
      renderGraspControls();
    }
  });
  window.addEventListener('keyup', (event) => {
    if (!agvController) return;
    const command = agvKeyCommands[event.code];
    if (!command) return;
    event.preventDefault();
    agvKeys.delete(command);
    updateAgvKeyboardCommand();
  });
  window.addEventListener('blur', () => {
    stopAgv();
    cancelAgvNavigation();
  });
  renderAgvSpeedControls();
  renderAgvNavigationControls();

  startGraspEl?.addEventListener('click', () => {
    if ((modelVariant.hasAgv && !agvDocked) || graspState.running || explosionState.progress > 0) return;
    stopAgv();
    objectPositionController.setEnabled(false);
    if (tcpDrag?.isEnabled()) tcpDrag.setEnabled(false);
    view.clearPartSelection();
    graspDemo.start(selectedVisionTarget);
  });
  startStackEl?.addEventListener('click', () => {
    if ((modelVariant.hasAgv && !agvDocked) || graspState.running || explosionState.progress > 0) return;
    stopAgv();
    objectPositionController.setEnabled(false);
    if (tcpDrag?.isEnabled()) tcpDrag.setEnabled(false);
    view.clearPartSelection();
    graspDemo.startStack();
  });
  cancelGraspEl?.addEventListener('click', () => {
    if (graspDemo.cancel()) setStatus(t('status.graspCancelled'));
  });
  toggleContactVisualsEl?.addEventListener('click', () => {
    contactVisualsEnabled = !contactVisualsEnabled;
    if (!contactVisualsEnabled) view.setContactVisuals([], false);
    renderContactVisualsToggle();
  });
  renderGraspControls();
  renderObjectEditHud();
  renderContactVisualsToggle();
  toggleExplosionEl?.addEventListener('click', () => {
    objectPositionController.setEnabled(false);
    if (cameraViewsVisible) setCameraViewsVisible(false);
    if (tcpDrag?.isEnabled()) tcpDrag.setEnabled(false);
    view.playExplosion(1);
    panel.closeChips();
    setStatus(t('status.exploding'));
  });
  pauseExplosionEl?.addEventListener('click', () => {
    view.pauseExplosion();
    setStatus(t('status.explosionPaused', {
      percent: Math.round(view.getExplosionState().progress * 100)
    }));
  });
  reverseExplosionEl?.addEventListener('click', () => {
    view.playExplosion(-1);
    setStatus(t('status.assembling'));
  });
  explosionProgressEl?.addEventListener('pointerdown', (event) => event.stopPropagation());
  explosionProgressEl?.addEventListener('input', () => {
    const progress = Number(explosionProgressEl.value) / 100;
    objectPositionController.setEnabled(!modelVariant.hasAgv && progress <= 0);
    if (progress > 0 && tcpDrag?.isEnabled()) tcpDrag.setEnabled(false);
    if (progress > 0 && cameraViewsVisible) setCameraViewsVisible(false);
    view.setExplosionProgress(progress);
    panel.closeChips();
    setStatus(t('status.explosionProgress', { percent: Math.round(progress * 100) }));
  });
  let previousExplosionDirection = 0;
  view.onExplosionChange((state) => {
    const stoppedAtEndpoint = previousExplosionDirection !== 0 && state.direction === 0;
    previousExplosionDirection = state.direction;
    explosionState = state;
    const editingAvailable = !modelVariant.hasAgv && !graspState.running && state.progress <= 0 && state.direction === 0;
    if (objectPositionController.isEnabled() !== editingAvailable) {
      objectPositionController.setEnabled(editingAvailable);
    }
    renderExplosionControls();
    renderCameraViewsToggle();
    if (stoppedAtEndpoint && state.progress >= 1) setStatus(t('status.exploded'));
    if (stoppedAtEndpoint && state.progress <= 0) setStatus(t('status.assembled'));
  });
  renderExplosionControls();
  partExplorer = createPartExplorer(partExplorerEl, view);
  Object.entries(homePose(joints)).forEach(([name, amount]) => {
    panel.setTarget(name, amount);
    panel.setActual(name, amount);
  });

  tcpDrag = createTcpDrag({
    view,
    ik,
    physics,
    panel,
    clusterEl: dragClusterEl,
    markerEl: dragMarkerEl,
    hostEl: viewportEl,
    toggleEl: toggleDragEl,
    openEl: gripperOpenEl,
    closeEl: gripperCloseEl,
    onStatus: setStatus
  });

  view.render();
  panel.layout(view.projectWorld, data);

  resetEl.addEventListener('click', () => {
    stopAgv();
    setGoalPickMode(false);
    cancelAgvNavigation(false, true);
    agvDocked = !modelVariant.hasAgv;
    agvController?.releasePlanarPose();
    tcpDrag.stop();
    graspDemo.reset();
    telemetryPanel.reset();
    view.resetExplosion();
    view.clearPartSelection();
    renderExplosionControls();
    physics.reset();
    objectPositionController.reset();
    objectPositionController.setEnabled(!modelVariant.hasAgv);
    if (modelVariant.hasAgv) {
      const goal = agvNavigation?.state().goal;
      view.setNavigationGoalMarker(goal, 'selected');
      if (goal) {
        setAgvNavigationMessage('agv.goalSet', {
          x: goal.x.toFixed(2),
          y: goal.y.toFixed(2)
        });
      } else {
        setAgvNavigationMessage('agv.noGoal');
      }
    }
    Object.entries(physics.targets).forEach(([name, amount]) => {
      panel.setTarget(name, amount);
      panel.setActual(name, amount);
    });
    setStatus(t(modelVariant.hasAgv ? 'status.agvReady' : 'status.ready', { n: readyCount }));
    renderGraspControls();
  });

  setStatus(t(modelVariant.hasAgv ? 'status.agvReady' : 'status.ready', { n: files.length }));
  readyCount = files.length;
  finishLoading();

  let lastTelemetryAt = -Infinity;
  let animationFrameId = 0;
  const loop = () => {
    animationFrameId = 0;
    if (document.hidden) return;
    const now = performance.now();
    graspDemo.update();
    physics.step(
      graspDemo.isRunning()
        ? 12
        : modelVariant.hasAgv
          ? AGV_STEPS_PER_FRAME
          : STEPS_PER_FRAME
    );
    const angles = readAngles(data, joints);
    Object.entries(angles).forEach(([name, amount]) => panel.setActual(name, amount));
    if (graspDemo.isRunning()) {
      Object.entries(physics.targets).forEach(([name, amount]) => panel.setTarget(name, amount));
    }
    view.sync(data);
    tcpDrag.update(now);
    if (now - lastTelemetryAt >= 160) {
      objectControlState = objectPositionController.state();
      renderObjectEditHud();
      const contacts = contactTelemetry.sample(selectedVisionTarget, contactVisualsEnabled);
      if (contactVisualsEnabled) view.setContactVisuals(contacts.contacts, true);
      telemetryPanel.update(physics.telemetry(), contacts);
      lastTelemetryAt = now;
    }
    view.render();
    panel.layout(view.projectWorld, data);
    animationFrameId = requestAnimationFrame(loop);
  };
  const startLoop = () => {
    if (!document.hidden && !animationFrameId) animationFrameId = requestAnimationFrame(loop);
  };
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
      cancelAnimationFrame(animationFrameId);
      animationFrameId = 0;
      stopAgv();
      return;
    }
    lastTelemetryAt = -Infinity;
    startLoop();
  });
  startLoop();
}

main().catch((error) => {
  console.error(error);
  const message = error && error.message ? error.message : error;
  loadingOverlayEl?.classList.add('is-error');
  setLoadProgress({ key: 'status.fail', vars: { error: message } });
});
