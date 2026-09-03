import * as THREE from 'three';
import { GRIPPER_JOINTS } from './kinematics.js';
import { t, onLangChange } from './i18n.js';

const SETTLE_ERROR = 0.004;
const SETTLE_TIMEOUT_MS = 900;
const GRIPPER_OPEN = GRIPPER_JOINTS[0].max;
const GRIPPER_CLOSE = 0;
const FAB_CLEARANCE = 160;
const TOUCH_SERVO_SPEEDUP = 1.65;
const TOUCH_STATUS_INTERVAL_MS = 90;

export function createTcpDrag({
  view,
  ik,
  physics,
  panel,
  clusterEl,
  markerEl,
  hostEl,
  toggleEl,
  openEl,
  closeEl,
  onStatus
}) {
  const target = new THREE.Vector3();
  const plane = new THREE.Plane();
  const cameraDir = new THREE.Vector3();
  let enabled = false;
  let dragging = false;
  let settling = false;
  let lastTime = 0;
  let settleStart = 0;
  let ikAngles = {};
  let activePointerId = null;
  let touchDragging = false;
  let targetClamped = false;
  let lastStatusTime = 0;

  function syncTargets(angles) {
    ik.armNames.forEach((name) => {
      physics.setTarget(name, angles[name]);
      panel.setTarget(name, angles[name]);
    });
  }

  function captureAngles() {
    const next = {};
    ik.armNames.forEach((name) => {
      next[name] = physics.targets[name];
    });
    return next;
  }

  function tcpVec() {
    const pos = ik.tcpPosition();
    return new THREE.Vector3(pos.x, pos.y, pos.z);
  }

  function syncGripperButtons() {
    const width = physics.targets.joint7 ?? 0;
    const open = width >= GRIPPER_OPEN * 0.5;
    openEl.classList.toggle('active', open);
    closeEl.classList.toggle('active', !open);
  }

  function setGripper(width) {
    physics.setTarget('joint7', width);
    panel.setTarget('joint7', width);
    syncGripperButtons();
    onStatus(width > 0 ? t('status.gripperOpen', { mm: (GRIPPER_OPEN * 1000).toFixed(0) }) : t('status.gripperClose'));
  }

  function updateMarker(pos) {
    if (!enabled || !pos) {
      clusterEl.classList.remove('active');
      return;
    }
    const projected = view.projectWorld(pos.x, pos.y, pos.z);
    clusterEl.classList.add('active');
    clusterEl.style.left = `${projected.x}px`;
    clusterEl.style.top = `${projected.y}px`;
    clusterEl.classList.toggle('flip-x', projected.x > hostEl.clientWidth - FAB_CLEARANCE);
    syncGripperButtons();
  }

  function applyOrbitLock() {
    view.setOrbitEnabled(!(enabled && dragging));
    hostEl.classList.toggle('tcp-drag', enabled);
    hostEl.classList.toggle('tcp-dragging', dragging);
    hostEl.classList.toggle('tcp-touch-dragging', dragging && touchDragging);
    document.documentElement.classList.toggle('tcp-touch-dragging', dragging && touchDragging);
    document.body.classList.toggle('tcp-touch-dragging', dragging && touchDragging);
  }

  function servoToward(now) {
    const elapsed = Math.max(0.012, (now - lastTime) / 1000 || 0.016);
    const dt = Math.min(0.05, elapsed * (touchDragging ? TOUCH_SERVO_SPEEDUP : 1));
    lastTime = now;
    const substeps = Math.max(1, Math.ceil(dt / 0.016));
    let result = null;
    for (let i = 0; i < substeps; i += 1) {
      result = ik.servoStep(target, dt / substeps, ikAngles);
    }
    if (result) syncTargets(result.angles);
    return result;
  }

  function setEnabled(next) {
    enabled = next;
    dragging = false;
    settling = false;
    activePointerId = null;
    touchDragging = false;
    panel.closeChips();
    toggleEl.classList.toggle('active', enabled);
    toggleEl.textContent = t(enabled ? 'btn.dragOff' : 'btn.dragOn');
    markerEl.classList.remove('dragging');
    applyOrbitLock();
    const tcp = tcpVec();
    target.copy(tcp);
    view.setDragVisuals({
      tcp,
      target,
      dragMode: enabled,
      dragging: false
    });
    updateMarker(tcp);
    onStatus(enabled ? t('status.dragOn') : t('status.dragOff'));
  }

  function onPointerDown(event) {
    if (!enabled || dragging) return;
    if (event.pointerType === 'mouse' && event.button !== 0) return;
    event.preventDefault();
    event.stopPropagation();
    dragging = true;
    activePointerId = event.pointerId;
    touchDragging = event.pointerType === 'touch';
    targetClamped = false;
    lastStatusTime = 0;
    settling = false;
    applyOrbitLock();
    panel.closeChips();
    lastTime = performance.now();
    ikAngles = captureAngles();
    target.copy(tcpVec());
    view.camera.getWorldDirection(cameraDir);
    plane.setFromNormalAndCoplanarPoint(cameraDir, target);
    markerEl.classList.add('dragging');
    try {
      markerEl.setPointerCapture(event.pointerId);
    } catch {
      // Some mobile browsers already own the pointer capture at this stage.
    }
  }

  function onPointerMove(event) {
    if (!dragging || event.pointerId !== activePointerId) return;
    if (event.cancelable) event.preventDefault();
    const samples = typeof event.getCoalescedEvents === 'function' ? event.getCoalescedEvents() : [];
    const sample = samples.length ? samples[samples.length - 1] : event;
    const hit = view.intersectPlane(sample.clientX, sample.clientY, plane);
    if (!hit) return;
    const bounded = ik.clampToWorkspace(hit);
    targetClamped = bounded.clamped;
    target.set(bounded.point.x, bounded.point.y, bounded.point.z);
    const result = servoToward(performance.now());
    const tcp = tcpVec();
    view.setDragVisuals({ tcp, target, dragMode: true, dragging: true });
    updateMarker(target);
    if (result) {
      onStatus(`${bounded.clamped ? t('status.clamped') : ''}${t('status.error', { mm: (result.error * 1000).toFixed(1) })}`);
    }
  }

  function onPointerUp(event) {
    if (!dragging || (event.pointerId != null && event.pointerId !== activePointerId)) return;
    dragging = false;
    markerEl.classList.remove('dragging');
    if (activePointerId != null && markerEl.hasPointerCapture(activePointerId)) {
      markerEl.releasePointerCapture(activePointerId);
    }
    applyOrbitLock();
    activePointerId = null;
    touchDragging = false;
    const tcp = tcpVec();
    if (tcp.distanceTo(target) > SETTLE_ERROR) {
      settling = true;
      settleStart = performance.now();
      lastTime = settleStart;
      onStatus(t('status.settling', { mm: (tcp.distanceTo(target) * 1000).toFixed(1) }));
      return;
    }
    view.setDragVisuals({ tcp, target, dragMode: true, dragging: false });
    updateMarker(tcp);
    onStatus(t('status.arrived', { mm: (tcp.distanceTo(target) * 1000).toFixed(1) }));
  }

  function update(now) {
    let tcp = tcpVec();
    if (!enabled) {
      view.setDragVisuals({ tcp, target, dragMode: false, dragging: false });
      updateMarker(null);
      return;
    }
    if (dragging && touchDragging) {
      const result = servoToward(now);
      tcp = tcpVec();
      view.setDragVisuals({ tcp, target, dragMode: true, dragging: true });
      updateMarker(target);
      if (result && now - lastStatusTime >= TOUCH_STATUS_INTERVAL_MS) {
        lastStatusTime = now;
        onStatus(`${targetClamped ? t('status.clamped') : ''}${t('status.error', { mm: (result.error * 1000).toFixed(1) })}`);
      }
      return;
    }
    if (settling && !dragging) {
      const result = servoToward(now);
      const error = result ? result.error : tcp.distanceTo(target);
      view.setDragVisuals({ tcp, target, dragMode: true, dragging: true });
      updateMarker(target);
      if ((result && result.reached) || error <= SETTLE_ERROR) {
        settling = false;
        view.setDragVisuals({ tcp, target, dragMode: true, dragging: false });
        updateMarker(tcp);
        onStatus(t('status.arrived', { mm: (error * 1000).toFixed(1) }));
      } else if (now - settleStart >= SETTLE_TIMEOUT_MS) {
        settling = false;
        view.setDragVisuals({ tcp, target, dragMode: true, dragging: false });
        updateMarker(tcp);
        onStatus(t('status.closest', { mm: (error * 1000).toFixed(1) }));
      } else {
        onStatus(t('status.settling', { mm: (error * 1000).toFixed(1) }));
      }
      return;
    }
    const follow = dragging ? target : tcp;
    view.setDragVisuals({ tcp, target, dragMode: true, dragging });
    updateMarker(follow);
  }

  function stop() {
    dragging = false;
    settling = false;
    activePointerId = null;
    touchDragging = false;
    markerEl.classList.remove('dragging');
    applyOrbitLock();
    if (enabled) {
      const tcp = tcpVec();
      target.copy(tcp);
      view.setDragVisuals({ tcp, target, dragMode: true, dragging: false });
      updateMarker(tcp);
    }
  }

  function applyLang() {
    toggleEl.textContent = t(enabled ? 'btn.dragOff' : 'btn.dragOn');
    openEl.textContent = t('btn.open');
    closeEl.textContent = t('btn.close');
    markerEl.title = t('marker.tcp');
    if (enabled && !dragging && !settling) onStatus(t('status.dragOn'));
  }

  function bindGripperButton(el, width) {
    el.addEventListener('pointerdown', (event) => {
      event.stopPropagation();
    });
    el.addEventListener('click', (event) => {
      event.stopPropagation();
      setGripper(width);
    });
  }

  function preventTouchNavigation(event) {
    if (dragging && touchDragging && event.cancelable) event.preventDefault();
  }

  function preventTouchStart(event) {
    if (enabled && event.cancelable) event.preventDefault();
  }

  toggleEl.addEventListener('click', () => setEnabled(!enabled));
  bindGripperButton(openEl, GRIPPER_OPEN);
  bindGripperButton(closeEl, GRIPPER_CLOSE);
  markerEl.addEventListener('pointerdown', onPointerDown, { passive: false });
  markerEl.addEventListener('touchstart', preventTouchStart, { passive: false });
  window.addEventListener('pointermove', onPointerMove, { passive: false });
  window.addEventListener('pointerup', onPointerUp);
  window.addEventListener('pointercancel', onPointerUp);
  window.addEventListener('touchmove', preventTouchNavigation, { passive: false });
  onLangChange(applyLang);

  return {
    update,
    stop,
    setEnabled,
    applyLang,
    isEnabled() {
      return enabled;
    }
  };
}
