export const GRASP_DOCK = Object.freeze({ x: 2.5, y: 0, yaw: 0 });
export const GRASP_WORKSPACE_OFFSET = Object.freeze({ x: 2.5, y: 0, z: 0.136 });

function clip(value, limit) {
  return Math.min(limit, Math.max(-limit, value));
}

function wrapAngle(value) {
  return Math.atan2(Math.sin(value), Math.cos(value));
}

export function createAgvNavigation({ drive, maxLinear, maxAngular, planPath, onChange }) {
  let goal = null;
  let path = [];
  let waypointIndex = 0;
  let active = false;
  let kind = 'point';
  let result = 'idle';
  let settleStepsRemaining = 0;
  let lastState = '';

  function snapshot(pose = drive.pose()) {
    const distance = goal ? Math.hypot(goal.x - pose.x, goal.y - pose.y) : 0;
    return {
      active,
      kind,
      result,
      goal: goal ? { ...goal } : null,
      pose,
      distance,
      path: path.map((point) => ({ ...point })),
      waypointIndex
    };
  }

  function notify(force = false, pose = drive.pose()) {
    const distance = goal ? Math.hypot(goal.x - pose.x, goal.y - pose.y) : 0;
    const signature = `${active}:${kind}:${result}:${goal?.x?.toFixed(2)}:${goal?.y?.toFixed(2)}:${distance.toFixed(2)}`;
    if (!force && signature === lastState) return null;
    lastState = signature;
    const state = snapshot(pose);
    onChange?.(state);
    return state;
  }

  function setGoal(next, nextKind = 'point') {
    if (!Number.isFinite(next?.x) || !Number.isFinite(next?.y)) return false;
    goal = {
      x: next.x,
      y: next.y,
      yaw: Number.isFinite(next.yaw) ? next.yaw : null
    };
    kind = nextKind;
    active = false;
    result = 'selected';
    settleStepsRemaining = 0;
    path = [];
    waypointIndex = 0;
    drive.releasePlanarPose?.();
    drive.stop();
    plan();
    notify(true);
    return path.length > 0;
  }

  function plan() {
    if (!goal) return false;
    const pose = drive.pose();
    const planned = planPath?.(pose, goal) || { ok: true, path: [goal] };
    if (!planned.ok || !planned.path?.length) {
      path = [];
      waypointIndex = 0;
      active = false;
      result = planned.reason || 'unreachable';
      return false;
    }
    path = planned.path.map((point) => ({ x: point.x, y: point.y, yaw: point.yaw }));
    waypointIndex = path.length > 1 ? 1 : 0;
    result = 'selected';
    return true;
  }

  function start(next = goal, nextKind = kind) {
    if (next && next !== goal) setGoal(next, nextKind);
    if (!goal) return false;
    if (!plan()) {
      drive.stop();
      notify(true);
      return false;
    }
    kind = nextKind;
    active = true;
    result = 'navigating';
    settleStepsRemaining = 0;
    drive.releasePlanarPose?.();
    notify(true);
    return true;
  }

  function cancel() {
    const wasActive = active;
    active = false;
    result = 'cancelled';
    settleStepsRemaining = 0;
    path = [];
    waypointIndex = 0;
    drive.releasePlanarPose?.();
    drive.stop();
    notify(true);
    return wasActive;
  }

  function update() {
    if (!active || !goal) return null;
    if (result === 'settling') {
      drive.stop();
      settleStepsRemaining -= 1;
      if (settleStepsRemaining > 0) return null;
      if (kind !== 'grasp') drive.releasePlanarPose?.();
      active = false;
      result = 'reached';
      return notify(true);
    }
    const pose = drive.pose();
    while (waypointIndex < path.length - 1) {
      const waypoint = path[waypointIndex];
      if (Math.hypot(waypoint.x - pose.x, waypoint.y - pose.y) > 0.09) break;
      waypointIndex += 1;
    }
    const waypoint = path[waypointIndex] || goal;
    const dx = waypoint.x - pose.x;
    const dy = waypoint.y - pose.y;
    const distance = Math.hypot(dx, dy);
    const finalWaypoint = waypointIndex >= path.length - 1;

    if (!finalWaypoint || distance > 0.035) {
      const headingError = wrapAngle(Math.atan2(dy, dx) - pose.yaw);
      const aligned = Math.max(0, Math.cos(headingError));
      const linear = Math.abs(headingError) > 0.75
        ? 0
        : Math.min(
            maxLinear(),
            finalWaypoint ? Math.max(0.12, distance * 0.75) : maxLinear()
          ) * aligned;
      const angular = clip(headingError * 2.2, maxAngular());
      drive.setCommand(linear, angular);
      return notify(false, pose);
    }

    const yawError = Number.isFinite(goal.yaw) ? wrapAngle(goal.yaw - pose.yaw) : 0;
    if (Math.abs(yawError) > 0.025) {
      drive.setCommand(0, clip(yawError * 2.2, maxAngular()));
      return notify(false, pose);
    }

    const finalPose = {
      x: goal.x,
      y: goal.y,
      yaw: Number.isFinite(goal.yaw) ? goal.yaw : pose.yaw
    };
    drive.holdPlanarPose?.(finalPose);
    settleStepsRemaining = 500;
    result = 'settling';
    return notify(true);
  }

  return { setGoal, start, cancel, update, state: snapshot };
}
