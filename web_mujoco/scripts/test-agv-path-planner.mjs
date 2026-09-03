import { LAB_NAV_OBSTACLES, planAgvPath } from '../src/agv-path-planner.js';
import { GRASP_DOCK } from '../src/agv-navigation.js';

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function distanceToRectangle(point, obstacle) {
  return Math.hypot(
    Math.max(Math.abs(point.x - obstacle.x) - obstacle.halfX, 0),
    Math.max(Math.abs(point.y - obstacle.y) - obstacle.halfY, 0)
  );
}

function assertSegmentsClear(path, obstacles, clearance) {
  for (let index = 1; index < path.length; index += 1) {
    const a = path[index - 1];
    const b = path[index];
    const length = Math.hypot(b.x - a.x, b.y - a.y);
    const samples = Math.max(1, Math.ceil(length / 0.025));
    for (let sample = 0; sample <= samples; sample += 1) {
      const ratio = sample / samples;
      const point = {
        x: a.x + (b.x - a.x) * ratio,
        y: a.y + (b.y - a.y) * ratio
      };
      assert(
        obstacles.every((obstacle) => distanceToRectangle(point, obstacle) >= clearance - 1e-6),
        `路径侵入障碍物安全距离：${JSON.stringify(point)}`
      );
    }
  }
}

const dock = planAgvPath({ x: -2.45, y: -0.55 }, GRASP_DOCK);
assert(dock.ok && dock.path.length >= 2, `抓取区路径规划失败：${dock.reason}`);
assertSegmentsClear(dock.path, LAB_NAV_OBSTACLES, 0.30);

const barrier = { id: 'barrier', x: 0, y: 0, halfX: 0.15, halfY: 0.90 };
const detour = planAgvPath(
  { x: -2, y: 0 },
  { x: 2, y: 0 },
  {
    bounds: { minX: -2.5, maxX: 2.5, minY: -1.8, maxY: 1.8 },
    obstacles: [barrier],
    footprintClearance: 0.25,
    resolution: 0.05
  }
);
assert(detour.ok, `单障碍绕行失败：${detour.reason}`);
assertSegmentsClear(detour.path, [barrier], 0.25);
assert(
  detour.path.some((point) => Math.abs(point.y) > 1.12),
  `路径没有绕过障碍：${JSON.stringify(detour.path)}`
);

const corridorObstacles = [
  { id: 'upper', x: 0, y: 0.70, halfX: 1.15, halfY: 0.25 },
  { id: 'lower', x: 0, y: -0.70, halfX: 1.15, halfY: 0.25 }
];
const corridor = planAgvPath(
  { x: -2, y: 0.18 },
  { x: 2, y: 0.18 },
  {
    bounds: { minX: -2.5, maxX: 2.5, minY: -1.5, maxY: 1.5 },
    obstacles: corridorObstacles,
    footprintClearance: 0.16,
    resolution: 0.05
  }
);
assert(corridor.ok, `双障碍通道规划失败：${corridor.reason}`);
assertSegmentsClear(corridor.path, corridorObstacles, 0.16);
const middle = corridor.rawPath.filter((point) => Math.abs(point.x) < 0.65);
assert(middle.length > 0, '双障碍通道内没有路径采样点');
assert(
  Math.max(...middle.map((point) => Math.abs(point.y))) <= 0.075,
  `路径没有保持在双障碍中间：${JSON.stringify(middle)}`
);

const blocked = planAgvPath(
  { x: -2, y: 0 },
  { x: 0, y: 0 },
  { obstacles: [barrier], footprintClearance: 0.25 }
);
assert(!blocked.ok && blocked.reason === 'goal-blocked', '障碍物内目标未被拒绝');

console.log(
  `path planner ok: dock=${dock.path.length}, detour=${detour.path.length}, corridor=${corridor.path.length}`
);
