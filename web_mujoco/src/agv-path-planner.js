export const AGV_NAV_BOUNDS = Object.freeze({
  minX: -3.65,
  maxX: 3.65,
  minY: -2.65,
  maxY: 2.65
});

export const AGV_FOOTPRINT_CLEARANCE = 0.30;

// Axis-aligned footprints from the generated laboratory scene. The task table
// is intentionally absent: its raised underside clears the chassis at DOCK.
export const LAB_NAV_OBSTACLES = Object.freeze([
  { id: 'storage-partition', x: -1.30, y: 1.80, halfX: 0.06, halfY: 0.90 },
  { id: 'north-partition', x: 0.70, y: 1.00, halfX: 1.00, halfY: 0.06 },
  { id: 'south-partition', x: -0.80, y: -1.60, halfX: 1.20, halfY: 0.06 },
  { id: 'delivery-partition', x: 1.40, y: -1.80, halfX: 0.06, halfY: 0.90 },
  { id: 'storage-rack', x: -3.00, y: 1.75, halfX: 0.60, halfY: 0.55 },
  { id: 'dynamic-pallet', x: 0, y: 1.72, halfX: 0.35, halfY: 0.225 }
]);

const SQRT2 = Math.sqrt(2);
const DIRECTIONS = [
  [1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1],
  [1, 1, SQRT2], [1, -1, SQRT2], [-1, 1, SQRT2], [-1, -1, SQRT2]
];

function distanceToRectangle(point, obstacle) {
  const dx = Math.max(Math.abs(point.x - obstacle.x) - obstacle.halfX, 0);
  const dy = Math.max(Math.abs(point.y - obstacle.y) - obstacle.halfY, 0);
  return Math.hypot(dx, dy);
}

function clearanceAt(point, obstacles, bounds) {
  let clearance = Math.min(
    point.x - bounds.minX,
    bounds.maxX - point.x,
    point.y - bounds.minY,
    bounds.maxY - point.y
  );
  for (const obstacle of obstacles) {
    clearance = Math.min(clearance, distanceToRectangle(point, obstacle));
  }
  return clearance;
}

function isFree(point, obstacles, bounds, footprintClearance) {
  if (
    point.x < bounds.minX || point.x > bounds.maxX ||
    point.y < bounds.minY || point.y > bounds.maxY
  ) return false;
  return obstacles.every(
    (obstacle) => distanceToRectangle(point, obstacle) >= footprintClearance
  );
}

class MinHeap {
  constructor() {
    this.items = [];
  }

  push(item) {
    this.items.push(item);
    let index = this.items.length - 1;
    while (index > 0) {
      const parent = (index - 1) >> 1;
      if (this.items[parent].priority <= item.priority) break;
      this.items[index] = this.items[parent];
      index = parent;
    }
    this.items[index] = item;
  }

  pop() {
    if (!this.items.length) return null;
    const first = this.items[0];
    const tail = this.items.pop();
    if (this.items.length && tail) {
      let index = 0;
      while (true) {
        const left = index * 2 + 1;
        const right = left + 1;
        if (left >= this.items.length) break;
        const child = right < this.items.length && this.items[right].priority < this.items[left].priority
          ? right
          : left;
        if (this.items[child].priority >= tail.priority) break;
        this.items[index] = this.items[child];
        index = child;
      }
      this.items[index] = tail;
    }
    return first;
  }
}

function segmentClearance(a, b, obstacles, bounds, footprintClearance, resolution) {
  const distance = Math.hypot(b.x - a.x, b.y - a.y);
  const steps = Math.max(1, Math.ceil(distance / (resolution * 0.45)));
  let minimum = Infinity;
  for (let index = 0; index <= steps; index += 1) {
    const ratio = index / steps;
    const point = {
      x: a.x + (b.x - a.x) * ratio,
      y: a.y + (b.y - a.y) * ratio
    };
    if (!isFree(point, obstacles, bounds, footprintClearance)) return -Infinity;
    minimum = Math.min(minimum, clearanceAt(point, obstacles, bounds));
  }
  return minimum;
}

function smoothPath(path, obstacles, bounds, footprintClearance, resolution) {
  if (path.length <= 2) return path;
  const result = [path[0]];
  let start = 0;
  while (start < path.length - 1) {
    let selected = start + 1;
    let originalMinimum = Infinity;
    for (let candidate = start + 1; candidate < path.length; candidate += 1) {
      originalMinimum = Math.min(
        originalMinimum,
        clearanceAt(path[candidate], obstacles, bounds)
      );
      const directMinimum = segmentClearance(
        path[start], path[candidate], obstacles, bounds, footprintClearance, resolution
      );
      // Do not let smoothing undo A*'s preference for the middle of a passage.
      if (directMinimum + resolution * 0.35 >= originalMinimum) selected = candidate;
    }
    result.push(path[selected]);
    start = selected;
  }
  return result;
}

export function planAgvPath(start, goal, options = {}) {
  const resolution = options.resolution || 0.10;
  const footprintClearance = options.footprintClearance || AGV_FOOTPRINT_CLEARANCE;
  const bounds = options.bounds || AGV_NAV_BOUNDS;
  const obstacles = options.obstacles || LAB_NAV_OBSTACLES;
  if (!isFree(start, obstacles, bounds, footprintClearance)) {
    return { ok: false, reason: 'start-blocked', path: [] };
  }
  if (!isFree(goal, obstacles, bounds, footprintClearance)) {
    return { ok: false, reason: 'goal-blocked', path: [] };
  }

  const columns = Math.floor((bounds.maxX - bounds.minX) / resolution) + 1;
  const rows = Math.floor((bounds.maxY - bounds.minY) / resolution) + 1;
  const toCell = (point) => ({
    x: Math.max(0, Math.min(columns - 1, Math.round((point.x - bounds.minX) / resolution))),
    y: Math.max(0, Math.min(rows - 1, Math.round((point.y - bounds.minY) / resolution)))
  });
  const toPoint = (x, y) => ({
    x: bounds.minX + x * resolution,
    y: bounds.minY + y * resolution
  });
  const cellIndex = (x, y) => y * columns + x;
  const startCell = toCell(start);
  const goalCell = toCell(goal);
  const startIndex = cellIndex(startCell.x, startCell.y);
  const goalIndex = cellIndex(goalCell.x, goalCell.y);
  const count = columns * rows;
  const costs = new Float64Array(count);
  costs.fill(Infinity);
  const parents = new Int32Array(count);
  parents.fill(-1);
  const closed = new Uint8Array(count);
  const open = new MinHeap();
  costs[startIndex] = 0;
  open.push({ index: startIndex, x: startCell.x, y: startCell.y, priority: 0 });

  while (open.items.length) {
    const current = open.pop();
    if (!current || closed[current.index]) continue;
    closed[current.index] = 1;
    if (current.index === goalIndex) break;
    for (const [dx, dy, scale] of DIRECTIONS) {
      const x = current.x + dx;
      const y = current.y + dy;
      if (x < 0 || x >= columns || y < 0 || y >= rows) continue;
      const index = cellIndex(x, y);
      if (closed[index]) continue;
      const point = toPoint(x, y);
      if (!isFree(point, obstacles, bounds, footprintClearance)) continue;
      if (dx && dy) {
        if (
          !isFree(toPoint(current.x + dx, current.y), obstacles, bounds, footprintClearance) ||
          !isFree(toPoint(current.x, current.y + dy), obstacles, bounds, footprintClearance)
        ) continue;
      }
      const clearance = Math.max(0.04, clearanceAt(point, obstacles, bounds));
      const centerPenalty = 0.055 / (clearance * clearance);
      const nextCost = costs[current.index] + resolution * scale * (1 + centerPenalty);
      if (nextCost >= costs[index]) continue;
      costs[index] = nextCost;
      parents[index] = current.index;
      const heuristic = Math.hypot(goalCell.x - x, goalCell.y - y) * resolution;
      open.push({ index, x, y, priority: nextCost + heuristic });
    }
  }

  if (!Number.isFinite(costs[goalIndex])) {
    return { ok: false, reason: 'unreachable', path: [] };
  }
  const reversed = [];
  let cursor = goalIndex;
  while (cursor >= 0) {
    const x = cursor % columns;
    const y = Math.floor(cursor / columns);
    reversed.push(toPoint(x, y));
    if (cursor === startIndex) break;
    cursor = parents[cursor];
  }
  const rawPath = reversed.reverse();
  rawPath[0] = { x: start.x, y: start.y };
  rawPath[rawPath.length - 1] = { x: goal.x, y: goal.y };
  const path = smoothPath(
    rawPath, obstacles, bounds, footprintClearance, resolution
  );
  if (Number.isFinite(goal.yaw)) path[path.length - 1].yaw = goal.yaw;
  return { ok: true, reason: 'planned', path, rawPath, cost: costs[goalIndex] };
}
