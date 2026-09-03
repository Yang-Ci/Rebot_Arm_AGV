import * as THREE from 'three';

const AXES = ['x', 'y', 'z'];
const EPSILON = 1e-6;

export function boxesOverlap(a, b, clearance = 0) {
  const gap = clearance * 0.5;
  return (
    a.min.x - gap < b.max.x + gap && a.max.x + gap > b.min.x - gap &&
    a.min.y - gap < b.max.y + gap && a.max.y + gap > b.min.y - gap &&
    a.min.z - gap < b.max.z + gap && a.max.z + gap > b.min.z - gap
  );
}

function overlapOnAxis(a, b, axis, clearance) {
  return Math.min(a.max[axis], b.max[axis]) - Math.max(a.min[axis], b.min[axis]) + clearance;
}

function moveEntry(entry, axis, amount) {
  entry.offset[axis] += amount;
  entry.box.min[axis] += amount;
  entry.box.max[axis] += amount;
}

function clampToGround(entry, groundZ) {
  if (entry.box.min.z >= groundZ) return;
  moveEntry(entry, 'z', groundZ - entry.box.min.z);
}

function separatePair(a, b, clearance, groundZ, centerA, centerB) {
  const overlaps = AXES.map((axis) => overlapOnAxis(a.box, b.box, axis, clearance));
  if (overlaps.some((amount) => amount <= 0)) return false;

  let axisIndex = 0;
  if (overlaps[1] < overlaps[axisIndex]) axisIndex = 1;
  if (overlaps[2] < overlaps[axisIndex]) axisIndex = 2;
  const axis = AXES[axisIndex];
  a.box.getCenter(centerA);
  b.box.getCenter(centerB);
  const tieBreak = String(a.id).localeCompare(String(b.id)) <= 0 ? -1 : 1;
  const sign = Math.abs(centerA[axis] - centerB[axis]) < EPSILON
    ? tieBreak
    : (centerA[axis] < centerB[axis] ? -1 : 1);
  const shift = overlaps[axisIndex] * 0.5 + EPSILON;
  moveEntry(a, axis, sign * shift);
  moveEntry(b, axis, -sign * shift);
  clampToGround(a, groundZ);
  clampToGround(b, groundZ);
  return true;
}

function firstOverlap(entry, settled, clearance) {
  return settled.find((other) => boxesOverlap(entry.box, other.box, clearance)) || null;
}

/**
 * Mutates each entry's Box3 and Vector3 offset until every padded AABB is
 * disjoint. The pair solver keeps the radial layout compact; the ordered pass
 * is a deterministic guarantee for pathological/interlocking source bounds.
 */
export function resolveExplosionLayout(entries, {
  clearance = 0.012,
  groundZ = 0,
  iterations = 96
} = {}) {
  const centerA = new THREE.Vector3();
  const centerB = new THREE.Vector3();

  entries.forEach((entry) => clampToGround(entry, groundZ));
  for (let iteration = 0; iteration < iterations; iteration += 1) {
    let collisions = 0;
    for (let i = 0; i < entries.length; i += 1) {
      for (let j = i + 1; j < entries.length; j += 1) {
        if (separatePair(entries[i], entries[j], clearance, groundZ, centerA, centerB)) {
          collisions += 1;
        }
      }
    }
    if (collisions === 0) break;
  }

  // Resolve any stubborn cyclic overlap one item at a time. Previously placed
  // boxes never move, so the invariant stays true for the complete prefix.
  const settled = [];
  entries.forEach((entry, index) => {
    const escape = entry.escapeDirection?.clone() || new THREE.Vector3();
    escape.z = Math.abs(escape.z) + 0.18;
    if (escape.lengthSq() < EPSILON) {
      const phase = index * 2.399963229728653;
      escape.set(Math.cos(phase), Math.sin(phase), 0.35);
    }
    escape.normalize();

    let guard = 0;
    while (firstOverlap(entry, settled, clearance) && guard < 512) {
      const step = Math.max(0.01, clearance);
      AXES.forEach((axis) => moveEntry(entry, axis, escape[axis] * step));
      clampToGround(entry, groundZ);
      guard += 1;
    }

    // A vertical shelf is an absolute fallback if the preferred ray was nearly
    // tangent to a large box for too long.
    if (firstOverlap(entry, settled, clearance)) {
      const highest = settled.reduce((top, other) => Math.max(top, other.box.max.z), groundZ);
      moveEntry(entry, 'z', highest + clearance - entry.box.min.z);
    }
    settled.push(entry);
  });

  return entries;
}

