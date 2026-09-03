import assert from 'node:assert/strict';
import * as THREE from 'three';
import { boxesOverlap, resolveExplosionLayout } from '../src/explosion-layout.js';

const clearance = 0.014;
const entries = Array.from({ length: 24 }, (_, index) => {
  const phase = index * 2.399963229728653;
  const size = new THREE.Vector3(
    0.08 + (index % 4) * 0.035,
    0.06 + (index % 3) * 0.04,
    0.05 + (index % 5) * 0.025
  );
  const center = new THREE.Vector3(
    (index % 3) * 0.012,
    (index % 4) * -0.01,
    size.z * 0.5
  );
  const offset = new THREE.Vector3(Math.cos(phase), Math.sin(phase), 0.2)
    .normalize()
    .multiplyScalar(0.06);
  return {
    id: index,
    box: new THREE.Box3().setFromCenterAndSize(center, size).translate(offset),
    offset,
    escapeDirection: offset.clone().normalize()
  };
});

resolveExplosionLayout(entries, { clearance, groundZ: 0.003 });

entries.forEach((entry) => {
  assert(entry.box.min.z >= 0.003 - 1e-9, `part ${entry.id} fell through the ground`);
});
for (let i = 0; i < entries.length; i += 1) {
  for (let j = i + 1; j < entries.length; j += 1) {
    assert(
      !boxesOverlap(entries[i].box, entries[j].box, clearance),
      `parts ${entries[i].id} and ${entries[j].id} still overlap`
    );
  }
}

console.log(`Explosion layout: ${entries.length} padded AABBs resolved without overlap.`);

