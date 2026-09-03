import assert from 'node:assert/strict';
import {
  PRESENTATION_SHARE,
  evaluateExplosionTimeline
} from '../src/explosion-timeline.js';

const stageCount = 8;
const atStart = evaluateExplosionTimeline(0, stageCount);
assert.equal(atStart.presentationAmount, 0);
assert.equal(atStart.explosionAmount, 0);
assert.equal(atStart.stageAmount(0), 0);

const afterPresentation = evaluateExplosionTimeline(PRESENTATION_SHARE, stageCount);
assert.equal(afterPresentation.presentationAmount, 1);
assert.equal(afterPresentation.explosionAmount, 0);

for (let stage = 0; stage < stageCount; stage += 1) {
  const explosionPhase = (stage + 0.5) / stageCount;
  const progress = PRESENTATION_SHARE + explosionPhase * (1 - PRESENTATION_SHARE);
  const state = evaluateExplosionTimeline(progress, stageCount);
  assert.equal(state.activeStage, stage);
  assert(state.stageAmount(stage) > 0 && state.stageAmount(stage) < 1);
  if (stage > 0) assert.equal(state.stageAmount(stage - 1), 1);
  if (stage < stageCount - 1) assert.equal(state.stageAmount(stage + 1), 0);
}

let previous = Array(stageCount).fill(0);
for (let step = 0; step <= 200; step += 1) {
  const state = evaluateExplosionTimeline(step / 200, stageCount);
  const current = previous.map((_, stage) => state.stageAmount(stage));
  current.forEach((amount, stage) => {
    assert(amount >= previous[stage], `stage ${stage} moved backwards at step ${step}`);
    assert(amount >= 0 && amount <= 1, `stage ${stage} left the normalized range`);
  });
  previous = current;
}

const atEnd = evaluateExplosionTimeline(1, stageCount);
assert.equal(atEnd.presentationAmount, 1);
assert.equal(atEnd.explosionAmount, 1);
for (let stage = 0; stage < stageCount; stage += 1) {
  assert.equal(atEnd.stageAmount(stage), 1);
}

console.log(`Explosion timeline: presentation plus ${stageCount} ordered stages verified.`);
