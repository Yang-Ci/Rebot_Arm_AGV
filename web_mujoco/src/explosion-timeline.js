export const PRESENTATION_SHARE = 0.24;
export const EXPLOSION_TIMELINE_DURATION_MS = 6200;

function clamp01(value) {
  return Math.max(0, Math.min(1, Number(value) || 0));
}

export function smoothStep(value) {
  const amount = clamp01(value);
  return amount * amount * (3 - 2 * amount);
}

export function evaluateExplosionTimeline(progress, stageCount) {
  const normalized = clamp01(progress);
  const count = Math.max(1, Math.trunc(stageCount) || 1);
  const presentationAmount = smoothStep(normalized / PRESENTATION_SHARE);
  const explosionAmount = clamp01(
    (normalized - PRESENTATION_SHARE) / (1 - PRESENTATION_SHARE)
  );
  const scaledStage = explosionAmount * count;
  const activeStage = explosionAmount >= 1
    ? count - 1
    : Math.min(count - 1, Math.max(0, Math.floor(scaledStage)));

  return {
    progress: normalized,
    presentationAmount,
    explosionAmount,
    activeStage,
    stageAmount(stageIndex) {
      return smoothStep(scaledStage - stageIndex);
    }
  };
}

