export const MODEL_VARIANTS = Object.freeze({
  arm: Object.freeze({
    id: 'arm',
    sceneXml: 'rs_grasp_scene.xml',
    labelKey: 'variant.arm',
    hasAgv: false,
    view: Object.freeze({
      position: [0.85, -0.95, 0.62],
      target: [0.28, 0, 0.16],
      far: 20
    })
  }),
  agv: Object.freeze({
    id: 'agv',
    sceneXml: 'rs_agv_scene.xml',
    labelKey: 'variant.agv',
    hasAgv: true,
    view: Object.freeze({
      position: [5.8, -6.8, 4.8],
      target: [0, 0, 0.35],
      far: 30
    })
  })
});

export function selectedModelVariant(search = window.location.search) {
  const id = new URLSearchParams(search).get('variant');
  return MODEL_VARIANTS[id] || MODEL_VARIANTS.arm;
}

export function switchModelVariant(id) {
  const variant = MODEL_VARIANTS[id] || MODEL_VARIANTS.arm;
  const url = new URL(window.location.href);
  url.searchParams.set('variant', variant.id);
  window.location.assign(url);
}
