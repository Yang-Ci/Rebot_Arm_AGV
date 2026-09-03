import { t, onLangChange } from './i18n.js';

export function createPartExplorer(root, view) {
  const selectEl = root.querySelector('#part-select');
  const emptyEl = root.querySelector('#part-empty');
  const detailsEl = root.querySelector('#part-details');
  const nameEl = root.querySelector('#part-name');
  const kindEl = root.querySelector('#part-kind');
  const bodyEl = root.querySelector('#part-body');
  const jointEl = root.querySelector('#part-joint');
  const materialEl = root.querySelector('#part-material');
  const descriptionEl = root.querySelector('#part-description');
  const isolateEl = root.querySelector('#part-isolate');
  const clearEl = root.querySelector('#part-clear');
  const catalog = view.getParts();
  let selected = null;
  let isolated = false;

  function populateSelect() {
    const current = selected?.id || '';
    selectEl.replaceChildren(
      Object.assign(document.createElement('option'), {
        value: '',
        textContent: t('part.selectPlaceholder')
      }),
      ...catalog.map((part) => Object.assign(document.createElement('option'), {
        value: part.id,
        textContent: `${part.label} · ${part.body}`
      }))
    );
    selectEl.value = current;
  }

  function render() {
    populateSelect();
    emptyEl.hidden = Boolean(selected);
    detailsEl.hidden = !selected;
    isolateEl.disabled = !selected;
    clearEl.disabled = !selected;
    isolateEl.classList.toggle('active', isolated);
    isolateEl.textContent = t(isolated ? 'part.showAll' : 'part.isolate');
    clearEl.textContent = t('part.clear');
    if (!selected) return;
    nameEl.textContent = selected.label;
    kindEl.textContent = t(`part.kind.${selected.kind}`);
    bodyEl.textContent = selected.body;
    jointEl.textContent = selected.joint || t('part.none');
    materialEl.textContent = selected.material || t('part.none');
    descriptionEl.textContent = t(`part.desc.${selected.kind}`, {
      body: selected.body,
      joint: selected.joint || t('part.none')
    });
  }

  selectEl.addEventListener('change', () => {
    if (!selectEl.value) {
      view.clearPartSelection();
      return;
    }
    view.selectPart(selectEl.value);
  });
  isolateEl.addEventListener('click', () => view.setPartIsolated(!isolated));
  clearEl.addEventListener('click', () => view.clearPartSelection());
  view.onPartSelectionChange((state) => {
    selected = state.part;
    isolated = state.isolated;
    render();
  });
  onLangChange(render);
  render();

  return { applyLang: render };
}

