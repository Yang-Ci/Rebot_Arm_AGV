import { t } from './i18n.js';

const PAD = 8;
const GAP = 8;

function formatFixed(value, digits) {
  const factor = 10 ** digits;
  let rounded = Math.round(value * factor) / factor;
  if (Math.abs(rounded) < 0.5 / factor) rounded = 0;
  return rounded.toFixed(digits);
}

function format(joint, amount) {
  if (joint.unit === 'm') return `${formatFixed(amount * 1000, 1)} mm`;
  return `${formatFixed((amount * 180) / Math.PI, 1)}°`;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function packColumn(items, height) {
  if (!items.length) return;
  const content = items.reduce((sum, entry) => sum + entry.h, 0);
  const gaps = Math.max(0, items.length - 1);
  const available = Math.max(1, height - PAD * 2);
  const gap = gaps ? Math.max(0, Math.min(GAP, (available - content) / gaps)) : 0;
  const blockH = content + gap * gaps;
  let start = clamp((height - blockH) / 2, PAD, Math.max(PAD, height - PAD - blockH));
  items.forEach((entry) => {
    entry.chipY = start;
    start += entry.h + gap;
  });
}

export function createJointCallouts(root, joints, onChange) {
  root.innerHTML = '';
  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  svg.classList.add('callout-lines');
  svg.setAttribute('aria-hidden', 'true');
  root.append(svg);

  const chips = {};
  let placedWidth = -1;
  let placedHeight = -1;
  let showAllGuides = false;

  joints.visible.forEach((joint) => {
    const color = joint.color || '#8fd7c1';
    const chip = document.createElement('div');
    chip.className = 'callout-chip is-hidden';
    chip.dataset.joint = joint.name;
    chip.style.setProperty('--callout-color', color);

    const header = document.createElement('div');
    header.className = 'callout-header';
    const name = document.createElement('span');
    name.className = 'callout-name';
    name.textContent = joint.name === 'joint7' ? t('joint.gripper') : joint.label;
    const value = document.createElement('strong');
    const actualEl = document.createElement('span');
    actualEl.className = 'actual';
    const targetEl = document.createElement('span');
    targetEl.className = 'target-readout';
    value.append(actualEl, targetEl);
    header.append(name, value);

    const body = document.createElement('div');
    body.className = 'callout-body';
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = String(joint.min);
    slider.max = String(joint.max);
    slider.step = joint.unit === 'm' ? '0.0005' : '0.01';
    slider.value = String(joint.home);
    slider.style.accentColor = color;
    slider.addEventListener('input', () => {
      onChange(joint.name, Number(slider.value));
      targetEl.textContent = format(joint, Number(slider.value));
    });
    body.append(slider);
    chip.addEventListener('pointerdown', (event) => event.stopPropagation());
    chip.addEventListener('mouseenter', () => {
      if (isTcpBusy()) return;
      if (!window.matchMedia('(hover: hover) and (pointer: fine)').matches) return;
      setOpen(chip, true);
    });
    chip.addEventListener('mouseleave', () => {
      if (!window.matchMedia('(hover: hover) and (pointer: fine)').matches) return;
      setOpen(chip, false);
    });
    chip.addEventListener('click', (event) => {
      if (event.target.closest('input')) return;
      if (isTcpBusy()) return;
      if (window.matchMedia('(hover: hover) and (pointer: fine)').matches) return;
      setOpen(chip, !chip.classList.contains('is-open'));
    });
    chip.append(header, body);
    root.append(chip);

    const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
    line.setAttribute('class', 'callout-line');
    line.style.setProperty('--callout-color', color);
    line.style.stroke = color;
    line.style.fill = 'none';
    const dot = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
    dot.setAttribute('class', 'callout-dot');
    dot.setAttribute('r', '3.5');
    dot.style.setProperty('--callout-color', color);
    dot.style.fill = color;
    svg.append(line, dot);

    chips[joint.name] = {
      chip,
      slider,
      actualEl,
      targetEl,
      joint,
      line,
      dot,
      color,
      side: joint.column || 'right',
      attachX: 0,
      attachY: 0
    };
  });

  function invalidateLayout() {
    placedWidth = -1;
  }

  function isTcpBusy() {
    const host = root.parentElement;
    return Boolean(host && host.classList.contains('tcp-dragging'));
  }

  function setOpen(chip, open) {
    root.querySelectorAll('.callout-chip.is-open').forEach((el) => {
      if (el !== chip) el.classList.remove('is-open');
    });
    chip.classList.toggle('is-open', open);
    invalidateLayout();
  }

  function closeOpenChips() {
    let changed = false;
    root.querySelectorAll('.callout-chip.is-open').forEach((el) => {
      el.classList.remove('is-open');
      changed = true;
    });
    if (changed) invalidateLayout();
  }

  const host = root.parentElement;
  if (host) {
    host.addEventListener('pointerdown', (event) => {
      if (!event.target.closest('.callout-chip')) closeOpenChips();
    });
  }

  function writeTarget(item, amount) {
    item.slider.value = String(amount);
    item.targetEl.textContent = format(item.joint, amount);
  }

  function writeActual(item, amount) {
    item.actualEl.textContent = format(item.joint, amount);
  }

  function placeCards() {
    const width = Math.max(1, root.clientWidth);
    const height = Math.max(1, root.clientHeight);
    if (width === placedWidth && height === placedHeight) return;
    placedWidth = width;
    placedHeight = height;

    const left = [];
    const right = [];
    joints.visible.forEach((joint) => {
      const item = chips[joint.name];
      item.chip.classList.remove('is-hidden');
      const w = Math.max(1, item.chip.offsetWidth);
      const h = Math.max(1, item.chip.offsetHeight);
      const twoColumns = width >= 280;
      const side = twoColumns ? item.side : 'right';
      item.chip.dataset.side = side;
      const entry = { item, w, h, side };
      (side === 'right' ? right : left).push(entry);
    });

    left.reverse();
    packColumn(left, height);
    packColumn(right, height);

    [...left, ...right].forEach((entry) => {
      const { item, chipY, side } = entry;
      item.chip.style.left = side === 'right' ? 'auto' : `${PAD}px`;
      item.chip.style.right = side === 'right' ? `${PAD}px` : 'auto';
      item.chip.style.transform = `translateY(${chipY}px)`;
    });
  }

  function attachPoint(item) {
    const rootRect = root.getBoundingClientRect();
    const chipRect = item.chip.getBoundingClientRect();
    const side = item.chip.dataset.side || item.side;
    return {
      x: side === 'right' ? chipRect.left - rootRect.left : chipRect.right - rootRect.left,
      y: chipRect.top - rootRect.top + chipRect.height / 2
    };
  }

  function layout(projectWorld, data) {
    placeCards();
    joints.visible.forEach((joint) => {
      const item = chips[joint.name];
      const screen = projectWorld(
        data.xanchor[joint.id * 3],
        data.xanchor[joint.id * 3 + 1],
        data.xanchor[joint.id * 3 + 2]
      );
      if (!screen.visible) {
        item.line.setAttribute('visibility', 'hidden');
        item.dot.setAttribute('visibility', 'hidden');
        return;
      }
      const focused = item.chip.classList.contains('is-open') || item.chip.matches(':hover, :focus-within');
      const showGuide = showAllGuides || focused;
      item.line.setAttribute('visibility', showGuide ? 'visible' : 'hidden');
      item.dot.setAttribute('visibility', showGuide ? 'visible' : 'hidden');
      const attach = attachPoint(item);
      item.attachX = attach.x;
      item.attachY = attach.y;
      item.line.setAttribute('x1', String(screen.x));
      item.line.setAttribute('y1', String(screen.y));
      item.line.setAttribute('x2', String(attach.x));
      item.line.setAttribute('y2', String(attach.y));
      item.dot.setAttribute('cx', String(screen.x));
      item.dot.setAttribute('cy', String(screen.y));
    });
  }

  return {
    layout,
    closeChips: closeOpenChips,
    setGuidesVisible(visible) {
      showAllGuides = Boolean(visible);
    },
    applyLang() {
      const gripper = chips.joint7;
      if (gripper) {
        gripper.chip.querySelector('.callout-name').textContent = t('joint.gripper');
        placedWidth = -1;
      }
    },
    setTarget(name, amount) {
      const item = chips[name];
      if (!item) return;
      writeTarget(item, amount);
    },
    setActual(name, amount) {
      const item = chips[name];
      if (!item) return;
      writeActual(item, amount);
    }
  };
}
