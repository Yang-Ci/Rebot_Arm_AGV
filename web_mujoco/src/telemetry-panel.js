const COLORS = ['#5ee0ff', '#8fd7c1', '#f5d76e', '#ff9f5a', '#ff7ab6', '#7aa2ff', '#d4b3ff'];
const MAX_SAMPLES = 150;

function drawSeries(canvas, history, key, minimumRange) {
  const context = canvas?.getContext('2d');
  if (!context) return;
  const width = canvas.width;
  const height = canvas.height;
  context.clearRect(0, 0, width, height);
  context.fillStyle = '#020b18';
  context.fillRect(0, 0, width, height);
  context.strokeStyle = 'rgba(75, 142, 172, 0.16)';
  context.lineWidth = 1;
  for (let row = 1; row < 4; row += 1) {
    const y = (height * row) / 4;
    context.beginPath();
    context.moveTo(0, y + 0.5);
    context.lineTo(width, y + 0.5);
    context.stroke();
  }
  if (key === 'torque') {
    context.strokeStyle = 'rgba(136, 204, 224, 0.28)';
    context.beginPath();
    context.moveTo(0, height / 2 + 0.5);
    context.lineTo(width, height / 2 + 0.5);
    context.stroke();
  }
  if (history.length < 2) return;
  let range = minimumRange;
  for (const sample of history) {
    for (const joint of sample) range = Math.max(range, Math.abs(joint[key]));
  }
  const seriesCount = history[0].length;
  for (let jointIndex = 0; jointIndex < seriesCount; jointIndex += 1) {
    context.strokeStyle = COLORS[jointIndex] || '#ffffff';
    context.lineWidth = jointIndex === seriesCount - 1 ? 1.35 : 1;
    context.beginPath();
    history.forEach((sample, sampleIndex) => {
      const x = (sampleIndex / (MAX_SAMPLES - 1)) * width;
      const normalized = sample[jointIndex][key] / range;
      const y = key === 'torque'
        ? height * 0.5 - normalized * height * 0.43
        : height - Math.abs(normalized) * height * 0.86 - 4;
      if (sampleIndex === 0) context.moveTo(x, y);
      else context.lineTo(x, y);
    });
    context.stroke();
  }
}

export function createTelemetryPanel(root) {
  const contactCountEl = root.querySelector('#contact-count');
  const graspForceEl = root.querySelector('#grasp-force');
  const rmsErrorEl = root.querySelector('#rms-error');
  const barsEl = root.querySelector('#torque-bars');
  const torqueCanvas = root.querySelector('#torque-chart');
  const errorCanvas = root.querySelector('#error-chart');
  const history = [];
  let rows = [];

  function buildRows(joints) {
    barsEl.replaceChildren(...joints.map((joint, index) => {
      const row = document.createElement('div');
      row.className = 'torque-row';
      row.innerHTML = `
        <span>${index < 6 ? `J${index + 1}` : 'Grip'}</span>
        <span class="torque-track"><i class="torque-fill"></i></span>
        <span class="torque-value">0.00 N·m</span>`;
      row.style.setProperty('--joint-color', COLORS[index]);
      return row;
    }));
    rows = [...barsEl.querySelectorAll('.torque-row')].map((row) => ({
      row,
      fill: row.querySelector('.torque-fill'),
      value: row.querySelector('.torque-value')
    }));
  }

  function update(physicsSample, contactSample) {
    if (!rows.length) buildRows(physicsSample.joints);
    contactCountEl.textContent = String(contactSample.activeContacts);
    graspForceEl.textContent = `${contactSample.graspForce.toFixed(1)} N`;
    rmsErrorEl.textContent = `${(physicsSample.rmsError * 1000).toFixed(1)} mrad`;

    physicsSample.joints.forEach((joint, index) => {
      const elements = rows[index];
      if (!elements) return;
      const ratio = Math.min(1, Math.abs(joint.torque) / Math.max(1e-6, joint.torqueLimit));
      elements.fill.style.width = `${ratio * 50}%`;
      elements.fill.classList.toggle('is-negative', joint.torque < 0);
      elements.value.textContent = `${joint.torque.toFixed(2)} N·m`;
    });

    history.push(physicsSample.joints.map((joint) => ({ torque: joint.torque, error: joint.error })));
    if (history.length > MAX_SAMPLES) history.shift();
    drawSeries(torqueCanvas, history, 'torque', 1);
    drawSeries(errorCanvas, history, 'error', 0.004);
  }

  function reset() {
    history.length = 0;
    rows = [];
    barsEl.replaceChildren();
    contactCountEl.textContent = '0';
    graspForceEl.textContent = '0.0 N';
    rmsErrorEl.textContent = '0.0 mrad';
    drawSeries(torqueCanvas, history, 'torque', 1);
    drawSeries(errorCanvas, history, 'error', 0.004);
  }

  reset();
  return { update, reset };
}
