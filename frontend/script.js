/* ===========================================================
   ROVER-X CONTROL DECK — logic
   Talks to the ESP32 over HTTP. Adjust ROVER_BASE_URL if your
   ESP32 web server isn't on the same origin as this page.
   =========================================================== */

const ROVER_BASE_URL = ''; // e.g. 'http://192.168.4.1' if hosted separately
const GAUGE_CIRCUMFERENCE = 578; // 2 * PI * r(92), matches styles.css

// ---- state ----
let mode = 'manual';        // 'manual' | 'auto'
let running = false;        // engine started?
let speed = 0;               // 0-100
let telemetryInterval = null;

// ---- element refs ----
const statusDot = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');
const phaseChip = document.getElementById('phaseChip');

const gaugeFill = document.getElementById('gaugeFill');
const speedValue = document.getElementById('speedValue');
const speedSlider = document.getElementById('speedSlider');

const btnManual = document.getElementById('btnManual');
const btnAuto = document.getElementById('btnAuto');
const modeSlider = document.getElementById('modeSlider');
const autoPanel = document.getElementById('autoPanel');

const engineBtn = document.getElementById('engineBtn');
const engineLabel = document.getElementById('engineLabel');

const lineVal = document.getElementById('lineVal');
const distVal = document.getElementById('distVal');

/* ===========================================================
   NETWORK — replace the fetch paths with your ESP32 routes
   =========================================================== */
async function sendCommand(path, params = {}) {
  const query = new URLSearchParams(params).toString();
  const url = `${ROVER_BASE_URL}/${path}${query ? '?' + query : ''}`;
  try {
    const res = await fetch(url, { method: 'GET' });
    setConnection(res.ok ? 'online' : 'offline');
    return res.ok ? res.json().catch(() => ({})) : null;
  } catch (err) {
    setConnection('offline');
    return null;
  }
}

async function pollTelemetry() {
  const data = await sendCommand('telemetry');
  if (data) {
    if (typeof data.lineError !== 'undefined') lineVal.textContent = data.lineError;
    if (typeof data.distance !== 'undefined') distVal.textContent = `${data.distance} cm`;
  }
}

function setConnection(state) {
  statusDot.classList.remove('online', 'offline');
  if (state === 'online') {
    statusDot.classList.add('online');
    statusText.textContent = 'LINKED';
  } else if (state === 'offline') {
    statusDot.classList.add('offline');
    statusText.textContent = 'NO SIGNAL';
  } else {
    statusText.textContent = 'LINKING…';
  }
}

/* ===========================================================
   GAUGE / SPEED
   =========================================================== */
function setSpeed(value) {
  speed = Math.max(0, Math.min(100, Number(value)));
  speedValue.textContent = speed;
  speedSlider.value = speed;
  speedSlider.style.setProperty('--fill', `${speed}%`);

  const offset = GAUGE_CIRCUMFERENCE - (speed / 100) * GAUGE_CIRCUMFERENCE;
  gaugeFill.style.strokeDashoffset = offset;

  // colour ramps from cyan -> amber -> red as throttle climbs
  let color = 'var(--cyan)';
  if (speed > 80) color = 'var(--red)';
  else if (speed > 50) color = 'var(--amber)';
  gaugeFill.style.stroke = color;

  if (running) sendCommand('speed', { value: speed });
}

speedSlider.addEventListener('input', (e) => setSpeed(e.target.value));

/* ===========================================================
   MODE TOGGLE
   =========================================================== */
function setMode(newMode) {
  mode = newMode;
  const isAuto = mode === 'auto';

  btnManual.classList.toggle('active', !isAuto);
  btnAuto.classList.toggle('active', isAuto);
  modeSlider.classList.toggle('pos-auto', isAuto);

  autoPanel.classList.toggle('hidden-panel', !isAuto);

  phaseChip.textContent = isAuto
    ? 'PHASE 2/3 · AUTONOMOUS'
    : 'PHASE 1 · MANUAL OVERRIDE';

  clearInterval(telemetryInterval);
  if (isAuto && running) {
    telemetryInterval = setInterval(pollTelemetry, 500);
  }

  sendCommand('mode', { value: mode });
}

btnManual.addEventListener('click', () => setMode('manual'));
btnAuto.addEventListener('click', () => setMode('auto'));

/* ===========================================================
   ENGINE START / STOP
   =========================================================== */
function setRunning(state) {
  running = state;
  engineBtn.classList.toggle('running', running);
  engineLabel.textContent = running ? 'STOP RUN' : 'START RUN';

  if (running) {
    sendCommand('engine', { value: 'start' });
    if (mode === 'auto') telemetryInterval = setInterval(pollTelemetry, 500);
  } else {
    sendCommand('engine', { value: 'stop' });
    clearInterval(telemetryInterval);
  }
}

engineBtn.addEventListener('click', () => setRunning(!running));

/* ===========================================================
   INIT
   =========================================================== */
setSpeed(0);
setMode('manual');
setConnection('connecting');

// Try an initial handshake with the ESP32 so the status dot is accurate
sendCommand('ping');