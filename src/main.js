import * as THREE from 'three';
import { advance, sample, STEP, ROBOT_RADIUS } from './physics.js';
import { SCENARIOS, solidos } from './world.js';
import { sense, SENSOR } from './sensor.js';
import { gridOf, FREE, OCCUPIED, createMap, seedFootprint, integrate, counts, center, markBump } from './mapping.js';
import { frontiers } from './planner.js';
import { createExplorer, startExplorer, pauseExplorer, exploreStep, setLeash } from './explorer.js';
import { wheelSpeeds } from './motion.js';
import { detectMarkers } from './vision.js';
import { createKeyframeSelector, selectKeyframe } from './keyframes.js';
import { evaluatePerception } from './perception-evaluation.js';
import { fuseInventory } from './inventory.js';
import { createOdometry, readMotion, odomPose, SENSORES_RUIDOSOS } from './odometry.js';
import { integrateFusedMotion, gateMotionCommand, motionFusionSummary } from './motion-fusion.js';
import { mulberry32 } from './noise.js';
import { createMissionMapping, updateMissionMapping, missionSummary } from './mission-mapping.js';
import { exportSlam } from './slam.js';
import { compose, between } from './pose-graph.js';
import './style.css';

const $ = (selector) => document.querySelector(selector);
function setText(selector, value) {
  const node = $(selector);
  if (node.textContent !== value) node.textContent = value;
}
function setHTML(selector, value) {
  const node = $(selector);
  if (node.innerHTML !== value) node.innerHTML = value;
}
const viewport = $('#viewport');
const map = $('#map');
const ctx = map.getContext('2d');
const occupancy = $('#occupancy'), occupancyCtx = occupancy.getContext('2d');
const controls = { forward: false, back: false, left: false, right: false };
const keys = new Set();
const pointers = new Map();
const keyMap = { w: 'forward', arrowup: 'forward', s: 'back', arrowdown: 'back', a: 'left', arrowleft: 'left', d: 'right', arrowright: 'right' };
const ACTIVE_STATES = ['barrido', 'planificando', 'navegando', 'retorno', 'retroceso', 'relocalizando'];
let pose, history, trail, startTime, step, accumulator = 0, lastTime = null, lastMapDraw = -Infinity, renderer, grid, explorer, readings, frontierCells, autoCommand, latestScan, lastApplied = {}, lastBump = false, bumpCount = 0;
let belief, beliefSeed, odomLog, sensorRandom, scanCorrections = 0, scanDrift = 0, lastMed = { avance: 0, giro: 0 }, odomNoisy = $('#odometry-noise')?.checked ?? true;
let scenario = SCENARIOS.partition;
let rebuildWalls = () => {};
let selector, keyframes, inventory, captureActive, captureLimit, captureRequested, evaluationReport, selectedFrameId, frameGroups = [];
let trialWorker = null, batchTrials = [], batchComparison = null, batchScenario = null, batchSelection = 'comparison';
let mission;
let sensorsStarted = false;
let sensorSteps = 0;
function frameMapPose(frame) {
  const anchor = frame.slam_anchor;
  return anchor != null && mission.slam?.nodes[anchor] ? compose(mission.slam.nodes[anchor].pose, frame.slam_local) : frame.pose;
}

function reset() {
  cancelBatch(true);
  stop();
  pose = { x: 0, z: 0, yaw: 0, collision: false };
  const seedInput = $('#sensor-seed');
  if (!seedInput.reportValidity()) seedInput.value = '7';
  beliefSeed = Number(seedInput.value) >>> 0; sensorsStarted = false; sensorSteps = 0;
  belief = createOdometry(pose, odomNoisy ? { ...SENSORES_RUIDOSOS, seed: beliefSeed } : { seed: beliefSeed });
  sensorRandom = mulberry32((beliefSeed ^ 0x51f3) >>> 0);
  scanCorrections = 0; scanDrift = 0; lastMed = { avance: 0, giro: 0 };
  odomLog = [{ paso: 0, x: pose.x, z: pose.z, yaw_rad: pose.yaw }];
  startTime = Date.now(); step = 0; accumulator = 0;
  history = [sample(pose, step, startTime)];
  trail = [{ x: pose.x, z: pose.z }];
  grid = createMap(scenario.mapConfig); seedFootprint(grid, odomPose(belief)); explorer = createExplorer(); readings = []; frontierCells = frontiers(grid); autoCommand = {}; latestScan = null; lastApplied = {}; lastBump = false; bumpCount = 0;
  mission = createMissionMapping(grid, { noisy: odomNoisy, loopClosure: $('#slam-on').checked });
  selector = createKeyframeSelector(); keyframes = []; inventory = []; captureActive = false; captureLimit = false; captureRequested = false; evaluationReport = null; selectedFrameId = null; frameGroups = [];
  syncLeash(); updateUI(); drawMap(); drawOccupancy(); renderGallery();
}
function syncControls() {
  for (const name of Object.keys(controls)) controls[name] = [...keys].some(key => keyMap[key] === name) || [...pointers.values()].includes(name);
  document.querySelectorAll('[data-control]').forEach(button => button.classList.toggle('pressed', controls[button.dataset.control]));
}
function stop() { keys.clear(); pointers.clear(); syncControls(); if (explorer) pauseExplorer(explorer); autoCommand = {}; }
window.addEventListener('keydown', event => {
  const key = event.key.toLowerCase();
  if (event.target.matches('select, input, textarea') || event.target.closest('[role="button"]')) return;
  if (key === ' ') { event.preventDefault(); stop(); return; }
  if (keyMap[key] && !event.ctrlKey && !event.metaKey && !event.altKey) { event.preventDefault(); if (!event.repeat) { captureActive = true; pauseExplorer(explorer); keys.add(key); } syncControls(); }
});
window.addEventListener('keyup', event => { keys.delete(event.key.toLowerCase()); syncControls(); });
window.addEventListener('blur', stop);
document.addEventListener('visibilitychange', () => { if (document.hidden) { stop(); lastTime = null; accumulator = 0; } });
document.querySelectorAll('[data-control]').forEach(button => {
  button.addEventListener('pointerdown', event => { event.preventDefault(); captureActive = true; pauseExplorer(explorer); button.setPointerCapture(event.pointerId); pointers.set(event.pointerId, button.dataset.control); syncControls(); });
  for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) button.addEventListener(type, event => { pointers.delete(event.pointerId); syncControls(); });
  button.addEventListener('keydown', event => { if (event.key === 'Enter') { event.preventDefault(); if (!event.repeat) { captureActive = true; pauseExplorer(explorer); pointers.set('keyboard', button.dataset.control); } syncControls(); } });
  button.addEventListener('keyup', event => { if (event.key === 'Enter') { pointers.delete('keyboard'); syncControls(); } });
  button.addEventListener('blur', () => { pointers.delete('keyboard'); syncControls(); });
});
$('#stop').addEventListener('click', stop);
$('#explore').addEventListener('click', () => { if (ACTIVE_STATES.includes(explorer.state)) stop(); else { captureActive = true; keys.clear(); pointers.clear(); syncControls(); startExplorer(explorer); } updateUI(); });
$('#capture').addEventListener('click', () => { captureActive = true; captureRequested = true; });
$('#frame-select').addEventListener('change', event => selectFrame(Number(event.target.value)));
$('#evaluate-perception').addEventListener('click', () => { const input = $('#min-detail'); if (!input.reportValidity()) return; evaluationReport = evaluatePerception(scenario.boards, keyframes, { minLadoReferencia: Number(input.value) }); renderEvaluation(); });
$('#min-detail').addEventListener('input', () => { evaluationReport = null; renderEvaluation(); });
$('#reset').addEventListener('click', reset);
$('#scenario').addEventListener('change', event => { scenario = SCENARIOS[event.target.value]; rebuildWalls(); reset(); });
$('#odometry-noise').addEventListener('change', event => { odomNoisy = event.target.checked; reset(); });
$('#slam-on').addEventListener('change', reset);
$('#sensor-seed').addEventListener('change', reset);
function syncLeash() {
  if (!$('#leash-on').checked) { setLeash(explorer, null); return; }
  const presupuestoS = Number($('#leash-budget').value), maxDistanciaM = Number($('#leash-dist').value);
  if (presupuestoS > 0 && maxDistanciaM > 0) setLeash(explorer, { presupuestoS, maxDistanciaM });
  else setLeash(explorer, null);
}
$('#leash-on').addEventListener('change', syncLeash);
$('#leash-budget').addEventListener('change', syncLeash);
$('#leash-dist').addEventListener('change', syncLeash);
const comparisonColors = ['#cbd5cf', '#ede3ba', '#f8faf7', '#47665c', '#d36968'];
const trialColors = ['#cbd5cf', '#f8faf7', '#47665c'];
function paintBatchMap(canvas, values, meta, colors) {
  canvas.width = meta.width; canvas.height = meta.height;
  const ctx = canvas.getContext('2d'), data = ctx.createImageData(meta.width, meta.height);
  const rgb = colors.map(color => [parseInt(color.slice(1, 3), 16), parseInt(color.slice(3, 5), 16), parseInt(color.slice(5, 7), 16)]);
  for (let i = 0; i < values.length; i++) { const color = rgb[values[i]] ?? rgb[0]; data.data.set([color[0], color[1], color[2], 255], i * 4); }
  ctx.putImageData(data, 0, 0);
}
function batchControls(running) { $('#batch-start').disabled = running; $('#batch-cancel').disabled = !running; $('#batch-export').disabled = !batchComparison; }
function alignmentLabel(alignment) {
  if (!alignment) return 'Alineación pendiente';
  const labels = { referencia: 'Referencia del lote', correccion_observada: 'Corrección por observaciones', origen_comun_consistente: 'Origen común consistente', origen_comun_alineacion_ambigua: 'Ambigua: se conserva el origen común', origen_comun_sin_evidencia: 'Sin evidencia: se conserva el origen común' };
  return labels[alignment.status] ?? alignment.status;
}
function cancelBatch(clear = false) {
  const wasRunning = Boolean(trialWorker);
  if (trialWorker) { trialWorker.terminate(); trialWorker = null; }
  if (clear) { batchTrials = []; batchComparison = null; batchScenario = null; batchSelection = 'comparison'; renderBatch(); }
  batchControls(false);
  if (wasRunning) setText('#batch-status', clear ? 'Comparación cancelada por reinicio o cambio de escena.' : `Comparación cancelada · ${batchTrials.length}/10 pasadas terminadas.`);
  else if (clear) setText('#batch-status', 'Listo para comparar la escena actual.');
}
function renderBatch() {
  const select = $('#batch-map-select'); select.replaceChildren();
  const option = document.createElement('option'); option.value = 'comparison'; option.textContent = 'Comparación de las 10 pasadas'; select.append(option);
  for (const trial of batchTrials) { const item = document.createElement('option'); item.value = String(trial.id); item.textContent = `Pasada ${trial.id} · ${trial.state}`; select.append(item); }
  select.value = batchSelection;
  const trial = batchTrials.find(item => String(item.id) === batchSelection), target = trial ? { values: trial.map, grid: trial.grid, colors: trialColors } : batchComparison ? { values: batchComparison.classes, grid: batchComparison.grid, colors: comparisonColors } : null;
  $('#batch-legend').hidden = Boolean(trial);
  const canvas = $('#batch-map');
  if (target) paintBatchMap(canvas, target.values, target.grid, target.colors); else { canvas.width = 80; canvas.height = 80; const c = canvas.getContext('2d'); c.fillStyle = '#e9eeeb'; c.fillRect(0, 0, 80, 80); }
  setText('#batch-cell', 'Selecciona una celda para ver sus votos.');
  const summary = batchComparison ? Object.entries(batchComparison.stats ?? {}).map(([name, value]) => `${name.replaceAll('_', ' ')}: ${typeof value === 'number' ? Number(value.toFixed(2)) : String(value)}`).join(' · ') : 'El consenso compara observaciones; no es la verdad del entorno.';
  const alignment = trial && batchComparison?.alignments?.find(item => item.id === trial.id);
  setText('#batch-summary', trial ? `Pasada individual: gris desconocido, blanco libre, verde oscuro ocupado. ${alignmentLabel(alignment)}. El mapa individual se muestra sin transformar.` : `${summary}${batchComparison ? ` · Alineaciones sin confirmar: ${batchComparison.stats.alineaciones_sin_confirmar}; se conserva el origen común en esos casos. El consenso no es verdad del entorno.` : ''}`);
  const list = $('#batch-trials'); list.replaceChildren();
  if (!batchTrials.length) { const p = document.createElement('p'); p.textContent = 'Las diez pasadas aparecerán aquí.'; list.append(p); }
  for (const item of batchTrials) {
    const button = document.createElement('button'); button.type = 'button'; button.className = 'trial-card'; button.setAttribute('aria-label', `Examinar pasada ${item.id}`);
    const thumbnail = document.createElement('canvas'); paintBatchMap(thumbnail, item.map, item.grid, trialColors);
    const text = document.createElement('span'), title = document.createElement('strong'), details = document.createElement('small');
    const transform = batchComparison?.alignments?.find(alignment => alignment.id === item.id);
    title.textContent = `Pasada ${item.id} · ${item.state}`; details.textContent = `Semilla ${item.seed} · ${item.direction} · ${Number(item.seconds).toFixed(1)} s · ${item.summary?.slam?.closures ?? 0} cierres${transform ? ` · ${alignmentLabel(transform)}` : ''}`;
    text.append(title, details); button.append(thumbnail, text); button.addEventListener('click', () => { batchSelection = String(item.id); renderBatch(); }); list.append(button);
  }
  batchControls(Boolean(trialWorker));
}
$('#batch-map-select').addEventListener('change', event => { batchSelection = event.target.value; renderBatch(); });
$('#batch-map').addEventListener('click', event => {
  const canvas = event.currentTarget, rect = canvas.getBoundingClientRect(), x = Math.floor((event.clientX - rect.left) * canvas.width / rect.width), z = Math.floor((event.clientY - rect.top) * canvas.height / rect.height);
  if (x < 0 || x >= canvas.width || z < 0 || z >= canvas.height) return;
  const index = z * canvas.width + x;
  if (batchSelection === 'comparison' && batchComparison) setText('#batch-cell', `Celda ${x}, ${z} · libres ${batchComparison.freeVotes[index]} · ocupadas ${batchComparison.occupiedVotes[index]} · observadas ${batchComparison.observedVotes[index]} / 10`);
  else { const trial = batchTrials.find(item => String(item.id) === batchSelection); if (trial) setText('#batch-cell', `Celda ${x}, ${z} · ${['desconocida', 'libre', 'ocupada'][trial.map[index]] ?? 'sin dato'}`); }
});
$('#batch-start').addEventListener('click', () => {
  cancelBatch(true); batchScenario = scenario.id; batchTrials = []; batchComparison = null;
  try {
    const worker = new Worker(new URL('./trials.worker.js', import.meta.url), { type: 'module' }); trialWorker = worker; batchControls(true); setText('#batch-status', 'Calculando · 0/10 pasadas terminadas.');
    worker.onmessage = event => {
      if (trialWorker !== worker) return;
      const msg = event.data;
      if (msg.type === 'progress') setText('#batch-status', `Calculando · ${msg.completed}/10 pasadas terminadas.`);
      if (msg.type === 'trial') { batchTrials.push(msg.trial); if (batchTrials.length === 1) batchSelection = String(msg.trial.id); setText('#batch-status', `Calculando · ${msg.completed}/10 pasadas terminadas.`); renderBatch(); }
      if (msg.type === 'complete') { batchTrials = msg.trials; batchComparison = msg.comparison; batchSelection = 'comparison'; worker.terminate(); trialWorker = null; renderBatch(); setText('#batch-status', 'Comparación terminada · 10/10 pasadas.'); }
      if (msg.type === 'error') { worker.terminate(); trialWorker = null; batchControls(false); setText('#batch-status', `No se pudo completar la comparación: ${msg.message}`); }
    };
    worker.onerror = () => { if (trialWorker !== worker) return; worker.terminate(); trialWorker = null; batchControls(false); setText('#batch-status', 'No se pudo ejecutar la comparación. Revisa la consola del navegador.'); };
    worker.postMessage({ type: 'start', scenarioId: batchScenario, baseSeed: Number($('#sensor-seed').value) >>> 0, leash: explorer.leash, loopClosure: $('#slam-on').checked });
  } catch (error) { trialWorker = null; batchControls(false); setText('#batch-status', `No se pudo iniciar la comparación: ${error.message}`); }
});
$('#batch-cancel').addEventListener('click', () => cancelBatch());
$('#batch-export').addEventListener('click', () => {
  if (!batchComparison) return;
  const comparison = { ...batchComparison, freeVotes: Array.from(batchComparison.freeVotes), occupiedVotes: Array.from(batchComparison.occupiedVotes), observedVotes: Array.from(batchComparison.observedVotes), classes: Array.from(batchComparison.classes) };
  const data = { formato: 'homemind-comparacion-10', escenario: batchScenario, pasadas: batchTrials.map(trial => ({ ...trial, map: Array.from(trial.map) })), comparacion: comparison };
  const url = URL.createObjectURL(new Blob([JSON.stringify(data)], { type: 'application/json' })), link = document.createElement('a'); link.href = url; link.download = `homemind-comparacion-${batchScenario}.json`; link.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
});
$('#export').addEventListener('click', () => {
  const metadata = gridOf(grid);
  const bp = odomPose(belief);
  const data = { formato: 'homemind-fase-4', version: 1, escenario: scenario.id, descripcion: odomNoisy ? 'Odometría medida con encoders y giróscopo ruidosos; la verdad solo aparece en registros del observador.' : 'Pose exacta del simulador, no una localización estimada.', unidades: { posicion: 'metros', yaw: 'radianes', timestamp: 'ISO 8601 UTC (tiempo simulado)' }, coordenadas: 'Origen en el centro; X hacia la derecha y Z hacia abajo en la vista superior. Yaw 0 mira hacia -Z; positivo gira hacia -X.', intervalo_segundos: STEP, radio_robot_m: ROBOT_RADIUS, sensor: { alcance_m: SENSOR.range, fov_horizontal_rad: SENSOR.fov, rayos: SENSOR.rays, intervalo_pasos: 6, sin_retorno: null, ruido_m: odomNoisy ? .02 : 0 }, odometria: { modo: odomNoisy ? 'ruidosa' : 'exacta', parametros: odomNoisy ? { ...SENSORES_RUIDOSOS } : null, semilla: beliefSeed, error_final_m: Number(Math.hypot(bp.x - pose.x, bp.z - pose.z).toFixed(4)), registros: odomLog }, mapa: { origen_xz_m: [metadata.originX, metadata.originZ], resolucion_m: metadata.resolution, ancho: metadata.width, alto: metadata.height, orden: 'filas Z, columnas X', leyenda: { 0: 'desconocido', 1: 'libre', 2: 'ocupado' }, valores: Array.from(grid) }, exploracion: { estado: explorer.state, tiempo_autonomo_s: explorer.elapsed, localizacion: explorer.localizacion, correa: explorer.leash, base: explorer.base, posiciones_barridas: explorer.scanned, ruta_celdas: explorer.route }, keyframes, inventario: inventory, evaluacion_observador: evaluationReport, lecturas: readings, registros: history };
  data.slam = exportSlam(mission.slam);
  data.fusion_movimiento = motionFusionSummary(mission.motion);
  data.keyframes = keyframes.map(frame => ({ ...frame, pose_mapa: frameMapPose(frame) }));
  const url = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' }));
  const link = document.createElement('a'); link.href = url; link.download = 'homemind-registro.json'; link.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
});

function updateUI() {
  const bp = odomPose(belief);
  setHTML('#x', `${bp.x.toFixed(2)} <small>m</small>`);
  setHTML('#z', `${bp.z.toFixed(2)} <small>m</small>`);
  setHTML('#yaw', `${Math.round(bp.yaw * 180 / Math.PI)} <small>°</small>`);
  setText('#odom-error', `ERROR ODOM ${Math.hypot(bp.x - pose.x, bp.z - pose.z).toFixed(2)} m`);
  const moving = Object.values(lastApplied).some(Boolean);
  setText('#motion-state', moving ? '● EN MOVIMIENTO' : '● DETENIDO');
  setText('#collision', mission.motion.blockedDirection ? 'AVANCE BLOQUEADO · REOBSERVAR' : pose.collision ? 'CONTACTO CON PARED' : 'SIN CONTACTO');
  setText('#samples', `${history.length} ${history.length === 1 ? 'REGISTRO' : 'REGISTROS'}`);
  const labels = { inactivo: 'Exploración lista', barrido: 'Explorando · barrido del entorno', planificando: 'Explorando · buscando ruta', navegando: 'Explorando · avanzando a una frontera', retorno: 'Retornando a zona segura', refugio: 'En zona segura · retorno completado', retroceso: 'Retrocediendo tras contacto', relocalizando: 'Reobservando sin avanzar para confirmar la posición', localizacion_perdida: 'Detenido · no se pudo confirmar la posición', pausado: 'Exploración pausada', completado: 'Exploración terminada · sin fronteras pendientes', limitado: 'Exploración limitada · quedan fronteras sin una nueva observación segura', bloqueado: 'Exploración detenida · posición sin ruta segura' };
  setText('#explorer-status', labels[explorer.state]);
  const running = ACTIVE_STATES.includes(explorer.state);
  setHTML('#explore', running ? 'Ⅱ <span>Pausar</span>' : explorer.state === 'pausado' ? '▶ <span>Reanudar</span>' : '▶ <span>Explorar</span>');
  $('#explore').disabled = explorer.state === 'completado';
  const stats = counts(grid), percent = ((stats.libre + stats.ocupado) / grid.length * 100).toFixed(1);
  setText('#map-stats', `LIBRES ${stats.libre}  ·  OCUPADAS ${stats.ocupado}  ·  DESCONOCIDAS ${stats.desconocido}  ·  FRONTERAS ${frontierCells.length}  ·  OBSERVADO ${percent}% DE ${grid.length} CELDAS`);
  const metadata = gridOf(grid); setText('#map-scale', `${metadata.width * metadata.resolution} m`);
  setText('#frame-count', `${keyframes.length} / 400 KEYFRAMES`);
  setText('#capture-status', captureLimit ? 'Límite de 400 keyframes alcanzado. Reinicia para una nueva sesión.' : captureActive ? 'Captura activa · se guarda al cambiar la pose' : 'Inicia la captura o mueve el robot.');
  renderConsole(bp);
}
function renderConsole(bp) {
  const fmt = v => `${v < 0 ? '−' : '+'}${Math.abs(v).toFixed(2)}`;
  const wheels = wheelSpeeds(lastApplied || {});
  const lines = [`MOTORES  izq ${fmt(wheels.izq)} m/s · der ${fmt(wheels.der)} m/s · avance ${fmt(wheels.avance)} · giro ${fmt(wheels.giro)} rad/s`];
  if (latestScan) {
    const dists = latestScan.rays.map(r => r.distance).filter(Number.isFinite);
    const kinds = { wall: 0, floor: 0, invalid: 0 };
    for (const r of latestScan.rays) kinds[r.kind ?? (r.distance === null ? 'invalid' : 'wall')] = (kinds[r.kind ?? (r.distance === null ? 'invalid' : 'wall')] ?? 0) + 1;
    lines.push(`LIDAR    ${latestScan.rays.length} rayos · min ${dists.length ? Math.min(...dists).toFixed(2) : '—'} m · pared ${kinds.wall ?? 0} · suelo ${kinds.floor ?? 0} · inválidos ${kinds.invalid ?? 0}`);
  } else lines.push('LIDAR    sin lectura todavía');
  const last = keyframes.at(-1);
  lines.push(last
    ? `CÁMARA   ${keyframes.length} keyframes · último #${last.id} ${last.motivo} · ${last.detections.length ? last.detections.map(d => `${d.label}${d.frontal ? ' ✓' : ''}`).join(' / ') : 'sin marcadores'}`
    : 'CÁMARA   sin imágenes todavía');
  lines.push(inventory.length ? `INVENT   ${inventory.length} ${inventory.length === 1 ? 'objeto único' : 'objetos únicos'} · ${inventory.map(e => `${e.id}×${e.observaciones}`).join(' / ')}` : 'INVENT   sin objetos todavía');
  lines.push(`ODOM     ${odomNoisy ? 'ruidosa' : 'exacta'} · error ${Math.hypot(bp.x - pose.x, bp.z - pose.z).toFixed(2)} m · semilla ${beliefSeed} · scan ${scanCorrections}× ${scanDrift.toFixed(2)} m · topes ${bumpCount}`);
  lines.push(`SENS     enc ${fmt(lastMed.avance * 60)} m/s · giro ${fmt(lastMed.giro * 60)} rad/s`);
  const fusion = motionFusionSummary(mission.motion);
  const fusionLabels = { sin_referencia: 'Esperando LiDAR', contacto: 'Contacto: empuje detenido', reobservando: 'Reobservando tras soltar o cambiar dirección', lidar_insuficiente: 'LiDAR sin evidencia suficiente de avance', coherente: 'LiDAR y encoders coherentes', verificando_patinaje: 'Comprobando posible patinaje', patinaje_confirmado: 'Patinaje confirmado por LiDAR', desactivada: 'Control ideal' };
  lines.push(`FUSIÓN   ${fusionLabels[fusion.status]} · ruedas ${fmt(fusion.encoderM * 60)} m/s · aceptado ${fmt(fusion.acceptedM * 60)} m/s · descartado ${fusion.suppressedM.toFixed(3)} m · ajuste LiDAR ${fusion.correctedM.toFixed(3)} m`);
  const localization = explorer.localizacion;
  const slam = missionSummary(mission);
  const slamLabels = { aprendiendo: 'Guardando referencias del recorrido', sin_referencia_distinguible: 'Sin geometría suficiente para reconocer un lugar', verificando: 'Comprobando un posible regreso', coincidencia_rechazada: 'Coincidencia descartada; sigue observando', optimizacion_rechazada: 'Corrección descartada; mantiene la estimación', mapa_corregido: 'Mapa reconstruido tras un regreso confirmado', limite_de_memoria: 'Límite del registro alcanzado; corrección histórica detenida', control_ideal: 'Control ideal · sin correcciones', desactivado: 'Corrección histórica desactivada' };
  setText('#slam-status', `${slam.status === 'contacto_pendiente' ? 'Atendiendo contacto antes de corregir el mapa' : slamLabels[slam.status] ?? slam.status} · ${slam.submaps} submapas · ${slam.closures} cierres${slam.confirmations ? ` · comprobación ${slam.confirmations}/3` : ''}`);
  setText('#slam-events', slam.events.length ? slam.events.slice(-4).map(e => `${e.time.toFixed(1)} s · cierre ${e.reference} → ${e.current} · ajuste ${e.correctionM.toFixed(2)} m / ${(e.correctionYaw * 180 / Math.PI).toFixed(1)}° · mapa reconstruido`).join('\n') : 'Todavía no hay cierres confirmados. El mapa se actualiza con cada lectura; una coincidencia ambigua se descarta.');
  lines.push(`MAPEO    ${slam.submaps} submapas · ${slam.closures} cierres · ${slam.candidates} candidatos · ${slam.rejected} descartados`);
  const uncertainty = localization.confirmed ? 'Posición confirmada' : `Sin localización fiable: ${localization.lostSeconds.toFixed(1)} s${explorer.leash ? ` de ${explorer.leash.presupuestoS} s` : ' (límite desactivado)'}`;
  const streak = localization.streak > 0 && localization.streak < 3 ? ` · Reconfirmando ${localization.streak}/3` : '';
  setText('#localization-status', `${uncertainty}${streak}`);
  lines.push(`LOCALIZ  ${uncertainty}${streak} · tiempo autónomo ${explorer.elapsed.toFixed(1)} s`);
  lines.push(explorer.leash && explorer.base
    ? `CORREA   base estimada a ${Math.hypot(bp.x - explorer.base.x, bp.z - explorer.base.z).toFixed(2)} m (máx ${explorer.leash.maxDistanciaM} m) · retorno ${explorer.retorno ?? 'no'}`
    : 'CORREA   desactivada');
  setText('#console', lines.join('\n'));
}
function renderEvaluation() {
  setText('#evaluation-result', evaluationReport ? `${evaluationReport.aprobado ? 'Evaluación aprobada' : 'Evaluación incompleta'} · ${evaluationReport.encontrados}/${evaluationReport.total} frontales · ${evaluationReport.conDetalle}/${evaluationReport.total} con detalle suficiente (≥ ${evaluationReport.umbral_lado_px_480} px)${evaluationReport.pendientes.length ? ` · Pendientes: ${evaluationReport.pendientes.join(', ')}` : ''}${evaluationReport.ambiguos.length ? ` · Ambiguos: ${evaluationReport.ambiguos.join(', ')}` : ''}` : 'Evaluación del observador: aún no realizada.');
  const evidence = $('#evaluation-evidence');
  evidence.replaceChildren();
  evidence.hidden = !evaluationReport;
  if (!evaluationReport) return;
  for (const result of evaluationReport.tableros) {
    const card = document.createElement('article');
    card.className = 'frame-card';
    const frame = result.observado_frontalmente ? keyframes[result.keyframe - 1] : null;
    if (frame) {
      const image = document.createElement('img');
      image.src = frame.imagen_png;
      image.alt = `Evidencia frontal del marcador ${result.label}, keyframe ${result.keyframe}`;
      card.append(image);
      card.tabIndex = 0; card.setAttribute('role', 'button'); card.setAttribute('aria-label', `Examinar keyframe ${result.keyframe}`);
      card.addEventListener('click', () => selectFrame(result.keyframe));
      card.addEventListener('keydown', event => { if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); selectFrame(result.keyframe); } });
    }
    const caption = document.createElement('div');
    caption.className = 'frame-meta';
    const title = document.createElement('strong');
    title.textContent = result.label;
    const details = document.createElement('small');
    details.textContent = frame
      ? `Keyframe ${result.keyframe} · ${result.tiempo_s.toFixed(1)} s · ángulo ${result.angulo_grados}° · ${result.lado_min_px} px · ${result.detalle_suficiente ? 'detalle suficiente' : 'pendiente: poco detalle'}`
      : 'Sin evidencia frontal verificable';
    caption.append(title, details); card.append(caption); evidence.append(card);
  }
}
const INVENTORY_COLORS = { rojo: '#e5484d', azul: '#3e8ef7', verde: '#46a758', amarillo: '#f5b301' };
function renderInventory() {
  const list = $('#inventory-list'); list.replaceChildren();
  setText('#inventory-count', `${inventory.length} ${inventory.length === 1 ? 'OBJETO' : 'OBJETOS'}`);
  if (!inventory.length) { const empty = document.createElement('p'); empty.className = 'gallery-empty'; empty.textContent = keyframes.length ? 'Ninguna detección con posición verificable todavía.' : 'El inventario aparecerá al capturar imágenes con marcadores.'; list.append(empty); return; }
  for (const entry of [...inventory].sort((a, b) => a.id.localeCompare(b.id, 'es'))) {
    const card = document.createElement('article'); card.className = 'frame-card';
    card.tabIndex = 0; card.setAttribute('role', 'button'); card.setAttribute('aria-label', `Examinar mejor evidencia de ${entry.id}, keyframe ${entry.mejor_keyframe}`);
    card.addEventListener('click', () => selectFrame(entry.mejor_keyframe));
    card.addEventListener('keydown', event => { if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); selectFrame(entry.mejor_keyframe); } });
    const meta = document.createElement('div'); meta.className = 'frame-meta';
    const title = document.createElement('strong');
    const dot = document.createElement('i'); dot.className = 'inventory-dot'; dot.style.background = INVENTORY_COLORS[entry.label] ?? '#9aaba1';
    title.append(dot, document.createTextNode(`${entry.id} · x ${entry.x.toFixed(2)} m, z ${entry.z.toFixed(2)} m`));
    const details = document.createElement('small');
    details.textContent = `${entry.observaciones} ${entry.observaciones === 1 ? 'observación' : 'observaciones'} · ${entry.frontales} frontales · mejor: keyframe #${entry.mejor_keyframe} · ${entry.primera_vista_s.toFixed(1)}–${entry.ultima_vista_s.toFixed(1)} s`;
    meta.append(title, details); card.append(meta); list.append(card);
  }
}
function renderGallery() {
  const picker = $('#frame-select'); picker.replaceChildren();
  if (!keyframes.length) { const option = document.createElement('option'); option.value = ''; option.textContent = 'Sin imágenes'; picker.append(option); }
  keyframes.forEach(frame => { const option = document.createElement('option'); option.value = String(frame.id); option.textContent = `#${frame.id} · ${frame.tiempo_s.toFixed(1)} s · x ${frame.pose.x.toFixed(2)}, z ${frame.pose.z.toFixed(2)}`; picker.append(option); });
  if (selectedFrameId) picker.value = String(selectedFrameId);
  const gallery = $('#gallery'); gallery.replaceChildren();
  if (!keyframes.length) { const empty = document.createElement('p'); empty.className = 'gallery-empty'; empty.textContent = 'Las últimas 12 imágenes aparecerán aquí.'; gallery.append(empty); }
  for (const frame of keyframes.slice(-12).reverse()) {
    const card = document.createElement('article'); card.className = 'frame-card';
    card.tabIndex = 0; card.setAttribute('role', 'button'); card.setAttribute('aria-label', `Examinar keyframe ${frame.id}`);
    card.addEventListener('click', () => selectFrame(frame.id)); card.addEventListener('keydown', event => { if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); selectFrame(frame.id); } });
    const preview = document.createElement('div'); preview.className = 'frame-preview';
    const img = document.createElement('img'); img.src = frame.imagen_png; img.alt = `Keyframe a los ${frame.tiempo_s.toFixed(1)} segundos`; preview.append(img);
    for (const detection of frame.detections) {
      const box = document.createElement('span'); box.className = 'detection-box';
      box.style.left = `${100 * detection.bbox.x / frame.calibracion.width}%`; box.style.top = `${100 * detection.bbox.y / frame.calibracion.height}%`;
      box.style.width = `${100 * detection.bbox.width / frame.calibracion.width}%`; box.style.height = `${100 * detection.bbox.height / frame.calibracion.height}%`;
      box.title = `${detection.label}${detection.frontal ? ' · frontal útil' : ''}`; preview.append(box);
    }
    const meta = document.createElement('div'); meta.className = 'frame-meta';
    const title = document.createElement('strong'); title.textContent = `#${frame.id} · ${frame.tiempo_s.toFixed(1)} s · ${frame.motivo}`;
    const details = document.createElement('small'); details.textContent = `x ${frame.pose.x.toFixed(2)} · z ${frame.pose.z.toFixed(2)} · ${(frame.pose.yaw * 180 / Math.PI).toFixed(0)}°`;
    const hits = document.createElement('small'); hits.textContent = frame.detections.length ? frame.detections.map(d => `${d.label}${d.frontal ? ' ✓ frontal' : ' · parcial'}`).join(' / ') : 'Sin detecciones';
    meta.append(title, details, hits); card.append(preview, meta); gallery.append(card);
  }
  const last = keyframes.at(-1), total = keyframes.reduce((sum, frame) => sum + frame.detections.length, 0);
  inventory = fuseInventory(keyframes.map(frame => ({ ...frame, pose: frameMapPose(frame) })));
  setText('#latest-detections', last ? `Última imagen: ${last.detections.length ? last.detections.map(d => `${d.label} (${d.frontal ? 'frontal útil' : 'no frontal'})`).join(', ') : 'sin marcadores'} · ${total} observaciones con repeticiones · ${inventory.length} ${inventory.length === 1 ? 'objeto único' : 'objetos únicos'}` : 'Sin observaciones todavía');
  renderInventory();
  renderEvaluation();
  renderFrameDetail();
}
function selectFrame(id) { if (!keyframes.some(frame => frame.id === id)) return; selectedFrameId = id; $('#frame-select').value = String(id); renderFrameDetail(); drawOccupancy(); }
function renderFrameDetail() {
  const container = $('#frame-detail'); container.replaceChildren();
  const frame = keyframes.find(item => item.id === selectedFrameId);
  if (!frame) { const p = document.createElement('p'); p.textContent = 'Selecciona una imagen en la lista, la galería o el mapa.'; container.append(p); return; }
  const left = document.createElement('div'), preview = document.createElement('div'); preview.className = 'frame-preview selected-preview';
  const img = document.createElement('img'); img.src = frame.imagen_png; img.alt = `Imagen original del keyframe ${frame.id}`; preview.append(img);
  for (const d of frame.detections) { const box = document.createElement('span'); box.className = 'detection-box'; Object.assign(box.style, { left: `${100 * d.bbox.x / frame.calibracion.width}%`, top: `${100 * d.bbox.y / frame.calibracion.height}%`, width: `${100 * d.bbox.width / frame.calibracion.width}%`, height: `${100 * d.bbox.height / frame.calibracion.height}%` }); preview.append(box); }
  const meta = document.createElement('p'); meta.textContent = `#${frame.id} · ${frame.tiempo_s.toFixed(2)} s · posición cámara x ${frame.pose.x.toFixed(2)} m, z ${frame.pose.z.toFixed(2)} m · orientación ${(frame.pose.yaw * 180 / Math.PI).toFixed(1)}° · FOV horizontal 80°`;
  if (frame.slam_anchor != null) { const corrected = frameMapPose(frame); meta.textContent += ` · en mapa corregido x ${corrected.x.toFixed(2)}, z ${corrected.z.toFixed(2)} m, ${(corrected.yaw * 180 / Math.PI).toFixed(1)}°`; }
  left.append(preview, meta); container.append(left);
  const crops = document.createElement('div'); crops.className = 'frame-crops';
  if (!frame.detections.length) { const p = document.createElement('p'); p.textContent = 'No se detectaron marcadores en esta imagen.'; crops.append(p); }
  for (const d of frame.detections) {
    const crop = document.createElement('div'), canvas = document.createElement('canvas'), caption = document.createElement('small'); canvas.width = 160; canvas.height = 160;
    canvas.className = 'crop-preview'; caption.textContent = `${d.label} · ${d.lado_min_px ?? Math.min(d.bbox.width, d.bbox.height)} px lado corto · ${d.frontal ? 'frontal' : 'no frontal'} · ampliación no añade detalle`;
    crop.append(canvas, caption); crops.append(crop);
    const source = new Image(); source.onload = () => { const c = canvas.getContext('2d'); c.imageSmoothingEnabled = false; const b = d.bbox; c.drawImage(source, b.x, b.y, b.width, b.height, 0, 0, canvas.width, canvas.height); }; source.src = frame.imagen_png;
  }
  container.append(crops);
}
function drawOccupancy() {
  const size = Math.max(1, Math.floor(occupancy.clientWidth * devicePixelRatio));
  if (occupancy.width !== size || occupancy.height !== size) occupancy.width = occupancy.height = size;
  const meta = gridOf(grid), g = occupancyCtx, s = size / Math.max(meta.width, meta.height), offsetX = (size - meta.width * s) / 2, offsetZ = (size - meta.height * s) / 2;
  g.clearRect(0, 0, size, size);
  for (let i = 0; i < grid.length; i++) { g.fillStyle = grid[i] === FREE ? '#f8faf7' : grid[i] === OCCUPIED ? '#47665c' : '#cbd5cf'; g.fillRect(offsetX + (i % meta.width) * s, offsetZ + Math.floor(i / meta.width) * s, Math.ceil(s), Math.ceil(s)); }
  g.fillStyle = '#e9b65f'; for (const i of frontierCells) g.fillRect(offsetX + (i % meta.width) * s, offsetZ + Math.floor(i / meta.width) * s, Math.ceil(s), Math.ceil(s));
  const xy = p => [offsetX + (p.x - meta.originX) / meta.resolution * s, offsetZ + (p.z - meta.originZ) / meta.resolution * s];
  const bp = odomPose(belief);
  const [x, y] = xy(bp);
  if (latestScan) { const [sx, sy] = xy(latestScan.pose); g.fillStyle = 'rgba(67, 167, 130, .15)'; g.beginPath(); g.moveTo(sx, sy); for (const ray of latestScan.rays) { const distance = ray.kind === 'invalid' ? 0 : ray.distance ?? 0; const [rx, ry] = xy({ x: latestScan.pose.x - Math.sin(ray.angle) * distance, z: latestScan.pose.z - Math.cos(ray.angle) * distance }); g.lineTo(rx, ry); } g.closePath(); g.fill(); }
  g.strokeStyle = '#398f79'; g.lineWidth = Math.max(2, s); g.beginPath(); g.moveTo(x, y); for (const i of explorer.route) { const [rx, ry] = xy(center(i, grid)); g.lineTo(rx, ry); } g.stroke();
  for (const entry of inventory) { const [ix, iy] = xy(entry); g.fillStyle = INVENTORY_COLORS[entry.label] ?? '#9aaba1'; g.strokeStyle = '#fff'; g.lineWidth = Math.max(1, size * .003); g.beginPath(); g.arc(ix, iy, Math.max(3, size * .009), 0, Math.PI * 2); g.fill(); g.stroke(); }
  const groups = new Map();
  for (const frame of keyframes) { const p = frameMapPose(frame), k = `${Math.round(p.x / .35)},${Math.round(p.z / .35)}`; if (!groups.has(k)) groups.set(k, []); groups.get(k).push(frame); }
  frameGroups = [];
  for (const frames of groups.values()) { const chosen = frames.find(frame => frame.id === selectedFrameId) || frames.at(-1), [gx, gy] = xy(frameMapPose(chosen)); frameGroups.push({ x: gx, y: gy, frames }); g.fillStyle = chosen.id === selectedFrameId ? '#7136ab' : '#a18ab4'; g.strokeStyle = '#fff'; g.lineWidth = Math.max(1, size * .004); g.beginPath(); g.arc(gx, gy, Math.max(4, size * .012), 0, Math.PI * 2); g.fill(); g.stroke(); }
  const selected = keyframes.find(frame => frame.id === selectedFrameId);
  if (selected) { const mapped = frameMapPose(selected), [fx, fy] = xy(mapped), length = size * .06; g.save(); g.translate(fx, fy); g.rotate(-mapped.yaw); g.strokeStyle = '#7136ab'; g.lineWidth = Math.max(1.5, size * .005); g.beginPath(); g.moveTo(0, 0); g.lineTo(-length * Math.sin(SENSOR.fov / 2), -length * Math.cos(SENSOR.fov / 2)); g.moveTo(0, 0); g.lineTo(length * Math.sin(SENSOR.fov / 2), -length * Math.cos(SENSOR.fov / 2)); g.stroke(); g.restore(); }
  g.save(); g.translate(x, y); g.rotate(-bp.yaw); g.fillStyle = '#173f3d'; g.beginPath(); g.arc(0, 0, Math.max(ROBOT_RADIUS / meta.resolution * s, 4), 0, Math.PI * 2); g.fill(); g.fillStyle = '#bdf3d8'; g.beginPath(); g.moveTo(0, -s * 2.2); g.lineTo(-s, s); g.lineTo(s, s); g.fill(); g.restore();
}
occupancy.addEventListener('click', event => {
  const rect = occupancy.getBoundingClientRect(), x = (event.clientX - rect.left) * occupancy.width / rect.width, y = (event.clientY - rect.top) * occupancy.height / rect.height;
  const radius = Math.max(12 * occupancy.width / rect.width, occupancy.width * .025);
  const hit = frameGroups.map(group => ({ group, distance: Math.hypot(group.x - x, group.y - y) })).filter(item => item.distance <= radius).sort((a, b) => a.distance - b.distance)[0];
  if (hit) { const frames = hit.group.frames, current = frames.findIndex(frame => frame.id === selectedFrameId); selectFrame(frames[(current + 1) % frames.length].id); }
});
function drawMap() {
  const size = Math.max(1, Math.floor(map.clientWidth * devicePixelRatio));
  if (map.width !== size || map.height !== size) { map.width = size; map.height = size; }
  const c = size / 2, bounds = scenario.bounds, width = bounds.maxX - bounds.minX, depth = bounds.maxZ - bounds.minZ, scale = size * .77 / Math.max(width, depth);
  const left = c + bounds.minX * scale, top = c + bounds.minZ * scale;
  ctx.clearRect(0, 0, size, size); ctx.fillStyle = '#e9eeeb'; ctx.fillRect(0, 0, size, size);
  ctx.fillStyle = '#f8faf7'; ctx.fillRect(left, top, width * scale, depth * scale);
  ctx.strokeStyle = '#e5ebe7'; ctx.lineWidth = size * .003;
  for (let i = Math.ceil(bounds.minX); i < bounds.maxX; i++) { ctx.beginPath(); ctx.moveTo(c + i * scale, top); ctx.lineTo(c + i * scale, top + depth * scale); ctx.stroke(); }
  for (let i = Math.ceil(bounds.minZ); i < bounds.maxZ; i++) { ctx.beginPath(); ctx.moveTo(left, c + i * scale); ctx.lineTo(left + width * scale, c + i * scale); ctx.stroke(); }
  setText('#observer-scale', `${width} × ${depth} m`);
  ctx.strokeStyle = '#788f84'; ctx.lineWidth = size * .012; ctx.lineCap = 'butt';
  for (const [x1, z1, x2, z2] of scenario.walls) { ctx.beginPath(); ctx.moveTo(c + x1 * scale, c + z1 * scale); ctx.lineTo(c + x2 * scale, c + z2 * scale); ctx.stroke(); }
  ctx.strokeStyle = '#b08a7e'; ctx.lineWidth = size * .006;
  for (const [x1, z1, x2, z2] of scenario.obstaculos ?? []) { ctx.beginPath(); ctx.moveTo(c + x1 * scale, c + z1 * scale); ctx.lineTo(c + x2 * scale, c + z2 * scale); ctx.stroke(); }
  for (const board of scenario.boards) { ctx.fillStyle = board.color; ctx.beginPath(); ctx.arc(c + board.x * scale, c + board.z * scale, Math.max(size * .012, 3), 0, Math.PI * 2); ctx.fill(); }
  ctx.strokeStyle = '#49a995'; ctx.lineWidth = size * .009; ctx.lineJoin = 'round'; ctx.beginPath();
  trail.forEach((p, i) => { const x = c + p.x * scale, y = c + p.z * scale; if (!i) ctx.moveTo(x, y); else ctx.lineTo(x, y); });
  ctx.lineTo(c + pose.x * scale, c + pose.z * scale); ctx.stroke();
  const x = c + pose.x * scale, y = c + pose.z * scale;
  ctx.save(); ctx.translate(x, y); ctx.rotate(-pose.yaw); ctx.fillStyle = 'rgba(25, 85, 79, .13)'; ctx.beginPath(); ctx.arc(0, 0, size * .065, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = '#173f3d'; ctx.beginPath(); ctx.arc(0, 0, Math.max(ROBOT_RADIUS * scale, size * .025), 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = '#bdf3d8'; ctx.beginPath(); ctx.moveTo(0, -size * .038); ctx.lineTo(-size * .017, size * .012); ctx.lineTo(size * .017, size * .012); ctx.closePath(); ctx.fill(); ctx.restore();
}

reset();
try {
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.setClearColor(0xc8d1cd); viewport.prepend(renderer.domElement);
  const scene = new THREE.Scene(); scene.background = new THREE.Color(0xcbd4d0);
  scene.add(new THREE.HemisphereLight(0xf9fff9, 0x9ca9a1, 2.1));
  const light = new THREE.DirectionalLight(0xffffff, 1.3); light.position.set(-2, 6, 3); scene.add(light);
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshStandardMaterial({ color: 0xbfc9c1, roughness: 1 })); floor.rotation.x = -Math.PI / 2; scene.add(floor);
  const floorLines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xafbeb4 })); floorLines.position.y = .002; scene.add(floorLines);
  const wallMaterial = new THREE.MeshStandardMaterial({ color: 0xe6ebe5, roughness: 1, side: THREE.DoubleSide });
  const wallGroup = new THREE.Group(), boardGroup = new THREE.Group(); scene.add(wallGroup, boardGroup);
  rebuildWalls = () => {
    const b = scenario.bounds, width = b.maxX - b.minX, depth = b.maxZ - b.minZ;
    floor.geometry.dispose(); floor.geometry = new THREE.PlaneGeometry(width, depth); floor.position.set((b.minX + b.maxX) / 2, 0, (b.minZ + b.maxZ) / 2);
    const lines = []; for (let x = Math.ceil(b.minX); x <= b.maxX; x++) lines.push(x, 0, b.minZ, x, 0, b.maxZ); for (let z = Math.ceil(b.minZ); z <= b.maxZ; z++) lines.push(b.minX, 0, z, b.maxX, 0, z);
    floorLines.geometry.dispose(); floorLines.geometry = new THREE.BufferGeometry(); floorLines.geometry.setAttribute('position', new THREE.Float32BufferAttribute(lines, 3));
    for (const mesh of [...wallGroup.children]) { mesh.geometry.dispose(); wallGroup.remove(mesh); }
    for (const mesh of [...boardGroup.children]) { mesh.geometry.dispose(); mesh.material.dispose(); boardGroup.remove(mesh); }
    for (const [x1, z1, x2, z2] of scenario.walls) { const mesh = new THREE.Mesh(new THREE.PlaneGeometry(Math.hypot(x2 - x1, z2 - z1), 2.8), wallMaterial); mesh.position.set((x1 + x2) / 2, 1.4, (z1 + z2) / 2); mesh.rotation.y = -Math.atan2(z2 - z1, x2 - x1); wallGroup.add(mesh); }
    for (const [x1, z1, x2, z2] of scenario.obstaculos ?? []) { const mesh = new THREE.Mesh(new THREE.PlaneGeometry(Math.hypot(x2 - x1, z2 - z1), .18), wallMaterial); mesh.position.set((x1 + x2) / 2, .09, (z1 + z2) / 2); mesh.rotation.y = -Math.atan2(z2 - z1, x2 - x1); wallGroup.add(mesh); }
    for (const board of scenario.boards) { const material = new THREE.MeshBasicMaterial({ color: board.color, side: THREE.FrontSide, toneMapped: false }); const mesh = new THREE.Mesh(new THREE.PlaneGeometry(board.size, board.size), material); mesh.position.set(board.x, board.y, board.z); mesh.rotation.y = board.yaw; boardGroup.add(mesh); }
  };
  rebuildWalls();
  const camera = new THREE.PerspectiveCamera(65, 1, .03, 30); camera.rotation.order = 'YXZ';
  const captureWidth = 480, captureHeight = 270, captureCamera = new THREE.PerspectiveCamera(65, captureWidth / captureHeight, .03, 30);
  captureCamera.rotation.order = 'YXZ'; captureCamera.fov = 2 * Math.atan(Math.tan(SENSOR.fov / 2) / captureCamera.aspect) * 180 / Math.PI; captureCamera.updateProjectionMatrix();
  const target = new THREE.WebGLRenderTarget(captureWidth, captureHeight, { depthBuffer: true }); target.texture.colorSpace = THREE.SRGBColorSpace;
  const pixels = new Uint8Array(captureWidth * captureHeight * 4), flipped = new Uint8ClampedArray(pixels.length);
  const frameCanvas = document.createElement('canvas'); frameCanvas.width = captureWidth; frameCanvas.height = captureHeight; const frameContext = frameCanvas.getContext('2d');
  function captureFrame(reason) {
    captureCamera.position.copy(camera.position); captureCamera.rotation.copy(camera.rotation);
    renderer.setRenderTarget(target); renderer.render(scene, captureCamera); renderer.readRenderTargetPixels(target, 0, 0, captureWidth, captureHeight, pixels); renderer.setRenderTarget(null);
    const stride = captureWidth * 4; for (let row = 0; row < captureHeight; row++) flipped.set(pixels.subarray((captureHeight - row - 1) * stride, (captureHeight - row) * stride), row * stride);
    const image = new ImageData(new Uint8ClampedArray(flipped), captureWidth, captureHeight); const detections = detectMarkers(image);
    frameContext.putImageData(image, 0, 0); const f = captureWidth / (2 * Math.tan(SENSOR.fov / 2));
    keyframes.push({ id: keyframes.length + 1, pose: odomPose(belief), pose_real: { x: pose.x, z: pose.z, yaw: pose.yaw }, tiempo_s: step * STEP, motivo: reason, imagen_png: frameCanvas.toDataURL('image/png'), calibracion: { width: captureWidth, height: captureHeight, fx: f, fy: f, cx: captureWidth / 2, cy: captureHeight / 2, fov_horizontal_rad: SENSOR.fov }, detections });
    if (mission.slam) { const anchor = mission.slam.nodes.length - 1; Object.assign(keyframes.at(-1), { slam_anchor: anchor, slam_local: between(mission.slam.nodes[anchor].pose, odomPose(belief)) }); }
    if (!selectedFrameId) selectedFrameId = keyframes.at(-1).id;
    evaluationReport = null; renderGallery();
  }
  function resize() { const width = viewport.clientWidth, height = viewport.clientHeight; renderer.setSize(width, height, false); camera.aspect = width / height; camera.fov = 2 * Math.atan(Math.tan(SENSOR.fov / 2) / camera.aspect) * 180 / Math.PI; camera.updateProjectionMatrix(); drawMap(); }
  new ResizeObserver(resize).observe(viewport); new ResizeObserver(drawMap).observe(map); new ResizeObserver(drawOccupancy).observe(occupancy);
  renderer.setAnimationLoop(time => {
    if (lastTime !== null && !document.hidden) accumulator += Math.min((time - lastTime) / 1000, .1);
    lastTime = time;
    while (accumulator >= STEP) {
      const active = ACTIVE_STATES.includes(explorer.state);
      let bp = odomPose(belief);
      if ((active || Object.values(controls).some(Boolean)) && sensorSteps % 6 === 0) {
        const rays = odomNoisy ? sense(pose, scenario.walls, { ...SENSOR, noise: .02, random: sensorRandom }) : sense(pose, scenario.walls);
        const mapping = updateMissionMapping(mission, grid, rays, bp, explorer, step * STEP, SENSOR, lastBump);
        Object.assign(belief, mapping.pose); bp = odomPose(belief);
        if (mapping.loop.rebuilt) renderGallery();
        scanCorrections = mission.corrections; scanDrift = mission.correctionDistance;
        latestScan = { pose: { ...bp }, rays: mapping.rays }; readings.push({ paso: step, x: pose.x, z: pose.z, yaw_rad: pose.yaw, distancias_m: rays.map(r => r.distance), tipos: rays.map(r => r.kind ?? (r.distance === null ? 'invalid' : 'wall')) }); odomLog.push({ paso: step, x: bp.x, z: bp.z, yaw_rad: bp.yaw }); frontierCells = frontiers(grid);
      }
      autoCommand = active ? exploreStep(explorer, grid, bp, STEP, lastBump) : {};
      const applied = gateMotionCommand(mission.motion, active ? autoCommand : controls);
      lastApplied = applied;
      const prev = { x: pose.x, z: pose.z, yaw: pose.yaw };
      pose = advance(pose, applied, STEP, solidos(scenario));
      // Calibrate at the known launch pose until the first motor command.
      // Once started, gyro noise also runs while stationary.
      if (sensorsStarted || Object.values(applied).some(Boolean)) { sensorsStarted = true; lastMed = integrateFusedMotion(mission.motion, belief, readMotion(belief, pose, prev, applied, STEP, pose.collision), STEP); sensorSteps++; }
      step++;
      if (pose.collision && !active) { markBump(grid, bp, applied.back && !applied.forward); bumpCount++; }
      else if (pose.collision) bumpCount++;
      lastBump = pose.collision;
      history.push(sample(pose, step, startTime)); accumulator -= STEP;
      const lastPoint = trail[trail.length - 1];
      if (Math.hypot(pose.x - lastPoint.x, pose.z - lastPoint.z) >= .01) trail.push({ x: pose.x, z: pose.z });
    }
    camera.position.set(pose.x, .35, pose.z); camera.rotation.y = pose.yaw;
    if (captureActive && !document.hidden && !captureLimit) { const selection = selectKeyframe(selector, odomPose(belief), step * STEP, { force: captureRequested }); captureRequested = false; if (selection.capture) captureFrame(selection.reason); if (selection.reason === 'limite') captureLimit = true; }
    renderer.render(scene, camera); updateUI(); if (time - lastMapDraw >= 100) { drawMap(); drawOccupancy(); lastMapDraw = time; }
  });
  resize();
} catch (error) { console.error(error); $('#webgl-error').hidden = false; }
