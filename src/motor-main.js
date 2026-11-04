// HomeMind V2: same lab information as V1, powered by the navigation engine
// (src/navigation.js). The engine never sees the true pose; advance, sense
// and readMotion below are the simulator adapter (encoders + lidar stand-ins).
import { createNavigator } from './navigation.js';
import { sense, projectScan, SENSOR } from './sensor.js';
import { SCENARIOS, solidos } from './world.js';
import { advance, STEP } from './physics.js';
import { createOdometry, readMotion, SENSORES_RUIDOSOS } from './odometry.js';
import { mulberry32 } from './noise.js';
import { gridOf, counts, center, markBump, FREE, OCCUPIED } from './mapping.js';
import { wheelSpeeds } from './motion.js';
import { detectObjects, CAMERA } from './camera.js';
import { createVista3D } from './vista3d.js';

const $ = s => document.querySelector(s);
const occ = $('#occupancy').getContext('2d'), obs = $('#observer').getContext('2d');
const zero = () => ({ forward: false, back: false, left: false, right: false });
const LABELS = { inactivo: 'Exploración lista', barrido: 'Explorando · barrido del entorno', planificando: 'Explorando · buscando ruta', navegando: 'Explorando · avanzando a una frontera', retorno: 'Retornando a zona segura', refugio: 'En zona segura · retorno completado', retroceso: 'Retrocediendo tras contacto', relocalizando: 'Reobservando para confirmar la posición', localizacion_perdida: 'Detenido · sin posición confirmada', pausado: 'Exploración pausada', completado: 'Exploración terminada · sin fronteras', limitado: 'Exploración limitada · fronteras sin observación segura', bloqueado: 'Exploración detenida · sin ruta segura', destino: 'Navegando al destino', destino_alcanzado: 'Destino alcanzado' };
let scenario, nav, pose, adapter, sensorCfg, camCfg, step, lastMotion, lastCommand, latestRays, latestDets, scanPose, running, seed, trail, bumps;
let priorGuardado = null;
let marcasFusionadas = [];
const vista3d = createVista3D($('#vista3d'));
let modo3d = 'orbita';
$('#vista-orbita').addEventListener('click', () => { modo3d = 'orbita'; $('#vista-orbita').classList.add('primary'); $('#vista-robot').classList.remove('primary'); });
$('#vista-robot').addEventListener('click', () => { modo3d = 'robot'; $('#vista-robot').classList.add('primary'); $('#vista-orbita').classList.remove('primary'); });
// Host-side landmark fusion (belief frame): boards the camera saw, clustered
// like the inventory does. Exported with the map and fed back as priorMarcas
// on the next mission, where they disambiguate twin rooms.
function fusionarMarcas(detecciones) {
  if (!detecciones?.length || nav.state === 'relocalizando') return;
  const b = nav.belief;
  for (const d of detecciones) {
    const wx = b.x - Math.sin(b.yaw + d.bearing) * d.dist, wz = b.z - Math.cos(b.yaw + d.bearing) * d.dist;
    const hit = marcasFusionadas.find(m => m.label === d.label && Math.hypot(m.x - wx, m.z - wz) < .75);
    if (hit) { hit.x += (wx - hit.x) / ++hit.vistas; hit.z += (wz - hit.z) / hit.vistas; }
    else marcasFusionadas.push({ label: d.label, x: wx, z: wz, vistas: 1 });
  }
}

const select = $('#escenario');
for (const s of Object.values(SCENARIOS)) { const o = document.createElement('option'); o.value = s.id; o.textContent = s.label; select.append(o); }
select.value = 'casamoderna';

const keys = new Set();
const keyMap = { w: 'forward', arrowup: 'forward', s: 'back', arrowdown: 'back', a: 'left', arrowleft: 'left', d: 'right', arrowright: 'right' };
window.addEventListener('keydown', e => { const k = e.key.toLowerCase(); if (keyMap[k] && !e.target.matches('select, input, textarea')) { e.preventDefault(); keys.add(k); } });
window.addEventListener('keyup', e => keys.delete(e.key.toLowerCase()));
const manualCommand = () => { const c = zero(); for (const k of keys) if (keyMap[k]) c[keyMap[k]] = true; return c; };

function applyLeash() {
  nav.setLeash($('#correa').checked
    ? { presupuestoS: Number($('#correa-tiempo').value) || 60, maxDistanciaM: Number($('#correa-dist').value) || 5 }
    : null);
}
const SALIDAS_X = [{ x: .7, z: .5, yaw: .7 }, { x: -.7, z: .5, yaw: -.8 }, { x: .5, z: -.7, yaw: 1.9 }];
function reset() {
  scenario = SCENARIOS[select.value];
  const noisy = $('#ruido').checked;
  seed = (Math.random() * 2 ** 32) >>> 0;
  const inicio = scenario.inicio ?? { x: 0, z: 0, yaw: 0 };
  try {
    nav = createNavigator(scenario.mapConfig, { noisy, loopClosure: $('#slam').checked, seed, initialPose: inicio, priorMap: priorGuardado ?? undefined, priorFuerza: .5 });
  } catch {
    priorGuardado = null;
    $('#mapa-nombre').textContent = 'incompatible con este escenario';
    nav = createNavigator(scenario.mapConfig, { noisy, loopClosure: $('#slam').checked, seed, initialPose: inicio });
  }
  applyLeash();
  pose = { ...inicio, collision: false };
  if (priorGuardado && $('#salida-x').checked) pose = { ...SALIDAS_X[seed % SALIDAS_X.length], collision: false };
  nav.start();
  adapter = createOdometry(pose, noisy ? { ...SENSORES_RUIDOSOS, seed } : { seed });
  sensorCfg = noisy ? { ...SENSOR, noise: .02, random: mulberry32((seed ^ 0x51f3) >>> 0) } : SENSOR;
  camCfg = noisy ? { ...CAMERA, random: mulberry32((seed ^ 0xca3a) >>> 0) } : CAMERA;
  step = 0; lastMotion = null; lastCommand = zero(); latestRays = null; latestDets = null; marcasFusionadas = []; trail = [{ x: pose.x, z: pose.z }]; bumps = 0;
  running = true; $('#toggle').textContent = 'Pausar';
  draw();
}
function tick() {
  const perFrame = Number($('#velocidad').value);
  for (let i = 0; i < perFrame && !nav.terminal; i++, step++) {
    const manual = manualCommand(), manualActive = Object.values(manual).some(Boolean);
    if (manualActive && !['pausado', 'inactivo'].includes(nav.state)) nav.pause();
    let rays = null, detecciones = null;
    if (step % 6 === 0) {
      rays = sense(pose, scenario.walls, sensorCfg); scanPose = nav.belief; latestRays = projectScan(rays, scanPose, sensorCfg);
      detecciones = detectObjects(pose, [...scenario.boards, ...(scenario.muebles ?? [])], scenario.walls, camCfg); latestDets = detecciones;
      fusionarMarcas(detecciones);
    }
    const auto = nav.step(STEP, { rays, motion: lastMotion, detecciones });
    const command = manualActive ? manual : auto;
    lastCommand = command;
    const prev = { x: pose.x, z: pose.z, yaw: pose.yaw };
    const next = advance(pose, command, STEP, solidos(scenario));
    lastMotion = readMotion(adapter, next, prev, command, STEP, next.collision);
    if (next.collision) { bumps++; if (manualActive) markBump(nav.map, nav.belief, command.back && !command.forward); }
    pose = next;
    const last = trail.at(-1);
    if (Math.hypot(pose.x - last.x, pose.z - last.z) >= .05) trail.push({ x: pose.x, z: pose.z });
  }
  draw();
  if (nav.terminal) { running = false; $('#toggle').textContent = 'Terminado'; }
  if (running) requestAnimationFrame(tick);
}
function drawMapDiscovered() {
  const map = nav.map, g = gridOf(map), size = 640, s = size / Math.max(g.width, g.height);
  const ox = (size - g.width * s) / 2, oz = (size - g.height * s) / 2, c = occ;
  c.fillStyle = '#e9eeeb'; c.fillRect(0, 0, size, size);
  for (let i = 0; i < map.length; i++) {
    c.fillStyle = map[i] === FREE ? '#f8faf7' : map[i] === OCCUPIED ? '#47665c' : '#cbd5cf';
    c.fillRect(ox + (i % g.width) * s, oz + Math.floor(i / g.width) * s, Math.ceil(s), Math.ceil(s));
  }
  const xy = p => [ox + (p.x - g.originX) / g.resolution * s, oz + (p.z - g.originZ) / g.resolution * s];
  c.fillStyle = '#e9b65f';
  for (const i of nav.fronts()) c.fillRect(ox + (i % g.width) * s, oz + Math.floor(i / g.width) * s, Math.ceil(s), Math.ceil(s));
  if (latestRays && scanPose) {
    const [sx, sy] = xy(scanPose);
    c.fillStyle = 'rgba(67,167,130,.15)'; c.beginPath(); c.moveTo(sx, sy);
    for (const ray of latestRays) { const d = ray.kind === 'invalid' ? 0 : ray.distance ?? 0; const [rx, ry] = xy({ x: scanPose.x - Math.sin(ray.angle) * d, z: scanPose.z - Math.cos(ray.angle) * d }); c.lineTo(rx, ry); }
    c.closePath(); c.fill();
  }
  c.strokeStyle = '#398f79'; c.lineWidth = Math.max(2, s); c.beginPath();
  const route = nav.explorer.route;
  if (route.length) { const [x0, y0] = xy(nav.belief); c.moveTo(x0, y0); for (const i of route) { const [rx, ry] = xy(center(i, map)); c.lineTo(rx, ry); } c.stroke(); }
  if (nav.destination) {
    const [dx, dy] = xy(nav.destination);
    c.strokeStyle = '#b03a8c'; c.lineWidth = 2.5; c.beginPath(); c.arc(dx, dy, 8, 0, Math.PI * 2); c.stroke();
    c.beginPath(); c.moveTo(dx - 6, dy - 6); c.lineTo(dx + 6, dy + 6); c.moveTo(dx + 6, dy - 6); c.lineTo(dx - 6, dy + 6); c.stroke();
  }
  const b = nav.belief, [bx, by] = xy(b);
  c.save(); c.translate(bx, by); c.rotate(-b.yaw); c.fillStyle = '#173f3d';
  c.beginPath(); c.arc(0, 0, Math.max(4, s * 1.1), 0, Math.PI * 2); c.fill();
  c.fillStyle = '#bdf3d8'; c.beginPath(); c.moveTo(0, -s * 2.2); c.lineTo(-s, s); c.lineTo(s, s); c.fill(); c.restore();
}
function drawObserver() {
  const size = 640, b = scenario.bounds, w = b.maxX - b.minX, d = b.maxZ - b.minZ, scale = size * .8 / Math.max(w, d);
  const c = obs, cx = size / 2, left = cx + b.minX * scale, top = cx + b.minZ * scale;
  c.clearRect(0, 0, size, size); c.fillStyle = '#e9eeeb'; c.fillRect(0, 0, size, size);
  c.fillStyle = '#f8faf7'; c.fillRect(left, top, w * scale, d * scale);
  c.strokeStyle = '#788f84'; c.lineWidth = size * .012;
  for (const [x1, z1, x2, z2] of scenario.walls) { c.beginPath(); c.moveTo(cx + x1 * scale, cx + z1 * scale); c.lineTo(cx + x2 * scale, cx + z2 * scale); c.stroke(); }
  c.strokeStyle = '#b08a7e'; c.lineWidth = size * .006;
  for (const [x1, z1, x2, z2] of scenario.obstaculos ?? []) { c.beginPath(); c.moveTo(cx + x1 * scale, cx + z1 * scale); c.lineTo(cx + x2 * scale, cx + z2 * scale); c.stroke(); }
  for (const board of scenario.boards) { c.fillStyle = board.color; c.beginPath(); c.arc(cx + board.x * scale, cx + board.z * scale, Math.max(size * .012, 3), 0, Math.PI * 2); c.fill(); }
  c.strokeStyle = '#398f79'; c.lineWidth = 2; c.beginPath();
  trail.forEach((p, i) => i ? c.lineTo(cx + p.x * scale, cx + p.z * scale) : c.moveTo(cx + p.x * scale, cx + p.z * scale));
  c.stroke();
  c.save(); c.translate(cx + pose.x * scale, cx + pose.z * scale); c.rotate(-pose.yaw); c.fillStyle = '#173f3d';
  c.beginPath(); c.moveTo(0, -9); c.lineTo(-6, 7); c.lineTo(6, 7); c.closePath(); c.fill(); c.restore();
}
const fmt = v => `${v < 0 ? '−' : '+'}${Math.abs(v).toFixed(2)}`;
function draw() {
  drawMapDiscovered(); drawObserver();
  vista3d.update(nav, scenario, modo3d);
  const sum = nav.summary(), b = nav.belief, cells = counts(nav.map);
  $('#map-stats').textContent = `LIBRES ${cells.libre} · OCUPADAS ${cells.ocupado} · DESCONOCIDAS ${cells.desconocido} · FRONTERAS ${sum.fronteras} · OBSERVADO ${sum.conocidas_pct}%`;
  $('#explorer-status').textContent = LABELS[nav.state] ?? nav.state;
  const relo = sum.relocalizacion;
  $('#relo-estado').textContent = !priorGuardado ? '' : nav.state === 'relocalizando' ? 'Relocalizando automáticamente (giro + avance)…'
    : relo?.found ? `Relocalizado ✓ · acuerdo ${relo.score}` : relo?.revocada ? 'Ubicación revocada por desacuerdo: reintentando…' : relo ? 'Sin relocalizar: explorando desde la pose supuesta' : 'Mapa previo cargado';
  $('#t-x').textContent = `${b.x.toFixed(2)} m`; $('#t-z').textContent = `${b.z.toFixed(2)} m`;
  $('#t-yaw').textContent = `${(b.yaw * 180 / Math.PI).toFixed(0)}°`;
  $('#t-error').textContent = `${Math.hypot(b.x - pose.x, b.z - pose.z).toFixed(2)} m`;
  const moving = Object.values(lastCommand).some(Boolean);
  $('#motion-state').textContent = moving ? '● EN MOVIMIENTO' : '● DETENIDO';
  $('#t-col').textContent = pose.collision ? 'CONTACTO' : 'SIN CONTACTO';
  $('#t-time').textContent = `${(step * STEP).toFixed(1)} s`;
  $('#t-bumps').textContent = String(bumps);
  $('#t-ret').textContent = nav.explorer.retorno ?? 'no';
  const dest = nav.destination;
  $('#t-dest').textContent = dest ? `x ${dest.x.toFixed(2)} · z ${dest.z.toFixed(2)} · a ${Math.hypot(b.x - dest.x, b.z - dest.z).toFixed(2)} m` : '—';
  const wheels = wheelSpeeds(lastCommand);
  const lines = [`MOTORES  izq ${fmt(wheels.izq)} m/s · der ${fmt(wheels.der)} m/s · avance ${fmt(wheels.avance)} · giro ${fmt(wheels.giro)} rad/s`];
  if (latestRays) {
    const dists = latestRays.map(r => r.distance).filter(Number.isFinite);
    const kinds = { wall: 0, floor: 0, invalid: 0 };
    for (const r of latestRays) kinds[r.kind ?? (r.distance === null ? 'invalid' : 'wall')] = (kinds[r.kind ?? (r.distance === null ? 'invalid' : 'wall')] ?? 0) + 1;
    lines.push(`LIDAR    ${latestRays.length} rayos · min ${dists.length ? Math.min(...dists).toFixed(2) : '—'} m · pared ${kinds.wall ?? 0} · suelo ${kinds.floor ?? 0} · inválidos ${kinds.invalid ?? 0}`);
  } else lines.push('LIDAR    sin lectura todavía');
  lines.push(`ODOM     ${$('#ruido').checked ? 'ruidosa' : 'exacta'} · error ${Math.hypot(b.x - pose.x, b.z - pose.z).toFixed(2)} m · semilla ${seed} · scan ${sum.correcciones_scan}× ${sum.deriva_corregida_m} m · topes ${bumps}`);
  lines.push(`SENS     enc ${fmt((lastMotion?.avance ?? 0) * 60)} m/s · giro ${fmt((lastMotion?.giro ?? 0) * 60)} rad/s`);
  lines.push(`MAPEO    ${sum.slam.submaps} submapas · ${sum.slam.closures} cierres · ${sum.slam.candidates} candidatos · ${sum.slam.rejected} descartados`);
  const loc = nav.explorer.localizacion;
  lines.push(`LOCALIZ  ${loc.confirmed ? 'Posición confirmada' : `Sin localización fiable: ${loc.lostSeconds.toFixed(1)} s`} · tiempo autónomo ${nav.explorer.elapsed.toFixed(1)} s`);
  lines.push(latestDets?.length
    ? `CAMARA   ve ${latestDets.map(d => `${d.label} a ${d.dist.toFixed(1)} m`).join(' · ')} · marcas conocidas ${marcasFusionadas.length} · previas ${sum.marcas_previas}`
    : `CAMARA   sin objetos en vista · marcas conocidas ${marcasFusionadas.length} · previas ${sum.marcas_previas}`);
  lines.push(nav.explorer.leash && nav.explorer.base
    ? `CORREA   base estimada a ${Math.hypot(b.x - nav.explorer.base.x, b.z - nav.explorer.base.z).toFixed(2)} m (máx ${nav.explorer.leash.maxDistanciaM} m) · retorno ${nav.explorer.retorno ?? 'no'}`
    : 'CORREA   desactivada');
  $('#console').textContent = lines.join('\n');
}
$('#toggle').addEventListener('click', () => {
  if (nav.terminal) return;
  if (['barrido', 'planificando', 'navegando', 'retorno', 'retroceso', 'relocalizando'].includes(nav.state)) { nav.pause(); running = false; $('#toggle').textContent = 'Reanudar'; }
  else { nav.start(); running = true; $('#toggle').textContent = 'Pausar'; requestAnimationFrame(tick); }
});
$('#reiniciar').addEventListener('click', reset);
$('#exportar').addEventListener('click', () => {
  const data = { ...nav.saveMap(), marcas: marcasFusionadas.map(({ label, x, z }) => ({ label, x, z })), escenario: scenario.id, semilla: seed, resumen: nav.summary() };
  const url = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' }));
  const a = document.createElement('a'); a.href = url; a.download = 'homemind-mapa.json'; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});
$('#mapa-file').addEventListener('change', async event => {
  const file = event.target.files[0];
  if (!file) return;
  try {
    const data = JSON.parse(await file.text());
    if (!Array.isArray(data.celdas) || !data.grid) throw new Error('formato');
    priorGuardado = data;
    $('#mapa-nombre').textContent = file.name;
  } catch { $('#mapa-nombre').textContent = 'archivo inválido'; priorGuardado = null; }
  reset();
});
$('#salida-x').addEventListener('change', reset);
function irAlDestino() {
  const destino = { x: Number($('#dest-x').value), z: Number($('#dest-z').value) };
  try { nav.setDestination(destino); } catch { return; }
  nav.start();
  if (!running) { running = true; $('#toggle').textContent = 'Pausar'; requestAnimationFrame(tick); }
}
$('#ir').addEventListener('click', irAlDestino);
$('#limpiar-destino').addEventListener('click', () => nav.setDestination(null));
$('#occupancy').addEventListener('click', event => {
  const rect = event.target.getBoundingClientRect(), g = gridOf(nav.map);
  const px = (event.clientX - rect.left) * 640 / rect.width, py = (event.clientY - rect.top) * 640 / rect.height;
  const s = 640 / Math.max(g.width, g.height), ox = (640 - g.width * s) / 2, oz = (640 - g.height * s) / 2;
  $('#dest-x').value = (g.originX + (px - ox) / s * g.resolution).toFixed(1);
  $('#dest-z').value = (g.originZ + (py - oz) / s * g.resolution).toFixed(1);
  irAlDestino();
});
for (const id of ['#escenario', '#ruido', '#slam']) $(id).addEventListener('change', reset);
for (const id of ['#correa', '#correa-dist', '#correa-tiempo']) $(id).addEventListener('change', applyLeash);
reset();
requestAnimationFrame(tick);
