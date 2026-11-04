import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { extractGlbGeometry } from './glb-geometry.js';
import { createScanSession } from './scan-session.js';
import { createVistaDescubierta } from './vista-descubierta.js';
import { readCalibration } from './scan-calibration.js';
import { counts, center, FREE, OCCUPIED, UNKNOWN } from './mapping.js';
import { destinationFromPointer, selectMapDestination } from './scan-destination.js';
import { diagnosticarNavegacion } from './debug-nav.js';
import { createRobotCameraSensor } from './robot-camera-sensor.js';
import { createAiPanel } from './ai-panel.js';
import { faceObjectCommand } from './ai-grounding.js';
import { wheelSpeeds } from './motion.js';
import { lidarViewRays } from './lidar-view.js';
import { t, aplicarIdioma, idiomaActual } from './i18n.js';
import { elegirPuntoDeFoto, crearRegistroFotosZonas } from './foto-viewpoint.js';
const $ = s => document.querySelector(s), url = '/scans/polycam-2026-10-28/10_28_2026-varias-habitaciones.glb';
let session, geometry, calibration, running = false, frame = null, groundStopped = false;
let viewMode = 'plan';
let manualMode = false, automaticSpeed = '16';
let requestedRoute = [];
let goalError = null;
let cameraSensor, aiPanel, aiTarget = null, aiBusy = false;
const driveInputs = new Map();
function driveCommand() {
  const directions = new Set(driveInputs.values());
  return Object.fromEntries(['forward', 'back', 'left', 'right'].map(k => [k, directions.has(k)]));
}
function updateDriveControls() {
  const command = driveCommand();
  for (const button of document.querySelectorAll('[data-drive]')) {
    button.classList.toggle('selected', command[button.dataset.drive]);
    button.setAttribute('aria-pressed', String(command[button.dataset.drive]));
  }
  $('#manual-mode').classList.toggle('selected', manualMode);
  $('#manual-mode').setAttribute('aria-pressed', String(manualMode));
  $('#drive-mode').textContent = manualMode ? t('direccionManualEstado') : session?.nav.destination ? t('navegacionDestino') : t('exploracionAutomatica');
  $('#toggle').textContent = manualMode ? t('explorarAuto') : running ? t('pausar') : session?.nav.destination && session.nav.state !== 'destino_alcanzado' ? t('continuarDestino') : t('explorar');
}
function clearDriveInputs() { driveInputs.clear(); updateDriveControls(); }
function enterManual() {
  if (!session || aiBusy) return;
  if (!manualMode) {
    aiTarget = null;
    session.nav.setDestination(null); requestedRoute = []; goalError = null;
    automaticSpeed = $('#speed').value;
    manualMode = true; $('#speed').value = '1'; $('#speed').disabled = true;
    if (session.nav.state === 'inactivo') session.nav.start();
    session.nav.pause(); view('robot');
  }
  groundStopped = false; running = true;
  $('#nav-status').textContent = t('manualStatus');
  updateDriveControls();
  if (frame === null) frame = requestAnimationFrame(tick);
}
function stopDriving() {
  if (!session) return;
  running = false; clearDriveInputs(); session.nav.pause();
  $('#nav-status').textContent = t('detenidoStatus');
  draw();
}
const captureMaterials = [], cameraHeight = .6, cameraHorizontalFov = 80;
let cameraPitchDeg = 12;
try { const raw = localStorage.getItem('homemind-camera-pitch'); const saved = Number(raw); if (raw !== null && Number.isFinite(saved) && saved >= 0 && saved <= 30) cameraPitchDeg = saved; } catch { /* sin persistencia */ }
const scene = new THREE.Scene(); scene.background = new THREE.Color('#e4ebe5');
const renderer = new THREE.WebGLRenderer({ canvas: $('#room-view'), antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.localClippingEnabled = true;
const camera = new THREE.PerspectiveCamera(45, 1, .01, 100), controls = new OrbitControls(camera, renderer.domElement);
const robot = new THREE.Group(), body = new THREE.Mesh(new THREE.CylinderGeometry(.10, .10, .6, 24), new THREE.MeshBasicMaterial({ color: '#ec8d42' }));
body.position.y = .3; robot.add(body);
const nose = new THREE.Mesh(new THREE.BoxGeometry(.09, .03, .22), new THREE.MeshBasicMaterial({ color: '#173f3d' })); nose.position.set(0, .62, -.12); robot.add(nose); scene.add(robot);
function render() {
  if (viewMode === 'robot' && session) {
    // This is the simulator's optical view at the physical camera mount.
    // It never supplies poses, pixels or mesh geometry to the navigator.
    const p = session.pose, dx = -Math.sin(p.yaw), dz = -Math.cos(p.yaw);
    camera.up.set(0, 1, 0);
    camera.fov = THREE.MathUtils.radToDeg(2 * Math.atan(Math.tan(THREE.MathUtils.degToRad(cameraHorizontalFov / 2)) / camera.aspect));
    camera.updateProjectionMatrix();
    camera.position.set(p.x + dx * .11, session.world.floorY + cameraHeight, p.z + dz * .11);
    camera.lookAt(camera.position.x + dx, camera.position.y - Math.tan(THREE.MathUtils.degToRad(cameraPitchDeg)), camera.position.z + dz);
  } else controls.update();
  renderer.render(scene, camera);
}
controls.addEventListener('change', () => renderer.render(scene, camera));
new ResizeObserver(() => { const parent = renderer.domElement.parentElement; renderer.setSize(parent.clientWidth, parent.clientHeight, false); camera.aspect = parent.clientWidth / parent.clientHeight; camera.updateProjectionMatrix(); render(); }).observe(renderer.domElement.parentElement);
function view(mode = 'plan') {
  if (!session) return;
  viewMode = mode;
  const firstPerson = mode === 'robot', top = mode === 'plan';
  controls.enabled = !firstPerson; robot.visible = !firstPerson;
  camera.fov = 45; camera.updateProjectionMatrix();
  for (const material of captureMaterials) { material.clippingPlanes = firstPerson ? [] : [observerCut]; material.needsUpdate = true; }
  for (const [id, name] of [['top', 'plan'], ['orbit', 'orbit'], ['robot-camera', 'robot']]) {
    $(`#${id}`).classList.toggle('selected', name === mode);
    $(`#${id}`).setAttribute('aria-pressed', String(name === mode));
  }
  $('#scene-title').textContent = firstPerson ? t('camaraMovimiento') : t('capturaObservador');
  $('#scene-label').textContent = firstPerson ? t('camaraVirtual') : t('mallaCompleta');
  $('#scene-help').textContent = firstPerson ? t('camaraAyuda') : t('escenaAyuda');
  if (firstPerson) { render(); return; }
  const b = session.world.bounds, size = Math.max(b.maxX - b.minX, b.maxZ - b.minZ), tangent = Math.tan(Math.PI / 8);
  const fit = 1.6 + size / (2 * tangent) * Math.max(1, 1 / camera.aspect) * 1.05;
  camera.up.set(0, top ? 0 : 1, top ? -1 : 0); camera.position.set(top ? 0 : fit * .5, top ? fit : fit * .7, top ? 0 : fit * .6);
  controls.target.set(0, .15, 0); controls.enableRotate = !top; render();
}
function drawMap() {
  const reference = $('#reference').checked, map = reference ? session.world.map : session.nav.map, g = map.grid;
  const c = $('#known-map').getContext('2d'), size = 640, unit = size / Math.max(g.width, g.height), ox = (size - g.width * unit) / 2, oz = (size - g.height * unit) / 2;
  c.fillStyle = '#cbd5cf'; c.fillRect(0, 0, size, size);
  for (let i = 0; i < map.length; i++) { c.fillStyle = map[i] === FREE ? '#fbfcf8' : map[i] === OCCUPIED ? '#41675a' : '#cbd5cf'; c.fillRect(ox + (i % g.width) * unit, oz + Math.floor(i / g.width) * unit, unit + .2, unit + .2); }
  const pixel = p => [ox + (p.x - g.originX) / g.resolution * unit, oz + (p.z - g.originZ) / g.resolution * unit];
  if (reference) { c.beginPath(); for (const p of session.trail) c.lineTo(...pixel(p)); c.strokeStyle = '#4f9db0'; c.lineWidth = 2; c.stroke(); }
  for (const [index, observation] of (aiPanel?.observations ?? []).entries()) if (observation.point) {
    const [px, pz] = pixel(observation.point); c.fillStyle = '#8b49bc'; c.beginPath(); c.arc(px, pz, 7, 0, Math.PI * 2); c.fill();
    c.font = 'bold 14px sans-serif'; c.fillText(`IA ${index + 1}`, px + 9, pz - 6);
  }
  const goal = session.nav.destination;
  if (goal) {
    const route = ['destino', 'pausado'].includes(session.nav.state) ? (session.nav.explorer.route.length ? session.nav.explorer.route : requestedRoute) : session.nav.state === 'planificando' ? requestedRoute : [];
    c.beginPath(); c.moveTo(...pixel(session.nav.belief));
    for (const i of route) c.lineTo(...pixel(center(i, session.nav.map)));
    c.strokeStyle = '#228ac5'; c.lineWidth = 3; c.stroke();
    const [gx, gz] = pixel(goal);
    c.beginPath(); c.arc(gx, gz, 9, 0, Math.PI * 2); c.fillStyle = '#dd543e'; c.fill(); c.strokeStyle = '#fff'; c.lineWidth = 2; c.stroke();
    c.beginPath(); c.moveTo(gx - 4, gz); c.lineTo(gx + 4, gz); c.moveTo(gx, gz - 4); c.lineTo(gx, gz + 4); c.stroke();
  }
  const p = reference ? session.pose : session.nav.belief, [x, z] = pixel(p);
  c.beginPath(); c.arc(x, z, .10 / g.resolution * unit, 0, Math.PI * 2); c.fillStyle = '#ec8d4280'; c.fill(); c.strokeStyle = '#bb6322'; c.stroke();
  c.beginPath(); c.moveTo(x, z); c.lineTo(x - Math.sin(p.yaw) * 35, z - Math.cos(p.yaw) * 35); c.stroke();
  $('#map-label').textContent = reference ? t('referenciaFisica') : t('mapaDescubierto');
}
const vistaDescubierta = createVistaDescubierta($('#descubierto-3d'));
function drawLidar() {
  const canvas = $('#lidar-map'), c = canvas.getContext('2d');
  const map = session.nav.map, g = map.grid, unit = 640 / Math.max(g.width, g.height);
  const ox = (640 - g.width * unit) / 2, oz = (640 - g.height * unit) / 2;
  c.fillStyle = '#101613'; c.fillRect(0, 0, 640, 640);
  for (let i = 0; i < map.length; i++) {
    if (map[i] === 0) continue;
    c.fillStyle = map[i] === FREE ? '#31463a' : '#d9e8db';
    c.fillRect(ox + (i % g.width) * unit, oz + Math.floor(i / g.width) * unit, unit + .2, unit + .2);
  }
  const pixel = p => [ox + (p.x - g.originX) / g.resolution * unit, oz + (p.z - g.originZ) / g.resolution * unit];
  const scan = session.lastScan;
  if (scan) {
    const b = scan.belief, [bx, bz] = pixel(b);
    for (const ray of lidarViewRays(scan)) {
      const [px, pz] = pixel(ray);
      c.beginPath(); c.moveTo(bx, bz); c.lineTo(px, pz);
      c.strokeStyle = ray.kind === 'wall' ? 'rgba(255,96,72,.6)' : 'rgba(170,225,190,.20)'; c.lineWidth = ray.kind === 'wall' ? 1.4 : 1; c.stroke();
      if (ray.kind === 'wall') { c.beginPath(); c.arc(px, pz, 2.2, 0, Math.PI * 2); c.fillStyle = '#ff6048'; c.fill(); }
    }
    c.beginPath(); c.arc(bx, bz, 6, 0, Math.PI * 2); c.fillStyle = '#ec8d42'; c.fill(); c.strokeStyle = '#101613'; c.stroke();
    c.beginPath(); c.moveTo(bx, bz); c.lineTo(bx - Math.sin(b.yaw) * 26, bz - Math.cos(b.yaw) * 26); c.strokeStyle = '#ec8d42'; c.lineWidth = 2; c.stroke();
  }
}
let debugCache = { clave: '', texto: '' };
function actualizarDebug(forzar = false) {
  if (!session) return;
  const nav = session.nav, e = nav.explorer, b = nav.belief;
  const clave = [nav.map.revision, nav.state, e.route.length, Math.round(b.x * 20), Math.round(b.z * 20), Math.round(b.yaw * 20), e.destino?.x, e.destino?.z].join(':');
  if (!forzar && clave === debugCache.clave) { $('#debug-out').textContent = debugCache.texto; return; }
  const d = diagnosticarNavegacion({ map: nav.map, creencia: b, explorer: e, ultimoComando: session.lastCommand });
  debugCache = { clave, texto: [`MOTIVO   ${d.motivo}`, ...d.lineas].join('\n') };
  $('#debug-out').textContent = debugCache.texto;
}
function detenerAutomatico() {
  running = false;
  if (frame !== null) { cancelAnimationFrame(frame); frame = null; }
  clearDriveInputs();
}
// Single debug commands run through the same session step as manual
// driving (footprint guard included) and leave automatic navigation paused.
function ejecutarComandoDebug(comando, pasos, etiqueta) {
  if (!session || aiBusy) return;
  detenerAutomatico();
  if (session.nav.state === 'inactivo') session.nav.start();
  for (let i = 0; i < pasos; i++) { const r = session.step(comando); if (r.stopped) break; }
  groundStopped = false;
  $('#nav-status').textContent = t('debugComando', { cmd: etiqueta });
  draw(); actualizarDebug(true);
}
function replanificarDebug() {
  if (!session || aiBusy) return;
  detenerAutomatico();
  const nav = session.nav, e = nav.explorer;
  nav.start();
  e.route = []; e.safeRevision = -1;
  if (['navegando', 'destino', 'retroceso', 'barrido'].includes(e.state)) e.state = 'planificando';
  session.step();
  $('#nav-status').textContent = t('debugReplan');
  draw(); actualizarDebug(true);
}
function actualizarConsola() {
  const d = session.diagnostics, b = session.nav.belief, p = session.pose, c = counts(session.nav.map);
  const scan = session.lastScan;
  let lidar = t('cLidarSin');
  if (scan) {
    const pared = scan.rays.filter(r => r.kind === 'wall'), suelo = scan.rays.filter(r => r.kind === 'floor'), invalidos = scan.rays.length - pared.length - suelo.length;
    const dists = scan.rays.filter(r => r.kind !== 'invalid').map(r => r.distance);
    lidar = t('cLidar', { n: scan.rays.length, pared: pared.length, suelo: suelo.length, invalidos, min: Math.min(...dists).toFixed(2), media: (dists.reduce((a, x) => a + x, 0) / dists.length).toFixed(2) });
  }
  const w = wheelSpeeds(session.lastCommand ?? { forward: false, back: false, left: false, right: false });
  const goal = session.nav.destination;
  $('#consola').textContent = [
    `${t('cEstado')}    ${manualMode ? t('cManual') : session.nav.state} · t ${d.seconds.toFixed(1)} s · ${running ? t('cCorriendo') : t('cDetenido')}`,
    `${t('cPoseCreida')}   x ${b.x.toFixed(2)} · z ${b.z.toFixed(2)} · yaw ${(b.yaw * 180 / Math.PI).toFixed(0)}°`,
    `${t('cPoseReal')} x ${p.x.toFixed(2)} · z ${p.z.toFixed(2)} · ${t('cError')} ${d.positionError.toFixed(3)} m`,
    lidar,
    t('cRuedas', { izq: w.izq.toFixed(2), der: w.der.toFixed(2), avance: w.avance.toFixed(2), giro: (w.giro * 180 / Math.PI).toFixed(0) }),
    t('cRecorrido', { distancia: d.distance.toFixed(2), contactos: d.contacts, paradas: d.groundStops }),
    t('cMapa', { libres: c.libre, ocupadas: c.ocupado, pct: session.nav.summary().conocidas_pct }),
    goal ? t('cDestino', { x: goal.x.toFixed(2), z: goal.z.toFixed(2), d: Math.hypot(goal.x - b.x, goal.z - b.z).toFixed(2) }) : t('cSinDestino'),
  ].join('\n');
}
function draw() {
  const p = session.pose, d = session.diagnostics, b = session.nav.belief, c = counts(session.nav.map);
  robot.position.set(p.x, session.world.floorY, p.z); robot.rotation.y = p.yaw; render(); drawMap();
  vistaDescubierta.update(session); drawLidar(); actualizarConsola(); actualizarDebug();
  drawGoalStatus();
  const fields = { [t('tEstadoMotor')]: session.nav.state, [t('tTiempo')]: `${d.seconds.toFixed(1)} s`, [t('tRecorrido')]: `${d.distance.toFixed(2)} m`, [t('tContactos')]: d.contacts, [t('tParadas')]: d.groundStops, [t('tError')]: `${d.positionError.toFixed(3)} m`, [t('tCeldas')]: `${c.libre} / ${c.ocupado}`, [t('tPosicion')]: `${b.x.toFixed(2)}, ${b.z.toFixed(2)} m` };
  if (manualMode) fields[t('tEstadoMotor')] = running ? t('tManual') : t('tManualDetenido');
  $('#telemetry').replaceChildren(...Object.entries(fields).flatMap(([label, value]) => { const dt = document.createElement('dt'), dd = document.createElement('dd'); dt.textContent = label; dd.textContent = value; return [dt, dd]; }));
}
function tick() {
  frame = null; if (!running) return;
  for (let i = 0; i < (manualMode ? 1 : Number($('#speed').value)); i++) {
    let result;
    if (!manualMode && aiTarget && !aiTarget.aligned && (aiTarget.aligning || session.nav.state === 'destino_alcanzado')) {
      aiTarget.aligning = true;
      const turn = faceObjectCommand(session.nav.belief, aiTarget);
      if (!turn) { aiTarget.aligned = true; aiTarget.aligning = false; result = { stopped: true, groundStop: false }; }
      else result = session.step(turn);
    } else result = session.step(manualMode ? driveCommand() : null);
    if (result.stopped && aiTarget && !aiTarget.aligned && !manualMode && session.nav.state === 'destino_alcanzado' && !result.groundStop) continue;
    if (result.stopped) { running = false; groundStopped = result.groundStop; clearDriveInputs(); break; }
  }
  draw();
  if (!running) { updateDriveControls(); if (session.nav.state === 'limitado') mostrarCierre(); $('#nav-status').textContent = groundStopped ? t('sueloDetenido') : aiTarget?.aligned ? `Aproximación completada: mirando hacia ${aiTarget.label}. Orientación funcional del objeto sin determinar.` : session.nav.state === 'destino_alcanzado' ? t('destinoAlcanzadoStatus') : t('pruebaDetenida', { estado: session.nav.state }); }
  else {
    const escapeMessages = { giro: 'orientandoSalida', observando: 'observandoSalida', avance: 'saliendoBloqueo' };
    $('#nav-status').textContent = session.nav.state === 'retroceso'
      ? t(escapeMessages[session.nav.explorer.escape?.phase] ?? (session.nav.explorer.recoveryCause === 'suelo' ? 'sueloRecuperando' : 'recuperandoRuta'))
      : session.nav.destination ? t('continuando') : t('explorando');
    frame = requestAnimationFrame(tick);
  }
}
let cierreMostrado = false;
const fotosZonas = crearRegistroFotosZonas();
try { $('#cierre-fotos').checked = localStorage.getItem('homemind-cierre-fotos') === '1'; } catch { /* sin persistencia */ }
$('#cierre-fotos').addEventListener('change', () => {
  try { localStorage.setItem('homemind-cierre-fotos', $('#cierre-fotos').checked ? '1' : '0'); } catch { /* sin persistencia */ }
  if ($('#cierre-fotos').checked) { cierreMostrado = false; mostrarCierre(); }
  else $('#cierre-panel').hidden = true;
});
// Camera tilt calibration, shared by the live robot view and every
// simulated capture (closure photos and the AI panel use the same mount).
const pitchInput = $('#camera-pitch');
pitchInput.value = String(cameraPitchDeg);
$('#camera-pitch-value').textContent = `${cameraPitchDeg}°`;
pitchInput.addEventListener('input', () => {
  cameraPitchDeg = Number(pitchInput.value);
  $('#camera-pitch-value').textContent = `${cameraPitchDeg}°`;
  try { localStorage.setItem('homemind-camera-pitch', String(cameraPitchDeg)); } catch { /* sin persistencia */ }
  cameraSensor?.setPitch(cameraPitchDeg);
  if (viewMode === 'robot') render();
});
// Both views are simulated snapshots; the proposed pose never moves the robot.
function fotoZona(centro) {
  const belief = { ...session.nav.belief }, floorY = session.world.floorY;
  const punto = elegirPuntoDeFoto(session.nav.map, belief, centro, { ancho: (session.world.radius ?? .1) * 2 });
  camera.up.set(0, 0, -1); camera.fov = 45; camera.updateProjectionMatrix();
  camera.position.set(centro.x, floorY + 3.4, centro.z + .01);
  camera.lookAt(centro.x, floorY, centro.z);
  renderer.render(scene, camera);
  const top = renderer.domElement.toDataURL('image/jpeg', .85);
  const captura = cameraSensor.capture({ x: punto.x, z: punto.z, yaw: punto.yaw }, floorY, belief, session.diagnostics.seconds);
  view(viewMode); render();
  return { top, pov: captura.image, punto, belief, centro: { ...centro }, tiempo: captura.time, calibracion: { ...captura.calibration } };
}
function figuraCierre(src, caption, alt) {
  const figura = document.createElement('figure');
  figura.className = 'cierre-fig';
  const img = document.createElement('img');
  img.className = 'cierre-foto'; img.src = src; img.alt = alt;
  const leyenda = document.createElement('figcaption');
  leyenda.textContent = caption;
  figura.append(img, leyenda);
  return figura;
}
// Geometric projection diagram over the believed map: the advance
// corridor exactly the element's width (believed position → photo
// point — the only thing allowed to trace a route), the scanning cone
// that validates it (80° FOV, detection only), the zone cells in red
// and the interfering object as a red cross.
function dibujarProyeccion(canvas, celdas, centro, punto, belief) {
  const map = session.nav.map, g = map.grid, b = belief;
  const ctx = canvas.getContext('2d'), W = canvas.width, H = canvas.height;
  const refs = [b, punto, centro, ...(punto.interferencia ? [punto.interferencia] : [])];
  const margen = .7;
  const minX = Math.min(...refs.map(p => p.x)) - margen, maxX = Math.max(...refs.map(p => p.x)) + margen;
  const minZ = Math.min(...refs.map(p => p.z)) - margen, maxZ = Math.max(...refs.map(p => p.z)) + margen;
  const escala = Math.min(W / (maxX - minX), H / (maxZ - minZ));
  const px = x => (x - minX) * escala, py = z => (z - minZ) * escala, celdaPx = g.resolution * escala;
  ctx.fillStyle = '#cbd5cf'; ctx.fillRect(0, 0, W, H);
  const col0 = Math.max(0, Math.floor((minX - g.originX) / g.resolution)), col1 = Math.min(g.width - 1, Math.ceil((maxX - g.originX) / g.resolution));
  const row0 = Math.max(0, Math.floor((minZ - g.originZ) / g.resolution)), row1 = Math.min(g.height - 1, Math.ceil((maxZ - g.originZ) / g.resolution));
  for (let row = row0; row <= row1; row++) for (let col = col0; col <= col1; col++) {
    const i = row * g.width + col;
    if (map[i] === UNKNOWN) continue;
    ctx.fillStyle = map[i] === FREE ? '#fbfcf8' : '#41675a';
    ctx.fillRect(px(g.originX + col * g.resolution), py(g.originZ + row * g.resolution), celdaPx + .5, celdaPx + .5);
  }
  ctx.fillStyle = 'rgba(221,84,62,.9)';
  for (const i of celdas) { const p = center(i, map); ctx.fillRect(px(p.x) - celdaPx / 2, py(p.z) - celdaPx / 2, celdaPx, celdaPx); }
  const dir = yaw => ({ x: -Math.sin(yaw), z: -Math.cos(yaw) });
  const mitadFov = THREE.MathUtils.degToRad(cameraHorizontalFov / 2), largo = Math.min(punto.distanciaLibre, punto.distancia + .3);
  const d1 = dir(punto.yaw - mitadFov), d2 = dir(punto.yaw + mitadFov);
  ctx.beginPath(); ctx.moveTo(px(punto.x), py(punto.z));
  ctx.lineTo(px(punto.x + d1.x * largo), py(punto.z + d1.z * largo));
  ctx.lineTo(px(punto.x + d2.x * largo), py(punto.z + d2.z * largo));
  ctx.closePath(); ctx.fillStyle = 'rgba(236,200,60,.35)'; ctx.fill();
  // Advance corridor: a rectangle of exactly the element's width —
  // never the scanning cone — from the believed position to the photo
  // point. That swept width is the route; the cone only detects.
  const ancho = (session.world.radius ?? .1) * 2, avanceLen = Math.hypot(punto.x - b.x, punto.z - b.z);
  if (avanceLen > 1e-6) {
    const ex = (punto.x - b.x) / avanceLen, ez = (punto.z - b.z) / avanceLen, ox = -ez * ancho / 2, oz = ex * ancho / 2;
    ctx.beginPath();
    ctx.moveTo(px(b.x + ox), py(b.z + oz)); ctx.lineTo(px(punto.x + ox), py(punto.z + oz));
    ctx.lineTo(px(punto.x - ox), py(punto.z - oz)); ctx.lineTo(px(b.x - ox), py(b.z - oz));
    ctx.closePath(); ctx.fillStyle = 'rgba(34,138,197,.30)'; ctx.fill();
    ctx.strokeStyle = 'rgba(34,138,197,.8)'; ctx.lineWidth = 1.5; ctx.stroke();
  }
  if (punto.interferencia) {
    const ix = px(punto.interferencia.x), iy = py(punto.interferencia.z), r = Math.max(6, .12 * escala);
    ctx.beginPath(); ctx.moveTo(ix - r, iy - r); ctx.lineTo(ix + r, iy + r); ctx.moveTo(ix + r, iy - r); ctx.lineTo(ix - r, iy + r);
    ctx.strokeStyle = '#d42a1e'; ctx.lineWidth = 3; ctx.stroke();
  }
  ctx.beginPath(); ctx.arc(px(b.x), py(b.z), 5, 0, Math.PI * 2); ctx.fillStyle = '#ec8d42'; ctx.fill(); ctx.strokeStyle = '#101613'; ctx.lineWidth = 1.5; ctx.stroke();
  ctx.beginPath(); ctx.arc(px(punto.x), py(punto.z), 5, 0, Math.PI * 2); ctx.fillStyle = '#173f3d'; ctx.fill(); ctx.strokeStyle = '#fff'; ctx.stroke();
}
function mostrarCierre() {
  if (!$('#cierre-fotos').checked || cierreMostrado || !session || session.nav.state !== 'limitado') return;
  cierreMostrado = true;
  const map = session.nav.map, g = map.grid;
  const revisadas = new Set(fotosZonas.exportar().filter(r => r.decision).flatMap(r => r.celdas));
  const pendientes = new Set(session.nav.explorer.fronts.filter(i => !revisadas.has(i)));
  const zonas = [];
  while (pendientes.size) {
    const semilla = pendientes.values().next().value, cola = [semilla], celdas = [];
    pendientes.delete(semilla);
    while (cola.length) {
      const actual = cola.pop(); celdas.push(actual);
      const x = actual % g.width, z = Math.floor(actual / g.width);
      for (let dz = -1; dz <= 1; dz++) for (let dx = -1; dx <= 1; dx++) {
        const n = (z + dz) * g.width + (x + dx);
        if (x + dx >= 0 && z + dz >= 0 && x + dx < g.width && z + dz < g.height && pendientes.has(n)) { pendientes.delete(n); cola.push(n); }
      }
    }
    if (celdas.length >= 2) zonas.push(celdas);
  }
  if (!zonas.length) return;
  $('#cierre-panel').hidden = false;
  const lista = $('#cierre-lista');
  lista.replaceChildren();
  zonas.forEach((celdas, indice) => {
    const puntos = celdas.map(i => center(i, map));
    const centro = { x: puntos.reduce((a, p) => a + p.x, 0) / puntos.length, z: puntos.reduce((a, p) => a + p.z, 0) / puntos.length };
    let pegados = 0, total = 0;
    for (const i of celdas) {
      const x = i % g.width, z = Math.floor(i / g.width);
      for (const [dx, dz] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const n = (z + dz) * g.width + (x + dx), x2 = x + 2 * dx, z2 = z + 2 * dz;
        if (x + dx < 0 || z + dz < 0 || x + dx >= g.width || z + dz >= g.height || map[n] !== UNKNOWN) continue;
        total++;
        if (x2 < 0 || z2 < 0 || x2 >= g.width || z2 >= g.height || map[z2 * g.width + x2] === OCCUPIED) pegados++;
      }
    }
    const propuesta = t(total > 0 && pegados === total ? 'cierrePropuestaSinPaso' : 'cierrePropuestaRevisar');
    const foto = fotosZonas.obtener(celdas, () => fotoZona(centro));
    if (foto.decision) return;
    const fila = document.createElement('div');
    fila.className = 'cierre-zona';
    const altZona = t('cierreZona', { n: indice + 1, celdas: celdas.length, x: centro.x.toFixed(1), z: centro.z.toFixed(1) });
    const figTop = figuraCierre(foto.top, t('cierreVistaTop'), altZona);
    const figPov = figuraCierre(foto.pov, t('cierreVistaPov'), altZona);
    const diagrama = document.createElement('canvas');
    diagrama.width = 300; diagrama.height = 210; diagrama.className = 'cierre-diagrama';
    dibujarProyeccion(diagrama, celdas, foto.centro, foto.punto, foto.belief);
    const figGeo = document.createElement('figure');
    figGeo.className = 'cierre-fig';
    const leyendaGeo = document.createElement('figcaption');
    leyendaGeo.textContent = `${t('cierreDiagrama')} · ${foto.punto.interferencia ? t('cierreInterferencia', { d: foto.punto.distanciaLibre.toFixed(1) }) : t('cierreSinInterferencia')}`;
    figGeo.append(diagrama, leyendaGeo);
    const texto = document.createElement('span');
    const puntoTexto = foto.punto.modo === 'proyectado'
      ? t('cierrePuntoFoto', { d: foto.punto.distancia.toFixed(1), libre: foto.punto.distanciaLibre.toFixed(1) })
      : t('cierrePuntoFotoDesde', { d: foto.punto.distancia.toFixed(1) });
    texto.textContent = `${altZona} · ${puntoTexto} · ${propuesta} · x ${foto.punto.x.toFixed(2)} z ${foto.punto.z.toFixed(2)} yaw ${foto.punto.yaw.toFixed(2)} rad · ${foto.tiempo.toFixed(1)} s · pitch ${foto.calibracion.pitchDeg}°`;
    const ver = document.createElement('button'); ver.textContent = t('cierreVer');
    ver.addEventListener('click', () => { view('plan'); controls.target.set(centro.x, session.world.floorY, centro.z); camera.position.set(centro.x, session.world.floorY + 3.4, centro.z + .01); controls.update(); render(); });
    const cerrar = document.createElement('button'); cerrar.textContent = t('cierreCerrar');
    cerrar.addEventListener('click', () => {
      fotosZonas.decidir(celdas, 'sin-paso');
      session.nav.closeFrontiers(celdas);
      fila.remove();
      if (session.nav.state === 'completado') { $('#cierre-panel').hidden = true; $('#nav-status').textContent = t('cierreCompletado'); }
      else $('#nav-status').textContent = t('cierreQuedan', { n: lista.childElementCount });
      draw();
    });
    const paso = document.createElement('button'); paso.textContent = t('cierreHayPaso');
    paso.addEventListener('click', () => { fotosZonas.decidir(celdas, 'hay-paso'); fila.remove(); if (!lista.childElementCount) $('#cierre-panel').hidden = true; });
    fila.append(figTop, figPov, figGeo, texto, ver, cerrar, paso);
    lista.append(fila);
  });
}
function reset() { cierreMostrado = false; fotosZonas.limpiar(); const panel = $('#cierre-panel'); if (panel) panel.hidden = true;
  running = false; if (frame !== null) cancelAnimationFrame(frame); frame = null; groundStopped = false;
  if (manualMode) $('#speed').value = automaticSpeed;
  manualMode = false; $('#speed').disabled = false; clearDriveInputs();
  requestedRoute = [];
  goalError = null;
  aiTarget = null; aiPanel?.reset();
  session = createScanSession(geometry, { scale: calibration?.scale ?? 1, noisy: $('#noise').checked });
  $('#toggle').textContent = t('explorar'); $('#nav-status').textContent = t('listo');
  const stats = session.world.stats;
  $('#geometry-info').textContent = t('geometriaInfo', { suelo: stats.freeAreaM2.toFixed(2), obstaculos: stats.occupiedAreaM2.toFixed(2), piso: stats.floorY.toFixed(3) });
  $('#scale-info').textContent = calibration ? t('escalaCalibrada', { factor: calibration.scale.toFixed(4) }) : t('escalaSin');
  draw();
}
$('#toggle').addEventListener('click', () => {
  if (manualMode) {
    manualMode = false; running = false; groundStopped = false;
    $('#speed').disabled = false; $('#speed').value = automaticSpeed; clearDriveInputs();
  } else if (session.nav.state === 'destino_alcanzado' || aiTarget?.aligned) {
    session.nav.setDestination(null); requestedRoute = []; goalError = null; aiTarget = null; running = false;
  } else if (groundStopped) reset();
  // A limited/blocked mission resumes from the believed map it already
  // built (startExplorer restarts those states); only a completed one
  // starts over. Resetting here used to wipe 40+ m of discovered map.
  else if (session.nav.terminal && !['limitado', 'bloqueado'].includes(session.nav.state)) reset();
  running = !running;
  if (running) { session.nav.start(); $('#toggle').textContent = t('pausar'); $('#nav-status').textContent = session.nav.destination ? t('continuando') : t('explorando'); if (frame === null) frame = requestAnimationFrame(tick); }
  else { session.nav.pause(); updateDriveControls(); $('#nav-status').textContent = t('pausada'); draw(); }
});
$('#reset').addEventListener('click', reset); $('#noise').addEventListener('change', reset); $('#reference').addEventListener('change', drawMap);
$('#top').addEventListener('click', () => view('plan')); $('#orbit').addEventListener('click', () => view('orbit'));
$('#robot-camera').addEventListener('click', () => view('robot'));
function drawGoalStatus() {
  const goal = session.nav.destination;
  $('#cancel-goal').disabled = !goal;
  if (goalError) { $('#goal-status').textContent = goalError; return; }
  if (!goal) { $('#goal-status').textContent = t('sinDestino'); return; }
  const distance = Math.hypot(goal.x - session.nav.belief.x, goal.z - session.nav.belief.z);
  const state = aiTarget?.aligned ? 'Aproximación y mirada completadas' : aiTarget?.aligning ? 'Orientándose hacia el objeto observado' : session.nav.state === 'destino_alcanzado' ? t('llegoDestino') : !running ? t('destinoEnPausa') : t('navegandoDestino');
  $('#goal-status').textContent = t('destinoDistancia', { estado: state, x: goal.x.toFixed(2), z: goal.z.toFixed(2), d: distance.toFixed(2) });
}
$('#known-map').addEventListener('click', e => {
  if (!session || aiBusy) return;
  try {
    const point = destinationFromPointer(e.clientX, e.clientY, e.currentTarget.getBoundingClientRect(), session.nav.map.grid);
    navigateToPoint(point);
  } catch (error) { goalError = error.message; drawGoalStatus(); }
});
function navigateToPoint(point, object = null) {
  const selected = selectMapDestination(session.nav, point, session.world.radius);
  goalError = null; requestedRoute = selected.route; aiTarget = object;
  manualMode = false; $('#speed').disabled = false; clearDriveInputs();
  groundStopped = false; running = true;
  $('#nav-status').textContent = object ? t('navegandoObjeto', { label: object.label }) : t('destinoColocado');
  view('robot'); draw(); updateDriveControls();
  if (frame === null) frame = requestAnimationFrame(tick);
}
$('#cancel-goal').addEventListener('click', () => {
  session.nav.setDestination(null); requestedRoute = []; goalError = null; aiTarget = null; stopDriving(); drawGoalStatus();
});
$('#manual-mode').addEventListener('click', enterManual);
$('#dbg-left').addEventListener('click', () => ejecutarComandoDebug({ left: true }, 30, 'girar izquierda 45°'));
$('#dbg-right').addEventListener('click', () => ejecutarComandoDebug({ right: true }, 30, 'girar derecha 45°'));
$('#dbg-forward').addEventListener('click', () => ejecutarComandoDebug({ forward: true }, 15, 'avanzar 0,25 m'));
$('#dbg-back').addEventListener('click', () => ejecutarComandoDebug({ back: true }, 15, 'retroceder 0,25 m'));
$('#dbg-scan').addEventListener('click', () => ejecutarComandoDebug({ left: true }, 240, 'barrido 360°'));
$('#dbg-replan').addEventListener('click', replanificarDebug);
$('#dbg-diagnose').addEventListener('click', () => { if (session) { draw(); actualizarDebug(true); } });
$('#drive-stop').addEventListener('click', stopDriving);
for (const button of document.querySelectorAll('[data-drive]')) {
  button.addEventListener('pointerdown', e => {
    if (e.button !== 0) return;
    e.preventDefault(); enterManual(); button.setPointerCapture(e.pointerId);
    driveInputs.set(`pointer:${e.pointerId}`, button.dataset.drive); updateDriveControls();
  });
  for (const event of ['pointerup', 'pointercancel', 'lostpointercapture']) button.addEventListener(event, e => {
    driveInputs.delete(`pointer:${e.pointerId}`); updateDriveControls();
  });
}
const driveKeys = { w: 'forward', arrowup: 'forward', s: 'back', arrowdown: 'back', a: 'left', arrowleft: 'left', d: 'right', arrowright: 'right' };
window.addEventListener('keydown', e => {
  if (!session || aiBusy || e.target.closest('input, select, textarea, [contenteditable="true"]') || e.ctrlKey || e.metaKey || e.altKey) return;
  const key = e.key.toLowerCase();
  if (key === ' ' || key === 'escape') { e.preventDefault(); stopDriving(); return; }
  if (!driveKeys[key]) return;
  e.preventDefault(); if (e.repeat) return;
  enterManual(); driveInputs.set(`key:${key}`, driveKeys[key]); updateDriveControls();
});
window.addEventListener('keyup', e => { driveInputs.delete(`key:${e.key.toLowerCase()}`); updateDriveControls(); });
window.addEventListener('blur', () => { if (manualMode) stopDriving(); });
document.addEventListener('visibilitychange', () => { if (document.hidden && manualMode) stopDriving(); });
$('#export').addEventListener('click', () => {
  const data = { ...session.nav.saveMap(), resumen: session.nav.summary(), capturasZonas: fotosZonas.exportar(), captura: '10_28_2026-varias-habitaciones.glb', calibracion: calibration, observador: { ...session.diagnostics, geometria: session.world.stats, trayectoria: session.trail }, sensoresSimulados: true };
  const href = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' })), link = document.createElement('a'); link.href = href; link.download = 'homemind-prueba-polycam.json'; link.click(); setTimeout(() => URL.revokeObjectURL(href), 1000);
});
for (const boton of document.querySelectorAll('[data-lang]')) boton.addEventListener('click', () => {
  aplicarIdioma(boton.dataset.lang);
  updateDriveControls();
  if (session) { drawGoalStatus(); draw(); }
});
aplicarIdioma();
const observerCut = new THREE.Plane(new THREE.Vector3(0, -1, 0), 1.4);
aiPanel = createAiPanel({ session: () => session,
  capture: () => cameraSensor.capture(session.pose, session.world.floorY, session.nav.belief, session.diagnostics.seconds),
  stop: stopDriving, navigate: navigateToPoint, render: draw,
  busyChanged(busy) {
    aiBusy = busy;
    for (const id of ['toggle', 'reset', 'noise', 'manual-mode', 'drive-stop', 'speed', 'ai-analyze', 'ai-instruction', 'cancel-goal', 'dbg-left', 'dbg-right', 'dbg-forward', 'dbg-back', 'dbg-scan', 'dbg-replan', 'dbg-diagnose']) $(`#${id}`).disabled = busy;
    for (const button of document.querySelectorAll('[data-drive]')) button.disabled = busy;
    if (!busy) { $('#speed').disabled = manualMode; drawGoalStatus(); }
  },
});
async function initialize() {
  const response = await fetch(url); if (!response.ok) throw new Error('No se pudo leer el GLB'); const bytes = await response.arrayBuffer();
  geometry = extractGlbGeometry(bytes); calibration = readCalibration();
  const gltf = await new GLTFLoader().parseAsync(bytes, ''), box = new THREE.Box3().setFromObject(gltf.scene), center = box.getCenter(new THREE.Vector3()), scale = calibration?.scale ?? 1;
  const room = new THREE.Group(); room.add(gltf.scene); room.scale.setScalar(scale); room.position.set(-center.x * scale, -box.min.y * scale, -center.z * scale);
  gltf.scene.traverse(mesh => { if (!mesh.isMesh) return; const original = mesh.material; mesh.material = new THREE.MeshBasicMaterial({ map: original.map, color: original.color, side: THREE.DoubleSide, clippingPlanes: [observerCut] }); captureMaterials.push(mesh.material); });
  scene.add(room); cameraSensor = createRobotCameraSensor({ renderer, scene, room, robot, materials: captureMaterials, pitchDeg: cameraPitchDeg });
  reset(); view(); for (const id of ['toggle', 'reset', 'export', 'top', 'orbit', 'robot-camera', 'manual-mode', 'drive-stop', 'ai-analyze', 'dbg-left', 'dbg-right', 'dbg-forward', 'dbg-back', 'dbg-scan', 'dbg-replan', 'dbg-diagnose']) $(`#${id}`).disabled = false;
  for (const button of document.querySelectorAll('[data-drive]')) button.disabled = false;
  // Start exploring on load: the engine is expected to traverse the map,
  // not to sit waiting for a first click.
  $('#toggle').click();
}
initialize().catch(error => { $('#nav-status').textContent = error.message; console.error(error); });
