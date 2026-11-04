import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CALIBRATION_KEY, tileCalibration, readCalibration } from './scan-calibration.js';

const $ = selector => document.querySelector(selector);
const canvas = $('#scan-canvas'), viewport = $('#scan-viewport');
const modelUrl = '/scans/polycam-2026-10-28/10_28_2026-varias-habitaciones.glb';
const number = new Intl.NumberFormat('es', { maximumFractionDigits: 0 });
const meters = value => value.toLocaleString('es', { minimumFractionDigits: 2, maximumFractionDigits: 2 });
let renderer, controls, camera, scene, grid, room, dimensions, materials = [], mode = 'orbit', queued = false;
let originalDimensions, offset, scale = 1, measuring = false, points = [], pointerStart;
const markers = new THREE.Group();
function applyScale(calibration) {
  scale = calibration?.scale ?? 1;
  room.scale.setScalar(scale); room.position.copy(offset).multiplyScalar(scale);
  dimensions.copy(originalDimensions).multiplyScalar(scale);
  $('#scan-dimensions').textContent = `${meters(dimensions.x)} × ${meters(dimensions.z)} m`;
  $('#scan-height').textContent = `Altura capturada: ${meters(dimensions.y)} m`;
  $('#cut-height').max = String(dimensions.y);
  $('#scale-status').textContent = calibration ? `Ajuste por baldosa de 32,7 cm sin junta · factor ${scale.toFixed(4)}. Precisión limitada por los puntos y la malla.` : 'Escala aproximada del archivo. Referencia real: baldosa de 32,7 cm sin junta.';
  setView(mode);
}
const cutPlane = new THREE.Plane(new THREE.Vector3(0, -1, 0), 1.9);

function requestRender() {
  if (queued || !renderer) return;
  queued = true;
  requestAnimationFrame(() => { queued = false; controls.update(); renderer.render(scene, camera); });
}
function resize() {
  const width = viewport.clientWidth, height = viewport.clientHeight;
  renderer.setSize(width, height, false); camera.aspect = width / height; camera.updateProjectionMatrix(); requestRender();
}
function applyAppearance() {
  for (const pair of materials) {
    const next = $('#show-textures').checked ? pair.photo : pair.surface;
    next.clippingPlanes = $('#cut-enabled').checked ? [cutPlane] : [];
    next.needsUpdate = true;
    pair.mesh.material = next;
  }
  cutPlane.constant = Number($('#cut-height').value);
  $('#cut-value').textContent = `${meters(cutPlane.constant)} m`;
  $('#cut-height').disabled = !room || !$('#cut-enabled').checked;
  if (grid) grid.visible = $('#show-grid').checked;
  const label = { orbit: 'ÓRBITA', inside: 'INTERIOR', plan: 'PLANTA', complete: 'MODELO COMPLETO' }[mode];
  $('#view-label').textContent = `${label} · ${$('#cut-enabled').checked ? 'CORTE VISUAL' : 'CAPTURA COMPLETA'}`;
  requestRender();
}
function setView(next) {
  if (!room) return;
  mode = next;
  camera.fov = next === 'inside' ? 75 : 45;
  camera.updateProjectionMatrix();
  for (const id of ['orbit', 'inside', 'plan', 'complete']) {
    $(`#view-${id}`).classList.toggle('selected', next === id);
    $(`#view-${id}`).setAttribute('aria-pressed', String(next === id));
  }
  const radius = dimensions.length() / 2;
  const fit = radius / Math.sin(THREE.MathUtils.degToRad(camera.fov / 2)) * Math.max(1, 1 / camera.aspect) * 1.1;
  camera.up.set(0, 1, 0); controls.enableRotate = true; controls.enablePan = true;
  controls.minDistance = .08; controls.maxDistance = radius * 12;
  if (next === 'inside') {
    $('#cut-enabled').checked = false;
    const height = Math.min(1.5, dimensions.y * .6);
    camera.position.set(-dimensions.x * .2, height, dimensions.z * .35);
    controls.target.set(dimensions.x * .15, dimensions.y * .36, -dimensions.z * .15);
  } else if (next === 'plan') {
    $('#cut-enabled').checked = true;
    $('#cut-height').value = String(Math.min(1.4, dimensions.y * .65));
    controls.enableRotate = false; camera.up.set(0, 0, -1);
    const tangent = Math.tan(THREE.MathUtils.degToRad(camera.fov / 2));
    const planDistance = Number($('#cut-height').value) + Math.max(dimensions.z / (2 * tangent), dimensions.x / (2 * tangent * camera.aspect)) * 1.18;
    controls.target.set(0, 0, 0); camera.position.set(0, planDistance, 0);
  } else {
    $('#cut-enabled').checked = next === 'orbit';
    if (next === 'orbit') $('#cut-height').value = String(Math.min(1.9, dimensions.y * .82));
    controls.target.set(0, dimensions.y * .35, 0);
    camera.position.copy(new THREE.Vector3(1, 1.05, 1.25).normalize().multiplyScalar(fit).add(controls.target));
  }
  controls.update(); applyAppearance();
}
function showError(error) {
  $('#loading').hidden = false;
  $('#loading').textContent = `No se pudo abrir la captura: ${error.message}.`;
  $('#scan-status').textContent = 'Error al cargar el modelo';
  console.error(error);
}
async function initialize() {
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.localClippingEnabled = true;
  scene = new THREE.Scene(); scene.background = new THREE.Color('#e4ebe5');
  camera = new THREE.PerspectiveCamera(45, 1, .01, 200);
  controls = new OrbitControls(camera, canvas); controls.enableDamping = false;
  controls.addEventListener('change', requestRender);
  scene.add(new THREE.HemisphereLight('#ffffff', '#748478', 2));
  const sun = new THREE.DirectionalLight('#ffffff', 2); sun.position.set(3, 7, 4); scene.add(sun);
  new ResizeObserver(resize).observe(viewport); resize();

  const gltf = await new GLTFLoader().loadAsync(modelUrl, event => {
    if (event.lengthComputable) $('#loading').textContent = `Cargando captura · ${Math.round(event.loaded / event.total * 100)} %`;
  });
  const originalBox = new THREE.Box3().setFromObject(gltf.scene);
  dimensions = originalBox.getSize(new THREE.Vector3());
  if (![dimensions.x, dimensions.y, dimensions.z].every(v => Number.isFinite(v) && v > 0)) throw new Error('geometría vacía o dimensiones inválidas');
  const center = originalBox.getCenter(new THREE.Vector3());
  room = new THREE.Group(); room.add(gltf.scene);
  // Display normalization only. The GLB remains unchanged; minimum Y is a
  // framing reference, not a detected or robot-validated ground plane.
  offset = new THREE.Vector3(-center.x, -originalBox.min.y, -center.z);
  originalDimensions = dimensions.clone();
  room.position.copy(offset); scene.add(room); scene.add(markers);
  let triangles = 0, meshes = 0;
  const textures = new Set();
  gltf.scene.traverse(mesh => {
    if (!mesh.isMesh) return;
    if (Array.isArray(mesh.material)) throw new Error('material múltiple no previsto en esta captura');
    const original = mesh.material;
    if (original.map) textures.add(original.map.uuid);
    if (!mesh.geometry.attributes.normal) mesh.geometry.computeVertexNormals();
    const photo = new THREE.MeshBasicMaterial({ map: original.map, color: original.color, side: THREE.DoubleSide, transparent: original.transparent, opacity: original.opacity });
    const surface = new THREE.MeshStandardMaterial({ color: '#b8cec0', metalness: 0, roughness: .9, side: THREE.DoubleSide });
    // Polycam's photographs already contain illumination. Basic material
    // displays those colors directly rather than adding metallic reflections.
    materials.push({ mesh, photo, surface }); mesh.material = photo;
    triangles += (mesh.geometry.index?.count ?? mesh.geometry.attributes.position.count) / 3; meshes++;
  });
  const gridSize = Math.ceil(Math.max(dimensions.x, dimensions.z) * 1.6);
  grid = new THREE.GridHelper(gridSize, gridSize * 2, '#9ab5a1', '#cbd9cd'); grid.position.y = -.025; scene.add(grid);
  $('#scan-meshes').textContent = number.format(meshes);
  $('#scan-triangles').textContent = number.format(triangles);
  $('#scan-textures').textContent = `${textures.size} incorporadas`;
  $('#scan-dimensions').textContent = `${meters(dimensions.x)} × ${meters(dimensions.z)} m`;
  $('#scan-height').textContent = `Altura capturada: ${meters(dimensions.y)} m`;
  $('#cut-height').max = String(dimensions.y);
  $('#loading').hidden = true;
  $('#scan-status').textContent = 'Captura cargada · texturas originales';
  $('#save-render').disabled = false;
  $('#measure-tile').disabled = false;
  applyScale(readCalibration());
}
$('#measure-tile').addEventListener('click', () => {
  measuring = true; points = [];
  for (const marker of [...markers.children]) { markers.remove(marker); marker.geometry.dispose(); marker.material.dispose(); }
  setView('plan'); $('#measure-status').textContent = 'Selecciona el primer extremo del lado de una baldosa, sin incluir la junta.';
});
$('#reset-scale').addEventListener('click', () => { localStorage.removeItem(CALIBRATION_KEY); measuring = false; markers.clear(); if (room) applyScale(null); });
canvas.addEventListener('pointerdown', e => { pointerStart = { x: e.clientX, y: e.clientY }; });
canvas.addEventListener('pointerup', e => {
  if (!measuring || !pointerStart || Math.hypot(e.clientX - pointerStart.x, e.clientY - pointerStart.y) > 4) return;
  const rect = canvas.getBoundingClientRect(), ray = new THREE.Raycaster();
  ray.setFromCamera(new THREE.Vector2((e.clientX - rect.left) / rect.width * 2 - 1, -(e.clientY - rect.top) / rect.height * 2 + 1), camera);
  room.updateMatrixWorld(true);
  const hit = ray.intersectObject(room, true).find(h => h.point.y < .25 * scale && Math.abs(h.face.normal.y) > .8);
  if (!hit) { $('#measure-status').textContent = 'Ese punto no tiene suelo capturado. Selecciona el borde visible de una baldosa.'; return; }
  const p = hit.point.clone().divideScalar(scale); points.push({ x: p.x, y: p.y, z: p.z });
  const marker = new THREE.Mesh(new THREE.SphereGeometry(.018, 12, 8), new THREE.MeshBasicMaterial({ color: '#eb563a', depthTest: false }));
  marker.position.copy(hit.point); marker.renderOrder = 10; markers.add(marker); requestRender();
  if (points.length === 1) { $('#measure-status').textContent = 'Ahora el otro extremo del mismo lado de la baldosa, sin junta.'; return; }
  try {
    const calibration = tileCalibration(...points); localStorage.setItem(CALIBRATION_KEY, JSON.stringify(calibration));
    measuring = false; markers.clear(); applyScale(calibration);
    $('#measure-status').textContent = `Referencia guardada: ${(calibration.measuredCaptureM * 100).toFixed(1)} cm en la captura → 32,7 cm reales. Se aplicará también a la navegación.`;
  } catch (error) { points = []; $('#measure-status').textContent = error.message; }
});
for (const view of ['orbit', 'inside', 'plan', 'complete']) $(`#view-${view}`).addEventListener('click', () => setView(view));
for (const selector of ['#cut-enabled', '#cut-height', '#show-textures', '#show-grid']) $(selector).addEventListener('input', applyAppearance);
$('#save-render').addEventListener('click', () => {
  renderer.render(scene, camera);
  canvas.toBlob(blob => {
    if (!blob) return;
    const url = URL.createObjectURL(blob), link = document.createElement('a');
    link.href = url; link.download = `homemind-polycam-2026-10-28-${mode}.png`; link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }, 'image/png');
});
initialize().catch(showError);
