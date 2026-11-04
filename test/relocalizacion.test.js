import test from 'node:test';
import assert from 'node:assert/strict';
import { createNavigator } from '../src/navigation.js';
import { sense, SENSOR } from '../src/sensor.js';
import { SCENARIOS, solidos } from '../src/world.js';
import { advance, STEP } from '../src/physics.js';
import { createOdometry, readMotion, SENSORES_RUIDOSOS } from '../src/odometry.js';
import { mulberry32 } from '../src/noise.js';
import { OCCUPIED } from '../src/mapping.js';
import { detectObjects } from '../src/camera.js';
import { localizeGlobal } from '../src/scanmatch.js';

function drive(nav, scenario, { noisy = true, seed = 7, maxSteps = 24000, start = { x: 0, z: 0, yaw: 0 } } = {}) {
  const adapter = createOdometry(start, noisy ? { ...SENSORES_RUIDOSOS, seed } : { seed });
  const sensor = noisy ? { ...SENSOR, noise: .02, random: mulberry32((seed ^ 0x51f3) >>> 0) } : SENSOR;
  let pose = { ...start, collision: false }, motion = null, collisions = 0;
  nav.start();
  for (let step = 0; step < maxSteps && !nav.terminal; step++) {
    const rays = step % 6 === 0 ? sense(pose, scenario.walls, sensor) : null;
    const command = nav.step(STEP, { rays, motion });
    const prev = { x: pose.x, z: pose.z, yaw: pose.yaw };
    const next = advance(pose, command, STEP, solidos(scenario));
    motion = readMotion(adapter, next, prev, command, STEP, next.collision);
    collisions += Number(Boolean(next.collision));
    pose = next;
  }
  return { pose, collisions };
}

function savedPartition() {
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { noisy: false, seed: 7 });
  drive(nav, SCENARIOS.partition, { noisy: false });
  assert.equal(nav.state, 'completado');
  return nav.saveMap();
}

test('guardar y recargar un mapa previo con confianza decaída', () => {
  const saved = savedPartition();
  assert.equal(saved.formato, 'homemind-mapa');
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { noisy: false, priorMap: saved });
  assert.deepEqual(Array.from(nav.map), saved.celdas);
  const occupied = saved.celdas.findIndex(v => v === OCCUPIED);
  assert.ok(occupied >= 0);
  assert.equal(nav.map.logodds[occupied], .5);
  assert.ok(nav.summary().conocidas_pct > 50);
  assert.equal(nav.summary().mapa_previo, true);
  assert.throws(() => createNavigator(SCENARIOS.partition.mapConfig, { priorMap: { ...saved, grid: { ...saved.grid, width: 1 } } }), RangeError);
});

test('con mapa previo arranca relocalizándose solo y después explora', { timeout: 120000 }, () => {
  const saved = savedPartition();
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { noisy: false, priorMap: saved });
  const real = { x: 1.2, z: .9, yaw: -.7 };
  nav.start();
  assert.equal(nav.state, 'relocalizando');
  const { collisions } = drive(nav, SCENARIOS.partition, { noisy: false, start: real });
  assert.equal(nav.summary().relocalizacion.found, true, JSON.stringify(nav.summary().relocalizacion));
  assert.equal(nav.state, 'completado');
  assert.equal(collisions, 0);
});

test('la relocalización automática se puede desactivar', () => {
  const saved = savedPartition();
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { noisy: false, priorMap: saved, autoRelocalizar: false });
  nav.start();
  assert.equal(nav.state, 'barrido');
});

test('el mapa guardado lleva las marcas vistas dos o más veces, con posición', () => {
  const escenario = SCENARIOS.partition;
  const nav = createNavigator(escenario.mapConfig, { noisy: false, seed: 7 });
  const adapter = createOdometry({ x: 0, z: 0, yaw: 0 }, { seed: 7 });
  let pose = { x: 0, z: 0, yaw: 0, collision: false }, motion = null;
  nav.start();
  for (let step = 0; step < 24000 && !nav.terminal; step++) {
    const rays = step % 6 === 0 ? sense(pose, escenario.walls, SENSOR) : null;
    const detecciones = rays ? detectObjects(pose, escenario.boards, escenario.walls) : null;
    const command = nav.step(STEP, { rays, motion, detecciones });
    const prev = { ...pose };
    const next = advance(pose, command, STEP, solidos(escenario));
    motion = readMotion(adapter, next, prev, command, STEP, next.collision);
    pose = next;
  }
  assert.equal(nav.state, 'completado');
  const saved = nav.saveMap();
  assert.ok(Array.isArray(saved.marcas) && saved.marcas.length >= 2, JSON.stringify(saved.marcas));
  for (const board of escenario.boards) {
    const m = saved.marcas.find(m => m.label === board.label);
    assert.ok(m, `falta la marca ${board.label}: ${JSON.stringify(saved.marcas)}`);
    assert.ok(Math.hypot(m.x - board.x, m.z - board.z) < .6, `${board.label}: guardada ${m.x},${m.z} vs real ${board.x},${board.z}`);
  }
  // And they load back as prior landmarks for the next mission.
  const nav2 = createNavigator(escenario.mapConfig, { noisy: false, priorMap: saved, autoRelocalizar: false });
  assert.equal(nav2.summary().marcas_previas, saved.marcas.length);
});

test('la brújula recorta los rumbos de la búsqueda global', () => {
  const saved = savedPartition();
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { noisy: false, priorMap: saved, autoRelocalizar: false });
  const rays = sense({ x: 1.2, z: .9, yaw: -.7 }, SCENARIOS.partition.walls);
  const conBrujula = localizeGlobal(nav.map, rays, SENSOR, { yawCentro: -.7, yawTol: .45 });
  assert.ok(conBrujula.score >= .9, `score=${conBrujula.score}`);
  // One view still aliases across the partition wall: the truth must be
  // among the finalists (multi-view agreement picks it later), while a
  // compass-incompatible heading must produce no location at all.
  assert.ok((conBrujula.candidatos ?? []).some(c => Math.hypot(c.x - 1.2, c.z - .9) < .35), `candidatos: ${JSON.stringify(conBrujula.candidatos)}`);
  const contraBrujula = localizeGlobal(nav.map, rays, SENSOR, { yawCentro: 2.4, yawTol: .45 });
  assert.equal(contraBrujula.found, false, 'un rumbo incompatible con la brújula no debe ubicar');
});

test('sin acuerdo tras el primer ciclo, la relocalización activa hace una segunda pierna', { timeout: 120000 }, () => {
  const saved = savedPartition();
  const nav = createNavigator(SCENARIOS.empty.mapConfig, { noisy: false, priorMap: saved });
  const adapter = createOdometry({ x: 0, z: 0, yaw: 0 }, { seed: 7 });
  let pose = { x: 0, z: 0, yaw: 0, collision: false }, motion = null, piernas = 0, avanzando = false, estuvo = false;
  nav.start();
  for (let step = 0; step < 30000 && nav.state === 'relocalizando'; step++) {
    estuvo = true;
    const rays = step % 6 === 0 ? [] : null; // scans that explain nothing
    const command = nav.step(STEP, { rays, motion });
    if (command.forward && !avanzando) piernas++;
    avanzando = Boolean(command.forward);
    const prev = { ...pose };
    const next = advance(pose, command, STEP, solidos(SCENARIOS.empty));
    motion = readMotion(adapter, next, prev, command, STEP, next.collision);
    pose = next;
  }
  assert.ok(estuvo);
  assert.notEqual(nav.state, 'relocalizando');
  assert.equal(nav.summary().relocalizacion.found, false);
  assert.ok(piernas >= 2, `piernas de avance durante la búsqueda: ${piernas}`);
});

test('el adaptador de odometría emite brújula absoluta como lectura de sensor', () => {
  const exacto = createOdometry({ x: 0, z: 0, yaw: 0 }, { seed: 3 });
  const lectura = readMotion(exacto, { x: 0, z: 0, yaw: .8 }, { x: 0, z: 0, yaw: .8 }, { forward: false, back: false, left: false, right: false }, STEP, false);
  assert.ok(Math.abs(lectura.brujula - .8) < 1e-9, `brújula exacta: ${lectura.brujula}`);
  const ruidoso = createOdometry({ x: 0, z: 0, yaw: 0 }, { ...SENSORES_RUIDOSOS, seed: 3 });
  for (let i = 0; i < 50; i++) {
    const l = readMotion(ruidoso, { x: 0, z: 0, yaw: .8 }, { x: 0, z: 0, yaw: .8 }, { forward: false, back: false, left: false, right: false }, STEP, false);
    assert.ok(Math.abs(Math.atan2(Math.sin(l.brujula - .8), Math.cos(l.brujula - .8))) < .3, `brújula ruidosa fuera de tolerancia: ${l.brujula}`);
  }
});

test('sin mapa útil no inventa una pose', () => {
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { noisy: false });
  const found = nav.relocalize(sense({ x: 0, z: 0, yaw: 0 }, SCENARIOS.partition.walls));
  assert.equal(found.found, false);
  assert.deepEqual(nav.belief, { x: 0, z: 0, yaw: 0 });
});

test('con mapa previo y relocalización automática, la misión ruidosa desplazada termina localizada', { timeout: 180000 }, () => {
  const a = createNavigator(SCENARIOS.partition.mapConfig, { seed: 7 });
  drive(a, SCENARIOS.partition, {});
  const saved = a.saveMap();
  const real = { x: 1.2, z: .9, yaw: -.7 };
  const nav = createNavigator(SCENARIOS.partition.mapConfig, { seed: 8, priorMap: saved });
  const { pose, collisions } = drive(nav, SCENARIOS.partition, { seed: 8, start: real });
  assert.equal(nav.summary().relocalizacion.found, true, JSON.stringify(nav.summary().relocalizacion));
  assert.equal(collisions, 0);
  assert.ok(Math.hypot(nav.belief.x - pose.x, nav.belief.z - pose.z) < .35, JSON.stringify(nav.summary()));
});
