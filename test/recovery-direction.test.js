import test from 'node:test';
import assert from 'node:assert/strict';
import { createMap, FREE, OCCUPIED } from '../src/mapping.js';
import { createExplorer, exploreStep, routeAfterRetreat, routeAfterRecovery } from '../src/explorer.js';
import { predictMotion, STEP } from '../src/motion.js';

for (const yaw of [0, Math.PI / 2, -Math.PI / 2, Math.PI]) test(`salir de pared detras orientacion ${yaw}`, () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  const backward = { x: Math.sin(yaw), z: Math.cos(yaw) };
  const pose0 = { x: backward.x * -.045, z: backward.z * -.045, yaw };
  const axis = Math.abs(backward.x) > .5 ? 'x' : 'z', positive = backward[axis] > 0;
  const stripe = positive ? 22 : 17;
  for (let j = 0; j < 40; j++) map[axis === 'x' ? j * 40 + stripe : stripe * 40 + j] = OCCUPIED;
  const occupied = axis === 'x' ? 20 * 40 + stripe : stripe * 40 + 20;
  const e = createExplorer(.25); e.state = 'destino'; e.destino = { x: pose0.x - backward.x, z: pose0.z - backward.z }; e.route = [occupied];
  let pose = { ...pose0 }, back = 0, advance = 0;
  for (let i = 0; i < 400 && !['limitado', 'destino_alcanzado'].includes(e.state); i++) {
    const cmd = exploreStep(e, map, pose, STEP, false, false, true);
    back += Number(Boolean(cmd.back)); advance += Number(Boolean(cmd.forward));
    pose = predictMotion(pose, cmd, STEP);
  }
  assert.ok(advance > 0, `estado=${e.state}, back=${back}, advance=${advance}, yaw=${pose.yaw}`);
  assert.ok(Math.hypot(pose.x - pose0.x, pose.z - pose0.z) > .1);
});

test('espera una observacion nueva alineada antes de salir', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE); for (let x = 0; x < 40; x++) map[22 * 40 + x] = OCCUPIED;
  let pose = { x: 0, z: -.045, yaw: .08 }, e = createExplorer(.25);
  e.state = 'destino'; e.destino = { x: 0, z: -1 }; e.route = [22 * 40 + 20];
  let cmd = exploreStep(e, map, pose, STEP, false, false, false);
  for (let i = 0; i < 100 && e.escape?.phase !== 'observando'; i++) {
    assert.equal(Boolean(cmd.forward), false); assert.equal(Boolean(cmd.back), false);
    pose = predictMotion(pose, cmd, STEP);
    cmd = exploreStep(e, map, pose, STEP, false, false, false);
  }
  assert.equal(e.escape?.phase, 'observando');
  for (let i = 0; i < 30; i++) {
    cmd = exploreStep(e, map, pose, STEP, false, false, false);
    assert.equal(Boolean(cmd.forward), false);
    pose = predictMotion(pose, cmd, STEP);
  }
  cmd = exploreStep(e, map, pose, STEP, false, false, true);
  assert.ok(cmd.forward > 0);
});

test('un nuevo obstaculo tras orientar detiene el intento de salida', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE); for (let x = 0; x < 40; x++) map[22 * 40 + x] = OCCUPIED;
  let pose = { x: 0, z: -.045, yaw: 0 }, e = createExplorer(.25);
  e.state = 'destino'; e.destino = { x: 0, z: -1 }; e.route = [22 * 40 + 20];
  let cmd = exploreStep(e, map, pose, STEP);
  for (let i = 0; i < 100 && e.escape?.phase !== 'observando'; i++) {
    pose = predictMotion(pose, cmd, STEP); cmd = exploreStep(e, map, pose, STEP);
  }
  assert.equal(e.escape?.phase, 'observando');
  const p = e.escape.point, x = Math.floor((p.x - map.grid.originX) / .1), z = Math.floor((p.z - map.grid.originZ) / .1);
  map[z * 40 + x] = OCCUPIED; map.revision++;
  cmd = exploreStep(e, map, pose, STEP, false, false, true);
  assert.equal(Boolean(cmd.forward), false); assert.equal(e.state, 'planificando');
});

test('sin observacion nueva deja de esperar sin avanzar', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE); for (let x = 0; x < 40; x++) map[22 * 40 + x] = OCCUPIED;
  let pose = { x: 0, z: -.045, yaw: 0 }, e = createExplorer(.25);
  e.state = 'destino'; e.destino = { x: 0, z: -1 }; e.route = [22 * 40 + 20];
  let cmd = exploreStep(e, map, pose, STEP);
  for (let i = 0; i < 300 && e.state !== 'limitado'; i++) {
    assert.equal(Boolean(cmd.forward), false); pose = predictMotion(pose, cmd, STEP);
    cmd = exploreStep(e, map, pose, STEP);
  }
  assert.equal(e.state, 'limitado');
});

test('el selector puede demostrar salida frontal cuando retroceder no cabe', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE); for (let x = 0; x < 40; x++) map[22 * 40 + x] = OCCUPIED;
  const pose = { x: 0, z: -.045, yaw: 0 }, goal = { x: 0, z: -1 };
  assert.equal(routeAfterRetreat(map, pose, goal, { radius: .25 }), null);
  const proof = routeAfterRecovery(map, pose, goal, { radius: .25 });
  assert.equal(proof?.mode, 'forward'); assert.ok(proof.route.length > 0);
});

test('retorno recupera salida frontal y mantiene la base como objetivo', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE); for (let x = 0; x < 40; x++) map[22 * 40 + x] = OCCUPIED;
  let pose = { x: 0, z: -.045, yaw: 0 }, e = createExplorer(.25);
  e.state = 'planificando'; e.recoveryState = 'retorno'; e.retorno = 'retornando'; e.base = { x: 0, z: -1 };
  let moved = 0;
  for (let i = 0; i < 400 && e.state !== 'refugio'; i++) {
    const cmd = exploreStep(e, map, pose, STEP, false, false, true);
    moved += Number(Boolean(cmd.forward)); pose = predictMotion(pose, cmd, STEP);
  }
  assert.ok(moved > 0); assert.equal(e.state, 'refugio'); assert.equal(e.retorno, 'base');
});
