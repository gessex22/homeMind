import test from 'node:test';
import assert from 'node:assert/strict';
import { sense } from '../src/sensor.js';
import { createMap, integrate, seedFootprint, cell, UNKNOWN, FREE, OCCUPIED } from '../src/mapping.js';
import { plan, traversable, footprintClear } from '../src/planner.js';
import { createExplorer, startExplorer, exploreStep } from '../src/explorer.js';
import { advance, STEP } from '../src/physics.js';

test('el sensor respeta FOV y oclusión; un no retorno no revela espacio', () => {
  const pose = { x: 0, z: 0, yaw: 0 }, map = createMap();
  integrate(map, pose, sense(pose, [[-2, -1, 2, -1], [-2, -2, 2, -2]]));
  assert.equal(map[cell(0, -1.005)], OCCUPIED);
  assert.equal(map[cell(0, -1.5)], UNKNOWN);
  assert.equal(map[cell(2, 0)], UNKNOWN);
  const empty = createMap(); integrate(empty, pose, [{ angle: 0, distance: null }]);
  assert.equal(empty[cell(0, -1)], UNKNOWN);
});
test('ruta usa espacio libre y evita ocupadas con margen del radio', () => {
  const map = createMap();
  for (let z = 30; z < 50; z++) for (let x = 30; x < 50; x++) map[z * 80 + x] = FREE;
  map[cell(.5, 0)] = OCCUPIED;
  const result = plan(map, { x: 0, z: 0 }, [{ x: 0, z: 0 }]), safe = traversable(map);
  assert.ok(result.route.length > 0);
  for (const i of result.route) { assert.equal(map[i], FREE); assert.equal(safe[i], 1); }
  assert.equal(safe[cell(.3, 0)], 0);
});
test('un pasillo conocido más estrecho que el robot no es transitable', () => {
  const map = createMap();
  for (let z = 20; z < 60; z++) for (let x = 38; x < 42; x++) map[z * 80 + x] = FREE;
  const safe = traversable(map);
  assert.equal(map[cell(0, 0)], FREE);
  assert.equal(safe[cell(0, 0)], 0);
  assert.equal(plan(map, { x: 0, z: 0 }).reason, 'bloqueado');
});
test('exploración cerrada descubre la habitación y se detiene', { timeout: 120000 }, () => {
  const map = createMap(), explorer = createExplorer(); let pose = { x: 0, z: 0, yaw: 0, collision: false };
  seedFootprint(map, pose); startExplorer(explorer); let steps = 0;
  for (; steps < 90000 && !['completado', 'bloqueado', 'limitado'].includes(explorer.state); steps++) {
    if (steps % 6 === 0) integrate(map, pose, sense(pose));
    const command = exploreStep(explorer, map, pose, STEP); const next = advance(pose, command, STEP);
    if (command.forward || command.back) { assert.ok(footprintClear(map, pose)); assert.ok(footprintClear(map, next)); }
    if (command.forward || command.back) for (let a = 0; a < 24; a++) {
      const theta = a * Math.PI * 2 / 24;
      assert.equal(map[cell(next.x + .25 * Math.cos(theta), next.z + .25 * Math.sin(theta))], FREE, `huella desconocida paso ${steps}`);
    }
    pose = next;
    assert.equal(pose.collision, false, `colisión en paso ${steps}`);
  }
  let observed = 0, total = 0, wall = 0;
  for (let z = -2.95; z < 3; z += .1) for (let x = -2.95; x < 3; x += .1) { total++; if (map[cell(x, z)] === FREE) observed++; }
  for (let v = -2.95; v < 3; v += .1) for (const [x, z] of [[v, -3.005], [v, 3.005], [-3.005, v], [3.005, v]]) if (map[cell(x, z)] === OCCUPIED) wall++;
  assert.equal(explorer.state, 'completado', `estado ${explorer.state}, pasos ${steps}`);
  assert.ok(explorer.scanned.length > 1, `solo ${explorer.scanned.length} barrido(s)`);
  assert.ok(observed / total >= .95, `suelo ${observed}/${total}; paredes ${wall}; pasos ${steps}`);
  assert.ok(wall >= 210, `paredes detectadas ${wall}`);
});
