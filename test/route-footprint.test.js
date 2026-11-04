import test from 'node:test';
import assert from 'node:assert/strict';
import { createMap, center, FREE, OCCUPIED } from '../src/mapping.js';
import { createExplorer, startExplorer, setDestination, exploreStep } from '../src/explorer.js';
import { routeTo, traversable, footprintClear } from '../src/planner.js';
import { predictMotion, STEP } from '../src/motion.js';

function corridor(radius, rows) {
  const map = createMap({ originX: -1, originZ: -2, resolution: .05, width: 40, height: 80 });
  map.fill(OCCUPIED);
  const first = Math.floor((map.grid.width - rows) / 2);
  for (let row = 4; row < 76; row++) for (let col = first; col < first + rows; col++) map[row * 40 + col] = FREE;
  const start = { ...center(65 * 40 + first + Math.floor(rows / 2), map), yaw: 0 }, goal = center(15 * 40 + first + Math.floor(rows / 2), map);
  const explorer = createExplorer(radius); startExplorer(explorer); setDestination(explorer, goal);
  return { map, start, goal, explorer };
}
function run(fixture) {
  let pose = { ...fixture.start }, active = 0;
  for (let i = 0; i < 1800 && fixture.explorer.state !== 'destino_alcanzado'; i++) {
    // Sensor map changes invalidate the cached mask during a real run.
    if (i % 6 === 0) fixture.map.revision++;
    const command = exploreStep(fixture.explorer, fixture.map, pose, STEP);
    if (Object.values(command).some(Boolean)) active++;
    const next = predictMotion(pose, command, STEP);
    assert.ok(footprintClear(fixture.map, next, fixture.explorer.footprint));
    pose = next;
  }
  return { pose, active };
}
test('ejecuta rutas de un robot de 20 cm que la máscara de 50 cm rechazaba', () => {
  const f = corridor(.1, 8), route = routeTo(f.map, f.start, f.goal, { radius: .1 });
  assert.ok(route?.length > 0);
  const wrongMask = traversable(f.map, .25);
  assert.ok(route.some(i => !wrongMask[i]));
  const { pose, active } = run(f);
  assert.ok(active > 0, 'Debe producir movimiento, no alternar planificación y destino indefinidamente');
  assert.equal(f.explorer.state, 'destino_alcanzado');
  assert.ok(Math.hypot(pose.x - f.goal.x, pose.z - f.goal.z) < .1);
});
test('ejecuta el margen alternativo del planificador conservando la huella completa', () => {
  const f = corridor(.12, 5), route = routeTo(f.map, f.start, f.goal, { radius: .12 });
  assert.ok(route?.length > 0);
  const conservative = traversable(f.map, .12);
  assert.ok(route.some(i => !conservative[i]));
  run(f); assert.equal(f.explorer.state, 'destino_alcanzado');
});
test('un robot de 50 cm sigue sin poder atravesar un pasillo de 40 cm', () => {
  const f = corridor(.25, 8);
  assert.equal(routeTo(f.map, f.start, f.goal, { radius: .25 }), null);
});
