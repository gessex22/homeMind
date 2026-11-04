import test from 'node:test';
import assert from 'node:assert/strict';
import { createMap, FREE, OCCUPIED, UNKNOWN, center } from '../src/mapping.js';
import { routeTo, footprintClear } from '../src/planner.js';
import { createExplorer, exploreStep } from '../src/explorer.js';
import { predictMotion, wheelSpeeds, STEP } from '../src/motion.js';
import { createMotionFusion, gateMotionCommand } from '../src/motion-fusion.js';
import { createOdometry, readMotion } from '../src/odometry.js';

function setup(obstacle = false) {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  if (obstacle) for (let z = 13; z <= 24; z++) for (let x = 19; x <= 20; x++) map[z * 40 + x] = z === 18 ? UNKNOWN : OCCUPIED;
  const start = { ...center(30 * 40 + 10, map), yaw: 0 }, goal = center(10 * 40 + 30, map);
  const explorer = createExplorer(.18);
  explorer.destino = goal; explorer.state = 'destino'; explorer.route = routeTo(map, start, goal, { radius: .18 });
  assert.ok(explorer.route?.length);
  return { map, start, goal, explorer };
}
function run(f) {
  let pose = f.start, curved = 0, turns = 0, distance = 0, signChanges = 0, previousSign = 0;
  for (let i = 0; i < 1600 && f.explorer.state !== 'destino_alcanzado'; i++) {
    const command = exploreStep(f.explorer, f.map, pose, STEP);
    const sign = Math.sign((command.left || 0) - (command.right || 0));
    if (sign && previousSign && sign !== previousSign) signChanges++;
    if (sign) { turns++; previousSign = sign; }
    if (command.forward && sign) curved++;
    const next = predictMotion(pose, command, STEP);
    for (let j = 0; j <= 10; j++) assert.ok(footprintClear(f.map, { x: pose.x + (next.x - pose.x) * j / 10, z: pose.z + (next.z - pose.z) * j / 10 }, f.explorer.footprint));
    distance += Math.hypot(next.x - pose.x, next.z - pose.z); pose = next;
  }
  assert.equal(f.explorer.state, 'destino_alcanzado');
  return { pose, distance, curved, turns, signChanges };
}
test('la ruta diagonal combina avance y giro sin seguir cada celda en zigzag', () => {
  const f = setup(), result = run(f);
  assert.ok(result.curved > 20);
  assert.ok(result.distance < 3.1, `trayecto ${result.distance}`);
  assert.ok(result.signChanges < 8, `cambios ${result.signChanges}`);
});
test('el adelanto rodea una esquina ocupada y desconocida con toda la huella', () => {
  const f = setup(true), result = run(f);
  assert.ok(result.curved > 0);
  assert.ok(result.distance > Math.hypot(f.goal.x - f.start.x, f.goal.z - f.start.z));
});
test('fracciones llegan a ruedas y odometría y el bloqueo conserva la dirección', () => {
  const command = { forward: .4, back: false, left: .3, right: false };
  const wheel = wheelSpeeds(command);
  assert.equal(wheel.avance, .4); assert.ok(wheel.giro > 0 && wheel.giro < Math.PI / 2);
  assert.equal(wheelSpeeds({ forward: true }).avance, 1);
  assert.deepEqual(predictMotion({ x: 0, z: 0, yaw: 0 }, { forward: true }, 1), { x: 0, z: -1, yaw: 0 });
  const fusion = createMotionFusion(); fusion.blockedDirection = 1;
  assert.equal(gateMotionCommand(fusion, command).forward, false);
  assert.equal(gateMotionCommand(fusion, { ...command, forward: false, back: .2 }).back, .2);
  const pose = { x: 0, z: 0, yaw: 0 }, odom = createOdometry(pose, { patinaje: 1, seed: 1 });
  const measured = readMotion(odom, pose, pose, command, 1, true);
  assert.equal(measured.drive, .4); assert.ok(measured.avance > .2 && measured.avance <= .4);
});
test('puede alejarse de una pared casi tangente sin un bloqueo falso', () => {
  const map = createMap({ originX: -1, originZ: -1, resolution: .1, width: 30, height: 30 });
  map.fill(FREE);
  for (let z = 0; z < 30; z++) map[z * 30 + 10] = OCCUPIED;
  const pose = { x: .3501, z: .05, yaw: -Math.PI / 2 }, explorer = createExplorer(.25);
  explorer.state = 'destino'; explorer.destino = { x: .85, z: .05 };
  explorer.route = [10 * 30 + 18];
  const command = exploreStep(explorer, map, pose, STEP);
  assert.ok(command.forward > 0);
  assert.equal(explorer.state, 'destino');
});
test('el retroceso no atraviesa celdas desconocidas con un paso largo', () => {
  const config = { originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 };
  const map = createMap(config), floor = createMap(config);
  map.fill(FREE); floor.fill(FREE);
  for (let x = 0; x < 40; x++) { map[24 * 40 + x] = UNKNOWN; floor[24 * 40 + x] = UNKNOWN; }
  let pose = { x: .05, z: .05, yaw: 0 };
  const explorer = createExplorer(.18); explorer.state = 'navegando';
  let command = exploreStep(explorer, map, pose, 1, true);
  assert.equal(command.back, false); assert.equal(command.forward, false);
  // Recovery can orient toward a safe alternative instead of turning left
  // immediately; it must wait for a fresh scan before any translation.
  for (let i = 0; i < 300 && explorer.escape?.phase !== 'observando'; i++) {
    assert.equal(Boolean(command.forward), false); assert.equal(Boolean(command.back), false);
    pose = predictMotion(pose, command, STEP);
    command = exploreStep(explorer, map, pose, STEP);
  }
  assert.equal(explorer.escape?.phase, 'observando');
  command = exploreStep(explorer, map, pose, STEP);
  assert.equal(Boolean(command.forward), false);
  command = exploreStep(explorer, map, pose, STEP, false, false, true);
  assert.ok(command.forward > 0);
  for (let i = 0; i < 200 && explorer.escape; i++) {
    const next = predictMotion(pose, command, STEP);
    for (let j = 0; j <= 10; j++) {
      const t = j / 10;
      assert.ok(footprintClear(floor, { x: pose.x + (next.x - pose.x) * t, z: pose.z + (next.z - pose.z) * t }, .18));
    }
    pose = next;
    command = exploreStep(explorer, map, pose, STEP, false, false, true);
  }
  assert.equal(explorer.escape, null);
  assert.ok(footprintClear(map, pose, .18));
});
