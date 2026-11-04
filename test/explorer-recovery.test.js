import test from 'node:test';
import assert from 'node:assert/strict';
import { createMap, FREE, OCCUPIED, center, cell } from '../src/mapping.js';
import { createExplorer, exploreStep, routeAfterRetreat } from '../src/explorer.js';
import { STEP, predictMotion } from '../src/motion.js';
import { routeTo, footprintClear } from '../src/planner.js';
import { createNavigator } from '../src/navigation.js';

test('una señal de suelo detiene el avance y ordena retroceso seguro sin inventar pared', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  const pose = { ...center(20 * 40 + 20, map), yaw: 0 }, e = createExplorer(.2);
  e.state = 'navegando'; e.route = [19 * 40 + 20];
  const before = [...map];
  const command = exploreStep(e, map, pose, STEP, false, true);
  assert.equal(command.forward, false);
  assert.equal(command.back, true);
  assert.equal(e.state, 'retroceso');
  assert.deepEqual([...map], before);
});

test('explorar usa una sola celda y gira antes de avanzar; destino conserva curvas', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  const pose = { ...center(20 * 40 + 20, map), yaw: 0 }, e = createExplorer(.2);
  e.state = 'navegando'; e.route = [20 * 40 + 21, 19 * 40 + 21, 19 * 40 + 22];
  const original = [...e.route], command = exploreStep(e, map, pose, STEP);
  assert.equal(command.forward, false); assert.equal(command.right, true);
  assert.deepEqual(e.route, original);
});

test('un bloqueo sin contacto retrocede por espacio observado sin pintar obstáculo', () => {
  const map = createMap({ originX: -1, originZ: -1, resolution: .1, width: 20, height: 20 });
  map.fill(FREE);
  for (let x = 0; x < 20; x++) map[6 * 20 + x] = 0; // desconocido
  const pose = { x: .05, z: -.095, yaw: 0 }, e = createExplorer(.2);
  e.state = 'navegando'; e.route = [6 * 20 + 10];
  // Force movement verification even when the coarse planner mask is stale.
  e.safe = new Uint8Array(map.length).fill(1); e.safeRevision = map.revision;
  const before = [...map], command = exploreStep(e, map, pose, STEP);
  assert.equal(e.state, 'retroceso'); assert.equal(e.recoveryCause, 'mapa');
  assert.equal(command.back, true); assert.equal(command.forward, false); assert.ok(e.avoid.size > 0);
  assert.deepEqual([...map], before);
});

test('si el suelo también falla al retroceder, deja de retroceder y gira', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  const pose = { ...center(20 * 40 + 20, map), yaw: 0 }, e = createExplorer(.2);
  e.state = 'navegando'; e.route = [19 * 40 + 20];
  exploreStep(e, map, pose, STEP, false, true);
  const cmd = exploreStep(e, map, pose, STEP, false, true);
  assert.equal(cmd.back, false); assert.equal(cmd.left, true);
});


test('replanificar despues de suelo evita el paso rechazado sin cambiar el mapa', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  let pose = { ...center(24 * 40 + 20, map), yaw: 0 };
  const target = center(10 * 40 + 20, map), e = createExplorer(.2);
  e.state = 'destino'; e.destino = target; e.route = routeTo(map, pose, target, { radius: e.footprint });
  const before = [...map];
  let command = exploreStep(e, map, pose, STEP, false, true);
  pose = predictMotion(pose, command, STEP);
  for (let i = 0; i < 49; i++) {
    command = exploreStep(e, map, pose, STEP);
    pose = predictMotion(pose, command, STEP);
  }
  const route = routeTo(map, pose, target, { radius: e.footprint, avoid: e.avoid });
  assert.ok(route?.length > 0);
  assert.ok(route.every(i => !e.avoid.has(i)));
  assert.deepEqual([...map], before);
});

test('la relocalizacion automatica deja de avanzar ante la senal de suelo', () => {
  const grid = { originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 };
  const saved = createNavigator(grid, { noisy: false }).saveMap();
  const nav = createNavigator(grid, { noisy: false, priorMap: saved });
  nav.start();
  let command;
  for (let i = 0; i < 8; i++) command = nav.step(STEP, { rays: [], motion: { avance: 0, giro: 1, contact: false, drive: 0 } });
  assert.equal(command.forward, true);
  command = nav.step(STEP, { motion: { avance: 0, giro: 0, contact: false, drive: 1, groundStop: true } });
  assert.equal(command.forward, false);
  command = nav.step(STEP, { motion: { avance: 0, giro: 0, contact: false, drive: 0 } });
  assert.equal(command.left, true);
});

test('el atajo hacia destino no cruza celdas de suelo evitadas', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  const pose = { ...center(20 * 40 + 10, map), yaw: 0 }, e = createExplorer(.1);
  e.state = 'destino'; e.destino = center(18 * 40 + 13, map);
  e.route = [19 * 40 + 10, 18 * 40 + 10, 18 * 40 + 11, 18 * 40 + 12, 18 * 40 + 13];
  e.avoid.add(19 * 40 + 11);
  exploreStep(e, map, pose, STEP);
  assert.equal(e.route[0], 18 * 40 + 10);
});

test('contacto y retroceso usan el radio configurado para ubicar la marca', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  let observed = null; map.onBump = (pose, atras, dist) => { observed = { pose: { ...pose }, atras, dist }; };
  let pose = { ...center(20 * 40 + 20, map), yaw: 0 }, e = createExplorer(.1);
  const origin = { ...pose };
  e.state = 'navegando'; e.route = [19 * 40 + 20];
  let cmd = exploreStep(e, map, pose, STEP, true);
  pose = predictMotion(pose, cmd, STEP);
  for (let i = 0; i < 20; i++) { cmd = exploreStep(e, map, pose, STEP); pose = predictMotion(pose, cmd, STEP); }
  assert.ok(observed);
  assert.ok(Math.abs((observed.pose.z - observed.dist) - (origin.z - .1)) < 1e-9);
});

test('tras contacto con solape leve, retrocede hacia espacio observado', () => {
  const map = createMap({ originX: -1, originZ: -1, resolution: .1, width: 20, height: 20 });
  map.fill(FREE);
  for (let x = 0; x < 20; x++) map[7 * 20 + x] = OCCUPIED;
  const pose = { x: .05, z: .045, yaw: 0 }, e = createExplorer(.25);
  e.state = 'destino'; e.destino = { x: .05, z: -.7 }; e.route = [5 * 20 + 10];
  const before = [...map];
  const command = exploreStep(e, map, pose, STEP, true);
  assert.equal(command.forward, false); assert.equal(command.back, true);
  assert.equal(e.state, 'retroceso'); assert.deepEqual([...map], before);
});

test('el retroceso elimina un solape leve sin modificar el mapa', () => {
  const map = createMap({ originX: -1, originZ: -1, resolution: .1, width: 20, height: 20 });
  map.fill(FREE); for (let x = 0; x < 20; x++) map[7 * 20 + x] = OCCUPIED;
  let pose = { x: .05, z: .045, yaw: 0 }, e = createExplorer(.25);
  e.state = 'destino'; e.destino = { x: .05, z: -.7 }; e.route = [5 * 20 + 10];
  const before = [...map];
  let cmd = exploreStep(e, map, pose, STEP, false, true);
  for (let i = 0; i < 8; i++) {
    assert.equal(cmd.forward, false); assert.equal(cmd.back, true);
    const next = predictMotion(pose, cmd, STEP);
    assert.ok(next.z > pose.z); pose = next;
    cmd = exploreStep(e, map, pose, STEP);
  }
  assert.ok(pose.z > .15); assert.deepEqual([...map], before);
});

test('no usa el escape para entrar en una zona desconocida detras', () => {
  const map = createMap({ originX: -1, originZ: -1, resolution: .1, width: 20, height: 20 });
  map.fill(FREE); for (let x = 0; x < 20; x++) { map[7 * 20 + x] = OCCUPIED; map[13 * 20 + x] = 0; }
  const pose = { x: .05, z: .045, yaw: 0 }, e = createExplorer(.25);
  e.state = 'destino'; e.destino = { x: .05, z: -.7 }; e.route = [5 * 20 + 10];
  const cmd = exploreStep(e, map, pose, STEP, false, true);
  assert.equal(cmd.back, false); assert.equal(cmd.forward, false);
});

test('una ruta invalidada con solape inicia retirada y conserva destino', () => {
  const map = createMap({ originX: -1, originZ: -1, resolution: .1, width: 20, height: 20 });
  map.fill(FREE); for (let x = 0; x < 20; x++) map[7 * 20 + x] = OCCUPIED;
  const pose = { x: .05, z: .045, yaw: 0 }, e = createExplorer(.25), goal = { x: .05, z: -.7 };
  e.state = 'destino'; e.destino = goal; e.route = [5 * 20 + 10];
  const cmd = exploreStep(e, map, pose, STEP);
  assert.equal(cmd.back, true); assert.equal(e.state, 'retroceso'); assert.deepEqual(e.destino, goal);
});

test('un solape profundo no autoriza retroceso inseguro', () => {
  const map = createMap({ originX: -1, originZ: -1, resolution: .1, width: 20, height: 20 });
  map.fill(FREE); for (let x = 0; x < 20; x++) map[7 * 20 + x] = OCCUPIED;
  const pose = { x: .05, z: -.03, yaw: 0 }, e = createExplorer(.25);
  e.state = 'destino'; e.destino = { x: .05, z: -.7 }; e.route = [5 * 20 + 10];
  const cmd = exploreStep(e, map, pose, STEP, false, true);
  assert.equal(cmd.back, false); assert.equal(cmd.forward, false);
});

test('un robot pequeno encuentra ruta demostrada despues de salir de un solape de 4 cm', () => {
  const map = createMap({ originX: -1, originZ: -1, resolution: .1, width: 20, height: 20 });
  map.fill(FREE); for (let x = 0; x < 20; x++) map[7 * 20 + x] = OCCUPIED;
  const pose = { x: .05, z: -.14, yaw: 0 }, goal = { x: .05, z: .65 }, radius = .1;
  const escape = routeAfterRetreat(map, pose, goal, { radius });
  assert.ok(escape?.route.length > 0);
  const e = createExplorer(radius); e.state = 'planificando'; e.destino = goal;
  const cmd = exploreStep(e, map, pose, STEP);
  assert.equal(e.state, 'retroceso'); assert.equal(cmd.back, true);
});
test('ruta tras retirada no autoriza atravesar suelo desconocido', () => {
  const map = createMap({ originX: -1, originZ: -1, resolution: .1, width: 20, height: 20 });
  map.fill(FREE); for (let x = 0; x < 20; x++) { map[7 * 20 + x] = OCCUPIED; map[10 * 20 + x] = 0; }
  const pose = { x: .05, z: -.14, yaw: 0 };
  assert.equal(routeAfterRetreat(map, pose, { x: .05, z: .65 }, { radius: .1 }), null);
});

test('un retorno interrumpido por contacto reanuda la ruta a la base', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  let pose = { ...center(20 * 40 + 20, map), yaw: 0 }, e = createExplorer(.2);
  e.state = 'retorno'; e.retorno = 'retornando'; e.base = { x: pose.x, z: pose.z + 1 };
  e.route = [19 * 40 + 20];
  let cmd = exploreStep(e, map, pose, STEP, true);
  pose = predictMotion(pose, cmd, STEP);
  for (let i = 0; i < 52; i++) { cmd = exploreStep(e, map, pose, STEP); pose = predictMotion(pose, cmd, STEP); }
  assert.equal(e.state, 'retorno'); assert.ok(e.route.length > 0);
});

test('intentos agotados no limitan si el planificador todavía ve salida al destino', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  const pose = { ...center(20 * 40 + 20, map), yaw: 0 }, e = createExplorer(.2);
  e.destino = { x: pose.x, z: pose.z - 1.5 };
  // Seis bloqueos seguidos en el mismo lugar con ruta libre al destino:
  // agotar el presupuesto no debe declarar limitado mientras haya salida.
  for (let n = 0; n < 6; n++) {
    e.state = 'destino'; e.route = [19 * 40 + 20];
    exploreStep(e, map, pose, STEP, false, true);
    assert.notEqual(e.state, 'limitado', `bloqueo ${n + 1}: no debe limitar con ruta disponible`);
    assert.equal(e.state, 'retroceso');
  }
  assert.ok(e.recoveryAttempts <= 12);
});

test('intentos agotados sí limitan cuando no hay ninguna salida', () => {
  const map = createMap({ originX: -1, originZ: -1, resolution: .1, width: 20, height: 20 });
  map.fill(OCCUPIED); map[10 * 20 + 10] = FREE;
  const pose = { ...center(10 * 20 + 10, map), yaw: 0 }, e = createExplorer(.25);
  e.state = 'destino'; e.destino = { x: .05, z: -.7 };
  for (let n = 0; n < 20 && e.state !== 'limitado'; n++) {
    if (e.state === 'retroceso') e.state = 'destino';
    e.route = [5 * 20 + 10];
    exploreStep(e, map, pose, STEP, false, true);
  }
  assert.equal(e.state, 'limitado');
});

test('al bloquearse un paso, la celda objetivo queda marcada y no se reofrece', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  // Celda ocupada en diagonal: el centro objetivo pasa la máscara de ruta,
  // pero el disco de la huella la toca a mitad del avance.
  map[18 * 40 + 23] = OCCUPIED; // centro (0,35, -0,15)
  const objetivo = 17 * 40 + 20; // centro (0,05, -0,25)
  let pose = { x: .05, z: .05, yaw: 0 };
  const e = createExplorer(.25);
  e.state = 'destino'; e.destino = { x: .05, z: -.85 }; e.route = [objetivo];
  for (let i = 0; i < 200 && e.state !== 'retroceso'; i++) {
    const cmd = exploreStep(e, map, pose, STEP);
    pose = predictMotion(pose, cmd, STEP);
  }
  assert.equal(e.state, 'retroceso');
  assert.equal(e.recoveryCause, 'mapa');
  assert.ok(e.avoid.has(objetivo), 'la celda que no se pudo pisar debe quedar en evitar');
  const replanteada = routeTo(map, pose, e.destino, { radius: e.footprint, avoid: e.avoid });
  assert.ok(!replanteada || !replanteada.includes(objetivo), 'la próxima ruta no debe ofrecer la misma celda');
});

test('un freno solo por celdas evitadas replantea sin gastar recuperación', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  const objetivo = 17 * 40 + 20; // centro (0,05, -0,25)
  const pose = { x: .05, z: -.09, yaw: 0 }, e = createExplorer(.25);
  e.state = 'destino'; e.destino = { x: .05, z: -.85 }; e.route = [objetivo];
  e.avoid.add(18 * 40 + 20); // celda (0,05, -0,15): el próximo paso la rozaría
  const cmd = exploreStep(e, map, pose, STEP);
  assert.equal(cmd.forward, false, 'el paso frenado por evitar no debe ejecutarse');
  assert.equal(e.state, 'planificando', 'debe replanificar, no entrar en recuperación');
  assert.equal(e.recoveryAttempts, 0);
  assert.ok(e.avoid.has(objetivo), 'la celda ofrecida queda fuera de la próxima ruta');
});

test('se alinea antes de avanzar si solo la curva cruza una celda evitada', () => {
  const map = createMap({ originX: -2, originZ: -2, resolution: .1, width: 40, height: 40 });
  map.fill(FREE);
  let pose = { x: .001, z: .003, yaw: 0 };
  const e = createExplorer(.1), target = cell(-.25, -.25, map);
  e.state = 'destino'; e.destino = center(target, map); e.route = [target];
  const forbidden = cell(.001, -.005, map); e.avoid.add(forbidden);
  const before = [...map];
  let command = exploreStep(e, map, pose, STEP);
  assert.equal(command.forward, false);
  assert.ok(command.left > 0);
  assert.equal(e.state, 'destino');
  assert.deepEqual(e.route, [target]);
  for (let i = 0; i < 600 && e.state !== 'destino_alcanzado'; i++) {
    pose = predictMotion(pose, command, STEP);
    assert.notEqual(cell(pose.x, pose.z, map), forbidden);
    command = exploreStep(e, map, pose, STEP);
  }
  assert.equal(e.state, 'destino_alcanzado');
  assert.deepEqual([...e.avoid], [forbidden]);
  assert.equal(e.recoveryAttempts, 0);
  assert.deepEqual([...map], before);
});

test('no permite una retirada que aumente el solape existente', () => {
  const map = createMap({ originX: -1, originZ: -1, resolution: .1, width: 20, height: 20 });
  map.fill(FREE); for (let x = 0; x < 20; x++) map[12 * 20 + x] = OCCUPIED;
  const pose = { x: .05, z: -.045, yaw: 0 }, e = createExplorer(.25);
  e.state = 'destino'; e.destino = { x: .05, z: -.7 }; e.route = [5 * 20 + 10];
  const cmd = exploreStep(e, map, pose, STEP, false, true);
  assert.equal(cmd.back, false); assert.equal(cmd.forward, false);
});
