// Wall segments are the shared ground truth for physics, sensing and rendering.
const outer = [[-3, -3, 3, -3], [3, -3, 3, 3], [3, 3, -3, 3], [-3, 3, -3, -3]];
const boards = red => [
  { label: 'rojo', x: red.x, y: .35, z: red.z, yaw: 0, size: .48, color: '#ec3030' },
  { label: 'azul', x: 1.3, y: .35, z: -2.99, yaw: 0, size: .48, color: '#2459ef' },
  { label: 'amarillo', x: 2.99, y: .35, z: 1.4, yaw: -Math.PI / 2, size: .48, color: '#edcd16' },
];
export const SCENARIOS = {
  empty: { id: 'empty', label: 'Habitación vacía', walls: outer.map(w => [...w]), boards: boards({ x: -1, z: -2.99 }) },
  partition: { id: 'partition', label: 'Pared con abertura de 1 m', walls: [...outer.map(w => [...w]), [-3, -.8, .8, -.8], [1.8, -.8, 3, -.8]], boards: boards({ x: 0, z: -.79 }) },
  narrow: { id: 'narrow', label: 'Abertura estrecha de 0,3 m', walls: [...outer.map(w => [...w]), [-3, -.8, .9, -.8], [1.2, -.8, 3, -.8]], boards: boards({ x: 0, z: -.79 }) },
};
SCENARIOS.large = {
  id: 'large', label: 'Sala grande 12 × 16 m',
  walls: [[-6, -8, 6, -8], [6, -8, 6, 8], [6, 8, -6, 8], [-6, 8, -6, -8], [-6, -1, 1, -1], [2.4, -1, 6, -1]],
  boards: [
    { label: 'rojo', x: 0, y: .35, z: -.99, yaw: 0, size: .48, color: '#ec3030' },
    { label: 'azul', x: 1.5, y: .35, z: -7.99, yaw: 0, size: .48, color: '#2459ef' },
    { label: 'amarillo', x: 5.99, y: .35, z: 3, yaw: -Math.PI / 2, size: .48, color: '#edcd16' },
  ],
  bounds: { minX: -6, maxX: 6, minZ: -8, maxZ: 8 },
  mapConfig: { originX: -10, originZ: -10, resolution: .1, width: 200, height: 200 },
};
SCENARIOS.casa = {
  id: 'casa', label: 'Casa 10 × 8 m · 3 cuartos',
  walls: [
    [-5, -4, 5, -4], [5, -4, 5, 4], [5, 4, -5, 4], [-5, 4, -5, -4],
    [-1, -4, -1, 0], [-1, 1, -1, 4],
    [-5, 0, -3, 0], [-2, 0, -1, 0],
    [3, 2, 3, 2.5], [3, 3.5, 3, 4],
    [3, 2, 5, 2],
  ],
  boards: [
    { label: 'rojo', x: -3, y: .35, z: -3.99, yaw: 0, size: .48, color: '#ec3030' },
    { label: 'azul', x: -3, y: .35, z: 3.99, yaw: Math.PI, size: .48, color: '#2459ef' },
    { label: 'amarillo', x: 4.99, y: .35, z: 3, yaw: -Math.PI / 2, size: .48, color: '#edcd16' },
  ],
  bounds: { minX: -5, maxX: 5, minZ: -4, maxZ: 4 },
  mapConfig: { originX: -7, originZ: -6, resolution: .1, width: 140, height: 120 },
};
SCENARIOS.departamento = {
  id: 'departamento', label: 'Departamento 11 × 9 m · 6 cuartos',
  walls: [
    [-5.5, -4.5, 5.5, -4.5], [5.5, -4.5, 5.5, 4.5], [5.5, 4.5, -5.5, 4.5], [-5.5, 4.5, -5.5, -4.5],
    [-2, -4.5, -2, 0], [-2, 1, -2, 4.5],
    [-2, 1, .2, 1], [1.2, 1, 4, 1], [5, 1, 5.5, 1],
    [1.8, -4.5, 1.8, -2.2], [1.8, -1.2, 1.8, 1],
    [-5.5, -1.6, -4.4, -1.6], [-3.4, -1.6, -2, -1.6],
  ],
  boards: [
    { label: 'rojo', x: 4.6, y: .35, z: -4.49, yaw: 0, size: .48, color: '#ec3030' },
    { label: 'azul', x: -4.9, y: .35, z: -4.49, yaw: 0, size: .48, color: '#2459ef' },
    { label: 'amarillo', x: 5.49, y: .35, z: -2, yaw: -Math.PI / 2, size: .48, color: '#edcd16' },
  ],
  bounds: { minX: -5.5, maxX: 5.5, minZ: -4.5, maxZ: 4.5 },
  mapConfig: { originX: -7.5, originZ: -6.5, resolution: .1, width: 150, height: 130 },
};
SCENARIOS.casamoderna = {
  id: 'casamoderna', label: 'Casa moderna 10 × 7 m · cuartos gemelos',
  // Two bedrooms (x −5..−1.5 and 1.5..5, z 0..3) are geometrically
  // identical: lidar alone cannot tell them apart. Their wall boards
  // differ (rojo vs azul at the same relative spot), so only the camera
  // breaks the tie.
  walls: [
    [-5, -4, 5, -4], [5, -4, 5, 3], [5, 3, -5, 3], [-5, 3, -5, -4],
    [-5, 0, -2.6, 0], [-1.6, 0, -.6, 0], [.6, 0, 1.6, 0], [2.6, 0, 5, 0],
    [-1.5, 0, -1.5, 3], [1.5, 0, 1.5, 3],
    [0, -4, 0, -.7],
  ],
  boards: [
    { label: 'rojo', x: -3.2, y: .35, z: 2.99, yaw: Math.PI, size: .48, color: '#ec3030' },
    { label: 'azul', x: 3.2, y: .35, z: 2.99, yaw: Math.PI, size: .48, color: '#2459ef' },
    { label: 'amarillo', x: 0, y: .35, z: 2.99, yaw: Math.PI, size: .48, color: '#edcd16' },
    { label: 'azul', x: -4.99, y: .35, z: -1.5, yaw: -Math.PI / 2, size: .48, color: '#2459ef' },
    { label: 'rojo', x: 4.99, y: .35, z: -1.5, yaw: -Math.PI / 2, size: .48, color: '#ec3030' },
    { label: 'amarillo', x: -2, y: .35, z: -3.99, yaw: 0, size: .48, color: '#edcd16' },
  ],
  bounds: { minX: -5, maxX: 5, minZ: -4, maxZ: 3 },
  mapConfig: { originX: -7, originZ: -6, resolution: .1, width: 140, height: 120 },
  inicio: { x: -3, z: -2, yaw: 0 },
};
const casaWalls = SCENARIOS.casa.walls.map(w => [...w]);
const casaBoards = SCENARIOS.casa.boards.map(b => ({ ...b }));
const pata = (x, z) => [x - .03, z, x + .03, z];
const caja = (x, z, w, d) => [
  [x - w / 2, z - d / 2, x + w / 2, z - d / 2], [x + w / 2, z - d / 2, x + w / 2, z + d / 2],
  [x + w / 2, z + d / 2, x - w / 2, z + d / 2], [x - w / 2, z + d / 2, x - w / 2, z - d / 2],
];
SCENARIOS.desorden = {
  id: 'desorden', label: 'Casa con desorden · patas y cacharros',
  walls: [...casaWalls, pata(2.1, -1.3), pata(2.9, -1.3), pata(2.1, -.7), pata(2.9, -.7), pata(1.6, -.85), pata(1.6, -1.15)],
  obstaculos: [...caja(-2.5, -.9, .32, .2), [.5, -2.5, 2.5, -2.5], ...caja(-4, 2.5, .4, .4)],
  boards: casaBoards,
  bounds: { minX: -5, maxX: 5, minZ: -4, maxZ: 4 },
  mapConfig: { originX: -7, originZ: -6, resolution: .1, width: 140, height: 120 },
};
// Furniture: tall pieces are real geometry — the lidar sees them, physics
// collides with them, and fixed pieces (nevera, cocina, cama…) carry a
// camera label so they can act as landmarks. Each room is furnished
// differently on purpose: furniture is what breaks the twin-room tie and
// gives the big empty hall interior features to match against.
SCENARIOS.casamoderna.muebles = [
  { label: 'nevera', x: 4.6, z: -3.2, w: .8, d: .8, h: 1.8, color: '#cfd8dc' },
  { label: 'cocina', x: 4.2, z: -4.7, w: 1.6, d: .6, h: .9, color: '#8d9e94' },
  { label: 'sofa', x: -4.1, z: -3.65, w: 1.6, d: .7, h: .8, color: '#7a6a8f' },
  { label: 'mesa', x: -2.5, z: -1.2, w: 1.2, d: .8, h: .75, color: '#a9805a' },
  { label: 'cama', x: -4.3, z: 1.8, w: 1.4, d: .9, h: .5, color: '#5a7a9e' },
  { label: 'escritorio', x: -2, z: 2.8, w: .9, d: .4, h: .75, color: '#a9805a' },
  { label: 'cama', x: 4.2, z: 2.55, w: 1.6, d: .9, h: .5, color: '#5a7a9e' },
  { label: 'armario', x: 4.65, z: .6, w: .7, d: .5, h: 1.8, color: '#6e5741' },
  { label: 'estanteria', x: 0, z: 2.75, w: 1.6, d: .5, h: 1.6, color: '#6e5741' },
];
SCENARIOS.large.muebles = [
  { label: 'sofa', x: -3.5, z: 1.5, w: 1.8, d: .7, h: .8, color: '#7a6a8f' },
  { label: 'mesa', x: -3.8, z: 5, w: 1.6, d: 1, h: .75, color: '#a9805a' },
  { label: 'estanteria', x: 5.75, z: 5, w: .5, d: 1.6, h: 1.6, color: '#6e5741' },
  { label: 'nevera', x: 5, z: -6.5, w: .8, d: .8, h: 1.8, color: '#cfd8dc' },
  { label: 'cocina', x: -4, z: -7.7, w: 1.6, d: .6, h: .9, color: '#8d9e94' },
  { label: 'armario', x: -5.75, z: 6.5, w: .5, d: 1.4, h: 1.8, color: '#6e5741' },
];
for (const scene of [SCENARIOS.casamoderna, SCENARIOS.large])
  for (const m of scene.muebles) scene.walls.push(...caja(m.x, m.z, m.w, m.d));
// Physics sees every solid; the 2D sensor only sees walls (low clutter and
// cables are invisible to it but still collide).
export function solidos(scene) {
  return scene.obstaculos?.length ? [...scene.walls, ...scene.obstaculos] : scene.walls;
}
for (const scene of Object.values(SCENARIOS)) {
  scene.bounds ??= { minX: -3, maxX: 3, minZ: -3, maxZ: 3 };
  scene.mapConfig ??= { originX: -4, originZ: -4, resolution: .1, width: 80, height: 80 };
}
