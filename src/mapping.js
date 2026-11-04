export const GRID = Object.freeze({ origin: -4, originX: -4, originZ: -4, resolution: .1, width: 80, height: 80 });
export const UNKNOWN = 0, FREE = 1, OCCUPIED = 2;
// Log-odds fusion: walls need redundant confirmation (2+ hits at +.3) so an
// isolated noisy spike never becomes a wall, while one free pass (−.06)
// still opens fresh space. Confirmed walls don't erode: grazing free votes
// skip them, so passages keep their edges and exploration can complete.
// Drift can still plant phantom free cells outside walls; only scan matching
// fixes that, so noisy runs on big scenes may end limited instead of complete.
export const LOGODDS = Object.freeze({ impacto: .3, libre: -.06, min: -4, max: 4, umbralOcupado: .5, umbralLibre: -.05 });

export function gridOf(value = GRID) { return value.grid ?? (value.width ? value : GRID); }

export function createMap(config = GRID) {
  const grid = Object.freeze({
    originX: config.originX ?? config.origin ?? GRID.origin,
    originZ: config.originZ ?? config.origin ?? GRID.origin,
    resolution: config.resolution ?? GRID.resolution,
    width: config.width ?? GRID.width,
    height: config.height ?? GRID.height,
  });
  if (![grid.originX, grid.originZ, grid.resolution].every(Number.isFinite)
    || grid.resolution <= 0 || !Number.isInteger(grid.width) || !Number.isInteger(grid.height)
    || grid.width < 1 || grid.height < 1 || grid.width * grid.height > 1000000) throw new RangeError('Dominio de mapa inválido');
  const map = new Uint8Array(grid.width * grid.height);
  Object.defineProperty(map, 'grid', { value: grid });
  Object.defineProperty(map, 'logodds', { value: new Float32Array(grid.width * grid.height), writable: true });
  map.revision = 0;
  return map;
}

function logoddsOf(map) {
  if (!map.logodds || map.logodds.length !== map.length) map.logodds = new Float32Array(map.length);
  return map.logodds;
}

function stateOf(l) {
  return l >= LOGODDS.umbralOcupado ? OCCUPIED : l <= LOGODDS.umbralLibre ? FREE : UNKNOWN;
}

export function cell(x, z, map = GRID) {
  const g = gridOf(map);
  const col = Math.floor((x - g.originX) / g.resolution), row = Math.floor((z - g.originZ) / g.resolution);
  return col >= 0 && col < g.width && row >= 0 && row < g.height ? row * g.width + col : -1;
}

export function center(index, map = GRID) {
  const g = gridOf(map);
  return { x: g.originX + (index % g.width + .5) * g.resolution, z: g.originZ + (Math.floor(index / g.width) + .5) * g.resolution };
}

export function seedFootprint(map, pose, radius = .25) {
  const g = gridOf(map), L = logoddsOf(map);
  let changed = false;
  const col = Math.floor((pose.x - g.originX) / g.resolution), row = Math.floor((pose.z - g.originZ) / g.resolution);
  const n = Math.ceil(radius / g.resolution);
  for (let dz = -n; dz <= n; dz++) for (let dx = -n; dx <= n; dx++) {
    const x = col + dx, z = row + dz;
    if (x < 0 || x >= g.width || z < 0 || z >= g.height) continue;
    const i = z * g.width + x, p = center(i, map);
    if (Math.hypot(p.x - pose.x, p.z - pose.z) <= radius && map[i] === UNKNOWN) { map[i] = FREE; L[i] = -2; changed = true; }
  }
  if (changed) map.revision = (map.revision ?? 0) + 1;
}

export function integrate(map, pose, rays) {
  const g = gridOf(map), L = logoddsOf(map);
  let changed = false;
  function observe(i, delta) {
    if (i < 0) return;
    if (delta < 0 && map[i] === OCCUPIED) return;
    L[i] = Math.min(LOGODDS.max, Math.max(LOGODDS.min, L[i] + delta));
    const next = stateOf(L[i]);
    if (map[i] !== next) { map[i] = next; changed = true; }
  }
  for (const ray of rays) {
    if (ray.kind === 'invalid' || !Number.isFinite(ray.angle) || !Number.isFinite(ray.distance) || ray.distance <= 0) continue;
    const dx = -Math.sin(ray.angle), dz = -Math.cos(ray.angle);
    const floor = ray.kind === 'floor';
    const measuredEnd = floor ? ray.distance : ray.distance - g.resolution / 2;
    // Depth may see an obstacle across a gap in observed ground. Its endpoint
    // is occupied evidence; that does not certify a drivable path to it.
    const end = Number.isFinite(ray.freeDistance) ? Math.min(measuredEnd, Math.max(0, ray.freeDistance)) : measuredEnd;
    let prev = -2;
    for (let d = 0; d <= end; d += g.resolution / 4) {
      const i = cell(pose.x + dx * d, pose.z + dz * d, map);
      if (i !== prev) { observe(i, LOGODDS.libre); prev = i; }
    }
    const hit = cell(pose.x + dx * (ray.distance + (floor ? 0 : .005)), pose.z + dz * (ray.distance + (floor ? 0 : .005)), map);
    observe(hit, floor ? LOGODDS.libre : LOGODDS.impacto);
  }
  if (changed) map.revision = (map.revision ?? 0) + 1;
  return changed;
}

// Bump reflex: paint a 3x3 occupied block where the bumper hit (invisible
// obstacles only reveal themselves by contact). Strong evidence, sticky.
// Never paints the robot's own cell: standing somewhere proves it drivable.
export function markBump(map, pose, atras = false, dist = .25) {
  map.onBump?.(pose, atras, dist);
  const g = gridOf(map), L = logoddsOf(map);
  const s = atras ? 1 : -1;
  const cx = pose.x + s * Math.sin(pose.yaw) * dist, cz = pose.z + s * Math.cos(pose.yaw) * dist;
  const col = Math.floor((cx - g.originX) / g.resolution), row = Math.floor((cz - g.originZ) / g.resolution);
  const self = cell(pose.x, pose.z, map);
  let changed = false;
  for (let dz = -1; dz <= 1; dz++) for (let dx = -1; dx <= 1; dx++) {
    const x = col + dx, z = row + dz;
    if (x < 0 || z < 0 || x >= g.width || z >= g.height) continue;
    const i = z * g.width + x;
    if (i === self) continue;
    // Contact locates a surface at the bumper plane. Do not thicken that
    // surface toward the body: a 3x3 stencil used to trap small robots in
    // occupied cells that the contact itself had just created behind it.
    const nx = s * Math.sin(pose.yaw), nz = s * Math.cos(pose.yaw);
    const farX = g.originX + (x + (nx >= 0 ? 1 : 0)) * g.resolution;
    const farZ = g.originZ + (z + (nz >= 0 ? 1 : 0)) * g.resolution;
    if ((farX - pose.x) * nx + (farZ - pose.z) * nz < dist - 1e-9) continue;
    L[i] = LOGODDS.max;
    if (map[i] !== OCCUPIED) { map[i] = OCCUPIED; changed = true; }
  }
  if (changed) map.revision = (map.revision ?? 0) + 1;
  return changed;
}

export function counts(map) {
  const result = { libre: 0, ocupado: 0, desconocido: 0 };
  for (const value of map) result[value === FREE ? 'libre' : value === OCCUPIED ? 'ocupado' : 'desconocido']++;
  return result;
}
