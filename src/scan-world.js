// Ground-truth adapter derived from the observed mesh, never an input map for
// the navigation engine. Unknown floor is not manufactured or traversable.
import { createMap, center, cell, FREE, OCCUPIED, gridOf } from './mapping.js';
import { footprintClear } from './planner.js';
import { predictMotion } from './motion.js';
import { gaussian } from './noise.js';
import { SENSOR } from './sensor.js';

export const SCAN_BODY = Object.freeze({ radius: .10, height: .6, clearance: .08 });
// Capture holes: unknown floor within this distance of verified floor is
// bridgeable (a threshold strip Polycam failed to sweep). Anything wider
// stays a cliff: the footprint still stops there.
export const PUENTE_MAX_M = .15;
function triangleInfo(a, b, c) {
  const ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z, vx = c.x - a.x, vy = c.y - a.y, vz = c.z - a.z;
  const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx, norm = Math.hypot(nx, ny, nz);
  return { horizontal: norm ? Math.abs(ny) / norm : 0, area: norm / 2 };
}
function floorPlane(triangles, maxY) {
  const bins = new Map();
  for (let i = 0; i < triangles.length; i += 9) {
    const a = { x: triangles[i], y: triangles[i + 1], z: triangles[i + 2] }, b = { x: triangles[i + 3], y: triangles[i + 4], z: triangles[i + 5] }, c = { x: triangles[i + 6], y: triangles[i + 7], z: triangles[i + 8] };
    const info = triangleInfo(a, b, c), y = (a.y + b.y + c.y) / 3;
    if (info.horizontal < .92 || y > Math.min(.5, maxY * .22) || info.area < 1e-7) continue;
    const index = Math.round(y / .02); bins.set(index, (bins.get(index) ?? 0) + info.area);
  }
  let best = null, area = 0;
  for (const [index] of bins) { const support = (bins.get(index - 1) ?? 0) + bins.get(index) + (bins.get(index + 1) ?? 0); if (support > area) { area = support; best = index; } }
  if (best === null || area < .1) throw new Error('No hay suficiente evidencia de un suelo horizontal');
  let sum = 0, weight = 0;
  for (let d = -1; d <= 1; d++) { const w = bins.get(best + d) ?? 0; sum += (best + d) * .02 * w; weight += w; }
  return sum / weight;
}
function clipHeight(poly, height, keepAbove) {
  const result = [];
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length], insideA = keepAbove ? a.y >= height : a.y <= height, insideB = keepAbove ? b.y >= height : b.y <= height;
    if (insideA) result.push(a);
    if (insideA !== insideB) { const t = (height - a.y) / (b.y - a.y); result.push({ x: a.x + t * (b.x - a.x), y: height, z: a.z + t * (b.z - a.z) }); }
  }
  return result;
}
function coversPoint(a, b, c, x, z) {
  const det = (b.z - c.z) * (a.x - c.x) + (c.x - b.x) * (a.z - c.z);
  if (Math.abs(det) < 1e-12) return false;
  const u = ((b.z - c.z) * (x - c.x) + (c.x - b.x) * (z - c.z)) / det;
  const v = ((c.z - a.z) * (x - c.x) + (a.x - c.x) * (z - c.z)) / det;
  return u >= -1e-7 && v >= -1e-7 && u + v <= 1 + 1e-7;
}
function intersectsCell(poly, x, z, half) {
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i], b = poly[(i + 1) % poly.length], nx = b.z - a.z, nz = a.x - b.x;
    let min = Infinity, max = -Infinity;
    for (const p of poly) { const v = nx * p.x + nz * p.z; min = Math.min(min, v); max = Math.max(max, v); }
    const mid = nx * x + nz * z, radius = (Math.abs(nx) + Math.abs(nz)) * half;
    if (max < mid - radius - 1e-8 || min > mid + radius + 1e-8) return false;
  }
  return true;
}
export function buildScanWorld(geometry, { scale = 1, resolution = .05, radius = SCAN_BODY.radius } = {}) {
  if (!Number.isFinite(scale) || scale < .25 || scale > 4) throw new Error('Escala fuera de rango');
  const triangles = Float32Array.from(geometry.triangles, v => v * scale), b = geometry.bounds;
  const bounds = { minX: b.minX * scale, maxX: b.maxX * scale, minZ: b.minZ * scale, maxZ: b.maxZ * scale, maxY: b.maxY * scale };
  const config = { resolution, originX: Math.floor((bounds.minX - .3) / resolution) * resolution, originZ: Math.floor((bounds.minZ - .3) / resolution) * resolution };
  config.width = Math.ceil((bounds.maxX + .3 - config.originX) / resolution); config.height = Math.ceil((bounds.maxZ + .3 - config.originZ) / resolution);
  const map = createMap(config), g = gridOf(map), floorBits = new Uint8Array(map.length), occupied = new Uint8Array(map.length);
  const floorY = floorPlane(triangles, bounds.maxY), heights = [.08, .2, .35, .5, .6].map(v => v + floorY), segments = [];
  function cellsFor(poly, callback) {
    const minX = Math.max(0, Math.floor((Math.min(...poly.map(p => p.x)) - g.originX) / resolution)), maxX = Math.min(g.width - 1, Math.floor((Math.max(...poly.map(p => p.x)) - g.originX) / resolution));
    const minZ = Math.max(0, Math.floor((Math.min(...poly.map(p => p.z)) - g.originZ) / resolution)), maxZ = Math.min(g.height - 1, Math.floor((Math.max(...poly.map(p => p.z)) - g.originZ) / resolution));
    for (let row = minZ; row <= maxZ; row++) for (let col = minX; col <= maxX; col++) callback(row * g.width + col, g.originX + (col + .5) * resolution, g.originZ + (row + .5) * resolution);
  }
  const offsets = [[0, 0], [-.48, -.48], [.48, -.48], [-.48, .48], [.48, .48]];
  for (let i = 0; i < triangles.length; i += 9) {
    const a = { x: triangles[i], y: triangles[i + 1], z: triangles[i + 2] }, b = { x: triangles[i + 3], y: triangles[i + 4], z: triangles[i + 5] }, c = { x: triangles[i + 6], y: triangles[i + 7], z: triangles[i + 8] }, poly = [a, b, c];
    const low = Math.min(a.y, b.y, c.y), high = Math.max(a.y, b.y, c.y);
    if (low >= floorY - .045 && high <= floorY + .045 && triangleInfo(a, b, c).horizontal >= .85) {
      cellsFor(poly, (index, x, z) => { for (let k = 0; k < offsets.length; k++) if (coversPoint(a, b, c, x + offsets[k][0] * resolution, z + offsets[k][1] * resolution)) floorBits[index] |= 1 << k; });
    }
    // Near-horizontal geometry just above the floor plane is floor
    // undulation in a real capture, not an obstacle: skip it in the band.
    const floorLike = triangleInfo(a, b, c).horizontal >= .85 && high <= floorY + .22;
    if (!floorLike && high >= floorY + SCAN_BODY.clearance && low <= floorY + SCAN_BODY.height) {
      const clipped = clipHeight(clipHeight(poly, floorY + SCAN_BODY.clearance, true), floorY + SCAN_BODY.height, false);
      if (clipped.length) cellsFor(clipped, (index, x, z) => { if (intersectsCell(clipped, x, z, resolution / 2)) occupied[index] = 1; });
    }
    for (const height of heights) if (low <= height && high >= height && high - low > 1e-8) {
      const points = [];
      for (let j = 0; j < 3; j++) {
        const p = poly[j], q = poly[(j + 1) % 3];
        if ((p.y <= height && q.y >= height) || (q.y <= height && p.y >= height)) {
          if (Math.abs(q.y - p.y) < 1e-10) continue;
          const t = (height - p.y) / (q.y - p.y), x = p.x + t * (q.x - p.x), z = p.z + t * (q.z - p.z);
          if (!points.some(v => Math.hypot(v.x - x, v.z - z) < 1e-6)) points.push({ x, z });
        }
      }
      if (points.length >= 2 && Math.hypot(points[0].x - points[1].x, points[0].z - points[1].z) > 1e-5) segments.push([points[0].x, points[0].z, points[1].x, points[1].z]);
    }
  }
  let free = 0, obstacleCount = 0;
  for (let i = 0; i < map.length; i++) { if (occupied[i]) { map[i] = OCCUPIED; obstacleCount++; } else if (floorBits[i] === 31) { map[i] = FREE; free++; } }
  // Bridgeable cells: multi-source BFS from verified floor through unknown
  // cells only, up to PUENTE_MAX_M. Occupied cells never bridge.
  const puentes = new Uint8Array(map.length);
  {
    const dist = new Int32Array(map.length).fill(-1), queue = [];
    for (let i = 0; i < map.length; i++) if (map[i] === FREE) { dist[i] = 0; queue.push(i); }
    const maxPasos = Math.round(PUENTE_MAX_M / resolution);
    for (let head = 0; head < queue.length; head++) {
      const c = queue[head], x = c % g.width, z = Math.floor(c / g.width);
      for (const n of [x + 1 < g.width ? c + 1 : -1, x > 0 ? c - 1 : -1, z + 1 < g.height ? c + g.width : -1, z > 0 ? c - g.width : -1]) {
        if (n < 0 || dist[n] >= 0 || map[n] === OCCUPIED) continue;
        dist[n] = dist[c] + 1;
        if (dist[n] <= maxPasos) { puentes[n] = 1; queue.push(n); }
      }
    }
  }
  // Vertical surfaces sliced at five heights yield the same 2D segment
  // five times. Dedupe (1 cm tolerance, endpoint order normalized) so the
  // sensor and the drawing work with unique geometry only.
  const seenSeg = new Set(), uniqueSegments = [];
  for (const s of segments) {
    const [x1, z1, x2, z2] = s.map(v => Math.round(v * 100));
    const key = x1 < x2 || (x1 === x2 && z1 <= z2) ? `${x1},${z1}|${x2},${z2}` : `${x2},${z2}|${x1},${z1}`;
    if (seenSeg.has(key)) continue;
    seenSeg.add(key); uniqueSegments.push(s);
  }
  segments.length = 0; segments.push(...uniqueSegments);
  const buckets = Array.from({ length: map.length }, () => []);
  segments.forEach((s, index) => cellsFor([{ x: s[0], z: s[1] }, { x: s[2], z: s[3] }], i => buckets[i].push(index)));
  const clearance = new Float32Array(map.length); let start = null, best = 0, possibleStarts = 0;
  for (let i = 0; i < map.length; i++) if (map[i] === FREE) {
    const p = center(i, map);
    if (!footprintClear(map, p, radius)) continue;
    possibleStarts++;
    let distance = radius;
    for (let r = radius + .05; r <= .8 && footprintClear(map, p, r); r += .05) distance = r;
    clearance[i] = distance;
    if (distance > best) { best = distance; start = { ...p, yaw: 0 }; }
  }
  return { map, grid: g, floorBits, puentes, floorY, bounds, scale, radius, segments, buckets, initialPose: start,
    stats: { floorY, freeAreaM2: free * resolution ** 2, occupiedAreaM2: obstacleCount * resolution ** 2, unknownCells: map.length - free - obstacleCount,
      puentes: puentes.reduce((n, v) => n + v, 0), possibleStarts, clearanceM: best, triangleCount: triangles.length / 9, layers: heights.length, segments: segments.length } };
}
const cross = (ax, az, bx, bz) => ax * bz - az * bx;
export function senseScanWorld(world, pose, config = SENSOR) {
  const g = world.grid, r = g.resolution;
  return Array.from({ length: config.rays }, (_, i) => {
    const angle = pose.yaw + (config.rays === 1 ? 0 : i / (config.rays - 1) - .5) * config.fov, dx = -Math.sin(angle), dz = -Math.cos(angle);
    let hit = Infinity, freeDistance = 0, gap = false;
    const seen = new Set();
    let col = Math.floor((pose.x - g.originX) / r), row = Math.floor((pose.z - g.originZ) / r);
    const sxStep = dx >= 0 ? 1 : -1, szStep = dz >= 0 ? 1 : -1;
    let tx = Math.abs(dx) < 1e-12 ? Infinity : (g.originX + (col + (dx >= 0 ? 1 : 0)) * r - pose.x) / dx;
    let tz = Math.abs(dz) < 1e-12 ? Infinity : (g.originZ + (row + (dz >= 0 ? 1 : 0)) * r - pose.z) / dz;
    const deltaX = r / Math.abs(dx), deltaZ = r / Math.abs(dz);
    for (let d = 0; d <= config.range && d <= hit; ) {
      const index = col >= 0 && col < g.width && row >= 0 && row < g.height ? row * g.width + col : -1;
      if (index < 0) { gap = true; break; }
      if (!gap && (world.floorBits[index] === 31 || world.puentes[index])) freeDistance = Math.min(tx, tz, config.range) - 1e-6; else gap = true;
      for (const k of world.buckets[index]) if (!seen.has(k)) {
        seen.add(k); const [x1, z1, x2, z2] = world.segments[k], sx = x2 - x1, sz = z2 - z1;
        const denominator = cross(dx, dz, sx, sz); if (Math.abs(denominator) < 1e-10) continue;
        const qx = x1 - pose.x, qz = z1 - pose.z, t = cross(qx, qz, sx, sz) / denominator, u = cross(qx, qz, dx, dz) / denominator;
        if (t > .005 && u >= 0 && u <= 1) hit = Math.min(hit, t);
      }
      const next = Math.min(tx, tz);
      if (tx <= next + 1e-10) { col += sxStep; tx += deltaX; }
      if (tz <= next + 1e-10) { row += szStep; tz += deltaZ; }
      d = next;
    }
    if (hit <= config.range) {
      const distance = Math.max(.005, Math.min(config.range, hit + ((config.noise ?? 0) > 0 ? gaussian(config.random) * config.noise : 0)));
      return { angle, distance, freeDistance: Math.min(freeDistance, Math.max(0, hit - r / 2)), kind: 'wall' };
    }
    return config.ground && freeDistance > r ? { angle, distance: freeDistance, freeDistance, kind: 'floor' } : { angle, distance: null, kind: 'invalid' };
  });
}
// Footprint over the capture: verified floor and bridgeable holes pass;
// occupied cells and wide unknown gaps stop the body.
export function huellaTransitable(world, pose, radius = world.radius) {
  const g = world.grid;
  const minX = Math.floor((pose.x - radius - g.originX) / g.resolution), maxX = Math.floor((pose.x + radius - g.originX) / g.resolution);
  const minZ = Math.floor((pose.z - radius - g.originZ) / g.resolution), maxZ = Math.floor((pose.z + radius - g.originZ) / g.resolution);
  for (let z = minZ; z <= maxZ; z++) for (let x = minX; x <= maxX; x++) {
    const left = g.originX + x * g.resolution, top = g.originZ + z * g.resolution;
    const dx = Math.max(left - pose.x, 0, pose.x - left - g.resolution), dz = Math.max(top - pose.z, 0, pose.z - top - g.resolution);
    if (dx * dx + dz * dz > radius * radius) continue;
    if (x < 0 || x >= g.width || z < 0 || z >= g.height) return false;
    const i = z * g.width + x;
    if (world.map[i] === OCCUPIED) return false;
    if (world.map[i] !== FREE && !world.puentes[i]) return false;
  }
  return true;
}

export function advanceScanWorld(world, pose, command, dt) {
  const predicted = predictMotion(pose, command, dt), dx = predicted.x - pose.x, dz = predicted.z - pose.z, steps = Math.max(1, Math.ceil(Math.hypot(dx, dz) / .01));
  let p = { ...pose, yaw: predicted.yaw }, contact = false, groundStop = false;
  for (let i = 1; i <= steps; i++) {
    const candidate = { x: pose.x + dx * i / steps, z: pose.z + dz * i / steps, yaw: predicted.yaw };
    if (!huellaTransitable(world, candidate, world.radius)) {
      let lo = 0, hi = 1;
      for (let k = 0; k < 15; k++) {
        const t = (lo + hi) / 2, mid = { x: p.x + (candidate.x - p.x) * t, z: p.z + (candidate.z - p.z) * t };
        if (huellaTransitable(world, mid, world.radius)) lo = t; else hi = t;
      }
      p.x += (candidate.x - p.x) * lo; p.z += (candidate.z - p.z) * lo;
      // Classify the stop only for sensor/observer diagnostics. Unknown floor
      // produces a depth/cliff stop, not a fabricated bumper or occupied wall.
      const g = world.grid, n = Math.ceil(world.radius / g.resolution) + 1;
      const col = Math.floor((candidate.x - g.originX) / g.resolution), row = Math.floor((candidate.z - g.originZ) / g.resolution);
      for (let z = row - n; z <= row + n; z++) for (let x = col - n; x <= col + n; x++) {
        const nearestX = Math.max(g.originX + x * g.resolution - candidate.x, 0, candidate.x - g.originX - (x + 1) * g.resolution);
        const nearestZ = Math.max(g.originZ + z * g.resolution - candidate.z, 0, candidate.z - g.originZ - (z + 1) * g.resolution);
        if (Math.hypot(nearestX, nearestZ) > world.radius) continue;
        const idx = x >= 0 && x < g.width && z >= 0 && z < g.height ? z * g.width + x : -1;
        const value = idx >= 0 ? world.map[idx] : 0;
        if (value === OCCUPIED) contact = true; else if (value !== FREE && !(idx >= 0 && world.puentes[idx])) groundStop = true;
      }
      break;
    }
    p = candidate;
  }
  return { ...p, collision: contact, groundStop };
}
