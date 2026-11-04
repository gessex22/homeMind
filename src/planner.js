import { gridOf, UNKNOWN, FREE, cell, center } from './mapping.js';
// Only observations and pose are inputs. Scene geometry never enters this module.
const offsetCache = new Map();
function offsets(resolution, radius) {
  const key = `${resolution}:${radius}`;
  if (!offsetCache.has(key)) {
    const list = [], margin = radius + resolution * Math.SQRT2 / 2, n = Math.ceil(margin / resolution);
    for (let dz = -n; dz <= n; dz++) for (let dx = -n; dx <= n; dx++) if (Math.hypot(dx, dz) * resolution <= margin) list.push([dx, dz]);
    offsetCache.set(key, list);
  }
  return offsetCache.get(key);
}

export function frontiers(map) {
  const g = gridOf(map), result = [];
  for (let i = 0; i < map.length; i++) if (map[i] === FREE) {
    const x = i % g.width, z = Math.floor(i / g.width);
    // Free space at the working-domain edge remains a frontier, never a wall.
    if (x === 0 || z === 0 || x === g.width - 1 || z === g.height - 1
      || map[i - 1] === UNKNOWN || map[i + 1] === UNKNOWN || map[i - g.width] === UNKNOWN || map[i + g.width] === UNKNOWN) result.push(i);
  }
  return result;
}

export function traversable(map, radius = .25) {
  const g = gridOf(map), safe = new Uint8Array(map.length), around = offsets(g.resolution, radius);
  for (let i = 0; i < map.length; i++) if (map[i] === FREE) {
    const col = i % g.width, row = Math.floor(i / g.width);
    let valid = true;
    for (const [dx, dz] of around) {
      const x = col + dx, z = row + dz;
      if (x < 0 || z < 0 || x >= g.width || z >= g.height || map[z * g.width + x] !== FREE) { valid = false; break; }
    }
    safe[i] = Number(valid);
  }
  return safe;
}

export function footprintClear(map, pose, radius = .25) {
  const g = gridOf(map);
  const minX = Math.floor((pose.x - radius - g.originX) / g.resolution), maxX = Math.floor((pose.x + radius - g.originX) / g.resolution);
  const minZ = Math.floor((pose.z - radius - g.originZ) / g.resolution), maxZ = Math.floor((pose.z + radius - g.originZ) / g.resolution);
  for (let z = minZ; z <= maxZ; z++) for (let x = minX; x <= maxX; x++) {
    const left = g.originX + x * g.resolution, top = g.originZ + z * g.resolution;
    const dx = Math.max(left - pose.x, 0, pose.x - left - g.resolution), dz = Math.max(top - pose.z, 0, pose.z - top - g.resolution);
    if (dx * dx + dz * dz <= radius * radius && (x < 0 || x >= g.width || z < 0 || z >= g.height || map[z * g.width + x] !== FREE)) return false;
  }
  return true;
}

export function plan(map, pose, scanned = [], { reverse = false, radius = .25, cerradas = null, avoid = null } = {}) {
  const g = gridOf(map), start = cell(pose.x, pose.z, map);
  if (start < 0 || map[start] !== FREE) return { route: [], fronts: frontiers(map), reason: 'bloqueado', safe: traversable(map, radius) };
  const result = (reason, route = [], safe = traversable(map, radius)) => ({ route, fronts, reason, safe });
  // Only frontiers connected to the robot through free space are real:
  // noisy endpoints can paint phantom free cells beyond a wall, and those
  // pockets can never be observed or reached â€” counting them would make
  // every mission end limitado chasing ghosts.
  const comp = new Uint8Array(map.length), stack = [start]; comp[start] = 1;
  while (stack.length) {
    const c = stack.pop(), x = c % g.width, z = Math.floor(c / g.width);
    for (const n of [x + 1 < g.width ? c + 1 : -1, x > 0 ? c - 1 : -1, z + 1 < g.height ? c + g.width : -1, z > 0 ? c - g.width : -1])
      if (n >= 0 && !comp[n] && map[n] === FREE) { comp[n] = 1; stack.push(n); }
  }
  // Frontiers closed by verified inspection (photo/human) no longer count:
  // a strip against a confirmed wall is done, not pending.
  const fronts = frontiers(map).filter(f => comp[f] && !(cerradas && cerradas.has(f)));
  if (fronts.length === 0) return result('completado');

  const near = new Uint8Array(map.length), visited = new Uint8Array(map.length);
  function disk(x, z, radius, dest) {
    const n = Math.ceil(radius / g.resolution), col = Math.floor((x - g.originX) / g.resolution), row = Math.floor((z - g.originZ) / g.resolution);
    for (let dz = -n; dz <= n; dz++) for (let dx = -n; dx <= n; dx++) {
      const a = col + dx, b = row + dz;
      if (a < 0 || b < 0 || a >= g.width || b >= g.height) continue;
      const i = b * g.width + a, p = center(i, map);
      if (Math.hypot(p.x - x, p.z - z) < radius) dest[i] = 1;
    }
  }
  for (const f of fronts) { const p = center(f, map); disk(p.x, p.z, .61, near); }
  for (const p of scanned) disk(p.x, p.z, .8, visited);

  // Clearance-aware search: cells far from obstacles cost less, so routes
  // cross doorways through their centre instead of hugging a jamb â€” with
  // real drift, a corner-cutting path is a wall clip waiting to happen.
  const amplio = traversable(map, radius * 1.52);
  function buscar(safe, permitirVisitados = false) {
    const tight = !safe[start];
    const previous = new Int32Array(map.length).fill(-1), dist = new Float32Array(map.length).fill(Infinity);
    const heap = [start]; dist[start] = 0; previous[start] = start;
    let target = -1;
    const push = i => { heap.push(i); let c = heap.length - 1; while (c > 0) { const p = (c - 1) >> 1; if (dist[heap[p]] <= dist[heap[c]]) break; [heap[p], heap[c]] = [heap[c], heap[p]]; c = p; } };
    const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let c = 0; for (;;) { let l = 2 * c + 1, r = l + 1, m = c; if (l < heap.length && dist[heap[l]] < dist[heap[m]]) m = l; if (r < heap.length && dist[heap[r]] < dist[heap[m]]) m = r; if (m === c) break; [heap[m], heap[c]] = [heap[c], heap[m]]; c = m; } } return top; };
    while (heap.length) {
      const current = pop(), x = current % g.width, z = Math.floor(current / g.width);
      if ((current !== start || !tight) && near[current] && (permitirVisitados || !visited[current])) { target = current; break; }
      const neighbors = [x + 1 < g.width ? current + 1 : -1, x > 0 ? current - 1 : -1, z + 1 < g.height ? current + g.width : -1, z > 0 ? current - g.width : -1];
      for (const next of reverse ? neighbors.reverse() : neighbors) {
        if (next < 0 || !safe[next] || (avoid?.has(next) && next !== start)) continue;
        const nd = dist[current] + (amplio[next] ? 1 : 4);
        if (nd < dist[next]) { dist[next] = nd; previous[next] = current; push(next); }
      }
    }
    if (target < 0) return null;
    const route = []; for (let i = target; i !== start; i = previous[i]) route.push(i);
    return route.reverse();
  }

  // Preferred tier: full clearance. A 1 m doorway smeared by noisy mapping
  // leaves no cell with the full margin, so a second pass plans with the
  // robot's real footprint margin; stepAlong still guards every step with
  // footprintClear, and a truly impassable gap (0.3 m) stays blocked.
  const amplia = traversable(map, radius);
  const rutaAmplia = buscar(amplia);
  if (rutaAmplia) return result('ruta', rutaAmplia, amplia);
  const ajustada = traversable(map, radius * .8);
  const rutaAjustada = buscar(ajustada);
  if (rutaAjustada) return result('ruta', rutaAjustada, ajustada);
  // A tight-but-free start still tries to escape; only a start with no way
  // out reports blocked. Step-by-step footprint checks guard the motion.
  return result(!amplia[start] && !ajustada[start] ? 'bloqueado' : 'limitado', [], ajustada);
}

export function routeTo(map, from, to, { radius = .25, avoid = null, parcial = false } = {}) {
  const g = gridOf(map);
  const start = cell(from.x, from.z, map);
  if (start < 0 || map[start] !== FREE) return null;
  // Same two clearance tiers as plan(): full margin preferred, footprint
  // margin as fallback so doorways stay routable.
  for (const safe of [traversable(map, radius), traversable(map, radius * .8)]) {
    let goal = cell(to.x, to.z, map);
    if (goal < 0 || !safe[goal] || avoid?.has(goal)) {
      goal = -1;
      const col = Math.floor((to.x - g.originX) / g.resolution), row = Math.floor((to.z - g.originZ) / g.resolution);
      let best = Infinity;
      for (let dz = -5; dz <= 5; dz++) for (let dx = -5; dx <= 5; dx++) {
        const x = col + dx, z = row + dz;
        if (x < 0 || z < 0 || x >= g.width || z >= g.height) continue;
        const i = z * g.width + x, d = Math.hypot(dx, dz);
        if (safe[i] && !avoid?.has(i) && d < best) { best = d; goal = i; }
      }
      if (goal < 0 && !parcial) continue;
    }
    const amplio = traversable(map, radius * 1.52);
    const previous = new Int32Array(map.length).fill(-1), dist = new Float32Array(map.length).fill(Infinity);
    const heap = [start]; dist[start] = 0; previous[start] = start;
    const push = i => { heap.push(i); let c = heap.length - 1; while (c > 0) { const p = (c - 1) >> 1; if (dist[heap[p]] <= dist[heap[c]]) break; [heap[p], heap[c]] = [heap[c], heap[p]]; c = p; } };
    const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let c = 0; for (;;) { let l = 2 * c + 1, r = l + 1, m = c; if (l < heap.length && dist[heap[l]] < dist[heap[m]]) m = l; if (r < heap.length && dist[heap[r]] < dist[heap[m]]) m = r; if (m === c) break; [heap[m], heap[c]] = [heap[c], heap[m]]; c = m; } } return top; };
    while (heap.length) {
      const current = pop();
      if (current === goal) break;
      const x = current % g.width, z = Math.floor(current / g.width);
      for (const next of [x + 1 < g.width ? current + 1 : -1, x > 0 ? current - 1 : -1, z + 1 < g.height ? current + g.width : -1, z > 0 ? current - g.width : -1]) {
        if (next < 0 || !safe[next] || (avoid?.has(next) && next !== start)) continue;
        const nd = dist[current] + (amplio[next] ? 1 : 4);
        if (nd < dist[next]) { dist[next] = nd; previous[next] = current; push(next); }
      }
    }
    if (goal < 0 || previous[goal] < 0) {
      // Partial approach (opt-in): the goal itself is unreachable with the
      // full footprint, but a reachable safe cell may still be closer to it
      // than the start. Route there instead of refusing to move; only a
      // start already at the closest reachable cell reports no route.
      // Strict callers (return-to-base, destination validation) keep the
      // default null, which they read as "no proven route".
      if (!parcial) continue;
      // Only approach a goal already discovered as free: an unseen goal is
      // exploration's job (frontiers), not a blind partial drive.
      const goalCell = cell(to.x, to.z, map);
      if (goalCell < 0 || map[goalCell] !== FREE) continue;
      let best = -1, bestGoal = Infinity;
      for (let i = 0; i < map.length; i++) {
        if (i === start || !safe[i] || !Number.isFinite(dist[i])) continue;
        const p = center(i, map), d = Math.hypot(p.x - to.x, p.z - to.z);
        if (d < bestGoal || (d === bestGoal && dist[i] < dist[best])) { best = i; bestGoal = d; }
      }
      // Progress must be at least one cell: a closer cell whose centre is
      // already within the route's arrival tolerance would be re-issued
      // forever without the robot ever moving.
      if (best < 0 || bestGoal >= Math.hypot(from.x - to.x, from.z - to.z) - g.resolution) continue;
      const parcialRoute = []; for (let i = best; i !== start; i = previous[i]) parcialRoute.push(i);
      return parcialRoute.reverse();
    }
    const route = []; for (let i = goal; i !== start; i = previous[i]) route.push(i);
    return route.reverse();
  }
  return null;
}
