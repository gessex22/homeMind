import { createMap, seedFootprint, integrate, markBump, cell, OCCUPIED, FREE } from './mapping.js';
import { projectScan, SENSOR } from './sensor.js';
import { compose, between, wrap, optimizeGraph } from './pose-graph.js';

export const SLAM = Object.freeze({ maxScans: 6000, maxSubmaps: 120, minAge: 12, minTravel: 4,
  searchDistance: .75, searchYaw: .24, candidateRadius: 1.4, confirmations: 3, cooldown: 8 });
const clonePose = p => ({ x: p.x, z: p.z, yaw: p.yaw });
function pack(rays) {
  return { distances: Float32Array.from(rays, r => Number.isFinite(r.distance) ? r.distance : NaN),
    freeDistances: Float32Array.from(rays, r => Number.isFinite(r.freeDistance) ? r.freeDistance : NaN),
    kinds: Uint8Array.from(rays, r => r.kind === 'wall' ? 1 : r.kind === 'floor' ? 2 : 0) };
}
function unpack(record) {
  return Array.from(record.distances, (distance, i) => ({ distance, kind: ['invalid', 'wall', 'floor'][record.kinds[i]],
    ...(Number.isFinite(record.freeDistances?.[i]) ? { freeDistance: record.freeDistances[i] } : {}) }));
}
export function createSlam(map, initialPose = { x: 0, z: 0, yaw: 0 }, options = {}) {
  const slam = { config: { ...SLAM, ...options }, initialPose: clonePose(initialPose), nodes: [{ pose: clonePose(initialPose), time: 0, travel: 0 }],
    edges: [], records: [], contacts: [], submaps: [], travel: 0, previous: clonePose(initialPose), time: 0,
    closures: [], candidates: 0, rejected: 0, pending: null, lastSearch: -Infinity, lastClosure: -Infinity, limited: false, status: 'aprendiendo' };
  const onBump = (pose, back, distance) => {
    if (slam.limited) return;
    const anchor = slam.nodes.length - 1;
    slam.contacts.push({ anchor, local: between(slam.nodes[anchor].pose, pose), back, distance });
  };
  Object.defineProperty(map, 'onBump', { value: onBump, configurable: true });
  return slam;
}
function recordPose(slam, record) { return compose(slam.nodes[record.anchor].pose, record.local); }
function makeSubmap(slam, end = slam.records.length) {
  const first = Math.max(0, end - 25), records = slam.records.slice(first, end), anchor = records[0].anchor, origin = slam.nodes[anchor].pose;
  const map = createMap({ originX: -5, originZ: -5, resolution: .1, width: 100, height: 100 });
  for (const record of records) {
    const p = between(origin, recordPose(slam, record));
    integrate(map, p, projectScan(unpack(record), p, record.sensor));
  }
  const surfaces = [], buckets = new Map();
  for (const record of records) {
    const p = between(origin, recordPose(slam, record)), n = record.distances.length;
    function endpoint(i) {
      const a = p.yaw + (n === 1 ? 0 : i / (n - 1) - .5) * record.sensor.fov;
      return { x: p.x - Math.sin(a) * record.distances[i], z: p.z - Math.cos(a) * record.distances[i] };
    }
    for (let i = 4; i < n - 4; i += 4) {
      if (record.kinds[i] !== 1 || record.kinds[i - 4] !== 1 || record.kinds[i + 4] !== 1) continue;
      const q = endpoint(i), a = endpoint(i - 4), b = endpoint(i + 4), dx = b.x - a.x, dz = b.z - a.z, length = Math.hypot(dx, dz);
      if (length < .05 || length > .6) continue; // Do not fit across occlusion edges.
      const surface = { ...q, nx: -dz / length, nz: dx / length };
      surfaces.push(surface);
      const key = `${Math.floor(q.x / .2)},${Math.floor(q.z / .2)}`;
      if (!buckets.has(key)) buckets.set(key, []);
      buckets.get(key).push(surface);
    }
  }
  // A small likelihood field tolerates range quantization, but never treats
  // unknown as support for a match. Store a frozen, historical observation.
  const field = new Float32Array(map.length);
  for (let i = 0; i < map.length; i++) if (map[i] === OCCUPIED) {
    const x = i % 100, z = Math.floor(i / 100);
    for (let dz = -2; dz <= 2; dz++) for (let dx = -2; dx <= 2; dx++) {
      const a = x + dx, b = z + dz;
      if (a < 0 || a >= 100 || b < 0 || b >= 100) continue;
      const d = Math.hypot(dx, dz), credit = d === 0 ? 1 : d <= 1.5 ? .55 : d <= 2 ? .1 : 0;
      field[b * 100 + a] = Math.max(field[b * 100 + a], credit);
    }
  }
  return { id: end, end, anchor, map, field, surfaces, buckets, time: records.at(-1).time, travel: records.at(-1).travel };
}
function solve3(matrix, vector) {
  const rows = Array.from({ length: 3 }, (_, i) => [...matrix.slice(i * 3, i * 3 + 3), vector[i]]);
  for (let c = 0; c < 3; c++) {
    let pivot = c;
    for (let r = c + 1; r < 3; r++) if (Math.abs(rows[r][c]) > Math.abs(rows[pivot][c])) pivot = r;
    [rows[c], rows[pivot]] = [rows[pivot], rows[c]];
    if (Math.abs(rows[c][c]) < 1e-7) return null;
    const scale = rows[c][c]; for (let k = c; k < 4; k++) rows[c][k] /= scale;
    for (let r = 0; r < 3; r++) if (r !== c) { const f = rows[r][c]; for (let k = c; k < 4; k++) rows[r][k] -= f * rows[c][k]; }
  }
  return rows.map(r => r[3]);
}
// Continuous point-to-line refinement against raw historical returns avoids
// mistaking occupancy-cell quantization for a measured pose correction.
function refineSurfaces(submap, points, initial) {
  if (!submap.buckets?.size) return null;
  let pose = clonePose(initial), rms = Infinity, support = 0;
  for (let iteration = 0; iteration < 10; iteration++) {
    const H = new Float64Array(9), g = new Float64Array(3);
    let errors = 0, count = 0;
    for (const p of points) {
      const q = compose(pose, { ...p, yaw: 0 }), x = Math.floor(q.x / .2), z = Math.floor(q.z / .2);
      let closest, distance = .2 ** 2;
      for (let dz = -1; dz <= 1; dz++) for (let dx = -1; dx <= 1; dx++) for (const s of submap.buckets.get(`${x + dx},${z + dz}`) ?? []) {
        const d = (q.x - s.x) ** 2 + (q.z - s.z) ** 2;
        if (d < distance) { distance = d; closest = s; }
      }
      if (!closest) continue;
      const r = closest.nx * (q.x - closest.x) + closest.nz * (q.z - closest.z);
      const J = [closest.nx, closest.nz, closest.nx * (q.z - pose.z) - closest.nz * (q.x - pose.x)];
      const weight = Math.abs(r) <= .04 ? 1 : .04 / Math.abs(r);
      for (let a = 0; a < 3; a++) { g[a] -= weight * J[a] * r; for (let b = 0; b < 3; b++) H[a * 3 + b] += weight * J[a] * J[b]; }
      errors += r * r; count++;
    }
    if (count < points.length * .7 || H[0] * H[4] - H[1] ** 2 < .01 * (H[0] + H[4]) ** 2) return null;
    const delta = solve3(H, g); if (!delta) return null;
    const scale = Math.min(1, .08 / Math.max(.0001, Math.hypot(delta[0], delta[1])), .04 / Math.max(.0001, Math.abs(delta[2])));
    pose = { x: pose.x + delta[0] * scale, z: pose.z + delta[1] * scale, yaw: wrap(pose.yaw + delta[2] * scale) };
    rms = Math.sqrt(errors / count); support = count / points.length;
    if (Math.hypot(...delta) < .0002) break;
  }
  return { pose, rms, support };
}
function currentPoints(slam, pose) {
  const result = [];
  for (const record of slam.records.slice(-6)) {
    const p = between(pose, recordPose(slam, record));
    for (let i = 0; i < record.distances.length; i += 4) if (record.kinds[i] === 1 && record.distances[i] > .05) {
      const a = p.yaw + (record.distances.length === 1 ? 0 : i / (record.distances.length - 1) - .5) * record.sensor.fov;
      result.push({ x: p.x - Math.sin(a) * record.distances[i], z: p.z - Math.cos(a) * record.distances[i], ox: p.x, oz: p.z });
    }
  }
  return result.filter((_, i) => i % Math.max(1, Math.ceil(result.length / 100)) === 0);
}
function distinctive(points) {
  if (points.length < 15) return false;
  const mean = points.reduce((a, p) => ({ x: a.x + p.x / points.length, z: a.z + p.z / points.length }), { x: 0, z: 0 });
  let xx = 0, zz = 0, xz = 0;
  for (const p of points) { const x = p.x - mean.x, z = p.z - mean.z; xx += x * x; zz += z * z; xz += x * z; }
  return xx * zz - xz * xz > .015 * (xx + zz) ** 2;
}
// Public for deterministic rejection/recognition tests. Inputs contain only
// observed endpoints and an old submap, never simulator walls or true poses.
export function matchSubmap(submap, points, prior, config = SLAM) {
  if (!distinctive(points)) return { accepted: false, reason: 'geometria_ambigua' };
  function evaluate(pose) {
    const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    let credit = 0, matched = 0, contradictions = 0;
    for (const p of points) {
      const x = pose.x + c * p.x + s * p.z, z = pose.z - s * p.x + c * p.z;
      const index = cell(x, z, submap.map), v = submap.field[index] ?? 0;
      credit += v; matched += Number(v >= .55);
      if (submap.map[index] === FREE && v < .55) contradictions++;
    }
    const score = (credit - contradictions * .4) / points.length;
    // Quantized, noisy walls can give several poses exactly the same score.
    // Prefer the smaller correction only as a tie breaker, not as evidence.
    const rank = score - .002 * (Math.hypot(pose.x - prior.x, pose.z - prior.z) + 2 * Math.abs(wrap(pose.yaw - prior.yaw)));
    return { ...pose, score, rank, matched: matched / points.length, contradictions: contradictions / points.length };
  }
  const candidates = [];
  const n = Math.round(config.searchDistance / .15), ny = Math.round(config.searchYaw / .06);
  for (let a = -ny; a <= ny; a++) for (let z = -n; z <= n; z++) for (let x = -n; x <= n; x++) candidates.push(evaluate({ x: prior.x + x * .15, z: prior.z + z * .15, yaw: wrap(prior.yaw + a * .06) }));
  candidates.sort((a, b) => b.rank - a.rank);
  let best = candidates[0];
  for (let a = -2; a <= 2; a++) for (let z = -3; z <= 3; z++) for (let x = -3; x <= 3; x++) {
    const candidate = evaluate({ x: candidates[0].x + x * .025, z: candidates[0].z + z * .025, yaw: wrap(candidates[0].yaw + a * .015) });
    if (candidate.rank > best.rank) best = candidate;
  }
  const refined = refineSurfaces(submap, points, best);
  let refinementUsed = false;
  if (refined) {
    const next = evaluate(refined.pose);
    if (next.score >= best.score - .04) { best = next; refinementUsed = true; }
  }
  const rival = candidates.find(p => Math.hypot(p.x - best.x, p.z - best.z) > .3 || Math.abs(wrap(p.yaw - best.yaw)) > .12);
  const margin = best.score - (rival?.score ?? best.score);
  const boundary = Math.abs(best.x - prior.x) >= config.searchDistance - .025 || Math.abs(best.z - prior.z) >= config.searchDistance - .025 || Math.abs(wrap(best.yaw - prior.yaw)) >= config.searchYaw - .01;
  let blocked = 0;
  for (const p of points) {
    const origin = compose(best, { x: p.ox ?? 0, z: p.oz ?? 0, yaw: 0 }), end = compose(best, { ...p, yaw: 0 });
    const distance = Math.hypot(end.x - origin.x, end.z - origin.z);
    for (let d = .25; d < distance - .2; d += .2) {
      const i = cell(origin.x + (end.x - origin.x) * d / distance, origin.z + (end.z - origin.z) * d / distance, submap.map);
      if (submap.map[i] === OCCUPIED) { blocked++; break; }
    }
  }
  const accepted = refinementUsed && refined.rms <= .04 && best.matched >= .8 && best.score >= .65 && best.contradictions <= .12 && blocked / points.length <= .05 && margin >= .08 && !boundary;
  return { accepted, pose: clonePose(best), score: best.score, matched: best.matched, margin,
    residual: refined?.rms ?? null,
    reason: accepted ? 'coincidencia_distinguible' : boundary ? 'fuera_de_ventana' : 'evidencia_insuficiente_o_ambigua' };
}
function rebuild(slam, map) {
  const rebuilt = createMap(map.grid); seedFootprint(rebuilt, slam.initialPose);
  for (const record of slam.records) {
    const p = recordPose(slam, record);
    integrate(rebuilt, p, projectScan(unpack(record), p, record.sensor));
  }
  // Bumper evidence is retained and moves with its observed anchor as well.
  for (const contact of slam.contacts) markBump(rebuilt, compose(slam.nodes[contact.anchor].pose, contact.local), contact.back, contact.distance);
  map.set(rebuilt); map.logodds.set(rebuilt.logodds); map.revision++;
}
export function observeSlam(slam, map, rays, pose, sensor = SENSOR, time = 0, allowClosure = true) {
  if (slam.records.length >= slam.config.maxScans) { slam.limited = true; slam.status = 'limite_de_memoria'; return { pose, rebuilt: false }; }
  slam.time = time;
  slam.travel += Math.hypot(pose.x - slam.previous.x, pose.z - slam.previous.z); slam.previous = clonePose(pose);
  const last = slam.nodes.at(-1);
  if (Math.hypot(pose.x - last.pose.x, pose.z - last.pose.z) >= .5 || Math.abs(wrap(pose.yaw - last.pose.yaw)) >= .35) {
    const from = slam.nodes.length - 1;
    slam.nodes.push({ pose: clonePose(pose), time, travel: slam.travel });
    slam.edges.push({ from, to: from + 1, measurement: between(last.pose, pose), weight: 100, yawWeight: 400, kind: 'local' });
  }
  const anchor = slam.nodes.length - 1;
  slam.records.push({ anchor, local: between(slam.nodes[anchor].pose, pose), ...pack(rays), sensor: { fov: sensor.fov }, time, travel: slam.travel });
  if (slam.records.length >= 25 && slam.records.length % 15 === 10) {
    slam.submaps.push(makeSubmap(slam));
    if (slam.submaps.length > slam.config.maxSubmaps) slam.submaps.shift();
  }
  if (!allowClosure) { slam.pending = null; slam.status = 'contacto_pendiente'; return { pose, rebuilt: false }; }
  if (time - slam.lastSearch < .5 || time - slam.lastClosure < slam.config.cooldown) return { pose, rebuilt: false };
  slam.lastSearch = time;
  const points = currentPoints(slam, pose);
  if (!distinctive(points)) { slam.pending = null; slam.status = 'sin_referencia_distinguible'; return { pose, rebuilt: false }; }
  const possible = slam.submaps.filter(m => time - m.time >= slam.config.minAge && slam.travel - m.travel >= slam.config.minTravel
    && Math.hypot(pose.x - slam.nodes[m.anchor].pose.x, pose.z - slam.nodes[m.anchor].pose.z) <= slam.config.candidateRadius);
  possible.sort((a, b) => a.time - b.time);
  let candidate;
  for (const submap of possible.slice(0, 3)) {
    slam.candidates++;
    const reference = slam.nodes[submap.anchor].pose, match = matchSubmap(submap, points, between(reference, pose), slam.config);
    if (!match.accepted) { slam.rejected++; continue; }
    const target = compose(reference, match.pose), delta = compose(target, between(pose, { x: 0, z: 0, yaw: 0 }));
    if (Math.hypot(delta.x, delta.z) < .06 && Math.abs(delta.yaw) < .025) continue;
    candidate = { submap, match, target, delta }; break;
  }
  if (!candidate) { slam.pending = null; slam.status = possible.length ? 'coincidencia_rechazada' : 'aprendiendo'; return { pose, rebuilt: false }; }
  const old = slam.pending, consistent = old && old.submap === candidate.submap.id && time - old.time <= 1.1
    && Math.hypot(old.delta.x - candidate.delta.x, old.delta.z - candidate.delta.z) < .12 && Math.abs(wrap(old.delta.yaw - candidate.delta.yaw)) < .06;
  slam.pending = { submap: candidate.submap.id, time, delta: candidate.delta, count: consistent ? old.count + 1 : 1 };
  slam.status = 'verificando';
  if (slam.pending.count < slam.config.confirmations) return { pose, rebuilt: false };
  // Add a current node so the constraint attaches to this observation exactly.
  const previous = slam.nodes.length - 1, current = slam.nodes.length;
  slam.nodes.push({ pose: clonePose(pose), time, travel: slam.travel });
  slam.edges.push({ from: previous, to: current, measurement: between(slam.nodes[previous].pose, pose), weight: 100, yawWeight: 400, kind: 'local' });
  const loop = { from: candidate.submap.anchor, to: current, measurement: candidate.match.pose, weight: 900, yawWeight: 1600, kind: 'loop' };
  const before = slam.nodes.map(n => clonePose(n.pose)), result = optimizeGraph(slam.nodes, [...slam.edges, loop]);
  const residual = between(compose(result.poses[loop.from], loop.measurement), result.poses[current]);
  const bounded = result.poses.every((p, i) => Math.hypot(p.x - before[i].x, p.z - before[i].z) < 1.2 && Math.abs(wrap(p.yaw - before[i].yaw)) < .4);
  if (!bounded || !(result.after < result.before) || Math.hypot(residual.x, residual.z) > .12 || Math.abs(residual.yaw) > .06) {
    slam.nodes.pop(); slam.edges.pop(); slam.pending = null; slam.rejected++; slam.status = 'optimizacion_rechazada'; return { pose, rebuilt: false };
  }
  slam.edges.push(loop);
  result.poses.forEach((p, i) => { slam.nodes[i].pose = p; });
  const corrected = result.poses[current];
  // Last scan was initially attached to previous; move it to corrected current.
  const record = slam.records.at(-1); record.anchor = current; record.local = { x: 0, z: 0, yaw: 0 };
  rebuild(slam, map);
  // Optimization changes the internal geometry of historical submaps too;
  // rebuild their fields instead of merely moving stale rigid snapshots.
  slam.submaps = slam.submaps.map(m => makeSubmap(slam, m.end));
  const event = { time, reference: loop.from, current, score: candidate.match.score, confirmations: slam.pending.count,
    correctionM: Math.hypot(corrected.x - pose.x, corrected.z - pose.z), correctionYaw: wrap(corrected.yaw - pose.yaw), costBefore: result.before, costAfter: result.after };
  slam.closures.push(event); slam.pending = null; slam.lastClosure = time; slam.previous = clonePose(corrected); slam.status = 'mapa_corregido';
  return { pose: corrected, rebuilt: true, event, before, after: result.poses };
}
export function slamSummary(slam) {
  return { status: slam.status, scans: slam.records.length, submaps: slam.submaps.length, nodes: slam.nodes.length,
    candidates: slam.candidates, rejected: slam.rejected, confirmations: slam.pending?.count ?? 0,
    closures: slam.closures.length, events: slam.closures.slice(-12), limited: slam.limited };
}
export function exportSlam(slam) {
  if (!slam) return null;
  return { config: slam.config, initialPose: slam.initialPose, ...slamSummary(slam), nodes: slam.nodes, edges: slam.edges,
    contacts: slam.contacts,
    observations: slam.records.map(r => ({ ...r, distances: Array.from(r.distances, d => Number.isFinite(d) ? d : null),
      freeDistances: Array.from(r.freeDistances, d => Number.isFinite(d) ? d : null), kinds: Array.from(r.kinds) })),
    description: 'Poses históricas corregidas en nodes; cada lectura se coloca con compose(nodes[anchor].pose, local). Ángulos desde calibración, sin pose real. 0 inválido, 1 pared, 2 suelo. Keyframes conservan su pose de captura y exponen pose_mapa corregida por separado.' };
}
