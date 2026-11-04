import { cell, center, FREE, OCCUPIED } from './mapping.js';

// All passes start at the same known launch origin. Local SE(2) corrections
// below use occupied observations only; no scene or ground-truth pose inputs.
const identity = () => ({ x: 0, z: 0, yaw: 0 });
function transform(p, t) {
  const c = Math.cos(t.yaw), s = Math.sin(t.yaw);
  return { x: c * p.x - s * p.z + t.x, z: s * p.x + c * p.z + t.z };
}
function inverse(p, t) {
  const c = Math.cos(t.yaw), s = Math.sin(t.yaw), x = p.x - t.x, z = p.z - t.z;
  return { x: c * x + s * z, z: -s * x + c * z };
}
function occupiedPoints(trial) {
  const indices = [];
  for (let i = 0; i < trial.map.length; i++) if (trial.map[i] === OCCUPIED) indices.push(i);
  const stride = Math.max(1, Math.ceil(indices.length / 200));
  return indices.filter((_, i) => i % stride === 0).map(i => center(i, trial.grid));
}
export function alignMaps(reference, source) {
  const points = occupiedPoints(source), referenceCount = reference.map.reduce((n, v) => n + Number(v === OCCUPIED), 0);
  if (points.length < 20 || referenceCount < 20) return { ...identity(), status: 'origen_comun_sin_evidencia', applied: false };
  // A single straight wall cannot constrain translation along itself.
  const mean = points.reduce((a, p) => ({ x: a.x + p.x / points.length, z: a.z + p.z / points.length }), { x: 0, z: 0 });
  let xx = 0, zz = 0, xz = 0;
  for (const p of points) { const x = p.x - mean.x, z = p.z - mean.z; xx += x * x; zz += z * z; xz += x * z; }
  if (xx * zz - xz * xz < .0001 * (xx + zz) ** 2) return { ...identity(), status: 'origen_comun_alineacion_ambigua', applied: false };
  function score(t) {
    let total = 0, known = 0, matches = 0;
    for (const p of points) {
      const q = transform(p, t), i = cell(q.x, q.z, reference.grid), v = reference.map[i];
      if (v === OCCUPIED) { total++; known++; matches++; }
      else if (v === FREE) { total -= .7; known++; }
    }
    return { score: total / points.length, overlap: known / points.length, match: matches / points.length };
  }
  const candidates = [];
  for (let yi = -2; yi <= 2; yi++) for (let zi = -4; zi <= 4; zi++) for (let xi = -4; xi <= 4; xi++) {
    const t = { x: xi * .1, z: zi * .1, yaw: yi * .04 };
    candidates.push({ ...t, ...score(t) });
  }
  candidates.sort((a, b) => b.score - a.score || (Math.hypot(a.x, a.z) + Math.abs(a.yaw)) - (Math.hypot(b.x, b.z) + Math.abs(b.yaw)));
  const best = candidates[0], baseline = score(identity());
  const rival = candidates.find(t => Math.hypot(t.x - best.x, t.z - best.z) >= .199 || Math.abs(t.yaw - best.yaw) >= .079);
  const margin = best.score - (rival?.score ?? best.score);
  const reliable = best.match >= .5 && best.overlap >= .6 && margin >= .025;
  if (!reliable) return { ...identity(), status: 'origen_comun_alineacion_ambigua', applied: false, score: best.score, margin, overlap: best.overlap };
  const improves = best.score >= baseline.score + .02;
  return { ...(improves ? { x: best.x, z: best.z, yaw: best.yaw } : identity()),
    status: improves ? 'correccion_observada' : 'origen_comun_consistente', applied: improves,
    score: best.score, margin, overlap: best.overlap };
}

export function compareMaps(trials, { minVotes = 3 } = {}) {
  if (!trials.length || trials.length > 255) throw new RangeError('Se requieren entre 1 y 255 mapas');
  const grid = { ...trials[0].grid }, size = grid.width * grid.height;
  for (const trial of trials) if (trial.map.length !== trial.grid.width * trial.grid.height) throw new RangeError('Dimensiones de mapa inválidas');
  const freeVotes = new Uint8Array(size), occupiedVotes = new Uint8Array(size), observedVotes = new Uint8Array(size), classes = new Uint8Array(size);
  const alignments = trials.map((trial, i) => ({ id: trial.id, ...(i ? alignMaps(trials[0], trial) : { ...identity(), status: 'referencia', applied: false }) }));
  // Inverse sampling gives at most ONE vote per pass and destination cell.
  for (let pass = 0; pass < trials.length; pass++) for (let i = 0; i < size; i++) {
    const p = inverse(center(i, grid), alignments[pass]);
    const trial = trials[pass], source = cell(p.x, p.z, trial.grid), v = trial.map[source];
    if (v === FREE) freeVotes[i]++;
    if (v === OCCUPIED) occupiedVotes[i]++;
  }
  const stats = { pasadas: trials.length, desconocidas: 0, evidencia_insuficiente: 0, consenso_libre: 0, consenso_ocupado: 0, conflictos: 0,
    alineaciones_corregidas: alignments.filter(t => t.applied).length,
    alineaciones_sin_confirmar: alignments.filter(t => t.status.includes('ambigua') || t.status.includes('sin_evidencia')).length };
  for (let i = 0; i < size; i++) {
    const f = freeVotes[i], o = occupiedVotes[i], n = observedVotes[i] = f + o;
    if (!n) { classes[i] = 0; stats.desconocidas++; }
    else if (f && o) { classes[i] = 4; stats.conflictos++; }
    else if (n < minVotes) { classes[i] = 1; stats.evidencia_insuficiente++; }
    else if (f) { classes[i] = 2; stats.consenso_libre++; }
    else { classes[i] = 3; stats.consenso_ocupado++; }
  }
  return { grid, freeVotes, occupiedVotes, observedVotes, classes, stats, alignments, minVotes,
    caveat: 'Origen de lanzamiento común conocido. Alineación local rígida; si es ambigua se conserva ese origen y se señala. El sesgo compartido puede producir consenso erróneo. No se fusiona para navegar.' };
}
