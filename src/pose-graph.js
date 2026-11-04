// SE(2) in the simulator's X/Z convention: positive yaw turns toward -X.
export const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
export function compose(a, b) {
  const c = Math.cos(a.yaw), s = Math.sin(a.yaw);
  return { x: a.x + c * b.x + s * b.z, z: a.z - s * b.x + c * b.z, yaw: wrap(a.yaw + b.yaw) };
}
export function between(a, b) {
  const c = Math.cos(a.yaw), s = Math.sin(a.yaw), x = b.x - a.x, z = b.z - a.z;
  return { x: c * x - s * z, z: s * x + c * z, yaw: wrap(b.yaw - a.yaw) };
}
const dot = (a, b) => a.reduce((v, x, i) => v + x * b[i], 0);
function terms(poses, edge) {
  const a = poses[edge.from], b = poses[edge.to], q = between(a, b), c = Math.cos(a.yaw), s = Math.sin(a.yaw);
  const r = [q.x - edge.measurement.x, q.z - edge.measurement.z, wrap(q.yaw - edge.measurement.yaw)];
  const A = [-c, s, -q.z, -s, -c, q.x, 0, 0, -1], B = [c, -s, 0, s, c, 0, 0, 0, 1];
  const weight = [edge.weight ?? 100, edge.weight ?? 100, edge.yawWeight ?? 400];
  return { r, A, B, weight, i: edge.from * 3, j: edge.to * 3 };
}
function cost(poses, edges) {
  let result = 0;
  for (const edge of edges) { const t = terms(poses, edge); for (let r = 0; r < 3; r++) result += t.r[r] ** 2 * t.weight[r]; }
  return result;
}
// Sparse Gauss-Newton + preconditioned conjugate gradients. Node 0 is fixed
// to the known launch pose, removing global translation/rotation gauge freedom.
export function optimizeGraph(nodes, edges, { iterations = 8 } = {}) {
  let poses = nodes.map(n => ({ ...n.pose }));
  const before = cost(poses, edges), size = poses.length * 3;
  for (let iteration = 0; iteration < iterations; iteration++) {
    const rows = edges.map(e => terms(poses, e)), gradient = new Float64Array(size), diagonal = new Float64Array(size).fill(.0001);
    for (const t of rows) for (let r = 0; r < 3; r++) for (let k = 0; k < 3; k++) {
      if (t.i) { gradient[t.i + k] += t.A[r * 3 + k] * t.weight[r] * t.r[r]; diagonal[t.i + k] += t.A[r * 3 + k] ** 2 * t.weight[r]; }
      if (t.j) { gradient[t.j + k] += t.B[r * 3 + k] * t.weight[r] * t.r[r]; diagonal[t.j + k] += t.B[r * 3 + k] ** 2 * t.weight[r]; }
    }
    function multiply(vector) {
      const result = Float64Array.from(vector, v => .0001 * v);
      for (const t of rows) for (let r = 0; r < 3; r++) {
        let v = 0;
        for (let k = 0; k < 3; k++) { if (t.i) v += t.A[r * 3 + k] * vector[t.i + k]; if (t.j) v += t.B[r * 3 + k] * vector[t.j + k]; }
        for (let k = 0; k < 3; k++) { if (t.i) result[t.i + k] += t.A[r * 3 + k] * t.weight[r] * v; if (t.j) result[t.j + k] += t.B[r * 3 + k] * t.weight[r] * v; }
      }
      result[0] = vector[0]; result[1] = vector[1]; result[2] = vector[2];
      return result;
    }
    const delta = new Float64Array(size), residual = Float64Array.from(gradient, v => -v);
    let z = Float64Array.from(residual, (v, i) => v / diagonal[i]), direction = z.slice(), rz = dot(residual, z);
    for (let step = 0; step < Math.min(200, size * 2) && rz > 1e-14; step++) {
      const product = multiply(direction), denom = dot(direction, product);
      if (denom <= 0) break;
      const alpha = rz / denom;
      for (let i = 3; i < size; i++) { delta[i] += alpha * direction[i]; residual[i] -= alpha * product[i]; }
      z = Float64Array.from(residual, (v, i) => v / diagonal[i]);
      const next = dot(residual, z), beta = next / rz;
      for (let i = 3; i < size; i++) direction[i] = z[i] + beta * direction[i];
      rz = next;
    }
    let scale = 1, candidate;
    const oldCost = cost(poses, edges);
    for (let search = 0; search < 8; search++, scale /= 2) {
      candidate = poses.map((p, i) => i ? { x: p.x + delta[i * 3] * scale, z: p.z + delta[i * 3 + 1] * scale, yaw: wrap(p.yaw + delta[i * 3 + 2] * scale) } : { ...p });
      if (cost(candidate, edges) <= oldCost) break;
    }
    if (cost(candidate, edges) > oldCost || !candidate.every(p => [p.x, p.z, p.yaw].every(Number.isFinite))) break;
    poses = candidate;
    if (Math.sqrt(dot(delta, delta)) * scale < .0001) break;
  }
  return { poses, before, after: cost(poses, edges) };
}
