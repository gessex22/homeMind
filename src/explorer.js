import { plan, routeTo, traversable, footprintClear } from './planner.js';
import { center, gridOf, markBump, FREE } from './mapping.js';
import { TURN_SPEED, SPEED, predictMotion } from './motion.js';
import { createLocalizationMonitor, tickLocalization } from './localization.js';
import { sweepCovered } from './sweep-coverage.js';
const zero = () => ({ forward: false, back: false, left: false, right: false });
// Recovery/planning retries at the same spot before judging the mission
// limited (provided the planner also sees no way out; see recover()).
export const MAX_RECOVERY_ATTEMPTS = 12;
export function createExplorer(footprint = .25) { return { state: 'inactivo', resumeState: null, sweepCoverage: null, route: [], fronts: [], scanned: [], scanAngle: 0, safe: null, safeRevision: -1, base: null, destino: null, elapsed: 0, leash: null, retorno: null, retrocede: 0, retrocedido: 0, topesSeguidos: 0, bloqueos: 0, reintentos: 0, footprint, cerradas: new Set(), localizacion: createLocalizationMonitor(), relocalizeAngle: 0, avoid: new Set(), recoveryAttempts: 0, recoveryOrigin: null, recoveryCause: null }; }
export function startExplorer(e) {
  if (e.state === 'pausado' && e.resumeState) { e.state = e.resumeState; e.resumeState = null; return; }
  if (['inactivo', 'pausado', 'limitado', 'bloqueado', 'refugio', 'localizacion_perdida', 'destino_alcanzado'].includes(e.state)) {
    e.state = e.state === 'localizacion_perdida' && e.leash ? 'relocalizando' : 'barrido';
    e.scanAngle = 0; e.relocalizeAngle = 0; e.route = []; e.retorno = null; e.escape = null;
  }
}
export function setDestination(e, destino) {
  e.escape = null;
  if (destino == null) {
    e.destino = null;
    if (e.state === 'pausado' && ['destino', 'destino_alcanzado'].includes(e.resumeState)) { e.resumeState = 'planificando'; e.route = []; }
    if (['destino', 'destino_alcanzado'].includes(e.state)) { e.route = []; e.state = 'planificando'; }
    return;
  }
  if (!Number.isFinite(destino.x) || !Number.isFinite(destino.z)) throw new RangeError('Destino inválido');
  e.destino = { x: destino.x, z: destino.z };
  e.recoveryCause = null; e.recoveryState = null; e.recoveryAttempts = 0; e.recoveryOrigin = null;
  if (e.state === 'pausado') { e.resumeState = 'planificando'; e.route = []; }
  // Preempt frontier driving so the next plan heads for the destination.
  if (['navegando', 'destino', 'destino_alcanzado', 'barrido'].includes(e.state)) { e.route = []; e.state = 'planificando'; }
}
export function pauseExplorer(e) {
  if (!['inactivo', 'pausado', 'completado', 'limitado', 'bloqueado', 'refugio', 'localizacion_perdida'].includes(e.state)) {
    e.resumeState = e.state; e.state = 'pausado';
  }
}
export function setLeash(e, cfg) {
  if (cfg == null) { e.leash = null; return; }
  const { presupuestoS, maxDistanciaM, margen = 1.5 } = cfg;
  if (![presupuestoS, maxDistanciaM, margen].every(Number.isFinite) || presupuestoS <= 0 || maxDistanciaM <= 0 || margen < 1) throw new RangeError('Parámetros de retorno inválidos');
  e.leash = { presupuestoS, maxDistanciaM, margen };
}
// The swept circular footprint is a capsule. Check its exact distance to
// every intersecting grid cell, including unknown and out-of-map cells.
function segmentClear(map, from, to, radius, escape = false) {
  const g = gridOf(map), dx = to.x - from.x, dz = to.z - from.z, length2 = dx * dx + dz * dz;
  const minX = Math.floor((Math.min(from.x, to.x) - radius - g.originX) / g.resolution);
  const maxX = Math.floor((Math.max(from.x, to.x) + radius - g.originX) / g.resolution);
  const minZ = Math.floor((Math.min(from.z, to.z) - radius - g.originZ) / g.resolution);
  const maxZ = Math.floor((Math.max(from.z, to.z) + radius - g.originZ) / g.resolution);
  function pointRectDistance2(p, left, top, right, bottom) {
    const x = Math.max(left - p.x, 0, p.x - right), z = Math.max(top - p.z, 0, p.z - bottom);
    return x * x + z * z;
  }
  function cornerDistance2(x, z) {
    const t = length2 ? Math.max(0, Math.min(1, ((x - from.x) * dx + (z - from.z) * dz) / length2)) : 0;
    return (x - from.x - t * dx) ** 2 + (z - from.z - t * dz) ** 2;
  }
  let overlap = false, improving = false;
  for (let z = minZ; z <= maxZ; z++) for (let x = minX; x <= maxX; x++) {
    if (x >= 0 && z >= 0 && x < g.width && z < g.height && map[z * g.width + x] === FREE) continue;
    const left = g.originX + x * g.resolution, top = g.originZ + z * g.resolution;
    const right = left + g.resolution, bottom = top + g.resolution;
    const startDistance2 = pointRectDistance2(from, left, top, right, bottom);
    if (escape && startDistance2 <= radius * radius) {
      // Escape only a shallow, pre-existing overlap, never a new cell.
      const distance = Math.sqrt(startDistance2);
      const maxOverlap = Math.min(.075, Math.max(.05, g.resolution * Math.SQRT2), radius * .75);
      if (distance <= 1e-9 || distance + 1e-9 < radius - maxOverlap) return false;
      const nearestX = Math.max(left, Math.min(right, from.x));
      const nearestZ = Math.max(top, Math.min(bottom, from.z));
      const derivative = (from.x - nearestX) * dx + (from.z - nearestZ) * dz;
      if (derivative < -1e-12) return false;
      const endDistance2 = pointRectDistance2(to, left, top, right, bottom);
      if (endDistance2 + 1e-12 < startDistance2) return false;
      overlap = true;
      if (endDistance2 > startDistance2 + 1e-12) improving = true;
      continue;
    }
    let lo = 0, hi = 1;
    for (const [origin, delta, min, max] of [[from.x, dx, left, right], [from.z, dz, top, bottom]]) {
      if (delta === 0) { if (origin < min || origin > max) { lo = 1; hi = 0; break; } }
      else { const a = (min - origin) / delta, b = (max - origin) / delta; lo = Math.max(lo, Math.min(a, b)); hi = Math.min(hi, Math.max(a, b)); }
    }
    if (lo <= hi || Math.min(pointRectDistance2(from, left, top, right, bottom), pointRectDistance2(to, left, top, right, bottom),
      cornerDistance2(left, top), cornerDistance2(right, top), cornerDistance2(left, bottom), cornerDistance2(right, bottom)) <= radius * radius) return false;
  }
  return !overlap || improving;
}
// Prove a route exists after a bounded straight retreat from a shallow
// overlap. This uses only the belief map and the same swept-footprint guard
// as recovery; no obstacle cells are cleared or inferred to be free.
export function routeAfterRetreat(map, pose, goal, { radius = .25, avoid = null } = {}) {
  if (!Number.isFinite(pose.yaw)) return null;
  for (let d = .025; d <= .500001; d += .025) {
    const escaped = { x: pose.x + Math.sin(pose.yaw) * d, z: pose.z + Math.cos(pose.yaw) * d, yaw: pose.yaw };
    if (!avoidClear(map, pose, escaped, avoid) || !segmentClear(map, pose, escaped, radius, true)) continue;
    if (!footprintClear(map, escaped, radius)) continue;
    const route = routeTo(map, escaped, goal, { radius, avoid });
    if (route && (route.length || Math.hypot(escaped.x - goal.x, escaped.z - goal.z) < .35)) return { pose: escaped, route };
  }
  return null;
}
// Avoidance records forbidden robot-center cells, not obstacles. A robot
// already inside such a region may leave it, but cannot enter it again.
function avoidClear(map, from, to, avoid) {
  if (!avoid?.size) return true;
  const g = gridOf(map), dx = to.x - from.x, dz = to.z - from.z;
  const minX = Math.max(0, Math.floor((Math.min(from.x, to.x) - g.originX) / g.resolution));
  const maxX = Math.min(g.width - 1, Math.floor((Math.max(from.x, to.x) - g.originX) / g.resolution));
  const minZ = Math.max(0, Math.floor((Math.min(from.z, to.z) - g.originZ) / g.resolution));
  const maxZ = Math.min(g.height - 1, Math.floor((Math.max(from.z, to.z) - g.originZ) / g.resolution));
  const intervals = [];
  for (let z = minZ; z <= maxZ; z++) for (let x = minX; x <= maxX; x++) {
    if (!avoid.has(z * g.width + x)) continue;
    let lo = 0, hi = 1;
    for (const [origin, delta, min] of [[from.x, dx, g.originX + x * g.resolution], [from.z, dz, g.originZ + z * g.resolution]]) {
      const max = min + g.resolution;
      if (!delta) { if (origin < min || origin > max) { lo = 1; hi = 0; break; } }
      else { const a = (min - origin) / delta, b = (max - origin) / delta; lo = Math.max(lo, Math.min(a, b)); hi = Math.min(hi, Math.max(a, b)); }
    }
    if (lo <= hi) intervals.push([lo, hi]);
  }
  if (!intervals.length) return true;
  intervals.sort((a, b) => a[0] - b[0]);
  if (intervals[0][0] > 1e-9) return false;
  let end = intervals[0][1];
  for (const [lo, hi] of intervals.slice(1)) {
    if (lo > end + 1e-9) return false;
    end = Math.max(end, hi);
  }
  return true;
}
function sampledClear(map, from, to, radius, spacing = .025) {
  const distance = Math.hypot(to.x - from.x, to.z - from.z);
  const count = Math.max(1, Math.ceil(distance / Math.min(spacing, gridOf(map).resolution / 2)));
  const margin = distance / count / 2;
  for (let i = 0; i < count; i++) {
    const t = (i + .5) / count;
    if (!footprintClear(map, { x: from.x + (to.x - from.x) * t, z: from.z + (to.z - from.z) * t }, radius + margin)) return false;
  }
  return true;
}
function stepAlong(e, map, pose, dt, command) {
  const safe = e.safe && e.safeRevision === map.revision ? e.safe : traversable(map, e.footprint * .8);
  e.safe = safe; e.safeRevision = map.revision;
  if (e.route.some(i => !safe[i])) return 'invalid';
  while (e.route.length && Math.hypot(center(e.route[0], map).x - pose.x, center(e.route[0], map).z - pose.z) < .055) e.route.shift();
  if (!e.route.length) return 'empty';

  // Look ahead along the route, but only shortcut through fully observed
  // space with clearance for the whole robot, including around corners.
  let selected = 0;
  let limit = 0;
  while (e.state !== 'navegando' && limit + 1 < e.route.length && Math.hypot(center(e.route[limit + 1], map).x - pose.x, center(e.route[limit + 1], map).z - pose.z) <= .6) limit++;
  for (let i = limit; i > 0; i--) {
    if (avoidClear(map, pose, center(e.route[i], map), e.avoid) && sampledClear(map, pose, center(e.route[i], map), e.footprint)) { selected = i; break; }
  }
  if (selected) e.route.splice(0, selected);
  const target = center(e.route[0], map), dx = target.x - pose.x, dz = target.z - pose.z;
  const desired = Math.atan2(-dx, -dz), delta = Math.atan2(Math.sin(desired - pose.yaw), Math.cos(desired - pose.yaw));
  // Exploration tracks each planned cell, turning in place before advancing.
  // Destination and return use proportional steering on observed routes.
  if (e.state === 'navegando') {
    if (Math.abs(delta) > .035) command[delta > 0 ? 'left' : 'right'] = true;
    else command.forward = true;
  } else {
    const turn = Math.min(1, Math.abs(delta) * 1.5, Math.abs(delta) / (TURN_SPEED * dt));
    if (turn > .001) command[delta > 0 ? 'left' : 'right'] = turn;
    if (Math.abs(delta) < 1.1) command.forward = Math.min(1, Math.hypot(dx, dz) / (SPEED * dt), Math.max(.2, 1 - Math.abs(delta) / 1.1));
  }
  const next = predictMotion(pose, command, dt);
  // Keep a small safety reserve for measurement uncertainty. If the robot
  // already sits inside that reserve, allow a geometrically clear step that
  // escapes it instead of trapping the robot at the boundary.
  const huellaOk = !command.forward || (sampledClear(map, pose, next, e.footprint, .005)
    || (!footprintClear(map, pose, e.footprint + .0025)
      && footprintClear(map, next, e.footprint + .0025)
      && segmentClear(map, pose, next, e.footprint)));
  // A stop caused only by avoidance does not need obstacle recovery.
  // Distinguish a steering curve from a genuinely forbidden route segment.
  if (command.forward && huellaOk && !avoidClear(map, pose, next, e.avoid)) {
    command.forward = false;
    // A steering curve may enter a forbidden cell even when the straight
    // segment to the waypoint is clear. Align first instead of excluding
    // that valid waypoint and gradually sealing off the entire exit.
    if (Math.abs(delta) > .035 && avoidClear(map, pose, target, e.avoid)) return 'moving';
    e.avoid.add(e.route[0]); return 'invalid';
  }
  const safeStep = huellaOk && avoidClear(map, pose, next, e.avoid);
  if (!safeStep) {
    // If the curve drifts toward an edge, align in place first. A circular
    // footprint does not change during rotation; retry the translation later.
    if (Math.abs(delta) > .035 && footprintClear(map, pose, e.footprint)) command.forward = false;
    else return 'blocked';
  }
  return 'moving';
}
// A blocked direction is robot-side navigation memory, not an occupancy
// observation. Keep it separate from the map and exclude it from replans.
// The step handlers also add the exact route cell that could not be
// entered (planner tiers can accept a cell the full-footprint step guard
// rejects), so the next replan does not offer the same tight cell again.
function avoidAhead(e, map, pose) {
  const g = gridOf(map), distance = e.footprint + .1;
  const x = pose.x - Math.sin(pose.yaw) * distance, z = pose.z - Math.cos(pose.yaw) * distance;
  const n = Math.ceil(.18 / g.resolution), col = Math.floor((x - g.originX) / g.resolution), row = Math.floor((z - g.originZ) / g.resolution);
  for (let dz = -n; dz <= n; dz++) for (let dx = -n; dx <= n; dx++) {
    const cx = col + dx, cz = row + dz;
    if (cx >= 0 && cz >= 0 && cx < g.width && cz < g.height && Math.hypot(dx, dz) * g.resolution <= .18) e.avoid.add(cz * g.width + cx);
  }
}
// When reversing is blocked, find a direction that clears the existing
// overlap without entering a new non-free cell, then proves a route onward.
function forwardEscape(e, map, pose) {
  const goal = e.recoveryState === 'retorno' ? e.base : e.destino;
  const angles = [0, Math.PI / 4, -Math.PI / 4, Math.PI / 2, -Math.PI / 2, 3 * Math.PI / 4, -3 * Math.PI / 4, Math.PI];
  for (const offset of angles) {
    const yaw = Math.atan2(Math.sin(pose.yaw + offset), Math.cos(pose.yaw + offset));
    for (let d = .05; d <= .500001; d += .05) {
      const point = { x: pose.x - Math.sin(yaw) * d, z: pose.z - Math.cos(yaw) * d, yaw };
      if (!avoidClear(map, pose, point, e.avoid) || !segmentClear(map, pose, point, e.footprint, true)) continue;
      if (!footprintClear(map, point, e.footprint)) continue;
      let route = null;
      if (goal) {
        route = routeTo(map, point, goal, { radius: e.footprint, avoid: e.avoid });
        if (!route || (!route.length && Math.hypot(point.x - goal.x, point.z - goal.z) >= .35)) continue;
      } else if (plan(map, point, e.scanned, { radius: e.footprint, cerradas: e.cerradas, avoid: e.avoid }).reason !== 'ruta') continue;
      return { point, yaw, route };
    }
  }
  return null;
}
// A selector can offer a goal only when the current map proves either a
// safe retreat or a safe oriented exit with a connected route afterwards.
export function routeAfterRecovery(map, pose, goal, { radius = .25, avoid = null } = {}) {
  const retreat = routeAfterRetreat(map, pose, goal, { radius, avoid });
  if (retreat) return { ...retreat, mode: 'back' };
  const escape = forwardEscape({ footprint: radius, destino: goal, recoveryState: null, avoid, scanned: [], cerradas: null }, map, pose);
  return escape ? { pose: escape.point, route: escape.route, mode: 'forward' } : null;
}
function recover(e, map, pose, dt, cause, command) {
  Object.assign(command, zero());
  e.escape = null;
  if (!e.recoveryOrigin || Math.hypot(pose.x - e.recoveryOrigin.x, pose.z - e.recoveryOrigin.z) > .5) {
    e.recoveryAttempts = 0; e.recoveryOrigin = { x: pose.x, z: pose.z };
  }
  if (cause !== 'contacto') e.recoveryAttempts++;
  if (e.recoveryAttempts > MAX_RECOVERY_ATTEMPTS) {
    // Exhausted retries only end the mission when the planner also sees
    // no way out. If a route (full or partial) still exists after marking
    // the blocked cells to avoid, reset the budget and keep recovering:
    // the avoided cells change the route, so this is not the same retry.
    if (cause === 'suelo' || cause === 'mapa') avoidAhead(e, map, pose);
    const haySalida = e.destino
      ? (routeTo(map, pose, e.destino, { radius: e.footprint, avoid: e.avoid, parcial: true })?.length ?? 0) > 0
      : plan(map, pose, e.scanned, { radius: e.footprint, cerradas: e.cerradas, avoid: e.avoid }).reason === 'ruta';
    if (!haySalida) { e.state = 'limitado'; e.route = []; return; }
    e.recoveryAttempts = 0; e.recoveryOrigin = { x: pose.x, z: pose.z };
  }
  if (cause === 'suelo' || cause === 'mapa') avoidAhead(e, map, pose);
  e.recoveryState = e.state; e.state = 'retroceso'; e.recoveryCause = cause; e.topesSeguidos = 0; e.retrocedido = 0; e.route = [];
  const back = predictMotion(pose, { ...zero(), back: true }, dt);
  if (avoidClear(map, pose, back, e.avoid) && segmentClear(map, pose, back, e.footprint, true)) { e.retrocede = 50; command.back = true; }
  else {
    if (cause === 'contacto') markBump(map, pose, false, e.footprint);
    const escape = forwardEscape(e, map, pose);
    if (escape) { e.escape = { ...escape, phase: 'giro', waitSince: 0 }; }
    else { e.escape = null; e.retrocede = 30; command.left = true; }
  }
}
export function exploreStep(e, map, pose, dt, choque = false, groundStop = false, freshScan = false) {
  const command = zero();
  if (!['barrido', 'planificando', 'navegando', 'retorno', 'retroceso', 'relocalizando', 'destino'].includes(e.state)) return command;
  if (!e.base) e.base = { x: pose.x, z: pose.z };
  e.elapsed += dt;
  tickLocalization(e.localizacion, dt);
  if (e.leash && e.localizacion.lostSeconds + 1e-9 >= e.leash.presupuestoS && e.state !== 'relocalizando') {
    e.state = 'relocalizando'; e.relocalizeAngle = 0; e.route = []; e.retorno = 'relocalizando';
  }
  if (e.state === 'relocalizando') {
    if (e.localizacion.confirmed) { e.state = 'planificando'; e.retorno = null; return command; }
    if (e.relocalizeAngle < Math.PI * 2) { e.relocalizeAngle += TURN_SPEED * dt; command[e.scanDirection === -1 ? 'right' : 'left'] = true; return command; }
    e.state = 'localizacion_perdida'; e.retorno = 'sin_localizacion'; e.route = []; return command;
  }
  if (e.leash && !['retorno', 'retroceso'].includes(e.state) && Math.hypot(pose.x - e.base.x, pose.z - e.base.z) >= e.leash.maxDistanciaM) {
    if (!e.localizacion.confirmed) { e.state = 'relocalizando'; e.relocalizeAngle = 0; e.route = []; return command; }
    const back = routeTo(map, pose, e.base, { radius: e.footprint, avoid: e.avoid });
    if (back?.length) { e.route = back; e.safeRevision = -1; e.state = 'retorno'; e.retorno = 'retornando'; }
    else if (back) { e.state = 'refugio'; e.retorno = 'base'; return command; }
    else { e.state = 'bloqueado'; e.retorno = 'fallido'; return command; }
  }
  if ((choque || groundStop) && ['navegando', 'retorno', 'destino'].includes(e.state)) {
    e.scanned.push({ x: pose.x, z: pose.z });
    recover(e, map, pose, dt, choque ? 'contacto' : 'suelo', command);
    return command;
  }
  if (e.state === 'barrido') {
    if (e.scanAngle < Math.PI * 2 && !sweepCovered(e.sweepCoverage)) { command[e.scanDirection === -1 ? 'right' : 'left'] = true; e.scanAngle += TURN_SPEED * dt; return command; }
    e.scanned.push({ x: pose.x, z: pose.z }); e.state = 'planificando';
  }
  if (e.state === 'planificando') {
    // A return interrupted by recovery remains a return, never exploration.
    if (e.recoveryState === 'retorno' && e.base) {
      const back = routeTo(map, pose, e.base, { radius: e.footprint, avoid: e.avoid });
      if (back?.length) { e.route = back; e.safeRevision = -1; e.state = 'retorno'; e.recoveryState = null; return command; }
      if (back && Math.hypot(pose.x - e.base.x, pose.z - e.base.z) < .35) { e.state = 'refugio'; e.retorno = 'base'; return command; }
      if (!footprintClear(map, pose, e.footprint) && routeAfterRecovery(map, pose, e.base, { radius: e.footprint, avoid: e.avoid })) {
        e.state = 'retorno'; recover(e, map, pose, dt, 'mapa', command); return command;
      }
      e.state = 'bloqueado'; e.retorno = 'fallido'; return command;
    }
    // A destination outranks frontiers; without a known route yet, fall back
    // to exploration and retry on the next plan as the map grows.
    if (e.destino) {
      if (!footprintClear(map, pose, e.footprint)) {
        recover(e, map, pose, dt, 'solape', command); return command;
      }
      // Partial approach counts: advance the clear part toward the goal
      // instead of refusing to move when the full footprint cannot reach it.
      const hacia = routeTo(map, pose, e.destino, { radius: e.footprint, avoid: e.avoid, parcial: true });
      if (hacia && hacia.length) { e.route = hacia; e.safeRevision = -1; e.state = 'destino'; return command; }
      if (hacia) { e.state = e.recoveryCause && Math.hypot(pose.x - e.destino.x, pose.z - e.destino.z) >= .35 ? 'limitado' : 'destino_alcanzado'; e.route = []; return command; }
      if (e.recoveryCause) { e.state = 'limitado'; return command; }
    }
    const result = plan(map, pose, e.scanned, { reverse: e.scanDirection === -1, radius: e.footprint, cerradas: e.cerradas, avoid: e.avoid }); e.route = result.route; e.fronts = result.fronts; e.safe = result.safe; e.safeRevision = map.revision;
    e.state = result.reason === 'ruta' ? 'navegando' : result.reason;
    if (e.state === 'bloqueado') {
      recover(e, map, pose, dt, 'plan', command);
      return command;
    }
  }
  if (e.state === 'destino') {
    const status = stepAlong(e, map, pose, dt, command);
    if (status === 'blocked') { if (e.route[0] !== undefined) e.avoid.add(e.route[0]); recover(e, map, pose, dt, footprintClear(map, pose, e.footprint) ? 'mapa' : 'solape', command); return command; }
    if (status === 'invalid' && !footprintClear(map, pose, e.footprint)) { recover(e, map, pose, dt, 'solape', command); return command; }
    if (status === 'invalid' || status === 'empty') {
      if (Math.hypot(pose.x - e.destino.x, pose.z - e.destino.z) < .35) { e.state = 'destino_alcanzado'; e.route = []; }
      else e.state = 'planificando';
    }
    return command;
  }
  if (e.state === 'navegando') {
    const status = stepAlong(e, map, pose, dt, command);
    if (status === 'invalid') { if (!footprintClear(map, pose, e.footprint)) recover(e, map, pose, dt, 'solape', command); else e.state = 'planificando'; return command; }
    if (status === 'blocked') { if (e.route[0] !== undefined) e.avoid.add(e.route[0]); recover(e, map, pose, dt, footprintClear(map, pose, e.footprint) ? 'mapa' : 'solape', command); return command; }
    if (status === 'empty') { e.state = 'barrido'; e.scanAngle = 0; e.bloqueos = 0; }
    return command;
  }
  if (e.state === 'retorno') {
    const status = stepAlong(e, map, pose, dt, command);
    if (status === 'invalid' && !footprintClear(map, pose, e.footprint)) { recover(e, map, pose, dt, 'solape', command); return command; }
    if (status === 'blocked') { if (e.route[0] !== undefined) e.avoid.add(e.route[0]); recover(e, map, pose, dt, footprintClear(map, pose, e.footprint) ? 'mapa' : 'solape', command); return command; }
    if (status === 'invalid' || status === 'empty') {
      if (Math.hypot(pose.x - e.base.x, pose.z - e.base.z) < .35) { e.state = 'refugio'; e.retorno = 'base'; e.route = []; }
      else {
        const back = routeTo(map, pose, e.base, { radius: e.footprint, avoid: e.avoid });
        if (back && back.length) { e.route = back; e.safeRevision = -1; }
        else if (back) { e.state = 'refugio'; e.retorno = 'base'; e.route = []; }
        else { e.state = 'bloqueado'; e.retorno = 'fallido'; e.route = []; }
      }
    }
    return command;
  }
  if (e.state === 'retroceso') {
    if (e.escape) {
      const escape = e.escape;
      if (groundStop || choque) { e.escape = null; e.state = 'limitado'; return command; }
      const delta = Math.atan2(Math.sin(escape.yaw - pose.yaw), Math.cos(escape.yaw - pose.yaw));
      if (escape.phase === 'giro') {
        if (Math.abs(delta) > .035) command[delta > 0 ? 'left' : 'right'] = Math.min(1, Math.abs(delta) / (TURN_SPEED * dt));
        else { escape.phase = 'observando'; escape.waitSince = e.elapsed; }
        return command;
      }
      if (escape.phase === 'observando') {
        if (!freshScan) { if (e.elapsed - escape.waitSince > 3) { e.escape = null; e.state = 'limitado'; } return command; }
        if (!avoidClear(map, pose, escape.point, e.avoid) || !segmentClear(map, pose, escape.point, e.footprint, true)) {
          e.escape = null; e.state = 'planificando'; return command;
        }
        escape.phase = 'avance';
      }
      const distance = Math.hypot(escape.point.x - pose.x, escape.point.z - pose.z);
      if (distance < .02 && footprintClear(map, pose, e.footprint)) { e.escape = null; e.state = 'planificando'; return command; }
      if (Math.abs(delta) > .08) { escape.phase = 'giro'; return command; }
      command.forward = Math.min(.3, distance / (SPEED * dt));
      const next = predictMotion(pose, command, dt);
      if (!avoidClear(map, pose, next, e.avoid) || !segmentClear(map, pose, next, e.footprint, true)) {
        e.escape = null; e.state = 'planificando'; return zero();
      }
      return command;
    }
    if (groundStop) e.retrocede = Math.min(e.retrocede, 30);
    e.topesSeguidos = choque ? e.topesSeguidos + 1 : 0;
    if (e.retrocede > 30) {
      const back = predictMotion(pose, { ...zero(), back: true }, dt);
      if (e.topesSeguidos >= 3 || !avoidClear(map, pose, back, e.avoid) || !segmentClear(map, pose, back, e.footprint, true)) {
        if (e.recoveryCause === 'contacto') markBump(map, pose, false, e.footprint + e.retrocedido);
        e.retrocede = 30;
      } else {
        e.retrocede--; e.retrocedido += SPEED * dt;
        if (e.retrocede === 30 && e.recoveryCause === 'contacto') markBump(map, pose, false, e.footprint + e.retrocedido);
        command.back = true;
        return command;
      }
    }
    if (e.retrocede > 0) { e.retrocede--; command.left = true; return command; }
    e.state = 'planificando';
    return command;
  }
  return command;
}
