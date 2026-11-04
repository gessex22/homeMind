import { gridOf, cell, center, FREE, OCCUPIED, UNKNOWN } from './mapping.js';
import { footprintClear } from './planner.js';

// Two different projections, kept apart on purpose:
// - Detection/scanning: the camera cone and its center ray. Wide is
//   fine here; triangulation only needs to see and locate.
// - Advance/route tracing: a corridor exactly the vehicle's width.
//   A route exists only if that whole swept width is verified free —
//   a center ray slipping through a gap narrower than the body does
//   not make the gap a route.
export const ANCHO_VEHICULO = .2;

// Center-ray projection (visibility/detection) over the believed map:
// march from a viewpoint along its heading and measure the run before
// the first occupied cell. Unknown cells do not block sight —
// photographing the unknown is the point — but their first distance is
// reported so callers can tell seen-free from not-yet-seen.
export function proyeccionFrontal(map, punto, direccion, { maxDist = 3.2, paso = null } = {}) {
  const g = gridOf(map), step = paso ?? g.resolution / 2;
  const norma = Math.hypot(direccion.x, direccion.z);
  if (!(norma > 0) || !(maxDist > 0)) return { distanciaLibre: 0, distanciaDesconocida: 0, hayPaso: false, interferencia: null };
  const ux = direccion.x / norma, uz = direccion.z / norma;
  let desconocida = Infinity;
  for (let d = step; d <= maxDist; d += step) {
    const i = cell(punto.x + ux * d, punto.z + uz * d, map);
    if (i < 0) return { distanciaLibre: d - step, distanciaDesconocida: desconocida, hayPaso: false, interferencia: null };
    if (map[i] === OCCUPIED) return { distanciaLibre: d, distanciaDesconocida: desconocida, hayPaso: desconocida === Infinity || desconocida < d, interferencia: center(i, map) };
    if (map[i] === UNKNOWN && desconocida === Infinity) desconocida = d;
  }
  return { distanciaLibre: maxDist, distanciaDesconocida: desconocida, hayPaso: true, interferencia: null };
}

// Vehicle-width corridor projection (advance/route tracing): march the
// swept footprint — lateral samples across exactly `ancho` — and
// report where an occupied cell first cuts the corridor
// (distanciaLibre) and where verified-free floor first runs out
// (distanciaVerificada). A route is only real when the verified run
// covers the whole advance.
export function proyeccionCorredor(map, punto, direccion, { maxDist = 3.2, ancho = ANCHO_VEHICULO, paso = null } = {}) {
  const g = gridOf(map), step = paso ?? g.resolution / 2;
  const norma = Math.hypot(direccion.x, direccion.z);
  if (!(norma > 0) || !(maxDist > 0) || !(ancho > 0)) return { distanciaLibre: 0, distanciaVerificada: 0, hayPaso: false, interferencia: null };
  const ux = direccion.x / norma, uz = direccion.z / norma, radio = ancho / 2;
  // Classify the swept disc at one step: fully FREE, cut by an OCCUPIED
  // cell (returns its center), or merely unverified (unknown/outside).
  const disco = p => {
    const minX = Math.floor((p.x - radio - g.originX) / g.resolution), maxX = Math.floor((p.x + radio - g.originX) / g.resolution);
    const minZ = Math.floor((p.z - radio - g.originZ) / g.resolution), maxZ = Math.floor((p.z + radio - g.originZ) / g.resolution);
    let ocupado = null, libre = true;
    for (let z = minZ; z <= maxZ; z++) for (let x = minX; x <= maxX; x++) {
      const left = g.originX + x * g.resolution, top = g.originZ + z * g.resolution;
      const dx = Math.max(left - p.x, 0, p.x - left - g.resolution), dz = Math.max(top - p.z, 0, p.z - top - g.resolution);
      if (dx * dx + dz * dz > radio * radio) continue;
      if (x < 0 || z < 0 || x >= g.width || z >= g.height) { libre = false; continue; }
      const i = z * g.width + x;
      if (map[i] === OCCUPIED && !ocupado) ocupado = center(i, map);
      if (map[i] !== FREE) libre = false;
    }
    return { ocupado, libre };
  };
  let verificada = maxDist;
  for (let d = step; d <= maxDist; d += step) {
    const { ocupado, libre } = disco({ x: punto.x + ux * d, z: punto.z + uz * d });
    if (ocupado) return { distanciaLibre: d, distanciaVerificada: Math.min(verificada, d - step), hayPaso: false, interferencia: ocupado };
    if (!libre && verificada === maxDist) verificada = d - step;
  }
  return { distanciaLibre: maxDist, distanciaVerificada: verificada, hayPaso: verificada >= maxDist, interferencia: null };
}

// Choose where the element's camera should stand to photograph a zone:
// far enough back that the photo is legible, reachable through a
// vehicle-width corridor of verified-free floor, with its footprint
// (exactly the body radius) clear, and with sight unblocked all the
// way to the zone. Candidates are tried far-to-near; if none works,
// the honest fallback is the element's own believed position, looking
// at the zone from there, with whatever interferes named.
export function elegirPuntoDeFoto(map, desde, objetivo, { distanciaMax = 2.4, distanciaMin = .9, ancho = ANCHO_VEHICULO, paso = .1 } = {}) {
  const dx = objetivo.x - desde.x, dz = objetivo.z - desde.z, total = Math.hypot(dx, dz);
  const yawHacia = (ux, uz) => Math.atan2(-ux, -uz);
  if (!(total > 0)) return { x: desde.x, z: desde.z, yaw: 0, distancia: 0, distanciaLibre: 0, interferencia: null, modo: 'desde-elemento' };
  const ux = dx / total, uz = dz / total, yaw = yawHacia(ux, uz);
  for (let d = Math.min(distanciaMax, total); d >= distanciaMin; d -= paso) {
    const punto = { x: objetivo.x - ux * d, z: objetivo.z - uz * d };
    if (!footprintClear(map, punto, ancho / 2)) continue;
    const avance = Math.hypot(punto.x - desde.x, punto.z - desde.z);
    if (avance > 0) {
      const corredor = proyeccionCorredor(map, desde, { x: punto.x - desde.x, z: punto.z - desde.z }, { maxDist: avance, ancho });
      if (!corredor.hayPaso) continue;
    }
    const vista = proyeccionFrontal(map, punto, { x: ux, z: uz }, { maxDist: d + .3 });
    if (vista.distanciaLibre < d) continue;
    return { ...punto, yaw, distancia: d, distanciaLibre: vista.distanciaLibre, interferencia: null, modo: 'proyectado' };
  }
  const corredor = proyeccionCorredor(map, desde, { x: ux, z: uz }, { maxDist: total, ancho });
  const vista = proyeccionFrontal(map, desde, { x: ux, z: uz }, { maxDist: total + .3 });
  const interferencia = corredor.interferencia ?? (vista.distanciaLibre < total ? vista.interferencia : null);
  return { x: desde.x, z: desde.z, yaw, distancia: total, distanciaLibre: vista.distanciaLibre, interferencia, modo: 'desde-elemento' };
}

// Evidence must come from the actual sensor pose, never the proposed map point.
export function capturarFotoActual(sensor, session) {
  return sensor.capture(session.pose, session.world.floorY, session.nav.belief, session.diagnostics.seconds);
}

// Session snapshots are keyed by their cells, and never refreshed implicitly.
export function crearRegistroFotosZonas() {
  const registros = new Map();
  const idDe = celdas => [...celdas].sort((a, b) => a - b).join(',');
  return {
    obtener(celdas, capturar) {
      const id = idDe(celdas);
      if (!registros.has(id)) registros.set(id, structuredClone({ id, celdas: [...celdas].sort((a, b) => a - b), ...capturar(), fuente: 'observador-proyectado', evidenciaRobot: false, decision: null }));
      return structuredClone(registros.get(id));
    },
    decidir(celdas, decision) { const r = registros.get(idDe(celdas)); if (r) r.decision = decision; },
    exportar() { return structuredClone([...registros.values()]); },
    limpiar() { registros.clear(); },
  };
}
