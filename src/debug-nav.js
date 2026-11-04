import { center, cell, FREE, UNKNOWN } from './mapping.js';
import { footprintClear, frontiers, plan, routeTo } from './planner.js';

// Pure diagnosis over the believed map: why is the robot (not) moving?
// It only reads the map and the explorer state — the same inputs the
// engine itself uses — so the answer explains the engine's own decision,
// never the simulator's ground truth.
export function diagnosticarNavegacion({ map, creencia, explorer, ultimoComando = null }) {
  const footprint = explorer.footprint, destino = explorer.destino ?? null;
  const huellaActual = footprintClear(map, creencia, footprint);
  const lineas = [], datos = { estado: explorer.state, huellaActual };
  lineas.push(`ESTADO   ${explorer.state} · huella r=${footprint.toFixed(2)} m · posición creída (${creencia.x.toFixed(2)}, ${creencia.z.toFixed(2)}) yaw ${(creencia.yaw * 180 / Math.PI).toFixed(0)}°`);
  lineas.push(`HUELLA   ${huellaActual ? 'libre en la posición actual' : 'SOLAPADA con ocupado/desconocido en la posición actual'}`);
  if (explorer.route.length) {
    const siguiente = center(explorer.route[0], map);
    const deseado = Math.atan2(-(siguiente.x - creencia.x), -(siguiente.z - creencia.z));
    const delta = Math.atan2(Math.sin(deseado - creencia.yaw), Math.cos(deseado - creencia.yaw));
    datos.siguiente = siguiente;
    lineas.push(`RUTA     ${explorer.route.length} celdas · siguiente (${siguiente.x.toFixed(2)}, ${siguiente.z.toFixed(2)}) a ${Math.hypot(siguiente.x - creencia.x, siguiente.z - creencia.z).toFixed(2)} m · desvío ${(delta * 180 / Math.PI).toFixed(0)}°`);
  } else lineas.push('RUTA     sin ruta cargada');
  lineas.push(`EVITAR   ${explorer.avoid.size} celdas marcadas · barridos ${explorer.scanned.length} · fronteras ${frontiers(map).length}`);
  if (explorer.state === 'retroceso' || explorer.recoveryCause) {
    lineas.push(`RECUPERA causa ${explorer.recoveryCause ?? '—'} · intentos ${explorer.recoveryAttempts} · fase ${explorer.escape?.phase ?? 'retroceso/giro'}`);
  }
  if (ultimoComando) {
    const activo = Object.entries(ultimoComando).filter(([, v]) => v).map(([k, v]) => `${k}=${v === true ? 1 : Number(v).toFixed(2)}`);
    lineas.push(`COMANDO  ${activo.length ? activo.join(' · ') : 'todo en cero'}`);
    datos.comandoEnCero = activo.length === 0;
  }

  let motivo;
  if (explorer.state === 'retroceso') {
    motivo = `está recuperándose (${explorer.recoveryCause ?? 'bloqueo'}): retrocede/gira antes de replanificar`;
  } else if (!huellaActual) {
    motivo = 'la huella actual se solapa con una celda ocupada o desconocida: solo saldrá con una retirada segura comprobada';
  } else if (destino) {
    const estricta = routeTo(map, creencia, destino, { radius: footprint, avoid: explorer.avoid });
    const parcial = estricta ? null : routeTo(map, creencia, destino, { radius: footprint, avoid: explorer.avoid, parcial: true });
    datos.rutaDestino = estricta?.length ?? null; datos.rutaParcial = parcial?.length ?? null;
    lineas.push(`DESTINO  (${destino.x.toFixed(2)}, ${destino.z.toFixed(2)}) a ${Math.hypot(destino.x - creencia.x, destino.z - creencia.z).toFixed(2)} m · ruta completa ${estricta ? `${estricta.length} celdas` : 'no'} · acercamiento parcial ${parcial ? `${parcial.length} celdas` : 'no'}`);
    if (estricta) motivo = 'hay ruta completa al destino con la huella entera';
    else {
      const celdaDestino = cell(destino.x, destino.z, map);
      if (celdaDestino < 0 || map[celdaDestino] === UNKNOWN) motivo = 'el destino aún no está descubierto en el mapa creído: hay que explorar hasta observarlo';
      else if (parcial) motivo = 'sin ruta con la huella completa al destino, pero sí hay acercamiento parcial: avanzará la parte libre y replanteará';
      else motivo = 'destino descubierto pero sin acercamiento posible: ya está en la celda segura más cercana, o el progreso sería menor a una celda';
    }
    if (['limitado', 'bloqueado'].includes(explorer.state)) {
      motivo = `quedó ${explorer.state} tras agotar la recuperación (${explorer.recoveryAttempts} intentos, causa ${explorer.recoveryCause ?? '—'}), pero ahora el planificador dice: ${motivo} · pulsa «Replanificar ruta» y luego Explorar para retomarlo`;
    }
  } else {
    const diagnostico = plan(map, creencia, explorer.scanned, { radius: footprint, cerradas: explorer.cerradas, avoid: explorer.avoid });
    datos.plan = diagnostico.reason; datos.rutaPlan = diagnostico.route.length;
    lineas.push(`PLAN     ${diagnostico.reason}${diagnostico.route.length ? ` · ${diagnostico.route.length} celdas` : ''} · fronteras conectadas ${diagnostico.fronts.length}`);
    if (diagnostico.reason === 'ruta') motivo = 'hay ruta de exploración con la huella completa';
    else if (diagnostico.reason === 'limitado') motivo = 'exploración limitada: quedan fronteras, pero ninguna posición de observación segura alcanzable';
    else motivo = 'bloqueado: la posición actual no tiene celdas seguras alcanzables alrededor';
    if (['limitado', 'bloqueado'].includes(explorer.state)) {
      motivo = `quedó ${explorer.state} tras agotar la recuperación (${explorer.recoveryAttempts} intentos, causa ${explorer.recoveryCause ?? '—'}), pero ahora el planificador dice: ${motivo} · pulsa «Replanificar ruta» y luego Explorar para retomarlo`;
    }
  }
  if (datos.comandoEnCero && ['navegando', 'destino'].includes(explorer.state)) {
    motivo += ' · el último comando salió en cero (paso no seguro, esperando escaneo o alineando)';
  }
  return { motivo, lineas, datos };
}
