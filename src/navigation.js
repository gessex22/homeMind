// Navigation engine: drop the robot in an unknown area and it explores by
// frontiers, rebuilding an occupancy map. No preloaded route or map: the
// host only feeds sensor rays and measured wheel motion (encoders/gyro as
// avance+giro); the engine owns belief, map, corrections and commands.
import { createMap, seedFootprint, counts, gridOf, FREE, OCCUPIED } from './mapping.js';
import { localizeGlobal, scorePose, alignScan } from './scanmatch.js';
import { compose, between } from './pose-graph.js';
import { frontiers, plan } from './planner.js';
import { createExplorer, startExplorer, pauseExplorer, exploreStep, setLeash, setDestination } from './explorer.js';
import { createOdometry, odomPose, SENSORES_EXACTOS, SENSORES_RUIDOSOS } from './odometry.js';
import { integrateFusedMotion, gateMotionCommand, motionFusionSummary } from './motion-fusion.js';
import { createMissionMapping, updateMissionMapping, missionSummary } from './mission-mapping.js';
import { SENSOR } from './sensor.js';
import { observeSweep } from './sweep-coverage.js';

const zero = () => ({ forward: false, back: false, left: false, right: false });
export const NAV_TERMINAL = Object.freeze(['completado', 'bloqueado', 'limitado', 'refugio', 'localizacion_perdida', 'destino_alcanzado']);

export function createNavigator(mapConfig, {
  noisy = true, loopClosure = false, motionFusion = true, scanmatch = {},
  sensor = SENSOR, initialPose = { x: 0, z: 0, yaw: 0 }, odometry = {}, seed,
  priorMap = null, priorFuerza = .5, autoRelocalizar = true, priorMarcas = null, footprint = .25, adaptiveSweeps = false,
} = {}) {
  // Visual landmarks from the previous mission (label + position in the map
  // frame, e.g. fused board inventory). Geometry alone cannot separate
  // twin rooms; a seen object can. Hosts pass them directly or inside the
  // saved map file (`marcas`).
  const marcasPrevias = (priorMarcas ?? priorMap?.marcas ?? [])
    .filter(m => typeof m?.label === 'string' && Number.isFinite(m.x) && Number.isFinite(m.z))
    .map(m => ({ label: m.label, x: m.x, z: m.z }));
  let map, odom, mission, explorer, time, lastContact, relocalizacion = null, reloLineas = null, reloScans = null, reloOdom = null, reloGiroTotal = 0, reloAvanceTotal = 0, faseRelo = null, watchRelo = null, reintentos = 0, priorFrozen = null, marcasSesion = [], reloPausada = false;
  function loadPrior(saved, fuerza) {
    const g = gridOf(map);
    if (!saved || !Array.isArray(saved.celdas) || saved.celdas.length !== map.length
      || saved.grid?.width !== g.width || saved.grid?.height !== g.height
      || saved.grid?.resolution !== g.resolution || saved.grid?.originX !== g.originX || saved.grid?.originZ !== g.originZ) {
      throw new RangeError('Mapa previo incompatible con este dominio');
    }
    if (!Number.isFinite(fuerza) || fuerza < 0) throw new RangeError('Fuerza del mapa previo inválida');
    // Decayed confidence: prior cells guide matching from the start, but new
    // evidence can still move a wall the old mission painted wrong.
    for (let i = 0; i < map.length; i++) {
      if (saved.celdas[i] === OCCUPIED) { map[i] = OCCUPIED; map.logodds[i] = fuerza; }
      else if (saved.celdas[i] === FREE) { map[i] = FREE; map.logodds[i] = -fuerza; }
    }
    map.revision++;
  }
  // Session landmark memory: fuse camera detections (label + bearing +
  // distance from the host adapter) into map-frame positions using the
  // belief pose. Same label within 0.75 m clusters into one entry with a
  // running mean; single sightings stay unconfirmed and are not saved.
  function registrarMarcas(detecciones) {
    const b = odomPose(odom);
    for (const d of detecciones ?? []) {
      if (typeof d?.label !== 'string' || !Number.isFinite(d.bearing) || !Number.isFinite(d.dist) || d.dist <= 0) continue;
      const wx = b.x - Math.sin(b.yaw + d.bearing) * d.dist, wz = b.z - Math.cos(b.yaw + d.bearing) * d.dist;
      const e = marcasSesion.find(m => m.label === d.label && Math.hypot(m.x - wx, m.z - wz) < .75);
      if (e) { e.vistas++; e.x += (wx - e.x) / e.vistas; e.z += (wz - e.z) / e.vistas; }
      else marcasSesion.push({ label: d.label, x: wx, z: wz, vistas: 1 });
    }
  }
  const marcasConfirmadas = () => marcasSesion.filter(m => m.vistas >= 2);
  function reset(nextSeed = seed) {
    map = createMap(mapConfig);
    priorFrozen = null;
    if (priorMap) {
      loadPrior(priorMap, priorFuerza);
      // Frozen copy of the prior as loaded: post-commit checks must score
      // against this, never against the live map, which absorbs scans in
      // whatever frame was committed and would certify its own mistake.
      priorFrozen = createMap(mapConfig);
      priorFrozen.set(map);
      priorFrozen.logodds.set(map.logodds);
    }
    relocalizacion = null; reloLineas = null; reloScans = null; reloOdom = null; reloGiroTotal = 0; reloAvanceTotal = 0; faseRelo = null; watchRelo = null; reintentos = 0; marcasSesion = []; reloPausada = false;
    odom = createOdometry(initialPose, { ...(noisy ? SENSORES_RUIDOSOS : SENSORES_EXACTOS), ...odometry, ...(Number.isInteger(nextSeed) ? { seed: nextSeed } : {}) });
    mission = createMissionMapping(map, { noisy, loopClosure, motionFusion, scanmatch, initialPose });
    explorer = createExplorer(footprint);
    seedFootprint(map, odomPose(odom));
    time = 0; lastContact = false;
  }
  reset();

  const api = {
    start() {
      if (faseRelo) { reloPausada = false; return; }
      // With a prior map the engine first locates itself: it spins a full
      // turn, translates, and spins again (state 'relocalizando'), driven
      // entirely through step(). Only then does exploration begin.
      if (priorMap && autoRelocalizar && explorer.state === 'inactivo') {
        faseRelo = { etapa: 'giro1', pasos: 0, giro: 0, avance: 0 };
        relocalizacion = { found: false, score: 0, ambiguo: false, automatica: true };
        return;
      }
      startExplorer(explorer);
    },
    pause() { if (faseRelo) reloPausada = true; else pauseExplorer(explorer); },
    reset,
    setLeash(cfg) { setLeash(explorer, cfg); },
    // Navigate toward a point in the belief frame. If no route is known yet
    // the robot keeps exploring and retries as the map grows.
    setDestination(destino) {
      setDestination(explorer, destino);
      if (destino && explorer.state === 'completado') explorer.state = 'planificando';
    },
    get destination() { return explorer.destino ? { ...explorer.destino } : null; },
    // Verified closure: frontier cells a human (or a photo check) confirmed
    // have no passage beyond them. They stop counting as pending; if that
    // was all that remained, a limitado mission becomes completado. Closed
    // cells are listed in summary() so the closure is auditable.
    closeFrontiers(celdas) {
      for (const c of celdas ?? []) if (Number.isInteger(c) && c >= 0 && c < map.length) explorer.cerradas.add(c);
      if (['limitado', 'completado', 'planificando'].includes(explorer.state)) {
        const r = plan(map, odomPose(odom), explorer.scanned, { radius: footprint, cerradas: explorer.cerradas });
        explorer.fronts = r.fronts;
        explorer.state = r.reason === 'ruta' ? 'planificando' : r.reason;
        if (r.reason === 'ruta') { explorer.route = r.route; explorer.safe = r.safe; explorer.safeRevision = map.revision; }
      }
      return explorer.state;
    },
    saveMap() {
      const g = gridOf(map);
      // Landmarks ride along with the grid: geometry alone cannot
      // separate look-alike rooms, a remembered object can. Only
      // landmarks seen at least twice are saved.
      const marcas = marcasConfirmadas().map(m => ({ label: m.label, x: Number(m.x.toFixed(3)), z: Number(m.z.toFixed(3)), vistas: m.vistas }));
      return { formato: 'homemind-mapa', version: 1, grid: { originX: g.originX, originZ: g.originZ, resolution: g.resolution, width: g.width, height: g.height }, celdas: Array.from(map), marcas };
    },
    // Global relocalization against the (prior) map before starting. One view
    // can be ambiguous (look-alike places); call again after rotating in
    // place, passing the gyro turn since the previous call, and candidates
    // are intersected until one place explains every view. On success the
    // belief teleports to the found pose; truth is never consulted.
    // Motion between calls is the measured increment since the previous
    // scan: rotate by `giro`, then advance `avance` along the new heading.
    relocalize(rays, giro = 0, avance = 0, vistasMarcas = [], brujula = null) {
      if (explorer.state !== 'inactivo') throw new Error('Relocalizar solo antes de start');
      if (!Number.isFinite(giro) || !Number.isFinite(avance)) throw new RangeError('Movimiento de relocalización inválido');
      if (brujula != null && !Number.isFinite(brujula)) throw new RangeError('Lectura de brújula inválida');
      const wrapA = a => Math.atan2(Math.sin(a), Math.cos(a));
      if (!reloOdom) reloOdom = { x: 0, z: 0, yaw: 0 };
      reloGiroTotal += Math.abs(giro); reloAvanceTotal += Math.abs(avance);
      reloOdom.yaw = wrapA(reloOdom.yaw + giro);
      reloOdom.x -= Math.sin(reloOdom.yaw) * avance;
      reloOdom.z -= Math.cos(reloOdom.yaw) * avance;
      const scan = rays ?? [];
      const marcas = (vistasMarcas ?? []).filter(m => typeof m?.label === 'string' && Number.isFinite(m.bearing) && Number.isFinite(m.dist) && m.dist > 0);
      const result = localizeGlobal(map, scan, sensor, brujula != null ? { yawCentro: brujula, yawTol: .45 } : {});
      let found = null;
      if (!reloLineas) {
        if (result.found) found = result.pose;
        reloLineas = (result.candidatos ?? []).map(c => ({ ...c, odomAt: { ...reloOdom }, hits: 1 }));
        reloScans = [{ rays: scan, odom: { ...reloOdom }, marcas, brujula }];
      } else {
        reloScans.push({ rays: scan, odom: { ...reloOdom }, tras: reloAvanceTotal >= .3, marcas, brujula });
        if (reloScans.length > 120) reloScans.shift();
        // Track hypotheses by re-scoring their predicted pose directly: the
        // per-view candidate list is flaky, but a true pose scores ~1 under
        // the strict scorer and a false one collapses within two views.
        const extremos = scan.reduce((n, r) => n + (r.kind === 'wall' && Number.isFinite(r.distance) && r.distance > 0 ? 1 : 0), 0);
        for (const linea of reloLineas) {
          if (extremos < 12) break; // an empty view neither confirms nor refutes
          const pred = compose(linea, between(linea.odomAt, reloOdom));
          const s = scorePose(map, scan, pred, sensor);
          if (s >= .45) {
            // Snap to the local optimum before propagating: a quantized
            // candidate left to dead reckoning drifts off the walls and
            // its strict score dips until the true hypothesis starves.
            const fix = alignScan(map, scan, pred, sensor);
            const p = fix.corregido ? fix.pose : pred;
            linea.x = p.x; linea.z = p.z; linea.yaw = p.yaw; linea.odomAt = { ...reloOdom }; linea.hits++; linea.score = s; linea.misses = 0;
          } else linea.misses = (linea.misses ?? 0) + 1;
        }
        reloLineas = reloLineas.filter(l => (l.misses ?? 0) < 3);
        for (const c of result.candidatos ?? []) if (!reloLineas.some(l => Math.hypot(l.x - c.x, l.z - c.z) < .35)) reloLineas.push({ ...c, odomAt: { ...reloOdom }, hits: 1, misses: 0 });
        if (reloLineas.length > 24) reloLineas = reloLineas.sort((a, b) => b.hits - a.hits).slice(0, 24);
        // A hypothesis earns trust only by explaining every stored view,
        // including after translation: mirroring places predict a different
        // position once the robot moves, and their agreement collapses.
        // Short arcs alias even in good faith, so intersection commits wait
        // for a wide spin (>= 240°) plus real translation (>= 0.35 m).
        const acuerdoDe = l => {
          const puntajes = reloScans.map(s => ({ v: scorePose(map, s.rays, compose(l, between(reloOdom, s.odom)), sensor), tras: s.tras }));
          const media = xs => xs.length ? xs.reduce((a, b) => a + b.v, 0) / xs.length : 0;
          // Post-translation views are few but decisive: a mirrored place
          // collapses there even when its spin average still looks fine.
          const geometrico = Math.min(media(puntajes), media(puntajes.filter(p => p.tras)));
          // Visual agreement: every object the camera saw must land on a
          // prior landmark of the same label under this hypothesis. A twin
          // room scores ~1 geometrically but ~0 here. No objects seen means
          // no visual evidence either way, so geometry decides alone.
          if (!marcasPrevias.length) return geometrico;
          let vistos = 0, ok = 0;
          for (const s of reloScans) {
            if (!s.marcas?.length) continue;
            const usados = new Set();
            const pv = compose(l, between(reloOdom, s.odom));
            for (const m of s.marcas) {
              vistos++;
              const wx = pv.x - Math.sin(pv.yaw + m.bearing) * m.dist, wz = pv.z - Math.cos(pv.yaw + m.bearing) * m.dist;
              let best = -1, bd = .6;
              marcasPrevias.forEach((p, i) => { if (p.label !== m.label || usados.has(i)) return; const d = Math.hypot(p.x - wx, p.z - wz); if (d < bd) { bd = d; best = i; } });
              if (best >= 0) { usados.add(best); ok++; }
            }
          }
          return vistos ? Math.min(geometrico, ok / vistos) : geometrico;
        };
        // Compass gate: a hypothesis whose predicted heading at the
        // latest scan disagrees with the measured absolute heading is
        // not mature, however well its walls fit — mirrored places die
        // here without spending agreement budget on them.
        const maduras = reloGiroTotal < 4.2 || reloAvanceTotal < .35 ? [] : reloLineas.filter(l => l.hits >= 3)
          .filter(l => brujula == null || Math.abs(wrapA(compose(l, between(l.odomAt, reloOdom)).yaw - brujula)) <= .5)
          .map(l => ({ pose: l, acuerdo: acuerdoDe(l) }))
          .sort((a, b) => b.acuerdo - a.acuerdo);
        if (maduras.length && maduras[0].acuerdo >= .8 && (maduras.length === 1 || maduras[0].acuerdo - maduras[1].acuerdo > .02 || maduras[0].acuerdo >= .97)) found = maduras[0].pose;
      }
      relocalizacion = { found: Boolean(found), score: Number(result.score.toFixed(3)), ambiguo: !found && (result.candidatos?.length ?? 0) > 1 };
      if (found) {
        Object.assign(odom, found);
        reloLineas = null; reloScans = null; reloOdom = null; reloGiroTotal = 0; reloAvanceTotal = 0;
        // Post-commit watch: rotating in place aliases like the commit did,
        // so agreement is only judged after the robot truly translates;
        // there a look-alike's scans stop fitting the map.
        watchRelo = { scans: 0, suma: 0, tardias: 0, sumaTardia: 0, vistas: 0, maxDisp: 0, origen: { x: found.x, z: found.z }, listo: false };
        if (mission.slam) mission = createMissionMapping(map, { noisy, loopClosure, motionFusion, scanmatch, initialPose: found });
        return { found: true, pose: { ...found }, score: result.score };
      }
      return { found: false, pose: null, score: result.score, ambiguo: relocalizacion.ambiguo, candidatos: result.candidatos?.length ?? 0 };
    },
    // One control tick. `motion` is the measurement produced by the step the
    // host just executed ({ avance, giro, contact }); `rays` is a fresh scan
    // in the robot frame, or null between scans. Returns the next command.
    step(dt, { rays = null, motion = null, detecciones = null } = {}) {
      if (!Number.isFinite(dt) || dt <= 0) throw new RangeError('dt debe ser positivo');
      if (faseRelo) {
        // Automatic startup relocalization: the engine drives the maneuver
        // itself; scans arriving during it locate instead of mapping.
        if (motion) {
          const measured = { ...motion, drive: motion.drive ?? Math.sign(motion.avance ?? 0) };
          const accepted = integrateFusedMotion(mission.motion, odom, motion.contact && !mission.motion.enabled ? { ...measured, avance: 0 } : measured, dt);
          if (motion.contact && !mission.motion.enabled) mission.motion.lastRaw = { ...measured };
          faseRelo.giro += motion.giro ?? 0; faseRelo.avance += motion.contact ? 0 : (accepted.avanceAceptado ?? 0);
          lastContact = Boolean(motion.contact); if (Number.isFinite(motion.brujula)) faseRelo.brujula = motion.brujula;
        }
        if (reloPausada) return zero();
        time += dt;
        faseRelo.pasos++;
        if (rays) {
          const r = api.relocalize(rays, faseRelo.giro, faseRelo.avance, detecciones ?? [], faseRelo.brujula ?? null);
          faseRelo.giro = 0; faseRelo.avance = 0;
          if (r.found) { faseRelo = null; startExplorer(explorer); return zero(); }
        }
        // Active relocalization: spin–advance–spin first; if that leaves
        // no confident hypothesis, do not give up in place — turn ~90°,
        // translate again and spin once more. The second leg looks at
        // the room from a different axis, which is what breaks ties
        // between look-alike places. Only then explore unlocated.
        if (faseRelo.etapa === 'giro1' && reloGiroTotal >= 6.3) { faseRelo.etapa = 'avance'; faseRelo.pasos = 0; }
        else if (faseRelo.etapa === 'avance' && (reloAvanceTotal >= .7 || faseRelo.pasos > 90)) { faseRelo.etapa = 'giro2'; faseRelo.pasos = 0; }
        else if (faseRelo.etapa === 'giro2' && reloGiroTotal >= 12.6) { faseRelo.etapa = 'giroActivo'; faseRelo.pasos = 0; }
        else if (faseRelo.etapa === 'giroActivo' && reloGiroTotal >= 14.17) { faseRelo.etapa = 'avanceActivo'; faseRelo.pasos = 0; }
        else if (faseRelo.etapa === 'avanceActivo' && (reloAvanceTotal >= 1.4 || faseRelo.pasos > 90)) { faseRelo.etapa = 'giro3'; faseRelo.pasos = 0; }
        else if (faseRelo.etapa === 'giro3' && reloGiroTotal >= 20.47) { faseRelo = null; startExplorer(explorer); return zero(); }
        const avanzando = faseRelo.etapa === 'avance' || faseRelo.etapa === 'avanceActivo';
        const delante = rays?.slice(Math.floor(rays.length * .4), Math.ceil(rays.length * .6)).some(r => r.kind === 'wall' && Number.isFinite(r.distance) && r.distance < footprint + .15);
        if (avanzando && (lastContact || delante || motion?.groundStop)) {
          faseRelo.etapa = faseRelo.etapa === 'avance' ? 'giro2' : 'giro3';
          faseRelo.pasos = 0;
          return gateMotionCommand(mission.motion, zero());
        }
        return gateMotionCommand(mission.motion, avanzando ? { ...zero(), forward: true } : { ...zero(), left: true });
      }
      if (explorer.state === 'inactivo') return zero();
      time += dt;
      let freshScan = Boolean(rays?.some(ray => ray.kind !== 'invalid' && Number.isFinite(ray.distance) && ray.distance > 0));
      if (motion) {
        integrateFusedMotion(mission.motion, odom, motion, dt);
        lastContact = Boolean(motion.contact);
      }
      if (rays) {
        const preWatch = watchRelo ? odomPose(odom) : null;
        const mapping = updateMissionMapping(mission, map, rays, odomPose(odom), explorer, time, sensor, lastContact);
        freshScan &&= !mapping.deferred;
        Object.assign(odom, mapping.pose);
        if (adaptiveSweeps) explorer.sweepCoverage = observeSweep(explorer.sweepCoverage, odomPose(odom), sensor.fov);
        if (watchRelo && preWatch) {
          watchRelo.vistas++;
          const disp = Math.hypot(preWatch.x - watchRelo.origen.x, preWatch.z - watchRelo.origen.z);
          watchRelo.maxDisp = Math.max(watchRelo.maxDisp, disp);
          if (!watchRelo.listo && disp >= .3) watchRelo.listo = true;
          // Congruent rooms fit a false pose perfectly for the first metres:
          // only long-horizon agreement, after truly leaving the start area,
          // separates them, so the verdict waits for late scans too.
          if (watchRelo.listo) {
            const s = scorePose(priorFrozen ?? map, rays, preWatch, sensor);
            watchRelo.scans++; watchRelo.suma += s;
            if (watchRelo.maxDisp >= 2) { watchRelo.tardias++; watchRelo.sumaTardia += s; }
          }
          const tempranoMalo = watchRelo.scans >= 8 && watchRelo.maxDisp < 2 && watchRelo.suma / watchRelo.scans < .45;
          const tardioListo = watchRelo.tardias >= 10;
          if (tempranoMalo || tardioListo || watchRelo.vistas > 900) {
            const malo = tempranoMalo || (tardioListo && watchRelo.sumaTardia / watchRelo.tardias < .75);
            watchRelo = null;
            if (malo) {
              relocalizacion = { found: false, revocada: true, score: relocalizacion?.score ?? 0 };
              // Undo scans integrated in the revoked frame before retrying.
              if (priorFrozen) { map.set(priorFrozen); map.logodds.set(priorFrozen.logodds); map.revision++; }
              if (reintentos < 2 && priorMap) {
                reintentos++;
                explorer.state = 'inactivo';
                faseRelo = { etapa: 'giro1', pasos: 0, giro: 0, avance: 0 };
              }
            }
          }
        }
      }
      if (detecciones?.length) registrarMarcas(detecciones);
      return gateMotionCommand(mission.motion, exploreStep(explorer, map, odomPose(odom), dt, lastContact, Boolean(motion?.groundStop), freshScan));
    },
    get state() { return faseRelo ? (reloPausada ? 'pausado' : 'relocalizando') : explorer.state; },
    get terminal() { return NAV_TERMINAL.includes(explorer.state); },
    get map() { return map; },
    get belief() { return odomPose(odom); },
    get explorer() { return explorer; },
    fronts() { return frontiers(map); },
    summary() {
      const cells = counts(map);
      return {
        estado: faseRelo ? (reloPausada ? 'pausado' : 'relocalizando') : explorer.state, terminal: NAV_TERMINAL.includes(explorer.state),
        creencia: odomPose(odom), tiempo_s: Number(time.toFixed(3)),
        celdas: cells, conocidas_pct: Number(((cells.libre + cells.ocupado) / map.length * 100).toFixed(2)),
        fronteras: frontiers(map).length, zonas_cerradas: explorer.cerradas.size, barridos: explorer.scanned.length, barrido_adaptativo: adaptiveSweeps,
        correcciones_scan: mission.corrections, deriva_corregida_m: Number(mission.correctionDistance.toFixed(3)),
        retorno: explorer.retorno ?? 'no', base: explorer.base ? { ...explorer.base } : null,
        destino: explorer.destino ? { ...explorer.destino } : null,
        recuperacion: explorer.state === 'retroceso' ? {
          causa: explorer.recoveryCause,
          fase: explorer.escape?.phase ?? (explorer.retrocede > 30 ? 'retroceso' : 'giro'),
          rumbo_objetivo_rad: explorer.escape?.yaw ?? null,
          salida: explorer.escape?.point ? { ...explorer.escape.point } : null,
        } : null,
        mapa_previo: Boolean(priorMap), marcas_previas: marcasPrevias.length, marcas_sesion: marcasConfirmadas().length, relocalizacion,
        slam: missionSummary(mission), fusion_movimiento: motionFusionSummary(mission.motion),
      };
    },
  };
  return api;
}
