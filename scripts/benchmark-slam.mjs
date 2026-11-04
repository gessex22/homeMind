import { evaluateScenario } from './evaluate-room.mjs';
import { SCENARIOS } from '../src/world.js';
import { writeFileSync } from 'node:fs';

const comparisons = [];
for (const id of ['partition', 'casa', 'large']) for (const seed of [7, 7926]) {
  const results = [false, true].map(loopClosure => {
    const { report: r } = evaluateScenario(SCENARIOS[id], 24000, { odometry: {}, seed, loopClosure });
    return { loopClosure, state: r.estado, errorM: r.error_odom_m, yawDegrees: r.error_yaw_grados,
      floorPercent: r.suelo_observado_porcentaje, wallPercent: r.pared_observada_porcentaje,
      collisions: r.colisiones, unsafeSteps: r.pasos_con_huella_fuera_del_espacio_libre_conocido,
      seconds: r.segundos_simulados, slam: r.slam, closureObserver: r.observador_cierres };
  });
  const comparison = { scenario: id, seed, baseline: results[0], corrected: results[1] };
  comparisons.push(comparison); console.log(JSON.stringify(comparison));
}
const output = process.argv[2];
if (output) writeFileSync(output, JSON.stringify({ description: 'Mismos sensores, sesgo y semillas. Las rutas pueden divergir tras una corrección; comparación de misiones, no de trayectoria idéntica.', comparisons }, null, 2));
if (comparisons.some(c => c.corrected.collisions || c.corrected.unsafeSteps)) process.exitCode = 1;
