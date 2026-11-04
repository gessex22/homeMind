import { SCENARIOS } from '../src/world.js';
import { runTrial, trialOptions, TRIAL_COUNT } from '../src/trials.js';
import { compareMaps } from '../src/map-comparison.js';
const scene = SCENARIOS[process.argv[2] ?? 'partition'];
if (!scene) throw new RangeError('Escena desconocida');
const trials = [];
for (let i = 0; i < TRIAL_COUNT; i++) {
  const trial = runTrial(scene, trialOptions(i)); trials.push(trial);
  console.log(JSON.stringify({ pasada: trial.id, seed: trial.seed, estado: trial.state, segundos: trial.seconds, ...trial.summary }));
}
console.log(JSON.stringify({ escena: scene.id, ...compareMaps(trials).stats }));
