import { SCENARIOS } from './world.js';
import { runTrial, trialOptions, TRIAL_COUNT } from './trials.js';
import { compareMaps } from './map-comparison.js';

self.onmessage = event => {
  if (event.data.type !== 'start') return;
  try {
    const { scenarioId, baseSeed = 7, leash = null, loopClosure = true } = event.data, scene = SCENARIOS[scenarioId];
    if (!scene) throw new RangeError('Escena desconocida');
    const trials = [];
    self.postMessage({ type: 'progress', completed: 0, total: TRIAL_COUNT });
    for (let i = 0; i < TRIAL_COUNT; i++) {
      const trial = runTrial(scene, { ...trialOptions(i, baseSeed), leash, loopClosure });
      trials.push(trial);
      self.postMessage({ type: 'trial', trial, completed: i + 1, total: TRIAL_COUNT });
    }
    self.postMessage({ type: 'complete', trials, comparison: compareMaps(trials), total: TRIAL_COUNT });
  } catch (error) { self.postMessage({ type: 'error', message: error.message }); }
};
