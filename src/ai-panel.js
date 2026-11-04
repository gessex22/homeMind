import { groundDetection, approachForObject } from './ai-grounding.js';

export function createAiPanel({ capture, session, stop, navigate, render, busyChanged }) {
  const $ = id => document.getElementById(id);
  let busy = false, last = null, observations = [];
  function reset() {
    last = null; observations = []; $('ai-results').replaceChildren(); $('ai-preview').hidden = true;
    $('ai-export').disabled = true; $('ai-status').textContent = 'Escribe qué quieres encontrar en la vista actual de la cámara.';
  }
  async function analyze() {
    if (busy) return;
    busy = true; busyChanged(true); stop();
    const capturedSession = session();
    try {
      observations = []; last = null; $('ai-results').replaceChildren(); $('ai-preview').hidden = true; $('ai-export').disabled = true;
      const frame = capture();
      $('ai-status').textContent = 'Consultando Muse Spark y SAM 3.1 con la captura de la cámara…';
      const response = await fetch('/api/ai/analyze', { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ image: frame.image, width: frame.width, height: frame.height, instruction: $('ai-instruction').value }) });
      const result = await response.json(); if (!response.ok) throw new Error(result.error ?? 'Error al consultar Meta.');
      if (session() !== capturedSession) return;
      observations = result.detections.map(d => ({ ...d, point: groundDetection(frame.measure(d), frame.belief, frame.calibration) }));
      last = { ...result, detections: observations, capture: { image: frame.image, width: frame.width, height: frame.height, belief: frame.belief, time: frame.time, calibration: frame.calibration }, instruction: $('ai-instruction').value };
      const canvas = $('ai-preview'), ctx = canvas.getContext('2d'), picture = new Image(); picture.src = frame.image; await picture.decode();
      canvas.width = frame.width; canvas.height = frame.height; ctx.drawImage(picture, 0, 0); canvas.hidden = false;
      $('ai-results').replaceChildren();
      for (const [index, observation] of observations.entries()) {
        const bytes = Uint8Array.from(atob(observation.mask.pixels), c => c.charCodeAt(0)), maskCanvas = document.createElement('canvas');
        maskCanvas.width = observation.mask.width; maskCanvas.height = observation.mask.height;
        const maskCtx = maskCanvas.getContext('2d'), raster = maskCtx.createImageData(maskCanvas.width, maskCanvas.height);
        bytes.forEach((bit, i) => { if (bit) { raster.data.set([35, 184, 145, 110], i * 4); } }); maskCtx.putImageData(raster, 0, 0);
        const b = observation.bounds; ctx.drawImage(maskCanvas, b.left, b.top, b.right - b.left, b.bottom - b.top);
        ctx.strokeStyle = '#f6b73b'; ctx.lineWidth = 2; ctx.strokeRect(b.left, b.top, b.right - b.left, b.bottom - b.top);
        ctx.fillStyle = '#fff'; ctx.font = 'bold 16px sans-serif'; ctx.fillText(`${index + 1}. ${observation.label}`, b.left, Math.max(18, b.top - 6));
        const card = document.createElement('div'), title = document.createElement('strong'), details = document.createElement('p'), button = document.createElement('button');
        title.textContent = `${index + 1}. ${observation.label}`;
        details.textContent = observation.point ? `Profundidad virtual ${observation.point.depth.toFixed(2)} m · posición estimada (${observation.point.x.toFixed(2)}, ${observation.point.z.toFixed(2)}) m · ${observation.point.samples} retornos. Orientación del objeto sin determinar.` : 'Objeto visible sin profundidad virtual suficiente: no se puede colocar en el mapa.';
        button.textContent = 'Acercarse a este objeto'; button.disabled = !observation.point;
        button.addEventListener('click', () => {
          try {
            const approach = approachForObject(session().nav, observation.point, session().world.radius);
            if (!approach) throw new Error('No hay una aproximación conocida donde quepa el robot. Explora más suelo antes de acercarte.');
            navigate(approach, { ...observation.point, label: observation.label });
            $('ai-status').textContent = `Navegando a una posición libre cerca de ${observation.label}. La geometría del objeto se estimó con profundidad virtual.`;
          } catch (error) { $('ai-status').textContent = error.message; }
        });
        card.append(title, details, button); $('ai-results').append(card);
      }
      $('ai-status').textContent = `${result.observation.description} · SAM: ${observations.length} coincidencias · ${(result.trace.elapsedMs / 1000).toFixed(1)} s. Modelos: ${result.trace.vision.model}${result.trace.segmentation ? ' + ' + result.trace.segmentation.model : ''}.`;
      if (result.observation.found && observations.length === 0) $('ai-status').textContent += ' SAM no confirmó el objeto; no se creó un destino.';
      $('ai-export').disabled = false; render();
    } catch (error) { $('ai-status').textContent = error.message; }
    finally { busy = false; busyChanged(false); }
  }
  $('ai-analyze').addEventListener('click', analyze);
  $('ai-export').addEventListener('click', () => {
    if (!last) return;
    const url = URL.createObjectURL(new Blob([JSON.stringify(last, null, 2)], { type: 'application/json' })), a = document.createElement('a');
    a.href = url; a.download = 'homemind-observacion-meta.json'; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  fetch('/api/ai/status').then(r => r.json()).then(status => {
    $('ai-connection').textContent = status.configured ? 'Meta API · credencial disponible desde Muse Code / entorno' : 'Configura MODEL_API_KEY o inicia sesión con API key en Muse Code.';
  }).catch(() => { $('ai-connection').textContent = 'La IA requiere el servidor local de desarrollo: npm run dev.'; });
  return { reset, get busy() { return busy; }, get observations() { return observations; } };
}
