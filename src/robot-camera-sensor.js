import * as THREE from 'three';

export function createRobotCameraSensor({ renderer, scene, room, robot, materials, pitchDeg = 12 }) {
  const width = 640, height = 360, cameraHeight = .6, offset = .11;
  const fov = THREE.MathUtils.radToDeg(2 * Math.atan(Math.tan(THREE.MathUtils.degToRad(40)) / (width / height)));
  const target = new THREE.WebGLRenderTarget(width, height);
  target.texture.colorSpace = THREE.SRGBColorSpace;
  // Downward tilt calibration: positive degrees look below the horizon,
  // so floor-level zones and obstacle bases stay in frame.
  let pitch = THREE.MathUtils.clamp(pitchDeg, -10, 35);
  return {
    get pitchDeg() { return pitch; },
    setPitch(deg) { if (Number.isFinite(deg)) pitch = THREE.MathUtils.clamp(deg, -10, 35); return pitch; },
    capture(pose, floorY, belief, time) {
    const camera = new THREE.PerspectiveCamera(fov, width / height, .01, 100), dx = -Math.sin(pose.yaw), dz = -Math.cos(pose.yaw);
    camera.position.set(pose.x + dx * offset, floorY + cameraHeight, pose.z + dz * offset);
    camera.lookAt(camera.position.x + dx, camera.position.y - Math.tan(THREE.MathUtils.degToRad(pitch)), camera.position.z + dz); camera.updateMatrixWorld(true);
    const previousTarget = renderer.getRenderTarget(), visible = robot.visible, clips = materials.map(m => m.clippingPlanes);
    const pixels = new Uint8Array(width * height * 4);
    try {
      robot.visible = false; materials.forEach(m => { m.clippingPlanes = []; });
      renderer.setRenderTarget(target); renderer.render(scene, camera); renderer.readRenderTargetPixels(target, 0, 0, width, height, pixels);
    } finally { renderer.setRenderTarget(previousTarget); robot.visible = visible; materials.forEach((m, i) => { m.clippingPlanes = clips[i]; }); }
    const canvas = document.createElement('canvas'); canvas.width = width; canvas.height = height;
    const ctx = canvas.getContext('2d'), image = ctx.createImageData(width, height);
    for (let y = 0; y < height; y++) image.data.set(pixels.subarray((height - 1 - y) * width * 4, (height - y) * width * 4), y * width * 4);
    ctx.putImageData(image, 0, 0);
    return { image: canvas.toDataURL('image/jpeg', .9), width, height, belief: { ...belief }, time,
      calibration: { horizontalFovDeg: 80, height: cameraHeight, forwardOffset: offset, range: 3.8, pitchDeg: pitch },
      measure(detection) {
        const bytes = Uint8Array.from(atob(detection.mask.pixels), c => c.charCodeAt(0)), mask = detection.mask, box = detection.bounds;
        if (bytes.length !== mask.width * mask.height) return [];
        const stride = Math.max(1, Math.ceil(Math.sqrt(bytes.length / 100))), points = [], ray = new THREE.Raycaster();
        ray.near = .1; ray.far = 3.8;
        room.updateMatrixWorld(true);
        for (let y = Math.floor(stride / 2); y < mask.height; y += stride) for (let x = Math.floor(stride / 2); x < mask.width; x += stride) {
          if (!bytes[y * mask.width + x]) continue;
          const u = box.left + (x + .5) / mask.width * (box.right - box.left), v = box.top + (y + .5) / mask.height * (box.bottom - box.top);
          ray.setFromCamera(new THREE.Vector2(u / width * 2 - 1, 1 - v / height * 2), camera);
          const hit = ray.intersectObject(room, true)[0];
          if (hit) { const local = camera.worldToLocal(hit.point.clone()); points.push({ x: local.x, y: local.y, z: local.z }); }
        }
        return points;
      },
    };
  } };
}
