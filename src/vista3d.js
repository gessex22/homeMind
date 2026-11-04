// 3D view of the robot's own scan: renders ONLY what the belief map has
// discovered (free floor, occupied walls extruded, boards once their cell
// is known), plus the robot at its believed pose. Unknown stays empty:
// this is the room as the robot has scanned it, not the ground truth.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { gridOf, FREE, OCCUPIED, UNKNOWN, cell } from './mapping.js';

export function createVista3D(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setSize(640, 640, false);
  renderer.shadowMap.enabled = true;
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#dfe8e2');
  scene.fog = new THREE.Fog('#dfe8e2', 18, 34);
  const camera = new THREE.PerspectiveCamera(50, 1, .05, 100);
  camera.position.set(0, 11, 9);
  const controls = new OrbitControls(camera, canvas);
  controls.target.set(0, 0, 0);
  controls.maxPolarAngle = Math.PI * .49;
  scene.add(new THREE.HemisphereLight('#ffffff', '#5d7268', 1.1));
  const sun = new THREE.DirectionalLight('#fff2dd', 1.6);
  sun.position.set(6, 10, 4);
  sun.castShadow = true;
  scene.add(sun);
  const base = new THREE.Mesh(
    new THREE.PlaneGeometry(40, 40),
    new THREE.MeshStandardMaterial({ color: '#c3d2c9', roughness: 1 }),
  );
  base.rotation.x = -Math.PI / 2;
  base.position.y = -.02;
  base.receiveShadow = true;
  scene.add(base);

  const robot = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.CylinderGeometry(.16, .18, .22, 24),
    new THREE.MeshStandardMaterial({ color: '#173f3d', roughness: .5 }),
  );
  body.position.y = .11;
  body.castShadow = true;
  const nose = new THREE.Mesh(
    new THREE.ConeGeometry(.07, .18, 16),
    new THREE.MeshStandardMaterial({ color: '#bdf3d8', roughness: .4 }),
  );
  nose.rotation.x = -Math.PI / 2;
  nose.position.set(0, .16, -.22);
  const mast = new THREE.Mesh(
    new THREE.CylinderGeometry(.015, .015, .3, 8),
    new THREE.MeshStandardMaterial({ color: '#47665c', roughness: .6 }),
  );
  mast.position.y = .36;
  robot.add(body, nose, mast);
  scene.add(robot);

  let builtRevision = -1, builtScenario = null, mapGroup = null;
  function rebuild(nav, scenario) {
    if (mapGroup) { scene.remove(mapGroup); mapGroup.traverse(o => { o.geometry?.dispose?.(); }); }
    mapGroup = new THREE.Group();
    const map = nav.map, g = gridOf(map), res = g.resolution;
    const walls = [], floors = [];
    for (let i = 0; i < map.length; i++) {
      if (map[i] === UNKNOWN) continue;
      const x = g.originX + (i % g.width + .5) * res, z = g.originZ + (Math.floor(i / g.width) + .5) * res;
      (map[i] === OCCUPIED ? walls : floors).push([x, z]);
    }
    const floorMesh = new THREE.InstancedMesh(
      new THREE.BoxGeometry(res, .04, res),
      new THREE.MeshStandardMaterial({ color: '#eef3ee', roughness: .95 }),
      Math.max(1, floors.length),
    );
    const m4 = new THREE.Matrix4();
    floors.forEach(([x, z], i) => { m4.makeTranslation(x, .02, z); floorMesh.setMatrixAt(i, m4); });
    floorMesh.count = floors.length;
    floorMesh.receiveShadow = true;
    const wallMesh = new THREE.InstancedMesh(
      new THREE.BoxGeometry(res, 2.3, res),
      new THREE.MeshStandardMaterial({ color: '#47665c', roughness: .85 }),
      Math.max(1, walls.length),
    );
    walls.forEach(([x, z], i) => { m4.makeTranslation(x, 1.15, z); wallMesh.setMatrixAt(i, m4); });
    wallMesh.count = walls.length;
    wallMesh.castShadow = true;
    wallMesh.receiveShadow = true;
    mapGroup.add(floorMesh, wallMesh);
    // Boards appear only once the robot's map knows their cell: discovered
    // objects, at their believed (map-frame) positions.
    for (const b of scenario.boards ?? []) {
      const idx = cell(b.x, b.z, map);
      if (idx < 0 || map[idx] === UNKNOWN) continue;
      const panel = new THREE.Mesh(
        new THREE.BoxGeometry(b.size, b.size, .04),
        new THREE.MeshStandardMaterial({ color: b.color, roughness: .5 }),
      );
      panel.position.set(b.x, .8, b.z);
      panel.rotation.y = b.yaw ?? 0;
      panel.castShadow = true;
      mapGroup.add(panel);
    }
    // Furniture appears once its cell is known, at believed positions.
    for (const m of scenario.muebles ?? []) {
      const idx = cell(m.x, m.z, map);
      if (idx < 0 || map[idx] === UNKNOWN) continue;
      const box = new THREE.Mesh(
        new THREE.BoxGeometry(m.w, m.h ?? .8, m.d),
        new THREE.MeshStandardMaterial({ color: m.color, roughness: .7 }),
      );
      box.position.set(m.x, (m.h ?? .8) / 2, m.z);
      box.castShadow = true;
      mapGroup.add(box);
    }
    scene.add(mapGroup);
    builtRevision = map.revision;
    builtScenario = scenario;
  }

  return {
    update(nav, scenario, modo = 'orbita') {
      if (nav.map.revision !== builtRevision || scenario !== builtScenario) rebuild(nav, scenario);
      const b = nav.belief;
      robot.position.set(b.x, 0, b.z);
      robot.rotation.y = b.yaw;
      if (modo === 'robot') {
        controls.enabled = false;
        camera.position.set(b.x, .42, b.z);
        const fx = -Math.sin(b.yaw), fz = -Math.cos(b.yaw);
        camera.lookAt(b.x + fx * 3, .35, b.z + fz * 3);
      } else {
        controls.enabled = true;
        controls.update();
      }
      renderer.render(scene, camera);
    },
  };
}
