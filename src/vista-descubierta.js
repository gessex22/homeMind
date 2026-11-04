// 3D reconstruction of what the ROBOT has discovered: only its believed
// map (free floor, occupied walls extruded) and its believed pose. The
// captured mesh is never drawn here — this is the robot's own rebuild.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { gridOf, FREE, OCCUPIED, UNKNOWN } from './mapping.js';

export function createVistaDescubierta(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setSize(640, 480, false);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#e4ebe5');
  const camera = new THREE.PerspectiveCamera(50, 4 / 3, .05, 200);
  const controls = new OrbitControls(camera, canvas);
  controls.maxPolarAngle = Math.PI * .49;
  scene.add(new THREE.HemisphereLight('#ffffff', '#5d7268', 1.15));
  const sun = new THREE.DirectionalLight('#fff2dd', 1.5);
  sun.position.set(6, 12, 4);
  scene.add(sun);
  const robot = new THREE.Group();
  const body = new THREE.Mesh(new THREE.CylinderGeometry(.10, .10, .6, 24), new THREE.MeshBasicMaterial({ color: '#ec8d42' }));
  body.position.y = .3;
  const nose = new THREE.Mesh(new THREE.BoxGeometry(.09, .03, .22), new THREE.MeshBasicMaterial({ color: '#173f3d' }));
  nose.position.set(0, .62, -.12);
  robot.add(body, nose);
  scene.add(robot);
  let builtRevision = -1, builtMap = null, group = null, framed = false;
  function rebuild(nav) {
    if (group) { scene.remove(group); group.traverse(o => o.geometry?.dispose?.()); }
    group = new THREE.Group();
    const map = nav.map, g = gridOf(map), res = g.resolution;
    const walls = [], floors = [];
    for (let i = 0; i < map.length; i++) {
      if (map[i] === UNKNOWN) continue;
      const x = g.originX + (i % g.width + .5) * res, z = g.originZ + (Math.floor(i / g.width) + .5) * res;
      (map[i] === OCCUPIED ? walls : floors).push([x, z]);
    }
    const m4 = new THREE.Matrix4();
    const floorMesh = new THREE.InstancedMesh(new THREE.BoxGeometry(res, .03, res), new THREE.MeshStandardMaterial({ color: '#fbfcf8', roughness: .95 }), Math.max(1, floors.length));
    floors.forEach(([x, z], i) => { m4.makeTranslation(x, .015, z); floorMesh.setMatrixAt(i, m4); });
    floorMesh.count = floors.length;
    const wallMesh = new THREE.InstancedMesh(new THREE.BoxGeometry(res, 1.1, res), new THREE.MeshStandardMaterial({ color: '#41675a', roughness: .85 }), Math.max(1, walls.length));
    walls.forEach(([x, z], i) => { m4.makeTranslation(x, .55, z); wallMesh.setMatrixAt(i, m4); });
    wallMesh.count = walls.length;
    group.add(floorMesh, wallMesh);
    scene.add(group);
    if (!framed) {
      const cx = g.originX + g.width * res / 2, cz = g.originZ + g.height * res / 2, size = Math.max(g.width, g.height) * res;
      camera.position.set(cx, size * 1.05, cz + size * .55);
      controls.target.set(cx, 0, cz);
      framed = true;
    }
    builtRevision = map.revision;
    builtMap = map;
  }
  return {
    update(session) {
      const nav = session.nav;
      if (nav.map.revision !== builtRevision || nav.map !== builtMap) rebuild(nav);
      const b = nav.belief;
      robot.position.set(b.x, session.world.floorY, b.z);
      robot.rotation.y = b.yaw;
      controls.update();
      renderer.render(scene, camera);
    },
  };
}
