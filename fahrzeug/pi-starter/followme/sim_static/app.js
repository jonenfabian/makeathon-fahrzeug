import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { GLTFLoader } from './vendor/GLTFLoader.js';

const $ = (id) => document.getElementById(id);
let state, lease = null, framePending = false;
let toastTimer;
function notify(message) {
  $('toast').textContent = message; $('toast').hidden = false;
  clearTimeout(toastTimer); toastTimer = setTimeout(() => { $('toast').hidden = true; }, 4500);
}
async function post(path, body = {}) {
  const response = await fetch(`/api/${path}`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || 'Befehl konnte nicht ausgeführt werden.');
  return result;
}
function action(button, fn) {
  $(button).addEventListener('click', async () => {
    $(button).disabled = true;
    try { await fn(); } catch (error) { notify(error.message); }
    finally { $(button).disabled = false; }
  });
}
action('start', async () => { if (state) lease = (await post('start', { revision: state.revision })).lease; });
action('stop', async () => { lease = null; await post('stop'); });
action('reset', async () => { lease = null; await post('reset', { scenario: $('scenario').value }); });
action('add-box', () => post('box', { x_m: Number($('box-x').value), y_m: Number($('box-y').value) }));
$('hidden').addEventListener('change', async () => {
  try { await post('hide', { hidden: $('hidden').checked }); } catch (error) { notify(error.message); }
});
$('scenario').addEventListener('change', async () => {
  lease = null;
  try { await post('reset', { scenario: $('scenario').value }); } catch (error) { notify(error.message); }
});
setInterval(async () => {
  if (!lease) return;
  try { await post('heartbeat', { lease }); }
  catch (error) { lease = null; notify(error.message); }
}, 250);
window.addEventListener('pagehide', () => {
  if (lease) navigator.sendBeacon('/api/stop', new Blob(['{}'], { type: 'application/json' }));
});

const host = $('world');
const scene = new THREE.Scene(); scene.background = new THREE.Color('#eef2ed');
scene.fog = new THREE.Fog('#eef2ed', 12, 24);
const camera = new THREE.PerspectiveCamera(40, 1, .02, 45);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap; renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1;
host.append(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true; controls.maxPolarAngle = Math.PI / 2 - .04; controls.minDistance = .7; controls.maxDistance = 16;
function centerView() { camera.position.set(-2.8, 3.4, 4.1); controls.target.set(1.6, .05, 0); controls.update(); }
centerView(); $('view-reset').addEventListener('click', centerView);
scene.add(new THREE.HemisphereLight(0xffffff, 0x829380, 2.7));
const sun = new THREE.DirectionalLight(0xfff9e8, 3.4); sun.position.set(-3, 7, 4); sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048); Object.assign(sun.shadow.camera, { left: -6, right: 8, top: 6, bottom: -6 });
sun.shadow.bias = -.0002; scene.add(sun);
const ground = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), new THREE.MeshStandardMaterial({ color: '#e9eee6', roughness: 1 }));
ground.rotation.x = -Math.PI / 2; ground.receiveShadow = true; scene.add(ground);
const grid = new THREE.GridHelper(20, 20, '#bdcbb9', '#d2ddcd'); grid.position.y = .002; scene.add(grid);
const robot = new THREE.Group(); scene.add(robot);
const robotHalo = new THREE.Mesh(new THREE.RingGeometry(.16, .18, 48), new THREE.MeshBasicMaterial({ color: '#76a28b', transparent: true, opacity: .5, side: THREE.DoubleSide }));
robotHalo.rotation.x = -Math.PI / 2; robotHalo.position.y = .004; robot.add(robotHalo);
new GLTFLoader().load('/fahrzeug.glb', (gltf) => {
  gltf.scene.scale.setScalar(.001);
  gltf.scene.traverse((node) => { if (node.isMesh) { node.castShadow = true; node.receiveShadow = true; } });
  robot.add(gltf.scene); $('scene-loading').textContent = '';
}, undefined, () => { $('scene-loading').textContent = 'Das Fahrzeugmodell konnte nicht geladen werden. Seite neu laden.'; });
const worldObjects = new THREE.Group(); scene.add(worldObjects);
const textureLoader = new THREE.TextureLoader();
const textures = new Map();
function markerTexture(id) {
  if (!textures.has(id)) {
    const texture = textureLoader.load(`/api/tag/${id}.png`);
    texture.colorSpace = THREE.SRGBColorSpace; texture.magFilter = THREE.NearestFilter;
    textures.set(id, texture);
  }
  return textures.get(id);
}
const boxMaterial = new THREE.MeshStandardMaterial({ color: '#cfb48d', roughness: .92 });
const tapeMaterial = new THREE.MeshStandardMaterial({ color: '#b99c73', roughness: .95 });
const personGroup = new THREE.Group();
const personBody = new THREE.Mesh(new THREE.CapsuleGeometry(.12, .55, 6, 12), new THREE.MeshStandardMaterial({ color: '#adbcab', roughness: .9 }));
personBody.position.set(.16, .61, 0); personBody.castShadow = true;
const personHead = new THREE.Mesh(new THREE.SphereGeometry(.105, 20, 16), new THREE.MeshStandardMaterial({ color: '#e0cbb0', roughness: .9 }));
personHead.position.set(.16, 1.08, 0); personHead.castShadow = true;
personGroup.add(personBody, personHead); scene.add(personGroup);
let objectSignature = '';
function clearWorldObjects() {
  worldObjects.traverse((object) => { if (object.isMesh) object.geometry.dispose(); });
  worldObjects.clear();
}
function updateWorld(world) {
  const signature = JSON.stringify([world.boxes, world.tags.filter((tag) => tag.marker_id !== 300)]);
  if (signature !== objectSignature) {
    clearWorldObjects(); objectSignature = signature;
    world.boxes.forEach((box) => {
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(box.length_m, box.height_m, box.width_m), boxMaterial);
      mesh.position.set(box.x_m, box.height_m / 2, -box.y_m); mesh.castShadow = true; mesh.receiveShadow = true;
      const tape = new THREE.Mesh(new THREE.BoxGeometry(box.length_m + .001, .002, .045), tapeMaterial);
      tape.position.y = box.height_m / 2 + .001; mesh.add(tape); worldObjects.add(mesh);
    });
    world.tags.filter((tag) => tag.marker_id !== 300).forEach((tag) => {
      const mesh = new THREE.Mesh(new THREE.PlaneGeometry(tag.size_m * 4 / 3, tag.size_m * 4 / 3), new THREE.MeshBasicMaterial({ map: markerTexture(tag.marker_id), side: THREE.DoubleSide }));
      mesh.position.set(tag.x_m, tag.z_m, -tag.y_m); mesh.rotation.y = -Math.PI / 2;
      worldObjects.add(mesh);
      const pole = new THREE.Mesh(new THREE.CylinderGeometry(.009, .009, tag.z_m, 8), tapeMaterial);
      pole.position.set(tag.x_m + .01, tag.z_m / 2, -tag.y_m); worldObjects.add(pole);
    });
  }
  personGroup.position.set(world.target.x_m, 0, -world.target.y_m);
  personMarker.visible = !world.target.hidden;
}
const personMarker = new THREE.Mesh(new THREE.PlaneGeometry(.168 * 4 / 3, .168 * 4 / 3), new THREE.MeshBasicMaterial({ map: markerTexture(300), side: THREE.DoubleSide }));
personMarker.position.set(0, .45, 0); personMarker.rotation.y = -Math.PI / 2; personGroup.add(personMarker);
const trail = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: '#7aa78d', transparent: true, opacity: .8 })); scene.add(trail);
new ResizeObserver(() => {
  const width = host.clientWidth, height = host.clientHeight;
  renderer.setSize(width, height, false); camera.aspect = width / height; camera.updateProjectionMatrix();
}).observe(host);
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });

function value(id, number, unit = '') { $(id).innerHTML = `${number == null ? '–' : number.toFixed(2)}<small>${unit}</small>`; }
async function refresh() {
  try {
    const response = await fetch('/api/state'); if (!response.ok) throw new Error();
    state = await response.json();
    if (!state.running) lease = null;
    const p = state.perception, m = state.mcu;
    $('elapsed').textContent = `${String(Math.floor(state.elapsed_s / 60)).padStart(2, '0')}:${String(Math.floor(state.elapsed_s % 60)).padStart(2, '0')}`;
    $('start').disabled = state.running || Boolean(state.error);
    $('start').textContent = state.running ? 'Versuch läuft' : '▶ Versuch starten';
    $('add-box').disabled = state.running; $('run-dot').classList.toggle('active', state.running);
    $('run-label').textContent = state.running ? (lease ? 'Dein Versuch läuft' : 'Anderer Browser fährt') : 'Bereit für deinen Versuch';
    $('scenario').value = state.world.scenario;
    $('hidden').checked = state.world.target.hidden;
    $('exercise-title').textContent = state.scenarios[state.world.scenario].name;
    $('exercise-text').textContent = state.scenarios[state.world.scenario].exercise;
    $('controller').textContent = p.state;
    value('target-distance', p.target?.distance_m, ' m');
    value('front-distance', m?.fresh && m.tof_mm !== 65535 ? m.tof_mm / 1000 : null, ' m');
    value('speed', m?.fresh ? (m.v_l_mm_s + m.v_r_mm_s) / 2000 : null, ' m/s');
    $('pico').textContent = m ? `${m.state}${m.flag_names.length ? ` · ${m.flag_names.join(', ')}` : ''}` : 'KEINE VERBINDUNG';
    // Fehler der Aufgabendatei bleibt sichtbar, bis ein Start die korrigierte Datei lädt. Start bleibt möglich.
    $('reason').textContent = state.task_error && !state.running ? `Fehler in eurer Aufgabendatei: ${state.task_error}. Datei korrigieren, dann Versuch starten.` : state.reason;
    $('reason').classList.toggle('error', Boolean(state.error || (state.task_error && !state.running)));
    $('image-age').textContent = p.fresh ? `${Math.round(p.age_s * 1000)} ms Bildalter` : 'Kein frisches Bild';
    $('tags').textContent = p.markers.length ? p.markers.map((t) => `ID ${t.marker_id}`).join(' · ') : 'Kein Marker erkannt';
    $('wheels').textContent = m ? `${m.v_l_mm_s} / ${m.v_r_mm_s} mm/s` : '–';
    $('encoders').textContent = m ? `${m.ticks_l} / ${m.ticks_r}` : '–';
    $('command').textContent = `${state.command[0].toFixed(2)} m/s / ${state.command[1].toFixed(2)} rad/s`;
    $('task').textContent = state.task;
    robot.position.set(state.pose.x_m, 0, -state.pose.y_m); robot.rotation.y = state.pose.yaw_rad;
    updateWorld(state.world);
    trail.geometry.dispose(); trail.geometry = new THREE.BufferGeometry().setFromPoints(state.trail.map(([x, y]) => new THREE.Vector3(x, .006, -y)));
    if (!framePending) {
      framePending = true;
      $('camera').onload = $('camera').onerror = () => { framePending = false; };
      $('camera').src = `/api/frame.jpg?t=${Date.now()}`;
    }
  } catch {
    $('run-label').textContent = 'Verbindung unterbrochen'; $('run-dot').classList.remove('active');
    $('start').disabled = true; $('reason').textContent = 'Das Lernlabor antwortet nicht. Prüfe das Terminal und lade die Seite neu.';
    $('reason').classList.add('error');
  }
  setTimeout(refresh, 200);
}
refresh();
