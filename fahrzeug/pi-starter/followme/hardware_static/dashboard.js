/* Direkte Teambedienung. Eine Browserfahrt endet beim Verlassen oder Verbindungsverlust. */
const $ = id => document.getElementById(id);
let state = null, lease = null, operation = 0, busy = false, heartbeatBusy = false;
let serverInstance = null, connected = false, cameraRetry = null;
const number = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '—';
const set = (id, value) => { $(id).textContent = value; };
function notice(message) { $('notice').hidden = !message; set('notice', message || ''); }
// Bildstrom nur im sichtbaren Tab offen halten. Jeder MJPEG-Strom belegt dauerhaft eine der wenigen
// Verbindungen des Browsers zum Pi, sonst kommen ab etwa sechs Tabs Status und Stopp nicht mehr durch.
function openCamera() {
  clearTimeout(cameraRetry); cameraRetry = null;
  if (!document.hidden) $('camera').src = '/stream?' + Date.now();
}
function closeCamera() {
  clearTimeout(cameraRetry); cameraRetry = null;
  $('camera').removeAttribute('src');
}
$('camera').addEventListener('error', () => {
  // Fahrprogramm neu gestartet oder Netz weg: nicht beim eingefrorenen letzten Bild bleiben.
  if ($('camera').hasAttribute('src') && !cameraRetry) cameraRetry = setTimeout(openCamera, 1000);
});
// Technische Browsertexte („Failed to fetch“, „signal timed out“) nie anzeigen.
const message = (error, fallback) => error?.httpStatus ? error.message : fallback;
async function post(path, body = {}) {
  let response, data;
  try {
    response = await fetch(path, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body), signal:AbortSignal.timeout(3000)});
    data = await response.json();
  } catch (_) {
    throw new Error('Keine Antwort vom Fahrprogramm.');
  }
  if (!response.ok) {
    const error = new Error(data.error || 'Befehl fehlgeschlagen.');
    error.httpStatus = response.status;
    throw error;
  }
  return data;
}
function render(data) {
  state = data;
  // Neues Fahrprogramm (andere Serverkennung) oder Verbindung wieder da: Bildstrom frisch öffnen.
  const instance = data.control?.instance || null;
  if (!connected || instance !== serverInstance) openCamera();
  if (!connected) notice('');  // Hinweis „nicht erreichbar“ gilt nicht mehr
  connected = true; serverInstance = instance;
  const pico = data.pico, view = data.perception || {}, control = data.control || {}, front = data.sensors?.front;
  const fresh = pico?.fresh && data.connection?.state === 'fresh';
  set('backend', data.runtime?.simulated ? 'Firmware-Simulator' : 'Echtes Fahrzeug');
  set('vehicle', data.vehicle_id || location.hostname);
  set('task', data.runtime?.task || '—');
  set('pico-state', fresh ? pico.state : 'Keine frischen Daten');
  set('flags', fresh ? (pico.flags.length ? pico.flags.join(' · ') : 'Keine Fehlerflags') : '—');
  set('owner', control.active ? (control.owner === 'browser' ? (lease ? 'Dieser Browser' : 'Anderer Browser') : 'Pi-Terminal') : 'Frei');
  set('drive-title', control.active ? (fresh && pico.state === 'RUN' ? 'Fahrt läuft' : 'Start wird geprüft') : control.task_error ? 'Aufgabe korrigieren' : control.stopped_confirmed ? 'Bereit für deinen Versuch' : 'Status prüfen');
  // Fehler der Aufgabendatei für alle offenen Seiten, bis ein Start die korrigierte Datei lädt.
  $('task-error').hidden = !control.task_error;
  set('task-error', control.task_error ? 'Fehler in eurer Aufgabendatei: ' + control.task_error : '');
  set('reason', control.reason || 'Dieser Stream bietet nur Beobachtung.');
  $('start').disabled = busy || !control.enabled || control.active || !fresh || pico.state !== 'STANDBY' || Math.abs(pico.v_l_mm_s) > 2 || Math.abs(pico.v_r_mm_s) > 2;
  $('start').textContent = busy ? 'Starten …' : 'Fahrt starten';
  set('target', number(view.fresh ? view.target?.distance_m : null));
  set('front', Number.isFinite(front?.distance_mm) ? number(front.distance_mm / 1000) : '—');
  const noTarget = fresh && pico.tof_raw_mm === 65535 && front?.state === 'legacy_unknown';
  set('front-note', noTarget ? 'Kein Ziel · keine Freifahrt-Meldung' : front?.distance_mm != null ? 'Meter · Einzelmessalter unbekannt' : 'Kein gültiger Messwert');
  set('speed', number(fresh ? (pico.v_l_mm_s + pico.v_r_mm_s) / 2000 : null));
  set('controller', view.fresh ? view.controller_state || '—' : '—');
  const command = control.command;
  set('command', command ? `Soll ${number(command.v_mps)} m/s · ${number(command.omega_rad_s)} rad/s` : 'Keine Sollwerte');
  set('left', fresh ? `${pico.v_l_mm_s} mm/s` : '—'); set('right', fresh ? `${pico.v_r_mm_s} mm/s` : '—');
  set('ticks', fresh ? `${pico.ticks_l} / ${pico.ticks_r}` : '—');
  set('voltage', `${number(fresh ? pico.vmot_mv / 1000 : null)} V`);
  set('status-age', fresh ? `${number(pico.status_age_s * 1000, 0)} ms Statusalter` : 'Pico-Status fehlt oder ist veraltet');
  set('image-age', Number.isFinite(view.age_s) ? `${number(view.age_s * 1000, 0)} ms Bildalter` : 'Warte auf frisches Bild');
  set('markers', view.fresh ? (view.markers?.map(m => `ID ${m.marker_id}`).join(' · ') || 'Kein Marker erkannt') : 'Keine frische Erkennung');
  set('image-state', view.fresh ? 'Live' : 'Bild veraltet');
  $('camera').parentElement.classList.toggle('stale', !view.fresh);
  if (control.error) notice(control.error);
}
async function poll() {
  try {
    const response = await fetch('/status', {cache:'no-store', signal:AbortSignal.timeout(2000)});
    if (!response.ok) throw new Error('Fahrprogramm antwortet nicht.');
    render(await response.json());
  } catch (_) {
    // Kein alter Wert bleibt stehen: Alles, was vom Fahrprogramm stammt, wird neutral.
    connected = false; state = null;
    $('start').disabled = true; set('drive-title', 'Verbindung unterbrochen');
    set('backend', 'Keine Verbindung'); set('pico-state', 'Unbekannt');
    ['target','front','speed','controller','left','right','ticks','voltage','flags','owner','task'].forEach(id => set(id, '—'));
    set('reason', 'Zustand unbekannt. Stillstand am Fahrzeug prüfen.');
    set('command', 'Keine Sollwerte'); set('front-note', 'Kein gültiger Messwert');
    set('status-age', 'Keine Verbindung zum Fahrprogramm'); set('image-age', 'Kein Bild');
    set('markers', 'Keine frische Erkennung');
    $('task-error').hidden = true; set('task-error', '');
    set('image-state', 'Kein Bild'); $('camera').parentElement.classList.add('stale');
    notice('Fahrprogramm nicht erreichbar. Läuft es noch auf dem Pi? Nach Wiederverbindung bewusst neu starten.');
    // Keine Wiederaufnahme durch einen später eintreffenden Start.
    if (lease) { leave(); }
  } finally { setTimeout(poll, 400); }
}
function release(token) {
  const blob = new Blob([JSON.stringify({lease:token})], {type:'application/json'});
  navigator.sendBeacon('/api/release', blob);
}
function leave() {
  operation++;
  if (lease) release(lease);
  lease = null;
}
$('start').addEventListener('click', async () => {
  if (!state?.control || busy) return;
  const current = ++operation;
  busy = true; notice(''); render(state);
  try {
    const result = await post('/api/start', {instance:state.control.instance, revision:state.control.revision});
    if (current !== operation || document.hidden) release(result.lease);
    else lease = result.lease;
  } catch (error) {
    // Ladefehler zeigt das Feld der Aufgabendatei; Netzfehler verständlich statt Browsertext.
    if (!error.httpStatus) notice('Keine Antwort vom Fahrprogramm.');
    else if (!error.message.startsWith('Aufgabendatei konnte nicht geladen werden') && !error.message.startsWith('Aufgabe lädt zu lange')) notice(error.message);
    // Auch eine noch vorbereitende Startanfrage durch eine neue Bediengeneration entwerten.
    if (!error.httpStatus) {
      try { await post('/api/stop'); } catch (_) { /* Verbindungsverlust beendet die Browserfahrt. */ }
    }
  }
  finally { busy = false; }
});
$('stop').addEventListener('click', async () => {
  operation++; lease = null; notice('');
  try { await post('/api/stop'); } catch (error) {
    notice((error.httpStatus ? error.message : 'Stopp nicht bestätigt.') + ' Stillstand am Fahrzeug prüfen.');
  }
});
setInterval(async () => {
  if (!lease || heartbeatBusy) return;
  const token = lease;
  heartbeatBusy = true;
  try { await post('/api/heartbeat', {lease:token}); }
  catch (error) { if (lease === token) { leave(); notice(message(error, 'Verbindung zum Fahrprogramm unterbrochen. Die Browserfahrt endet.')); } }
  finally { heartbeatBusy = false; }
}, 400);
window.addEventListener('pagehide', leave);
document.addEventListener('visibilitychange', () => {
  if (document.hidden) { leave(); closeCamera(); } else if (connected) openCamera();
});
if (document.hidden) closeCamera();  // im Hintergrund geöffneter Tab hält keinen Bildstrom
poll();
