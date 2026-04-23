/* =====================================================================
 *  WebUI.h — SoftAP Web Dashboard + WebSocket (thư viện ESP32 chính thức)
 *
 *  Dùng:  WebServer (port 80)  cho trang HTML
 *         WebSocketsServer (port 81, thư viện Links2004/arduinoWebSockets)
 *
 *  API:
 *    webUIInit()   — Khởi động WiFi AP + server (gọi trong setup())
 *    webUILoop()   — Xử lý client (gọi trong task Core 0)
 *    webUIBroadcast() — Gửi JSON telemetry tới mọi WS client
 * =====================================================================*/
#ifndef WEBUI_H
#define WEBUI_H

#include "Config.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>

static WebServer      g_httpServer(WEB_PORT);
static WebSocketsServer g_wsServer(WS_PORT);
static Preferences    g_prefs;

/* -----------------------------------------------------------------------
 *  Trang HTML Dashboard (gzip không cần thiết ở kích thước này)
 * --------------------------------------------------------------------- */
static const char HTML_PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1.0"/>
<title>SmartMarketBot Dashboard</title>
<style>
  :root{--bg:#0d1117;--card:#161b22;--border:#30363d;--accent:#58a6ff;--green:#3fb950;--red:#f85149;--text:#e6edf3;--muted:#8b949e}
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;min-height:100vh;padding:16px}
  h1{text-align:center;color:var(--accent);font-size:1.4rem;margin-bottom:16px;letter-spacing:2px}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;max-width:900px;margin:0 auto}
  @media(max-width:600px){.grid{grid-template-columns:1fr}}
  .card{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:14px}
  .card h2{font-size:.85rem;color:var(--muted);text-transform:uppercase;letter-spacing:1px;margin-bottom:10px}
  /* Joystick */
  #jsZone{width:160px;height:160px;background:#1c2128;border:2px solid var(--border);border-radius:50%;position:relative;touch-action:none;margin:0 auto;cursor:crosshair}
  #jsKnob{width:52px;height:52px;background:var(--accent);border-radius:50%;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);pointer-events:none;box-shadow:0 0 16px #58a6ff88;transition:top .05s,left .05s}
  /* Mode toggle */
  .toggle-row{display:flex;align-items:center;gap:12px;justify-content:center;margin-top:10px}
  .mode-btn{padding:8px 20px;border:none;border-radius:8px;font-size:.85rem;cursor:pointer;font-weight:600;transition:.15s}
  .mode-btn.active{background:var(--accent);color:#000}
  .mode-btn:not(.active){background:var(--border);color:var(--text)}
  /* Sensor bars */
  .sensor-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .s-item{display:flex;flex-direction:column;gap:4px}
  .s-label{font-size:.75rem;color:var(--muted)}
  .s-bar-bg{background:#1c2128;border-radius:4px;height:10px;overflow:hidden}
  .s-bar{height:10px;border-radius:4px;background:var(--green);transition:width .2s}
  .s-val{font-size:.8rem;font-weight:600}
  /* RPM cards */
  .rpm-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .rpm-item{background:#1c2128;border-radius:8px;padding:8px;text-align:center}
  .rpm-val{font-size:1.3rem;font-weight:700;color:var(--accent)}
  .rpm-lbl{font-size:.7rem;color:var(--muted)}
  /* Speed slider */
  input[type=range]{width:100%;accent-color:var(--accent)}
  .estop{width:100%;padding:12px;background:var(--red);border:none;border-radius:10px;color:#fff;font-size:1rem;font-weight:700;cursor:pointer;margin-top:8px;letter-spacing:1px}
  .status-dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--green);margin-right:6px;animation:blink 1.2s infinite}
  @keyframes blink{0%,100%{opacity:1}50%{opacity:.3}}
</style>
</head>
<body>
<h1>&#x1F916; SmartMarketBot Dashboard</h1>
<div class="grid">
  <!-- Joystick + Mode -->
  <div class="card">
    <h2>&#x1F579; Điều khiển</h2>
    <div id="jsZone"><div id="jsKnob"></div></div>
    <div class="toggle-row">
      <button class="mode-btn active" id="btnManual" onclick="setMode(0)">Lái tay</button>
      <button class="mode-btn"        id="btnAuto"   onclick="setMode(1)">Tự hành</button>
    </div>
    <input type="range" id="spdSlider" min="0" max="100" value="60"
           oninput="sendSpeed(this.value)" style="margin-top:12px"/>
    <div style="text-align:center;font-size:.8rem;color:var(--muted)">Tốc độ: <span id="spdVal">60</span>%</div>
    <button class="estop" onclick="sendEstop()">&#x26A0; DỪNG KHẨN CẤP</button>
  </div>

  <!-- Cảm biến khoảng cách -->
  <div class="card">
    <h2>&#x1F4E1; Cảm biến khoảng cách</h2>
    <div class="sensor-grid" id="sensorGrid">
      <!-- Tạo động bằng JS -->
    </div>
  </div>

  <!-- RPM 4 bánh -->
  <div class="card">
    <h2>&#x1F6DE; Tốc độ bánh xe (RPM)</h2>
    <div class="rpm-grid">
      <div class="rpm-item"><div class="rpm-val" id="rFL">0</div><div class="rpm-lbl">Trước Trái</div></div>
      <div class="rpm-item"><div class="rpm-val" id="rFR">0</div><div class="rpm-lbl">Trước Phải</div></div>
      <div class="rpm-item"><div class="rpm-val" id="rRL">0</div><div class="rpm-lbl">Sau Trái</div></div>
      <div class="rpm-item"><div class="rpm-val" id="rRR">0</div><div class="rpm-lbl">Sau Phải</div></div>
    </div>
  </div>

  <!-- Kết nối + quãng đường -->
  <div class="card">
    <h2>&#x1F4F6; Trạng thái</h2>
    <p style="font-size:.85rem"><span class="status-dot"></span>WebSocket <span id="wsStatus">đang kết nối...</span></p>
    <div style="margin-top:10px;font-size:.82rem;color:var(--muted)">
      <p>Quãng đường TB (m): <b id="distAvg" style="color:var(--text)">0.00</b></p>
      <p>Chế độ hiện tại: <b id="modeLabel" style="color:var(--accent)">Lái tay</b></p>
    </div>
    <button onclick="odomReset()" style="margin-top:12px;padding:7px 16px;border:1px solid var(--border);background:transparent;color:var(--text);border-radius:7px;cursor:pointer;font-size:.8rem">Reset Quãng đường</button>
  </div>
</div>

<script>
// ── WebSocket ──────────────────────────────────────────────────────────
const WS_URL = `ws://${location.hostname}:81`;
let ws, retryTimer;
const sensorDefs = [
  {id:'dLF', label:'LiDAR Trước', max:500},
  {id:'dLB', label:'LiDAR Sau',   max:500},
  {id:'dUF', label:'Siêu âm Trước', max:300},
  {id:'dUB', label:'Siêu âm Sau',   max:300},
  {id:'dUL', label:'Siêu âm Trái',  max:300},
  {id:'dUR', label:'Siêu âm Phải',  max:300},
];

function buildSensorGrid() {
  const g = document.getElementById('sensorGrid');
  g.innerHTML = sensorDefs.map(s=>`
    <div class="s-item">
      <span class="s-label">${s.label}</span>
      <div class="s-bar-bg"><div class="s-bar" id="bar_${s.id}" style="width:0%"></div></div>
      <span class="s-val" id="val_${s.id}">-- cm</span>
    </div>`).join('');
}

function updateSensor(id, val, max) {
  const v = document.getElementById('val_'+id);
  const b = document.getElementById('bar_'+id);
  if (!v) return;
  v.textContent = val + ' cm';
  const pct = Math.min(100, val / max * 100);
  b.style.width = pct + '%';
  b.style.background = val < 15 ? '#f85149' : val < 50 ? '#d29922' : '#3fb950';
}

function connectWS() {
  ws = new WebSocket(WS_URL);
  ws.onopen  = () => { document.getElementById('wsStatus').textContent='đã kết nối'; clearInterval(retryTimer); };
  ws.onclose = () => { document.getElementById('wsStatus').textContent='mất kết nối'; retryTimer=setTimeout(connectWS,2000); };
  ws.onmessage = e => {
    try {
      const d = JSON.parse(e.data);
      updateSensor('dLF', d.lf ?? 0, 500);
      updateSensor('dLB', d.lb ?? 0, 500);
      updateSensor('dUF', d.uf ?? 0, 300);
      updateSensor('dUB', d.ub ?? 0, 300);
      updateSensor('dUL', d.ul ?? 0, 300);
      updateSensor('dUR', d.ur ?? 0, 300);
      document.getElementById('rFL').textContent = (d.rFL ?? 0).toFixed(0);
      document.getElementById('rFR').textContent = (d.rFR ?? 0).toFixed(0);
      document.getElementById('rRL').textContent = (d.rRL ?? 0).toFixed(0);
      document.getElementById('rRR').textContent = (d.rRR ?? 0).toFixed(0);
      const da = ((d.dFL??0)+(d.dRL??0)+(d.dFR??0)+(d.dRR??0))/4;
      document.getElementById('distAvg').textContent = da.toFixed(2);
      document.getElementById('modeLabel').textContent = d.mode===1?'Tự hành':'Lái tay';
    } catch(ex) {}
  };
}

// ── Joystick ───────────────────────────────────────────────────────────
const zone  = document.getElementById('jsZone');
const knob  = document.getElementById('jsKnob');
const R = 80; let dragging=false, jX=0, jY=0;
let jsTimer = null;

function clamp(v,lo,hi){return Math.max(lo,Math.min(hi,v));}

function joyUpdate(cx,cy) {
  const rect = zone.getBoundingClientRect();
  const ox = cx - rect.left - rect.width/2;
  const oy = cy - rect.top  - rect.height/2;
  const dist = Math.sqrt(ox*ox+oy*oy);
  const scale = dist > R ? R/dist : 1;
  jX = Math.round(ox*scale/R*100);
  jY = Math.round(-oy*scale/R*100); // Y đảo: lên = tiến
  knob.style.left = (rect.width/2 + ox*scale) + 'px';
  knob.style.top  = (rect.height/2 + oy*scale) + 'px';
  sendJoy(jX, jY);
}

function joyRelease() {
  dragging=false; jX=0; jY=0;
  knob.style.left='50%'; knob.style.top='50%';
  sendJoy(0,0);
}

zone.addEventListener('mousedown',  e=>{dragging=true; joyUpdate(e.clientX,e.clientY);});
zone.addEventListener('mousemove',  e=>{if(dragging) joyUpdate(e.clientX,e.clientY);});
zone.addEventListener('mouseup',    joyRelease);
zone.addEventListener('mouseleave', joyRelease);
zone.addEventListener('touchstart', e=>{e.preventDefault();dragging=true;joyUpdate(e.touches[0].clientX,e.touches[0].clientY);},{passive:false});
zone.addEventListener('touchmove',  e=>{e.preventDefault();if(dragging)joyUpdate(e.touches[0].clientX,e.touches[0].clientY);},{passive:false});
zone.addEventListener('touchend',   joyRelease);

// ── Gửi lệnh ─────────────────────────────────────────────────────────
function wsSend(obj) { if(ws && ws.readyState===1) ws.send(JSON.stringify(obj)); }
function sendJoy(x,y){ wsSend({t:'joy',x,y}); }
function sendSpeed(v){ document.getElementById('spdVal').textContent=v; wsSend({t:'spd',v:parseInt(v)}); }
function setMode(m) {
  document.getElementById('btnManual').classList.toggle('active', m===0);
  document.getElementById('btnAuto'  ).classList.toggle('active', m===1);
  wsSend({t:'mode',m});
}
function sendEstop(){ wsSend({t:'estop'}); }
function odomReset(){ wsSend({t:'odomReset'}); }

// ── Init ──────────────────────────────────────────────────────────────
buildSensorGrid();
connectWS();
</script>
</body>
</html>
)rawhtml";

/* -----------------------------------------------------------------------
 *  WebSocket callback — nhận lệnh từ Dashboard
 * --------------------------------------------------------------------- */
static void onWebSocketEvent(uint8_t num, WStype_t type,
                             uint8_t *payload, size_t length) {
  if (type == WStype_TEXT) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (err) return;
    const char *t = doc["t"];
    if (!t) return;

    if (strcmp(t, "joy") == 0) {
      g_state.cmdX = (int16_t)constrain((int)doc["x"].as<int>(), -100, 100);
      g_state.cmdY = (int16_t)constrain((int)doc["y"].as<int>(), -100, 100);
    } else if (strcmp(t, "spd") == 0) {
      uint16_t pct = doc["v"].as<uint16_t>();
      if (pct > 100) pct = 100;
      g_state.baseSpeed = (uint16_t)((uint32_t)pct * PWM_MAX / 100);
      // Lưu vào Flash
      g_prefs.begin(NVS_NAMESPACE, false);
      g_prefs.putUInt("baseSpeed", g_state.baseSpeed);
      g_prefs.end();
    } else if (strcmp(t, "mode") == 0) {
      g_state.mode = (RobotMode)doc["m"].as<uint8_t>();
    } else if (strcmp(t, "estop") == 0) {
      g_state.estop = true;
    } else if (strcmp(t, "odomReset") == 0) {
      extern void odomResetDistance();
      odomResetDistance();
    }
  }
}

/* -----------------------------------------------------------------------
 *  Broadcast telemetry JSON tới mọi client
 * --------------------------------------------------------------------- */
inline void webUIBroadcast() {
  JsonDocument doc;
  doc["lf"]   = g_state.lidarFront;
  doc["lb"]   = g_state.lidarBack;
  doc["uf"]   = g_state.usFront;
  doc["ub"]   = g_state.usBack;
  doc["ul"]   = g_state.usLeft;
  doc["ur"]   = g_state.usRight;
  doc["rFL"]  = g_state.rpmFL;
  doc["rRL"]  = g_state.rpmRL;
  doc["rFR"]  = g_state.rpmFR;
  doc["rRR"]  = g_state.rpmRR;
  doc["dFL"]  = g_state.distFL;
  doc["dRL"]  = g_state.distRL;
  doc["dFR"]  = g_state.distFR;
  doc["dRR"]  = g_state.distRR;
  doc["mode"] = (uint8_t)g_state.mode;
  doc["estop"]= g_state.estop;

  char buf[512];
  serializeJson(doc, buf, sizeof(buf));
  g_wsServer.broadcastTXT(buf);
}

/* -----------------------------------------------------------------------
 *  Khởi tạo
 * --------------------------------------------------------------------- */
inline void webUIInit() {
  // Đọc tốc độ đã lưu từ Flash
  g_prefs.begin(NVS_NAMESPACE, true);
  g_state.baseSpeed = g_prefs.getUInt("baseSpeed", PWM_MAX * 60 / 100);
  g_prefs.end();

  // SoftAP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[WiFi] SoftAP '%s'  IP: %s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());

  // HTTP route
  g_httpServer.on("/", HTTP_GET, []() {
    g_httpServer.send_P(200, "text/html", HTML_PAGE);
  });
  g_httpServer.begin();

  // WebSocket
  g_wsServer.begin();
  g_wsServer.onEvent(onWebSocketEvent);
  Serial.printf("[WS] WebSocket server trên port %d\n", WS_PORT);
}

inline void webUILoop() {
  g_httpServer.handleClient();
  g_wsServer.loop();
}

#endif // WEBUI_H
