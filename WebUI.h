/* =====================================================================
 *  WebUI.h — HMI: SoftAP + WebSocket (LiDAR ưu tiên, siêu âm = an toàn)
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

static const char HTML_PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1.0,viewport-fit=cover"/>
<meta name="theme-color" content="#0a0e14"/>
<title>SmartMarketBot | Edge HMI</title>
<!-- Khong dung CDN: SoftAP khong co Internet, font ngoai lam trinh duyet treo lau -->
<style>
:root{
  --bg0:#080b0f;--bg1:#0d1219;--card:#111820;--line:#1e2836;--line2:#2a3545;
  --text:#e8edf4;--muted:#8a97ab;--accent:#2dd4bf;--accent2:#38bdf8;--amber:#fbbf24;--ok:#22c55e;--war:#f59e0b;--bad:#ef4444;
  --glow:0 0 40px rgba(45,212,191,.12);
}
*{box-sizing:border-box;margin:0;padding:0}
body{
  min-height:100dvh;background:radial-gradient(1200px 600px at 50% -20%,#12202e 0%,var(--bg0) 55%,#050608 100%);
  color:var(--text);font-family:system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;padding:12px 14px 24px;
}
.wrap{max-width:1000px;margin:0 auto}
.brand{
  text-align:center;padding:18px 8px 22px;
  background:linear-gradient(180deg,rgba(17,24,32,.6),transparent);border-bottom:1px solid var(--line);border-radius:0 0 20px 20px;
  box-shadow:var(--glow);
}
.brand h1{
  font-size:clamp(1.25rem,4vw,1.65rem);font-weight:700;letter-spacing:.12em;
  background:linear-gradient(90deg,var(--accent),var(--accent2),#a78bfa);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text;
}
.brand p{margin-top:6px;font-size:.78rem;color:var(--muted);line-height:1.45;max-width:36rem;margin-left:auto;margin-right:auto}
.pillrow{display:flex;flex-wrap:wrap;gap:6px;justify-content:center;margin-top:10px}
.pill{
  font-size:.65rem;letter-spacing:.04em;padding:4px 10px;border-radius:99px;
  background:rgba(45,212,191,.1);color:var(--accent);border:1px solid rgba(45,212,191,.25);
}
.pill.safety{background:rgba(245,158,11,.1);color:var(--amber);border-color:rgba(245,158,11,.3)}
.appgrid{display:grid;grid-template-columns:1.1fr 0.9fr;gap:12px;margin-top:14px}
@media(max-width:900px){.appgrid{grid-template-columns:1fr}}
.card{
  background:linear-gradient(160deg,rgba(20,28,40,.9),rgba(8,12,18,.95));
  border:1px solid var(--line);border-radius:16px;padding:16px;box-shadow:0 4px 24px rgba(0,0,0,.35);
}
.card h2{
  font-size:.72rem;font-weight:600;text-transform:uppercase;letter-spacing:.14em;color:var(--muted);margin-bottom:12px;
  display:flex;align-items:center;gap:8px
}
h2 .dot{width:6px;height:6px;border-radius:50%;background:var(--accent);box-shadow:0 0 8px var(--accent)}
h2 .dot.safety{background:var(--amber);box-shadow:0 0 8px var(--amber)}
/* LiDAR hero */
.lidar2{display:grid;grid-template-columns:1fr 1fr;gap:12px}
@media(max-width:520px){.lidar2{grid-template-columns:1fr}}
.lidar{
  position:relative;border-radius:20px;padding:4px;
  background:conic-gradient(from 200deg,rgba(45,212,191,.4),rgba(56,189,248,.2),rgba(167,139,250,.3),rgba(45,212,191,.2));
  box-shadow:0 0 0 1px rgba(255,255,255,.06) inset;
}
.lidar-in{
  background:var(--card);border-radius:16px;padding:16px 12px 14px;
  min-height:180px;display:flex;flex-direction:column;align-items:center;justify-content:center;
  border:1px solid var(--line2);
}
.lidar-arc{
  --p:.35;width:120px;height:120px;border-radius:50%;
  background:conic-gradient(var(--accent) calc(var(--p) * 1turn),#1a2330 0);
  padding:4px;
}
.lidar-arc>div{
  width:100%;height:100%;border-radius:50%;background:var(--bg0);
  display:flex;flex-direction:column;align-items:center;justify-content:center;
}
.lidar-num{
  font-family:ui-monospace,"Cascadia Mono","Segoe UI Mono",Consolas,monospace;font-size:1.75rem;font-weight:700;letter-spacing:-.02em;
  line-height:1.1;
}
.lidar-unit{font-size:.7rem;color:var(--muted);font-weight:500}
.lidar-ax{font-size:.68rem;font-weight:600;letter-spacing:.12em;margin-top:10px;color:var(--accent2)}
.lidar-ax b{color:var(--text);font-weight:600}
/* Bumpers */
.bump-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.b-item{
  background:#0c1016;border:1px solid var(--line);border-radius:10px;padding:8px 10px;
  border-left:3px solid var(--amber);
}
.b-item .bn{font-size:.65rem;color:var(--muted);text-transform:uppercase;letter-spacing:.06em}
.b-item .val{font-family:ui-monospace,Consolas,monospace;font-size:1rem;font-weight:600;margin-top:2px}
.b-bar{height:5px;border-radius:3px;background:#1a2330;margin-top:6px;overflow:hidden}
.b-fill{height:100%;border-radius:3px;transition:width .2s,background .2s}
/* Map placeholder */
.slam{
  min-height:120px;border:1px dashed var(--line2);border-radius:12px;
  display:flex;flex-direction:column;align-items:center;justify-content:center;gap:8px;padding:16px;
  background:repeating-linear-gradient(-12deg,transparent,transparent 8px,rgba(255,255,255,.02) 8px,rgba(255,255,255,.02) 9px);
}
.slam h3{font-size:.78rem;font-weight:600;color:var(--muted);text-align:center;max-width:20rem}
.slam p{font-size:.7rem;color:var(--muted);text-align:center;opacity:.85;max-width:24rem}
/* Joystick */
#jsZone{
  width:150px;height:150px;border-radius:50%;
  background:radial-gradient(circle at 40% 35%,#1a2638,#0a0d12 70%);
  border:2px solid var(--line2);position:relative;touch-action:none;margin:0 auto;cursor:crosshair;box-shadow:0 0 0 1px rgba(0,0,0,.4) inset;
}
#jsKnob{
  width:48px;height:48px;border-radius:50%;
  background:linear-gradient(160deg,#5eead4,#22d3ee);
  position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);
  box-shadow:0 4px 20px rgba(45,212,191,.4);pointer-events:none;transition:top .04s,left .04s;
}
.toggle-row{display:flex;align-items:center;gap:8px;justify-content:center;margin-top:12px;flex-wrap:wrap}
.mode-btn{
  padding:8px 18px;border:none;border-radius:10px;font-size:.78rem;cursor:pointer;font-weight:600;
  font-family:inherit;transition:transform .1s,box-shadow .15s,background .15s
}
.mode-btn:hover{transform:translateY(-1px)}
.mode-btn.active{background:linear-gradient(135deg,#2dd4bf,#22d3ee);color:#041016;box-shadow:0 4px 16px rgba(34,211,238,.3)}
.mode-btn:not(.active){background:var(--line);color:var(--text);border:1px solid var(--line2)}
input[type=range]{width:100%;accent-color:var(--accent);margin-top:10px}
.estop{
  width:100%;padding:12px;border:none;border-radius:12px;margin-top:10px;cursor:pointer;font-weight:700;letter-spacing:.04em;
  font-family:inherit;font-size:.92rem;
  background:linear-gradient(135deg,#f87171,#ef4444);color:#1a0505;box-shadow:0 4px 20px rgba(239,68,68,.3);
}
.rpm-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.rpm-item{background:#0c1016;border:1px solid var(--line);border-radius:10px;padding:10px 8px;text-align:center}
.rpm-val{font-family:ui-monospace,Consolas,monospace;font-size:1.15rem;font-weight:700;color:var(--accent2)}
.rpm-lbl{font-size:.62rem;color:var(--muted);text-transform:uppercase;letter-spacing:.06em;margin-top:2px}
.status-line{font-size:.8rem;margin-top:8px;display:flex;align-items:center;gap:6px;flex-wrap:wrap}
.sdot{width:8px;height:8px;border-radius:50%;background:var(--ok);box-shadow:0 0 8px var(--ok);animation:bl 1.2s ease infinite}
@keyframes bl{0%,100%{opacity:1}50%{opacity:.25}}
.badge-estop{
  display:none;padding:3px 8px;border-radius:6px;font-size:.65rem;font-weight:700;
  background:rgba(239,68,68,.2);color:#fecaca;border:1px solid rgba(239,68,68,.4);
}
.badge-estop.on{display:inline}
</style>
</head>
<body>
<div class="wrap">
  <header class="brand">
    <h1>SMARTMARKETBOT</h1>
    <p>IoT Edge HMI &mdash; lớp định vị chính: <strong>LiDAR</strong> (khoảng cách, nền tảng bản đồ / SLAM). <strong>HC-SR04</strong> chỉ phục vụ dừng cận cảnh, không dùng để lập bản đồ.</p>
    <div class="pillrow">
      <span class="pill">TF-Luna &middot; quét 2 hướng</span>
      <span class="pill safety">HC-SR04 &middot; vùng an toàn</span>
    </div>
  </header>

  <div class="appgrid">
    <div>
      <div class="card" style="margin-bottom:12px">
        <h2><span class="dot"></span> LiDAR &mdash; tầm nhìn chính</h2>
        <div class="lidar2">
          <div class="lidar">
            <div class="lidar-in">
              <div class="lidar-arc" id="ringF" style="--p:0.2"><div>
                <span class="lidar-num" id="vLF">--</span>
                <span class="lidar-unit">cm</span>
              </div></div>
              <div class="lidar-ax">TRƯỚC <b>LiDAR</b></div>
            </div>
          </div>
          <div class="lidar">
            <div class="lidar-in">
              <div class="lidar-arc" id="ringB" style="--p:0.2"><div>
                <span class="lidar-num" id="vLB">--</span>
                <span class="lidar-unit">cm</span>
              </div></div>
              <div class="lidar-ax">SAU · <b>LiDAR</b></div>
            </div>
          </div>
        </div>
      </div>

      <div class="card" style="margin-bottom:12px">
        <h2><span class="dot safety"></span> Bumper (HC-SR04) &mdash; dừng cận cảnh</h2>
        <p style="font-size:.68rem;color:var(--muted);margin-bottom:8px;opacity:.9">Khi vật cản rất gần (&lt;~50cm): giảm tốc / dừng. Không thay thế LiDAR để lập bản đồ.</p>
        <div class="bump-grid" id="bumpBox"></div>
      </div>

      <div class="card">
        <h2><span class="dot"></span> Bản đồ / SLAM (giai đoạn tích hợp)</h2>
        <div class="slam">
          <h3>Khu vực sẽ hiển thị bản đồ &amp; tuyến đường khi tích hợp Back-end (MQTT, ROS2, ...)</h3>
          <p>Quét bản đồ và điều hướng: module lớn &mdash; ESP32 có thể đẩy scan 2D, pose, odometry; server/edge xử lý SLAM và lập kế hoạch đường đi.</p>
        </div>
      </div>
    </div>

    <div>
      <div class="card" style="margin-bottom:12px">
        <h2>Điều khiển</h2>
        <div id="jsZone"><div id="jsKnob"></div></div>
        <div class="toggle-row">
          <button class="mode-btn active" id="btnManual" onclick="setMode(0)">Lái tay</button>
          <button class="mode-btn" id="btnAuto" onclick="setMode(1)">Tự hành (demo)</button>
        </div>
        <input type="range" id="spdSlider" min="0" max="100" value="60" oninput="sendSpeed(this.value)"/>
        <div style="text-align:center;font-size:.75rem;color:var(--muted);margin-top:4px">Tốc độ: <span id="spdVal">60</span>%</div>
        <button class="estop" type="button" onclick="sendEstop()">DỪNG KHẨN CẤP</button>
      </div>

      <div class="card" style="margin-bottom:12px">
        <h2>Bánh xe (RPM)</h2>
        <div class="rpm-grid">
          <div class="rpm-item"><div class="rpm-val" id="rFL">0</div><div class="rpm-lbl">T.Trai</div></div>
          <div class="rpm-item"><div class="rpm-val" id="rFR">0</div><div class="rpm-lbl">T.Phai</div></div>
          <div class="rpm-item"><div class="rpm-val" id="rRL">0</div><div class="rpm-lbl">S.Trai</div></div>
          <div class="rpm-item"><div class="rpm-val" id="rRR">0</div><div class="rpm-lbl">S.Phai</div></div>
        </div>
      </div>

      <div class="card">
        <h2>Trạng thái</h2>
        <p class="status-line">
          <span class="sdot"></span><span>WS </span><span id="wsStatus">...</span>
          <span class="badge-estop" id="eBadge">E-STOP</span>
        </p>
        <p style="font-size:.82rem;margin-top:8px;color:var(--muted)">Quãng đường TB: <b id="distAvg" style="color:var(--text)">0.00</b> m</p>
        <p style="font-size:.82rem;margin-top:4px;color:var(--muted)">Chế độ: <b id="modeLabel" style="color:var(--accent)">Lái tay</b></p>
        <button type="button" onclick="odomReset()" style="margin-top:10px;padding:6px 14px;border:1px solid var(--line2);background:transparent;color:var(--text);border-radius:8px;cursor:pointer;font-size:.75rem">Reset odom</button>
      </div>
    </div>
  </div>
</div>
<script>
const WS_URL='ws://'+location.hostname+':81';
let ws,retry;
const LIDAR_MAX_CM=800, US_BAR_MAX_CM=160;
const B=[{k:'F',i:'dUF'},{k:'B',i:'dUB'},{k:'L',i:'dUL'},{k:'P',i:'dUR'}];
function buildBump(){
  document.getElementById('bumpBox').innerHTML=B.map(b=>`
    <div class="b-item">
      <div class="bn">US ${b.k==='L'?'Trai':b.k==='P'?'Phai':b.k==='F'?'Truoc':'Sau'}</div>
      <div class="val" id="val_${b.i}">-- cm</div>
      <div class="b-bar"><div class="b-fill" id="bar_${b.i}" style="width:0%"></div></div>
    </div>`).join('');
}
function setRing(id,cm){
  const el=document.getElementById(id);
  if(!el)return;
  const p=Math.max(0,Math.min(1,cm/LIDAR_MAX_CM));
  el.style.setProperty('--p',p);
}
function onL(num,id){
  const e=document.getElementById(id);
  if(e)e.textContent=Math.round(num);
}
function onU(cm,max,id){
  const v=document.getElementById('val_'+id);
  const f=document.getElementById('bar_'+id);
  if(!v||!f)return;
  v.textContent=cm+' cm';
  const w=Math.min(100,cm/max*100);
  f.style.width=w+'%';
  let c=cm<20? '#ef4444' : (cm<100? '#f59e0b' : '#22c55e');
  f.style.background=c;
}
function connectWS(){
  ws=new WebSocket(WS_URL);
  ws.onopen=()=>{document.getElementById('wsStatus').textContent='đã nối'; if(retry)clearTimeout(retry);};
  ws.onclose=()=>{document.getElementById('wsStatus').textContent='mất kết nối…'; retry=setTimeout(connectWS,2000);};
  ws.onmessage=e=>{
    try{
      const d=JSON.parse(e.data);
      const lf=d.lf??0, lb=d.lb??0;
      onL(lf,'vLF'); onL(lb,'vLB');
      setRing('ringF',lf); setRing('ringB',lb);
      onU(d.uf??0,US_BAR_MAX_CM,'dUF'); onU(d.ub??0,US_BAR_MAX_CM,'dUB');
      onU(d.ul??0,US_BAR_MAX_CM,'dUL'); onU(d.ur??0,US_BAR_MAX_CM,'dUR');
      document.getElementById('rFL').textContent=Math.round(d.rFL??0);
      document.getElementById('rFR').textContent=Math.round(d.rFR??0);
      document.getElementById('rRL').textContent=Math.round(d.rRL??0);
      document.getElementById('rRR').textContent=Math.round(d.rRR??0);
      const da=((d.dFL??0)+(d.dRL??0)+(d.dFR??0)+(d.dRR??0))/4;
      document.getElementById('distAvg').textContent=da.toFixed(2);
      document.getElementById('modeLabel').textContent=(d.mode===1)?'Tự hành (demo)':'Lái tay';
      const eb=document.getElementById('eBadge');
      if(d.estop) eb.classList.add('on'); else eb.classList.remove('on');
    }catch(x){}
  };
}
const zone=document.getElementById('jsZone');
const knob=document.getElementById('jsKnob');
const R=70;
let drag=false;
function jUp(cx,cy){
  const r=zone.getBoundingClientRect();
  let ox=cx-r.left-r.width/2, oy=cy-r.top-r.height/2;
  const dist=Math.sqrt(ox*ox+oy*oy);
  if(dist>R){ ox=ox*R/dist; oy=oy*R/dist; }
  const jX=Math.round(ox/R*100), jY=Math.round(-oy/R*100);
  knob.style.left=(r.width/2+ox)+'px';
  knob.style.top=(r.height/2+oy)+'px';
  wsS({t:'joy',x:jX,y:jY});
}
function jRel(){ drag=false; knob.style.left='50%'; knob.style.top='50%'; wsS({t:'joy',x:0,y:0}); }
function wsS(o){ if(ws && ws.readyState===1) ws.send(JSON.stringify(o)); }
function sendSpeed(v){ document.getElementById('spdVal').textContent=v; wsS({t:'spd',v:parseInt(v,10)}); }
function setMode(m){
  document.getElementById('btnManual').classList.toggle('active',m===0);
  document.getElementById('btnAuto').classList.toggle('active',m===1);
  wsS({t:'mode',m});
}
function sendEstop(){ wsS({t:'estop'}); }
function odomReset(){ wsS({t:'odomReset'}); }
zone.addEventListener('mousedown',e=>{drag=true; jUp(e.clientX,e.clientY);});
zone.addEventListener('mousemove',e=>{if(drag) jUp(e.clientX,e.clientY);});
zone.addEventListener('mouseup',jRel);
zone.addEventListener('mouseleave',jRel);
zone.addEventListener('touchstart',e=>{e.preventDefault(); drag=true; jUp(e.touches[0].clientX,e.touches[0].clientY);},{passive:false});
zone.addEventListener('touchmove',e=>{e.preventDefault(); if(drag) jUp(e.touches[0].clientX,e.touches[0].clientY);},{passive:false});
zone.addEventListener('touchend',jRel);
buildBump();
connectWS();
</script>
</body>
</html>
)rawhtml";

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

inline void webUIInit() {
  g_prefs.begin(NVS_NAMESPACE, true);
  g_state.baseSpeed = g_prefs.getUInt("baseSpeed", PWM_MAX * 60 / 100);
  g_prefs.end();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[WiFi] SoftAP '%s'  IP: %s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());
  g_httpServer.on("/", HTTP_GET, []() {
    g_httpServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    g_httpServer.sendHeader("Pragma", "no-cache");
    g_httpServer.send_P(200, "text/html; charset=utf-8", HTML_PAGE);
  });
  g_httpServer.begin();
  g_wsServer.begin();
  g_wsServer.onEvent(onWebSocketEvent);
  Serial.printf("[WS] WebSocket server trên port %d\n", WS_PORT);
}

inline void webUILoop() {
  g_httpServer.handleClient();
  g_wsServer.loop();
}

#endif
