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
#include <cstdio>
#include "esp_heap_caps.h"

/** Đọc nhiệt độ chip (°C). Trả về NAN nếu không đọc được. */
inline float readChipTempCelsius() {
  float t = temperatureRead();
  if (t < -40.f || t > 125.f) return NAN;
  return t;
}

/** 0 = OK, 1 = cảnh báo, 2 = nghiêm trọng (nhiệt + heap SRAM nội bộ). */
inline int computeHealthLevel(float tempC, uint32_t heapIntFree) {
  if (heapIntFree < 20000u) return 2;
  if (tempC == tempC && tempC >= 90.f) return 2;
  if (heapIntFree < 45000u) return 1;
  if (tempC == tempC && tempC >= 80.f) return 1;
  return 0;
}

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
  --gap:12px;--nav-h:48px;
  --safe-t:max(12px, env(safe-area-inset-top, 0px));
  --safe-b:max(16px, env(safe-area-inset-bottom, 0px));
  --safe-x:max(14px, env(safe-area-inset-left, 0px), env(safe-area-inset-right, 0px));
}
@media (prefers-reduced-motion:reduce){
  *,*::before,*::after{animation-duration:.01ms!important;animation-iteration-count:1!important;transition-duration:.01ms!important}
}
html{scroll-behavior:smooth;-webkit-text-size-adjust:100%}
*{box-sizing:border-box;margin:0;padding:0}
body{
  min-height:100dvh;background:radial-gradient(1200px 600px at 50% -20%,#12202e 0%,var(--bg0) 55%,#050608 100%);
  color:var(--text);font-family:system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
  padding:var(--safe-t) var(--safe-x) var(--safe-b);
}
.wrap{max-width:1100px;margin:0 auto}
.brand{
  text-align:center;padding:clamp(14px,3vw,20px) 8px clamp(16px,3vw,22px);
  background:linear-gradient(180deg,rgba(17,24,32,.6),transparent);border-bottom:1px solid var(--line);border-radius:0 0 20px 20px;
  box-shadow:var(--glow);
}
.brand h1{
  font-size:clamp(1.2rem,3.8vw,1.65rem);font-weight:700;letter-spacing:.12em;
  background:linear-gradient(90deg,var(--accent),var(--accent2),#a78bfa);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text;
}
.brand .desc{margin-top:6px;font-size:clamp(.72rem,2.8vw,.8rem);color:var(--muted);line-height:1.5;max-width:36rem;margin-left:auto;margin-right:auto}
.pillrow{display:flex;flex-wrap:wrap;gap:6px;justify-content:center;margin-top:10px}
.pill{
  font-size:.65rem;letter-spacing:.04em;padding:5px 11px;border-radius:99px;
  background:rgba(45,212,191,.1);color:var(--accent);border:1px solid rgba(45,212,191,.25);
}
.pill.safety{background:rgba(245,158,11,.1);color:var(--amber);border-color:rgba(245,158,11,.3)}
.secnav{
  position:sticky;top:0;z-index:50;display:flex;gap:6px;justify-content:center;flex-wrap:wrap;
  padding:10px 4px 12px;margin-top:4px;
  background:linear-gradient(180deg,rgba(8,11,15,.92) 60%,rgba(8,11,15,.75) 100%);
  backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);
  border-bottom:1px solid rgba(30,40,54,.6);
}
.secnav a{
  text-decoration:none;font-size:.68rem;font-weight:600;letter-spacing:.06em;text-transform:uppercase;
  padding:10px 14px;border-radius:999px;border:1px solid var(--line2);color:var(--muted);background:rgba(17,24,32,.85);
  -webkit-tap-highlight-color:transparent;touch-action:manipulation;transition:color .15s,border-color .15s,background .15s;
  min-height:44px;display:inline-flex;align-items:center;justify-content:center;
}
.secnav a:hover,.secnav a:focus-visible{color:var(--text);border-color:rgba(45,212,191,.45);outline:none}
.secnav a:focus-visible{box-shadow:0 0 0 2px rgba(56,189,248,.5)}
.secnav a.active{color:var(--accent);border-color:rgba(45,212,191,.45);background:rgba(45,212,191,.1)}
.appgrid{
  display:grid;gap:var(--gap);margin-top:6px;
  grid-template-columns:1fr;
  grid-template-areas:"drive" "sense" "monitor";
}
.col{scroll-margin-top:calc(var(--nav-h) + 8px);display:flex;flex-direction:column;gap:var(--gap)}
.col-sense{grid-area:sense}
.col-drive{grid-area:drive}
.col-monitor{grid-area:monitor}
.rail-title{
  font-size:.62rem;font-weight:700;text-transform:uppercase;letter-spacing:.16em;color:var(--muted);
  margin:2px 0 0 2px;opacity:.92;
}
@media (min-width:900px){
  .appgrid{grid-template-columns:1.05fr minmax(272px,.95fr) 1fr;grid-template-areas:"sense drive monitor";align-items:start}
}
@media (orientation:landscape) and (max-height:520px) and (max-width:899px){
  .appgrid{grid-template-columns:1fr 1fr;grid-template-areas:"sense drive" "monitor monitor"}
  .brand .desc{display:none}
  .brand{padding:10px 8px 12px}
  .pillrow{display:none}
  .lidar-in{min-height:140px;padding:12px 10px}
  .lidar-arc{width:clamp(96px,26vw,118px);height:clamp(96px,26vw,118px)}
  .lidar-num{font-size:clamp(1.35rem,4vw,1.65rem)}
  .slam{min-height:88px;padding:12px;gap:6px}
  .slam p{display:none}
  .secnav{padding:6px 4px 8px}
  .bump-grid{grid-template-columns:repeat(4,1fr);gap:6px}
  .b-item{padding:6px 8px}
  .b-item .val{font-size:.9rem}
}
@media (orientation:landscape) and (max-height:400px) and (max-width:899px){
  .rail-title{display:none}
  .card{padding:12px 12px 14px}
}
.card{
  background:linear-gradient(160deg,rgba(20,28,40,.9),rgba(8,12,18,.95));
  border:1px solid var(--line);border-radius:16px;padding:clamp(14px,3.5vw,18px);box-shadow:0 4px 24px rgba(0,0,0,.35);
}
.card h2{
  font-size:.7rem;font-weight:600;text-transform:uppercase;letter-spacing:.12em;color:var(--muted);margin-bottom:12px;
  display:flex;align-items:center;gap:8px;line-height:1.25
}
h2 .dot{width:6px;height:6px;border-radius:50%;background:var(--accent);box-shadow:0 0 8px var(--accent);flex-shrink:0}
h2 .dot.safety{background:var(--amber);box-shadow:0 0 8px var(--amber)}
.lidar2{display:grid;grid-template-columns:1fr 1fr;gap:12px}
@media (max-width:520px) and (orientation:portrait){.lidar2{grid-template-columns:1fr}}
.lidar{
  position:relative;border-radius:20px;padding:4px;
  background:conic-gradient(from 200deg,rgba(45,212,191,.4),rgba(56,189,248,.2),rgba(167,139,250,.3),rgba(45,212,191,.2));
  box-shadow:0 0 0 1px rgba(255,255,255,.06) inset;
}
.lidar-in{
  background:var(--card);border-radius:16px;padding:16px 12px 14px;
  min-height:clamp(158px,36vw,200px);display:flex;flex-direction:column;align-items:center;justify-content:center;
  border:1px solid var(--line2);
}
.lidar-arc{
  --p:.35;width:clamp(104px,32vw,126px);height:clamp(104px,32vw,126px);border-radius:50%;aspect-ratio:1;
  background:conic-gradient(var(--accent) calc(var(--p) * 1turn),#1a2330 0);
  padding:4px;
}
.lidar-arc>div{
  width:100%;height:100%;border-radius:50%;background:var(--bg0);
  display:flex;flex-direction:column;align-items:center;justify-content:center;
}
.lidar-num{
  font-family:ui-monospace,"Cascadia Mono","Segoe UI Mono",Consolas,monospace;font-size:clamp(1.45rem,5vw,1.85rem);font-weight:700;letter-spacing:-.02em;
  line-height:1.1;
}
.lidar-unit{font-size:.7rem;color:var(--muted);font-weight:500}
.lidar-ax{font-size:.65rem;font-weight:600;letter-spacing:.1em;margin-top:10px;color:var(--accent2);text-align:center}
.lidar-ax b{color:var(--text);font-weight:600}
.bump-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.b-item{
  background:#0c1016;border:1px solid var(--line);border-radius:10px;padding:10px 10px;
  border-left:3px solid var(--amber);
}
.b-item .bn{font-size:.62rem;color:var(--muted);text-transform:uppercase;letter-spacing:.06em}
.b-item .val{font-family:ui-monospace,Consolas,monospace;font-size:.95rem;font-weight:600;margin-top:3px}
.b-bar{height:6px;border-radius:3px;background:#1a2330;margin-top:8px;overflow:hidden}
.b-fill{height:100%;border-radius:3px;transition:width .2s,background .15s}
.slam{
  min-height:112px;border:1px dashed var(--line2);border-radius:12px;
  display:flex;flex-direction:column;align-items:center;justify-content:center;gap:8px;padding:16px;
  background:repeating-linear-gradient(-12deg,transparent,transparent 8px,rgba(255,255,255,.02) 8px,rgba(255,255,255,.02) 9px);
}
.slam h3{font-size:.76rem;font-weight:600;color:var(--muted);text-align:center;max-width:22rem;line-height:1.35}
.slam p{font-size:.68rem;color:var(--muted);text-align:center;opacity:.88;max-width:26rem;line-height:1.45}
.health-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px 12px;font-size:.76rem;color:var(--muted)}
@media (min-width:380px){.health-grid.cols-3{grid-template-columns:repeat(3,1fr)}}
.health-grid b{color:var(--accent2);font-weight:600;word-break:break-word}
.h-ok{color:var(--ok)!important}.h-war{color:var(--war)!important}.h-bad{color:var(--bad)!important}
#hiLine{margin-top:10px;font-size:.74rem;line-height:1.35;padding:8px 10px;border-radius:8px;background:#0c1016;border:1px solid var(--line2)}
.ctrl-stack{display:flex;flex-direction:column;align-items:stretch;gap:12px}
#jsZone{
  width:min(172px,44vw);height:min(172px,44vw);max-width:188px;max-height:188px;border-radius:50%;
  background:radial-gradient(circle at 40% 35%,#1a2638,#0a0d12 70%);
  border:2px solid var(--line2);position:relative;touch-action:none;margin:0 auto;cursor:crosshair;box-shadow:0 0 0 1px rgba(0,0,0,.4) inset;
  flex-shrink:0;
}
#jsKnob{
  width:clamp(44px,30%,56px);height:clamp(44px,30%,56px);border-radius:50%;
  background:linear-gradient(160deg,#5eead4,#22d3ee);
  position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);
  box-shadow:0 4px 20px rgba(45,212,191,.4);pointer-events:none;transition:top .04s linear,left .04s linear;
}
.range-wrap{margin-top:4px;padding:8px 0;min-height:48px;display:flex;align-items:center}
input[type=range]{width:100%;accent-color:var(--accent);height:8px;-webkit-appearance:none;appearance:none;background:transparent}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;border-radius:50%;background:var(--accent);cursor:pointer;border:2px solid var(--bg0);box-shadow:0 2px 8px rgba(0,0,0,.35)}
input[type=range]::-moz-range-thumb{width:22px;height:22px;border-radius:50%;background:var(--accent);cursor:pointer;border:2px solid var(--bg0)}
.toggle-row{display:flex;align-items:stretch;gap:10px;justify-content:center;margin-top:4px;flex-wrap:wrap}
.mode-btn{
  flex:1;min-width:min(100%,140px);min-height:46px;padding:10px 16px;border:none;border-radius:12px;font-size:.78rem;cursor:pointer;font-weight:600;
  font-family:inherit;transition:transform .1s,box-shadow .15s,background .15s;
  -webkit-tap-highlight-color:transparent;touch-action:manipulation;
}
.mode-btn:hover{transform:translateY(-1px)}
.mode-btn.active{background:linear-gradient(135deg,#2dd4bf,#22d3ee);color:#041016;box-shadow:0 4px 16px rgba(34,211,238,.3)}
.mode-btn:not(.active){background:var(--line);color:var(--text);border:1px solid var(--line2)}
.estop{
  width:100%;padding:14px 16px;border:none;border-radius:12px;margin-top:4px;cursor:pointer;font-weight:700;letter-spacing:.04em;
  font-family:inherit;font-size:.95rem;
  background:linear-gradient(135deg,#f87171,#ef4444);color:#1a0505;box-shadow:0 4px 20px rgba(239,68,68,.3);
  min-height:52px;-webkit-tap-highlight-color:transparent;touch-action:manipulation;
}
.btn-ghost{
  margin-top:8px;padding:10px 16px;border:1px solid var(--line2);background:rgba(12,16,22,.6);color:var(--text);
  border-radius:10px;cursor:pointer;font-size:.78rem;font-family:inherit;width:100%;min-height:44px;
  -webkit-tap-highlight-color:transparent;touch-action:manipulation;
}
.btn-ghost:active{background:var(--line)}
.rpm-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.rpm-item{background:#0c1016;border:1px solid var(--line);border-radius:10px;padding:11px 8px;text-align:center}
.rpm-val{font-family:ui-monospace,Consolas,monospace;font-size:clamp(1rem,3.5vw,1.2rem);font-weight:700;color:var(--accent2)}
.rpm-lbl{font-size:.6rem;color:var(--muted);text-transform:uppercase;letter-spacing:.06em;margin-top:3px}
.status-line{font-size:.8rem;margin-top:6px;display:flex;align-items:center;gap:8px;flex-wrap:wrap}
.sdot{width:8px;height:8px;border-radius:50%;background:var(--ok);box-shadow:0 0 8px var(--ok);animation:bl 1.2s ease infinite;flex-shrink:0}
@keyframes bl{0%,100%{opacity:1}50%{opacity:.25}}
.badge-estop{
  display:none;padding:4px 9px;border-radius:6px;font-size:.65rem;font-weight:700;
  background:rgba(239,68,68,.2);color:#fecaca;border:1px solid rgba(239,68,68,.4);
}
.badge-estop.on{display:inline}
.stat-row{font-size:.8rem;margin-top:10px;color:var(--muted);line-height:1.45}
.stat-row b{color:var(--text);font-weight:600}
.stat-row+.stat-row{margin-top:6px}
#spark{display:block;width:100%;max-width:100%;height:52px;border-radius:8px;border:1px solid var(--line);margin-top:10px;background:#0c1016}
details{font-size:.72rem;color:var(--muted)}
details summary{cursor:pointer;user-select:none;color:var(--accent2);padding:6px 0;min-height:44px;display:flex;align-items:center;-webkit-tap-highlight-color:transparent}
details pre{
  margin-top:6px;padding:10px;background:#0c1016;border:1px solid var(--line);border-radius:8px;
  overflow:auto;max-height:min(42vh,280px);font-size:.62rem;color:var(--text);white-space:pre-wrap;word-break:break-word;
  -webkit-overflow-scrolling:touch;
}
.hint{font-size:.68rem;color:var(--muted);margin-bottom:10px;line-height:1.45;opacity:.92}
.spd-label{text-align:center;font-size:.76rem;color:var(--muted);margin-top:4px}
.health-grid .mac-row{grid-column:1/-1}
.details-block{margin-top:4px}
.mode-manual{color:var(--accent2)!important;font-weight:600}
.mode-auto{color:var(--amber)!important;font-weight:600}
</style>
</head>
<body>
<div class="wrap">
  <header class="brand">
    <h1>SMARTMARKETBOT</h1>
    <p class="desc">IoT Edge HMI &mdash; lớp định vị chính: <strong>LiDAR</strong> (khoảng cách, nền tảng bản đồ / SLAM). <strong>HC-SR04</strong> chỉ phục vụ dừng cận cảnh, không dùng để lập bản đồ.</p>
    <div class="pillrow">
      <span class="pill">TF-Luna &middot; quét 2 hướng</span>
      <span class="pill safety">HC-SR04 &middot; vùng an toàn</span>
    </div>
  </header>

  <nav class="secnav" aria-label="Mục chính">
    <a href="#sec-sense">Cảm biến</a>
    <a href="#sec-drive">Điều khiển</a>
    <a href="#sec-monitor">Giám sát</a>
  </nav>

  <div class="appgrid">
    <div class="col col-sense" id="sec-sense">
      <p class="rail-title">Khoảng cách &amp; an toàn</p>
      <div class="card">
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

      <div class="card">
        <h2><span class="dot safety"></span> Bumper (HC-SR04) &mdash; dừng cận cảnh</h2>
        <p class="hint">Khi vật cản rất gần (&lt;~50&nbsp;cm): giảm tốc / dừng. Không thay thế LiDAR để lập bản đồ.</p>
        <div class="bump-grid" id="bumpBox"></div>
      </div>

      <div class="card">
        <h2><span class="dot"></span> Bản đồ / SLAM (giai đoạn tích hợp)</h2>
        <div class="slam">
          <h3>Khu vực sẽ hiển thị bản đồ &amp; tuyến đường khi tích hợp Back-end (MQTT, ROS2, …)</h3>
          <p>ESP32 đẩy scan / pose / odometry; server xử lý SLAM và lập kế hoạch đường đi.</p>
        </div>
      </div>
    </div>

    <div class="col col-drive" id="sec-drive">
      <p class="rail-title">Vận hành</p>
      <div class="card">
        <h2>Điều khiển</h2>
        <div class="ctrl-stack">
          <div id="jsZone"><div id="jsKnob"></div></div>
          <div class="toggle-row">
            <button type="button" class="mode-btn active" id="btnManual" onclick="setMode(0)">Lái tay</button>
            <button type="button" class="mode-btn" id="btnAuto" onclick="setMode(1)">Tự hành (demo)</button>
          </div>
          <div class="range-wrap">
            <input type="range" id="spdSlider" min="0" max="100" value="60" oninput="sendSpeed(this.value)" aria-label="Tốc độ nền phần trăm"/>
          </div>
          <div class="spd-label">Tốc độ: <span id="spdVal">60</span>%</div>
          <button type="button" class="estop" onclick="sendEstop()">DỪNG KHẨN CẤP</button>
        </div>
      </div>

      <div class="card">
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
          <span class="sdot" aria-hidden="true"></span><span>WS</span> <span id="wsStatus">...</span>
          <span class="badge-estop" id="eBadge">E-STOP</span>
        </p>
        <p class="stat-row">Quãng đường TB: <b id="distAvg">0.00</b> m</p>
        <p class="stat-row">Chế độ: <b id="modeLabel" class="mode-manual">Lái tay</b></p>
        <button type="button" class="btn-ghost" onclick="odomReset()">Reset odom</button>
      </div>
    </div>

    <div class="col col-monitor" id="sec-monitor">
      <p class="rail-title">Hệ thống</p>
      <div class="card">
        <h2><span class="dot"></span> ESP – nhiệt &amp; sức khỏe</h2>
        <div class="health-grid">
          <div>Nhiệt chip: <b id="hiTemp">--</b> °C</div>
          <div>CPU: <b id="hiCpu">--</b> MHz</div>
          <div>Heap (nội): <b id="hiHeap">--</b> KB</div>
          <div>PSRAM: <b id="hiPsram">--</b></div>
          <div>Uptime: <b id="hiUp">--</b></div>
          <div>Client Wi‑Fi: <b id="hiAp">--</b></div>
        </div>
        <div id="hiLine"><span id="hiMsg">Đang chờ dữ liệu…</span></div>
        <canvas id="spark" width="300" height="52" aria-hidden="true"></canvas>
      </div>

      <div class="card">
        <h2><span class="dot"></span> Giám sát sâu</h2>
        <div class="health-grid cols-3">
          <div>Chip: <b id="dvChip">--</b></div>
          <div>Flash: <b id="dvFlash">--</b></div>
          <div>Build: <b id="dvBuild">--</b></div>
          <div>Kênh AP: <b id="dvCh">--</b></div>
          <div class="mac-row">MAC AP: <b id="dvMac" style="font-size:.68rem;font-weight:500">--</b></div>
          <div>Heap min: <b id="dvHmin">--</b> KB</div>
          <div>Joy X / Y: <b id="dvJoy">--</b></div>
          <div>Tốc độ nền: <b id="dvSpd">--</b>%</div>
          <div>Tuổi LiDAR: <b id="dvLfAge">--</b></div>
          <div>Tuổi US: <b id="dvUsAge">--</b></div>
        </div>
        <details class="details-block">
          <summary>Payload JSON thô (debug)</summary>
          <pre id="rawJ">—</pre>
        </details>
      </div>
    </div>
  </div>
</div>
<script>
const WS_URL='ws://'+location.hostname+':81';
let ws,retry;
const LIDAR_MAX_CM=800, US_BAR_MAX_CM=160;
const B=[{k:'F',i:'dUF'},{k:'B',i:'dUB'},{k:'L',i:'dUL'},{k:'P',i:'dUR'}];
const spark=[]; const SPARK_N=48;
function fmtAge(ms){
  if(ms==null||ms<0)return 'chưa có dữ liệu';
  if(ms<1000)return ms+' ms';
  if(ms<60000)return (ms/1000).toFixed(1)+' s';
  const m=Math.floor(ms/60000), s=Math.floor((ms%60000)/1000);
  return m+' ph '+s+' s';
}
function pushSpark(v){
  if(!(v>=0)||!isFinite(v))return;
  spark.push(v); if(spark.length>SPARK_N)spark.shift();
  const c=document.getElementById('spark'); if(!c)return;
  const x=c.getContext('2d'), w=c.width, h=c.height;
  x.fillStyle='#0c1016'; x.fillRect(0,0,w,h);
  if(spark.length<2)return;
  let mn=Math.min(...spark), mx=Math.max(...spark);
  if(mx<=mn)mx=mn+1;
  const sc=t=>(h-4)-((t-mn)/(mx-mn))*(h-8);
  x.strokeStyle='#2dd4bf'; x.lineWidth=1.5; x.beginPath();
  spark.forEach((t,i)=>{
    const px=i/(spark.length-1)*(w-2)+1, py=sc(t);
    i?x.lineTo(px,py):x.moveTo(px,py);
  });
  x.stroke();
}
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
      const ml=document.getElementById('modeLabel');
      ml.textContent=(d.mode===1)?'Tự hành (demo)':'Lái tay';
      ml.className=(d.mode===1)?'mode-auto':'mode-manual';
      const eb=document.getElementById('eBadge');
      if(d.estop) eb.classList.add('on'); else eb.classList.remove('on');
      if(d.tempC!=null && d.tempC>=0){
        const te=document.getElementById('hiTemp');
        te.textContent=(Math.round(d.tempC*10)/10).toFixed(1);
        te.className= d.tempC>=80?'h-bad':(d.tempC>=70?'h-war':'h-ok');
      }else{
        const te=document.getElementById('hiTemp');
        te.textContent='N/A'; te.className='';
      }
      if(d.cpuMHz!=null) document.getElementById('hiCpu').textContent=d.cpuMHz;
      if(d.heapIn!=null) document.getElementById('hiHeap').textContent=(d.heapIn/1024).toFixed(1);
      const pt=d.psTot??0, pf=d.psFree??0;
      document.getElementById('hiPsram').textContent= pt>0
        ? ( (pf/1048576).toFixed(2)+' / '+(pt/1048576).toFixed(2)+' MB' )
        : 'Tắt';
      const ms=d.upMs??0, s=Math.floor(ms/1000), m=Math.floor(s/60), h=Math.floor(m/60);
      document.getElementById('hiUp').textContent= h>0? (h+'h '+(m%60)+'m') : (m>0? (m+'m '+(s%60)+'s') : (s+'s'));
      document.getElementById('hiAp').textContent= d.apCli??0;
      const hl=document.getElementById('hiLine'), hm=document.getElementById('hiMsg');
      hl.className=''; hm.className='';
      if(d.health===2){
        hl.style.borderColor='#ef4444'; hm.textContent='Cảnh báo: nhiệt cao hoặc RAM nội bộ thấp — giảm tải / tản nhiệt.';
        hm.className='h-bad';
      }else if(d.health===1){
        hl.style.borderColor='#f59e0b'; hm.textContent='Chú ý: theo dõi nhiệt hoặc heap.';
        hm.className='h-war';
      }else{
        hl.style.borderColor='var(--line2)'; hm.textContent='Trạng thái: bình thường.';
        hm.className='h-ok';
      }
      if(d.tempC!=null && d.tempC>=0) pushSpark(d.tempC);
      if(d.chip!=null) document.getElementById('dvChip').textContent=String(d.chip);
      if(d.flashKB!=null) document.getElementById('dvFlash').textContent=d.flashKB+' KB';
      if(d.build!=null) document.getElementById('dvBuild').textContent=String(d.build);
      if(d.mac!=null) document.getElementById('dvMac').textContent=String(d.mac);
      if(d.ch!=null) document.getElementById('dvCh').textContent=String(d.ch);
      if(d.hMin!=null) document.getElementById('dvHmin').textContent=(d.hMin/1024).toFixed(1);
      document.getElementById('dvJoy').textContent=(d.cx??0)+' / '+(d.cy??0);
      if(d.spdPct!=null) document.getElementById('dvSpd').textContent=String(d.spdPct);
      document.getElementById('dvLfAge').textContent=fmtAge(d.lfAge);
      document.getElementById('dvUsAge').textContent=fmtAge(d.usAge);
      if(d.spdPct!=null){
        const sl=document.getElementById('spdSlider');
        if(sl && document.activeElement!==sl){
          sl.value=d.spdPct;
          document.getElementById('spdVal').textContent=String(d.spdPct);
        }
      }
      document.getElementById('rawJ').textContent=JSON.stringify(d,null,2);
    }catch(x){}
  };
}
const zone=document.getElementById('jsZone');
const knob=document.getElementById('jsKnob');
let drag=false;
function jUp(cx,cy){
  const r=zone.getBoundingClientRect();
  const R=Math.max(24, Math.min(r.width,r.height)/2 - 10);
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
  const ml=document.getElementById('modeLabel');
  ml.textContent=m===1?'Tự hành (demo)':'Lái tay';
  ml.className=m===1?'mode-auto':'mode-manual';
  wsS({t:'mode',m});
}
function sendEstop(){ wsS({t:'estop'}); }
function odomReset(){ wsS({t:'odomReset'}); }
function initSecNav(){
  const links=[...document.querySelectorAll('.secnav a')];
  const secs=['sec-sense','sec-drive','sec-monitor'].map(id=>document.getElementById(id)).filter(Boolean);
  if(!links.length||!secs.length)return;
  const setActive=id=>{
    links.forEach(a=>a.classList.toggle('active',a.getAttribute('href')==='#'+id));
  };
  if('IntersectionObserver' in window){
    const io=new IntersectionObserver((ents)=>{
      const vis=ents.filter(e=>e.isIntersecting&&e.intersectionRatio>0.06);
      if(!vis.length)return;
      vis.sort((a,b)=>b.intersectionRatio-a.intersectionRatio);
      setActive(vis[0].target.id);
    },{threshold:[0,0.06,0.12,0.22,0.38],rootMargin:'-10% 0px -50% 0px'});
    secs.forEach(s=>io.observe(s));
  }else{
    links[0].classList.add('active');
  }
  links.forEach(a=>{
    a.addEventListener('click',()=>{
      const id=a.getAttribute('href').slice(1);
      requestAnimationFrame(()=>setActive(id));
    });
  });
}
zone.addEventListener('mousedown',e=>{drag=true; jUp(e.clientX,e.clientY);});
zone.addEventListener('mousemove',e=>{if(drag) jUp(e.clientX,e.clientY);});
zone.addEventListener('mouseup',jRel);
zone.addEventListener('mouseleave',jRel);
zone.addEventListener('touchstart',e=>{e.preventDefault(); drag=true; jUp(e.touches[0].clientX,e.touches[0].clientY);},{passive:false});
zone.addEventListener('touchmove',e=>{e.preventDefault(); if(drag) jUp(e.touches[0].clientX,e.touches[0].clientY);},{passive:false});
zone.addEventListener('touchend',jRel);
initSecNav();
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

  float tC = readChipTempCelsius();
  if (tC == tC && tC >= -40.f && tC <= 125.f) {
    doc["tempC"] = (double)((int)(tC * 10.f + 0.5f)) / 10.0;
  } else {
    doc["tempC"] = -1.0;
  }
  uint32_t heapInt = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  doc["heap"]   = (uint32_t)ESP.getFreeHeap();
  doc["heapIn"] = heapInt;
  if (psramFound()) {
    doc["psFree"] = (uint32_t)ESP.getFreePsram();
    doc["psTot"]  = (uint32_t)ESP.getPsramSize();
  } else {
    doc["psFree"] = 0;
    doc["psTot"]  = 0;
  }
  doc["upMs"]  = (uint32_t)millis();
  doc["apCli"] = WiFi.softAPgetStationNum();
  doc["cpuMHz"] = ESP.getCpuFreqMHz();
  doc["health"] = computeHealthLevel(tC, heapInt);

  doc["chip"]    = ESP.getChipModel();
  doc["flashKB"] = (uint32_t)(ESP.getFlashChipSize() / 1024u);
  char buildBuf[48];
  snprintf(buildBuf, sizeof(buildBuf), "%s %s", __DATE__, __TIME__);
  doc["build"] = buildBuf;
  doc["mac"]   = WiFi.softAPmacAddress();
  doc["ch"]    = (int)WiFi.channel();

  doc["hMin"] = (uint32_t)heap_caps_get_minimum_free_size(
      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  doc["cx"] = (int)g_state.cmdX;
  doc["cy"] = (int)g_state.cmdY;
  uint32_t spdPct =
      (g_state.baseSpeed * 100u) / (uint32_t)(PWM_MAX ? PWM_MAX : 1u);
  doc["spdPct"] = spdPct;

  const uint32_t nowMs = (uint32_t)millis();
  if (g_state.lidarLastUpdateMs == 0u) {
    doc["lfAge"] = -1;
  } else {
    uint32_t age = nowMs - g_state.lidarLastUpdateMs;
    doc["lfAge"] = (int32_t)(age > 86400000u ? 86400000 : age);
  }
  if (g_state.usLastUpdateMs == 0u) {
    doc["usAge"] = -1;
  } else {
    uint32_t ageU = nowMs - g_state.usLastUpdateMs;
    doc["usAge"] = (int32_t)(ageU > 86400000u ? 86400000 : ageU);
  }

  char buf[1200];
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

#endif // WEBUI_H
