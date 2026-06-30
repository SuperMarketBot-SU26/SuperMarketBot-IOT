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
#include <cstring>
#include <esp_wifi.h>
#include "RobotTelemetry.h"
#include "SensorLayout.h"
#include "MotorLayout.h"

Preferences g_prefs;
#include "CtrlJson.h"
#include "VisionTablet.h"
#include "VisionHttps.h"
#include "MqttClient.h"

static WebServer      g_httpServer(WEB_PORT);
static WebSocketsServer g_wsServer(WS_PORT);

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
  body{background-attachment:scroll!important}
}
html{scroll-behavior:smooth;-webkit-text-size-adjust:100%}
*{box-sizing:border-box;margin:0;padding:0}
body{
  min-height:100dvh;color:var(--text);font-family:system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
  padding:var(--safe-t) var(--safe-x) var(--safe-b);
  background-color:#050608;
  background-image:
    radial-gradient(1200px 600px at 50% -15%,rgba(18,45,58,.85) 0%,transparent 58%),
    radial-gradient(700px 480px at 80% 95%,rgba(45,212,191,.07) 0%,transparent 55%),
    url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 1440 900'%3E%3Cdefs%3E%3ClinearGradient id='a' x1='0' y1='0' x2='0' y2='1'%3E%3Cstop stop-color='%230c1826'/%3E%3Cstop offset='1' stop-color='%23050608'/%3E%3C/linearGradient%3E%3Cpattern id='p' width='56' height='56' patternUnits='userSpaceOnUse'%3E%3Cpath d='M56 0H0V56' fill='none' stroke='%232a3545' stroke-width='0.7' opacity='.4'/%3E%3C/pattern%3E%3C/defs%3E%3Crect width='100%25' height='100%25' fill='url(%23a)'/%3E%3Crect width='100%25' height='100%25' fill='url(%23p)'/%3E%3Cg opacity='.11' fill='%232dd4bf' transform='translate%28720 520%29'%3E%3Crect x='-150' y='-78' width='300' height='156' rx='26'/%3E%3Ccircle cx='-108' cy='-108' r='30'/%3E%3Ccircle cx='108' cy='-108' r='30'/%3E%3Ccircle cx='-108' cy='108' r='30'/%3E%3Ccircle cx='108' cy='108' r='30'/%3E%3Cpath d='M0-118 L42-168 H-42Z'/%3E%3Cpath d='M-40-40 H40 M0-40 V40' stroke='%2338bdf8' stroke-width='6' stroke-linecap='round' opacity='.35' fill='none'/%3E%3C/g%3E%3C/svg%3E");
  background-size:auto,cover;
  background-position:center -80px,center bottom;
  background-repeat:no-repeat;
  background-attachment:fixed;
}
.wrap{position:relative;z-index:1;max-width:1100px;margin:0 auto}
.brand{
  text-align:center;padding:clamp(14px,3vw,20px) 8px clamp(16px,3vw,22px);
  background:linear-gradient(180deg,rgba(17,24,32,.72),transparent);border-bottom:1px solid var(--line);border-radius:0 0 20px 20px;
  box-shadow:var(--glow);backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);
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
.vision-wrap{position:relative;background:#060a10;border-radius:10px;border:1px solid var(--line2);aspect-ratio:4/3;overflow:hidden}
.vision-wrap video{width:100%;height:100%;object-fit:contain;display:block;background:#000}
.vision-tag{position:absolute;top:8px;left:8px;font-size:.68rem;font-weight:600;padding:4px 8px;border-radius:6px;background:rgba(0,0,0,.55);border:1px solid rgba(255,255,255,.1);color:var(--accent)}
.vision-ctl{display:flex;flex-wrap:wrap;gap:6px;margin-top:10px;align-items:center}
.vision-btn{font-size:.72rem;padding:6px 10px;border-radius:8px;border:1px solid var(--line2);background:#15202b;color:var(--text);cursor:pointer}
.vision-btn.on{border-color:var(--accent);color:var(--accent)}
.vision-sel{font-size:.72rem;padding:6px 8px;border-radius:8px;border:1px solid var(--line2);background:#15202b;color:var(--text)}
#vWrap{cursor:pointer}
#vWrap.live{cursor:default}
  .b-item{padding:6px 8px}
  .b-item .val{font-size:.9rem}
}
@media (orientation:landscape) and (max-height:400px) and (max-width:899px){
  .rail-title{display:none}
  .card{padding:12px 12px 14px}
}
.card{
  background:linear-gradient(160deg,rgba(20,28,40,.88),rgba(8,12,18,.93));
  border:1px solid var(--line);border-radius:16px;padding:clamp(14px,3.5vw,18px);box-shadow:0 4px 24px rgba(0,0,0,.35);
  backdrop-filter:blur(8px);-webkit-backdrop-filter:blur(8px);
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
.link-badge{font-size:.58rem;font-weight:700;letter-spacing:.08em;padding:3px 10px;border-radius:99px;border:1px solid var(--line2);margin-left:6px;vertical-align:middle;display:inline-block}
.link-badge.on{background:rgba(34,197,94,.14);color:var(--ok);border-color:rgba(34,197,94,.38)}
.link-badge.off{background:rgba(71,85,105,.22);color:#94a3b8;border-color:var(--line2)}
.lidar-in.sensor-off{opacity:.52;filter:grayscale(.92)}
.lidar-in.sensor-off .lidar-arc{opacity:.32}
.lidar-in.sensor-off .lidar-num{color:var(--muted)!important}
.bump-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.b-item{
  background:#0c1016;border:1px solid var(--line);border-radius:10px;padding:10px 10px;
  border-left:3px solid var(--amber);
}
.b-item .bn{font-size:.62rem;color:var(--muted);text-transform:uppercase;letter-spacing:.06em}
.b-item .val{font-family:ui-monospace,Consolas,monospace;font-size:.95rem;font-weight:600;margin-top:3px}
.b-item.sensor-off{opacity:.58;border-left-color:#475569}
.b-item.sensor-off .val{color:var(--muted)}
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
/* Thanh tốc độ: track đầy đủ + fill, không chỉ “chấm” */
.spd-block{
  margin:10px 0 0;padding:12px 14px 14px;background:rgba(12,16,22,.92);
  border:1px solid var(--line2);border-radius:14px;
  box-shadow:0 2px 12px rgba(0,0,0,.2),inset 0 1px 0 rgba(255,255,255,.04);
}
.spd-block:first-of-type{margin-top:6px}
.spd-block__head{display:flex;justify-content:space-between;align-items:center;gap:10px;margin-bottom:10px}
.spd-block__label{font-size:.68rem;font-weight:600;letter-spacing:.05em;text-transform:uppercase;color:var(--muted)}
.spd-block__hint{font-size:.62rem;color:var(--muted);opacity:.88;margin-top:2px;line-height:1.35}
.spd-block__badge{
  font-family:ui-monospace,"Segoe UI Mono",Consolas,monospace;font-weight:700;font-size:.88rem;
  min-width:3.2ch;text-align:right;padding:5px 11px;border-radius:10px;border:1px solid rgba(45,212,191,.35);
  background:linear-gradient(165deg,rgba(45,212,191,.18),rgba(8,14,20,.9));color:var(--accent);
  box-shadow:0 0 16px rgba(45,212,191,.08);
}
.spd-block--auto .spd-block__badge{
  border-color:rgba(251,191,36,.4);
  background:linear-gradient(165deg,rgba(251,191,36,.16),rgba(8,14,20,.9));color:var(--amber);
  box-shadow:0 0 16px rgba(251,191,36,.08);
}
.spd-block__ticks{
  display:flex;justify-content:space-between;margin-top:8px;padding:0 2px;
  font-size:.58rem;color:var(--muted);letter-spacing:.02em;
}
input.spd-range{
  --spd-pct:60%;
  --fill-a:var(--accent);--fill-b:var(--accent2);
  display:block;width:100%;height:36px;margin:0 -2px;cursor:pointer;background:transparent;
  -webkit-appearance:none;appearance:none;
}
input.spd-range--auto{--fill-a:#fbbf24;--fill-b:#38bdf8}
/* WebKit: vạch nền + phần đã chọn */
input.spd-range::-webkit-slider-runnable-track{
  height:14px;border-radius:7px;
  background:linear-gradient(90deg,var(--fill-a),var(--fill-b)) 0 / var(--spd-pct) 100% no-repeat #1a2330;
  box-shadow:inset 0 2px 6px rgba(0,0,0,.45),0 0 0 1px rgba(255,255,255,.05);
}
input.spd-range::-webkit-slider-thumb{
  -webkit-appearance:none;width:28px;height:28px;margin-top:-7px;border-radius:9px;box-sizing:border-box;
  background:linear-gradient(180deg,#f1f5f9,#cbd5e1);
  border:3px solid var(--fill-a);
  box-shadow:0 4px 14px rgba(45,212,191,.35),0 2px 4px rgba(0,0,0,.4);
}
.spd-block--auto input.spd-range::-webkit-slider-thumb{
  border-color:#fbbf24;box-shadow:0 4px 14px rgba(251,191,36,.35),0 2px 4px rgba(0,0,0,.4);
}
/* Firefox */
input.spd-range::-moz-range-track{
  height:14px;border-radius:7px;background:#1a2330;
  box-shadow:inset 0 2px 6px rgba(0,0,0,.45);
}
input.spd-range::-moz-range-progress{
  height:14px;border-radius:7px 0 0 7px;
  background:linear-gradient(90deg,var(--fill-a),var(--fill-b));
}
input.spd-range::-moz-range-thumb{
  width:26px;height:26px;border:none;border-radius:9px;box-sizing:border-box;
  background:linear-gradient(180deg,#f1f5f9,#cbd5e1);
  border:3px solid var(--fill-a);box-shadow:0 4px 12px rgba(0,0,0,.25);
}
.spd-block--auto input.spd-range::-moz-range-thumb{border-color:#fbbf24}
input[type=range]:not(.spd-range){width:100%;accent-color:var(--accent);height:8px;-webkit-appearance:none;appearance:none;background:transparent}
input[type=range]:not(.spd-range)::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;border-radius:50%;background:var(--accent);cursor:pointer;border:2px solid var(--bg0);box-shadow:0 2px 8px rgba(0,0,0,.35)}
input[type=range]:not(.spd-range)::-moz-range-thumb{width:22px;height:22px;border-radius:50%;background:var(--accent);cursor:pointer;border:2px solid var(--bg0)}
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
.rpm-item.sensor-off .rpm-val{color:#64748b!important;font-weight:600}
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
.health-grid .mac-row{grid-column:1/-1}
.details-block{margin-top:4px}
.layout-form{font-size:.74rem;color:var(--muted)}
.layout-row{display:grid;grid-template-columns:1.15fr 1fr 1fr;gap:8px;align-items:center;margin-bottom:8px}
@media(max-width:520px){.layout-row{grid-template-columns:1fr;gap:6px}}
.layout-row label{font-weight:600;color:var(--text);font-size:.7rem}
.layout-row select{width:100%;padding:8px 10px;border-radius:8px;border:1px solid var(--line2);background:#0c1016;color:var(--text);font-size:.72rem}
.layout-mot{display:grid;grid-template-columns:1.05fr 1.35fr 0.95fr;gap:8px;align-items:center;margin-bottom:8px}
@media(max-width:520px){.layout-mot{grid-template-columns:1fr;gap:6px}}
.layout-mot label{font-weight:600;color:var(--text);font-size:.7rem}
.pin-table{width:100%;font-size:.62rem;border-collapse:collapse;margin:12px 0;color:var(--muted)}
.pin-table th,.pin-table td{border:1px solid var(--line2);padding:7px 8px;text-align:left;vertical-align:top}
.pin-table th{background:#0c1016;color:var(--accent2);font-weight:600}
.pin-table code{font-family:ui-monospace,Consolas,monospace;font-size:.58rem;color:var(--text)}
.layout-lid{margin:12px 0;padding:10px;border:1px solid var(--line2);border-radius:10px;background:#0c1016}
.layout-lid label{display:block;margin-bottom:6px;font-size:.72rem;color:var(--muted)}
.mode-manual{color:var(--accent2)!important;font-weight:600}
.mode-auto{color:var(--amber)!important;font-weight:600}
</style>
</head>
<body>
<div class="wrap">
  <header class="brand">
    <h1>SMARTMARKETBOT</h1>
    <p class="desc">IoT Edge HMI &mdash; <strong>4× HC-SR04</strong> (trái/phải trước–sau): dừng &lt;30&nbsp;cm, tự hành lách theo bên trống. Vòng tròn lớn = min trước/sau; thanh 4 góc = từng cảm biến.</p>
    <div class="pillrow">
      <span class="pill">TF-Luna &middot; quét 2 hướng</span>
      <span class="pill safety">Né vật · LiDAR (+ SR04 tùy chọn)</span>
    </div>
  </header>

  <nav class="secnav" aria-label="Mục chính">
    <a href="#sec-vision">Camera</a>
    <a id="lnkVisionHttps" href="#">Camera HTTPS</a>
    <a href="#sec-sense">Cảm biến</a>
    <a href="#sec-drive">Điều khiển</a>
    <a href="#sec-monitor">Giám sát</a>
    <a href="#sec-layout">Bố trí cảm biến</a>
    <a href="#sec-mot-layout">Động cơ TB6612</a>
  </nav>

  <div class="appgrid">
    <div class="col col-sense" id="sec-sense">
      <p class="rail-title">Khoảng cách &amp; an toàn</p>
      <div class="card" id="sec-vision">
        <h2><span class="dot"></span> Camera tablet</h2>
        <p class="hint">Camera can <b>HTTPS</b>. Neu bao loi trinh duyet: bam <b>Camera HTTPS</b> tren menu (hoac <code id="vHttpsUrl">https://192.168.4.1/vision</code>), chap nhan chung chi tu ky, roi Bat camera.</p>
        <div class="vision-wrap" id="vWrap">
          <video id="tabCam" playsinline autoplay muted></video>
          <span class="vision-tag" id="vTag">Cham de bat</span>
        </div>
        <div class="vision-ctl">
          <button type="button" class="vision-btn" id="vStart" style="border-color:var(--accent);color:var(--accent)">Bat camera</button>
          <select class="vision-sel" id="vProf"><option value="0">QVGA</option><option value="1">HVGA</option><option value="2">VGA</option></select>
          <button type="button" class="vision-btn" id="vFh">Lat ngang</button>
          <button type="button" class="vision-btn" id="vSnap">Chup anh</button>
          <a class="vision-btn" id="vGoHttps" href="#" style="text-decoration:none;display:inline-block">Mo HTTPS</a>
        </div>
        <p class="hint" id="vMsg" style="margin-top:8px;min-height:1.2em"></p>
      </div>
      <div class="card">
        <h2><span class="dot"></span> LiDAR &mdash; tầm nhìn chính</h2>
        <p class="hint" style="margin-top:-6px">Nhãn <b>ON/OFF</b>: LiDAR <b>ON</b> khi có khung UART hợp lệ gần đây. Bumper bốn góc: <b>ON</b> khi có bản sao khoảng cách từ LiDAR (F/B) / hoặc echo SR04 nếu bật phần cứng. Encoder <b>ON</b> khi có xung gần đây. Số đứng ~800&nbsp;cm nhưng ON: xem bytes UART ở Giám sát.</p>
        <div class="lidar2">
          <div class="lidar">
            <div class="lidar-in sensor-off" id="lidCardF">
              <div class="lidar-arc" id="ringF" style="--p:0.2"><div>
                <span class="lidar-num" id="vLF">—</span>
                <span class="lidar-unit">cm</span>
              </div></div>
              <div class="lidar-ax">TRƯỚC <b id="lblTxtLF">US min</b> <span class="link-badge off" id="stLF">OFF</span></div>
            </div>
          </div>
          <div class="lidar">
            <div class="lidar-in sensor-off" id="lidCardB">
              <div class="lidar-arc" id="ringB" style="--p:0.2"><div>
                <span class="lidar-num" id="vLB">—</span>
                <span class="lidar-unit">cm</span>
              </div></div>
              <div class="lidar-ax">SAU · <b id="lblTxtLB">US min</b> <span class="link-badge off" id="stLB">OFF</span></div>
            </div>
          </div>
        </div>
      </div>

      <div class="card">
        <h2><span class="dot safety"></span> Bumper — bốn góc (bản sao LiDAR hoặc HC-SR04)</h2>
        <p class="hint">Trước/sau lấy từ TF-Luna (cùng số như trên cung LiDAR). Trái/phải: không có LiDAR ngang → hiển thị “xa” (<code>OFF</code> link). Có thể bật lại HC-SR04 thật trong <code>Config.h</code>. <b>OFF</b> khi chưa có frame hợp lệ / chưa nối dây.</p>
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
            <button type="button" class="mode-btn" id="btnRoute" onclick="setMode(2)">MQTT (Lộ trình)</button>
          </div>
          <div class="spd-block spd-block--manual">
            <div class="spd-block__head">
              <div>
                <div class="spd-block__label">Tốc độ · Lái tay</div>
                <div class="spd-block__hint">Áp dụng khi kéo joystick</div>
              </div>
              <span class="spd-block__badge" id="spdVal">60%</span>
            </div>
            <input type="range" class="spd-range spd-range--manual" id="spdSlider" min="0" max="100" value="60"
              oninput="sendSpeed(this.value)" aria-label="Tốc độ lái tay phần trăm"/>
            <div class="spd-block__ticks"><span>0%</span><span>50%</span><span>100%</span></div>
          </div>
          <div class="spd-block spd-block--manual">
            <div class="spd-block__head">
              <div>
                <div class="spd-block__label">Trượt ngang · Strafe</div>
                <div class="spd-block__hint">Mecanum: trái/phải không cần xoay</div>
              </div>
              <span class="spd-block__badge" id="strVal">50%</span>
            </div>
            <input type="range" class="spd-range spd-range--manual" id="strSlider" min="0" max="100" value="50"
              oninput="sendStrafe(this.value)" aria-label="Trượt ngang Mecanum"/>
            <div class="spd-block__ticks"><span>Trái</span><span>50%</span><span>Phải</span></div>
          </div>
          <div class="spd-block spd-block--auto">
            <div class="spd-block__head">
              <div>
                <div class="spd-block__label">Tốc độ · Tự hành</div>
                <div class="spd-block__hint">Demo né vật — LiDAR trước/sau (15–100%)</div>
              </div>
              <span class="spd-block__badge" id="spdAutoVal">60%</span>
            </div>
            <input type="range" class="spd-range spd-range--auto" id="spdAutoSlider" min="15" max="100" value="60"
              oninput="sendSpeedAuto(this.value)" aria-label="Tốc độ tự hành phần trăm"/>
            <div class="spd-block__ticks"><span>15%</span><span>50%</span><span>100%</span></div>
          </div>
          <div class="spd-block spd-block--auto">
            <div class="spd-block__head">
              <div>
                <div class="spd-block__label">Tốc độ · Tránh vật</div>
                <div class="spd-block__hint">Khi dạt chéo, đi lùi, xoay dò (15–100%)</div>
              </div>
              <span class="spd-block__badge" id="spdSwerveVal">40%</span>
            </div>
            <input type="range" class="spd-range spd-range--auto" id="spdSwerveSlider" min="15" max="100" value="40"
              oninput="sendSpeedSwerve(this.value)" aria-label="Tốc độ tránh vật phần trăm"/>
            <div class="spd-block__ticks"><span>15%</span><span>45%</span><span>100%</span></div>
          </div>
          <p class="hint" style="margin-top:8px;font-size:.68rem">Demo: <b>đi thẳng</b> → trước &lt; <code>AUTO_LIDAR_BLOCK_CM</code> thì <b>dừng</b> (<code>AUTO_STOP_HOLD_MS</code>) → <b>xoay quét</b> CW rồi CCW (<code>AUTO_SCAN_*_MS</code>) → lại <b>đi thẳng</b>. LiDAR sau vẫn đo trong lúc dừng. Không mắt ngang → không bẻ hông (trừ khi bật SR04).</p>
          <button type="button" class="estop" onclick="sendEstop()">DỪNG KHẨN CẤP</button>
        </div>
      </div>

      <div class="card">
        <h2>Bánh xe (RPM)</h2>
        <p class="hint" style="margin-top:-4px;margin-bottom:8px;font-size:.68rem"><b>OFF</b> khi vài giây gần đây <b>chưa có xung</b> encoder — lăn nhẹ bánh để kiểm.</p>
        <div class="rpm-grid">
          <div class="rpm-item sensor-off" id="rpmLF"><div class="rpm-val" id="rFL">OFF</div><div class="rpm-lbl">T.Trai</div></div>
          <div class="rpm-item sensor-off" id="rpmRF"><div class="rpm-val" id="rFR">OFF</div><div class="rpm-lbl">T.Phai</div></div>
          <div class="rpm-item sensor-off" id="rpmLR"><div class="rpm-val" id="rRL">OFF</div><div class="rpm-lbl">S.Trai</div></div>
          <div class="rpm-item sensor-off" id="rpmRR"><div class="rpm-val" id="rRR">OFF</div><div class="rpm-lbl">S.Phai</div></div>
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
          <div class="mac-row">Pin (ước lượng): <b id="hiBatV">Tắt</b> V · <b id="hiBatPct">—</b>%</div>
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
          <div>Tốc độ tay / auto: <b id="dvSpd">--</b>% / <b id="dvSpdAuto">--</b>%</div>
          <div>Tuổi LiDAR: <b id="dvLfAge">--</b></div>
          <div>Tuổi US: <b id="dvUsAge">--</b></div>
          <div class="mac-row">UART Luna (byte tích lũy): L1=<b id="dvLr1">0</b> · L2=<b id="dvLr2">0</b></div>
          <div class="mac-row hint" style="font-size:.72rem;line-height:1.5;color:var(--muted);margin-top:2px"><b>Tuổi LiDAR</b> chỉ đổi khi ESP nhận <b>đủ khung 9 byte</b> hợp lệ (<code>59&nbsp;59</code>… checksum đúng). Chữ <b>chưa có dữ liệu</b> nghĩa là chưa bao giờ parse được — kể cả khi đã nối dây. Hai số LiDAR ~800&nbsp;cm lúc đầu là <b>mặc định trong code</b>, không phải đo thật. <b>L1/L2</b> = byte đã vào UART: <b>0</b> → không có sóng serial (TX/RX, GND, 5&nbsp;V, chân Mode); <b>tăng</b> mà tuổi vẫn “chưa có” → thường Luna đang <b>I²C</b> (chân Mode kéo GND) hoặc baud/format lệch. Trong JSON có thêm <code>lfOn</code>/<code>lbOn</code>, mảng 4 phần tử <code>usOn</code>/<code>encOn</code> (1=tín hiệu gần đây).</div>
        </div>
        <details class="details-block">
          <summary>Payload JSON thô (debug)</summary>
          <pre id="rawJ">—</pre>
        </details>
      </div>

      <div class="card" id="sec-layout">
        <h2><span class="dot"></span> Bố trí cảm biến (team — không đổi dây GPIO)</h2>
        <p class="hint">Mỗi <b>góc xe</b> chọn <b>một</b> siêu âm vật lý và <b>một</b> encoder vật lý (0–3, không trùng trong từng loại). LiDAR: chọn UART nào là phía <b>trước</b> xe.</p>
        <div class="layout-form" id="layGrid"></div>
        <div class="layout-lid">
          <label>LiDAR phía <b>trước</b> xe đang là UART</label>
          <select id="lidFrontSel" aria-label="LiDAR trước">
            <option value="0">Serial1 — GPIO TX17 / RX18</option>
            <option value="1">Serial2 — GPIO TX1 / RX2</option>
          </select>
        </div>
        <button type="button" class="mode-btn" style="margin-top:8px;width:100%" onclick="saveLayout()">Lưu vào bộ nhớ robot (NVS)</button>
        <p class="hint" id="layMsg" style="margin-top:8px;min-height:1.2em"></p>
      </div>

      <div class="card" id="sec-mot-layout">
        <h2><span class="dot"></span> Bố trí động cơ (2× TB6612)</h2>
        <p class="hint">Chỉ một driver chạy 2 bánh: kiểm tra <b>VM</b>, <b>GND</b>, <b>STBY→GPIO47</b> cho <b>cả hai</b> IC. Bánh quay ngược khi tiến: thử <b>Đảo chiều</b>; lắp nhầm kênh A/B: dùng cột <b>Kênh driver thật</b> (mỗi giá trị 0–3 dùng đúng 1 lần).</p>
        <table class="pin-table" aria-label="Chân TB6612 tới ESP32">
          <thead><tr><th>IC / kênh</th><th>Góc (chuẩn)</th><th>GPIO ESP32-S3</th><th>Cực ra motor</th></tr></thead>
          <tbody>
            <tr><td>TB6612 <b>#1</b> A</td><td>Trái trước FL</td><td><code>PWM 4</code>, AIN1 <code>5</code>, AIN2 <code>6</code></td><td><code>AO1 – AO2</code></td></tr>
            <tr><td>TB6612 <b>#1</b> B</td><td>Trái sau RL</td><td><code>PWM 7</code>, BIN1 <code>8</code>, BIN2 <code>9</code></td><td><code>BO1 – BO2</code></td></tr>
            <tr><td>TB6612 <b>#2</b> A</td><td>Phải trước FR</td><td><code>PWM 21</code>, AIN1 <code>45</code>, AIN2 <code>46</code></td><td><code>AO1 – AO2</code></td></tr>
            <tr><td>TB6612 <b>#2</b> B</td><td>Phải sau RR</td><td><code>PWM 40</code>, BIN1 <code>41</code>, BIN2 <code>42</code></td><td><code>BO1 – BO2</code></td></tr>
            <tr><td colspan="2"><b>STBY</b> (chân STBY hai IC nối chung)</td><td colspan="2"><code>GPIO 47</code> mức HIGH = mở driver</td></tr>
          </tbody>
        </table>
        <div class="layout-form" id="motGrid"></div>
        <button type="button" class="mode-btn" style="margin-top:8px;width:100%" onclick="saveMotorLayout()">Lưu vào bộ nhớ robot (NVS)</button>
        <p class="hint" id="motLayMsg" style="margin-top:8px;min-height:1.2em"></p>
      </div>
    </div>
  </div>
</div>
<script>
const WS_URL='ws://'+location.hostname+':81';
let ws,retry;
const LIDAR_MAX_CM=800, US_BAR_MAX_CM=200;
const SLOT_LBL=['Trái trước','Trái sau','Phải trước','Phải sau'];
const PHY_US=[{v:0,t:'US Trước (Echo 10)'},{v:1,t:'US Sau (Echo 11)'},{v:2,t:'US Trái (Echo 12)'},{v:3,t:'US Phải (Echo 13)'}];
const PHY_ENC=[{v:0,t:'Enc FL (GPIO39)'},{v:1,t:'Enc RL (GPIO16)'},{v:2,t:'Enc FR (GPIO3)'},{v:3,t:'Enc RR (GPIO48)'}];
const PHY_MOT=[
  {v:0,t:'#1-A FL — PWM4, AIN 5/6 → AO1-AO2'},
  {v:1,t:'#1-B RL — PWM7, BIN 8/9 → BO1-BO2'},
  {v:2,t:'#2-A FR — PWM21, AIN 45/46 → AO1-AO2'},
  {v:3,t:'#2-B RR — PWM40, BIN 41/42 → BO1-BO2'}
];
const B=[{lbl:'Trái trước',i:'dULF'},{lbl:'Trái sau',i:'dULR'},{lbl:'Phải trước',i:'dURF'},{lbl:'Phải sau',i:'dURR'}];
const spark=[]; const SPARK_N=48;
function paintSpdTrack(el,raw){
  if(!el)return;
  const mn=parseFloat(el.min)||0,mx=parseFloat(el.max)||100;
  let v=parseFloat(raw);
  if(!isFinite(v))v=mn;
  v=Math.min(mx,Math.max(mn,v));
  const pct=mx>mn?((v-mn)/(mx-mn))*100:0;
  el.style.setProperty('--spd-pct',(Math.round(pct*100)/100)+'%');
}
function fmtAge(ms){
  if(ms==null||ms<0)return 'chưa có dữ liệu';
  if(ms<1000)return ms+' ms';
  if(ms<60000)return (ms/1000).toFixed(1)+' s';
  const m=Math.floor(ms/60000), s=Math.floor((ms%60000)/1000);
  return m+' ph '+s+' s';
}
/** Tuổi LiDAR: nếu có byte UART nhưng chưa parse được khung → báo rõ hơn. */
function fmtLidarAge(ms,l1,l2){
  if(ms!=null && ms>=0) return fmtAge(ms);
  const b=(Number(l1)||0)+(Number(l2)||0);
  if(b>0) return 'UART có byte, chưa khung hợp lệ';
  return 'chưa có dữ liệu';
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
    <div class="b-item sensor-off">
      <div class="bn">HC-SR04 · ${b.lbl}</div>
      <div class="val" id="val_${b.i}">OFF</div>
      <div class="b-bar"><div class="b-fill" id="bar_${b.i}" style="width:0%"></div></div>
    </div>`).join('');
}
function buildLayoutGrid(){
  const g=document.getElementById('layGrid'); if(!g)return;
  let h='<div class="layout-row" style="font-size:.65rem;text-transform:uppercase;letter-spacing:.06em;border-bottom:1px solid var(--line);padding-bottom:6px;margin-bottom:10px"><span>Góc xe</span><span>Siêu âm vật lý</span><span>Encoder vật lý</span></div>';
  for(let i=0;i<4;i++){
    h+=`<div class="layout-row"><label>${SLOT_LBL[i]}</label><select id="layUs${i}"></select><select id="layEnc${i}"></select></div>`;
  }
  g.innerHTML=h;
  for(let i=0;i<4;i++){
    const su=document.getElementById('layUs'+i), se=document.getElementById('layEnc'+i);
    PHY_US.forEach(o=>{const e=document.createElement('option'); e.value=o.v; e.textContent=o.t; su.appendChild(e);});
    PHY_ENC.forEach(o=>{const e=document.createElement('option'); e.value=o.v; e.textContent=o.t; se.appendChild(e);});
  }
}
function isPerm4(a){
  if(a.length!==4)return false; const s=new Set(a); return s.size===4 && a.every(v=>v>=0&&v<=3);
}
function applyLayoutPayload(d){
  if(!d||d.t!=='layout')return;
  for(let i=0;i<4;i++){
    const su=document.getElementById('layUs'+i), se=document.getElementById('layEnc'+i);
    if(d.us&&d.us[i]!=null&&su)su.value=String(d.us[i]);
    if(d.enc&&d.enc[i]!=null&&se)se.value=String(d.enc[i]);
  }
  if(d.lidF!=null){ const s=document.getElementById('lidFrontSel'); if(s) s.value=String(d.lidF); }
}
function buildMotorGrid(){
  const g=document.getElementById('motGrid'); if(!g)return;
  let h='<div class="layout-mot" style="font-size:.65rem;text-transform:uppercase;letter-spacing:.06em;border-bottom:1px solid var(--line);padding-bottom:6px;margin-bottom:10px"><span>Góc xe</span><span>Kênh TB6612 đang nối motor này</span><span>Chiều</span></div>';
  for(let i=0;i<4;i++){
    h+=`<div class="layout-mot"><label>${SLOT_LBL[i]}</label><select id="motMap${i}"></select><select id="motInv${i}"><option value="0">Tiến = như code</option><option value="1">Đảo chiều (lùi↔tiến)</option></select></div>`;
  }
  g.innerHTML=h;
  for(let i=0;i<4;i++){
    const sm=document.getElementById('motMap'+i);
    PHY_MOT.forEach(o=>{const e=document.createElement('option'); e.value=String(o.v); e.textContent=o.t; sm.appendChild(e);});
  }
}
function applyMotorPayload(d){
  if(!d||d.t!=='motLayout')return;
  for(let i=0;i<4;i++){
    const sm=document.getElementById('motMap'+i), si=document.getElementById('motInv'+i);
    if(d.mapMot&&d.mapMot[i]!=null&&sm) sm.value=String(d.mapMot[i]);
    if(d.motInv&&d.motInv[i]!=null&&si) si.value=String(d.motInv[i]);
  }
}
function saveMotorLayout(){
  const mapMot=[], motInv=[];
  for(let i=0;i<4;i++){
    mapMot.push(parseInt(document.getElementById('motMap'+i).value,10));
    motInv.push(parseInt(document.getElementById('motInv'+i).value,10));
  }
  const msg=document.getElementById('motLayMsg');
  if(!isPerm4(mapMot)){ msg.textContent='Lỗi: 4 kênh phải chọn đủ 0–3, không trùng.'; return; }
  msg.textContent='Đang gửi…';
  wsS({t:'motLayout',mapMot,motInv});
}
function saveLayout(){
  const us=[], enc=[];
  for(let i=0;i<4;i++){
    us.push(parseInt(document.getElementById('layUs'+i).value,10));
    enc.push(parseInt(document.getElementById('layEnc'+i).value,10));
  }
  const msg=document.getElementById('layMsg');
  if(!isPerm4(us)||!isPerm4(enc)){ msg.textContent='Lỗi: mỗi loại phải chọn đủ 0–3 không trùng.'; return; }
  const lidF=parseInt(document.getElementById('lidFrontSel').value,10);
  msg.textContent='Đang gửi…';
  wsS({t:'layout',us,enc,lidF});
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
function onU(cm,max,id,active){
  const v=document.getElementById('val_'+id);
  const f=document.getElementById('bar_'+id);
  if(!v||!f)return;
  const on=active!==false;
  const box=v.closest('.b-item');
  if(box) box.classList.toggle('sensor-off',!on);
  if(!on){
    v.textContent='OFF';
    f.style.width='0%';
    f.style.background='#475569';
    return;
  }
  v.textContent=cm+' cm';
  const w=Math.min(100,cm/max*100);
  f.style.width=w+'%';
  let c=cm<20? '#ef4444' : (cm<100? '#f59e0b' : '#22c55e');
  f.style.background=c;
}
function connectWS(){
  ws=new WebSocket(WS_URL);
  ws.onopen=()=>{
    document.getElementById('wsStatus').textContent='đã nối';
    if(retry)clearTimeout(retry);
    wsS({t:'mode',m:0});
    wsS({t:'joy',x:0,y:0});
    setTimeout(()=>{ wsS({t:'layoutGet'}); wsS({t:'motorLayoutGet'}); }, 2500);
  };
  ws.onclose=()=>{document.getElementById('wsStatus').textContent='mất kết nối…'; retry=setTimeout(connectWS,2000);};
  let uiPending=null, uiRaf=0;
  function flushTelemetry(){
    uiRaf=0;
    if(!uiPending) return;
    const d=uiPending; uiPending=null;
    applyTelemetry(d);
  }
  function scheduleTelemetry(d){
    uiPending=d;
    if(!uiRaf) uiRaf=requestAnimationFrame(flushTelemetry);
  }
  ws.onmessage=e=>{
    try{
      const d=JSON.parse(e.data);
      if(d.t==='layout'){
        applyLayoutPayload(d);
        const m=document.getElementById('layMsg');
        if(m) m.textContent='Đã đồng bộ bố trí từ robot.';
        return;
      }
      if(d.t==='layoutErr'){
        const m=document.getElementById('layMsg');
        if(m) m.textContent='Lỗi: '+(d.msg||'map không hợp lệ');
        return;
      }
      if(d.t==='motLayout'){
        applyMotorPayload(d);
        const m=document.getElementById('motLayMsg');
        if(m) m.textContent='Đã đồng bộ bố trí motor từ robot.';
        return;
      }
      if(d.t==='motLayoutErr'){
        const m=document.getElementById('motLayMsg');
        if(m) m.textContent='Lỗi: '+(d.msg||'mapMot/motInv không hợp lệ');
        return;
      }
      scheduleTelemetry(d);
    }catch(err){}
  };
}
function applyTelemetry(d){
      const isLidar = d.senMode === 'lidar';
      const lblF = document.getElementById('lblTxtLF'), lblB = document.getElementById('lblTxtLB');
      if(lblF) lblF.textContent = isLidar ? 'LiDAR' : 'US min';
      if(lblB) lblB.textContent = isLidar ? 'LiDAR' : 'US min';

      const lf=d.lf??0, lb=d.lb??0;
      const lfOn=d.lfOn!==undefined?!!d.lfOn:true, lbOn=d.lbOn!==undefined?!!d.lbOn:true;
      const stLF=document.getElementById('stLF'), stLB=document.getElementById('stLB');
      if(stLF){ stLF.textContent=lfOn?'ON':'OFF'; stLF.classList.toggle('on',lfOn); stLF.classList.toggle('off',!lfOn); }
      if(stLB){ stLB.textContent=lbOn?'ON':'OFF'; stLB.classList.toggle('on',lbOn); stLB.classList.toggle('off',!lbOn); }
      const cf=document.getElementById('lidCardF'), cB=document.getElementById('lidCardB');
      if(cf) cf.classList.toggle('sensor-off',!lfOn);
      if(cB) cB.classList.toggle('sensor-off',!lbOn);
      const vLfEl=document.getElementById('vLF'), vLbEl=document.getElementById('vLB');
      if(lfOn){ onL(lf,'vLF'); setRing('ringF',lf); }
      else { if(vLfEl) vLfEl.textContent='—'; setRing('ringF',0); }
      if(lbOn){ onL(lb,'vLB'); setRing('ringB',lb); }
      else { if(vLbEl) vLbEl.textContent='—'; setRing('ringB',0); }

      const ua=(Array.isArray(d.usOn)&&d.usOn.length===4)?d.usOn:null;
      onU(d.usLF??0,US_BAR_MAX_CM,'dULF', ua?!!ua[0]:true);
      onU(d.usLR??0,US_BAR_MAX_CM,'dULR', ua?!!ua[1]:true);
      onU(d.usRF??0,US_BAR_MAX_CM,'dURF', ua?!!ua[2]:true);
      onU(d.usRR??0,US_BAR_MAX_CM,'dURR', ua?!!ua[3]:true);

      const en=(Array.isArray(d.encOn)&&d.encOn.length===4)?d.encOn:null;
      function setRpmVis(slotId,valId,rpm,on){
        const slot=document.getElementById(slotId), rv=document.getElementById(valId);
        if(slot) slot.classList.toggle('sensor-off',!on);
        if(rv) rv.textContent=on?String(Math.round(rpm)):'OFF';
      }
      setRpmVis('rpmLF','rFL',d.rFL??0, en?!!en[0]:true);
      setRpmVis('rpmLR','rRL',d.rRL??0, en?!!en[1]:true);
      setRpmVis('rpmRF','rFR',d.rFR??0, en?!!en[2]:true);
      setRpmVis('rpmRR','rRR',d.rRR??0, en?!!en[3]:true);
      const da=((d.dFL??0)+(d.dRL??0)+(d.dFR??0)+(d.dRR??0))/4;
      document.getElementById('distAvg').textContent=da.toFixed(2);
      const ml=document.getElementById('modeLabel');
      ml.textContent=(d.mode===2)?'MQTT (Lộ trình)':((d.mode===1)?'Tự hành':'Lái tay');
      ml.className=(d.mode===1||d.mode===2)?'mode-auto':'mode-manual';
      const btnMan = document.getElementById('btnManual');
      const btnAut = document.getElementById('btnAuto');
      const btnRot = document.getElementById('btnRoute');
      if(btnMan) btnMan.classList.toggle('active', d.mode===0);
      if(btnAut) btnAut.classList.toggle('active', d.mode===1);
      if(btnRot) btnRot.classList.toggle('active', d.mode===2);
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
      if(d.batPct!=null && d.batPct>=0){
        document.getElementById('hiBatV').textContent=(Math.round((d.batV??0)*10)/10).toFixed(1);
        document.getElementById('hiBatPct').textContent=String(d.batPct);
      }else{
        document.getElementById('hiBatV').textContent='Tắt';
        document.getElementById('hiBatPct').textContent='—';
      }
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
      if(d.spdAutoPct!=null) document.getElementById('dvSpdAuto').textContent=String(d.spdAutoPct);
      document.getElementById('dvLfAge').textContent=fmtLidarAge(d.lfAge,d.lr1,d.lr2);
      document.getElementById('dvUsAge').textContent=fmtAge(d.usAge);
      if(d.lr1!=null)document.getElementById('dvLr1').textContent=String(d.lr1);
      if(d.lr2!=null)document.getElementById('dvLr2').textContent=String(d.lr2);
      if(d.spdPct!=null){
        const sl=document.getElementById('spdSlider');
        const touched = window._lastSliderTouch && (Date.now() - window._lastSliderTouch < 3000);
        if(sl && document.activeElement!==sl && !touched){
          sl.value=d.spdPct;
          paintSpdTrack(sl,d.spdPct);
          document.getElementById('spdVal').textContent=String(d.spdPct)+'%';
        }
      }
      if(d.spdAutoPct!=null){
        const sa=document.getElementById('spdAutoSlider');
        const touched = window._lastSliderTouch && (Date.now() - window._lastSliderTouch < 3000);
        if(sa && document.activeElement!==sa && !touched){
          const av=Math.max(15,Math.min(100,d.spdAutoPct));
          sa.value=av;
          paintSpdTrack(sa,av);
          document.getElementById('spdAutoVal').textContent=String(d.spdAutoPct)+'%';
        }
      }
      if(d.spdSwervePct!=null){
        const ss=document.getElementById('spdSwerveSlider');
        const touched = window._lastSliderTouch && (Date.now() - window._lastSliderTouch < 3000);
        if(ss && document.activeElement!==ss && !touched){
          const sv=Math.max(15,Math.min(100,d.spdSwervePct));
          ss.value=sv;
          paintSpdTrack(ss,sv);
          document.getElementById('spdSwerveVal').textContent=String(d.spdSwervePct)+'%';
        }
      }
      if(!window._rawJTick) window._rawJTick=0;
      if((++window._rawJTick%8)===0){
        const rj=document.getElementById('rawJ');
        if(rj) rj.textContent=JSON.stringify(d);
      }
}
const zone=document.getElementById('jsZone');
const knob=document.getElementById('jsKnob');
let drag=false, gStrafe=0;
function jUp(cx,cy){
  const r=zone.getBoundingClientRect();
  const R=Math.max(24, Math.min(r.width,r.height)/2 - 10);
  let ox=cx-r.left-r.width/2, oy=cy-r.top-r.height/2;
  const dist=Math.sqrt(ox*ox+oy*oy);
  if(dist>R){ ox=ox*R/dist; oy=oy*R/dist; }
  const jX=Math.round(ox/R*100), jY=Math.round(-oy/R*100);
  knob.style.left=(r.width/2+ox)+'px';
  knob.style.top=(r.height/2+oy)+'px';
  wsS({t:'joy',x:jX,y:jY,s:gStrafe});
}
function jRel(){ drag=false; knob.style.left='50%'; knob.style.top='50%'; wsS({t:'joy',x:0,y:0,s:gStrafe}); }
function sendStrafe(v){
  gStrafe=Math.round((parseInt(v,10)-50)/50*100);
  document.getElementById('strVal').textContent=v+'%';
  wsS({t:'joy',x:0,y:0,s:gStrafe});
}
function wsS(o){ if(ws && ws.readyState===1) ws.send(JSON.stringify(o)); }
function sendSpeed(v){
  window._lastSliderTouch = Date.now();
  const el=document.getElementById('spdSlider');
  paintSpdTrack(el,v);
  document.getElementById('spdVal').textContent=v+'%';
  wsS({t:'spd',v:parseInt(v,10)});
}
function sendSpeedAuto(v){
  window._lastSliderTouch = Date.now();
  const el=document.getElementById('spdAutoSlider');
  paintSpdTrack(el,v);
  document.getElementById('spdAutoVal').textContent=v+'%';
  wsS({t:'spdAuto',v:parseInt(v,10)});
}
function sendSpeedSwerve(v){
  window._lastSliderTouch = Date.now();
  const el=document.getElementById('spdSwerveSlider');
  paintSpdTrack(el,v);
  document.getElementById('spdSwerveVal').textContent=v+'%';
  wsS({t:'spdSwerve',v:parseInt(v,10)});
}
function setMode(m){
  document.getElementById('btnManual').classList.toggle('active',m===0);
  document.getElementById('btnAuto').classList.toggle('active',m===1);
  document.getElementById('btnRoute').classList.toggle('active',m===2);
  const ml=document.getElementById('modeLabel');
  ml.textContent=(m===2)?'MQTT (Lộ trình)':((m===1)?'Tự hành (demo)':'Lái tay');
  ml.className=(m===1||m===2)?'mode-auto':'mode-manual';
  wsS({t:'mode',m});
}
function sendEstop(){ wsS({t:'estop'}); }
function odomReset(){ wsS({t:'odomReset'}); }
(function(){
  const v=document.getElementById('tabCam');
  if(!v)return;
  const prof=[[320,240],[480,320],[640,480]];
  let stream=null,fh=0;
  const msg=document.getElementById('vMsg'),tag=document.getElementById('vTag'),wrap=document.getElementById('vWrap');
  const vHttps='https://'+location.hostname+'/vision';
  const uEl=document.getElementById('vHttpsUrl');
  if(uEl)uEl.textContent=vHttps;
  document.querySelectorAll('#lnkVisionHttps,#vGoHttps').forEach(a=>{a.href=vHttps;});
  function setM(t){if(msg)msg.textContent=t||'';}
  async function vStart(){
    if(!window.isSecureContext){
      setM('Trinh duyet chan camera tren HTTP. Mo: '+vHttps+' — chap nhan canh bao SSL, roi Bat camera.');
      return;
    }
    if(!navigator.mediaDevices||!navigator.mediaDevices.getUserMedia){
      setM('Khong co API camera. Mo '+vHttps+' bang Chrome/Safari moi.');
      return;
    }
    const p=prof[parseInt(document.getElementById('vProf').value,10)||0];
    if(stream)stream.getTracks().forEach(t=>t.stop());
    const tries=[
      {video:{width:{ideal:p[0]},height:{ideal:p[1]},facingMode:{ideal:'environment'}},audio:false},
      {video:{width:{ideal:p[0]},height:{ideal:p[1]}},audio:false},
      {video:true,audio:false}
    ];
    let err=null;
    for(const c of tries){
      try{
        stream=await navigator.mediaDevices.getUserMedia(c);
        v.srcObject=stream; await v.play();
        if(wrap)wrap.classList.add('live');
        if(tag)tag.textContent=p[0]+'x'+p[1];
        setM('');
        return;
      }catch(e){err=e;}
    }
    setM('Loi: '+(err&&err.message?err.message:'tu choi quyen camera'));
  }
  document.getElementById('vStart').onclick=vStart;
  document.getElementById('vProf').onchange=vStart;
  if(wrap)wrap.onclick=()=>{if(!stream)vStart();};
  document.getElementById('vFh').onclick=function(){fh=!fh;v.style.transform='scaleX('+(fh?-1:1)+')';this.classList.toggle('on',fh);};
  document.getElementById('vSnap').onclick=function(){
    if(!v.videoWidth)return;
    const c=document.createElement('canvas');c.width=v.videoWidth;c.height=v.videoHeight;
    c.getContext('2d').drawImage(v,0,0);
    c.toBlob(b=>{if(!b)return;const a=document.createElement('a');a.href=URL.createObjectURL(b);a.download='smb-'+Date.now()+'.jpg';a.click();setM('Da tai anh');},'image/jpeg',0.88);
  };
})();
function initSecNav(){
  const links=[...document.querySelectorAll('.secnav a')];
  const secs=['sec-vision','sec-sense','sec-drive','sec-monitor','sec-layout','sec-mot-layout'].map(id=>document.getElementById(id)).filter(Boolean);
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
buildLayoutGrid();
buildMotorGrid();
buildBump();
(function(){const a=document.getElementById('spdSlider'),b=document.getElementById('spdAutoSlider'),c=document.getElementById('spdSwerveSlider');
  if(a){paintSpdTrack(a,a.value);document.getElementById('spdVal').textContent=a.value+'%';}
  if(b){paintSpdTrack(b,b.value);document.getElementById('spdAutoVal').textContent=b.value+'%';}
  if(c){paintSpdTrack(c,c.value);document.getElementById('spdSwerveVal').textContent=c.value+'%';}})();
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
    if (t && strcmp(t, "layoutGet") == 0) {
      sensorLayoutReplyToClient(g_wsServer, num);
      return;
    }
    if (t && strcmp(t, "layout") == 0) {
      if (sensorLayoutApplyJson(doc, g_prefs)) {
        sensorLayoutReplyToClient(g_wsServer, num);
      } else {
        JsonDocument errDoc;
        errDoc["t"] = "layoutErr";
        errDoc["msg"] = "us/enc phai hoan vi 0..3, lidF 0 hoac 1";
        char b[120];
        serializeJson(errDoc, b, sizeof(b));
        g_wsServer.sendTXT(num, b);
      }
      return;
    }
    if (t && strcmp(t, "motorLayoutGet") == 0) {
      motorLayoutReplyToClient(g_wsServer, num);
      return;
    }
    if (t && strcmp(t, "motLayout") == 0) {
      if (motorLayoutApplyJson(doc, g_prefs)) {
        motorLayoutReplyToClient(g_wsServer, num);
      } else {
        JsonDocument errDoc;
        errDoc["t"] = "motLayoutErr";
        errDoc["msg"] = "mapMot phai hoan vi 0..3, motInv 0 hoac 1";
        char b[120];
        serializeJson(errDoc, b, sizeof(b));
        g_wsServer.sendTXT(num, b);
      }
      return;
    }
    robotApplyControlJson(doc);
  }
}

inline void webUIBroadcast() {
  /* Không có client WS → khỏi serialize JSON (giảm lag AP khi chỉ mở HTTP tĩnh). */
  if (g_wsServer.connectedClients() == 0) return;

  static uint8_t s_telemPhase = 0;
  s_telemPhase++;

  JsonDocument doc;
  robotTelemetryFillJson(doc, (s_telemPhase % 5u) == 0u);  /* mỗi ~1.75s: gói đầy đủ */

  char buf[1200];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n >= sizeof(buf)) return;
  g_wsServer.broadcastTXT(buf);
}
inline void webUIInit() {
  g_prefs.begin(NVS_NAMESPACE, true);
  g_state.baseSpeed = g_prefs.getUInt("baseSpeed", PWM_MAX * 60 / 100);
  g_state.autoBaseSpeed = g_prefs.getUInt("autoBaseSpeed", PWM_MAX * 60 / 100);
  g_state.swerveBaseSpeed = g_prefs.getUInt("swerveSpeed", PWM_MAX * 40 / 100);
  g_state.rotateBaseSpeed = g_prefs.getUInt("rotateSpeed", PWM_MAX * 30 / 100);
  g_state.imuYawScale = (float)g_prefs.getUInt("yawScale", 100) / 100.0f;
  sensorLayoutLoad(g_prefs);
  motorLayoutLoad(g_prefs);
  g_prefs.end();

  batteryMonitorInit();

  WiFi.persistent(false);
#if WIFI_STA_ENABLE
  WiFi.mode(WIFI_AP_STA);   // Vừa phát AP, vừa kết nối router
#else
  WiFi.mode(WIFI_AP);
#endif
  WiFi.setSleep(false);

  bool apOk = WiFi.softAP(AP_SSID, AP_PASS, AP_WIFI_CHANNEL, 0, AP_MAX_CLIENTS);
  esp_wifi_set_ps(WIFI_PS_NONE);
  if (!apOk) {
    Serial.println(F("[WiFi] softAP() tra ve false — thu tat mo nguon / kiem tra core Arduino"));
  }
  Serial.printf("[WiFi] SoftAP: \"%s\"  IP: %s  kenh: %d  MAC: %s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str(), (int)WiFi.channel(),
                WiFi.softAPmacAddress().c_str());
  Serial.println(
      F("[WiFi] Dien thoai ket noi VAO mang do robot phat (khong phai WiFi nha). Mat khau: AP_PASS trong Config.h"));

#if WIFI_STA_ENABLE
  /* Thử kết nối lần lượt các SSID đã cấu hình — kết nối được cái đầu tiên tìm thấy.
   * Robot vẫn chạy SoftAP trong khi thử STA, MQTT chỉ bật khi STA thành công. */
  {
    struct { const char* ssid; const char* pass; } staList[] = {
      { STA_SSID,   STA_PASS   },
      { STA_SSID_2, STA_PASS_2 },
      { STA_SSID_3, STA_PASS_3 },
      { STA_SSID_4, STA_PASS_4 },
      { STA_SSID_5, STA_PASS_5 },
    };
    constexpr int STA_LIST_COUNT = sizeof(staList) / sizeof(staList[0]);
    bool staOk = false;

    for (int si = 0; si < STA_LIST_COUNT && !staOk; si++) {
      if (staList[si].ssid == nullptr || strlen(staList[si].ssid) == 0) continue;
      Serial.printf("[WiFi] STA [%d/%d]: dang ket noi \"%s\"...\n",
                    si + 1, STA_LIST_COUNT, staList[si].ssid);
      WiFi.begin(staList[si].ssid, staList[si].pass);

      uint32_t staStart = millis();
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print('.');
        if (millis() - staStart > STA_CONNECT_TIMEOUT_MS) {
          Serial.printf("\n[WiFi] \"%s\" timeout — thu SSID tiep theo.\n", staList[si].ssid);
          WiFi.disconnect();
          delay(500);
          break;
        }
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] STA OK! SSID=\"%s\"  IP=%s\n",
                      staList[si].ssid, WiFi.localIP().toString().c_str());
        // Luôn bật sẵn MQTT để robot có thể nhận tín hiệu điều hướng từ Backend / Web Manager
        g_mqttEnabled = true;
        staOk = true;
      }
    }
    if (!staOk) {
      Serial.println(F("[WiFi] Tat ca SSID that bai — AP-only mode, MQTT disabled."));
    }
  }
#endif

  g_httpServer.on("/", HTTP_GET, []() {
    g_httpServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    g_httpServer.sendHeader("Pragma", "no-cache");
    g_httpServer.send_P(200, "text/html; charset=utf-8", HTML_PAGE);
  });
  g_httpServer.on("/vision", HTTP_GET, []() {
    String loc = String("https://") + WiFi.localIP().toString() + "/vision";
    g_httpServer.sendHeader("Location", loc);
    g_httpServer.send(302, "text/plain", "Camera can HTTPS — redirecting");
  });
  g_httpServer.on("/status", HTTP_GET, []() {
    String j = "{\"ip\":\"" + WiFi.localIP().toString() +
               "\",\"wifi\":\"" + WiFi.SSID() + "\",\"camera\":\"tablet\",\"clients\":0}";
    g_httpServer.send(200, "application/json", j);
  });
  g_httpServer.begin();
  visionHttpsBegin();
  g_wsServer.begin();
  g_wsServer.onEvent(onWebSocketEvent);
  Serial.printf("[WS] WebSocket server trên port %d\n", WS_PORT);

#if WIFI_STA_ENABLE
  mqttInit();
#endif
}

inline void webUILoop() {
  g_httpServer.handleClient();
  g_wsServer.loop();
#if WIFI_STA_ENABLE
  // Tự động bật/tắt MQTT dựa trên g_state.mode
  if (g_state.mode == MODE_WAYPOINT) {
    if (!g_mqttEnabled) {
      g_mqttEnabled = true;
      g_mqttLastReconnectMs = 0; // Buộc thử kết nối lại ngay lập tức
      Serial.println(F("[MQTT] Che do Lo trinh kich hoat -> Bat ket noi MQTT..."));
    }
  } else {
    // Để nhận lệnh từ Web Manager (Gửi Test Lộ trình), ta không bao giờ tắt MQTT nữa
    // (Đã xoá logic ngắt kết nối MQTT khi về chế độ Manual)
  }
#endif
}

#endif // WEBUI_H
