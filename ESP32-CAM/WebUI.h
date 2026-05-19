/* =====================================================================
 *  WebUI.h — Dashboard (PROGMEM, không CDN)
 * =====================================================================*/
#ifndef ESP32_CAM_WEB_UI_H
#define ESP32_CAM_WEB_UI_H

#include <WebServer.h>

static const char CAM_DASH_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#0a1628">
<title>SmartMarketBot Vision</title>
<style>
:root{--bg:#070d18;--card:#101c32;--line:#1e3358;--txt:#e8f0ff;--muted:#8ba3c7;--accent:#2ee6a8;--accent2:#3b9eff}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:radial-gradient(900px 400px at 0 -10%,#132a4a,transparent),var(--bg);color:var(--txt);min-height:100vh;padding:14px}
.wrap{max-width:1080px;margin:0 auto}
header{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px;margin-bottom:14px}
.brand{display:flex;align-items:center;gap:10px}
.logo{width:42px;height:42px;border-radius:11px;background:linear-gradient(135deg,var(--accent),var(--accent2));display:grid;place-items:center;font-weight:800;font-size:13px;color:#041018}
h1{font-size:1.1rem} .sub{font-size:.78rem;color:var(--muted)}
.badge{padding:5px 11px;border-radius:999px;font-size:.72rem;border:1px solid var(--line);background:#0d1526;color:var(--accent)}
.badge.bad{color:#ff6b8a}
.grid{display:grid;gap:12px}@media(min-width:880px){.grid{grid-template-columns:1.45fr .85fr}}
.panel{background:linear-gradient(180deg,#101c32,#0d1526);border:1px solid var(--line);border-radius:14px;overflow:hidden;box-shadow:0 10px 32px rgba(0,0,0,.35)}
.ph{padding:11px 14px;border-bottom:1px solid var(--line);display:flex;justify-content:space-between;align-items:center}
.ph h2{font-size:.88rem}
.pb{padding:12px 14px 14px}
.vw{position:relative;background:#000;border-radius:10px;border:1px solid #1a2d4d;aspect-ratio:4/3;overflow:hidden}
.vw img{width:100%;height:100%;object-fit:contain;display:block}
.vtag{position:absolute;top:8px;left:8px;font-size:.7rem;font-weight:600;padding:4px 9px;border-radius:7px;background:rgba(0,0,0,.55);border:1px solid rgba(255,255,255,.1)}
.vtag em{color:var(--accent);font-style:normal}
.ld{position:absolute;inset:0;display:grid;place-items:center;background:rgba(0,0,0,.45);font-size:.82rem;color:var(--muted)}
.ld.hide{display:none}
.stats{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}@media(min-width:420px){.stats{grid-template-columns:repeat(4,1fr)}}
.st{background:rgba(0,0,0,.22);border:1px solid var(--line);border-radius:9px;padding:9px 10px}
.st label{display:block;font-size:.65rem;color:var(--muted);text-transform:uppercase;letter-spacing:.05em}
.st strong{display:block;margin-top:3px;font-size:1rem}
.act{display:flex;flex-wrap:wrap;gap:7px;margin-top:12px}
.btn{border:none;cursor:pointer;padding:9px 13px;border-radius:9px;font-size:.8rem;font-weight:600;text-decoration:none;display:inline-block}
.b1{background:linear-gradient(135deg,var(--accent),#1fc88a);color:#041018}
.b2{background:#152744;color:var(--txt);border:1px solid var(--line)}
.b3{background:transparent;color:var(--muted);border:1px dashed var(--line)}
.lk{margin-top:12px;font-size:.76rem;color:var(--muted)} .lk a{color:var(--accent2);text-decoration:none}
.ft{margin-top:14px;text-align:center;font-size:.7rem;color:var(--muted)}
.ctl{margin-top:10px;padding-top:10px;border-top:1px solid var(--line)}
.ctl label{font-size:.72rem;color:var(--muted);display:block;margin-bottom:6px}
.row{display:flex;flex-wrap:wrap;gap:6px;align-items:center}
.sel{padding:7px 10px;border-radius:8px;background:#152744;border:1px solid var(--line);color:var(--txt);font-size:.78rem}
.b4{padding:7px 10px;border-radius:8px;background:#152744;border:1px solid var(--line);color:var(--txt);font-size:.75rem;cursor:pointer}
.b4.on{border-color:var(--accent);color:var(--accent)}
#profHint{font-size:.7rem;color:var(--muted);margin-top:6px}
</style>
</head>
<body>
<div class="wrap">
<header>
<div class="brand"><div class="logo">SMB</div><div><h1>SmartMarketBot Vision</h1><p class="sub">ESP32-CAM</p></div></div>
<div id="badge" class="badge">...</div>
</header>
<div class="grid">
<section class="panel">
<div class="ph"><h2>Live preview</h2><label style="font-size:.78rem;color:var(--muted)"><input type="checkbox" id="liveOn" checked> Auto</label></div>
<div class="pb">
<div class="vw"><div id="ld" class="ld">Dang ket noi stream...</div><img id="cam" src="/stream" alt="live"><div class="vtag"><span id="modeTag">MJPEG</span> <em id="fps">-- fps</em></div></div>
<div class="act">
<button class="btn b1" type="button" id="snap">Anh HD VGA</button>
<button class="btn b2" type="button" id="ref">Khoi dong lai stream</button>
<button class="btn b3" type="button" id="pollBtn">Che do poll</button>
</div>
<div class="ctl">
<label>Huong anh (sensor — ap dung ca capture)</label>
<div class="row">
<button type="button" class="b4" id="fh">Lat ngang</button>
<button type="button" class="b4" id="fv">Lat doc</button>
<button type="button" class="b4" id="f180">Xoay 180</button>
<button type="button" class="b4" id="rstF">Mac dinh</button>
</div>
<label style="margin-top:8px">Xoay khung nhin (chi tren man hinh)</label>
<div class="row">
<button type="button" class="b4" id="r0">0</button>
<button type="button" class="b4" id="r90">90</button>
<button type="button" class="b4" id="r180d">180</button>
<button type="button" class="b4" id="r270">270</button>
</div>
<label style="margin-top:8px">Chat luong live</label>
<select class="sel" id="prof">
<option value="0">Muot (QVGA)</option>
<option value="1">Can bang (HVGA)</option>
<option value="2">Net (VGA, cham hon)</option>
</select>
<p id="profHint">Muot: ~12-18 fps | Net: ~6-10 fps tren WiFi robot</p>
</div>
</div>
</section>
<section class="panel">
<div class="ph"><h2>He thong</h2></div>
<div class="pb">
<div class="stats">
<div class="st"><label>IP</label><strong id="ip">-</strong></div>
<div class="st"><label>RSSI</label><strong id="rssi">-</strong></div>
<div class="st"><label>Heap</label><strong id="heap">-</strong></div>
<div class="st"><label>Uptime</label><strong id="up">-</strong></div>
</div>
<p class="lk"><a href="/status">/status</a> &middot; <a href="/capture">/capture</a> &middot; <a href="/preview">/preview</a> &middot; <a href="/stream">/stream</a></p>
</div>
</section>
</div>
<p class="ft">S3 dieu khien robot &middot; AI tren backend</p>
</div>
<script>
const cam=document.getElementById('cam'),ld=document.getElementById('ld'),fps=document.getElementById('fps');
const modeTag=document.getElementById('modeTag'),liveOn=document.getElementById('liveOn');
let mode='stream',busy=0,n=0,t0=performance.now();
function updFps(){const s=(performance.now()-t0)/1000;if(s>=0.8){fps.textContent=Math.round(n/s)+' fps';n=0;t0=performance.now();}}
function startStream(){mode='stream';modeTag.textContent='MJPEG';ld.classList.remove('hide');cam.src='/stream?'+Date.now();}
function startPoll(){mode='poll';modeTag.textContent='POLL';function loop(){if(!liveOn.checked||busy||mode!=='poll')return;busy=1;const u='/preview?'+Date.now(),im=new Image();im.onload=()=>{cam.src=u;ld.classList.add('hide');n++;updFps();busy=0;requestAnimationFrame(loop);};im.onerror=()=>{busy=0;setTimeout(loop,80);};im.src=u;}loop();}
cam.addEventListener('load',()=>{if(mode==='stream'){ld.classList.add('hide');n++;updFps();}});
cam.addEventListener('error',()=>{if(mode==='stream')startPoll();});
setTimeout(()=>{if(mode==='stream'&&cam.naturalWidth<8)startPoll();},4500);
async function st(){try{const j=await(await fetch('/status')).json();
document.getElementById('ip').textContent=j.ip||'-';
document.getElementById('rssi').textContent=(j.wifi!=null)?j.wifi+' dBm':'-';
document.getElementById('heap').textContent=j.heap?Math.round(j.heap/1024)+' KB':'-';
document.getElementById('up').textContent=j.uptime?j.uptime+' s':'-';
const b=document.getElementById('badge');
if(j.camera==='ok'){b.textContent='Camera OK';b.classList.remove('bad');}
else{b.textContent='Camera loi';b.classList.add('bad');}}catch(e){}}
document.getElementById('snap').onclick=()=>window.open('/capture?'+Date.now(),'_blank');
document.getElementById('ref').onclick=()=>startStream();
document.getElementById('pollBtn').onclick=()=>startPoll();
let cssRot=0;
function setCssRot(deg){cssRot=deg;cam.style.transform='rotate('+deg+'deg)';}
async function setFlip(h,v){
  await fetch('/orientation?hmirror='+(h?1:0)+'&vflip='+(v?1:0));
  document.getElementById('fh').classList.toggle('on',!!h);
  document.getElementById('fv').classList.toggle('on',!!v);
}
document.getElementById('fh').onclick=async()=>{const j=await(await fetch('/orientation')).json();await setFlip(!j.hmirror,j.vflip);};
document.getElementById('fv').onclick=async()=>{const j=await(await fetch('/orientation')).json();await setFlip(j.hmirror,!j.vflip);};
document.getElementById('f180').onclick=()=>setFlip(1,1);
document.getElementById('rstF').onclick=()=>setFlip(1,1);
document.getElementById('r0').onclick=()=>setCssRot(0);
document.getElementById('r90').onclick=()=>setCssRot(90);
document.getElementById('r180d').onclick=()=>setCssRot(180);
document.getElementById('r270').onclick=()=>setCssRot(270);
document.getElementById('prof').onchange=async(e)=>{
  await fetch('/profile?mode='+e.target.value);
  startStream();
};
async function loadOrient(){
  try{const j=await(await fetch('/orientation')).json();
  document.getElementById('fh').classList.toggle('on',!!j.hmirror);
  document.getElementById('fv').classList.toggle('on',!!j.vflip);}catch(e){}}
st();setInterval(st,3000);loadOrient();
</script>
</body>
</html>
)HTML";

inline void webUiSendDashboard(WebServer &srv) {
  srv.send_P(200, "text/html; charset=utf-8", CAM_DASH_HTML);
}

#endif
