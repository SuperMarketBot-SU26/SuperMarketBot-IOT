/* =====================================================================
 *  VisionTablet.h — Demo camera tablet (getUserMedia), phục vụ /vision
 *  Mo cung WiFi SmartMarketBot (S3 SoftAP). Backend/FE tich hop sau.
 * =====================================================================*/
#ifndef VISION_TABLET_H
#define VISION_TABLET_H

#include <WebServer.h>

static const char VISION_TABLET_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#0a1628">
<title>SmartMarketBot Vision (tablet)</title>
<style>
:root{--bg:#070d18;--card:#101c32;--line:#1e3358;--txt:#e8f0ff;--muted:#8ba3c7;--accent:#2ee6a8;--accent2:#3b9eff}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:var(--bg);color:var(--txt);padding:14px;max-width:960px;margin:0 auto}
a{color:var(--accent2)}
header{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:10px;margin-bottom:12px}
h1{font-size:1.1rem}
.sub{font-size:.78rem;color:var(--muted)}
.panel{background:linear-gradient(180deg,var(--card),#0d1526);border:1px solid var(--line);border-radius:14px;padding:14px}
.vw{position:relative;background:#000;border-radius:10px;border:1px solid var(--line);aspect-ratio:4/3;overflow:hidden;margin-top:10px}
.vw video{width:100%;height:100%;object-fit:contain;display:block;background:#000}
.vw .tap{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;background:rgba(0,0,0,.72);cursor:pointer;font-size:.9rem;font-weight:600;color:var(--accent);padding:12px;text-align:center}
.vw.live .tap{display:none}
.tag{position:absolute;top:8px;left:8px;font-size:.7rem;padding:4px 9px;border-radius:7px;background:rgba(0,0,0,.55);border:1px solid rgba(255,255,255,.1);z-index:2}
.tag em{color:var(--accent);font-style:normal}
.row{display:flex;flex-wrap:wrap;gap:7px;margin-top:12px;align-items:center}
.btn{border:none;cursor:pointer;padding:9px 12px;border-radius:9px;font-size:.8rem;font-weight:600}
.b1{background:linear-gradient(135deg,var(--accent),#1fc88a);color:#041018}
.b2{background:#152744;color:var(--txt);border:1px solid var(--line)}
.b2.on{border-color:var(--accent);color:var(--accent)}
.sel{padding:8px;border-radius:8px;border:1px solid var(--line);background:#152744;color:var(--txt)}
.hint{font-size:.75rem;color:var(--muted);margin-top:10px;line-height:1.5}
.stats{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;margin-top:12px}
.st{background:rgba(0,0,0,.2);border:1px solid var(--line);border-radius:8px;padding:8px 10px;font-size:.8rem}
.st label{display:block;font-size:.65rem;color:var(--muted);text-transform:uppercase}
.st strong{font-size:1rem}
</style>
</head>
<body>
<header>
<div>
<h1>Vision — camera tablet</h1>
<p class="sub">Demo SmartMarketBot · WiFi <b>SmartMarketBot</b> (ESP32-S3)</p>
</div>
<a href="http://192.168.4.1/">← Robot HMI (HTTP)</a>
</header>
<section class="panel">
<p class="hint">Dung <b>https://</b> (khong phai http). Lan dau: chap nhan <b>chung chi tu ky</b> (Advanced / Tiep tuc). Camera tren tablet, khong qua ESP32-CAM.</p>
<div class="vw" id="vw">
<video id="cam" playsinline autoplay muted></video>
<div class="tap" id="tap">Cham de bat camera</div>
<div class="tag"><em id="mode">—</em> · <span id="fps">— fps</span></div>
</div>
<div class="row">
<button type="button" class="btn b1" id="start">Bat camera</button>
<select class="sel" id="prof">
<option value="0">Muot QVGA</option>
<option value="1">Can bang HVGA</option>
<option value="2">Net VGA</option>
</select>
<button type="button" class="btn b2" id="fh">Lat ngang</button>
<button type="button" class="btn b2" id="fv">Lat doc</button>
<button type="button" class="btn b2" id="snap">Chup &amp; tai anh</button>
</div>
<p class="hint" id="msg"></p>
<div class="stats">
<div class="st"><label>Robot IP</label><strong id="rip">—</strong></div>
<div class="st"><label>WiFi</label><strong id="rwifi">—</strong></div>
</div>
</section>
<script>
const v=document.getElementById('cam'),prof=[[320,240],[480,320],[640,480]];
let stream=null,fh=0,fv=0,n=0,t0=performance.now();
function setMsg(t){document.getElementById('msg').textContent=t||'';}
function paint(){v.style.transform='scaleX('+(fh?-1:1)+') scaleY('+(fv?-1:1)+')';}
function updFps(){const s=(performance.now()-t0)/1000;if(s>=0.8){document.getElementById('fps').textContent=Math.round(n/s)+' fps';n=0;t0=performance.now();}}
async function start(){
  if(!window.isSecureContext){
    setMsg('Can HTTPS. Neu dang http:// — doi thanh https://'+location.hostname+'/vision');
    return;
  }
  if(!navigator.mediaDevices||!navigator.mediaDevices.getUserMedia){
    setMsg('Khong co API camera. Thu Chrome hoac Safari moi.');
    return;
  }
  const p=prof[parseInt(document.getElementById('prof').value,10)||0];
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
      document.getElementById('vw').classList.add('live');
      document.getElementById('mode').textContent='Live '+p[0]+'x'+p[1];
      setMsg('');
      return;
    }catch(e){err=e;}
  }
  setMsg('Loi: '+(err&&err.message?err.message:'tu choi quyen camera'));
}
v.addEventListener('loadeddata',()=>{n++;updFps();});
document.getElementById('start').onclick=start;
document.getElementById('tap').onclick=start;
document.getElementById('vw').onclick=e=>{if(e.target.id==='vw'&&!stream)start();};
document.getElementById('prof').onchange=start;
document.getElementById('fh').onclick=()=>{fh=!fh;document.getElementById('fh').classList.toggle('on',fh);paint();};
document.getElementById('fv').onclick=()=>{fv=!fv;document.getElementById('fv').classList.toggle('on',fv);paint();};
document.getElementById('snap').onclick=()=>{
  if(!v.videoWidth)return;
  const c=document.createElement('canvas');c.width=v.videoWidth;c.height=v.videoHeight;
  const x=c.getContext('2d');x.save();x.translate(fh?c.width:0,fv?c.height:0);x.scale(fh?-1:1,fv?-1:1);
  x.drawImage(v,0,0);x.restore();
  c.toBlob(b=>{if(!b)return;const a=document.createElement('a');a.href=URL.createObjectURL(b);a.download='smb-'+Date.now()+'.jpg';a.click();setMsg('Da tai anh JPG');},'image/jpeg',0.88);
};
async function st(){
  try{
    const j=await(await fetch('/status')).json();
    document.getElementById('rip').textContent=j.ip||location.hostname;
    document.getElementById('rwifi').textContent=(j.wifi!=null)?j.wifi+' dBm':'—';
  }catch(e){document.getElementById('rip').textContent=location.hostname;}
}
document.getElementById('rip').textContent=location.hostname;
st(); setInterval(st,4000);
</script>
</body>
</html>
)HTML";

inline void visionTabletSendPage(WebServer &srv) {
  srv.sendHeader("Cache-Control", "no-store");
  srv.send_P(200, "text/html; charset=utf-8", VISION_TABLET_HTML);
}

#endif
