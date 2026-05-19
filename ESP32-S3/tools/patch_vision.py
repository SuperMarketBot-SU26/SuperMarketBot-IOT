# -*- coding: utf-8 -*-
import pathlib

p = pathlib.Path(__file__).resolve().parent.parent / "WebUI.h"
text = p.read_text(encoding="utf-8")

marker = '      <p class="rail-title">Vận hành</p>\n      <motion></motion><motion></motion><div class="card">'
if marker not in text:
    marker = '      <p class="rail-title">Vận hành</p>\n      <div class="card">'

insert = """      <p class="rail-title">Vận hành</p>

      <motion></motion><motion></motion><motion></motion><div class="card" id="sec-vision" style="scroll-margin-top:calc(var(--nav-h) + 8px)">
        <h2><span class="dot"></span> Vision — camera tablet (demo)</h2>
        <p class="hint" style="margin-top:-4px">Hotspot <b>SmartMarketBot</b> (ESP32-S3). Camera tablet — khong qua ESP32-CAM. Backend/FE sau.</p>
        <div class="vision-wrap">
          <video id="tabCam" playsinline autoplay muted></video>
          <div class="vision-tag" id="visTag">Chua bat</div>
        </div>
        <div class="vision-ctl">
          <button type="button" class="vision-btn" id="visStart">Bat camera</button>
          <select class="vision-sel" id="visProf" aria-label="Chat luong">
            <option value="0">Muot QVGA</option>
            <option value="1">Can bang HVGA</option>
            <option value="2">Net VGA</option>
          </select>
          <button type="button" class="vision-btn" id="visFh">Lat ngang</button>
          <button type="button" class="vision-btn" id="visFv">Lat doc</button>
          <button type="button" class="vision-btn" id="visSnap">Chup anh</button>
        </div>
        <p class="hint" id="visMsg" style="margin-top:8px;min-height:1.2em"></p>
      </div>

      <div class="card">"""

if marker not in text:
    raise SystemExit("HTML marker not found")
text = text.replace(marker, insert.replace("<motion></motion>", ""), 1)

js_add = r"""
function initTabletVision(){
  const v=document.getElementById('tabCam'),msg=document.getElementById('visMsg'),tag=document.getElementById('visTag');
  if(!v)return;
  let stream=null,fh=0,fv=0;
  const prof=[[320,240],[480,320],[640,480]];
  function setMsg(t){if(msg)msg.textContent=t||'';}
  function paintFlip(){v.style.transform='scaleX('+(fh?-1:1)+') scaleY('+(fv?-1:1)+')';}
  async function start(){
    try{
      const p=prof[parseInt(document.getElementById('visProf').value,10)||0];
      if(stream)stream.getTracks().forEach(t=>t.stop());
      stream=await navigator.mediaDevices.getUserMedia({video:{width:{ideal:p[0]},height:{ideal:p[1]}},facingMode:'environment'},audio:false});
      v.srcObject=stream; await v.play();
      tag.textContent='Live '+p[0]+'x'+p[1]; setMsg('Camera OK — demo tablet');
    }catch(e){setMsg('Loi: '+e.message+' — cap quyen camera'); tag.textContent='Loi';}
  }
  document.getElementById('visStart').onclick=start;
  document.getElementById('visProf').onchange=start;
  document.getElementById('visFh').onclick=()=>{fh=!fh;document.getElementById('visFh').classList.toggle('on',fh);paintFlip();};
  document.getElementById('visFv').onclick=()=>{fv=!fv;document.getElementById('visFv').classList.toggle('on',fv);paintFlip();};
  document.getElementById('visSnap').onclick=()=>{
    if(!v.videoWidth)return;
    const c=document.createElement('canvas');c.width=v.videoWidth;c.height=v.videoHeight;
    const x=c.getContext('2d');x.save();x.translate(fh?c.width:0,fv?c.height:0);x.scale(fh?-1:1,fv?-1:1);
    x.drawImage(v,0,0);x.restore();
    c.toBlob(b=>{if(!b)return;const a=document.createElement('a');a.href=URL.createObjectURL(b);a.download='smb-'+Date.now()+'.jpg';a.click();setMsg('Da tai anh (backend sau)');},'image/jpeg',0.88);
  };
}
"""

js_marker = "initSecNav();"
if js_marker not in text:
    raise SystemExit("js marker not found")
if "initTabletVision" not in text:
    text = text.replace(js_marker, js_add + js_marker, 1)

nav_old = "const secs=['sec-sense','sec-drive','sec-monitor','sec-layout','sec-mot-layout']"
nav_new = "const secs=['sec-vision','sec-sense','sec-drive','sec-monitor','sec-layout','sec-mot-layout']"
text = text.replace(nav_old, nav_new, 1)

end_marker = "connectWS();"
if "initTabletVision();" not in text:
    text = text.replace(end_marker, "initTabletVision();\n" + end_marker, 1)

p.write_text(text, encoding="utf-8")
print("patched OK")
