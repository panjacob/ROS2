#pragma once
// Control page served at "/". Talks to /cmd, /status, /config, /reboot.

static const char WEBPAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>NanoBee</title>
<style>
body{font-family:sans-serif;margin:0;padding:10px;background:#1e1e1e;color:#ddd;max-width:900px}
h3{margin:14px 0 6px}section{background:#2a2a2a;padding:10px;border-radius:6px;margin-bottom:10px}
button{padding:8px 12px;margin:2px;border:0;border-radius:4px;background:#3c6;color:#000;font-weight:bold;touch-action:none;user-select:none}
button.g{background:#555;color:#ddd}button.r{background:#d33;color:#fff}
#estop{font-size:22px;padding:14px 30px}
input{background:#111;color:#ddd;border:1px solid #555;padding:5px;width:80px}
table{border-collapse:collapse}td{padding:3px 6px}
#st{font-family:monospace;font-size:16px}
#log{font-family:monospace;white-space:pre;height:140px;overflow:auto;background:#111;padding:6px}
textarea{width:100%;height:320px;background:#111;color:#ddd;font-family:monospace}
</style></head><body>
<section>
<button id="estop" class="r" onclick="cmd('!')">STOP</button>
<button class="g" onclick="cmd('S')">Smooth stop</button>
<button onclick="cmd('E 1')">Enable</button>
<button class="g" onclick="cmd('E 0')">Disable</button>
<div id="st">connecting...</div>
</section>

<section><h3>Joints</h3>
Jog speed <input id="jspd" value="1000"> accel <input id="jacc" value="2000">
<table id="jt"></table>
<div>
Speed <input id="mspd" value="1000"> accel <input id="macc" value="2000">
<button onclick="move('M')">Move to targets</button>
<button class="g" onclick="move('R')">Move relative</button>
<button class="g" onclick="cmd('Z')">Set all to 0</button>
<button class="g" onclick="copyPos()">Targets = current</button>
</div></section>

<section><h3>Gripper (servo)</h3>
<input type="range" id="srv" min="500" max="2500" value="1500" style="width:60%" oninput="srvv.textContent=this.value" onchange="cmd('G '+this.value)">
<span id="srvv">1500</span> us <button class="g" onclick="cmd('G 0')">Off</button>
</section>

<section><h3>Console</h3>
<input id="cin" style="width:70%" placeholder="e.g. M 100 0 0 0 0 500" onkeydown="if(event.key=='Enter'){cmd(this.value);this.value=''}">
<div id="log"></div></section>

<section><h3>Settings (config.yaml)</h3>
<input type="file" id="yf" accept=".yaml,.yml,.txt" style="width:auto">
<button class="g" onclick="loadCfg()">Reload from board</button>
<button onclick="saveCfg()">Save &amp; apply</button>
<button class="r" onclick="reboot()">Reboot</button>
<textarea id="cfg" spellcheck="false"></textarea></section>

<script>
const N=5,$=id=>document.getElementById(id);
let pos=[0,0,0,0,0],jv=[0,0,0,0,0],jt=null;
function log(t){const l=$('log');l.textContent=t+'\n'+l.textContent.slice(0,5000)}
async function cmd(c,quiet){
  try{const t=await (await fetch('/cmd?c='+encodeURIComponent(c))).text();
    if(!quiet||t!='ok')log('> '+c+'   '+t);return t}catch(e){log('network error')}}
async function poll(){
  try{const t=await (await fetch('/status')).text(),ls=t.split('\n');
    $('st').textContent=ls[0];
    const m=/P:([-\d,]+)/.exec(ls[0]);
    if(m){pos=m[1].split(',').map(Number);pos.forEach((p,j)=>$('p'+j).textContent=p)}
    ls.slice(1).filter(x=>x).forEach(log)}catch(e){$('st').textContent='disconnected'}
  setTimeout(poll,300)}
function sendJog(){cmd('J '+jv.join(' ')+' '+$('jacc').value,true)}
function jog(j,d){
  jv[j]=d*Number($('jspd').value);sendJog();
  if(d&&!jt)jt=setInterval(sendJog,200);
  if(!jv.some(v=>v)){clearInterval(jt);jt=null}}
function move(c){
  const v=[];for(let j=0;j<N;j++)v.push(Number($('t'+j).value)||0);
  cmd(c+' '+v.join(' ')+' '+$('mspd').value+' '+$('macc').value)}
function copyPos(){pos.forEach((p,j)=>$('t'+j).value=p)}
async function loadCfg(){$('cfg').value=await (await fetch('/config')).text()}
async function saveCfg(){
  const r=await fetch('/config',{method:'POST',headers:{'Content-Type':'text/plain'},body:$('cfg').value});
  log('config: '+await r.text())}
async function reboot(){if(confirm('Reboot board?')){await fetch('/reboot',{method:'POST'});log('rebooting...')}}
$('yf').onchange=e=>{const f=e.target.files[0];if(f)f.text().then(t=>{$('cfg').value=t;log('loaded '+f.name+', press Save')})};
const names=['J0 (X)','J1 (Y)','J2 (Z)','J3 (E0)','J4 (E1)'];
for(let j=0;j<N;j++){
  const tr=$('jt').insertRow(),mk=(txt,cls)=>{const b=document.createElement('button');b.textContent=txt;if(cls)b.className=cls;return b};
  tr.insertCell().textContent=names[j];
  const pc=tr.insertCell();pc.id='p'+j;pc.style.minWidth='70px';pc.textContent='0';
  const c=tr.insertCell();
  [[-1,'\u25C0 -'],[1,'+ \u25B6']].forEach(([d,t])=>{const b=mk(t);
    b.onpointerdown=e=>{b.setPointerCapture(e.pointerId);jog(j,d)};
    b.onpointerup=b.onpointercancel=()=>jog(j,0);
    b.oncontextmenu=e=>e.preventDefault();c.appendChild(b)});
  const ti=document.createElement('input');ti.id='t'+j;ti.value='0';tr.insertCell().appendChild(ti);
  if(j<3){const h=mk('Home','g');h.onclick=()=>cmd('H '+j);tr.insertCell().appendChild(h)}
}
window.onblur=()=>{if(jt){jv.fill(0);sendJog();clearInterval(jt);jt=null}};
poll();loadCfg();
</script></body></html>)HTML";
