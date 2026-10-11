#pragma once
#include <Arduino.h>

// Single-page UI served from flash. Talks to /api/* endpoints in WebServer.ino.
const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>Projector Dowser</title>
<style>
body{font:15px system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:12px;max-width:560px;margin:auto}
h1{font-size:18px}h2{font-size:14px;color:#9ab;margin:18px 0 6px;text-transform:uppercase}
.c{background:#1c1c1c;border-radius:8px;padding:12px;margin-bottom:12px}
button{background:#2a6;color:#fff;border:0;border-radius:6px;padding:12px 16px;font-size:16px;margin:2px}
button.s{background:#444}button.d{background:#a33}
.bar{height:14px;background:#333;border-radius:7px;overflow:hidden}.bar i{display:block;height:100%;background:#2a6;width:0}
.r{display:flex;justify-content:space-between;align-items:center;margin:6px 0}
input{width:120px;background:#000;color:#eee;border:1px solid #444;border-radius:4px;padding:6px}
input[type=range]{width:100%}#m{color:#fc6;min-height:1.2em}
</style></head><body>
<h1>Projector Dowser <small id=ip></small></h1>
<div class=c>
<div class=r><span>Position <b id=pos>-</b>&deg; &rarr; <b id=tgt>-</b>&deg;</span><span id=mv></span></div>
<div class=bar><i id=bar></i></div>
<div class=r><button onclick="mv('up')">Up</button><button onclick="mv('down')">Down</button></div>
<div class=r><span>Fade (s)</span><input id=mf type=number min=0 max=60 step=.1></div>
<input id=ang type=range min=0 max=180 oninput="al.textContent=this.value" onchange="mv(this.value)">
<div class=r><span>Angle <b id=al>-</b></span></div>
</div>
<div id=m></div>
<h2>Configuration</h2>
<div class=c id=cfg></div>
<div class=c><button onclick="reboot()" class=d>Reboot</button>
<small>Changes autosave after ~3 s. Network settings apply after reboot.</small></div>
<script>
const F=[['down','Down preset','n'],['up','Up preset','n'],['dnbound','Lower bound','n'],['upbound','Upper bound','n'],
['default','Default position','n'],['fade','Default fade (s)','n'],['heartbeat','Heartbeat (0/1)','n'],
['telemetry','Telemetry (0 reply,1 fixed,2 bcast)','n'],['targetip','Fixed target IP','t'],
['targetport0','Target port 0','n'],['targetport1','Target port 1','n'],['targetport2','Target port 2','n'],
['localport','Local OSC port*','n'],['static','Static IP (0/1)*','n'],['ip','Static IP*','t'],
['mask','Subnet mask*','t'],['gw','Gateway*','t']];
const $=i=>document.getElementById(i);
const msg=t=>{$('m').textContent=t;setTimeout(()=>$('m').textContent='',4000)};
async function api(m,u){try{const r=await fetch(u,{method:m});const j=await r.json();
if(!r.ok)msg(j.error||'Error');return j}catch(e){msg('Device unreachable')}}
function mv(to){const f=$('mf').value;api('POST','/api/move?to='+to+(f!==''?'&fade='+f:''))}
async function poll(){const s=await api('GET','/api/status');if(!s)return;
$('pos').textContent=s.pos;$('tgt').textContent=s.target;$('mv').textContent=s.moving?'moving':'';
$('ip').textContent=s.ip;
$('bar').style.width=(100*(s.pos-s.dnbound)/Math.max(1,s.upbound-s.dnbound))+'%'}
async function load(){const c=await api('GET','/api/config');if(!c)return;
$('cfg').innerHTML=F.map(([k,l,t])=>`<div class=r><label>${l}</label><input id=k_${k} value="${c[k]}" onchange="save('${k}',this.value)"></div>`).join('');
$('mf').placeholder=c.fade;$('ang').min=c.dnbound;$('ang').max=c.upbound}
async function save(k,v){const r=await api('POST','/api/config?'+k+'='+encodeURIComponent(v));
if(r&&r.ok){msg('Saved '+k);load()}}
async function reboot(){if(confirm('Reboot device?')){await api('POST','/api/reboot');msg('Rebooting...')}}
load();poll();setInterval(poll,500);
</script></body></html>)HTML";
