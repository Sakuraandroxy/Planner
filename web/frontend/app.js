let state={version:0,status:"starting",task:"",step:0,max_steps:20,pose:[0,0,0],yaw:0,collided:false,reasoning:"",reasoning_summary:"",scene_analysis:"",candidates:[],selected:[],error:"",model_name:""};
const camImg=document.getElementById("camImage");
const camPh=document.getElementById("camPlaceholder");
const sStep=document.getElementById("sStep");
const sStatus=document.getElementById("sStatus");
const sScene=document.getElementById("sScene");
const sReasoning=document.getElementById("sReasoning");
const sCandidates=document.getElementById("sCandidates");
const sModel=document.getElementById("sModel");
const taskInput=document.getElementById("taskInput");
const taskBtn=document.getElementById("taskBtn");
const depthImg=document.getElementById("depthImage");
const depthPh=document.getElementById("depthPlaceholder");
depthImg.onload=function(){depthImg.style.display="block";if(depthPh)depthPh.style.display="none"};
depthImg.onerror=function(){depthImg.style.display="none";if(depthPh){depthPh.style.display="block";depthPh.textContent="depth load failed"}};
var es=new EventSource("/events");
es.onmessage=function(e){try{var s=JSON.parse(e.data);state=s;updateUI(s)}catch(err){console.warn(err)}};
function updateTask(){var v=taskInput.value.trim();if(!v)return;taskBtn.disabled=true;fetch("/task",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({task:v})}).then(function(r){return r.json()}).then(function(d){taskBtn.disabled=false;taskBtn.textContent="OK";taskInput.value=d.task||taskInput.value;setTimeout(function(){taskBtn.textContent="Update"},1500)}).catch(function(){taskBtn.disabled=false;taskBtn.textContent="Error";setTimeout(function(){taskBtn.textContent="Update"},1500)})}
taskInput.addEventListener("keydown",function(e){if(e.key==="Enter")updateTask()});
function updateUI(s){
if(s.frame_version>0){camImg.src="/frame?t="+s.frame_version;camImg.style.display="block";camPh.style.display="none"}
if(s.depth_version>0){depthImg.src="/depth_frame?t="+s.depth_version}else if(depthPh){depthPh.textContent="depth waiting... bytes="+(s.depth_bytes||0)}
sModel.textContent=s.model_name;
if(!s.model_name)sModel.style.display="none";
sStep.textContent=s.step+"/"+s.max_steps;
if(s.collided){sStatus.textContent="COLLISION";sStatus.className="badge badge-error"}else{sStatus.textContent=s.status;sStatus.className="badge badge-"+s.status}
sScene.textContent=s.scene_analysis||"-";
var txt=s.reasoning_summary||s.reasoning||"";
if(!txt)sReasoning.innerHTML='<span class="ph-muted">thinking...</span>';else sReasoning.textContent=txt;
if(s.candidates&&s.candidates.length>0){
var selStr=JSON.stringify(s.selected||[]);
var items=[];
for(var ci=0;ci<s.candidates.length;ci++){var c=s.candidates[ci];var d=c.delta||{};var isSel=JSON.stringify(c.actions)===selStr;items.push('<div class="cand-item'+(isSel?' selected':'')+'">'+(isSel?'<span class="cand-select-badge">SELECTED</span>':'')+'<div class="cand-actions">['+c.actions.join(", ")+']</div><div class="cand-delta">dx='+d.dx.toFixed(1)+' dy='+d.dy.toFixed(1)+' dz='+d.dz.toFixed(1)+' d\u03c6='+d.dphi.toFixed(1)+'</div></div>')}
sCandidates.innerHTML=items.join("\n")
}else{sCandidates.innerHTML='<span class="ph-muted">waiting for candidates...</span>'}
}
