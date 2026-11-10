import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {OBJLoader} from 'three/addons/loaders/OBJLoader.js';
const $=id=>document.getElementById(id),classColors={desconocido:'#8b959a',grava:'#bcac88',roca:'#b4c4cf',plantas:'#65b784',barro:'#97694a',tierra:'#c79664'};
const specs={pose:[['x','X · m',0],['y','Y · m',0],['z','Z · m',1.2],['roll','Balanceo · °',0],['pitch','Cabeceo · °',0],['yaw','Guiñada · °',0]],velocity:[['vx','Vx · m/s',0],['vy','Vy · m/s',0],['vz','Vz · m/s',0],['wx','ωx · rad/s',0],['wy','ωy · rad/s',0],['wz','ωz · rad/s',0]],physics:[['speed','Velocidad tangencial · m/s',0],['steering','Dirección · °',0],['friction','Fricción general · μ',.7],['stiffness','Rigidez general · N/m',1000000]]};
for(const [section,items]of Object.entries(specs))for(const[id,label,val]of items){const el=document.createElement('label');el.textContent=label;const input=document.createElement('input');Object.assign(input,{id,type:'number',value:val,step:'any'});if(['x','y','z','roll','pitch'].includes(id)){input.readOnly=true;input.title='Selecciona la posición con el ratón sobre la nube'};input.oninput=()=>{if(id==='yaw')return;updateVehicleSettings()};if(id==='yaw')input.onchange=guarded(()=>rotateVehicle(Number(input.value)));el.append(input);$(section).append(el)}
let recordedPath=[],info=null,frames=[],index=0,playing=false,last=0,selected=null,route=[],terrain={zones:[],classes:{desconocido:{friction:.7,stiffness:1e6},grava:{friction:.65,stiffness:8e5},roca:{friction:.85,stiffness:2e6},plantas:{friction:.45,stiffness:3e5},barro:{friction:.25,stiffness:1e5},tierra:{friction:.6,stiffness:6e5}}},overview=[],cloudPoints=[],cloudLabels=[],cloud=null,vehicleDots=null,meshVersion=0,csvAvailable=false,appliedPose=null,busy=false;
let renderPending=false,mapPending=false,arrowCursor=0,metricValues=[],mapBoundsSource=null,mapBoundsValue=null,terrainRevision=0;
const arrowPool=[],arrowDirection=new THREE.Vector3(),mapBackgrounds=new Map();
function fail(e){const message=e.message||String(e);$('error').textContent=message;const dialog=document.querySelector('dialog[open]');if(dialog){let alert=dialog.querySelector('.dialogError');if(!alert){alert=document.createElement('p');alert.className='dialogError notice';alert.style.color='#ffb19c';alert.setAttribute('role','alert');dialog.querySelector('.body').prepend(alert)}alert.textContent=message}}
async function request(url,body){const r=await fetch(url,body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const data=await r.json();if(!r.ok)throw Error(data.error||`Error ${r.status}`);return data}
function guarded(fn){return async e=>{if(busy)return;try{$('error').textContent='';document.querySelectorAll('.dialogError').forEach(el=>el.remove());await fn(e)}catch(err){fail(err)}}}
for(const button of document.querySelectorAll('[data-modal]'))button.onclick=()=>{const d=$(button.dataset.modal);d.showModal();button.setAttribute('aria-expanded','true');if(d.id==='resultsDialog'){renderMetrics();selection()}if(d.id==='routeDialog'||d.id==='terrainDialog')requestAnimationFrame(drawMaps)};
for(const button of document.querySelectorAll('.close,.closeAction'))button.onclick=()=>button.closest('dialog').close();
for(const dialog of document.querySelectorAll('dialog'))dialog.addEventListener('close',()=>{document.querySelector(`[data-modal="${dialog.id}"]`)?.setAttribute('aria-expanded','false')});
for(const dialog of document.querySelectorAll('dialog'))dialog.addEventListener('click',e=>{if(e.target===dialog){const r=dialog.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)dialog.close()}});
const host=$('scene'),scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(50,1,.05,100000);camera.up.set(0,0,1);camera.position.set(14,-18,14);const renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));host.prepend(renderer.domElement);const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;
const grid=new THREE.GridHelper(100,50,0x424b5b,0x2c3340);grid.rotation.x=Math.PI/2;scene.add(grid,new THREE.AxesHelper(2));const vehicleGroup=new THREE.Group(),vectors=new THREE.Group(),paths=new THREE.Group(),chassis=new THREE.Group();scene.add(vehicleGroup,vectors,paths,chassis);chassis.visible=false;const headingArrow=new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(),2.5,0x64b5ff,.55,.3);headingArrow.visible=false;scene.add(headingArrow);scene.add(new THREE.HemisphereLight(0xd6eeff,0x334250,2));const light=new THREE.DirectionalLight(0xffffff,2);light.position.set(10,-10,20);scene.add(light);
new ResizeObserver(()=>{const w=host.clientWidth,h=host.clientHeight;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();requestRender()}).observe(host);
controls.addEventListener('change',requestRender);
function dispose(group){for(const child of [...group.children]){group.remove(child);child.traverse(c=>{c.geometry?.dispose();if(Array.isArray(c.material))c.material.forEach(m=>m.dispose());else c.material?.dispose()})}}
function loadMesh(){
  const version=++meshVersion;dispose(chassis);chassis.visible=false;
  if(appliedPose?.wheel_centers){for(const center of appliedPose.wheel_centers){const tire=new THREE.Mesh(new THREE.CylinderGeometry(appliedPose.tire_radius,appliedPose.tire_radius,appliedPose.tire_width,24),new THREE.MeshPhongMaterial({color:0x333b46}));tire.position.fromArray(center);chassis.add(tire)}}
  const pose=frames[index]||appliedPose;
  if(pose)updateChassis(pose);
  const match=(appliedPose?.vehicle||$('vehicle').value).match(/(5t|7t|10t)/);if(!match)return;
  new OBJLoader().load(`/assets/data/vehicle/MAN/meshes/MAN_${match[1]}_chassis.obj`,obj=>{
    if(version!==meshVersion)return;
    obj.traverse(c=>{if(c.isMesh)c.material=new THREE.MeshPhongMaterial({color:0x567492,transparent:true,opacity:.7,side:THREE.DoubleSide})});
    chassis.add(obj);updateChassis(frames[index]||appliedPose);requestRender();
  },undefined,()=>{if(version===meshVersion)fail(Error('No se pudo cargar el modelo del vehículo'))});
  requestRender();
}
function yawOf(pose){if(!pose?.orientation)return 0;const [w,x,y,z]=pose.orientation;return Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))}
function updateChassis(f){
  chassis.visible=Boolean(f?.position&&f?.orientation);headingArrow.visible=chassis.visible;
  if(chassis.visible){chassis.position.fromArray(f.position);const q=f.orientation;chassis.quaternion.set(q[1],q[2],q[3],q[0]).normalize();
    const yaw=yawOf(f);headingArrow.position.fromArray([f.position[0],f.position[1],f.position[2]+1.8]);headingArrow.setDirection(new THREE.Vector3(Math.cos(yaw),Math.sin(yaw),0));
  }
}
const fmt=(v,d=2)=>Number(v||0).toLocaleString('es',{maximumFractionDigits:d,minimumFractionDigits:d}),vec=v=>(v||[0,0,0]).map(x=>fmt(x)).join(' / ');
function getClass(point,base='desconocido'){let name=base;for(const zone of terrain.zones)if(Math.hypot(...point.map((n,i)=>n-zone.center[i]))<=zone.radius)name=zone.class;return classColors[name]?name:'desconocido'}
function updatePointPositions(object,points){
  const geometry=object.geometry;
  let position=geometry.getAttribute('position');
  if(!position||position.count!==points.length){if(position)geometry.dispose();position=new THREE.Float32BufferAttribute(new Float32Array(points.length*3),3);position.setUsage(THREE.DynamicDrawUsage);geometry.setAttribute('position',position)}
  for(let i=0;i<points.length;i++)position.array.set(points[i],i*3);
  position.needsUpdate=true;geometry.computeBoundingSphere();
}
function recolorCloud(){
  if(!cloud)return;
  let colors=cloud.geometry.getAttribute('color');
  if(!colors||colors.count!==cloudPoints.length){if(colors)cloud.geometry.dispose();colors=new THREE.Float32BufferAttribute(new Float32Array(cloudPoints.length*3),3);colors.setUsage(THREE.DynamicDrawUsage);cloud.geometry.setAttribute('color',colors)}
  const palette=Object.fromEntries(Object.entries(classColors).map(([name,color])=>[name,new THREE.Color(color)])),mapping=info?.cloud?.label_mapping||{};
  for(let i=0;i<cloudPoints.length;i++){const label=cloudLabels[i],base=typeof label==='string'&&classColors[label]?label:mapping[String(label)]||'desconocido',color=palette[getClass(cloudPoints[i],base)];colors.array[i*3]=color.r;colors.array[i*3+1]=color.g;colors.array[i*3+2]=color.b}
  colors.needsUpdate=true;requestRender();
}
function showCloud(data){
  cloudPoints=data.points||[];cloudLabels=data.labels||[];
  if(!cloud){cloud=new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({size:.06,vertexColors:true}));scene.add(cloud)}
  updatePointPositions(cloud,cloudPoints);recolorCloud();
  $('cloudInfo').textContent=`${cloudPoints.length.toLocaleString('es')} puntos · mapa completo`;
}
function fit(){
  const bounds=cloud?.geometry.boundingSphere,center=bounds?.center||new THREE.Vector3(...(info?.center||[0,0,0]));
  const radius=Math.max(bounds?.radius||0,1),halfFov=THREE.MathUtils.degToRad(camera.fov/2),angle=Math.min(halfFov,Math.atan(Math.tan(halfFov)*camera.aspect)),distance=radius/Math.sin(angle)*1.1;
  controls.target.copy(center);camera.position.copy(center).add(new THREE.Vector3(9,-12,9).normalize().multiplyScalar(distance));
  camera.far=Math.max(100000,distance+radius*2);camera.updateProjectionMatrix();controls.update();requestRender();
}
function setCatalog(data){
  const previous=$('vehicle').value;const catalog=data.catalog||[];$('vehicle').replaceChildren();
  for(const item of catalog){if(item.compatible===false||['excluded','pending'].includes(item.status))continue;const option=document.createElement('option');option.value=item.id;option.textContent=item.name.replace(' · rígido experimental','');$('vehicle').append(option)}
  if([...$('vehicle').options].some(o=>o.value===previous))$('vehicle').value=previous;
  else if(data.model?.id)$('vehicle').value=data.model.id;
  $('modelName').textContent=$('vehicle').selectedOptions[0]?.textContent||'Sin colocar';loadMesh();
}
async function loadInfo(data,{reset=true}={}){const changed=Boolean(info?.cloud?.sha256&&info.cloud.sha256!==data.cloud?.sha256);if(changed){route=[];terrain.zones=[];renderRoute();renderTerrain()}info=data;overview=data.points||data.overview||[];setCatalog(data);if(reset){const middle=data.center||overview[0]||[0,0,0];const c=overview.length?overview.reduce((best,p)=>Math.hypot(p[0]-middle[0],p[1]-middle[1])<Math.hypot(best[0]-middle[0],best[1]-middle[1])?p:best,overview[0]):middle;['x','y'].forEach((id,i)=>$(id).value=c[i]);$('z').value=c[2]+1.2;frames=[];appliedPose=null;route=[];headingArrow.visible=false;recordedPath=[];csvAvailable=false;updateCsvDownload();playing=false;chassis.visible=false;vehicleDots=null;dispose(vehicleGroup);resetVectors();$('timeline').max=0;$('time').textContent='0,00 s';$('liveMetric').textContent='En espera';$('metrics').replaceChildren();metricValues=[];$('warnings').textContent='Sin simulación';$('selection').textContent='Selecciona un punto del vehículo para inspeccionar sus fuerzas.';drawPaths()}if(overview.length||data.cloud){showCloud(data);fit();$('status').textContent='Mapa listo'}else $('status').textContent='Carga una nube PLY para comenzar';syncControls();renderRoute();renderStability()}
function drawPaths(){dispose(paths);const add=(points,color)=>{if(points.length>1)paths.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points.map(p=>new THREE.Vector3(...p))),new THREE.LineBasicMaterial({color})))};add(route.map(p=>[p.x,p.y,p.z]),0x51d8e5);add((frames.length?frames:recordedPath).map(f=>f.position).filter(Boolean),0xf5bc58);requestRender()}
function values(){
  const result={};for(const items of Object.values(specs))for(const[id]of items){const v=Number($(id).value);if(!Number.isFinite(v))throw Error('Introduce un número válido: '+id);result[id]=v}
  return {...result,dt:.01,duration:2};
}
function validateRoute(){for(const p of route)if(!['x','y','z','speed','acceleration'].every(k=>Number.isFinite(p[k]))||p.speed<0||p.speed>100||p.acceleration< -20||p.acceleration>20)throw Error('Ruta: coordenadas finitas, velocidad entre 0 y 100 m/s y aceleración entre −20 y 20 m/s²')}
function restorePose(pose){for(const items of Object.values(specs))for(const[id]of items)if(pose[id]!==undefined)$(id).value=pose[id]}
function syncControls(){
  const loaded=Boolean(info?.cloud);
  $('simulate').disabled=busy||!loaded||!appliedPose;
  $('heading').disabled=$('rotateLeft').disabled=$('rotateRight').disabled=busy||!appliedPose;
  $('rotationControl').hidden=!appliedPose;
  $('saveProject').disabled=busy||!loaded;
  $('loadProject').disabled=busy;$('file').disabled=busy;
  $('loadCloud').disabled=busy||!$('file').files.length;
  $('play').disabled=busy||frames.length<2;$('timeline').disabled=busy||frames.length<2;
}
function updateCsvDownload(){
  const link=$('downloadCsv');link.setAttribute('aria-disabled',String(!csvAvailable));
  if(csvAvailable){link.href='/api/export.csv';link.download='fusionforce.csv';link.removeAttribute('tabindex')}
  else{link.removeAttribute('href');link.removeAttribute('download');link.setAttribute('tabindex','-1')}
  $('csvState').textContent=csvAvailable?'CSV disponible':'';
}
function project(){anchorRoute();validateRoute();return {vehicle:$('vehicle').value,pose:values(),applied_pose:appliedPose,route,terrain,frames,playback_index:index,csv_available:csvAvailable,trajectory:frames.length?frames.map(f=>({time:f.time,position:f.position,orientation:f.orientation,velocity:f.velocity})):recordedPath,warnings:$('warnings').textContent}}
function invalidateResults(){
  frames=[];recordedPath=[];index=0;playing=false;selected=null;csvAvailable=false;
  dispose(vehicleGroup);vehicleDots=null;resetVectors();$('metrics').replaceChildren();metricValues=[];
  $('warnings').textContent='Sin simulación';$('selection').textContent='Selecciona un punto del vehículo para inspeccionar sus fuerzas.';
  $('timeline').max=0;$('timeline').value=0;$('time').textContent='0,00 s';$('play').textContent='▶';$('liveMetric').textContent='Clic para colocar · ⇧ clic para orientar';
  updateCsvDownload();updateChassis(appliedPose);syncControls();renderStability();
}
function normalizeHeading(degrees){return ((degrees+180)%360+360)%360-180}
function syncHeading(){const yaw=appliedPose?THREE.MathUtils.radToDeg(yawOf(appliedPose)):0;$('heading').value=Math.round(yaw);$('headingValue').textContent=`${Math.round(yaw)}°`}
function updateVehicleSettings(){
  if(busy||!appliedPose)return;
  try{appliedPose.values={...values(),x:appliedPose.position[0],y:appliedPose.position[1],z:appliedPose.position[2]};invalidateResults();drawPaths();drawMaps()}catch(e){fail(e)}
}
function anchorRoute(){
  if(!appliedPose){route=[];return}
  const [x,y,z]=appliedPose.position,old=route[0]||{};
  const first={x,y,z,speed:old.speed??1,acceleration:old.acceleration??.5,yaw:THREE.MathUtils.radToDeg(yawOf(appliedPose))};
  if(!route.length)route.push(first);else route[0]=first;
}
function placementSupportKey(){return JSON.stringify(['ground-contact-v2',terrain,Number($('friction').value),Number($('stiffness').value)])}
async function placeVehicle(x,y,yaw=Number($('yaw').value)){
  if(!info?.cloud)throw Error('Carga una nube de puntos');
  return runBusy('Calculando apoyo del vehículo…',async()=>{
    const previous=appliedPose,previousValues=previous?.values;
    try{
      const data=await request('/api/place',{...values(),x,y,yaw:normalizeHeading(yaw),vehicle:$('vehicle').value,terrain});
      const p={...values(),x:data.position[0],y:data.position[1],z:data.position[2],roll:data.roll,pitch:data.pitch,yaw:data.yaw};
      appliedPose={...data,values:p,vehicle:$('vehicle').value,supportKey:placementSupportKey()};restorePose(p);syncHeading();invalidateResults();anchorRoute();renderRoute();loadMesh();
      $('modelName').textContent=$('vehicle').selectedOptions[0]?.textContent;$('status').textContent='Vehículo apoyado sobre el terreno';completeProgress();return data;
    }catch(e){
      if(previousValues)restorePose(previousValues);if(previous)$('vehicle').value=previous.vehicle;syncHeading();
      throw e;
    }
  });
}
async function rotateVehicle(yaw){if(!appliedPose)return;return placeVehicle(appliedPose.position[0],appliedPose.position[1],yaw)}
async function runBusy(label,fn){
  if(busy)return;busy=true;playing=false;$('play').textContent='▶';syncControls();
  const locked=[...document.querySelectorAll('dialog input,dialog select,dialog button:not(.close),nav button')].map(el=>[el,el.disabled]);for(const [el]of locked)el.disabled=true;
  const progress=$('loadProgress');progress.hidden=false;progress.removeAttribute('value');$('progressLabel').hidden=false;$('progressLabel').textContent=label;$('status').textContent=label;
  try{return await fn()}catch(e){progress.hidden=true;$('progressLabel').textContent=e.message;$('status').textContent='No se pudo completar la operación';throw e}
  finally{for(const [el,disabled]of locked)el.disabled=disabled;busy=false;syncControls()}
}
function upload(url,file){
  return new Promise((resolve,reject)=>{
    const xhr=new XMLHttpRequest();xhr.open('POST',url);xhr.setRequestHeader('Content-Type','application/octet-stream');
    xhr.upload.onprogress=e=>{if(e.lengthComputable){$('loadProgress').max=100;$('loadProgress').value=e.loaded/e.total*100;$('progressLabel').textContent=`Transferencia · ${Math.round(e.loaded/e.total*100)} %`}};
    xhr.upload.onload=()=>{$('loadProgress').removeAttribute('value');$('status').textContent='Preparando mapa…'};
    xhr.onload=()=>{try{const data=JSON.parse(xhr.responseText);if(xhr.status<200||xhr.status>=300)throw Error(data.error||'No se pudo importar el archivo');resolve(data)}catch(e){reject(e)}};
    xhr.onerror=()=>reject(Error('No se pudo conectar con el visor'));xhr.onabort=()=>reject(Error('Importación cancelada'));xhr.send(file);
  });
}
function completeProgress(){$('loadProgress').value=100;$('loadProgress').setAttribute('aria-valuetext','Completado · 100 %');$('progressLabel').textContent='Completado · 100 %'}
function showSimulationProgress(progress){
  const p=progress||{},fraction=Math.max(0,Math.min(.999,Number(p.fraction)||0)),percent=Math.floor(fraction*100);
  $('loadProgress').value=percent;
  const stages={preparation:'Preparando simulación',simulation:'Simulando',finalizing:'Guardando resultados',complete:'Completado'};
  const stage=stages[p.stage]||'Calculando simulación';
  const steps=Number.isFinite(p.processed)&&p.total?` · ${p.processed}/${p.total} pasos`:'';
  const destination=p.until_destination&&Number.isFinite(p.distance_remaining);
  const detail=destination?`${stage} · ${percent} % · ${p.processed||0} pasos · ${p.estimated_steps||p.total} previstos · ${fmt(p.time,1)} s · destino a ${fmt(p.distance_remaining,1)} m`:`${stage} · ${percent} %${steps}`;
  $('progressLabel').textContent=detail;$('loadProgress').setAttribute('aria-valuetext',detail);$('loadProgress').title=destination?`Paso: 0,01 s · ${p.estimated_steps||p.total} pasos previstos para la ruta`:detail;$('status').textContent=stage;
}
async function simulationJob(payload){
  const started=await request('/api/simulation/start',payload),id=started.job_id;
  if(!id)throw Error('El servidor no ha iniciado la simulación');
  for(;;){
    const job=await request(`/api/simulation/job?job_id=${encodeURIComponent(id)}`);
    if(job.job_id!==id)throw Error('La simulación ha cambiado; vuelve a intentarlo');
    showSimulationProgress(job.progress);
    if(job.status==='complete'){if(!job.result)throw Error('La simulación terminó sin resultados');return job.result}
    if(['error','cancelled'].includes(job.status))throw Error(job.error||'La simulación no se pudo completar');
    await new Promise(resolve=>setTimeout(resolve,250));
  }
}
async function simulate(){
  if(!appliedPose)throw Error('Coloca el vehículo antes de simular');
  if(appliedPose.supportKey!==placementSupportKey())await placeVehicle(appliedPose.position[0],appliedPose.position[1],THREE.MathUtils.radToDeg(yawOf(appliedPose)));
  anchorRoute();validateRoute();
  await runBusy('Calculando simulación…',async()=>{
    csvAvailable=false;updateCsvDownload();const payload={...appliedPose.values,vehicle:appliedPose.vehicle,dt:.01,route:route.length>1?route:[],until_destination:route.length>1,terrain};
    const data=await simulationJob(payload);frames=data.frames||[];selected=null;index=0;recordedPath=[];
    csvAvailable=Boolean(data.csv);updateCsvDownload();$('timeline').max=Math.max(0,frames.length-1);drawPaths();renderFrame(0);
    const warnings=[...(data.warnings||[]),data.stop_reason].filter(Boolean);if(data.stopped_reason&&!warnings.length)warnings.push({stall:'Destino no alcanzado: el vehículo no avanza',timeout:'Destino no alcanzado: se agotó el tiempo de cálculo',coverage:'Destino no alcanzado: faltan datos de terreno'}[data.stopped_reason]||'La simulación se detuvo antes del destino');$('warnings').textContent=warnings.join('\n')||(data.status==='destination_reached'?'Destino alcanzado':'Simulación completada');
    $('status').textContent=warnings.length?'Simulación con avisos':data.status==='destination_reached'?'Destino alcanzado':'Simulación completada';completeProgress();if(data.status==='destination_reached')$('progressLabel').textContent=`Destino alcanzado · ${fmt(frames.at(-1)?.time,1)} s · 100 %`;
    playing=frames.length>1;last=performance.now();$('play').textContent=playing?'Ⅱ':'▶';requestRender();
    for(const dialog of document.querySelectorAll('dialog[open]'))dialog.close();
  });
}
function resetVectors(){for(const helper of arrowPool)helper.visible=false;arrowCursor=0}
function arrow(point,value,color,scale){
  if(!value)return;
  arrowDirection.fromArray(value);const magnitude=arrowDirection.length();
  if(!(magnitude>1e-9))return;
  let helper=arrowPool[arrowCursor++];
  if(!helper){helper=new THREE.ArrowHelper(new THREE.Vector3(0,1,0),new THREE.Vector3(),1,color,.14,.07);arrowPool.push(helper);vectors.add(helper)}
  helper.visible=true;helper.position.fromArray(point);helper.setDirection(arrowDirection.normalize());helper.setLength(Math.min(5,Math.log10(1+magnitude)*scale),.14,.07);helper.setColor(color);
}
function selection(){if(selected===null||!frames[index])return;const f=frames[index],id=f.contact_ids?.[selected],cornerNames={front_left:'Esquina delantera izquierda',front_right:'Esquina delantera derecha',rear_left:'Esquina trasera izquierda',rear_right:'Esquina trasera derecha'},label=cornerNames[id]||`Punto ${id||selected+1}`;$('selection').textContent=`${label}\nPosición: ${vec(f.points[selected])} m\nNormal: ${vec(f.normal?.[selected])} N\nFricción: ${vec(f.friction?.[selected])} N\nMomento: ${vec(f.moments?.[selected])} N·m\nContacto (peso): ${fmt(f.contact?.[selected],4)}`}
const stabilityLabels={
  fz_left_N:'Fz,L · N',fz_right_N:'Fz,R · N',fz_total_N:'Fz,L + Fz,R · N',
  h_m:'h · altura nominal del CM · m',track_m:'tw · ancho de vía · m',gravity_m_s2:'g · m/s²',ay_body_m_s2:'ay · aceleración lateral local · m/s²',phi_rad:'φ · balanceo · rad',ay_dot_m_s3:'d(ay)/dt · m/s³',phi_dot_rad_s:'dφ/dt · rad/s',horizon_s:'Δt · horizonte · s',derivative_dt_s:'Paso de la derivada · s',base_ltr_kinematic:'2h/tw · (ay/g + φ)',prediction_term:'Término predictivo',
  mass_kg:'m · masa del chasis · kg',half_track_m:'b · semiancho de vía · m',distance_m:'d · distancia al pivote · m',inertia_cg_kg_m2:'ICG · inercia de balanceo · kg·m²',inertia_pivot_kg_m2:'Ip · inercia en pivote · kg·m²',omega_body_x_rad_s:'ωx · balanceo local · rad/s',phi_critical_rad:'φc · umbral nominal · rad',omega_critical_rad_s:'ωc · umbral nominal · rad/s',k1:'k₁ · coeficiente nominal',k2:'k₂ · coeficiente nominal',roll_term:'k₁ · |φ| / φc',omega_term:'k₂ · (ωx / ωc)²',
  fx_N:'Fx · N',fy_N:'Fy · N',fz_N:'Fz · N',mx_Nm:'Mx · N·m',my_Nm:'My · N·m',mz_Nm:'Mz · N·m',z_ref_m:'zref · plano respecto al CM · m',x_ref_m:'x(zref) · avance · m',y_ref_m:'y(zref) · izquierda · m',dx_dz:'dx/dz = Fx/Fz',dy_dz:'dy/dz = Fy/Fz',torsion_residual_Nm:'τres · torsión residual · N·m',cog_x_m:'CM · X global · m',cog_y_m:'CM · Y global · m',cog_z_m:'CM · Z global · m',yaw_rad:'Rumbo del marco · rad'
};
let stabilityRender={frames:null,index:-1,width:0};
function stabilityNumber(value){return Number.isFinite(value)?value.toLocaleString('es',{maximumFractionDigits:5}):'No disponible'}
function stabilityPlotData(key,component=null){
  return frames.slice(0,index+1).map(f=>{
    const value=f.stability?.[key]?.value,v=component===null?value:Array.isArray(value)?value[component]:null;
    return [f.time,Number.isFinite(v)?v:null];
  });
}
// Keep extrema in each pixel-sized bucket, and keep missing samples as gaps.
function stabilityChartSamples(data,budget){
  const size=Math.max(1,Math.ceil(data.length/Math.max(1,budget))),out=[];
  let bucket=[];
  const flush=()=>{if(!bucket.length)return;let lo=bucket[0],hi=lo;for(const p of bucket){if(p[1]<lo[1])lo=p;if(p[1]>hi[1])hi=p}const chosen=[bucket[0],lo,hi,bucket[bucket.length-1]].sort((a,b)=>a[0]-b[0]);for(const p of chosen)if(out.at(-1)!==p)out.push(p);bucket=[]};
  for(const point of data){if(point[1]===null||!Number.isFinite(point[0])){flush();if(out.at(-1)!==null)out.push(null)}else{bucket.push(point);if(bucket.length>=size)flush()}}flush();return out;
}
function drawStabilityChart(id,series){
  const canvas=$(id),ctx=canvas.getContext('2d'),width=canvas.clientWidth||320,height=canvas.clientHeight||66,ratio=Math.min(devicePixelRatio||1,2);
  canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,width,height);
  const left=40,right=8,top=8,bottom=17,w=Math.max(1,width-left-right),h=Math.max(1,height-top-bottom);
  let minimum=0,maximum=0,first=Infinity,last=-Infinity,count=0;
  for(const {data} of series)for(const [time,value]of data)if(value!==null&&Number.isFinite(time)){minimum=Math.min(minimum,value);maximum=Math.max(maximum,value);first=Math.min(first,time);last=Math.max(last,time);count++}
  if(!count){ctx.fillStyle='#8f98a9';ctx.font='10px system-ui';ctx.fillText('Sin datos en este intervalo',left,top+h/2);return}
  const span=maximum-minimum,pad=span>1e-12?span*.12:Math.max(.1,Math.abs(maximum)*.1);minimum-=pad;maximum+=pad;
  const start=Number.isFinite(frames[0]?.time)?frames[0].time:first,end=Math.max(Number.isFinite(frames[index]?.time)?frames[index].time:last,start+.01);
  const x=t=>left+(t-start)/(end-start)*w,y=v=>top+(maximum-v)/(maximum-minimum)*h;
  ctx.font='9px system-ui';ctx.strokeStyle='#3a4353';ctx.fillStyle='#a8b1c1';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(left,top);ctx.lineTo(left,top+h);ctx.lineTo(left+w,top+h);ctx.stroke();
  ctx.fillText(stabilityNumber(maximum),2,top+7);ctx.fillText(stabilityNumber(minimum),2,top+h);ctx.fillText(stabilityNumber(start)+' s',left,height-3);ctx.textAlign='right';ctx.fillText(stabilityNumber(end)+' s',width-right,height-3);ctx.textAlign='left';
  if(minimum<=0&&maximum>=0){ctx.strokeStyle='#454e60';ctx.setLineDash([3,3]);ctx.beginPath();ctx.moveTo(left,y(0));ctx.lineTo(left+w,y(0));ctx.stroke();ctx.setLineDash([])}
  for(const {data,color}of series){ctx.strokeStyle=color;ctx.lineWidth=1.5;ctx.beginPath();let connected=false;for(const point of stabilityChartSamples(data,Math.ceil(w))){if(point===null){connected=false;continue}if(connected)ctx.lineTo(x(point[0]),y(point[1]));else ctx.moveTo(x(point[0]),y(point[1]));connected=true}ctx.stroke();const latest=data.at(-1);if(latest?.[1]!==null&&Number.isFinite(latest?.[1])){ctx.fillStyle=color;ctx.beginPath();ctx.arc(x(latest[0]),y(latest[1]),2,0,Math.PI*2);ctx.fill()}}
}
function renderStability(force=false){
  const width=$('loadChart').clientWidth;
  if(!force&&stabilityRender.frames===frames&&stabilityRender.index===index&&stabilityRender.width===width)return;
  stabilityRender={frames,index,width};
  const frame=frames[index],stability=frame?.stability;
  $('stabilityTime').textContent=frame&&Number.isFinite(frame.time)?stabilityNumber(frame.time)+' s':'Sin simulación';
  $('stabilityState').textContent=!frame?'Simula para ver la evolución de los índices.':!stability?'Este resultado no contiene índices. Vuelve a simular para calcularlos.':'Curvas hasta el instante seleccionado · valores sin recortar.';
  for(const key of ['ltr','pltr','zml','si']){
    const entry=stability?.[key],value=entry?.value;
    const valid=key==='zml'?Array.isArray(value)&&value.length===2&&value.every(Number.isFinite):Number.isFinite(value);
    $(key+'Value').textContent=valid?(key==='zml'?'X '+stabilityNumber(value[0])+' / Y '+stabilityNumber(value[1])+' m':stabilityNumber(value)):'No disponible';
    $(key+'Reason').textContent=entry?.reason||(!frame?'Sin simulación.':!entry?'No calculado en este resultado.':!valid?'Valor no disponible.':'');
    const params=$(key+'Params');params.replaceChildren();
    for(const [name,param]of Object.entries(entry?.params||{})){const label=document.createElement('dt'),number=document.createElement('dd');label.textContent=stabilityLabels[name]||name;number.textContent=stabilityNumber(param);params.append(label,number)}
  }
  drawStabilityChart('loadChart',[{data:stabilityPlotData('ltr'),color:'#58d9bb'},{data:stabilityPlotData('pltr'),color:'#ffbf69'}]);
  drawStabilityChart('zmlChart',[{data:stabilityPlotData('zml',0),color:'#78b7ff'},{data:stabilityPlotData('zml',1),color:'#cb9cff'}]);
  drawStabilityChart('siChart',[{data:stabilityPlotData('si'),color:'#72d3e5'}]);
}
function renderMetrics(){const f=frames[index];if(!f||!$('resultsDialog').open)return;let row=0;for(const [label,value]of [['Tiempo',fmt(f.time)+' s'],['Posición · m',vec(f.position)],['Velocidad · m/s',vec(f.velocity)],['Aceleración · m/s²',vec(f.acceleration)],['Aceleración angular · rad/s²',vec(f.angular_acceleration)],['Par total · N·m',vec(f.torque)],['Objetivo · m/s',fmt(f.target?.speed)],['Error de seguimiento XYZ · m',fmt(f.tracking_error)],['Error de velocidad · m/s',fmt(f.speed_error)],['Aceleración tangencial · m/s²',fmt(f.tangential_acceleration)],['Error de aceleración · m/s²',fmt(f.acceleration_error)]]){let b=metricValues[row++];if(!b){const el=document.createElement('div');el.className='metric';const a=document.createElement('span');b=document.createElement('b');a.textContent=label;el.append(a,b);$('metrics').append(el);metricValues.push(b)}b.textContent=value}}
function renderFrame(i){if(!frames.length){renderStability();return}index=Math.max(0,Math.min(i,frames.length-1));const f=frames[index];updateChassis(f);resetVectors();const points=f.points||[];if(!vehicleDots){vehicleDots=new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({color:0xf4f7fc,size:.13}));vehicleGroup.add(vehicleDots)}updatePointPositions(vehicleDots,points);points.forEach((p,j)=>{arrow(p,f.normal?.[j],0x50dcba,.45);arrow(p,f.friction?.[j],0xffb957,.45);arrow(p,f.moments?.[j],0xc095ff,.35)});$('timeline').value=index;$('time').textContent=fmt(f.time)+' s';$('liveMetric').textContent=`t ${fmt(f.time)} s · ${(f.contact||[]).filter(v=>v>.5).length} contactos`;renderMetrics();renderStability();selection();drawMaps();requestRender()}
function renderRoute(){anchorRoute();$('routeCount').textContent=`${route.length} poses · inicio fijo`;$('undoWaypoint').disabled=route.length<=1;$('clearRoute').disabled=route.length<=1;drawMaps();drawPaths()}
function renderTerrain(){terrainRevision++;recolorCloud();const rows=$('classRows');rows.replaceChildren();$('zoneClass').replaceChildren();$('terrainLegend').replaceChildren();for(const [name,color]of Object.entries(classColors)){const option=document.createElement('option');option.value=name;option.textContent=name;$('zoneClass').append(option);const span=document.createElement('span');span.style.color=color;span.textContent='● '+name;$('terrainLegend').append(span);terrain.classes[name]??={friction:.7,stiffness:1e6};const tr=document.createElement('tr'),label=document.createElement('td');label.textContent=name;label.style.color=color;tr.append(label);for(const key of ['friction','stiffness']){const td=document.createElement('td'),input=document.createElement('input');Object.assign(input,{type:'number',value:terrain.classes[name][key],step:'any',min:key==='friction'?0:1000,max:key==='friction'?3:1e8});input.setAttribute('aria-label',`${name}: ${key}`);input.oninput=()=>{if(busy)return;terrain.classes[name][key]=Number(input.value);if(name==='desconocido')$(key).value=input.value};td.append(input);tr.append(td)}rows.append(tr)}$('zoneCount').textContent=`${terrain.zones.length} zonas`;drawMaps()}
function refreshTerrainView(){renderTerrain()}
function mapBounds(){if(mapBoundsSource===overview)return mapBoundsValue;mapBoundsSource=overview;if(!overview.length)return mapBoundsValue={minX:-10,maxX:10,minY:-10,maxY:10};let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;for(const p of overview){minX=Math.min(minX,p[0]);maxX=Math.max(maxX,p[0]);minY=Math.min(minY,p[1]);maxY=Math.max(maxY,p[1])}const size=Math.max(maxX-minX,maxY-minY,1)*1.08,cx=(maxX+minX)/2,cy=(maxY+minY)/2;return mapBoundsValue={minX:cx-size/2,maxX:cx+size/2,minY:cy-size/2,maxY:cy+size/2}}
function mapTransform(canvas){const b=mapBounds(),w=canvas.clientWidth||750,h=canvas.clientHeight||280,scale=Math.min(w/(b.maxX-b.minX),h/(b.maxY-b.minY)),cx=(b.minX+b.maxX)/2,cy=(b.minY+b.maxY)/2;return {w,h,scale,point:p=>[w/2+(p[0]-cx)*scale,h/2-(p[1]-cy)*scale],world:(x,y)=>[cx+(x-w/2)/scale,cy-(y-h/2)/scale]}}
function drawMap(id,isTerrain){const canvas=$(id);if(!canvas.clientWidth)return;const t=mapTransform(canvas),dpr=Math.min(devicePixelRatio,2);if(canvas.width!==Math.floor(t.w*dpr))canvas.width=t.w*dpr;if(canvas.height!==Math.floor(t.h*dpr))canvas.height=t.h*dpr;
  let entry=mapBackgrounds.get(id);
  if(!entry||entry.overview!==overview||entry.revision!==terrainRevision||entry.w!==t.w||entry.h!==t.h||entry.dpr!==dpr){
    const background=document.createElement('canvas');
    background.width=canvas.width;background.height=canvas.height;const ctx=background.getContext('2d');ctx.scale(dpr,dpr);ctx.fillStyle='#1b1d23';ctx.fillRect(0,0,t.w,t.h);for(let i=0;i<overview.length;i++){const p=overview[i],[x,y]=t.point(p),base=info?.cloud?.label_mapping?.[String(cloudLabels[i])]||'desconocido';ctx.fillStyle=classColors[getClass(p,base)];ctx.fillRect(x,y,2,2)}for(const zone of terrain.zones){const [x,y]=t.point(zone.center);ctx.strokeStyle=classColors[zone.class]||'#8b959a';ctx.beginPath();ctx.arc(x,y,Math.max(2,zone.radius*t.scale),0,Math.PI*2);ctx.stroke()}
    entry={background,overview,revision:terrainRevision,w:t.w,h:t.h,dpr};mapBackgrounds.set(id,entry);
  }
  const ctx=canvas.getContext('2d');ctx.setTransform(1,0,0,1,0,0);ctx.drawImage(entry.background,0,0);ctx.setTransform(dpr,0,0,dpr,0,0);ctx.lineWidth=1;
  if(route.length){ctx.strokeStyle='#0a84ff';ctx.lineWidth=2;ctx.beginPath();route.forEach((p,i)=>{const [x,y]=t.point([p.x,p.y]);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y)});ctx.stroke();route.forEach((p,i)=>{const [x,y]=t.point([p.x,p.y]);ctx.fillStyle='#0a84ff';ctx.beginPath();ctx.arc(x,y,4,0,Math.PI*2);ctx.fill();ctx.fillStyle='#fff';ctx.fillText(String(i+1),x+6,y-6)})}drawMapHeading(ctx,t);if(isTerrain){const [x,y]=t.point([Number($('zoneX').value),Number($('zoneY').value)]);ctx.strokeStyle='#f5f5f7';ctx.beginPath();ctx.moveTo(x-7,y);ctx.lineTo(x+7,y);ctx.moveTo(x,y-7);ctx.lineTo(x,y+7);ctx.stroke()}}
function drawMapHeading(ctx,t){
  const pose=frames[index]||appliedPose;if(!pose)return;
  const [x,y]=t.point(pose.position),yaw=yawOf(pose),dx=Math.cos(yaw),dy=-Math.sin(yaw),length=22;
  ctx.strokeStyle='#64b5ff';ctx.fillStyle='#64b5ff';ctx.lineWidth=2.5;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+dx*length,y+dy*length);ctx.stroke();
  ctx.beginPath();ctx.moveTo(x+dx*length,y+dy*length);ctx.lineTo(x+dx*(length-7)-dy*4,y+dy*(length-7)+dx*4);ctx.lineTo(x+dx*(length-7)+dy*4,y+dy*(length-7)-dx*4);ctx.closePath();ctx.fill();
}
function drawMaps(){if(mapPending)return;mapPending=true;requestAnimationFrame(()=>{mapPending=false;drawMap('routeMap',false);drawMap('terrainMap',true)})}
function mapClick(canvas,event){if(!overview.length)throw Error('Carga primero una nube');const r=canvas.getBoundingClientRect(),xy=mapTransform(canvas).world(event.clientX-r.left,event.clientY-r.top);let nearest=overview[0],distance=Infinity;for(const p of overview){const d=(p[0]-xy[0])**2+(p[1]-xy[1])**2;if(d<distance){distance=d;nearest=p}}if(distance>.64)throw Error('Selecciona un punto con cobertura de terreno');return nearest}
async function appendWaypointAt(p){
  if(!appliedPose){await placeVehicle(p[0],p[1]);return}
  const last=route.at(-1)||{x:appliedPose.position[0],y:appliedPose.position[1]},yaw=THREE.MathUtils.radToDeg(Math.atan2(p[1]-last.y,p[0]-last.x));
  await runBusy('Colocando pose de trayectoria…',async()=>{
    const data=await request('/api/place',{...values(),x:p[0],y:p[1],yaw,vehicle:appliedPose.vehicle,terrain,purpose:'waypoint'});
    invalidateResults();anchorRoute();route.push({x:data.position[0],y:data.position[1],z:data.position[2],speed:1,acceleration:.5,yaw:data.yaw});renderRoute();completeProgress();$('status').textContent='Pose añadida a la trayectoria';
  });
}
$('routeMap').onclick=guarded(async e=>{const p=mapClick($('routeMap'),e);await appendWaypointAt(p)});
$('terrainMap').onclick=guarded(e=>{const p=mapClick($('terrainMap'),e);['zoneX','zoneY','zoneZ'].forEach((id,i)=>$(id).value=p[i]);drawMaps()});
$('undoWaypoint').onclick=()=>{if(busy||route.length<=1)return;route.pop();invalidateResults();renderRoute()};
$('clearRoute').onclick=()=>{if(busy)return;route=route.slice(0,1);invalidateResults();renderRoute()};

$('addZone').onclick=guarded(async()=>{const center=['zoneX','zoneY','zoneZ'].map(id=>Number($(id).value)),radius=Number($('zoneRadius').value);if(!center.every(Number.isFinite)||!Number.isFinite(radius)||radius<=0||radius>1000)throw Error('Introduce un centro finito y un radio entre 0 y 1000 m');terrain.zones.push({center,radius,class:$('zoneClass').value,source:'manual'});await refreshTerrainView()});$('undoZone').onclick=guarded(async()=>{terrain.zones.pop();await refreshTerrainView()});
$('file').onchange=()=>{syncControls()};
$('loadCloud').onclick=guarded(()=>runBusy('Cargando nube…',async()=>{
  const file=$('file').files[0];if(!file)throw Error('Selecciona un archivo PLY');if(file.size>100000000)throw Error('El archivo supera los 100 MB');
  const data=await upload('/api/load',file);await loadInfo(data);completeProgress();$('file').value='';$('cloudDialog').close();
}));
$('vehicle').onchange=guarded(async()=>{if(appliedPose)await placeVehicle(appliedPose.position[0],appliedPose.position[1]);syncControls()});
$('heading').oninput=()=>{$('headingValue').textContent=`${$('heading').value}°`};
$('heading').onchange=guarded(()=>rotateVehicle(Number($('heading').value)));
$('rotateLeft').onclick=guarded(()=>rotateVehicle(THREE.MathUtils.radToDeg(yawOf(appliedPose))+15));
$('rotateRight').onclick=guarded(()=>rotateVehicle(THREE.MathUtils.radToDeg(yawOf(appliedPose))-15));$('simulate').onclick=guarded(simulate);
$('saveProject').onclick=guarded(()=>runBusy('Guardando proyecto…',async()=>{
  const response=await fetch('/api/project/export',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({project:project()})});
  if(!response.ok){const data=await response.json();throw Error(data.error||'No se pudo guardar el proyecto')}
  const url=URL.createObjectURL(await response.blob()),link=document.createElement('a');link.href=url;link.download='FusionForce.fusionforce';document.body.append(link);link.click();link.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);
  completeProgress();$('projectState').textContent='Proyecto guardado';$('status').textContent='Proyecto guardado';
}));
$('loadProject').onclick=()=>{$('projectFile').click()};
$('projectFile').onchange=guarded(()=>runBusy('Abriendo proyecto…',async()=>{
  const input=$('projectFile'),file=input.files[0];if(!file)return;
  try{
    const data=await upload('/api/project/import',file),p=data.project;await loadInfo(data.info);restorePose(p.pose||{});
    if(p.vehicle)$('vehicle').value=p.vehicle;
    route=p.route||[];recordedPath=p.trajectory||[];terrain=p.terrain||terrain;frames=p.frames||[];appliedPose=p.applied_pose||null;
    playing=false;index=Math.max(0,Math.min(Number(p.playback_index)||0,frames.length-1));csvAvailable=Boolean(data.csv_available);updateCsvDownload();
    $('warnings').textContent=p.warnings||'Sin simulación';$('timeline').max=Math.max(0,frames.length-1);renderRoute();renderTerrain();syncHeading();loadMesh();
    $('modelName').textContent=$('vehicle').selectedOptions[0]?.textContent;
    if(frames.length)renderFrame(index);else if(appliedPose)updateChassis(appliedPose);
    fit();completeProgress();$('projectState').textContent='Proyecto restaurado';$('status').textContent='Proyecto restaurado';$('projectDialog').close();
  }finally{input.value=''}
}));

$('downloadCsv').onclick=event=>{if($('downloadCsv').getAttribute('aria-disabled')==='true')event.preventDefault()};$('reset').onclick=()=>{playing=false;$('play').textContent='▶';renderFrame(0)};$('fit').onclick=fit;$('play').onclick=()=>{if(frames.length<2)return;if(index===frames.length-1)renderFrame(0);playing=!playing;last=performance.now();$('play').textContent=playing?'Ⅱ':'▶';requestRender()};$('timeline').oninput=()=>{playing=false;$('play').textContent='▶';renderFrame(Number($('timeline').value))};
const ray=new THREE.Raycaster();let pointerDown=null;renderer.domElement.addEventListener('pointerdown',e=>{pointerDown=[e.clientX,e.clientY]});renderer.domElement.addEventListener('click',guarded(async e=>{if(busy||!pointerDown||Math.hypot(e.clientX-pointerDown[0],e.clientY-pointerDown[1])>5)return;const r=renderer.domElement.getBoundingClientRect();ray.setFromCamera(new THREE.Vector2((e.clientX-r.left)/r.width*2-1,-(e.clientY-r.top)/r.height*2+1),camera);ray.params.Points.threshold=.2;const vh=vehicleDots&&ray.intersectObject(vehicleDots)[0];if(vh){selected=vh.index;selection();$('resultsDialog').showModal();renderMetrics();return}if(!cloud)return;ray.params.Points.threshold=.4;const hit=ray.intersectObject(cloud)[0];if(hit){const p=cloudPoints[hit.index];if(e.shiftKey&&appliedPose){const yaw=THREE.MathUtils.radToDeg(Math.atan2(p[1]-appliedPose.position[1],p[0]-appliedPose.position[0]));await rotateVehicle(yaw)}else await placeVehicle(p[0],p[1])}}));
function requestRender(){if(renderPending)return;renderPending=true;requestAnimationFrame(animate)}
function animate(now){
  renderPending=false;
  if(playing&&frames.length&&now-last>=Math.max(.001,(frames[index+1]?.time??frames[index].time)-frames[index].time)*1000){renderFrame(index+1);last=now;if(index===frames.length-1){playing=false;$('play').textContent='▶'}}
  controls.update();renderer.render(scene,camera);
  if(playing)requestRender();
}
for(const key of ['friction','stiffness'])$(key).onchange=()=>{terrain.classes.desconocido[key]=Number($(key).value);renderTerrain()};window.addEventListener('resize',()=>{drawMaps();renderStability(true)});renderRoute();renderTerrain();syncControls();renderStability();requestRender();request('/api/info').then(loadInfo).catch(e=>{$('status').textContent='Motor no disponible';fail(e)});
