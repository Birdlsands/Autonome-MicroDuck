import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import loadMujoco from '@mujoco/mujoco';
import './three-imports.js';
const $=id=>document.getElementById(id);
const host=$('view'), status=$('status'), log=$('log'), metrics=$('metrics');
const scene=new THREE.Scene(); scene.background=new THREE.Color(0x101925);
const camera=new THREE.PerspectiveCamera(45,1,.01,100); camera.position.set(.48,.55,.7);
const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));host.appendChild(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);orbit.target.set(0,.14,0);orbit.enableDamping=true;
scene.add(new THREE.HemisphereLight(0xffffff,0x44536c,2.2));
const light=new THREE.DirectionalLight(0xffffff,3);light.position.set(1,2,1);scene.add(light);
const floor=new THREE.Mesh(new THREE.PlaneGeometry(12,12),new THREE.MeshStandardMaterial({color:0x334154,roughness:1}));floor.rotation.x=-Math.PI/2;scene.add(floor);
const grid=new THREE.GridHelper(12,100,0x69839c,0x445469);grid.position.y=.001;scene.add(grid);
const axes=new THREE.Group();scene.add(axes);
const loader=new STLLoader();
const stat=(txt)=>{status.textContent=txt;log.textContent=txt;};
function resize(){const w=host.clientWidth,h=host.clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix()}window.addEventListener('resize',resize);resize();
function mjVec(x,y,z){return new THREE.Vector3(x,z,-y)}
function matrixQuaternion(data,idx){const m=new THREE.Matrix4();const o=idx*9;const r=data;
 // MuJoCo's Z-up rotation matrix -> Three.js Y-up.
 const C=new THREE.Matrix4().set(1,0,0,0,0,0,1,0,0,-1,0,0,0,0,0,1);
 const R=new THREE.Matrix4().set(r[o],r[o+1],r[o+2],0,r[o+3],r[o+4],r[o+5],0,r[o+6],r[o+7],r[o+8],0,0,0,0,1);
 const inv=C.clone().invert();m.multiplyMatrices(C,R).multiply(inv);
 return new THREE.Quaternion().setFromRotationMatrix(m);
}
let sim=null,bodyMeshes=[], mode='stand',time=0,accum=0,ready=false;
const MAX_DT=.03;
async function start(){
 try{
 stat('Chargement de MuJoCo WebAssembly…');const mj=await loadMujoco();
 stat('Chargement du modèle physique officiel…');
 const xml=await (await fetch('/robot/robot_walk.xml')).text();if(!xml.includes('<mujoco'))throw Error('robot_walk.xml manquant : exécuter npm run assets');
 const doc=new DOMParser().parseFromString(xml,'text/xml');
 const assetPaths=[...doc.querySelectorAll('asset > mesh')].map(el=>({name:el.getAttribute('name')||'',file:el.getAttribute('file')||'',scale:el.getAttribute('scale')||'1 1 1'}));
 mj.FS.mkdir('/work');mj.FS.mkdir('/work/assets');
 mj.FS.writeFile('/work/robot_walk.xml',xml);
 for(const a of assetPaths){const file=a.file.replace(/^assets\//,'');const response=await fetch('/robot/assets/'+file);if(!response.ok)throw Error('STL manquant : '+file);mj.FS.writeFile('/work/assets/'+file,new Uint8Array(await response.arrayBuffer()));}
 const includes=[...doc.querySelectorAll('include[file]')];for(const el of includes){const path=el.getAttribute('file');const response=await fetch('/robot/'+path);if(!response.ok)throw Error('Include XML manquant : '+path);mj.FS.writeFile('/work/'+path,await response.text());}
 let model=mj.MjModel.mj_loadXML('/work/robot_walk.xml');if(!model)throw Error('MuJoCo n’a pas pu compiler le robot_walk.xml');
 const data=new mj.MjData(model);sim={mj,model,data};
 stat('Construction du rendu des corps…');
 const meshes=[];
 for(const a of assetPaths){const file=a.file.replace(/^assets\//,'');const response=await fetch('/robot/assets/'+file);const geometry=loader.parse(await response.arrayBuffer());geometry.computeVertexNormals();const scale=a.scale.trim().split(/\s+/).map(Number);geometry.scale(scale[0],scale[1],scale[2]);meshes.push(geometry)}
 const materials=[new THREE.MeshStandardMaterial({color:0xe6e9eb,roughness:.63}),new THREE.MeshStandardMaterial({color:0xeaa15f,roughness:.65})];
 // MuJoCo geom_world pose is authoritative. Only visual geoms are rendered.
 for(let g=0;g<model.ngeom;g++){
  const type=model.geom_type[g],dataId=model.geom_dataid[g];let geometry;
  if(type===7 && dataId>=0 && dataId<meshes.length)geometry=meshes[dataId];
  else if(type===6){const s=model.geom_size;geometry=new THREE.BoxGeometry(2*s[3*g],2*s[3*g+1],2*s[3*g+2])}
  else if(type===2){const s=model.geom_size;geometry=new THREE.SphereGeometry(s[3*g],14,10)}
  else if(type===5){const s=model.geom_size;geometry=new THREE.CylinderGeometry(s[3*g],s[3*g],2*s[3*g+1],12)}
  else continue;
  const mesh=new THREE.Mesh(geometry,materials[g%5===0?1:0]);scene.add(mesh);bodyMeshes.push({g,mesh});
 }
 // Explicit floor from MuJoCo is represented by the visible grid / Three floor.
 const timestep=model.opt.timestep;ready=true;for(const id of ['stand','walk','stop','reset'])$(id).disabled=false;
 stat('MuJoCo en marche — simulation dynamique réelle.');
 function step(){
  const {model,data}=sim;
  if(mode!=='stop'){
   const jn=Math.min(model.nu, data.ctrl.length);
   for(let i=0;i<jn;i++){
    // Experimental command: walk oscillates actuator targets. Not an RL policy.
    const swing=mode==='walk'? .12*Math.sin(time*7+i*.85):0;
    const target=swing;
    data.ctrl[i]=Math.max(model.actuator_ctrlrange[2*i]||-1,Math.min(model.actuator_ctrlrange[2*i+1]||1,target));
   }
  } else data.ctrl.fill(0);
  mj.mj_step(model,data);time+=timestep;
 }
 let prev=performance.now();
 function frame(now){requestAnimationFrame(frame);let dt=Math.min(MAX_DT,(now-prev)/1000);prev=now;accum+=dt;let steps=0;
 while(accum>=timestep && steps<20){step();accum-=timestep;steps++}
 const {data,model}=sim;
 for(const {g,mesh} of bodyMeshes){const k=3*g;mesh.position.copy(mjVec(data.geom_xpos[k],data.geom_xpos[k+1],data.geom_xpos[k+2]));mesh.quaternion.copy(matrixQuaternion(data.geom_xmat,g));}
 if(Math.floor(time*4)!==Math.floor((time-dt)*4))metrics.textContent=`Temps simulé : ${time.toFixed(2)} s\nContacts MuJoCo : ${data.ncon}\nCorps : ${model.nbody} · Geoms : ${model.ngeom}\nArticulations : ${model.njnt} · Actionneurs : ${model.nu}\nMode : ${mode}\n\nSystème physique : ACTIF`;
 orbit.update();renderer.render(scene,camera);
 }
 requestAnimationFrame(frame);
 }catch(err){console.error(err);stat('ÉCHEC PHYSIQUE : '+(err?.message||err));metrics.textContent='Aucun résultat physique validé';}
}
$('stand').onclick=()=>mode='stand';$('walk').onclick=()=>mode='walk';$('stop').onclick=()=>mode='stop';$('reset').onclick=()=>{if(sim){sim.mj.mj_resetData(sim.model,sim.data);mode='stand';time=0;}};
start();
