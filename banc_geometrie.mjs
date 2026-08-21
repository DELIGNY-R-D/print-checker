import * as G from './geometrie.js';
const box=(x0,y0,z0,x1,y1,z1)=>{const p=[[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0],[x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]];
 const q=[[0,1,2,3],[7,6,5,4],[4,5,1,0],[3,2,6,7],[1,5,6,2],[4,0,3,7]];const t=[];
 for(const[a,b,c,d]of q){t.push([p[a],p[b],p[c]]);t.push([p[a],p[c],p[d]]);}return t;};
let ok=0,ko=0;
const eq=(nom,got,att,tol=1e-6)=>{const bon=Math.abs(got-att)<=tol;console.log(`${bon?'  OK ':'  KO '} ${nom}: ${got} (attendu ${att})`);bon?ok++:ko++;};

const cube=box(0,0,0,10,20,30);
const a=G.analyze(cube);
eq('volume mm3',a.enclosedMm3,6000);
eq('cote X',a.size[0],10); eq('cote Z',a.size[2],30);
const h=G.meshHealth(cube);
eq('sante /100',h.score,100); eq('coques',h.shells,1); eq('aretes ouvertes',h.open,0);

const gros=G.meshTransform(cube,{scale:3});
eq('echelle x3',G.analyze(gros).size[2],90,1e-9);
const tourne=G.meshTransform(cube,{axis:'x',deg:90});
eq('rotation X 90 -> Y devient Z',G.analyze(tourne).size[2],20,1e-6);

const coupe=G.meshCut(cube,'z',12);
const ha=G.meshHealth(coupe.a), hb=G.meshHealth(coupe.b);
eq('moitie haute etanche',ha.open,0); eq('moitie basse etanche',hb.open,0);
eq('hauteur haute',G.analyze(coupe.a).size[2],18,1e-6);
eq('hauteur basse',G.analyze(coupe.b).size[2],12,1e-6);
eq('volumes conserves',G.analyze(coupe.a).enclosedMm3+G.analyze(coupe.b).enclosedMm3,6000,1e-3);

// deux cubes qui s'interpenetrent SANS partager de sommet : le cas qu'aucune soudure ne resout
const duo=[...box(0,0,0,10,10,10),...box(5,5,5,15,15,15)];
eq('avant fusion : coques',G.meshHealth(duo).shells,2);
const f=G.meshFuse(duo,{pitch:0.5});
eq('apres fusion : coques',G.meshHealth(f.tris).shells,1);
eq('apres fusion : etanche',G.meshHealth(f.tris).open,0);
const vf=G.analyze(f.tris).enclosedMm3;      // union = 1000+1000-125 = 1875
console.log(`  ~~  volume union ${vf.toFixed(0)} mm3 (theorique 1875, pas de grille 0,5)`);
if(Math.abs(vf-1875)/1875<0.06){console.log('  OK  volume union a moins de 6%');ok++}else{console.log('  KO  volume union hors tolerance');ko++}

const stl=G.trisToSTL(cube);
eq('STL binaire : octets',stl.byteLength,84+cube.length*50);

// ── orRotateTo : la rotation doit amener `from` exactement sur `to` ─────────
const ptY=[[[0,5,0],[0,5,0],[0,5,0]]];
const qY=G.orRotateTo(ptY,[0,1,0],[0,0,1])[0][0];
eq('orRotateTo Y->Z : x',qY[0],0); eq('orRotateTo Y->Z : y',qY[1],0); eq('orRotateTo Y->Z : z',qY[2],5);
const ptZ=[[[0,0,7],[0,0,7],[0,0,7]]];
const qZ=G.orRotateTo(ptZ,[0,0,1],[0,0,-1])[0][0];   // cas vecteurs opposes (angle pi)
eq('orRotateTo 180 : x',qZ[0],0); eq('orRotateTo 180 : y',qZ[1],0); eq('orRotateTo 180 : z',qZ[2],-7);

// ── orOptimize : piece en T, pire pose possible tant qu'on ne la retourne pas.
// Poteau fin (2x2mm) surmonte d'une large traverse (16x2mm) — a l'endroit, la
// traverse pend presque entierement dans le vide (gros surplomb, contact
// minuscule) ; retournee, la traverse devient la base (contact plein,
// surplomb nul). C'est exactement le cas qui justifie de coupler orientation
// et supports plutot que les traiter separement.
const poteau=box(4,4,0, 6,6,20), traverse=box(-3,4,18, 13,6,20);
const T=G.meshFuse([...poteau,...traverse],{}).tris;
const orRes=await G.orOptimize(T,{bed:[220,220,250],seuil:0.4});
console.log(`  ~~  T: actuelle supVol=${orRes.current.supVol.toFixed(0)}mm3 over=${orRes.current.over.toFixed(0)}mm2 · meilleure="${orRes.best.nom}" supVol=${orRes.best.supVol.toFixed(0)}mm3`);
if(orRes.best.nom!=='orientation actuelle' && orRes.best.supVol<orRes.current.supVol){console.log('  OK  orOptimize choisit une pose avec moins de support que l\'actuelle');ok++}
else{console.log('  KO  orOptimize n\'a pas prefere la pose sans surplomb');ko++}
if(orRes.best.over<orRes.current.over){console.log('  OK  la pose retenue a moins de surplomb critique');ok++}
else{console.log('  KO  surplomb non ameliore');ko++}
// determinisme : deux appels identiques -> meme classement
const orRes2=await G.orOptimize(T,{bed:[220,220,250],seuil:0.4});
if(orRes.best.nom===orRes2.best.nom && Math.abs(orRes.best.supVol-orRes2.best.supVol)<1e-6){console.log('  OK  orOptimize est deterministe');ok++}
else{console.log('  KO  orOptimize non deterministe entre deux appels identiques');ko++}

// ── slAdaptiveSchedule : fin sur une pente douce, grossier sur une paroi franche
// Bas = boite droite (parois verticales) 0-20mm. Haut = une seule facette en
// rampe tres peu inclinee (11,3 deg de l'horizontale) de 20 a 28mm — la pente
// qui, par construction (h = cusp*tan(beta)), doit forcer les couches les
// plus fines possibles (hmin), alors que les parois verticales n'imposent rien
// (bornees a hmax).
const droite=box(0,0,0,40,10,20);
const rampe=[[[0,0,20],[40,0,28],[40,10,28]],[[0,0,20],[40,10,28],[0,10,20]]];
const piece=[...droite,...rampe];
const sch=G.slAdaptiveSchedule(piece,{bed:[220,220,250], firstH:0.25, hmin:0.08, hmax:0.28, cusp:0.2});
eq('adaptatif : hauteur totale',sch.zs[sch.zs.length-1],28,1e-6);
const hFinale=sch.hs[sch.hs.length-1];
if(hFinale>=0.08-1e-9){console.log(`  OK  derniere couche (${hFinale.toFixed(3)} mm) pas sous hmin (reliquat absorbe)`);ok++}
else{console.log(`  KO  derniere couche ${hFinale.toFixed(3)} mm sous hmin 0.08`);ko++}
const hAt=z=>{ for(let i=1;i<sch.zs.length;i++) if(sch.zs[i]>=z) return sch.hs[i]; return sch.hs[sch.hs.length-1]; };
const hMilieuBoite=hAt(10), hRampe=hAt(24);
console.log(`  ~~  h a mi-hauteur de la boite (paroi franche) = ${hMilieuBoite.toFixed(3)} mm · h dans la rampe (11,3°) = ${hRampe.toFixed(3)} mm`);
if(Math.abs(hMilieuBoite-0.28)<1e-6){console.log('  OK  paroi verticale -> hmax');ok++}else{console.log('  KO  paroi verticale non bornee a hmax');ko++}
if(Math.abs(hRampe-0.08)<1e-6){console.log('  OK  rampe peu inclinee -> hmin');ok++}else{console.log('  KO  rampe non bornee a hmin');ko++}
if(hRampe<hMilieuBoite){console.log('  OK  adaptatif : couches plus fines sur la pente douce que sur la paroi');ok++}else{console.log('  KO  pas d\'adaptation reelle');ko++}

// slSlice({adaptive:true}) doit tenir compte du schedule (moins de couches
// qu'en hmin partout, plus que en hmax partout) et ne pas regresser le mode
// uniforme existant.
const cfgAd={bed:[220,220,250], printerName:'test', materialKey:'PLA', name:'piece',
  firstH:0.25, hmin:0.08, hmax:0.28, cusp:0.2, adaptive:true,
  nozzle:0.4, lineW:0.45, perims:2, topBottom:3, infill:15,
  vPer:40, vFill:60, vFirst:20, vTrav:120, retract:1.5, retractV:35,
  skirtLoops:0, skirtGap:3, nozC:210, bedC:60, fan:100, fanLayer:2, preset:'marlin'};
const rAd=await G.slSlice(piece,cfgAd,()=>{});
const nUniformHmin=Math.ceil((28-0.25)/0.08)+1, nUniformHmax=Math.ceil((28-0.25)/0.28)+1;
console.log(`  ~~  adaptatif : ${rAd.nLayers} couches (uniforme hmin=${nUniformHmin}, uniforme hmax=${nUniformHmax}), gain annonce ${rAd.adaptive.gainPct.toFixed(0)}%`);
if(rAd.nLayers<nUniformHmin && rAd.nLayers>nUniformHmax){console.log('  OK  nombre de couches adaptatif entre les deux bornes uniformes');ok++}
else{console.log('  KO  nombre de couches adaptatif hors bornes attendues');ko++}
if(rAd.adaptive.gainPct>0){console.log('  OK  gain positif annonce vs hmin uniforme');ok++}else{console.log('  KO  gain non positif');ko++}
const cfgUni={...cfgAd, adaptive:false, layerH:0.2};
const rUni=await G.slSlice(piece,cfgUni,()=>{});
if(!rUni.adaptive){console.log('  OK  mode uniforme toujours sans stats adaptatives (pas de regression)');ok++}
else{console.log('  KO  mode uniforme a tort marque adaptatif');ko++}

console.log(`\n${ok} OK / ${ko} KO`);
process.exit(ko?1:0);
