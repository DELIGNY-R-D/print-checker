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

console.log(`\n${ok} OK / ${ko} KO`);
process.exit(ko?1:0);
