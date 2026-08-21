/**
 * geometrie.js — moteurs géométriques du 3D Print Checker.
 *
 * Module ES autonome : aucune dépendance, aucun DOM, aucun accès réseau.
 * Tout y est du calcul pur sur des tableaux de triangles [[x,y,z]×3], ce qui
 * le rend utilisable tel quel par n'importe quel autre projet de l'écosystème
 * (GDS, lampe studio, un script Node…) et testable sans navigateur.
 *
 * Il a été extrait de index.html le 19/08 : ces moteurs y étaient enfermés
 * dans une page monolithique, donc introuvables et non réutilisables — le
 * registre `hub architecture` ne voyait qu'une brique « Print Checker ».
 *
 * Ce qu'il contient :
 *   analyse           analyze, meshHealth
 *   tranchage         slPrepare, slTriSeg, slFill, slContours, slInfillLines,
 *                     slWriter, slScripts, slSlice
 *   distances         slDt1, slTranspose, slEdt, slField, slLocalThickness,
 *                     slOpenThin, slDilate
 *   fabricabilité     slAnalyzeThickness (épaisseurs, supports, ponts),
 *                     orSign, orEval (orientation)
 *   optimisation      orRotateTo, orCandidateDirs, orSupportPass, orOptimize
 *                     (orientation + supports comme un seul moteur, voir plus bas)
 *   édition           meshRepair, meshTransform, meshCut, meshFuse, trisToSTL
 *
 * Convention partagée : Z est la hauteur d'impression, les longueurs sont en
 * millimètres, et les normales sont supposées sortantes — d'où le signe du
 * volume signé appliqué partout (voir orSign).
 */

// ── Constantes partagées ────────────────────────────────────────────────────
export const OVERHANG_LIMIT = 50;               // ° au-delà desquels supports requis
export const FIL_D = 1.75;                      // mm, diamètre de filament
export const FIL_A = Math.PI*(FIL_D/2)*(FIL_D/2); // 2.405 mm² de section
const SL_MAXPIX = 9e5;                          // plafond de pixels par couche
const SL_MAXLAYERS = 4000;
const SL_BIG = 1e9;                             // « infini » fini : garde les paraboles calculables
const _slChan = typeof MessageChannel!=='undefined' ? new MessageChannel() : null;

// ── Analyse géométrique (volume, bbox, surface, surplomb) — Z = hauteur ─────
export function analyze(tris){
  let vol6=0, area=0;
  const mn=[1e9,1e9,1e9], mx=[-1e9,-1e9,-1e9];
  for(const [a,b,c] of tris){
    // volume signé (tétraèdres depuis l'origine)
    vol6 += a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0]) + a[2]*(b[0]*c[1]-b[1]*c[0]);
    // aire
    const ux=b[0]-a[0],uy=b[1]-a[1],uz=b[2]-a[2], vx=c[0]-a[0],vy=c[1]-a[1],vz=c[2]-a[2];
    const cx=uy*vz-uz*vy, cy=uz*vx-ux*vz, cz=ux*vy-uy*vx;
    area += Math.hypot(cx,cy,cz)/2;
    for(const p of [a,b,c]) for(let k=0;k<3;k++){ mn[k]=Math.min(mn[k],p[k]); mx[k]=Math.max(mx[k],p[k]); }
  }
  const enclosed = Math.abs(vol6/6);           // mm³
  const orient = vol6>=0 ? 1 : -1;
  const zmin = mn[2];
  // surplomb max (normales orientées vers le bas = face -Z), on saute la 1re couche
  let maxOverhang=0, overFaces=0;
  for(const [a,b,c] of tris){
    const ux=b[0]-a[0],uy=b[1]-a[1],uz=b[2]-a[2], vx=c[0]-a[0],vy=c[1]-a[1],vz=c[2]-a[2];
    let nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
    const l=Math.hypot(nx,ny,nz)||1; nz=nz/l*orient;
    const ccz=(a[2]+b[2]+c[2])/3;
    if(ccz-zmin<1) continue;                   // posé sur le plateau
    if(nz<0){ const ang=Math.asin(Math.min(1,-nz))*180/Math.PI; if(ang>maxOverhang)maxOverhang=ang; if(ang>OVERHANG_LIMIT)overFaces++; }
  }
  return {
    tris:tris.length, enclosedMm3:enclosed, areaMm2:area,
    size:[mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2]],   // X,Y,Z(hauteur)
    footprintMm2:(mx[0]-mn[0])*(mx[1]-mn[1]),
    maxOverhangDeg:+maxOverhang.toFixed(1), overhangFaces:overFaces,
  };
}

export function meshHealth(tris){
  const vmap=new Map(), verts=[], idx=[];
  const key=p=>`${Math.round(p[0]*1000)},${Math.round(p[1]*1000)},${Math.round(p[2]*1000)}`;
  for(const t of tris) for(const p of t){
    const k=key(p); let i=vmap.get(k);
    if(i===undefined){ i=verts.length; verts.push(p); vmap.set(k,i); }
    idx.push(i);
  }
  const nF=tris.length;
  let deg=0, dupF=0;
  const faceSeen=new Set();
  const edges=new Map(); // clé = lo*2^26+hi → {c: nb de faces, d: somme des sens}
  const parent=new Int32Array(verts.length); for(let i=0;i<parent.length;i++) parent[i]=i;
  const used=new Uint8Array(verts.length);
  const find=i=>{ while(parent[i]!==i){ parent[i]=parent[parent[i]]; i=parent[i]; } return i; };
  for(let f=0;f<nF;f++){
    const a=idx[f*3],b=idx[f*3+1],c=idx[f*3+2];
    if(a===b||b===c||a===c){ deg++; continue; }
    const A=verts[a],B=verts[b],C=verts[c];
    const ux=B[0]-A[0],uy=B[1]-A[1],uz=B[2]-A[2],vx=C[0]-A[0],vy=C[1]-A[1],vz=C[2]-A[2];
    if(Math.hypot(uy*vz-uz*vy,uz*vx-ux*vz,ux*vy-uy*vx)/2<1e-6){ deg++; continue; }
    const fk=(a<b?(b<c?`${a},${b},${c}`:(a<c?`${a},${c},${b}`:`${c},${a},${b}`)):(a<c?`${b},${a},${c}`:(b<c?`${b},${c},${a}`:`${c},${b},${a}`)));
    if(faceSeen.has(fk)) dupF++; else faceSeen.add(fk);
    used[a]=used[b]=used[c]=1;
    for(const [p,q] of [[a,b],[b,c],[c,a]]){
      const lo=p<q?p:q, hi=p<q?q:p, k=lo*67108864+hi;
      let e=edges.get(k); if(!e){ e={c:0,d:0}; edges.set(k,e); }
      e.c++; e.d+=(p===lo?1:-1);
      const rp=find(p),rq=find(q); if(rp!==rq) parent[rp]=rq;
    }
  }
  let open=0,nm=0,badDir=0;
  for(const e of edges.values()){
    if(e.c===1) open++;
    else if(e.c>2) nm++;
    else if(e.d!==0) badDir++;
  }
  const roots=new Set(); for(let i=0;i<verts.length;i++) if(used[i]) roots.add(find(i));
  const shells=roots.size||1;
  let score=100;
  if(open)   score-=Math.min(35, 8+Math.round(6*Math.log10(1+open)));
  if(nm)     score-=Math.min(25, 6+Math.round(5*Math.log10(1+nm)));
  if(badDir) score-=Math.min(15, 5+Math.round(4*Math.log10(1+badDir)));
  if(deg)    score-=Math.min(10, 2+Math.round(3*Math.log10(1+deg)));
  if(dupF)   score-=Math.min(8,  2+Math.round(3*Math.log10(1+dupF)));
  return {score:Math.max(0,score), open, nm, badDir, deg, dupF, shells, watertight:open===0&&nm===0};
}

export function slYield(){
  return new Promise(r=>{
    if(!_slChan){ setTimeout(r); return; }
    _slChan.port1.onmessage=()=>r();
    _slChan.port2.postMessage(0);
  });
}

// Triangles → tableaux typés, pièce posée sur le plateau et centrée en XY.
export function slPrepare(tris, bed){
  let mnx=1e9,mny=1e9,mnz=1e9,mxx=-1e9,mxy=-1e9,mxz=-1e9,vol6=0;
  for(const t of tris){
    const [a,b,c]=t;
    vol6 += a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0]) + a[2]*(b[0]*c[1]-b[1]*c[0]);
    for(const p of t){
      if(p[0]<mnx)mnx=p[0]; if(p[0]>mxx)mxx=p[0];
      if(p[1]<mny)mny=p[1]; if(p[1]>mxy)mxy=p[1];
      if(p[2]<mnz)mnz=p[2]; if(p[2]>mxz)mxz=p[2];
    }
  }
  const orient=vol6>=0?1:-1;                    // maillage globalement retourné → on rétablit
  const tx=bed[0]/2-(mnx+mxx)/2, ty=bed[1]/2-(mny+mxy)/2, tz=-mnz;
  const n=tris.length;
  const V=new Float64Array(n*9), zlo=new Float64Array(n), zhi=new Float64Array(n), nx=new Float64Array(n), ny=new Float64Array(n);
  for(let i=0;i<n;i++){
    const t=tris[i]; let lo=1e9,hi=-1e9;
    for(let k=0;k<3;k++){
      const p=t[k], o=i*9+k*3;
      V[o]=p[0]+tx; V[o+1]=p[1]+ty; V[o+2]=p[2]+tz;
      if(V[o+2]<lo)lo=V[o+2]; if(V[o+2]>hi)hi=V[o+2];
    }
    zlo[i]=lo; zhi[i]=hi;
    const o=i*9;
    const ux=V[o+3]-V[o], uy=V[o+4]-V[o+1], uz=V[o+5]-V[o+2];
    const vx=V[o+6]-V[o], vy=V[o+7]-V[o+1], vz=V[o+8]-V[o+2];
    nx[i]=(uy*vz-uz*vy)*orient; ny[i]=(uz*vx-ux*vz)*orient;   // composantes XY de la normale
  }
  const ord=Array.from({length:n},(_,i)=>i).sort((a,b)=>zlo[a]-zlo[b]);
  return {V,zlo,zhi,nx,ny,n,ord,
          box:{x0:mnx+tx,x1:mxx+tx,y0:mny+ty,y1:mxy+ty},
          size:[mxx-mnx,mxy-mny,mxz-mnz]};
}

// Intersection triangle × plan z. Convention stricte "sous le plan = pz < z" :
// un sommet pile sur le plan compte comme au-dessus, ce qui garantit 0 ou
// exactement 2 arêtes coupées (jamais de segment fantôme sur une face plate).
export function slTriSeg(P,i,z,out){
  const o=i*9, az=P.V[o+2], bz=P.V[o+5], cz=P.V[o+8];
  const a=az<z, b=bz<z, c=cz<z;
  if(a===b&&b===c) return false;
  const V=P.V; let k=0;
  const cut=(p,q,pz,qz)=>{ const t=(z-pz)/(qz-pz); out[k++]=V[p]+(V[q]-V[p])*t; out[k++]=V[p+1]+(V[q+1]-V[p+1])*t; };
  if(a!==b) cut(o,o+3,az,bz);
  if(b!==c) cut(o+3,o+6,bz,cz);
  if(c!==a) cut(o+6,o,cz,az);
  if(k!==4) return false;
  // Orientation : d = ẑ × n = (-ny, nx) laisse la matière à gauche du segment,
  // ce qui rend la règle nonzero correcte (trous = contours en sens inverse).
  if((out[2]-out[0])*(-P.ny[i]) + (out[3]-out[1])*P.nx[i] < 0){
    const x=out[0],y=out[1]; out[0]=out[2]; out[1]=out[3]; out[2]=x; out[3]=y;
  }
  return true;
}

// Remplissage par balayage de lignes, règle nonzero (tri par insertion : 2 à 20
// intersections par ligne en pratique, un tri générique coûterait plus cher).
export function slFill(segs, R, mask, bb){
  mask.fill(0);
  bb[0]=1e9; bb[1]=-1; bb[2]=1e9; bb[3]=-1;   // i0,i1,j0,j1 de l'emprise remplie
  if(bb.length>4) bb[4]=2147483647;           // + largeur de segment minimale (µm)
  const {W,H,x0,y0,px,cnt,off,cx,cd}=R;
  cnt.fill(0);
  const ns=segs.length/4;
  const rowSpan=(ay,by)=>{
    const lo=ay<by?ay:by, hi=ay<by?by:ay;
    let j0=Math.ceil((lo-y0)/px-0.5), j1=Math.ceil((hi-y0)/px-0.5)-1;
    if(j0<0)j0=0; if(j1>H-1)j1=H-1;
    return [j0,j1];
  };
  for(let s=0;s<ns;s++){
    const ay=segs[s*4+1], by=segs[s*4+3]; if(ay===by) continue;
    const [j0,j1]=rowSpan(ay,by); for(let j=j0;j<=j1;j++) cnt[j]++;
  }
  let tot=0; for(let j=0;j<H;j++){ off[j]=tot; tot+=cnt[j]; }
  if(tot>cx.length){ R.cx=new Float64Array(tot*2); R.cd=new Int8Array(tot*2); }
  const CX=R.cx, CD=R.cd, fill=new Int32Array(H);
  for(let s=0;s<ns;s++){
    const ax=segs[s*4],ay=segs[s*4+1],bx=segs[s*4+2],by=segs[s*4+3];
    if(ay===by) continue;
    const [j0,j1]=rowSpan(ay,by), dir=by>ay?1:-1, inv=(bx-ax)/(by-ay);
    for(let j=j0;j<=j1;j++){
      const yr=y0+(j+0.5)*px, p=off[j]+fill[j]++;
      CX[p]=ax+(yr-ay)*inv; CD[p]=dir;
    }
  }
  for(let j=0;j<H;j++){
    const b=off[j], n=cnt[j]; if(n<2) continue;
    for(let a=b+1;a<b+n;a++){                        // tri par insertion sur la ligne
      const kx=CX[a], kd=CD[a]; let q=a-1;
      while(q>=b&&CX[q]>kx){ CX[q+1]=CX[q]; CD[q+1]=CD[q]; q--; }
      CX[q+1]=kx; CD[q+1]=kd;
    }
    let w=0, xs=0, base=j*W;
    for(let a=b;a<b+n;a++){
      const prev=w; w+=CD[a];
      if(prev===0&&w!==0) xs=CX[a];
      else if(prev!==0&&w===0){
        // Largeur du segment de matière, en millimètres RÉELS et indépendante
        // du pas de raster. Une paroi plus fine que le pas peut disparaître de
        // la rastérisation ; sa corde, elle, reste mesurable ici. Et comme
        // l'épaisseur perpendiculaire est toujours ≤ la corde horizontale, un
        // segment étroit PROUVE une paroi mince : c'est le garde-fou qui rend
        // un dépistage à basse résolution sûr.
        if(bb.length>4){ const wmm=Math.round((CX[a]-xs)*1000); if(wmm>0&&wmm<bb[4]) bb[4]=wmm; }
        let i0=Math.ceil((xs-x0)/px-0.5), i1=Math.ceil((CX[a]-x0)/px-0.5)-1;
        if(i0<0)i0=0; if(i1>W-1)i1=W-1;
        for(let i=i0;i<=i1;i++) mask[base+i]=1;
        if(i0<=i1){                       // emprise réelle de la couche → ROI
          if(i0<bb[0])bb[0]=i0; if(i1>bb[1])bb[1]=i1;
          if(j<bb[2])bb[2]=j;   if(j>bb[3])bb[3]=j;
        }
      }
    }
  }
}

// Transformée de distance 1D exacte (enveloppe inférieure de paraboles).
export function slDt1(f,d,v,zz,n){
  let k=0; v[0]=0; zz[0]=-1e20; zz[1]=1e20;
  for(let q=1;q<n;q++){
    let s=((f[q]+q*q)-(f[v[k]]+v[k]*v[k]))/(2*q-2*v[k]);
    while(s<=zz[k]){ k--; s=((f[q]+q*q)-(f[v[k]]+v[k]*v[k]))/(2*q-2*v[k]); }
    k++; v[k]=q; zz[k]=s; zz[k+1]=1e20;
  }
  k=0;
  for(let q=0;q<n;q++){
    while(zz[k+1]<q) k++;
    const dq=q-v[k]; d[q]=dq*dq+f[v[k]];
  }
}

// Transposition par blocs 32×32. La passe "colonnes" de l'EDT lirait sinon la
// grille avec un pas de W flottants : un défaut de cache par pixel, mesuré à
// lui seul comme le premier poste de coût du tranchage. Transposer puis
// balayer en lignes coûte deux parcours contigus et supprime le problème.
export function slTranspose(src,w,h,dst){
  const B=32;
  for(let j0=0;j0<h;j0+=B) for(let i0=0;i0<w;i0+=B){
    const jm=Math.min(j0+B,h), im=Math.min(i0+B,w);
    for(let j=j0;j<jm;j++){ const s=j*w;
      for(let i=i0;i<im;i++) dst[i*h+j]=src[s+i];
    }
  }
}

// EDT² de `seed` (0 = germe, SL_BIG = reste) → dst, en pixels².
export function slEdt(seed,W,H,dst,C){
  for(let j=0;j<H;j++){
    const b=j*W;
    for(let i=0;i<W;i++) C.f[i]=seed[b+i];
    slDt1(C.f,C.d,C.v,C.z,W);
    for(let i=0;i<W;i++) dst[b+i]=C.d[i];
  }
  slTranspose(dst,W,H,C.t);
  for(let i=0;i<W;i++){
    const b=i*H;
    for(let j=0;j<H;j++) C.f[j]=C.t[b+j];
    slDt1(C.f,C.d,C.v,C.z,H);
    for(let j=0;j<H;j++) C.t[b+j]=C.d[j];
  }
  slTranspose(C.t,H,W,dst);
}

// Champ signé F (en pixels) : +profondeur sous la peau à l'intérieur,
// -distance à la matière à l'extérieur. Le zéro tombe pile sur la frontière
// réelle (deux pixels voisins valent +0,5 et -0,5), donc les iso-contours de
// niveau (k+0,5)·largeur sont exactement les axes des périmètres.
export function slField(mask,W,H,F,C,withOutside){
  const WH=W*H;
  for(let i=0;i<WH;i++) C.seed[i]=mask[i]?SL_BIG:0;
  slEdt(C.seed,W,H,C.e1,C);
  if(withOutside){
    for(let i=0;i<WH;i++) C.seed[i]=mask[i]?0:SL_BIG;
    slEdt(C.seed,W,H,C.e2,C);
    for(let i=0;i<WH;i++) F[i]=mask[i]?Math.sqrt(C.e1[i])-0.5:-(Math.sqrt(C.e2[i])-0.5);
  } else {
    for(let i=0;i<WH;i++) F[i]=mask[i]?Math.sqrt(C.e1[i])-0.5:-1;
  }
}

// Marching squares interpolé, orienté matière-à-gauche. Chaque arête de cellule
// ne porte qu'un point : la table donne une permutation arête→arête, dont le
// parcours produit directement des boucles fermées (pas de recollage flou).
export function slContours(F,W,H,T){
  const next=new Map(), pt=new Map();
  const getH=(i,j)=>{ const id=2*(j*W+i);
    if(!pt.has(id)){ const a=F[j*W+i], b=F[j*W+i+1]; let t=(T-a)/(b-a);
      if(!isFinite(t))t=0.5; if(t<0)t=0; if(t>1)t=1; pt.set(id,[i+t,j]); } return id; };
  const getV=(i,j)=>{ const id=2*(j*W+i)+1;
    if(!pt.has(id)){ const a=F[j*W+i], b=F[(j+1)*W+i]; let t=(T-a)/(b-a);
      if(!isFinite(t))t=0.5; if(t<0)t=0; if(t>1)t=1; pt.set(id,[i,j+t]); } return id; };
  // Balayage à 2 lectures par cellule : les valeurs de la colonne courante
  // deviennent celles de gauche à l'itération suivante (idem pour leurs bits).
  for(let j=0;j<H-1;j++){
    const r0=j*W, r1=r0+W;
    let v00=F[r0], v01=F[r1], b0=v00>=T?1:0, b3=v01>=T?8:0;
    for(let i=0;i<W-1;i++){
    const v10=F[r0+i+1], v11=F[r1+i+1];
    const b1=v10>=T?2:0, b2=v11>=T?4:0;
    const c=b0|b1|b2|b3;
    const pv00=v00, pv01=v01;
    v00=v10; v01=v11; b0=b1>>1; b3=b2<<1;
    if(c===0||c===15) continue;
    const A=()=>getH(i,j), B=()=>getV(i+1,j), C2=()=>getH(i,j+1), D=()=>getV(i,j);
    const mid=(pv00+v10+v11+pv01)/4;
    switch(c){
      case 1:  next.set(A(),D()); break;
      case 2:  next.set(B(),A()); break;
      case 3:  next.set(B(),D()); break;
      case 4:  next.set(C2(),B()); break;
      case 6:  next.set(C2(),A()); break;
      case 7:  next.set(C2(),D()); break;
      case 8:  next.set(D(),C2()); break;
      case 9:  next.set(A(),C2()); break;
      case 11: next.set(B(),C2()); break;
      case 12: next.set(D(),B()); break;
      case 13: next.set(A(),B()); break;
      case 14: next.set(D(),A()); break;
      case 5:  if(mid>=T){ next.set(A(),B()); next.set(C2(),D()); }
               else { next.set(A(),D()); next.set(C2(),B()); } break;   // selle : le centre tranche
      case 10: if(mid>=T){ next.set(D(),A()); next.set(B(),C2()); }
               else { next.set(B(),A()); next.set(D(),C2()); } break;
    }
  }}
  const seen=new Set(), loops=[];
  for(const start of next.keys()){
    if(seen.has(start)) continue;
    const loop=[]; let e=start, guard=0;
    while(e!==undefined && !seen.has(e) && guard++<2e6){
      seen.add(e); const p=pt.get(e); loop.push(p[0],p[1]); e=next.get(e);
    }
    if(loop.length>=8) loops.push(loop);
  }
  return loops;
}

// Douglas-Peucker sur boucle fermée (le premier point est répété en fin de
// chaîne pour que la simplification ne coupe pas la fermeture).
export function slSimplify(p,tol){
  const n=p.length/2; if(n<4) return p;
  const keep=new Uint8Array(n); keep[0]=1; keep[n-1]=1;
  const st=[[0,n-1]], t2=tol*tol;
  while(st.length){
    const [a,b]=st.pop(); if(b<=a+1) continue;
    const ax=p[a*2],ay=p[a*2+1], dx=p[b*2]-ax, dy=p[b*2+1]-ay, L2=dx*dx+dy*dy;
    let best=-1,bd=t2;
    for(let i=a+1;i<b;i++){
      const qx=p[i*2]-ax, qy=p[i*2+1]-ay; let d2;
      if(L2>0){ let t=(qx*dx+qy*dy)/L2; if(t<0)t=0; else if(t>1)t=1;
        const ex=qx-t*dx, ey=qy-t*dy; d2=ex*ex+ey*ey; }
      else d2=qx*qx+qy*qy;
      if(d2>bd){ bd=d2; best=i; }
    }
    if(best>=0){ keep[best]=1; st.push([a,best],[best,b]); }
  }
  const out=[]; for(let i=0;i<n;i++) if(keep[i]) out.push(p[i*2],p[i*2+1]);
  return out;
}

// Lignes de remplissage clippées sur un masque : on marche le long de chaque
// ligne (repère tourné) et on garde les tronçons dans la matière. Zigzag :
// une ligne sur deux est parcourue à l'envers, ce qui divise les déplacements.
export function slInfillLines(mask,R,angDeg,spacing,minLen){
  const {W,H,x0,y0,px}=R, a=angDeg*Math.PI/180, ca=Math.cos(a), sa=Math.sin(a);
  let u0=1e9,u1=-1e9,w0=1e9,w1=-1e9;
  for(const X of [x0,x0+W*px]) for(const Y of [y0,y0+H*px]){
    const u=X*ca+Y*sa, w=-X*sa+Y*ca;
    if(u<u0)u0=u; if(u>u1)u1=u; if(w<w0)w0=w; if(w>w1)w1=w;
  }
  const step=px*0.6, res=[];
  let li=0;
  for(let w=Math.ceil(w0/spacing)*spacing; w<=w1; w+=spacing, li++){
    const runs=[]; let run=null;
    for(let u=u0;u<=u1;u+=step){
      const X=u*ca-w*sa, Y=u*sa+w*ca;
      const i=Math.floor((X-x0)/px), j=Math.floor((Y-y0)/px);
      const inside = i>=0&&j>=0&&i<W&&j<H&&mask[j*W+i];
      if(inside){ if(run){ run[2]=X; run[3]=Y; } else run=[X,Y,X,Y]; }
      else if(run){ runs.push(run); run=null; }
    }
    if(run) runs.push(run);
    const keep=runs.filter(r=>Math.hypot(r[2]-r[0],r[3]-r[1])>=minLen);
    if(li&1){ keep.reverse(); for(const r of keep){ const x=r[0],y=r[1]; r[0]=r[2]; r[1]=r[3]; r[2]=x; r[3]=y; } }
    for(const r of keep) res.push(r);
  }
  return res;
}

// ── Écriture du G-code ──────────────────────────────────────────────────────
export function slWriter(cfg){
  const out=[]; const w={out, e:0, x:null, y:null, f:-1, retracted:false, time:0, dist:0};
  const F=v=>Math.round(v*60);                       // mm/s → mm/min
  w.raw=s=>out.push(s);
  w.moveZ=(z,speed)=>{ out.push(`G1 Z${z.toFixed(3)} F${F(speed)}`); w.f=F(speed); };
  w.retract=()=>{ if(w.retracted||cfg.retract<=0) return;
    w.e-=cfg.retract; out.push(`G1 E${w.e.toFixed(5)} F${F(cfg.retractV)}`); w.f=F(cfg.retractV); w.retracted=true;
    w.time+=cfg.retract/cfg.retractV; };
  w.unretract=()=>{ if(!w.retracted) return;
    w.e+=cfg.retract; out.push(`G1 E${w.e.toFixed(5)} F${F(cfg.retractV)}`); w.f=F(cfg.retractV); w.retracted=false;
    w.time+=cfg.retract/cfg.retractV; };
  w.travel=(x,y)=>{
    // Première mise en position : la buse sort du start G-code, sa position
    // n'est pas connue ici — le mouvement est émis mais ne compte ni pour la
    // rétraction ni pour le temps (sinon une distance sentinelle empoisonne
    // l'estimation entière).
    const first=(w.x==null);
    const d=first?0:Math.hypot(x-w.x,y-w.y);
    if(!first&&d<1e-4) return;
    if(!first&&d>1.5) w.retract();
    const f=F(cfg.vTrav);
    out.push(`G0 X${x.toFixed(3)} Y${y.toFixed(3)}${f!==w.f?` F${f}`:''}`);
    w.f=f; w.x=x; w.y=y; w.time+=d/cfg.vTrav;
  };
  w.extrude=(x,y,speed,eF)=>{
    const d=Math.hypot(x-w.x,y-w.y); if(d<1e-4) return;
    w.unretract();
    w.e+=d*eF; w.dist+=d;
    const f=F(speed);
    out.push(`G1 X${x.toFixed(3)} Y${y.toFixed(3)} E${w.e.toFixed(5)}${f!==w.f?` F${f}`:''}`);
    w.f=f; w.x=x; w.y=y; w.time+=d/speed;
  };
  // Boucle fermée : on démarre au point le plus proche de la buse (moins de
  // déplacement, et la couture ne se promène pas d'une couche à l'autre).
  w.loop=(pts,speed,eF)=>{
    const n=pts.length/2; if(n<3) return;
    let s=0;
    if(w.x!=null){ let bd=1e18;
      for(let i=0;i<n;i++){ const dx=pts[i*2]-w.x, dy=pts[i*2+1]-w.y, d=dx*dx+dy*dy; if(d<bd){bd=d;s=i;} } }
    w.travel(pts[s*2],pts[s*2+1]);
    for(let k=1;k<=n;k++){ const i=(s+k)%n; w.extrude(pts[i*2],pts[i*2+1],speed,eF); }
  };
  return w;
}

export function slScripts(preset,cfg){
  const prime=(70*0.6*cfg.firstH/FIL_A).toFixed(5);
  const marlinStart=[
    '; --- start G-code (Marlin générique) ---',
    'G28 ; homing des 3 axes',
    `M140 S${cfg.bedC} ; chauffe le plateau`,
    `M104 S${cfg.nozC} ; chauffe la buse`,
    `M190 S${cfg.bedC} ; attend le plateau`,
    `M109 S${cfg.nozC} ; attend la buse`,
    'G90 ; coordonnees absolues',
    'M82 ; extrusion absolue',
    'G92 E0',
    'G1 Z2.0 F1200 ; degage en Z',
    'G1 X5 Y5 F3000',
    `G1 Z${cfg.firstH.toFixed(3)} F600`,
    `G1 X75 Y5 E${prime} F1000 ; ligne d'amorce`,
    'G92 E0 ; remise a zero de l\'extrudeur',
  ].join('\n');
  const marlinEnd=[
    '; --- end G-code ---',
    'M104 S0 ; coupe la buse',
    'M140 S0 ; coupe le plateau',
    'M107 ; coupe la ventilation',
    'G91',
    'G1 E-3 F1800 ; retracte',
    'G1 Z10 F600 ; degage en Z',
    'G90',
    `G1 X5 Y${Math.max(0,cfg.bed[1]-10)} F3000 ; sort la piece`,
    'M84 ; moteurs hors tension',
  ].join('\n');
  if(preset==='klipper') return {
    start:['; --- start G-code (Klipper) ---',
      '; START_PRINT doit exister dans ton printer.cfg (homing + chauffe + amorce).',
      '; Le controle prevol ne voit PAS l\'interieur des macros : il signalera',
      '; l\'absence de G28/M109, c\'est normal avec ce modele.',
      `START_PRINT EXTRUDER_TEMP=${cfg.nozC} BED_TEMP=${cfg.bedC}`,
      'G90','M82','G92 E0'].join('\n'),
    end:['; --- end G-code (Klipper) ---','END_PRINT','M104 S0','M140 S0','M107','M84'].join('\n')};
  if(preset==='bambu') return {
    start:['; --- start G-code (Bambu Lab) ---',
      '; COLLE ICI le start G-code de Bambu Studio pour ta machine.',
      '; Sans lui : pas de homing, pas de calibration de plateau, pas de purge.',
      `; Consignes visees : buse ${cfg.nozC} °C · plateau ${cfg.bedC} °C`,
      'G90','M82','G92 E0'].join('\n'),
    end:['; --- end G-code (Bambu Lab) ---','; COLLE ICI le end G-code de Bambu Studio.',
      'M104 S0','M140 S0','M107','M84'].join('\n')};
  return {start:marlinStart,end:marlinEnd};
}

// Hauteur de couche ADAPTATIVE : fine sur les surfaces peu inclinées (chaque
// couche y dessine un large « gradin » visible), grossière sur les parois
// quasi verticales (le gradin y est minuscule, quelle que soit l'épaisseur).
// Formule standard (le même principe que PrusaSlicer/Cura « Adaptive
// Layers ») : une facette dont la normale fait un angle β avec la verticale
// EST inclinée de β par rapport à l'horizontale (normale verticale = facette
// horizontale, normale horizontale = paroi verticale). Trancher à hauteur h
// une telle facette dessine un gradin de largeur h/tan(β) — donc, pour une
// largeur de gradin cible `cusp`, la hauteur admissible est h = cusp·tan(β).
// β→90° (paroi franche) → tan→∞ → h borné par hmax. β→0° (presque à plat)
// → tan→0 → h borné par hmin. Décision prise sur la PIRE facette (la plus
// à plat) présente dans la fenêtre d'anticipation [z, z+hmax] : c'est elle
// qui impose la couche la plus fine.
export function slAdaptiveSchedule(tris, cfg){
  const P=slPrepare(tris, cfg.bed);
  const height=P.size[2];
  const n=tris.length, nzAbs=new Float64Array(n);
  for(let i=0;i<n;i++){
    const o=i*9;
    const ux=P.V[o+3]-P.V[o],   uy=P.V[o+4]-P.V[o+1], uz=P.V[o+5]-P.V[o+2];
    const vx=P.V[o+6]-P.V[o],   vy=P.V[o+7]-P.V[o+1], vz=P.V[o+8]-P.V[o+2];
    const nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
    const l=Math.hypot(nx,ny,nz)||1;
    nzAbs[i]=Math.abs(nz)/l;
  }
  const order=P.ord;
  let ptr=0, active=[];
  const zs=[height<=cfg.firstH?height:cfg.firstH], hs=[zs[0]];
  let z=zs[0];
  while(z<height-1e-6){
    const zLook=Math.min(height, z+cfg.hmax);
    while(ptr<n && P.zlo[order[ptr]]<=zLook) active.push(order[ptr++]);
    active=active.filter(t=>P.zhi[t]>=z);
    let worst=0;                          // 0 = rien d'assez a plat -> paroi franche
    for(const t of active) if(nzAbs[t]>worst) worst=nzAbs[t];
    let h = worst<1e-6 ? cfg.hmax : cfg.cusp*Math.sqrt(Math.max(0,1-worst*worst))/worst;
    h=Math.max(cfg.hmin,Math.min(cfg.hmax,h));
    // Reliquat absorbé dans CETTE couche plutôt que laissé en dernière marche
    // séparée : sans ce garde-fou, le tout dernier pas peut tomber sous hmin
    // (une tranche de 0,03 mm rencontrée en test) — moins fiable à imprimer
    // qu'une dernière couche un peu plus épaisse que prévu.
    if(height-(z+h)<cfg.hmin) h=height-z;
    z+=h; zs.push(z); hs.push(h);
  }
  return {zs, hs, height};
}

// ── Boucle principale de tranchage ──────────────────────────────────────────
export async function slSlice(tris,cfg,onProgress){
  const P=slPrepare(tris,cfg.bed);
  const height=P.size[2];
  let nLayers, zPrint, zSlice, hOf, schedule=null;
  if(cfg.adaptive){
    schedule=slAdaptiveSchedule(tris,{bed:cfg.bed, firstH:cfg.firstH, hmin:cfg.hmin, hmax:cfg.hmax, cusp:cfg.cusp});
    nLayers=schedule.zs.length;
    zPrint=i=>schedule.zs[i];
    zSlice=i=>i===0?cfg.firstH/2:(schedule.zs[i-1]+schedule.zs[i])/2;
    hOf=i=>schedule.hs[i];
  } else {
    nLayers=Math.max(1,Math.floor((height-cfg.firstH)/cfg.layerH)+1);
    zPrint=i=>(i===0?cfg.firstH:cfg.firstH+i*cfg.layerH);
    zSlice=i=>(i===0?cfg.firstH/2:cfg.firstH+(i-0.5)*cfg.layerH);
    hOf=i=>(i===0?cfg.firstH:cfg.layerH);
  }
  if(nLayers>SL_MAXLAYERS) throw new Error(`${nLayers} couches — au-delà du plafond de ${SL_MAXLAYERS}. Augmente la hauteur de couche.`);
  const lw=cfg.lineW;
  // pas de raster : au moins 3 échantillons par largeur de ligne, borné par la mémoire
  const margin=cfg.skirtLoops>0 ? cfg.skirtGap+cfg.skirtLoops*lw+2*lw : 2*lw;
  // 3 pixels par largeur de ligne : les contours étant interpolés (sous-pixel),
  // l'erreur de trajectoire reste ~px/4 ≈ 0,04 mm, pour 44 % de pixels en moins
  // qu'à lw/4 — sur toutes les phases à la fois.
  let px=Math.min(lw/3,0.15);
  const spanX=P.box.x1-P.box.x0+2*margin, spanY=P.box.y1-P.box.y0+2*margin;
  while((Math.ceil(spanX/px)+2)*(Math.ceil(spanY/px)+2)>SL_MAXPIX) px*=1.25;
  const W=Math.ceil(spanX/px)+2, H=Math.ceil(spanY/px)+2, WH=W*H;
  const R={W,H,px,x0:P.box.x0-margin-px,y0:P.box.y0-margin-px,
           cnt:new Int32Array(H),off:new Int32Array(H),cx:new Float64Array(1024),cd:new Int8Array(1024)};
  const mx=Math.max(W,H);
  // e1/e2 en Float32 : les distances utiles (< 2^24 px²) y sont exactes, et ça
  // divise par deux la mémoire du plus gros tampon.
  const C={f:new Float64Array(mx),d:new Float64Array(mx),v:new Int32Array(mx),z:new Float64Array(mx+1),
           seed:new Float32Array(WH),e1:new Float32Array(WH),e2:new Float32Array(WH),t:new Float32Array(WH)};
  const F=new Float32Array(WH), infM=new Uint8Array(WH), solM=new Uint8Array(WH), sprM=new Uint8Array(WH), sub=new Uint8Array(WH);

  const N=cfg.topBottom, RING=2*N+1;
  const ring=[], ringBB=[];
  for(let r=0;r<RING;r++){ ring.push(new Uint8Array(WH)); ringBB.push(new Int32Array(4)); }
  const ringOf=new Int32Array(RING).fill(-1);
  const slot=i=>((i%RING)+RING)%RING;
  // Balayage monotone : la liste active ne contient que les triangles qui
  // coupent le plan courant, d'où un coût quasi linéaire en nombre de couches.
  let ptr=0, active=[];
  const seg=new Float64Array(4); let segs=[];
  const rasterize=(i,mask,bb)=>{
    const z=zSlice(i);
    while(ptr<P.n && P.zlo[P.ord[ptr]]<=z) active.push(P.ord[ptr++]);
    if(active.length) active=active.filter(t=>P.zhi[t]>=z);
    segs.length=0;
    for(const t of active) if(slTriSeg(P,t,z,seg)) segs.push(seg[0],seg[1],seg[2],seg[3]);
    slFill(segs,R,mask,bb);
  };
  const ensure=i=>{ if(i<0||i>=nLayers) return null;
    const r=slot(i);
    if(ringOf[r]!==i){ rasterize(i,ring[r],ringBB[r]); ringOf[r]=i; }
    return ring[r]; };

  const w=slWriter(cfg);
  const sc=slScripts(cfg.preset,cfg);
  const startScript=cfg.startScript!=null?cfg.startScript:sc.start;
  const endScript=cfg.endScript!=null?cfg.endScript:sc.end;
  const head=[];
  w.raw(startScript);
  const tolMm=Math.max(0.012,R.px*0.4);
  let emptyLayers=0;

  for(let i=0;i<=N&&i<nLayers;i++) ensure(i);          // amorce du tampon glissant

  // Profil de coût par phase — renvoyé avec le résultat (accessible via
  // PrintChecker), c'est ce qui a permis d'isoler les vrais goulots plutôt
  // que d'optimiser au jugé.
  const PROF={raster:0,field:0,contour:0,solid:0,infill:0};
  let _t=performance.now(), lastYield=_t;
  const _lap=k=>{ const n=performance.now(); PROF[k]+=n-_t; _t=n; };

  for(let i=0;i<nLayers;i++){
    ensure(i+N);
    const mask=ensure(i);
    _lap('raster');
    const z=zPrint(i), h=hOf(i), eF=lw*h/FIL_A;
    const vPer=i===0?cfg.vFirst:cfg.vPer, vFil=i===0?cfg.vFirst:cfg.vFill;
    w.raw(`;LAYER:${i}`); w.raw(`;Z:${z.toFixed(3)}`);
    w.moveZ(z,cfg.vTrav/2);
    if(i+1===cfg.fanLayer||(i===0&&cfg.fanLayer<=1))
      w.raw(cfg.fan>0?`M106 S${Math.round(cfg.fan*2.55)} ; ventilation ${cfg.fan} %`:'M107 ; ventilation coupee');
    // ROI : tout le travail de la couche (EDT, contours, remplissage) se fait sur
    // l'emprise réelle de la matière, pas sur la grille de la pièce entière —
    // une couche haute d'une pièce élancée occupe souvent 5 % de la grille.
    const bb=ringBB[slot(i)];
    if(bb[1]<0){ emptyLayers++; continue; }                       // couche vide
    const pad=(i===0&&cfg.skirtLoops>0)?Math.ceil((cfg.skirtGap+cfg.skirtLoops*lw+lw)/px)+2:2;
    const i0=Math.max(0,bb[0]-pad), i1=Math.min(W-1,bb[1]+pad);
    const j0=Math.max(0,bb[2]-pad), j1=Math.min(H-1,bb[3]+pad);
    const rw=i1-i0+1, rh=j1-j0+1;
    for(let j=0;j<rh;j++) sub.set(mask.subarray((j0+j)*W+i0,(j0+j)*W+i1+1), j*rw);
    const RS={W:rw,H:rh,px,x0:R.x0+i0*px,y0:R.y0+j0*px};
    const SX=ix=>RS.x0+(ix+0.5)*px, SY=iy=>RS.y0+(iy+0.5)*px;
    const toMm=loop=>{ const o=new Array(loop.length);
      for(let k=0;k<loop.length;k+=2){ o[k]=SX(loop[k]); o[k+1]=SY(loop[k+1]); } return o; };
    const emit=(loops,type)=>{
      if(!loops.length) return;
      if(type) w.raw(type);
      for(const L of loops){
        const mm=slSimplify(toMm(L).concat([SX(L[0]),SY(L[1])]),tolMm);
        if(mm.length>=8) w.loop(mm.slice(0,mm.length-2),vPer,eF);
      }
    };
    slField(sub,rw,rh,F,C,i===0&&cfg.skirtLoops>0);
    _lap('field');

    // jupe (première couche) — iso-contours négatifs du champ signé
    if(i===0&&cfg.skirtLoops>0){
      w.raw(';TYPE:SKIRT');
      for(let s=cfg.skirtLoops;s>=1;s--)
        emit(slContours(F,rw,rh,-(cfg.skirtGap+(s-0.5)*lw)/px),null);
    }
    // périmètres : intérieurs d'abord, extérieur en dernier (meilleur état de surface)
    for(let k=cfg.perims-1;k>=0;k--)
      emit(slContours(F,rw,rh,(k+0.5)*lw/px), k===0?';TYPE:WALL-OUTER':';TYPE:WALL-INNER');
    _lap('contour');
    // zone de remplissage = intérieur du dernier périmètre, moins 15 % de recouvrement
    const Ti=(cfg.perims+0.35)*lw/px, RWH=rw*rh;
    let nInf=0;
    for(let k=0;k<RWH;k++){ const v=F[k]>=Ti?1:0; infM[k]=v; nInf+=v; }
    if(nInf){
      // plein = ce qui n'est pas couvert par les N couches au-dessus (ou en dessous)
      const up=[],dn=[];
      for(let k=1;k<=N;k++){ up.push(ensure(i+k)); dn.push(ensure(i-k)); }
      // Boucles indexées volontairement : un `for…of` ici alloue un itérateur
      // par pixel et par voisine, soit des centaines de milliers d'objets par
      // couche — mesuré comme le premier poste de cette phase.
      const nu=up.length, nd=dn.length;
      for(let j=0;j<rh;j++){
        const rb=j*rw, fb=(j0+j)*W+i0;
        for(let ii=0;ii<rw;ii++){
          const k=rb+ii;
          if(!infM[k]){ solM[k]=0; sprM[k]=0; continue; }
          const fk=fb+ii;
          let open=false;
          for(let q=0;q<nu;q++){ const m=up[q]; if(!m||!m[fk]){ open=true; break; } }
          if(!open) for(let q=0;q<nd;q++){ const m=dn[q]; if(!m||!m[fk]){ open=true; break; } }
          const s=open?1:0; solM[k]=s; sprM[k]=s?0:1;
        }
      }
      _lap('solid');
      const ang=(i&1)?135:45;
      const solid=slInfillLines(solM,RS,ang,lw,lw*0.8);
      if(solid.length){ w.raw(';TYPE:SKIN');
        for(const r of solid){ w.travel(r[0],r[1]); w.extrude(r[2],r[3],vFil,eF); } }
      if(cfg.infill>0){
        const sp=lw*100/cfg.infill;
        const sparse=cfg.infill>=100?slInfillLines(sprM,RS,ang,lw,lw*0.8)
                                    :slInfillLines(sprM,RS,ang,sp,lw*1.5);
        if(sparse.length){ w.raw(';TYPE:FILL');
          for(const r of sparse){ w.travel(r[0],r[1]); w.extrude(r[2],r[3],vFil,eF); } }
      }
    }
    _lap('infill');
    // Respiration pilotée par le temps écoulé, pas par le compteur de couches :
    // une couche coûte de 2 ms (petite section) à 100 ms (grande), un pas fixe
    // rendrait la main soit trop souvent soit trop rarement.
    if(performance.now()-lastYield>100){
      onProgress&&onProgress((i+1)/nLayers);
      await slYield();
      lastYield=performance.now(); _t=lastYield;
    }
  }
  w.retract();
  w.raw(endScript);
  const timeS=Math.round(w.time*1.08);               // marge d'accélération/décélération
  // La matière est FOURNIE par l'appelant (cfg.mat), jamais lue dans un
  // catalogue global : c'est la seule dépendance qui reliait encore ces
  // moteurs aux données de la page. Repli neutre pour un usage hors UI.
  const mat=cfg.mat||{label:'matière',density:1.24,priceKg:22};
  const filM=w.e/1000;
  const weightG=w.e*FIL_A/1000*mat.density;
  head.push(
    '; generated by PrintChecker-Slicer v1 — tranchage raster, 100 % navigateur',
    `; date            : ${new Date().toISOString()}`,
    `; modele          : ${cfg.name}`,
    `; machine         : ${cfg.printerName} · plateau ${cfg.bed.join('×')} mm`,
    `; matiere         : ${mat.label} · buse ${cfg.nozC} °C · plateau ${cfg.bedC} °C`,
    schedule
      ? `; couche          : ADAPTATIVE ${cfg.hmin}-${cfg.hmax} mm (1re ${cfg.firstH} mm) · buse ${cfg.nozzle} mm · ligne ${lw.toFixed(3)} mm`
      : `; couche          : ${cfg.layerH} mm (1re ${cfg.firstH} mm) · buse ${cfg.nozzle} mm · ligne ${lw.toFixed(3)} mm`,
    `; perimetres      : ${cfg.perims} · pleines dessus/dessous ${cfg.topBottom} · remplissage ${cfg.infill} %`,
    `; raster          : ${px.toFixed(4)} mm/pixel · grille ${W}×${H}`,
    '; LIMITES         : pas de supports, pas de ponts, pas de parois < diametre de buse,',
    ';                   pas de lissage.',
    ';                   Verifie le resultat au controle prevol avant de lancer.',
    `;LAYER_COUNT:${nLayers}`,
    `;TIME:${timeS}`,
    `;Filament used: ${filM.toFixed(3)}m`);
  // Gain quantifié de la hauteur adaptative : couches économisées par rapport
  // à ce qu'aurait exigé la MÊME qualité (hmin) partout, uniformément — c'est
  // la comparaison honnête, pas juste "moins de couches que l'ancien réglage".
  let adaptive=null;
  if(schedule){
    const hs=schedule.hs;
    const nUniformMin=Math.max(1,Math.ceil((height-cfg.firstH)/cfg.hmin)+1);
    adaptive={hMin:Math.min(...hs), hMax:Math.max(...hs),
      hAvg:hs.reduce((a,b)=>a+b,0)/hs.length,
      nLayers, nUniformMin, gainPct:100*(1-nLayers/nUniformMin)};
  }
  return {gcode:head.join('\n')+'\n'+w.out.join('\n')+'\n',
          nLayers, timeS, filM, weightG, emptyLayers, px, grid:[W,H],
          costEur:weightG/1000*mat.priceKg, height, dist:w.dist, prof:PROF, adaptive};
}

// ══ RÉPARATION DU MAILLAGE ══════════════════════════════════════════════════
// Réparation EXPLICITE et traçable, jamais silencieuse : on annonce ce qu'on va
// faire, on le fait, on dit ce qui a changé, et on rend un fichier. Un outil de
// contrôle qui corrige en douce masquerait le défaut qu'il existe pour révéler.
// L'ordre des opérations n'est pas libre : souder d'abord (sinon deux sommets
// distants d'un micron font croire à un trou), nettoyer ensuite, puis orienter,
// et seulement à la fin boucher — un trou n'a de sens qu'une fois les faux
// trous supprimés.
export function meshRepair(tris, opt){
  const log={soudes:0, degeneres:0, doublons:0, reorientes:0, trous:0, tri_ajoutes:0,
             coques_retirees:0, faces_retirees:0};
  const tol=opt.tol||0.001;
  // ── 1. soudure des sommets quasi-coïncidents ──────────────────────────────
  const q=v=>Math.round(v/tol);
  const map=new Map(), V=[], idx=[];
  let brut=0;
  for(const t of tris) for(const p of t){
    brut++;
    const k=q(p[0])+','+q(p[1])+','+q(p[2]);
    let i=map.get(k);
    if(i===undefined){ i=V.length; V.push([p[0],p[1],p[2]]); map.set(k,i); }
    idx.push(i);
  }
  log.soudes=brut-V.length;
  // ── 2. dégénérés et doublons ──────────────────────────────────────────────
  let F=[]; const vus=new Set();
  for(let f=0;f<idx.length;f+=3){
    const a=idx[f],b=idx[f+1],c=idx[f+2];
    if(a===b||b===c||a===c){ log.degeneres++; continue; }
    const A=V[a],B=V[b],C=V[c];
    const ux=B[0]-A[0],uy=B[1]-A[1],uz=B[2]-A[2], vx=C[0]-A[0],vy=C[1]-A[1],vz=C[2]-A[2];
    const nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
    if(Math.hypot(nx,ny,nz)/2<1e-9){ log.degeneres++; continue; }
    const s=[a,b,c].slice().sort((x,y)=>x-y).join(',');
    if(vus.has(s)){ log.doublons++; continue; }
    vus.add(s); F.push([a,b,c]);
  }
  // ── 3. orientation cohérente ──────────────────────────────────────────────
  // On propage l'orientation de proche en proche : deux faces qui partagent une
  // arête doivent la parcourir en sens INVERSE. Puis on retourne le tout si le
  // volume signé est négatif, pour que les normales pointent vers l'extérieur.
  const ekey=(a,b)=>a<b?a+':'+b:b+':'+a;
  const adj=new Map();
  F.forEach((f,i)=>{ for(let e=0;e<3;e++){ const k=ekey(f[e],f[(e+1)%3]);
    let l=adj.get(k); if(!l){ l=[]; adj.set(k,l); } l.push(i); } });
  const vu=new Uint8Array(F.length), comp=new Int32Array(F.length).fill(-1);
  let nComp=0;
  for(let s=0;s<F.length;s++){
    if(vu[s]) continue;
    const pile=[s]; vu[s]=1; comp[s]=nComp;
    while(pile.length){
      const i=pile.pop(), f=F[i];
      for(let e=0;e<3;e++){
        const a=f[e], b=f[(e+1)%3];
        for(const j of adj.get(ekey(a,b))||[]){
          if(j===i||vu[j]) continue;
          const g=F[j];
          // même sens sur l'arête partagée = voisin retourné
          let meme=false;
          for(let e2=0;e2<3;e2++) if(g[e2]===a&&g[(e2+1)%3]===b) meme=true;
          if(meme){ const t=g[1]; g[1]=g[2]; g[2]=t; log.reorientes++; }
          vu[j]=1; comp[j]=nComp; pile.push(j);
        }
      }
    }
    nComp++;
  }
  const aire=f=>{ const A=V[f[0]],B=V[f[1]],C=V[f[2]];
    const ux=B[0]-A[0],uy=B[1]-A[1],uz=B[2]-A[2], vx=C[0]-A[0],vy=C[1]-A[1],vz=C[2]-A[2];
    return Math.hypot(uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx)/2; };
  // ── 4. coques parasites ───────────────────────────────────────────────────
  if(opt.coques&&nComp>1){
    const aireC=new Float64Array(nComp); let tot=0;
    F.forEach((f,i)=>{ const a=aire(f); aireC[comp[i]]+=a; tot+=a; });
    const seuil=tot*(opt.coquesPct||0.005);
    const drop=new Set(); for(let c=0;c<nComp;c++) if(aireC[c]<seuil) drop.add(c);
    if(drop.size&&drop.size<nComp){
      const avant=F.length;
      F=F.filter((f,i)=>!drop.has(comp[i]));
      log.coques_retirees=drop.size; log.faces_retirees=avant-F.length;
    }
  }
  // ── 5. bouchage des trous ─────────────────────────────────────────────────
  // Chaque arête de bord n'appartient qu'à une face. On les chaîne en boucles,
  // qu'on ferme par un éventail depuis leur barycentre. Une boucle plane se
  // referme exactement ; une boucle très gauchie donne une surface approchée,
  // d'où le plafond de taille et le décompte affiché.
  if(opt.trous){
    const bord=new Map();
    for(const f of F) for(let e=0;e<3;e++){
      const a=f[e], b=f[(e+1)%3], k=ekey(a,b);
      const c=bord.get(k); if(c) c.n++; else bord.set(k,{a,b,n:1});
    }
    const suiv=new Map();
    for(const {a,b,n} of bord.values()) if(n===1){ if(!suiv.has(b)) suiv.set(b,[]); suiv.get(b).push(a); }
    const dejaVu=new Set();
    for(const depart of suiv.keys()){
      if(dejaVu.has(depart)) continue;
      const boucle=[]; let cur=depart, garde=0;
      while(garde++<10000){
        if(dejaVu.has(cur)) break;
        dejaVu.add(cur); boucle.push(cur);
        const nx=(suiv.get(cur)||[]).find(v=>!dejaVu.has(v));
        if(nx===undefined) break;
        cur=nx;
      }
      if(boucle.length<3||boucle.length>(opt.trouMax||400)) continue;
      const g=[0,0,0];
      for(const i of boucle){ g[0]+=V[i][0]; g[1]+=V[i][1]; g[2]+=V[i][2]; }
      g[0]/=boucle.length; g[1]/=boucle.length; g[2]/=boucle.length;
      const gi=V.length; V.push(g);
      for(let i=0;i<boucle.length;i++){
        F.push([boucle[i], boucle[(i+1)%boucle.length], gi]);
        log.tri_ajoutes++;
      }
      log.trous++;
    }
  }
  // volume signé global : normales vers l'extérieur
  let v6=0;
  for(const f of F){ const a=V[f[0]],b=V[f[1]],c=V[f[2]];
    v6 += a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0]) + a[2]*(b[0]*c[1]-b[1]*c[0]); }
  if(v6<0){ for(const f of F){ const t=f[1]; f[1]=f[2]; f[2]=t; } log.retourne_global=true; }
  return {tris:F.map(f=>[V[f[0]],V[f[1]],V[f[2]]]), log};
}

// Export STL binaire : le format qu'acceptent tous les slicers.
export function trisToSTL(tris){
  const buf=new ArrayBuffer(84+tris.length*50), dv=new DataView(buf);
  new Uint8Array(buf,0,80).set(new TextEncoder().encode('3D Print Checker — maillage repare'));
  dv.setUint32(80,tris.length,true);
  let o=84;
  for(const [a,b,c] of tris){
    const ux=b[0]-a[0],uy=b[1]-a[1],uz=b[2]-a[2], vx=c[0]-a[0],vy=c[1]-a[1],vz=c[2]-a[2];
    let nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
    const l=Math.hypot(nx,ny,nz)||1; nx/=l; ny/=l; nz/=l;
    dv.setFloat32(o,nx,true); dv.setFloat32(o+4,ny,true); dv.setFloat32(o+8,nz,true); o+=12;
    for(const p of [a,b,c]){ dv.setFloat32(o,p[0],true); dv.setFloat32(o+4,p[1],true); dv.setFloat32(o+8,p[2],true); o+=12; }
    dv.setUint16(o,0,true); o+=2;
  }
  return buf;
}

// ── 1. Transformations ──────────────────────────────────────────────────────
// Échelle, rotation sur un axe, autour du centre de la pièce. Pur calcul.
export function meshTransform(tris,{scale=1,axis=null,deg=0}){
  let mn=[1e9,1e9,1e9], mx=[-1e9,-1e9,-1e9];
  for(const t of tris) for(const p of t) for(let k=0;k<3;k++){ if(p[k]<mn[k])mn[k]=p[k]; if(p[k]>mx[k])mx[k]=p[k]; }
  const c=[(mn[0]+mx[0])/2,(mn[1]+mx[1])/2,(mn[2]+mx[2])/2];
  const a=deg*Math.PI/180, co=Math.cos(a), si=Math.sin(a);
  const rot=p=>{
    const x=p[0]-c[0], y=p[1]-c[1], z=p[2]-c[2];
    let q;
    if(axis==='x') q=[x, y*co-z*si, y*si+z*co];
    else if(axis==='y') q=[x*co+z*si, y, -x*si+z*co];
    else if(axis==='z') q=[x*co-y*si, x*si+y*co, z];
    else q=[x,y,z];
    return [q[0]*scale+c[0], q[1]*scale+c[1], q[2]*scale+c[2]];
  };
  return tris.map(t=>t.map(rot));
}

// ── 2. Découpe par un plan ──────────────────────────────────────────────────
// Chaque triangle traversé est retaillé exactement (1 ou 2 morceaux selon le
// côté), puis la section ouverte est refermée : les arêtes de coupe se chaînent
// en boucles et se remplissent en éventail. Sans ce bouchage, on obtiendrait
// deux coquilles ouvertes, imprimables nulle part.
export function meshCut(tris, axis, pos){
  const K={x:0,y:1,z:2}[axis];
  const A=[], B=[], coupeA=[], coupeB=[];
  const inter=(p,q)=>{ const t=(pos-p[K])/(q[K]-p[K]);
    return [p[0]+(q[0]-p[0])*t, p[1]+(q[1]-p[1])*t, p[2]+(q[2]-p[2])*t]; };
  for(const t of tris){
    const s=t.map(p=>p[K]>=pos);
    const n=s.filter(Boolean).length;
    if(n===3){ A.push(t); continue; }
    if(n===0){ B.push(t); continue; }
    // un sommet seul d'un côté : le repère, puis découpe en 1 + 2 triangles
    const i=(n===1)?s.indexOf(true):s.indexOf(false);
    const p0=t[i], p1=t[(i+1)%3], p2=t[(i+2)%3];
    const a=inter(p0,p1), b=inter(p0,p2);
    const seul=[[p0,a,b]], reste=[[a,p1,p2],[a,p2,b]];
    if(n===1){ A.push(...seul); B.push(...reste); coupeA.push([a,b]); coupeB.push([b,a]); }
    else     { B.push(...seul); A.push(...reste); coupeB.push([a,b]); coupeA.push([b,a]); }
  }
  const boucher=(faces,aretes)=>{
    if(!aretes.length) return;
    const cle=p=>p.map(v=>Math.round(v*1000)).join(',');
    const suiv=new Map();
    for(const [a,b] of aretes) suiv.set(cle(a),{a,b});
    const vus=new Set();
    for(const [k0,e0] of suiv){
      if(vus.has(k0)) continue;
      const boucle=[]; let cur=e0, garde=0;
      while(cur&&!vus.has(cle(cur.a))&&garde++<20000){
        vus.add(cle(cur.a)); boucle.push(cur.a); cur=suiv.get(cle(cur.b));
      }
      if(boucle.length<3) continue;
      const g=[0,0,0];
      for(const p of boucle) for(let k=0;k<3;k++) g[k]+=p[k]/boucle.length;
      for(let i=0;i<boucle.length;i++) faces.push([boucle[i],boucle[(i+1)%boucle.length],g]);
    }
  };
  boucher(A,coupeA); boucher(B,coupeB);
  return {a:A, b:B};
}

// ── 3. Fusion en un seul corps ──────────────────────────────────────────────
// Union booléenne par la voie RASTER, pas analytique. On rastérise la pièce
// couche par couche avec la règle nonzero — laquelle réalise DÉJÀ l'union de
// toutes les coques, y compris quand elles s'interpénètrent sans partager le
// moindre sommet, ce qu'aucune soudure ne sait faire — puis on re-maille la
// grille obtenue. C'est approximatif au pas de grille, et ça ne casse jamais :
// le même compromis que la trancheuse, pour la même raison.
// Le re-maillage utilise les « surface nets » : un sommet par cellule traversée,
// placé au barycentre des coupures d'arêtes, puis un quad par arête de la
// grille qui change de signe. Pas de table à 256 cas comme les marching cubes,
// donc rien à recopier de travers.
export function meshFuse(tris, opt){
  const P=slPrepare(tris,[0,0,0]);
  const sx=P.box.x1-P.box.x0, sy=P.box.y1-P.box.y0, hz=P.size[2];
  let p=opt.pitch||Math.max(sx,sy,hz)/160;
  const MAXVOX=9e6;
  const dims=()=>[Math.ceil(sx/p)+4, Math.ceil(sy/p)+4, Math.ceil(hz/p)+4];
  while(dims().reduce((a,b)=>a*b,1)>MAXVOX) p*=1.22;
  const [NX,NY,NZ]=dims();
  const x0=P.box.x0-2*p, y0=P.box.y0-2*p, z0=-2*p;
  const R={W:NX,H:NY,px:p,x0,y0,cnt:new Int32Array(NY),off:new Int32Array(NY),
           cx:new Float64Array(1024),cd:new Int8Array(1024)};
  const vox=new Uint8Array(NX*NY*NZ), masque=new Uint8Array(NX*NY), bb=new Int32Array(5);
  const seg=new Float64Array(4);
  let ptr=0, actifs=[];
  for(let k=0;k<NZ;k++){
    const z=z0+(k+0.5)*p;
    while(ptr<P.n && P.zlo[P.ord[ptr]]<=z) actifs.push(P.ord[ptr++]);
    if(actifs.length) actifs=actifs.filter(t=>P.zhi[t]>=z);
    const segs=[];
    for(const t of actifs) if(slTriSeg(P,t,z,seg)) segs.push(seg[0],seg[1],seg[2],seg[3]);
    slFill(segs,R,masque,bb);
    vox.set(masque,k*NX*NY);
  }
  // surface nets
  const dedans=(i,j,k)=>(i<0||j<0||k<0||i>=NX||j>=NY||k>=NZ)?0:vox[k*NX*NY+j*NX+i];
  const idx=new Int32Array(NX*NY*NZ).fill(-1);
  const V=[];
  const COINS=[[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]];
  const ARETES=[[0,1],[1,2],[3,2],[0,3],[4,5],[5,6],[7,6],[4,7],[0,4],[1,5],[2,6],[3,7]];
  for(let k=0;k<NZ-1;k++) for(let j=0;j<NY-1;j++) for(let i=0;i<NX-1;i++){
    const s=COINS.map(([a,b,c])=>dedans(i+a,j+b,k+c));
    const somme=s.reduce((a,b)=>a+b,0);
    if(somme===0||somme===8) continue;
    let cx=0,cy=0,cz=0,n=0;
    for(const [a,b] of ARETES) if(s[a]!==s[b]){
      cx+=(COINS[a][0]+COINS[b][0])/2; cy+=(COINS[a][1]+COINS[b][1])/2; cz+=(COINS[a][2]+COINS[b][2])/2; n++;
    }
    idx[k*NX*NY+j*NX+i]=V.length;
    V.push([x0+(i+cx/n+0.5)*p, y0+(j+cy/n+0.5)*p, z0+(k+cz/n+0.5)*p]);
  }
  const F=[];
  const cellule=(i,j,k)=>(i<0||j<0||k<0||i>=NX-1||j>=NY-1||k>=NZ-1)?-1:idx[k*NX*NY+j*NX+i];
  const quad=(a,b,c,d,inv)=>{ if(a<0||b<0||c<0||d<0) return;
    if(inv){ F.push([V[a],V[c],V[b]]); F.push([V[a],V[d],V[c]]); }
    else   { F.push([V[a],V[b],V[c]]); F.push([V[a],V[c],V[d]]); } };
  for(let k=0;k<NZ;k++) for(let j=0;j<NY;j++) for(let i=0;i<NX;i++){
    const ici=dedans(i,j,k);
    if(dedans(i+1,j,k)!==ici) quad(cellule(i,j-1,k-1),cellule(i,j,k-1),cellule(i,j,k),cellule(i,j-1,k),!ici);
    if(dedans(i,j+1,k)!==ici) quad(cellule(i-1,j,k-1),cellule(i-1,j,k),cellule(i,j,k),cellule(i,j,k-1),!ici);
    if(dedans(i,j,k+1)!==ici) quad(cellule(i-1,j-1,k),cellule(i,j-1,k),cellule(i,j,k),cellule(i-1,j,k),!ici);
  }
  return {tris:F, pitch:p, grille:[NX,NY,NZ]};
}

// `orient` = signe du volume signé du maillage. Un STL globalement retourné a
// toutes ses normales à l'envers : sans cette correction, on mesurerait les
// faces du DESSUS en croyant compter les porte-à-faux, et aucune face ne serait
// jamais vue posée sur le plateau. C'est la correction que font déjà analyze()
// et la trancheuse — l'oublier ici donnait 0 cm² de contact sur une pièce à
// fond plat.
export function orSign(tris){
  let v6=0;
  for(const [a,b,c] of tris)
    v6 += a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0]) + a[2]*(b[0]*c[1]-b[1]*c[0]);
  return v6>=0?1:-1;
}

export function orEval(tris,u,orient){
  const SIN=Math.sin(OVERHANG_LIMIT*Math.PI/180);
  let over=0, tot=0, contact=0, mn=1e9, mx=-1e9;
  const proj=p=>p[0]*u[0]+p[1]*u[1]+p[2]*u[2];
  // 1re passe : hauteur (bornes de la projection sur l'axe vertical choisi)
  for(const t of tris) for(const p of t){ const z=proj(p); if(z<mn)mn=z; if(z>mx)mx=z; }
  // 2e passe : aires, en distinguant ce qui pose à plat de ce qui pend
  for(const t of tris){
    const [a,b,c]=t;
    const ux=b[0]-a[0],uy=b[1]-a[1],uz=b[2]-a[2], vx=c[0]-a[0],vy=c[1]-a[1],vz=c[2]-a[2];
    let nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
    const l=Math.hypot(nx,ny,nz); if(l<1e-12) continue;
    const A=l/2; tot+=A;
    const d=(nx*u[0]+ny*u[1]+nz*u[2])/l*orient;   // composante verticale de la normale
    const zc=(proj(a)+proj(b)+proj(c))/3;
    if(d<-0.995 && zc-mn<0.3) contact+=A;      // face posée sur le plateau
    else if(-d>SIN && zc-mn>1) over+=A;        // surplomb réel, hors 1re couche
  }
  return {over, tot, contact, h:mx-mn, pctOver:tot>0?over/tot*100:0};
}

// ── Optimisation d'orientation ───────────────────────────────────────────────
// « Trouver automatiquement la meilleure façon d'imprimer cette pièce » comme
// UN SEUL problème, pas deux : une pose n'est réellement bonne qu'en fonction
// des supports qu'elle entraîne (orienter seul serait incomplet), et améliorer
// les supports d'une mauvaise pose reste local (supporter seul ne corrige pas
// le choix de pose). D'où le pipeline : candidates → supports estimés pour
// CHACUNE → score à six termes explicites → meilleure pose + 2 alternatives.

// Rotation qui amène le vecteur unitaire `from` sur le vecteur unitaire `to`
// (formule de Rodrigues). `u` dans orEval n'est qu'une direction « haut »
// évaluée par projection, sans toucher au maillage — orRotateTo réalise
// PHYSIQUEMENT cette pose : après rotation, trancher avec Z comme axe vertical
// imprime exactement l'orientation candidate.
export function orRotateTo(tris, from, to){
  const l=Math.hypot(from[0],from[1],from[2])||1;
  const f=[from[0]/l,from[1]/l,from[2]/l];
  const dot=f[0]*to[0]+f[1]*to[1]+f[2]*to[2];
  if(dot>0.999999) return tris.map(t=>t.map(p=>[p[0],p[1],p[2]])); // déjà aligné
  let ax,ay,az,s,c=dot;
  if(dot<-0.999999){
    // vecteurs opposés : angle π, n'importe quel axe perpendiculaire convient
    const perp=Math.abs(f[0])<0.9?[1,0,0]:[0,1,0];
    ax=f[1]*perp[2]-f[2]*perp[1]; ay=f[2]*perp[0]-f[0]*perp[2]; az=f[0]*perp[1]-f[1]*perp[0];
    const al=Math.hypot(ax,ay,az)||1; ax/=al; ay/=al; az/=al; s=0;
  } else {
    ax=f[1]*to[2]-f[2]*to[1]; ay=f[2]*to[0]-f[0]*to[2]; az=f[0]*to[1]-f[1]*to[0];
    s=Math.hypot(ax,ay,az); ax/=s; ay/=s; az/=s;
  }
  const C=1-c;
  const M=[
    [c+ax*ax*C,    ax*ay*C-az*s, ax*az*C+ay*s],
    [ay*ax*C+az*s, c+ay*ay*C,    ay*az*C-ax*s],
    [az*ax*C-ay*s, az*ay*C+ax*s, c+az*az*C],
  ];
  const rot=p=>[M[0][0]*p[0]+M[0][1]*p[1]+M[0][2]*p[2],
                M[1][0]*p[0]+M[1][1]*p[1]+M[1][2]*p[2],
                M[2][0]*p[0]+M[2][1]*p[1]+M[2][2]*p[2]];
  return tris.map(t=>t.map(rot));
}

// Directions candidates : les 6 poses sur les axes (comportement historique du
// comparateur, toujours proposées pour ne pas régresser) + les normales des
// faces les plus étendues, regroupées par proximité angulaire (6°). Une face
// devient le bas du plateau quand sa normale sortante pointe à l'opposé de
// `u` (même convention que le seuil de contact dans orEval) — ce qui approxime
// « poser sur une face de l'enveloppe convexe » sans calculer d'enveloppe :
// sur une pièce imprimable, les grandes faces plates SONT en général des
// faces de l'enveloppe. Tri par aire décroissante → déterministe.
export function orCandidateDirs(tris, orient, maxN=18){
  const AXES=[
    {u:[0,0,1],  nom:'orientation actuelle', axe:true},
    {u:[0,0,-1], nom:'retournée (180°)',      axe:true},
    {u:[0,1,0],  nom:'basculée en arrière',   axe:true},
    {u:[0,-1,0], nom:'basculée en avant',     axe:true},
    {u:[1,0,0],  nom:'couchée sur la gauche', axe:true},
    {u:[-1,0,0], nom:'couchée sur la droite', axe:true},
  ];
  const byFace=[];
  for(const t of tris){
    const [a,b,c]=t;
    const ux=b[0]-a[0],uy=b[1]-a[1],uz=b[2]-a[2], vx=c[0]-a[0],vy=c[1]-a[1],vz=c[2]-a[2];
    let nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
    const l=Math.hypot(nx,ny,nz); if(l<1e-9) continue;
    byFace.push({u:[-nx/l*orient,-ny/l*orient,-nz/l*orient], area:l/2});
  }
  byFace.sort((p,q)=>q.area-p.area);
  // 20°, pas 6° : sur une surface facettée/organique (ex. les reliefs d'une
  // pièce), un seuil serré fait exploser le nombre de candidates quasi
  // identiques — chacune coûtant une passe de supports complète en aval.
  const COS=Math.cos(20*Math.PI/180);
  const merged=[];
  for(const f of byFace){
    let hit=null;
    for(const m of merged){ if(m.u[0]*f.u[0]+m.u[1]*f.u[1]+m.u[2]*f.u[2]>COS){ hit=m; break; } }
    if(hit) hit.area+=f.area;
    else { merged.push({u:f.u,area:f.area}); if(merged.length>=maxN*4) break; }
  }
  merged.sort((p,q)=>q.area-p.area);
  const out=AXES.slice();
  for(const m of merged){
    if(out.length-AXES.length>=maxN) break;
    if(out.some(o=>o.u[0]*m.u[0]+o.u[1]*m.u[1]+o.u[2]*m.u[2]>COS)) continue;
    out.push({u:m.u, nom:'face la plus étendue posée à plat', axe:false});
  }
  return out;
}

// Passe SUPPORTS + PONTS SEULE — sans champ de distance ni ouverture pour les
// parois fines. Mesuré : sur la démo (1300 triangles), une passe complète de
// slAnalyzeThickness en mode dépistage prenait 1,85 s par candidate, dont
// 1,5 s (81 %) dans slField + slOpenThin — exactement le poste qui avait déjà
// coûté le plus cher à la carte d'épaisseurs (voir plus haut), pour un
// résultat qu'une optimisation d'orientation n'utilise jamais. D'où cette
// version allégée, réutilisée par orOptimize ; slAnalyzeThickness reste la
// seule source de vérité pour les chiffres affichés une fois la pose choisie
// et appliquée — dupliquée plutôt que branchée dedans pour ne pas risquer de
// régression sur cette fonction déjà en production et finement vérifiée.
export async function orSupportPass(tris, cfg, onProgress){
  const P=slPrepare(tris, cfg.bed);
  const height=P.size[2];
  if(height<=0) return {supArea:0,supVol:0,supLayers:0,supIslands:0,supMaxIsland:0,
    brCount:0,brArea:0,brMaxSpan:0,brLong:0,height:0,P};
  const seuil=cfg.seuil;
  // Plancher de couches (60, pas 200) : pour CLASSER des orientations, la
  // résolution verticale de la carte d'épaisseurs n'apporte rien — mesuré :
  // sur une pièce facettée, 4 des 7 candidates montaient à 2-4 s chacune,
  // presque tout dans le nombre de couches et la finesse du pas.
  const step=Math.max(Math.min(seuil/2,0.25), height/(cfg.maxLayers||60));
  const nL=Math.max(1,Math.floor(height/step));
  const margin=2*seuil;
  let px=cfg.px||Math.max(0.3,seuil);
  const spanX=P.box.x1-P.box.x0+2*margin, spanY=P.box.y1-P.box.y0+2*margin;
  while((Math.ceil(spanX/px)+2)*(Math.ceil(spanY/px)+2)>4e5) px*=1.25;
  const W=Math.ceil(spanX/px)+2, H=Math.ceil(spanY/px)+2, WH=W*H;
  const R={W,H,px,x0:P.box.x0-margin-px,y0:P.box.y0-margin-px,
           cnt:new Int32Array(H),off:new Int32Array(H),cx:new Float64Array(1024),cd:new Int8Array(1024)};
  const mx=Math.max(W,H);
  const C={f:new Float64Array(mx),d:new Float64Array(mx),v:new Int32Array(mx),z:new Float64Array(mx+1),
           seed:new Float32Array(WH),t:new Float32Array(WH)};
  const dSup=new Float32Array(WH);
  const mask=new Uint8Array(WH), sub=new Uint8Array(WH), bb=new Int32Array(5);
  const OVER=OVERHANG_LIMIT;
  const rSup=Math.max(1,Math.round((step/Math.tan(OVER*Math.PI/180))/px));
  const SUPFILL=0.15, BRIDGE_MAX=25;
  const cellA=px*px;
  const roi={rw:0,rh:0,i0:0,j0:0};
  const prevM=new Uint8Array(WH), pv=new Uint8Array(WH), dil=new Uint8Array(WH), tmpD=new Uint8Array(WH),
        unsup=new Uint8Array(WH), supM=new Uint8Array(WH), lastZ=new Float32Array(WH);
  const seen=new Uint8Array(WH), stack=new Int32Array(WH);
  let supArea=0, supVol=0, supLayers=0, supIslands=0, supMaxIsland=0;
  let brCount=0, brArea=0, brMaxSpan=0, brLong=0;
  const toROI=(pad)=>{
    const i0=Math.max(0,bb[0]-pad), i1=Math.min(W-1,bb[1]+pad);
    const j0=Math.max(0,bb[2]-pad), j1=Math.min(H-1,bb[3]+pad);
    const rw=i1-i0+1, rh=j1-j0+1;
    for(let j=0;j<rh;j++) sub.set(mask.subarray((j0+j)*W+i0,(j0+j)*W+i1+1), j*rw);
    roi.rw=rw; roi.rh=rh; roi.i0=i0; roi.j0=j0;
  };
  let ptr=0, active=[]; const seg=new Float64Array(4); let segs=[];
  const zAt=i=>Math.min(height-1e-4,(i+0.5)*step);
  let lastYield=performance.now();
  for(let i=0;i<nL;i++){
    const z=zAt(i);
    while(ptr<P.n && P.zlo[P.ord[ptr]]<=z) active.push(P.ord[ptr++]);
    if(active.length) active=active.filter(t=>P.zhi[t]>=z);
    segs.length=0;
    for(const t of active) if(slTriSeg(P,t,z,seg)) segs.push(seg[0],seg[1],seg[2],seg[3]);
    slFill(segs,R,mask,bb);
    if(bb[1]>=0){
      toROI(Math.max(2,rSup+1));
      const {rw,rh,i0,j0}=roi, n0=rw*rh;
      if(i>0){
        for(let j=0;j<rh;j++) pv.set(prevM.subarray((j0+j)*W+i0,(j0+j)*W+i0+rw), j*rw);
        slDilate(pv,rw,rh,rSup,dil,tmpD);
        let nUn=0;
        for(let k=0;k<n0;k++){ const u=(sub[k]&&!dil[k])?1:0; unsup[k]=u; if(u)nUn++; }
        if(nUn){
          for(let k=0;k<n0;k++) supM[k]=(sub[k]&&dil[k])?1:0;
          for(let k=0;k<n0;k++) C.seed[k]=supM[k]?0:SL_BIG;
          slEdt(C.seed,rw,rh,dSup,C);
          const MAXSTEP=Math.ceil(BRIDGE_MAX*1.6/px);
          const walk=(cx,cy,dx,dy)=>{
            const len=Math.hypot(dx,dy)*px;
            for(let s=1;s<=MAXSTEP;s++){
              const x=cx+dx*s, y=cy+dy*s;
              if(x<0||y<0||x>=rw||y>=rh) return -1;
              const q=y*rw+x;
              if(supM[q]) return s*len;
              if(!sub[q]) return -1;
            }
            return -1;
          };
          const PAIRS=[[1,0],[0,1],[1,1],[1,-1]];
          let area=0, big=0, nIsl=0;
          seen.fill(0,0,n0);
          for(let p0=0;p0<n0;p0++){
            if(seen[p0]||!unsup[p0]) continue;
            let sp=0,n=0,vol=0,deep=p0,dmax=-1; stack[sp++]=p0; seen[p0]=1;
            while(sp){
              const p=stack[--sp]; n++;
              const x=p%rw, y=(p/rw)|0;
              if(dSup[p]>dmax){ dmax=dSup[p]; deep=p; }
              vol+=cellA*Math.max(0,z-lastZ[(j0+y)*W+i0+x])*SUPFILL;
              if(x>0)   {const q=p-1; if(!seen[q]&&unsup[q]){seen[q]=1;stack[sp++]=q;}}
              if(x<rw-1){const q=p+1; if(!seen[q]&&unsup[q]){seen[q]=1;stack[sp++]=q;}}
              if(y>0)   {const q=p-rw;if(!seen[q]&&unsup[q]){seen[q]=1;stack[sp++]=q;}}
              if(y<rh-1){const q=p+rw;if(!seen[q]&&unsup[q]){seen[q]=1;stack[sp++]=q;}}
            }
            const a=n*cellA;
            if(a<1) continue;
            const cx=deep%rw, cy=(deep/rw)|0;
            let span=Infinity;
            for(const [dx,dy] of PAIRS){
              const d1=walk(cx,cy,dx,dy), d2=walk(cx,cy,-dx,-dy);
              if(d1>=0&&d2>=0&&d1+d2<span) span=d1+d2;
            }
            if(isFinite(span)){
              brCount++; brArea+=a; if(span>brMaxSpan) brMaxSpan=span;
              if(span>BRIDGE_MAX) brLong++;
            } else {
              nIsl++; area+=a; if(a>big)big=a; supVol+=vol;
            }
          }
          if(area>0){ supArea+=area; supLayers++; supIslands+=nIsl; if(big>supMaxIsland)supMaxIsland=big; }
        }
      }
      for(let j=0;j<rh;j++){ const rb=j*rw, fb=(j0+j)*W+i0;
        for(let x=0;x<rw;x++) if(sub[rb+x]) lastZ[fb+x]=z; }
      prevM.set(mask);
    } else prevM.fill(0);
    if(performance.now()-lastYield>100){ onProgress&&onProgress((i+1)/nL); await slYield(); lastYield=performance.now(); }
  }
  return {supArea, supVol, supLayers, supIslands, supMaxIsland, supAngle:OVER,
          brCount, brArea, brMaxSpan, brLong, brMax:BRIDGE_MAX, height, P, nL, px, step};
}

// Score = supports (volume) + temps (proxy = hauteur, à volume constant plus
// de couches veut dire plus d'arrêts/reprises) + surface à supporter + hauteur
// relative (élancement, risque de vibration/renversement) + stabilité
// (contact plateau) + surplombs critiques — SIX termes gardés SÉPARÉS dans le
// résultat, jamais fondus en un nombre opaque : chaque écart reste affichable.
// Deux passes, comme le dépistage de la carte d'épaisseurs : orEval (gratuit,
// aucune rotation, aucun tranchage) présélectionne toutes les candidates ;
// seules les meilleures reçoivent la passe coûteuse (rotation réelle + carte
// des supports en résolution grossière — trancher chaque candidate en fin
// coûterait des dizaines de secondes par pièce pour un classement que la
// présélection donne déjà correctement à moindre coût).
const OR_W={supports:3, temps:1, surfSupport:1.5, hauteur:0.5, stabilite:1, surplombs:1};
export async function orOptimize(tris, cfg, onProgress){
  const orient=orSign(tris);
  const bed=cfg.bed, seuil=cfg.seuil;
  const all=orCandidateDirs(tris, orient, cfg.maxCand||18);
  const cheap=all.map(c=>({...c, ...orEval(tris,c.u,orient)}));
  // présélection bon marché : mêmes poids que l'ancien comparateur 6 axes
  // (porte-à-faux d'abord, hauteur ensuite, contact en appoint), pour ne pas
  // régresser sur ce que la géométrie seule décidait déjà.
  const preScore=r=>r.pctOver*3 + r.h*0.05 - (r.contact/Math.max(1e-6,r.tot))*50;
  const n=Math.min(cheap.length, cfg.shortlist||7);
  const short=cheap.slice().sort((a,b)=>preScore(a)-preScore(b)).slice(0,n);
  if(!short.some(c=>c.nom==='orientation actuelle')){
    const cur=cheap.find(c=>c.nom==='orientation actuelle'); if(cur) short.push(cur);
  }
  const px=Math.max(0.3, seuil);
  const survivors=[]; let discarded=0, i=0;
  for(const cand of short){
    const rot=orRotateTo(tris, cand.u, [0,0,1]);
    let x0=1e9,x1=-1e9,y0=1e9,y1=-1e9;
    for(const t of rot) for(const p of t){ if(p[0]<x0)x0=p[0]; if(p[0]>x1)x1=p[0]; if(p[1]<y0)y0=p[1]; if(p[1]>y1)y1=p[1]; }
    const fx=x1-x0, fy=y1-y0;
    // la rotation autour de l'axe d'impression (lacet) est libre : elle ne
    // change ni surplombs ni supports, seulement le placement sur le plateau
    const fits=(fx<=bed[0]&&fy<=bed[1])||(fx<=bed[1]&&fy<=bed[0]);
    if(!fits){ discarded++; i++; onProgress&&onProgress(i/short.length); continue; }
    const TH=await orSupportPass(rot,{bed,seuil,px});
    survivors.push({...cand, supVol:TH.supVol, supArea:TH.supArea,
      brCount:TH.brCount, height:TH.height, footprint:fx*fy});
    i++; onProgress&&onProgress(i/short.length);
  }
  if(!survivors.length) return {current:null, best:null, alternatives:[], discarded, n:short.length};
  const mm=k=>{
    let lo=Infinity,hi=-Infinity;
    for(const s of survivors){ const v=s[k]; if(v<lo)lo=v; if(v>hi)hi=v; }
    return {lo,hi};
  };
  const norm=(v,{lo,hi})=>hi>lo?(v-lo)/(hi-lo):0;
  const rSupVol=mm('supVol'), rSupArea=mm('supArea'), rOver=mm('over');
  const maxContact=Math.max(...survivors.map(x=>x.contact),1e-6);
  const maxHeight=Math.max(...survivors.map(x=>x.height),1e-6);
  const maxElance=Math.max(...survivors.map(x=>x.height/Math.sqrt(Math.max(1,x.footprint))),1e-6);
  const score=s=>{
    const elancement=s.height/Math.sqrt(Math.max(1,s.footprint));
    const stabBad=1-s.contact/maxContact;
    return OR_W.supports*norm(s.supVol,rSupVol)
         + OR_W.temps*(s.height/maxHeight)
         + OR_W.surfSupport*norm(s.supArea,rSupArea)
         + OR_W.hauteur*(elancement/maxElance)
         + OR_W.stabilite*stabBad
         + OR_W.surplombs*norm(s.over,rOver);
  };
  const ranked=survivors.map(s=>({...s, elancement:s.height/Math.sqrt(Math.max(1,s.footprint)), score:score(s)}))
                         .sort((a,b)=>a.score-b.score);
  const current=ranked.find(s=>s.nom==='orientation actuelle')||ranked[0];
  return {current, best:ranked[0], alternatives:ranked.slice(1,3), discarded, n:short.length, ranked};
}

// Épaisseur locale 2D par disques inscrits. Les pixels sont parcourus du plus
// profond au moins profond (tri par comptage, pas de tri générique) : le
// premier disque qui couvre un pixel est forcément le plus grand, d'où le
// court-circuit qui rend l'algorithme praticable.
export function slLocalThickness(F,W,H,out){
  const n=W*H;
  out.fill(0);
  let maxd=0;
  for(let i=0;i<n;i++){ const v=F[i]; if(v>maxd)maxd=v; }
  if(maxd<=0) return 0;
  // On ne peint QUE depuis l'axe médian. Un pixel dont un voisin porte un
  // disque qui l'englobe (F[voisin] ≥ F[p] + distance) est redondant : son
  // disque est inclus dans celui du voisin. Ce filtre est ce qui rend
  // l'algorithme praticable — et surtout, il remplace le court-circuit
  // "ce pixel est déjà couvert, je saute" qui, lui, était FAUX : le disque
  // sauté couvrait aussi d'autres pixels, dont certains encore vierges. C'est
  // ce qui laissait tout le pourtour des sections en valeurs aberrantes.
  const D1=1, D2=Math.SQRT2, ridge=[];
  for(let j=1;j<H-1;j++) for(let i=1;i<W-1;i++){
    const p=j*W+i, d=F[p]; if(d<=0) continue;
    if(F[p-1]>=d+D1||F[p+1]>=d+D1||F[p-W]>=d+D1||F[p+W]>=d+D1||
       F[p-W-1]>=d+D2||F[p-W+1]>=d+D2||F[p+W-1]>=d+D2||F[p+W+1]>=d+D2) continue;
    ridge.push(p);
  }
  const order=ridge;
  // La matière déborde d'un DEMI-PIXEL au-delà du dernier centre de pixel plein,
  // et le champ F suit la convention distance−0,5. La demi-épaisseur vraie vaut
  // donc F+0,5, pas F. Ce demi-pixel n'est pas un détail cosmétique : pris à F,
  // le disque inscrit s'arrête pile avant sa propre couronne, et tout le
  // pourtour de chaque section gardait sa profondeur locale (0,5 px → 0,1 mm),
  // ce qui faisait passer un montant de 8 mm pour une paroi de 0,11 mm.
  // Rayon de peinture et VALEUR ne sont pas la même quantité. Le rayon doit
  // valoir F+0,5 pour que le disque atteigne la surface réelle. La valeur, elle,
  // ne peut pas être 2×(F+0,5) : une paroi de 7 pixels et une de 8 donnent la
  // même distance maximale (4), donc ce choix surestimerait systématiquement
  // les largeurs impaires d'un pixel entier — assez pour faire passer une paroi
  // de 0,35 mm juste au-dessus d'un seuil de buse de 0,40. 2F+0,5 répartit
  // l'erreur symétriquement : ±un demi-pixel, sans biais.
  for(let k=0;k<order.length;k++){
    const p=order[k], rad=F[p]+0.5, t=2*F[p]+0.5;
    const px0=p%W, py0=(p/W)|0, r=Math.ceil(rad), r2=rad*rad;
    const jA=Math.max(0,py0-r), jB=Math.min(H-1,py0+r);
    for(let j=jA;j<=jB;j++){
      const dy=j-py0, w=Math.floor(Math.sqrt(Math.max(0,r2-dy*dy)));
      const iA=Math.max(0,px0-w), iB=Math.min(W-1,px0+w), base=j*W;
      for(let i=iA;i<=iB;i++){ const q=base+i; if(out[q]<t) out[q]=t; }
    }
  }
  return maxd;
}

// Dilatation binaire séparable (fenêtre carrée 2r+1). Sert à répondre à « cette
// matière repose-t-elle sur quelque chose », en tolérant le décalage normal
// d'une couche à l'autre : un mur à 50° avance de step/tan(50°) par couche, ce
// qui est imprimable ; au-delà, la buse extrude au-dessus du vide.
// Les deux passes balaient en lignes — jamais en colonnes, dont le pas de W
// flottants coûte un défaut de cache par pixel (leçon de l'EDT de la trancheuse).
export function slDilate(src,W,H,r,dst,tmp){
  for(let j=0;j<H;j++){
    const b=j*W;
    for(let i=0;i<W;i++){
      const a=i>r?i-r:0, z=i+r<W-1?i+r:W-1;
      let v=0; for(let k=a;k<=z;k++) if(src[b+k]){v=1;break;}
      tmp[b+i]=v;
    }
  }
  dst.fill(0,0,W*H);
  for(let j=0;j<H;j++){
    const b=j*W, a=j>r?j-r:0, z=j+r<H-1?j+r:H-1;
    for(let k=a;k<=z;k++){ const bk=k*W; for(let i=0;i<W;i++) if(tmp[bk+i]) dst[b+i]=1; }
  }
}

// « Cette matière est-elle plus épaisse que T ? » est exactement une OUVERTURE
// morphologique par un disque de rayon T/2 : l'ouverture efface tout ce qu'un
// tel disque ne peut pas couvrir de l'intérieur. Et une ouverture se calcule
// par deux transformées de distance, sans peindre un seul disque :
//   1. les centres valides sont les pixels de profondeur ≥ r  (déjà dans F) ;
//   2. la distance à ce jeu de centres dit si un disque nous recouvre.
// Le résultat est EXACT pour un seuil binaire — un disque plus grand qui tient
// contient toujours un disque de rayon r — alors que la peinture explicite des
// disques coûtait 81 % du temps total mesuré.
export function slOpenThin(F,sub,W,H,rPx,C,out){
  const n=W*H;
  for(let i=0;i<n;i++) C.seed[i]=(sub[i]&&F[i]>=rPx)?0:SL_BIG;
  slEdt(C.seed,W,H,C.e2,C);
  // Rayon d'érosion (rPx) et rayon de recouvrement (rPx+0,5) ne sont PAS le
  // même nombre : le disque inscrit centré sur un pixel de profondeur F a pour
  // rayon réel F+0,5, la matière débordant d'un demi-pixel au-delà du dernier
  // centre plein. Prendre rPx des deux côtés sous-couvre d'un demi-pixel et
  // déclare fines les bordures de n'importe quel volume épais.
  const lim=(rPx+0.5)*(rPx+0.5);
  let nThin=0;
  for(let i=0;i<n;i++){ const t=(sub[i]&&C.e2[i]>lim)?1:0; out[i]=t; nThin+=t; }
  return nThin;
}

export async function slAnalyzeThickness(tris,cfg,onProgress){
  const P=slPrepare(tris,cfg.bed);
  const height=P.size[2];
  if(height<=0) throw new Error('Modèle sans hauteur exploitable.');
  // RÉSOLUTIONS. L'épaisseur mesurée se quantifie par pas de 2×px : à 0,1 mm/px
  // une paroi de 0,35 mm ressort à 0,40 mm, soit pile sur un seuil de buse de
  // 0,4 — le verdict devient un tirage au sort. Il faut donc viser un quantum
  // nettement plus fin que le seuil de décision : px = seuil/8 → quantum au
  // quart du seuil. Idem en Z : une plaque plus mince que le pas de coupe est
  // indétectable, d'où le plancher à seuil/2 pour les petites pièces.
  const step=Math.max(Math.min(cfg.seuil/2,0.25), height/200);
  const nL=Math.max(1,Math.floor(height/step));
  const margin=2*cfg.seuil;
  // Mode DÉPISTAGE : raster grossier et seuil généreux (2× le seuil réel). Si
  // rien n'y descend sous le double du seuil, alors rien n'est sous le seuil —
  // conclusion sûre obtenue à une fraction du coût. Sinon on refait tout fin.
  const screen=!!cfg.screen;
  const thrCrit=screen?cfg.seuil*2:cfg.seuil;
  let px=cfg.px||Math.min(cfg.seuil/8,0.1);
  const spanX=P.box.x1-P.box.x0+2*margin, spanY=P.box.y1-P.box.y0+2*margin;
  while((Math.ceil(spanX/px)+2)*(Math.ceil(spanY/px)+2)>7e5) px*=1.25;
  const W=Math.ceil(spanX/px)+2, H=Math.ceil(spanY/px)+2, WH=W*H;
  const R={W,H,px,x0:P.box.x0-margin-px,y0:P.box.y0-margin-px,
           cnt:new Int32Array(H),off:new Int32Array(H),cx:new Float64Array(1024),cd:new Int8Array(1024)};
  const mx=Math.max(W,H);
  const C={f:new Float64Array(mx),d:new Float64Array(mx),v:new Int32Array(mx),z:new Float64Array(mx+1),
           seed:new Float32Array(WH),e1:new Float32Array(WH),e2:new Float32Array(WH),t:new Float32Array(WH)};
  const F=new Float32Array(WH), loc=new Float32Array(WH), mask=new Uint8Array(WH), sub=new Uint8Array(WH), bb=new Int32Array(5);
  let minSpan=Infinity;   // plus fine corde horizontale rencontrée, en mm
  const roi={rw:0,rh:0,i0:0,j0:0};
  // suivi des suites verticales : depuis quelle couche ce pixel est-il plein ?
  const runFrom=new Int32Array(WH).fill(-1);
  let minZrun=Infinity, minZpx=0;
  // ── supports : matière qui n'a rien sous elle ────────────────────────────
  const prevM=new Uint8Array(WH), pv=new Uint8Array(WH), dil=new Uint8Array(WH),
        tmpD=new Uint8Array(WH), unsup=new Uint8Array(WH), supM=new Uint8Array(WH);
  const dSup=new Float32Array(WH);
  // Ponts : une matière en l'air tenue de DEUX CÔTÉS OPPOSÉS est un pont, que
  // la buse franchit sans support ; tenue d'un seul côté, c'est un vrai
  // porte-à-faux. Ce qui décide n'est donc pas la surface mais la PORTÉE, et
  // c'est elle aussi qui dit si le cordon va s'affaisser.
  const BRIDGE_MAX=25;      // mm — au-delà, un pont s'affaisse même bien refroidi
  let brCount=0, brMaxSpan=0, brArea=0, brLong=0;
  const lastZ=new Float32Array(WH);
  const OVER=50;                                   // ° — même limite que la carte 3
  const rSup=Math.max(1,Math.round((step/Math.tan(OVER*Math.PI/180))/px));
  const SUPFILL=0.15;                              // remplissage typique d'un support
  let supArea=0, supVol=0, supLayers=0, supMaxIsland=0, supIslands=0;

  const cell=px*px*step, cellA=px*px;
  const MINZONE=0.5;   // mm² — sous cette surface, ce n'est pas une paroi, c'est un angle
  // Zones connexes de matière sous un seuil. Indispensable : dans un angle, le
  // plus grand disque inscrit tend vers zéro, donc CHAQUE coin de CHAQUE pièce
  // signale une épaisseur nulle. Un minimum brut est structurellement
  // inexploitable ; une paroi réellement mince, elle, forme une plage étendue.
  const seen=new Uint8Array(WH), stack=new Int32Array(WH),
        thinC=new Uint8Array(WH), thinW=new Uint8Array(WH);
  // Composantes connexes d'un masque binaire, filtrées par surface. `withVal`
  // demande en plus l'épaisseur minimale réelle — seul cas où le champ exact
  // d'épaisseur locale est nécessaire, donc calculé seulement là.
  const zonesOf=(bin,withVal)=>{
    const {rw,rh}=roi, n0=rw*rh;
    seen.fill(0,0,n0);
    let nb=0, area=0, minT=Infinity;
    for(let p0=0;p0<n0;p0++){
      if(seen[p0]||!bin[p0]) continue;
      let sp=0, n=0, lo=Infinity; stack[sp++]=p0; seen[p0]=1;
      while(sp){
        const p=stack[--sp]; n++;
        if(withVal){ const t=loc[p]*px; if(t<lo) lo=t; }
        const x=p%rw, y=(p/rw)|0;
        if(x>0)    { const q=p-1;  if(!seen[q]&&bin[q]){seen[q]=1;stack[sp++]=q;} }
        if(x<rw-1) { const q=p+1;  if(!seen[q]&&bin[q]){seen[q]=1;stack[sp++]=q;} }
        if(y>0)    { const q=p-rw; if(!seen[q]&&bin[q]){seen[q]=1;stack[sp++]=q;} }
        if(y<rh-1) { const q=p+rw; if(!seen[q]&&bin[q]){seen[q]=1;stack[sp++]=q;} }
      }
      if(n*cellA>=MINZONE){ nb++; area+=n*cellA; if(lo<minT)minT=lo; }
    }
    return {nb, area, minT};
  };
  // rayons de disque correspondant aux deux seuils (épaisseur = 2F+0,5 px)
  const rCrit=(thrCrit/px-0.5)/2, rWarn=(2*cfg.seuil/px-0.5)/2;
  // Recopie de l'emprise réelle dans une grille compacte : champ de distance,
  // épaisseur locale et zones connexes ne travaillent alors que sur la matière,
  // pas sur la grille de la pièce entière (une section haute d'une pièce
  // élancée en occupe quelques pour cent).
  const toROI=(pad=2)=>{
    const i0=Math.max(0,bb[0]-pad), i1=Math.min(W-1,bb[1]+pad);
    const j0=Math.max(0,bb[2]-pad), j1=Math.min(H-1,bb[3]+pad);
    const rw=i1-i0+1, rh=j1-j0+1;
    for(let j=0;j<rh;j++) sub.set(mask.subarray((j0+j)*W+i0,(j0+j)*W+i1+1), j*rw);
    roi.rw=rw; roi.rh=rh; roi.i0=i0; roi.j0=j0;
    _lap('copie');
    slField(sub,rw,rh,F,C,false);
    _lap('field');
  };
  let critZones=0, critVol=0, critMin=Infinity, critLayers=0, warnVol=0, totVol=0;
  let ptr=0, active=[]; const seg=new Float64Array(4); let segs=[];
  const zAt=i=>Math.min(height-1e-4,(i+0.5)*step);
  let lastYield=performance.now();
  const PROF={raster:0,zrun:0,field:0,ouverture:0,local:0,zones:0,supports:0,copie:0};
  let _t=performance.now(); const _lap=k=>{ const n=performance.now(); PROF[k]+=n-_t; _t=n; };

  for(let i=0;i<nL;i++){
    const z=zAt(i);
    while(ptr<P.n && P.zlo[P.ord[ptr]]<=z) active.push(P.ord[ptr++]);
    if(active.length) active=active.filter(t=>P.zhi[t]>=z);
    segs.length=0;
    for(const t of active) if(slTriSeg(P,t,z,seg)) segs.push(seg[0],seg[1],seg[2],seg[3]);
    slFill(segs,R,mask,bb);
    if(bb[4]<2147483647){ const s=bb[4]/1000; if(s<minSpan) minSpan=s; }
    _lap('raster');
    // suites verticales : une suite qui s'achève donne l'épaisseur en Z
    for(let k=0;k<WH;k++){
      if(mask[k]){ if(runFrom[k]<0) runFrom[k]=i; }
      else if(runFrom[k]>=0){
        const len=(i-runFrom[k])*step;
        if(len<minZrun){ minZrun=len; minZpx=1; } else if(len===minZrun) minZpx++;
        runFrom[k]=-1;
      }
    }
    _lap('zrun');
    if(bb[1]>=0){
      toROI(Math.max(2,rSup+1));
      const {rw,rh,i0,j0}=roi, n0=rw*rh;
      let np=0; for(let k=0;k<n0;k++) if(sub[k]) np++;
      totVol+=np*cell;
      // ── supports : la matière de cette couche repose-t-elle sur la précédente ?
      if(i>0){
        for(let j=0;j<rh;j++) pv.set(prevM.subarray((j0+j)*W+i0,(j0+j)*W+i0+rw), j*rw);
        slDilate(pv,rw,rh,rSup,dil,tmpD);
        let nUn=0;
        for(let k=0;k<n0;k++){ const u=(sub[k]&&!dil[k])?1:0; unsup[k]=u; if(u)nUn++; }
        if(nUn){
          // Matière de cette couche qui, elle, repose bien sur la précédente :
          // ce sont les ancrages possibles d'un pont.
          for(let k=0;k<n0;k++) supM[k]=(sub[k]&&dil[k])?1:0;
          for(let k=0;k<n0;k++) C.seed[k]=supM[k]?0:SL_BIG;
          slEdt(C.seed,rw,rh,dSup,C);      // distance à l'ancrage le plus proche
          // Depuis le point le plus éloigné de tout ancrage, on marche dans
          // quatre paires de directions opposées. Si deux directions opposées
          // atteignent toutes deux de la matière portée sans sortir de la
          // pièce, la zone est FRANCHIE — c'est un pont, pas un porte-à-faux —
          // et la somme des deux distances est sa portée.
          const MAXSTEP=Math.ceil(BRIDGE_MAX*1.6/px);
          const walk=(cx,cy,dx,dy)=>{
            const len=Math.hypot(dx,dy)*px;
            for(let s=1;s<=MAXSTEP;s++){
              const x=cx+dx*s, y=cy+dy*s;
              if(x<0||y<0||x>=rw||y>=rh) return -1;
              const q=y*rw+x;
              if(supM[q]) return s*len;
              if(!sub[q]) return -1;        // sorti de la matière : pas d'ancrage par là
            }
            return -1;
          };
          const PAIRS=[[1,0],[0,1],[1,1],[1,-1]];
          let area=0, big=0, nIsl=0;
          seen.fill(0,0,n0);
          for(let p0=0;p0<n0;p0++){
            if(seen[p0]||!unsup[p0]) continue;
            let sp=0,n=0,vol=0,deep=p0,dmax=-1; stack[sp++]=p0; seen[p0]=1;
            while(sp){
              const p=stack[--sp]; n++;
              const x=p%rw, y=(p/rw)|0;
              if(dSup[p]>dmax){ dmax=dSup[p]; deep=p; }
              vol+=cellA*Math.max(0,z-lastZ[(j0+y)*W+i0+x])*SUPFILL;
              if(x>0)   {const q=p-1; if(!seen[q]&&unsup[q]){seen[q]=1;stack[sp++]=q;}}
              if(x<rw-1){const q=p+1; if(!seen[q]&&unsup[q]){seen[q]=1;stack[sp++]=q;}}
              if(y>0)   {const q=p-rw;if(!seen[q]&&unsup[q]){seen[q]=1;stack[sp++]=q;}}
              if(y<rh-1){const q=p+rw;if(!seen[q]&&unsup[q]){seen[q]=1;stack[sp++]=q;}}
            }
            const a=n*cellA;
            if(a<1) continue;              // sous 1 mm², c'est du bruit de rastérisation
            const cx=deep%rw, cy=(deep/rw)|0;
            let span=Infinity;
            for(const [dx,dy] of PAIRS){
              const d1=walk(cx,cy,dx,dy), d2=walk(cx,cy,-dx,-dy);
              // La portée retenue est la PLUS COURTE des directions franchies :
              // c'est celle dans laquelle un slicer orientera ses cordons.
              if(d1>=0&&d2>=0&&d1+d2<span) span=d1+d2;
            }
            if(isFinite(span)){
              brCount++; brArea+=a; if(span>brMaxSpan) brMaxSpan=span;
              if(span>BRIDGE_MAX) brLong++;
            } else {
              nIsl++; area+=a; if(a>big)big=a; supVol+=vol;   // vrai porte-à-faux
            }
          }
          if(area>0){ supArea+=area; supLayers++; supIslands+=nIsl; if(big>supMaxIsland)supMaxIsland=big; }
        }
      }
      for(let j=0;j<rh;j++){ const rb=j*rw, fb=(j0+j)*W+i0;
        for(let x=0;x<rw;x++) if(sub[rb+x]) lastZ[fb+x]=z; }
      _lap('supports');
      // Ouvertures : deux transformées de distance au lieu de millions de
      // disques peints. L'épaisseur locale exacte n'est calculée que si une
      // zone critique existe vraiment — c'est-à-dire presque jamais.
      const nC=slOpenThin(F,sub,rw,rh,rCrit,C,thinC);
      // En dépistage, la seconde ouverture (matière « fragile ») ne sert à rien :
      // elle ne décide de rien, et c'est la moitié du coût des ouvertures.
      const nW=screen?0:slOpenThin(F,sub,rw,rh,rWarn,C,thinW);
      _lap('ouverture');
      // Ordre volontaire : le filtre de surface AVANT le calcul exact. Les coins
      // échouent toujours à l'ouverture (le disque inscrit y tend vers zéro) ;
      // sans ce test préalable, chaque couche de chaque pièce saine déclenchait
      // l'épaisseur locale exacte pour des artefacts finalement rejetés — 62 %
      // du temps restant, mesuré.
      if(nC){
        const pre=zonesOf(thinC,false);
        _lap('zones');
        if(pre.nb){
          if(screen){ critZones+=pre.nb; critVol+=pre.area*step; critLayers++; }
          else{
            slLocalThickness(F,rw,rh,loc);
            _lap('local');
            const zc=zonesOf(thinC,true);
            critZones+=zc.nb; critVol+=zc.area*step; critLayers++;
            if(zc.minT<critMin) critMin=zc.minT;
          }
        }
      }
      if(nW) warnVol+=zonesOf(thinW,false).area*step;
      _lap('zones');
      prevM.set(mask);
    } else prevM.fill(0);
    _lap('copie');
    if(performance.now()-lastYield>100){ onProgress&&onProgress((i+1)/nL); await slYield(); lastYield=performance.now(); _t=lastYield; }
  }
  // suites encore ouvertes au sommet
  for(let k=0;k<WH;k++) if(runFrom[k]>=0){
    const len=(nL-runFrom[k])*step;
    if(len<minZrun){ minZrun=len; minZpx=1; }
  }

  const s=cfg.seuil;
  return {P,R,C,F,loc,mask,sub,bb,roi,toROI,px,step,nL,height,seuil:s,W,H,
          minRobust:isFinite(critMin)?critMin:null,
          critZones, critVol, critLayers, warnVol, vol:totVol, minZone:MINZONE,
          supArea, supVol, supLayers, supIslands, supMaxIsland, supAngle:OVER, prof:PROF,
          brCount, brArea, brMaxSpan, brLong, brMax:BRIDGE_MAX,
          screen, minSpan:isFinite(minSpan)?minSpan:null,
          // Le dépistage n'est concluant que s'il ne trouve RIEN : ni zone sous
          // le double du seuil, ni corde plus fine que 1,5× le seuil (ce second
          // test rattrape une paroi trop fine pour survivre au raster grossier).
          needsFine: screen && (critLayers>0 || minSpan<cfg.seuil*1.5),
          minZ:isFinite(minZrun)?minZrun:null,
          // Une plaque n'est mesurée qu'au pas de coupe près : on alerte dès que
          // la barre d'incertitude croise le seuil, pas seulement la valeur
          // nominale — sous-estimer un risque est ici pire que sur-alerter.
          zThin:isFinite(minZrun)&&(minZrun-step/2)<s};
}
