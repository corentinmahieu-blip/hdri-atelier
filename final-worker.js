// Conservative image registration with a sensor prior.
// Harris corners, perspective-warped NCC patches and robust 3D rotation fitting.
// Assumes a fixed camera centre and approximate focal length. No translation/SLAM.
const regDot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const regCross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const regUnit=a=>{const n=Math.hypot(...a);return n>1e-9?a.map(v=>v/n):null;};
const regApply=(R,v)=>R.map(row=>regDot(row,v));
function cameraRay(x,y,w,h,hfov){const f=w/(2*Math.tan(hfov*Math.PI/360));return regUnit([(x-(w-1)/2)/f,((h-1)/2-y)/f,-1]);}
function worldRay(v,b){return v.map((_,i)=>b.right[i]*v[0]+b.up[i]*v[1]-b.forward[i]*v[2]);}
function imagePoint(v,b,w,h,hfov){const depth=regDot(v,b.forward);if(depth<.05)return null;const f=w/(2*Math.tan(hfov*Math.PI/360));return {x:(w-1)/2+f*regDot(v,b.right)/depth,y:(h-1)/2-f*regDot(v,b.up)/depth};}
function sample(g,w,h,x,y){if(x<0||y<0||x>=w-1||y>=h-1)return NaN;const ix=Math.floor(x),iy=Math.floor(y),dx=x-ix,dy=y-iy,i=iy*w+ix;return (g[i]*(1-dx)+g[i+1]*dx)*(1-dy)+(g[i+w]*(1-dx)+g[i+w+1]*dx)*dy;}
function registrationFrame(data,w,h,basis,hfov){
  const width=Math.min(384,w),height=Math.round(width*h/w),g=new Float32Array(width*height);
  for(let y=0;y<height;y++)for(let x=0;x<width;x++){
    const sx=Math.min(w-1,Math.round((x+.5)*w/width-.5)),sy=Math.min(h-1,Math.round((y+.5)*h/height-.5)),i=(sy*w+sx)*4;
    g[y*width+x]=(data[i]*.299+data[i+1]*.587+data[i+2]*.114);
  }
  // 3x3 low-pass before corner selection / patch correlation.
  const gray=g.slice();for(let y=1;y<height-1;y++)for(let x=1;x<width-1;x++){const i=y*width+x;gray[i]=(g[i]*4+(g[i-1]+g[i+1]+g[i-width]+g[i+width])*2+g[i-width-1]+g[i-width+1]+g[i+width-1]+g[i+width+1])/16;}
  const candidates=[];
  for(let y=8;y<height-8;y+=3)for(let x=8;x<width-8;x+=3){
    let xx=0,yy=0,xy=0;
    for(let dy=-2;dy<=2;dy++)for(let dx=-2;dx<=2;dx++){const i=(y+dy)*width+x+dx,gx=gray[i+1]-gray[i-1],gy=gray[i+width]-gray[i-width];xx+=gx*gx;yy+=gy*gy;xy+=gx*gy;}
    const score=xx*yy-xy*xy-.05*(xx+yy)**2;if(score>2e5)candidates.push({x,y,score});
  }
  candidates.sort((a,b)=>b.score-a.score);const features=[];
  for(const p of candidates){if(features.every(q=>(p.x-q.x)**2+(p.y-q.y)**2>11**2)){features.push(p);if(features.length===220)break;}}
  return {gray,width,height,basis,hfov,features};
}
function matchFrames(frame,ref){
  const matches=[],patch=4,radius=40,step=2,w=frame.width,h=frame.height;
  for(const key of ref.features){
    const ray=worldRay(cameraRay(key.x,key.y,ref.width,ref.height,ref.hfov),ref.basis);
    const predicted=imagePoint(ray,frame.basis,w,h,frame.hfov);
    if(!predicted||predicted.x<patch-radius||predicted.x>w-patch+radius||predicted.y<patch-radius||predicted.y>h-patch+radius)continue;
    const template=[];let mean=0,valid=true;
    for(let dy=-patch;dy<=patch;dy++)for(let dx=-patch;dx<=patch;dx++){
      const wr=worldRay(cameraRay(predicted.x+dx,predicted.y+dy,w,h,frame.hfov),frame.basis),q=imagePoint(wr,ref.basis,ref.width,ref.height,ref.hfov);
      const value=q?sample(ref.gray,ref.width,ref.height,q.x,q.y):NaN;
      if(!Number.isFinite(value))valid=false;template.push(value);mean+=value;
    }
    if(!valid)continue;mean/=template.length;let energy=0;for(let i=0;i<template.length;i++){template[i]-=mean;energy+=template[i]**2;}if(energy<800)continue;
    const scoreAt=(x,y)=>{let sum=0,sum2=0,corr=0,n=0;for(let dy=-patch;dy<=patch;dy++)for(let dx=-patch;dx<=patch;dx++){const v=frame.gray[(y+dy)*w+x+dx];sum+=v;sum2+=v*v;corr+=v*template[n++];}const e=sum2-sum*sum/n;return e>100?corr/Math.sqrt(energy*e):-1;};
    const left=Math.max(patch,Math.round(predicted.x)-radius),right=Math.min(w-patch-1,Math.round(predicted.x)+radius),top=Math.max(patch,Math.round(predicted.y)-radius),bottom=Math.min(h-patch-1,Math.round(predicted.y)+radius);
    const scores=[];let best={score:-1};
    for(let y=top;y<=bottom;y+=step)for(let x=left;x<=right;x+=step){const score=scoreAt(x,y);scores.push({x,y,score});if(score>best.score)best={x,y,score};}
    if(best.score<.7)continue;
    const coarse=best;
    for(let y=Math.max(patch,coarse.y-2);y<=Math.min(h-patch-1,coarse.y+2);y++)for(let x=Math.max(patch,coarse.x-2);x<=Math.min(w-patch-1,coarse.x+2);x++){const score=scoreAt(x,y);if(score>best.score)best={x,y,score};}
    let second=-1;for(const s of scores)if((s.x-best.x)**2+(s.y-best.y)**2>8**2)second=Math.max(second,s.score);
    if(best.score<.84||best.score-second<.07)continue;
    matches.push({a:cameraRay(best.x,best.y,w,h,frame.hfov),b:ray,x:best.x,y:best.y,score:best.score});
  }
  return matches;
}
function triad(a,b){const u=regUnit(a),cross=regUnit(regCross(a,b));if(!cross||Math.abs(regDot(a,b))>.998)return null;return [u,regCross(cross,u),cross];}
function rotationPair(p,q){const A=triad(p.a,q.a),B=triad(p.b,q.b);if(!A||!B)return null;return [0,1,2].map(i=>[0,1,2].map(j=>B[0][i]*A[0][j]+B[1][i]*A[1][j]+B[2][i]*A[2][j]));}
function solve3(A,b){const M=A.map((row,i)=>[...row,b[i]]);for(let j=0;j<3;j++){let p=j;for(let i=j+1;i<3;i++)if(Math.abs(M[i][j])>Math.abs(M[p][j]))p=i;[M[p],M[j]]=[M[j],M[p]];if(Math.abs(M[j][j])<1e-8)return null;const d=M[j][j];for(let k=j;k<4;k++)M[j][k]/=d;for(let i=0;i<3;i++)if(i!==j){const c=M[i][j];for(let k=j;k<4;k++)M[i][k]-=c*M[j][k];}}return M.map(row=>row[3]);}
function rotateVector(v,axis){const theta=Math.hypot(...axis);if(theta<1e-10)return v;const k=axis.map(x=>x/theta),c=Math.cos(theta),s=Math.sin(theta),cross=regCross(k,v),d=regDot(k,v);return v.map((x,i)=>x*c+cross[i]*s+k[i]*d*(1-c));}
function refineRotation(R,matches){
  for(let iter=0;iter<6;iter++){
    const A=Array.from({length:3},()=>[0,0,0]),b=[0,0,0];
    for(const m of matches){const q=regApply(R,m.a),v=regCross(q,m.b);for(let i=0;i<3;i++){b[i]+=v[i];for(let j=0;j<3;j++)A[i][j]+=(i===j?1:0)-q[i]*q[j];}}
    const delta=solve3(A,b);if(!delta||Math.hypot(...delta)<1e-9)break;
    const columns=[0,1,2].map(j=>rotateVector(R.map(row=>row[j]),delta));R=[0,1,2].map(i=>columns.map(c=>c[i]));
  }return R;
}
function fitRegistration(matches,prior,width,height){
  // Deduplicate source locations before counting support, including repeated references.
  matches=matches.sort((a,b)=>b.score-a.score).filter((m,i,list)=>list.slice(0,i).every(p=>(p.x-m.x)**2+(p.y-m.y)**2>5**2));
  if(matches.length<14)return {accepted:false,reason:'not-enough-matches',matches:matches.length};
  const threshold=Math.cos(.8*Math.PI/180);let best=[],bestR=null,seed=1847;
  const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
  for(let i=0;i<200;i++){
    const a=random(matches.length),b=random(matches.length);if(a===b)continue;const R=rotationPair(matches[a],matches[b]);if(!R)continue;
    const inliers=matches.filter(m=>regDot(regApply(R,m.a),m.b)>threshold);if(inliers.length>best.length){best=inliers;bestR=R;}
  }
  if(best.length<14||best.length/matches.length<.55)return {accepted:false,reason:'inconsistent-matches',matches:matches.length,inliers:best.length};
  bestR=refineRotation(bestR,best);best=matches.filter(m=>regDot(regApply(bestR,m.a),m.b)>threshold);
  const cells=new Set(best.map(m=>Math.floor(m.x/width*4)+','+Math.floor(m.y/height*3)));
  const spanX=Math.max(...best.map(m=>m.x))-Math.min(...best.map(m=>m.x)),spanY=Math.max(...best.map(m=>m.y))-Math.min(...best.map(m=>m.y));
  if(best.length<14||cells.size<4||spanX<width*.12||spanY<height*.18)return {accepted:false,reason:'poor-coverage',matches:matches.length,inliers:best.length};
  const basis={right:bestR.map(row=>row[0]),up:bestR.map(row=>row[1]),forward:bestR.map(row=>-row[2])};
  const deg=v=>Math.acos(Math.max(-1,Math.min(1,v)))*180/Math.PI;
  const correction=Math.max(deg(regDot(prior.forward,basis.forward)),deg(regDot(prior.up,basis.up)));
  const error=Math.sqrt(best.reduce((sum,m)=>sum+deg(regDot(regApply(bestR,m.a),m.b))**2,0)/best.length);
  if(correction>12||error>.55)return {accepted:false,reason:'excessive-correction',matches:matches.length,inliers:best.length,error};
  return {accepted:true,basis,inliers:best.length,matches:matches.length,error,correction};
}
class ImageRegistrar {
  constructor(){this.references=[];}
  register(data,w,h,basis,hfov,skip=false){
    const frame=registrationFrame(data,w,h,basis,hfov);
    const candidates=this.references.map(ref=>({ref,overlap:regDot(ref.basis.forward,basis.forward)})).filter(x=>x.overlap>.42).sort((a,b)=>b.overlap-a.overlap).slice(0,3);
    const matches=skip?[]:candidates.flatMap(({ref})=>matchFrames(frame,ref));
    const result=skip?{accepted:false,reason:'stored-pose'}:fitRegistration(matches,basis,frame.width,frame.height);
    if(!this.references.length)result.reason='first-photo';
    if(result.accepted)frame.basis=result.basis;
    this.references.push(frame);return {...result,basis:frame.basis};
  }
}

// High-resolution spherical preview. Select one source per pixel instead of
// averaging misaligned photos. This prevents blending ghosts, but seams remain
// possible when image registration or the camera-centre assumption is imperfect.
class Mosaic {
  constructor(width=4096,height=2048){
    this.width=width;this.height=height;this.data=new Uint8ClampedArray(width*height*4);this.score=new Float32Array(width*height);
    this.sx=Float32Array.from({length:width},(_,x)=>Math.sin(((x+.5)/width-.5)*Math.PI*2));
    this.cx=Float32Array.from({length:width},(_,x)=>Math.cos(((x+.5)/width-.5)*Math.PI*2));
    this.sp=Float32Array.from({length:height},(_,y)=>Math.sin((.5-(y+.5)/height)*Math.PI));
    this.cp=Float32Array.from({length:height},(_,y)=>Math.cos((.5-(y+.5)/height)*Math.PI));
  }
  add(data,iw,ih,basis,hfov,warp=null){
    const tx=Math.tan(hfov*Math.PI/360),ty=tx*ih/iw,{right:r,up:u,forward:f}=basis;
    const halfDiagonal=Math.atan(Math.hypot(tx,ty)),latitude=Math.asin(Math.max(-1,Math.min(1,f[2])));
    const y0=Math.max(0,Math.floor((.5-(latitude+halfDiagonal)/Math.PI)*this.height)-1),y1=Math.min(this.height,Math.ceil((.5-(latitude-halfDiagonal)/Math.PI)*this.height)+1);
    for(let row=y0;row<y1;row++){
      const cp=this.cp[row],z=this.sp[row];
      for(let col=0;col<this.width;col++){
        const x=this.sx[col]*cp,y=this.cx[col]*cp,depth=x*f[0]+y*f[1]+z*f[2];if(depth<=0)continue;
        const nx=(x*r[0]+y*r[1]+z*r[2])/(depth*tx),ny=(x*u[0]+y*u[1]+z*u[2])/(depth*ty);
        if(Math.abs(nx)>=1||Math.abs(ny)>=1)continue;
        const pixel=row*this.width+col,weight=depth**4*Math.min(1,(1-Math.abs(nx))*8,(1-Math.abs(ny))*8);
        if(weight<=this.score[pixel]+1e-7)continue;
        let px=(nx+1)*.5*(iw-1),py=(1-ny)*.5*(ih-1);
        if(warp){
          const gx=Math.max(0,Math.min(warp.cols-1,px/(iw-1)*(warp.cols-1))),gy=Math.max(0,Math.min(warp.rows-1,py/(ih-1)*(warp.rows-1)));
          const ax=Math.min(warp.cols-2,Math.floor(gx)),ay=Math.min(warp.rows-2,Math.floor(gy)),fx=gx-ax,fy=gy-ay;let ox=0,oy=0;
          for(let yy=0;yy<2;yy++)for(let xx=0;xx<2;xx++){const ww=(xx?fx:1-fx)*(yy?fy:1-fy),kk=((ay+yy)*warp.cols+ax+xx)*2;ox+=warp.values[kk]*ww;oy+=warp.values[kk+1]*ww;}
          px+=ox*iw;py+=oy*ih;if(px<0||py<0||px>=iw-1||py>=ih-1){continue;}
        }
        const ix=Math.floor(px),iy=Math.floor(py),dx=px-ix,dy=py-iy;
        this.score[pixel]=weight;
        const k=(iy*iw+ix)*4,kx=(iy*iw+Math.min(ix+1,iw-1))*4,ky=(Math.min(iy+1,ih-1)*iw+ix)*4,kxy=(Math.min(iy+1,ih-1)*iw+Math.min(ix+1,iw-1))*4,j=pixel*4;
        for(let c=0;c<3;c++)this.data[j+c]=(data[k+c]*(1-dx)+data[kx+c]*dx)*(1-dy)+(data[ky+c]*(1-dx)+data[kxy+c]*dx)*dy;
        this.data[j+3]=255;
      }
    }
  }
  pixels(){return this.data.slice();}
}

// Offline final assembly: ORB matches, conservative pair filtering and robust
// global rotation adjustment. Assumes one optical centre; no depth or local warp.

const finalDot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const finalCross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const finalNorm=a=>Math.hypot(...a);
function finalRotate(b,v){return {right:rotateVector(b.right,v),up:rotateVector(b.up,v),forward:rotateVector(b.forward,v)};}
function finalCopyBasis(b){return {right:[...b.right],up:[...b.up],forward:[...b.forward]};}
function warpAt(warp,x,y){
 if(!warp)return [0,0];const gx=Math.max(0,Math.min(warp.cols-1,x*(warp.cols-1))),gy=Math.max(0,Math.min(warp.rows-1,y*(warp.rows-1)));
 const ix=Math.min(warp.cols-2,Math.floor(gx)),iy=Math.min(warp.rows-2,Math.floor(gy)),dx=gx-ix,dy=gy-iy;const out=[0,0];
 for(let j=0;j<2;j++)for(let i=0;i<2;i++){const weight=(i?dx:1-dx)*(j?dy:1-dy),k=((iy+j)*warp.cols+ix+i)*2;out[0]+=warp.values[k]*weight;out[1]+=warp.values[k+1]*weight;}return out;
}
// Local corrections are estimated from an 80% subset and kept only if a
// reserved 20% subset reduces median correspondence residuals; nearby or
// repeated features can still be shared across different photo pairs.
function localCorrections(frames,edges,bases,group){
 const points=frames.map(()=>[]),cols=8,rows=6;
 for(const edge of edges){if(!group.has(edge.i)||!group.has(edge.j))continue;
  const f=frames[edge.i],g=frames[edge.j];let k=0;
  for(const [a,b] of edge.pairs){
   const u=worldRay(cameraRay(a.x,a.y,f.width,f.height,f.hfov),bases[edge.i]),v=worldRay(cameraRay(b.x,b.y,g.width,g.height,g.hfov),bases[edge.j]);
   if(finalNorm(finalCross(u,v))>.055){k++;continue;}
   const len=finalNorm([u[0]+v[0],u[1]+v[1],u[2]+v[2]]);if(!len){k++;continue;}
   const mid=[(u[0]+v[0])/len,(u[1]+v[1])/len,(u[2]+v[2])/len];
   for(const [id,p] of [[edge.i,a],[edge.j,b]]){
    const frame=frames[id],projected=imagePoint(mid,bases[id],frame.width,frame.height,frame.hfov);if(!projected)continue;
    const dx=(p.x-projected.x)/frame.width,dy=(p.y-projected.y)/frame.height;
    if(Math.hypot(dx,dy)>.035)continue;
    points[id].push({x:projected.x/frame.width,y:projected.y/frame.height,dx,dy,holdout:k%5===0});
   }k++;
  }
 }
 const warps=[],checks=[];
 for(const list of points){
  const train=list.filter(p=>!p.holdout),held=list.filter(p=>p.holdout),values=new Float32Array(cols*rows*2);
  if(train.length<30||held.length<8){warps.push(null);checks.push(null);continue;}
  for(let y=0;y<rows;y++)for(let x=0;x<cols;x++){
   const nx=x/(cols-1),ny=y/(rows-1);let sum=0,ux=0,uy=0;
   for(const p of train){const d=((p.x-nx)/.16)**2+((p.y-ny)/.19)**2;if(d>9)continue;const w=Math.exp(-d*2);sum+=w;ux+=w*p.dx;uy+=w*p.dy;}
   const i=(y*cols+x)*2;if(sum>2){values[i]=Math.max(-.015,Math.min(.015,ux/sum*.75));values[i+1]=Math.max(-.015,Math.min(.015,uy/sum*.75));}
  }
  const warp={cols,rows,values:Array.from(values)},errors=held.map(p=>{const [dx,dy]=warpAt(warp,p.x,p.y);return [Math.hypot(p.dx,p.dy),Math.hypot(p.dx-dx,p.dy-dy)];});
  const med=x=>x.sort((a,b)=>a-b)[Math.floor(x.length/2)],before=med(errors.map(x=>x[0])),after=med(errors.map(x=>x[1]));
  const accept=before>0&&after<before*.95;warps.push(accept?warp:null);checks.push({points:held.length,before,after,accepted:accept});
 }
 return {warps,checks};
}
class FinalEngine {
 constructor(cv,method="orb"){this.cv=cv;this.frames=[];this.edges=[];this.maxDistance=method==="akaze"?120:70;this.orb=method==="akaze"?new cv.AKAZE():new cv.ORB(1600,1.2,8,31,0,2,0,31,12);this.matcher=new cv.BFMatcher(cv.NORM_HAMMING,false);}
 add(data,width,height,basis,hfov,id){
  const cv=this.cv,rgba=cv.matFromArray(height,width,cv.CV_8UC4,data),gray=new cv.Mat(),mask=new cv.Mat(),keys=new cv.KeyPointVector(),des=new cv.Mat();
  try{
   cv.cvtColor(rgba,gray,cv.COLOR_RGBA2GRAY);this.orb.detectAndCompute(gray,mask,keys,des);
   const points=Array.from({length:keys.size()},(_,i)=>{const p=keys.get(i).pt,x=Math.max(0,Math.min(width-1,Math.round(p.x))),y=Math.max(0,Math.min(height-1,Math.round(p.y)));return {x:p.x,y:p.y,l:gray.data[y*width+x]};});
   this.frames.push({id,width,height,basis:finalCopyBasis(basis),hfov,points,des});
  }catch(e){des.delete();throw e;}finally{rgba.delete();gray.delete();mask.delete();keys.delete();}
 }
 pair(i,j){
  const cv=this.cv,a=this.frames[i],b=this.frames[j];if(a.des.rows<2||b.des.rows<2)return null;
  const matches=new cv.DMatchVectorVector();let src=null,dst=null,mask=null,H=null;
  try{
   this.matcher.knnMatch(a.des,b.des,matches,2);const pairs=[];
   for(let k=0;k<matches.size();k++){const v=matches.get(k);try{if(v.size()<2)continue;const m=v.get(0),n=v.get(1);if(m.distance < .78*n.distance&&m.distance<this.maxDistance)pairs.push([a.points[m.queryIdx],b.points[m.trainIdx]]);}finally{v.delete();}}
   if(pairs.length<18)return null;
   src=cv.matFromArray(pairs.length,1,cv.CV_32FC2,pairs.flatMap(p=>[p[0].x,p[0].y]));dst=cv.matFromArray(pairs.length,1,cv.CV_32FC2,pairs.flatMap(p=>[p[1].x,p[1].y]));mask=new cv.Mat();
   H=cv.findHomography(src,dst,cv.RANSAC,3,mask,1500,.995);
   if(H.empty()||mask.empty())return null;
   const inliers=pairs.filter((_,k)=>mask.data[k]);if(inliers.length<16)return null;
   // Even sampling avoids letting a dense repeated pattern dominate one pair.
   const sample=Array.from({length:Math.min(100,inliers.length)},(_,k)=>inliers[Math.floor(k*inliers.length/Math.min(100,inliers.length))]);
   return {i,j,pairs:sample,total:inliers.length};
  }finally{matches.delete();src?.delete();dst?.delete();mask?.delete();H?.delete();}
 }
 match(progress=()=>{}){
  const n=this.frames.length,candidates=[];
  for(let i=0;i<n;i++)for(let j=i+1;j<n;j++)if(finalDot(this.frames[i].basis.forward,this.frames[j].basis.forward)>.34)candidates.push([i,j]);
  this.edges=[];for(let k=0;k<candidates.length;k++){
   const [i,j]=candidates[k];const pair=this.pair(i,j);if(pair)this.edges.push(pair);
   if(k%4===0||k+1===candidates.length)progress(k+1,candidates.length,this.edges.length);
  }
  return {candidatePairs:candidates.length,edges:this.edges.length,points:this.edges.reduce((n,e)=>n+e.pairs.length,0)};
 }
 solve(progress=()=>{}){
  const n=this.frames.length,frames=this.frames,bases=frames.map(x=>finalCopyBasis(x.basis));
  const support=Array(n).fill(0);for(const e of this.edges){support[e.i]+=e.pairs.length;support[e.j]+=e.pairs.length;}
  const seen=new Set(),components=[];
  for(let i=0;i<n;i++)if(!seen.has(i)){const stack=[i],group=[];seen.add(i);while(stack.length){const v=stack.pop();group.push(v);for(const e of this.edges){const other=e.i===v?e.j:e.j===v?e.i:-1;if(other>=0&&!seen.has(other)){seen.add(other);stack.push(other);}}}components.push(group);}
  components.sort((a,b)=>b.length-a.length);const largestGroup=new Set(components[0]||[]);const anchors=new Set(components.map(group=>group.reduce((a,b)=>support[a]>=support[b]?a:b)));
  let cost0=0,cost=0,count=0,iterations=0,fovOffset=0;
  const fovCost=offset=>{let cost=0,count=0;
   for(const edge of this.edges){if(!largestGroup.has(edge.i)||!largestGroup.has(edge.j))continue;const f=frames[edge.i],g=frames[edge.j];
    for(const [a,b] of edge.pairs){const u=worldRay(cameraRay(a.x,a.y,f.width,f.height,f.hfov+offset),bases[edge.i]),v=worldRay(cameraRay(b.x,b.y,g.width,g.height,g.hfov+offset),bases[edge.j]),error=finalNorm(finalCross(u,v));cost+=Math.min(error*error,.04**2);count++;}}
   return count?cost/count+offset*offset*.0000008:Infinity;
  };
  for(let iter=0;iter<90;iter++){iterations=iter+1;
   const mats=Array.from({length:n},()=>[[0,0,0],[0,0,0],[0,0,0]]),vs=Array.from({length:n},()=>[0,0,0]);cost=0;count=0;
   for(const edge of this.edges){const f=frames[edge.i],g=frames[edge.j],bi=bases[edge.i],bj=bases[edge.j];
    for(const [a,b] of edge.pairs){
     const u=worldRay(cameraRay(a.x,a.y,f.width,f.height,f.hfov+fovOffset),bi),v=worldRay(cameraRay(b.x,b.y,g.width,g.height,g.hfov+fovOffset),bj),delta=finalCross(u,v),err=finalNorm(delta);
     // Very large outliers are never pulled into agreement; a Cauchy kernel protects against parallax.
     const weight=1/(1+(err/.025)**2);cost+=Math.min(err*err,.04**2);count++;
     for(const [idx,q,sign] of [[edge.i,u,1],[edge.j,v,-1]]){
      for(let k=0;k<3;k++){vs[idx][k]+=sign*delta[k]*weight;for(let l=0;l<3;l++)mats[idx][k][l]+=weight*((k===l?1:0)-q[k]*q[l]);}
     }
    }
   }
   if(iter===0)cost0=cost;
   let biggest=0;
   for(let i=0;i<n;i++){
    if(!largestGroup.has(i)||anchors.has(i)||support[i]<20)continue;
    for(let k=0;k<3;k++)mats[i][k][k]+=2;
    const d=solve3(mats[i],vs[i]);if(!d)continue;
    let norm=finalNorm(d);if(norm>.035){for(let k=0;k<3;k++)d[k]*=.035/norm;norm=.035;}
    for(let k=0;k<3;k++)d[k]*=.38;
    const candidate=finalRotate(bases[i],d),correction=Math.acos(Math.max(-1,Math.min(1,finalDot(candidate.forward,frames[i].basis.forward))));
    if(correction<.24){bases[i]=candidate;biggest=Math.max(biggest,norm);}
   }
   if(iter%10===9){
    const current=fovCost(fovOffset);let best=current,next=fovOffset;
    for(const candidate of [fovOffset-1,fovOffset+1])if(candidate>=-6&&candidate<=6&&frames.every(f=>f.hfov+candidate>=45&&f.hfov+candidate<=120)){const value=fovCost(candidate);if(value<best*.999){best=value;next=candidate;}}
    fovOffset=next;
   }
   if(iter%5===0)progress(iter+1,90);
   if(biggest<.00002)break;
  }
  // Refuse to report a trustworthy result for disconnected/poorly constrained sets.
  const adjustedFrames=frames.map(f=>({...f,hfov:f.hfov+fovOffset}));
  const local=localCorrections(adjustedFrames,this.edges,bases,largestGroup);
  // Modest global gain per photo based on matched non-clipped feature brightness.
  const gains=Array(n).fill(0),measurements=[];
  for(const e of this.edges){const values=e.pairs.filter(([a,b])=>a.l>30&&a.l<220&&b.l>30&&b.l<220).map(([a,b])=>Math.log((b.l+8)/(a.l+8))).sort((a,b)=>a-b);if(values.length>=12)measurements.push({i:e.i,j:e.j,d:values[Math.floor(values.length/2)]});}
  for(let pass=0;pass<40;pass++)for(let i=0;i<n;i++){
   if(!largestGroup.has(i)||anchors.has(i))continue;let sum=0,num=0;
   for(const e of measurements){if(e.i===i){sum+=gains[e.j]+e.d;num++;}else if(e.j===i){sum+=gains[e.i]-e.d;num++;}}
   if(num)gains[i]=Math.max(Math.log(.7),Math.min(Math.log(1.4),(sum/(num+0.15))));
  }
  const largest=components[0]?.length||0;
  return {bases,trusted:frames.map((_,i)=>largestGroup.has(i)),hfovs:adjustedFrames.map(f=>f.hfov),warps:local.warps,gains:gains.map(Math.exp),summary:{images:n,matchedImages:largest,fovOffset,locallyAdjusted:local.warps.filter(Boolean).length,localChecks:local.checks,pairs:this.edges.length,matches:this.edges.reduce((a,e)=>a+e.pairs.length,0),costBefore:cost0,costAfter:cost,iterations,partial:largest<n}};
 }
 dispose(){for(const f of this.frames)f.des.delete();this.orb.delete();this.matcher.delete();this.frames=[];this.edges=[];}
}

let finalEngine=null,finalMosaic=null;
self.onmessage=async({data:m})=>{
 try{
  if(m.type==='init'){
   importScripts('./opencv.js');const cv=await self.cv;
   if(!cv||typeof cv.ORB!=='function')throw Error('Moteur photo indisponible.');
   finalEngine=new FinalEngine(cv);self.postMessage({id:m.id,ok:true});
  }else if(m.type==='feature'){
   finalEngine.add(m.data,m.width,m.height,m.basis,m.hfov,m.shotId);self.postMessage({id:m.id,ok:true});
  }else if(m.type==='solve'){
   const matches=finalEngine.match((n,total,edges)=>self.postMessage({progress:`Recherche des raccords : ${n}/${total} · ${edges} paires reconnues`}));
   if(matches.edges<1)throw Error('Aucun raccord fiable détecté. Utilise les photos originales.');
   const result=finalEngine.solve((n,total)=>self.postMessage({progress:`Ajustement des images : ${n}/${total}`}));
   finalEngine.dispose();finalEngine=null;self.postMessage({id:m.id,...result});
  }else if(m.type==='renderInit'){
   finalMosaic=new Mosaic(m.width,m.height);self.postMessage({id:m.id,ok:true});
  }else if(m.type==='renderAdd'){
   if(m.gain!==1){for(let i=0;i<m.data.length;i+=4)for(let c=0;c<3;c++)m.data[i+c]=Math.min(255,m.data[i+c]*m.gain);}
   finalMosaic.add(m.data,m.width,m.height,m.basis,m.hfov,m.warp||null);self.postMessage({id:m.id,ok:true});
  }else if(m.type==='renderFinish'){
   const pixels=finalMosaic.pixels();finalMosaic=null;self.postMessage({id:m.id,pixels},[pixels.buffer]);
  }
 }catch(e){self.postMessage({id:m.id,error:e?.message||String(e)});}
};
