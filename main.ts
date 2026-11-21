import{mkdirSync as m,readdirSync as r,readFileSync as f,writeFileSync as w}from'node:fs';
const I=process.env.MAKEFUN_INPUT_DIR??'/input',O=process.env.MAKEFUN_OUTPUT_DIR??'/output';
let W,H,P,G,N,S=0;
let L=z=>z<-1?'u':z>1?'d':z<0?'l':'r';
function B(o,b,s){let d=[],p=[],q=[s];d[s]=0;for(let k=0;q[k];k++){let x=q[k];for(let z of N){let n=x+z;if(!o[n]||b.includes(n)||d[n]>=0)continue;d[n]=d[x]+1;p[n]=x;q.push(n)}}return[d,p]}
function K(o,b,x,y){if(x==y)return'';let[d,p]=B(o,b,x);if(d[y]==null)return null;let s='',n=y;while(n!=x){let q=p[n];s=L(n-q)+s;n=q}return s}
function V(cp,en,b0,gl,et){
let iB=new Uint8Array(W*H),iR=iB.slice();
for(let s of cp)iB[s]=iR[s]=1;
for(let e of en)iB[e]=1;
let dg=gl.map(g=>B(iR,[],g)[0]),dd=new Uint8Array(W*H);
for(let s of cp)if(!G[s]&&(dg.every(d=>d[s]==null)||(!iR[s-W]||!iR[s+W])&&(!iR[s-1]||!iR[s+1])))dd[s]=1;
let he=b=>{let s=0;for(let x of b){let m=1e9;for(let d of dg)if(d[x]>=0&&d[x]<m)m=d[x];s+=m}return s>1e8?1e9:s};
let hp=[],F=[];
function pu(i){hp.push(i);let c=hp.length-1;while(c>0){let p=c-1>>1;if(F[hp[c]]>=F[hp[p]])break;[hp[c],hp[p]]=[hp[p],hp[c]];c=p}}
function po(){let t=hp[0],l=hp.pop();if(hp.length){hp[0]=l;let i=0;for(;;){let c=2*i+1;if(c>=hp.length)break;if(c+1<hp.length&&F[hp[c+1]]<F[hp[c]])c++;if(F[hp[c]]>=F[hp[i]])break;[hp[i],hp[c]]=[hp[c],hp[i]];i=c}}return t}
let bS=[b0.slice().sort((a,b)=>a-b)],pS=[et],pa=[-1],pt=[-1],gS=[0],best=new Map();
let ky=(b,r)=>b.join(':')+':'+r;
let h0=he(bS[0]);if(h0>1e8)return null;
best.set(ky(bS[0],B(iB,bS[0],et)[0].findIndex(x=>x>=0)),0);F[0]=h0;pu(0);
while(hp.length&&S<2e6){
let s=po(),b=bS[s],g=gS[s],d=B(iB,b,pS[s])[0],rp=d.findIndex(x=>x>=0);
if(best.get(ky(b,rp))!==g)continue;
if(b.every(x=>G[x])){
if(en.some(e=>d[e]>=0)){
let ph=[];for(let t=s;t>0;t=pa[t])ph.push([pS[t],pt[t]]);ph.reverse();
let bx2=b0.slice(),mv='',py=et;
for(let[fr,to]of ph){let z=to-fr,wk=K(iB,bx2,py,fr-z);if(wk===null)return null;mv+=wk+L(z);bx2[bx2.indexOf(fr)]=to;py=fr}
for(let e of en){let wk=K(iB,bx2,py,e);if(wk!==null)return[mv+wk,e]}
return null}
continue}
S++;
for(let bi=0;bi<b.length;bi++){let bb=b[bi];
for(let z of N){let to=bb+z;
if(!iR[to]||b.includes(to))continue;
if(d[bb-z]==null)continue;
if(dd[to])continue;
let nb=b.slice();nb[bi]=to;nb.sort((a,b)=>a-b);
let nr=B(iB,nb,bb)[0].findIndex(x=>x>=0);
let k=ky(nb,nr),sg=best.get(k);
if(sg!==undefined&&sg<=g+1)continue;
let hh=he(nb);if(hh>1e8)continue;
let t2=bS.length;bS.push(nb);pS.push(bb);pa.push(s);pt.push(to);gS.push(g+1);F[t2]=g+1+hh;best.set(k,g+1);pu(t2)}}
}
return null}
function solve(t){
let r=t.split('\n').filter(x=>x.length),w=Math.max(...r.map(x=>x.length)),h=r.length;
W=w+2;H=h+2;N=[-W,W,-1,1];P=new Uint8Array(W*H),G=P.slice();
let bx=[],pl=-1;
r.forEach((row,y)=>{for(let x=0;x<row.length;x++){let ch=row[x],i=(y+1)*W+x+1;if(ch!='#'){P[i]=1;if('.*+'.includes(ch))G[i]=1;if('$*'.includes(ch))bx.push(i);if('@+'.includes(ch))pl=i}}});
bx.sort((a,b)=>a-b);
let hy=pl/W|0,sn=new Uint8Array(W*H),rm=[];
for(let i=0;i<W*H;i++){if(!P[i]||(i/W|0)==hy||sn[i])continue;
let cp=[],q=[i];sn[i]=1;
while(q.length){let c=q.pop();cp.push(c);for(let z of N){let n=c+z;if(!P[n]||(n/W|0)==hy||sn[n])continue;sn[n]=1;q.push(n)}}
let en=[],dr=-1;
for(let c of cp)for(let z of N){let n=c+z;if((n/W|0)==hy&&P[n]){dr=c;en.push(n)}}
rm.push([cp,en,dr,bx.filter(b=>cp.includes(b)),cp.filter(c=>G[c])])}
let bs=bx.slice(),mv='',py=pl;
for(let Q of rm.filter(x=>x[3].length).sort((a,b)=>a[2]-b[2])){
if(Q[3].length!=Q[4].length||!Q[1].length)return'';
let wk=K(P,bs,py,Q[1][0]);if(wk===null)return'';
mv+=wk;py=Q[1][0];
let sol=V(Q[0],Q[1],Q[3],Q[4],Q[1][0]);if(!sol)return'';
mv+=sol[0];bs=bs.filter(x=>!Q[3].includes(x)).concat(Q[4]);py=sol[1]}
return mv}
for(let id of r(I).sort()){let mv=solve(f(I+'/'+id+'/level.txt','utf8'));m(O+'/'+id,{recursive:true});w(O+'/'+id+'/solution.txt',mv+'\n')}
