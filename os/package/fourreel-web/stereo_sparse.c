#define _POSIX_C_SOURCE 200809L
#include "stereo_sparse.h"
#include "stereo_trace.h"
#include <math.h>
#include <stdlib.h>

static int inside(double x,double y,int w,int h)
{ return isfinite(x) && isfinite(y) && x>=6 && y>=6 && x<w-7 && y<h-7; }

/* Both directions search the measured epipolar curve independently. Comparing
 * normalized patches at quarter scale suppresses per-eye gain and blur differences. */
enum { PATCH_RADIUS=4, PATCH_SIDE=2*PATCH_RADIUS+1, PATCH_PIXELS=PATCH_SIDE*PATCH_SIDE, STEPS=128 };
struct Patch { int pixel[PATCH_PIXELS]; int sum, energy; };

static int make_patch(const uint8_t *im,double x,double y,struct Patch *p)
{
    if(!inside(x,y,320,200)) return -1;
    int ix=(int)x,iy=(int)y,ax=(int)((x-ix)*16+.5),ay=(int)((y-iy)*16+.5);
    const uint8_t *row=im+(iy-PATCH_RADIUS)*320+ix-PATCH_RADIUS;
    int sum=0,ss=0,k=0;
    for(int j=0;j<PATCH_SIDE;j++,row+=320) for(int i=0;i<PATCH_SIDE;i++){
        int a=row[i]*(16-ax)+row[i+1]*ax,b=row[i+320]*(16-ax)+row[i+321]*ax;
        int v=(a*(16-ay)+b*ay+128)>>8; p->pixel[k++]=v; sum+=v; ss+=v*v;
    }
    p->sum=sum;p->energy=PATCH_PIXELS*ss-sum*sum;
    return p->energy>PATCH_PIXELS*PATCH_PIXELS*9?0:-1;
}

static double similarity(const struct Patch *p,const uint8_t *im,int x,int y)
{
    const uint8_t *row=im+(y-PATCH_RADIUS)*320+x-PATCH_RADIUS;
    int sum=0,ss=0,dot=0,k=0;
    for(int j=0;j<PATCH_SIDE;j++,row+=320) for(int i=0;i<PATCH_SIDE;i++){
        int v=row[i];sum+=v;ss+=v*v;dot+=v*p->pixel[k++];
    }
    int energy=PATCH_PIXELS*ss-sum*sum, cov=PATCH_PIXELS*dot-sum*p->sum;
    return energy>PATCH_PIXELS*PATCH_PIXELS*9 && cov>0 ? cov/sqrt((double)p->energy*energy):0;
}

static int guided_search(const struct StereoImage p[2],const struct StereoCalibration *cal,int source,
                         const double from[2],double to[2],double *quality)
{
    struct Patch patch;
    if(make_patch(p[source].origin,(from[0]-1.5)/4,(from[1]-1.5)/4,&patch)) return -1;
    double ray[3],dir[3],shift[3];
    if(camera_ray(&cal->eye[source],from[0],from[1],ray) || ray[2]<=0) return -1;
    for(int j=0;j<3;j++) {
        dir[j]=0;shift[j]=source ? 0:cal->t[j];
        for(int h=0;h<3;h++) {
            double a=source ? cal->R[h*3+j]:cal->R[j*3+h];
            dir[j]+=a*ray[h]/ray[2];
            if(source) shift[j]-=a*cal->t[h];
        }
    }
    int xs[STEPS],ys[STEPS],n=0,best=-1;
    double scores[STEPS],top=0;
    const uint8_t *other=p[1-source].origin;
    for(int k=0;k<STEPS;k++) {
        double inv=.05+k*(1/.15-.05)/(STEPS-1),pt[3],uv[2];
        for(int j=0;j<3;j++) pt[j]=dir[j]+shift[j]*inv;
        if(pt[2]<=0 || camera_project(&cal->eye[1-source],pt,uv) ||
           uv[0]<256 || uv[0]>1024 || uv[1]<160 || uv[1]>640) continue;
        int x=(int)((uv[0]+.5)/4),y=(int)((uv[1]+.5)/4);
        if(n && x==xs[n-1] && y==ys[n-1]) continue;
        xs[n]=x;ys[n]=y;scores[n]=similarity(&patch,other,x,y);
        if(scores[n]>top){top=scores[n];best=n;} n++;
    }
    if(best<0 || top<.82) return -1;
    double second=-1;
    for(int i=0;i<n;i++) if(abs(xs[i]-xs[best])+abs(ys[i]-ys[best])>3 && scores[i]>second) second=scores[i];
    if(second<0 || 1-top>.80*(1-second)) return -1;
    /* Fit an unconstrained 2-D peak. A spurious winner must also pass the
     * independent reverse search and the native reprojection checks. */
    /* Quantized epipolar samples can sit beside a strong texture peak. */
    int bx=xs[best],by=ys[best],rx=bx,ry=by;
    for(int j=-1;j<=1;j++) for(int i=-1;i<=1;i++) {
        double q=similarity(&patch,other,bx+i,by+j);
        if(q>top){top=q;rx=bx+i;ry=by+j;}
    }
    int x=rx,y=ry;
    double s[3][3];
    for(int j=-1;j<=1;j++) for(int i=-1;i<=1;i++) s[j+1][i+1]=similarity(&patch,other,x+i,y+j);
    double gx=(s[1][2]-s[1][0])*.5,gy=(s[2][1]-s[0][1])*.5;
    double xx=2*top-s[1][0]-s[1][2], yy=2*top-s[0][1]-s[2][1];
    double xy=-(s[2][2]-s[0][2]-s[2][0]+s[0][0])*.25,det=xx*yy-xy*xy;
    if(xx<=0 || yy<=0 || det<=1e-6) return -1;
    double dx=(yy*gx-xy*gy)/det,dy=(xx*gy-xy*gx)/det;
    if(fabs(dx)>1.5 || fabs(dy)>1.5) return -1;
    to[0]=4*(x+dx)+1.5;to[1]=4*(y+dy)+1.5;*quality=top;
    return to[0]>=256 && to[0]<=1024 && to[1]>=160 && to[1]<=640 ? 0:-1;
}

static void half(struct StereoImage image,uint8_t *d,int w,int h)
{
    for(int y=0;y<h/2;++y) for(int x=0;x<w/2;++x) {
        const uint8_t *s=image.origin+2*y*image.stride+2*x*image.xstep;
        d[y*(w/2)+x]=(s[0]+s[image.xstep]+s[image.stride]+s[image.stride+image.xstep]+2)/4;
    }
}

/* Central 768x448 native crop at quarter resolution. All coordinates returned
 * to the caller use the same logical native orientation as calibration. */
enum { CW=192, CH=112, X0=256, Y0=176, COLS=5, ROWS=4, CELLS=COLS*ROWS, SIDE=7, DESC=SIDE*SIDE };
_Static_assert(CELLS<=STEREO_MAX_ANCHORS,"grid exceeds stereo point budget");
struct Feature { short x,y,patch[DESC]; };
struct Nearest { unsigned best,next,index; };
static void thumbnail(struct StereoImage src,uint8_t out[CH][CW])
{
    for(int y=0;y<CH;y++) {
        const uint8_t *p=src.origin+(Y0+4*y+1)*src.stride+(X0+1)*src.xstep;
        for(int x=0;x<CW;x++,p+=4*src.xstep)
            out[y][x]=(p[0]+p[src.xstep]+p[src.stride]+p[src.stride+src.xstep]+2)/4;
    }
}
static unsigned features(uint8_t im[CH][CW],struct Feature f[CELLS])
{
    unsigned count=0;
    for(int cy=0;cy<ROWS;cy++) for(int cx=0;cx<COLS;cx++) {
        int bx=0,by=0,best=150;
        for(int y=7+cy*(CH-14)/ROWS;y<7+(cy+1)*(CH-14)/ROWS;y++) for(int x=7+cx*(CW-14)/COLS;x<7+(cx+1)*(CW-14)/COLS;x++) {
            int xx=0,yy=0,xy=0;
            for(int j=-1;j<=1;j++) for(int i=-1;i<=1;i++){
                int dx=im[y+j][x+i+1]-im[y+j][x+i-1],dy=im[y+j+1][x+i]-im[y+j-1][x+i];
                xx+=dx*dx; yy+=dy*dy; xy+=dx*dy;
            }
            int score=(xx<yy?xx:yy)-abs(xy);
            if(score>best){ best=score; bx=x; by=y; }
        }
        if(!bx) continue;
        struct Feature *p=&f[count++];
        p->x=bx; p->y=by;
        int k=0,sum=0;
        for(int j=-6;j<=6;j+=2) for(int i=-6;i<=6;i+=2) {
            p->patch[k++]=im[by+j][bx+i]; sum+=im[by+j][bx+i];
        }
        int mean=(sum+DESC/2)/DESC;
        for(k=0;k<DESC;k++) p->patch[k]-=mean;
    }
    return count;
}
static void nearest(struct Nearest *v,unsigned cost,unsigned index)
{
    if(cost<v->best){ v->next=v->best; v->best=cost; v->index=index; }
    else if(cost<v->next) v->next=cost;
}
static int unique_patch(struct Nearest v)
{
    return v.next!=UINT32_MAX && v.best<=DESC*14 && 5*v.best+DESC<4*v.next;
}
static unsigned appearance_matches(const struct StereoImage eyes[2],struct StereoMatch *matches,
                                   struct SparseStats *stats,struct Trace *trace)
{
    uint8_t im[2][CH][CW];
    struct Feature f[2][CELLS];
    unsigned n[2];
    struct TraceMark t=trace_begin(trace);
    for(int e=0;e<2;e++) thumbnail(eyes[e],im[e]);
    trace_end(trace,t,TS_THUMB,TR_MAIN);
    t=trace_begin(trace);
    for(int e=0;e<2;e++) n[e]=features(im[e],f[e]);
    stats->candidates=n[0];
    trace_end(trace,t,TS_PATCH,TR_MAIN);
    t=trace_begin(trace);
    struct Nearest near[2][CELLS];
    for(int e=0;e<2;e++) for(unsigned k=0;k<n[e];k++) near[e][k]=(struct Nearest){UINT32_MAX,UINT32_MAX,0};
    for(unsigned i=0;i<n[0];i++) for(unsigned j=0;j<n[1];j++) {
        unsigned sad=0;
        for(unsigned k=0;k<DESC;k++) sad+=abs(f[0][i].patch[k]-f[1][j].patch[k]);
        nearest(&near[0][i],sad,j); nearest(&near[1][j],sad,i);
    }
    trace_end(trace,t,TS_SEARCH,TR_MAIN);
    t=trace_begin(trace);
    for(unsigned i=0;i<n[0];i++) {
        struct Nearest v=near[0][i];
        if(!unique_patch(v)) continue;
        ++stats->forward;
        struct Nearest back=near[1][v.index];
        if(!unique_patch(back) || back.index!=i) continue;
        struct Feature *l=&f[0][i],*r=&f[1][v.index];
        matches[stats->mutual++]=(struct StereoMatch){.xy={{X0+4*l->x+1.5,Y0+4*l->y+1.5},
                                                           {X0+4*r->x+1.5,Y0+4*r->y+1.5}}};
    }
    trace_end(trace,t,TS_MUTUAL,TR_MAIN);
    return stats->mutual;
}

unsigned stereo_match_pair(const struct StereoImage eyes[2],
                       const struct StereoCalibration *cal,struct StereoMatch *matches,struct SparseStats *stats,struct Trace *trace,
                       struct StereoCornerHistory *history)
{
    *stats=(struct SparseStats){0};
    if(!cal) return appearance_matches(eyes,matches,stats,trace);
    if(!stereo_calibration_valid(cal)) return 0;
    stats->calibrated=1;
    uint8_t *storage=malloc(640*400+2*320*200);
    if(!storage) return 0;
    struct StereoImage small[2];
    struct TraceMark timing=trace_begin(trace);
    for(int e=0;e<2;++e) {
        uint8_t *dst=storage+640*400+e*320*200;
        half(eyes[e],storage,1280,800);
        half((struct StereoImage){storage,640,1},dst,640,400);
        small[e]=(struct StereoImage){dst,320,1};
    }
    trace_end(trace,timing,TS_PYRAMID,TR_MAIN);
    timing=trace_begin(trace);
    struct StereoMatch candidates[CELLS];
    uint8_t origin[CELLS],chosen_bit[CELLS];
    unsigned nc=0,nm=0,nr=0,n=0;
    /* One spatially distributed corner per cell in the calibrated overlap. */
    for(int gy=0;gy<ROWS;++gy) for(int gx=0;gx<COLS;++gx) {
        int cell=gy*COLS+gx;
        int x0=64+gx*192/COLS,x1=64+(gx+1)*192/COLS;
        int y0=40+gy*120/ROWS,y1=40+(gy+1)*120/ROWS;
        double best[4]={40,40,40,40};
        int xs[4]={0},ys[4]={0};
        for(int y=y0;y<y1;++y) for(int x=x0;x<x1;++x) {
            double xx=0,xy=0,yy=0;
            for(int j=-2;j<=2;++j) for(int i=-2;i<=2;++i) {
                int pos=(y+j)*320+x+i;
                double dx=((int)small[1].origin[pos+1]-small[1].origin[pos-1])/2.0;
                double dy=((int)small[1].origin[pos+320]-small[1].origin[pos-320])/2.0;
                xx+=dx*dx; xy+=dx*dy; yy+=dy*dy;
            }
            double score=(xx+yy-sqrt((xx-yy)*(xx-yy)+4*xy*xy))/2;
            int q=(x>=(x0+x1)/2)+2*(y>=(y0+y1)/2);
            if(score>best[q]) { best[q]=score; xs[q]=x; ys[q]=y; }
        }
        unsigned available=0;
        for(int q=0;q<4;q++) if(xs[q]) available|=1u<<q;
        if(!available) continue;
        if(history) {
            unsigned untried=available & ~history->rejected[cell];
            if(untried) available=untried;
            else history->rejected[cell]=0;
        }
        int selected=-1;
        for(int q=0;q<4;q++) if((available&(1u<<q)) && (selected<0 || best[q]>best[selected])) selected=q;
        int cx=xs[selected],cy=ys[selected];
        origin[nc]=(uint8_t)cell;
        chosen_bit[cell]=(uint8_t)(1u<<selected);
        /* A successful triangulation below makes this region eligible again. */
        if(history) history->rejected[cell]|=chosen_bit[cell];
        candidates[nc++]=(struct StereoMatch){.xy={{0,0},{4*cx+1.5,4*cy+1.5}}};
    }
    trace_end(trace,timing,TS_CORNERS,TR_MAIN);
    timing=trace_begin(trace);
    for(unsigned c=0;c<nc;++c) {
        struct StereoMatch m=candidates[c];double score;
        if(guided_search(small,cal,1,m.xy[1],m.xy[0],&score)) continue;
        m.weight=score; origin[nm]=origin[c]; candidates[nm++]=m;
    }
    trace_end(trace,timing,TS_MATCH,TR_MAIN);
    timing=trace_begin(trace);
    for(unsigned c=0;c<nm;++c) {
        struct StereoMatch m=candidates[c]; double back[2],score;
        if(guided_search(small,cal,0,m.xy[0],back,&score) ||
           hypot(back[0]-m.xy[1][0],back[1]-m.xy[1][1])>1.5) continue;
        m.weight*=score; origin[nr]=origin[c]; candidates[nr++]=m;
    }
    trace_end(trace,timing,TS_REVERSE,TR_MAIN);
    timing=trace_begin(trace);
    for(unsigned c=0;c<nr;++c) {
        struct StereoMatch m=candidates[c];
        if(stereo_triangulate(cal,m.xy[0],m.xy[1],m.z)) continue;
        if(history) history->rejected[origin[c]] &= (uint8_t)~chosen_bit[origin[c]];
        matches[n++]=m;
    }
    trace_end(trace,timing,TS_TRIANGULATE,TR_MAIN);
    stats->candidates=nc; stats->forward=nm; stats->mutual=nr; stats->anchors=n;
    free(storage);
    return n;
}

unsigned stereo_sparse_views(const struct StereoImage eyes[2],const struct StereoCalibration *cal,
                             struct StereoMatch *matches,struct Trace *trace)
{
    struct SparseStats stats;
    return stereo_match_pair(eyes,cal,matches,&stats,trace,NULL);
}

unsigned stereo_sparse_timed(const uint8_t *left,const uint8_t *right,
                       const struct StereoCalibration *cal,struct StereoMatch *matches,struct Trace *trace)
{
    const struct StereoImage eyes[2]={{left,1280,1},{right,1280,1}};
    return stereo_sparse_views(eyes,cal,matches,trace);
}

unsigned stereo_sparse(const uint8_t *l,const uint8_t *r,const struct StereoCalibration *c,struct StereoMatch *m)
{ return stereo_sparse_timed(l,r,c,m,NULL); }
