#define _POSIX_C_SOURCE 200809L
#include "stereo_sparse.h"
#include "stereo_trace.h"
#include <math.h>
#include <stdlib.h>

static double sample(const uint8_t *s,int w,double x,double y)
{
    int ix=(int)x,iy=(int)y; double ax=x-ix,ay=y-iy;
    return (1-ay)*((1-ax)*s[iy*w+ix]+ax*s[iy*w+ix+1])+
           ay*((1-ax)*s[(iy+1)*w+ix]+ax*s[(iy+1)*w+ix+1]);
}
static int inside(double x,double y,int w,int h)
{ return isfinite(x) && isfinite(y) && x>=6 && y>=6 && x<w-7 && y<h-7; }

static double cost(const uint8_t *l,const uint8_t *r,int w,int h,
                   double lx,double ly,double rx,double ry)
{
    if(!inside(lx,ly,w,h) || !inside(rx,ry,w,h)) return 1e9;
    double ml=0,mr=0,d=0;
    for(int y=-3;y<=3;++y) for(int x=-3;x<=3;++x) {
        ml+=sample(l,w,lx+x,ly+y); mr+=sample(r,w,rx+x,ry+y);
    }
    ml/=49; mr/=49;
    for(int y=-3;y<=3;++y) for(int x=-3;x<=3;++x)
        d+=fabs(sample(l,w,lx+x,ly+y)-ml-sample(r,w,rx+x,ry+y)+mr);
    return d/49;
}

static int lk(const uint8_t *l,const uint8_t *r,int w,int h,
              double lx,double ly,double *rx,double *ry)
{
    for(unsigned it=0;it<12;++it) {
        if(!inside(lx,ly,w,h) || !inside(*rx,*ry,w,h)) return -1;
        double ml=0,mr=0;
        for(int y=-3;y<=3;++y) for(int x=-3;x<=3;++x) {
            ml+=sample(l,w,lx+x,ly+y); mr+=sample(r,w,*rx+x,*ry+y);
        }
        double bias=(ml-mr)/49,xx=0,xy=0,yy=0,bx=0,by=0;
        for(int y=-3;y<=3;++y) for(int x=-3;x<=3;++x) {
            double px=*rx+x,py=*ry+y;
            double gx=(sample(r,w,px+1,py)-sample(r,w,px-1,py))/2;
            double gy=(sample(r,w,px,py+1)-sample(r,w,px,py-1))/2;
            double d=sample(l,w,lx+x,ly+y)-sample(r,w,px,py)-bias;
            xx+=gx*gx; xy+=gx*gy; yy+=gy*gy; bx+=gx*d; by+=gy*d;
        }
        double det=xx*yy-xy*xy;
        if(det<1e-3 || xx+yy<1 || det/(xx+yy)<30) return -1;
        double dx=(yy*bx-xy*by)/det,dy=(xx*by-xy*bx)/det;
        if(!isfinite(dx) || !isfinite(dy) || hypot(dx,dy)>3) return -1;
        *rx+=dx; *ry+=dy;
        if(hypot(dx,dy)<0.03) return 0;
    }
    return -1;
}

static void half(const uint8_t *s,uint8_t *d,int w,int h)
{
    for(int y=0;y<h/2;++y) for(int x=0;x<w/2;++x) {
        int i=2*y*w+2*x;
        d[y*(w/2)+x]=(s[i]+s[i+1]+s[i+w]+s[i+w+1]+2)/4;
    }
}

unsigned stereo_sparse_timed(const uint8_t *left,const uint8_t *right,
                       const struct StereoCalibration *cal,struct StereoMatch *matches,struct Trace *trace)
{
    if(!stereo_calibration_valid(cal)) return 0;
    uint8_t *storage=malloc(2*(640*400+320*200));
    if(!storage) return 0;
    const uint8_t *p[2][3]={{left,storage,storage+640*400},
                           {right,storage+640*400+320*200,storage+2*640*400+320*200}};
    struct TraceMark timing=trace_begin(trace);
    for(int e=0;e<2;++e) { half(p[e][0],(uint8_t *)p[e][1],1280,800); half(p[e][1],(uint8_t *)p[e][2],640,400); }
    trace_end(trace,timing,TS_PYRAMID,TR_MAIN);
    timing=trace_begin(trace);
    struct StereoMatch candidates[96];
    unsigned nc=0,nm=0,nr=0,n=0;
    /* 12x8 cells, one well-conditioned corner each, <=96 native anchors. */
    for(int gy=0;gy<8;++gy) for(int gx=0;gx<12;++gx) {
        double best_corner=40; int cx=-1,cy=-1;
        for(int y=42+gy*14;y<54+gy*14;++y) for(int x=66+gx*15;x<79+gx*15;++x) {
            double xx=0,xy=0,yy=0;
            for(int j=-2;j<=2;++j) for(int i=-2;i<=2;++i) {
                int pos=(y+j)*320+x+i;
                double dx=((int)p[0][2][pos+1]-p[0][2][pos-1])/2.0;
                double dy=((int)p[0][2][pos+320]-p[0][2][pos-320])/2.0;
                xx+=dx*dx; xy+=dx*dy; yy+=dy*dy;
            }
            double score=(xx+yy-sqrt((xx-yy)*(xx-yy)+4*xy*xy))/2;
            if(score>best_corner) { best_corner=score; cx=x; cy=y; }
        }
        if(cx<0) continue;
        candidates[nc++]=(struct StereoMatch){.xy={{4*cx+1.5,4*cy+1.5},{0,0}}};
    }
    trace_end(trace,timing,TS_CORNERS,TR_MAIN);
    timing=trace_begin(trace);
    for(unsigned c=0;c<nc;++c) {
        double lx=candidates[c].xy[0][0],ly=candidates[c].xy[0][1],ray[3];
        double cx=(lx-1.5)/4,cy=(ly-1.5)/4;
        if(ds_ray(&cal->eye[0],lx,ly,ray) || ray[2]<=0) continue;
        double scores[128],positions[128][2],best=1e9; int winner=-1;
        for(int k=0;k<128;++k) {
            double inv=0.05+k*(1/0.15-0.05)/127,point[3],rpoint[3];
            for(int j=0;j<3;++j) point[j]=ray[j]/(ray[2]*inv);
            for(int j=0;j<3;++j) rpoint[j]=cal->R[j*3]*point[0]+cal->R[j*3+1]*point[1]+cal->R[j*3+2]*point[2]+cal->t[j];
            scores[k]=1e9; positions[k][0]=positions[k][1]=-1e6;
            if(rpoint[2]<=0 || ds_project(&cal->eye[1],rpoint,positions[k])) continue;
            double x=(positions[k][0]+.5)/4-.5,y=(positions[k][1]+.5)/4-.5;
            if(positions[k][0]<256 || positions[k][0]>1024 || positions[k][1]<160 || positions[k][1]>640) continue;
            scores[k]=cost(p[0][2],p[1][2],320,200,cx,cy,x,y);
            if(scores[k]<best) { best=scores[k]; winner=k; }
        }
        if(winner<0 || best>18) continue;
        double second=1e9;
        for(int k=0;k<128;++k) if(hypot(positions[k][0]-positions[winner][0],positions[k][1]-positions[winner][1])>12)
            second=fmin(second,scores[k]);
        if(second<best*1.25+1 || second==1e9) continue;
        double rx=positions[winner][0],ry=positions[winner][1]; int bad=0;
        for(int level=2;level>=0;--level) {
            int scale=1<<level;
            double x=(rx+.5)/scale-.5,y=(ry+.5)/scale-.5;
            if(lk(p[0][level],p[1][level],1280/scale,800/scale,
                  (lx+.5)/scale-.5,(ly+.5)/scale-.5,&x,&y)) { bad=1; break; }
            rx=(x+.5)*scale-.5; ry=(y+.5)*scale-.5;
        }
        if(bad || rx<256 || rx>1024 || ry<160 || ry>640) continue;
        candidates[nm++]=(struct StereoMatch){.xy={{lx,ly},{rx,ry}},.weight=1/(1+best)};
    }
    trace_end(trace,timing,TS_MATCH,TR_MAIN);
    timing=trace_begin(trace);
    for(unsigned c=0;c<nm;++c) {
        double lx=candidates[c].xy[0][0],ly=candidates[c].xy[0][1];
        double rx=candidates[c].xy[1][0],ry=candidates[c].xy[1][1];
        double backx=lx,backy=ly;
        if(lk(right,left,1280,800,rx,ry,&backx,&backy) || hypot(backx-lx,backy-ly)>.6 ||
           cost(left,right,1280,800,lx,ly,rx,ry)>20) continue;
        candidates[nr++]=candidates[c];
    }
    trace_end(trace,timing,TS_REVERSE,TR_MAIN);
    timing=trace_begin(trace);
    for(unsigned c=0;c<nr;++c) {
        struct StereoMatch m=candidates[c];
        if(stereo_triangulate(cal,m.xy[0],m.xy[1],m.z)) continue;
        matches[n++]=m;
    }
    trace_end(trace,timing,TS_TRIANGULATE,TR_MAIN);
    free(storage);
    return n;
}

unsigned stereo_sparse(const uint8_t *l,const uint8_t *r,const struct StereoCalibration *c,struct StereoMatch *m)
{ return stereo_sparse_timed(l,r,c,m,NULL); }
