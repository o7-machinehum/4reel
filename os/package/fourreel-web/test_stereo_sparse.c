#define _POSIX_C_SOURCE 200809L
#include "stereo_sparse.h"
#include "stereo_trace.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static double sample(const uint8_t *p,double x,double y)
{
    int i=(int)x,j=(int)y;
    if(i<0 || i>=1279 || j<0 || j>=799) return 0;
    double a=x-i,b=y-j;
    return (1-b)*((1-a)*p[j*1280+i]+a*p[j*1280+i+1])+b*((1-a)*p[(j+1)*1280+i]+a*p[(j+1)*1280+i+1]);
}
int main(void)
{
    struct StereoCalibration c={.eye={{{0,0.5,400,400,640,400}},{{0,0.5,400,400,640,400}}},
        .R={1,0,0,0,1,0,0,0,1},.t={-.07923,0,0}};
    uint8_t *l=malloc(1280*800),*r=calloc(1280*800,1);
    assert(l && r);
    for(int y=0;y<800;++y) for(int x=0;x<1280;++x)
        l[y*1280+x]=(uint8_t)(127+30*sin(x*.14+y*.2)+30*sin(x*.043-y*.132)+25*sin(x*.21-y*.15)+20*cos(x*.071+y*.069));
    for(int y=0;y<800;++y) for(int x=0;x<1280;++x) {
        double ray[3],p[3],xy[2];
        if(ds_ray(&c.eye[1],x,y,ray) || ray[2]<=0) continue;
        for(int i=0;i<3;++i) p[i]=ray[i]*2/ray[2];
        p[0]+=.07923;
        if(!ds_project(&c.eye[0],p,xy)) r[y*1280+x]=(uint8_t)round(sample(l,xy[0],xy[1]));
    }
    struct StereoMatch matches[STEREO_MAX_ANCHORS];
    struct Trace trace={.enabled=1};
    unsigned n=stereo_sparse_timed(l,r,&c,matches,&trace),good=0;
    assert(trace.count==5 && !trace.overflow);
    for(unsigned i=0;i<5;++i) {
        assert(trace.event[i].stage==TS_PYRAMID+i);
        assert(trace.event[i].resource==TR_MAIN);
        assert(trace.event[i].end>=trace.event[i].begin);
    }
    for(unsigned i=0;i<n;++i) good+=fabs(matches[i].z[0]-2)<.12;
    printf("synthetic DS plane: %u anchors, %u within 6%% of 2 m\n",n,good);
    assert(n>=12 && good*10>=n*9);
    for(int i=0;i<1280*800;++i) l[i]=r[i]=120;
    assert(stereo_sparse(l,r,&c,matches)==0);
    c.t[0]=0;
    assert(stereo_sparse(l,r,&c,matches)==0);
    free(l); free(r);
    puts("bounded sparse matching and textureless/invalid rejection: PASS");
    return 0;
}
