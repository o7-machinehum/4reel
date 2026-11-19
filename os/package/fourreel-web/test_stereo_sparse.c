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
        if(camera_ray(&c.eye[1],x,y,ray) || ray[2]<=0) continue;
        for(int i=0;i<3;++i) p[i]=ray[i]*2/ray[2];
        p[0]+=.07923;
        if(!camera_project(&c.eye[0],p,xy)) r[y*1280+x]=(uint8_t)round(sample(l,xy[0],xy[1]));
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
    assert(n>=12 && n<=20 && good*10>=n*9);
    struct StereoMatch from_dma[STEREO_MAX_ANCHORS];
    const int stride=1344;
    uint8_t *backing=calloc(2*stride*800,1);
    assert(backing);
    struct StereoImage eyes[2];
    for(int e=0;e<2;e++) {
        uint8_t *base=backing+e*stride*800;
        eyes[e]=(struct StereoImage){base+1279,stride,-1};
        for(int y=0;y<800;y++) for(int x=0;x<1280;x++)
            base[y*stride+1279-x]=(e?r:l)[y*1280+x];
    }
    assert(stereo_sparse_views(eyes,&c,from_dma,NULL)==n);
    for(unsigned i=0;i<n;i++) {
        for(int e=0;e<2;e++) {
            assert(from_dma[i].z[e]==matches[i].z[e]);
            for(int j=0;j<2;j++) assert(from_dma[i].xy[e][j]==matches[i].xy[e][j]);
        }
    }
    /* Same textured image with a known 16-native-pixel shift, using padded,
     * mirrored DMA views. No camera model or metric anchors are supplied. */
    for(int y=0;y<800;y++) for(int x=0;x<1264;x++)
        backing[stride*800+y*stride+1279-x]=l[y*1280+x+16];
    struct SparseStats stats;
    trace=(struct Trace){.enabled=1};
    n=stereo_match_pair(eyes,NULL,matches,&stats,&trace,NULL);
    good=0;
    for(unsigned i=0;i<n;i++) good+=matches[i].xy[0][0]-matches[i].xy[1][0]==16 &&
                                     matches[i].xy[0][1]==matches[i].xy[1][1];
    assert(n>=12 && good*10>=9*n && !stats.calibrated && !stats.anchors && stats.mutual==n);
    assert(stats.candidates<=20 && n<=20);
    assert(trace.count==4 && trace.event[0].stage==TS_THUMB && trace.event[3].stage==TS_MUTUAL);
    printf("image-only: %u mutual matches, %u at true displacement\n",n,good);
    /* No left texture: all 20 right seeds fail. Each cell must move to a
     * different textured subregion for the next pair and eventually wrap. */
    for(int i=0;i<stride*800;++i) backing[i]=120;
    struct StereoCornerHistory history={0};
    for(unsigned frame=0;frame<5;frame++) {
        assert(!stereo_match_pair(eyes,&c,matches,&stats,NULL,&history));
        assert(stats.candidates==STEREO_MAX_ANCHORS);
        for(unsigned j=0;j<STEREO_MAX_ANCHORS;j++)
            assert((unsigned)__builtin_popcount(history.rejected[j])==frame%4+1);
    }
    free(backing);
    for(int i=0;i<1280*800;++i) l[i]=r[i]=120;
    assert(stereo_sparse(l,r,&c,matches)==0);
    assert(stereo_sparse(l,r,NULL,matches)==0);
    c.t[0]=0;
    assert(stereo_sparse(l,r,&c,matches)==0);
    free(l); free(r);
    puts("bounded sparse matching and textureless/invalid rejection: PASS");
    return 0;
}
