#include "stereo_metric.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void)
{
    /* Analytical fixtures only. Never installed as device calibration. */
    struct StereoCalibration c={
        .eye={{{0,0.5,400,400,640,400}},{{0,0.5,400,400,640,400}}},
        .R={1,0,0,0,1,0,0,0,1}, .t={-0.07923,0,0}};
    assert(stereo_calibration_valid(&c));
    for(int ix=-3;ix<=3;++ix) for(int iy=-2;iy<=2;++iy) {
        double p[]={ix*0.2,iy*0.2,2},pr[]={p[0]+c.t[0],p[1],p[2]};
        double l[2],r[2],ray[3],z[2];
        assert(!camera_project(&c.eye[0],p,l));
        assert(!camera_project(&c.eye[1],pr,r));
        assert(!camera_ray(&c.eye[0],l[0],l[1],ray));
        assert(fabs(ray[0]/ray[2]-p[0]/p[2])<1e-10);
        assert(!stereo_triangulate(&c,l,r,z));
        assert(fabs(z[0]-2)<1e-9 && fabs(z[1]-2)<1e-9);
        r[1]+=4;
        assert(stereo_triangulate(&c,l,r,z));
    }
    double xy[]={640,400},z[2],ray[3];
    assert(stereo_triangulate(&c,xy,xy,z)); /* parallel rays */
    assert(camera_ray(&c.eye[0],NAN,400,ray));
    c.t[0]=79.23; assert(!stereo_calibration_valid(&c)); c.t[0]=-0.07923;
    c.R[0]=-1; assert(!stereo_calibration_valid(&c)); c.R[0]=1;
    c.eye[0].p[1]=0.9; c.eye[0].p[2]=40;
    assert(camera_ray(&c.eye[0],0,0,ray)); /* outside unprojection domain */

    struct CameraModel pin={.p={0,0,990,980,630,390},.distortion={.16,-.24,.002,-.001},.pinhole=1};
    double point[3]={.35,-.27,1}, uv[2], back[3];
    assert(!camera_project(&pin,point,uv) && !camera_ray(&pin,uv[0],uv[1],back));
    assert(fabs(back[0]/back[2]-point[0])<1e-8 && fabs(back[1]/back[2]-point[1])<1e-8);
    /* Rays differing by 2.5 vertical pixels can reproject within 1.5
     * pixels in each camera. Both eyes share the midpoint reprojection check. */
    struct StereoCalibration aligned={.eye={pin,pin},.R={1,0,0,0,1,0,0,0,1},.t={-.08,0,0}};
    double left[]={630,390}, right[]={590.4,392.5};
    assert(!stereo_triangulate(&aligned,left,right,z) && fabs(z[0]-2)<.03);
    right[1]=393.5;
    assert(stereo_triangulate(&aligned,left,right,z));
    double votes[STEREO_MAX_ANCHORS][2]={{.5,.5},{.5,.5},{.5,.5},{.5,.5},{8,8},{NAN,.5}};
    double factor=0;
    assert(stereo_scale(votes,6,&factor)==4 && fabs(factor-.5)<1e-10);
    assert(!stereo_scale(votes,2,&factor) && factor==.5);
    assert(!stereo_scale(votes,STEREO_MAX_ANCHORS+1,&factor) && factor==.5);
    for(unsigned i=0;i<6;i++) { votes[i][0]=1; votes[i][1]=4; }
    assert(!stereo_scale(votes,6,&factor) && factor==.5);
    float prediction[]={5,.5,.01}, metres[3];
    assert(!stereo_scale_map(prediction,metres,3,factor));
    assert(fabs(metres[0]-.1)<1e-7 && metres[1]==1 && fabs(metres[2]-50)<1e-4);
    prediction[1]=0;
    assert(stereo_scale_map(prediction,metres,3,factor));
    unsigned stride=1296;
    uint8_t *src=malloc(stride*800), *dst=malloc(320*224);
    assert(src && dst);
    memset(src,231,stride*800);
    for(unsigned y=0;y<800;++y) for(unsigned x=0;x<1280;++x) src[y*stride+x]=(x+y)%251;
    assert(!stereo_tensor_cpu(src,stride*800,stride,dst));
    for(unsigned i=0;i<320*12;++i) assert(dst[i]==0 && dst[320*212+i]==0);
    assert(dst[320*12]==3 && dst[320*13+1]==11);
    assert(stereo_tensor_cpu(src,stride*800-1,stride,dst));
    assert(stereo_tensor_cpu(src,stride*800,1279,dst));
    free(src); free(dst);
    puts("stereo geometry, robust fit, invalid cases and stride/padding: PASS");
    return 0;
}
