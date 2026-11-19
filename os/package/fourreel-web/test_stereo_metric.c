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
        assert(!ds_project(&c.eye[0],p,l));
        assert(!ds_project(&c.eye[1],pr,r));
        assert(!ds_ray(&c.eye[0],l[0],l[1],ray));
        assert(fabs(ray[0]/ray[2]-p[0]/p[2])<1e-10);
        assert(!stereo_triangulate(&c,l,r,z));
        assert(fabs(z[0]-2)<1e-9 && fabs(z[1]-2)<1e-9);
        r[1]+=4;
        assert(stereo_triangulate(&c,l,r,z));
    }
    double xy[]={640,400},z[2],ray[3];
    assert(stereo_triangulate(&c,xy,xy,z)); /* parallel rays */
    assert(ds_ray(&c.eye[0],NAN,400,ray));
    c.t[0]=79.23; assert(!stereo_calibration_valid(&c)); c.t[0]=-0.07923;
    c.R[0]=-1; assert(!stereo_calibration_valid(&c)); c.R[0]=1;
    c.eye[0].p[1]=0.9; c.eye[0].p[2]=40;
    assert(ds_ray(&c.eye[0],0,0,ray)); /* outside unprojection domain */

    struct StereoAnchor a[32]; struct StereoFit f;
    for(unsigned i=0;i<32;++i) a[i]=(struct StereoAnchor){
        .x=300+(i%8)*90,.y=200+(i/8)*130,
        .prediction=0.2+0.03*i,.inverse_z=1.3*(0.2+0.03*i)+0.1,.weight=1};
    a[2].inverse_z=5; a[6].prediction=NAN; a[25].inverse_z=4;
    assert(!stereo_fit(a,32,&f));
    assert(f.inliers==29 && fabs(f.scale-1.3)<1e-10 && fabs(f.shift-0.1)<1e-10);
    assert(stereo_support(a,32,&f,600,390)==1);
    assert(stereo_support(a,32,&f,50,390)==2);
    for(unsigned i=0;i<32;++i) a[i].prediction=1;
    assert(stereo_fit(a,32,&f)); /* no prediction spread */
    assert(stereo_fit(a,11,&f));
    for(unsigned i=0;i<32;++i) { a[i].prediction=i*.02; a[i].inverse_z=1; }
    assert(stereo_fit(a,32,&f)); /* no metric spread */

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
