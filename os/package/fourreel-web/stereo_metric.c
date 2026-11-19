#include "stereo_metric.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static double dot(const double *a, const double *b)
{ return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }

static int model_valid(const struct CameraModel *c)
{
    for (unsigned i=0; i<6; ++i) if (!isfinite(c->p[i])) return 0;
    if(c->pinhole) {
        for(unsigned i=0;i<4;i++) if(!isfinite(c->distortion[i])) return 0;
    } else if(!(c->p[0]>-1 && c->p[0]<3 && c->p[1]>0 && c->p[1]<1)) return 0;
    return
        c->p[2] > 0 && c->p[3] > 0 && c->p[4] >= 0 && c->p[4] < STEREO_W &&
        c->p[5] >= 0 && c->p[5] < STEREO_H;
}

static void radtan(const double d[4],double x,double y,double *u,double *v)
{
    double r2=x*x+y*y, radial=1+r2*(d[0]+r2*d[1]);
    *u=x*radial+2*d[2]*x*y+d[3]*(r2+2*x*x);
    *v=y*radial+d[2]*(r2+2*y*y)+2*d[3]*x*y;
}

int camera_ray(const struct CameraModel *c, double x, double y, double r[3])
{
    if (!model_valid(c) || !isfinite(x) || !isfinite(y) ||
        x<0 || x>=STEREO_W || y<0 || y>=STEREO_H) return -1;
    double xi=c->p[0], a=c->p[1];
    double mx=(x-c->p[4])/c->p[2], my=(y-c->p[5])/c->p[3];
    if(c->pinhole) {
        double u=mx,v=my;
        for(int i=0;i<20;i++) {
            double dx,dy;
            radtan(c->distortion,u,v,&dx,&dy);
            dx=mx-dx; dy=my-dy;
            if(!isfinite(dx) || !isfinite(dy)) return -1;
            if(fabs(dx)+fabs(dy)<1e-10) {
                double norm=sqrt(1+u*u+v*v);
                r[0]=u/norm; r[1]=v/norm; r[2]=1/norm;
                return 0;
            }
            u+=dx; v+=dy;
        }
        return -1;
    }
    double rr=mx*mx+my*my, disc=1-(2*a-1)*rr;
    if (disc <= 0) return -1;
    double mz=(1-a*a*rr)/(a*sqrt(disc)+1-a);
    double d=mz*mz+(1-xi*xi)*rr;
    if (d<=0 || mz*mz+rr<=0) return -1;
    double k=(mz*xi+sqrt(d))/(mz*mz+rr);
    r[0]=k*mx; r[1]=k*my; r[2]=k*mz-xi;
    double norm=sqrt(dot(r,r));
    if (!isfinite(norm) || norm<1e-12) return -1;
    for (int i=0; i<3; ++i) r[i]/=norm;
    return 0;
}

int camera_project(const struct CameraModel *c, const double p[3], double xy[2])
{
    if (!model_valid(c)) return -1;
    if(c->pinhole) {
        if(!(p[2]>1e-12)) return -1;
        double x,y;
        radtan(c->distortion,p[0]/p[2],p[1]/p[2],&x,&y);
        xy[0]=c->p[2]*x+c->p[4]; xy[1]=c->p[3]*y+c->p[5];
        return isfinite(xy[0]) && isfinite(xy[1]) ? 0:-1;
    }
    double xi=c->p[0], a=c->p[1], d1=sqrt(dot(p,p));
    double w1=a<=0.5 ? a/(1-a) : (1-a)/a;
    double q=2*w1*xi+xi*xi+1;
    if (!isfinite(d1) || d1<1e-12 || q<=0) return -1;
    double w2=(w1+xi)/sqrt(q);
    if (p[2] <= -w2*d1) return -1;
    double z=xi*d1+p[2], d2=sqrt(p[0]*p[0]+p[1]*p[1]+z*z);
    double denom=a*d2+(1-a)*z;
    if (denom<=1e-12) return -1;
    xy[0]=c->p[2]*p[0]/denom+c->p[4];
    xy[1]=c->p[3]*p[1]/denom+c->p[5];
    return isfinite(xy[0]) && isfinite(xy[1]) ? 0 : -1;
}

int stereo_calibration_valid(const struct StereoCalibration *c)
{
    if (!model_valid(&c->eye[0]) || !model_valid(&c->eye[1])) return 0;
    for (unsigned i=0; i<9; ++i) if (!isfinite(c->R[i])) return 0;
    for (unsigned i=0; i<3; ++i) if (!isfinite(c->t[i])) return 0;
    double baseline=sqrt(dot(c->t,c->t));
    /* Gross unit/rig mismatch check, not a constraint to nominal 79.23 mm. */
    if (baseline < 0.04 || baseline > 0.12) return 0;
    for (int i=0; i<3; ++i) for (int j=0; j<3; ++j)
        if (fabs(dot(c->R+3*i,c->R+3*j)-(i==j))>1e-5) return 0;
    double det=c->R[0]*(c->R[4]*c->R[8]-c->R[5]*c->R[7])
              -c->R[1]*(c->R[3]*c->R[8]-c->R[5]*c->R[6])
              +c->R[2]*(c->R[3]*c->R[7]-c->R[4]*c->R[6]);
    return fabs(det-1)<1e-5;
}

int stereo_calibration_load(const char *path, const char *unit,
                            struct StereoCalibration *out)
{
    FILE *f=fopen(path,"r");
    if (!f) return -1;
    struct StereoCalibration c={0};
    char version[32], orientation[64], extra;
    int ok=fscanf(f,"%31s %63s %64s %63s",version,c.unit,c.source_sha256,orientation)==4;
    int pinhole=ok && !strcmp(version,"4REEL_PINHOLE1");
    ok=ok && (pinhole || !strcmp(version,"4REEL_DS1")) && !strcmp(c.unit,unit) &&
        !strcmp(orientation,"left=1,right=0,hflip=both,right_sensor_rotation=180") &&
        strlen(c.source_sha256)==64;
    for(unsigned i=0;i<strlen(c.source_sha256);++i)
        if(!strchr("0123456789abcdef",c.source_sha256[i])) ok=0;
    for (int e=0;e<2;++e) {
        c.eye[e].pinhole=pinhole;
        for(int i=pinhole?2:0;i<6;++i) if(fscanf(f,"%lf",&c.eye[e].p[i])!=1) ok=0;
        if(pinhole) for(int i=0;i<4;++i) if(fscanf(f,"%lf",&c.eye[e].distortion[i])!=1) ok=0;
    }
    for(int i=0;i<9;++i) if(fscanf(f,"%lf",&c.R[i])!=1) ok=0;
    for(int i=0;i<3;++i) if(fscanf(f,"%lf",&c.t[i])!=1) ok=0;
    if(fscanf(f," %c",&extra)==1) ok=0;
    fclose(f);
    if (!ok || !stereo_calibration_valid(&c)) return -1;
    *out=c;
    return 0;
}

int stereo_triangulate(const struct StereoCalibration *c,
                       const double l[2], const double r[2], double z[2])
{
    double a[3], b[3], u[3];
    z[0]=z[1]=0;
    if (!stereo_calibration_valid(c) || camera_ray(&c->eye[0],l[0],l[1],a) ||
        camera_ray(&c->eye[1],r[0],r[1],b)) return -1;
    for(int i=0;i<3;++i) u[i]=dot(c->R+3*i,a);
    double uv=dot(u,b), den=1-uv*uv;
    /* sin(parallax)^2: ~0.57 degree minimum, limits distant noisy anchors. */
    if(den<1e-4) return -1;
    double ut=dot(u,c->t), bt=dot(b,c->t);
    double dl=(uv*bt-ut)/den, dr=(bt-uv*ut)/den;
    if(dl<=0 || dr<=0 || a[2]<=0 || b[2]<=0) return -1;
    /* Use the midpoint of the closest approach of the two measured rays.
     * Check its reprojection in both native images. */
    double pr[3],pl[3],err=0,projected[2];
    for(int i=0;i<3;i++) {
        double left=u[i]*dl+c->t[i],right=b[i]*dr;
        pr[i]=(left+right)*0.5;
        err+=(left-right)*(left-right);
    }
    if(err>0.01*0.01) return -1;
    for(int i=0;i<3;i++) {
        pl[i]=0;
        for(int j=0;j<3;j++) pl[i]+=c->R[3*j+i]*(pr[j]-c->t[j]);
    }
    if(pl[2]<0.15 || pl[2]>20 || pr[2]<0.15 || pr[2]>20) return -1;
    if(camera_project(&c->eye[0],pl,projected) || hypot(projected[0]-l[0],projected[1]-l[1])>1.5 ||
       camera_project(&c->eye[1],pr,projected) || hypot(projected[0]-r[0],projected[1]-r[1])>1.5) return -1;
    z[0]=pl[2]; z[1]=pr[2];
    return 0;
}

static int scalar_order(const void *a, const void *b)
{
    double x=*(const double *)a,y=*(const double *)b;
    return (x>y)-(x<y);
}

unsigned stereo_scale(const double votes[][2], unsigned n, double *factor)
{
    if(n<3 || n>STEREO_MAX_ANCHORS) return 0;
    double centres[STEREO_MAX_ANCHORS];
    unsigned usable=0;
    for(unsigned i=0;i<n;i++) {
        double l=votes[i][0], r=votes[i][1];
        if(isfinite(l) && isfinite(r) && l>0 && r>0)
            centres[usable++]=sqrt(l*r);
    }
    if(usable<3) return 0;
    qsort(centres,usable,sizeof(centres[0]),scalar_order);
    double median=sqrt(centres[(usable-1)/2]*centres[usable/2]);
    unsigned inliers=0;
    for(unsigned i=0;i<n;i++) {
        double l=votes[i][0], r=votes[i][1];
        if(l>=median/1.25 && l<=median*1.25 && r>=median/1.25 && r<=median*1.25)
            ++inliers;
    }
    if(inliers<3 || inliers*2<=usable) return 0;
    *factor=median;
    return inliers;
}

int stereo_scale_map(const float *prediction, float *metres, unsigned pixels, double factor)
{
    for(unsigned i=0;i<pixels;i++) {
        if(!(prediction[i]>0)) return -1;
        metres[i]=(float)(factor/prediction[i]);
        if(!isfinite(metres[i]) || metres[i]<=0) return -1;
    }
    return 0;
}

int stereo_tensor_cpu(const uint8_t *s,size_t bytes,unsigned stride,uint8_t *d)
{
    if(!s || !d || stride<STEREO_W || bytes<(size_t)stride*STEREO_H) return -1;
    memset(d,0,TENSOR_W*TENSOR_H);
    for(unsigned y=0;y<200;++y) for(unsigned x=0;x<320;++x) {
        unsigned sum=0;
        for(unsigned j=0;j<4;++j) for(unsigned i=0;i<4;++i)
            sum+=s[(4*y+j)*stride+4*x+i];
        d[(y+12)*320+x]=(sum+8)/16;
    }
    return 0;
}
