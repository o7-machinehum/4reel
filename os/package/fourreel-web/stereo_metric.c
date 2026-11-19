#include "stereo_metric.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static double dot(const double *a, const double *b)
{ return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }

static int ds_valid(const struct DsCamera *c)
{
    for (unsigned i=0; i<6; ++i) if (!isfinite(c->p[i])) return 0;
    return c->p[0] > -1 && c->p[0] < 3 && c->p[1] > 0 && c->p[1] < 1 &&
        c->p[2] > 0 && c->p[3] > 0 && c->p[4] >= 0 && c->p[4] < STEREO_W &&
        c->p[5] >= 0 && c->p[5] < STEREO_H;
}

int ds_ray(const struct DsCamera *c, double x, double y, double r[3])
{
    if (!ds_valid(c) || !isfinite(x) || !isfinite(y) ||
        x<0 || x>=STEREO_W || y<0 || y>=STEREO_H) return -1;
    double xi=c->p[0], a=c->p[1];
    double mx=(x-c->p[4])/c->p[2], my=(y-c->p[5])/c->p[3];
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

int ds_project(const struct DsCamera *c, const double p[3], double xy[2])
{
    if (!ds_valid(c)) return -1;
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
    if (!ds_valid(&c->eye[0]) || !ds_valid(&c->eye[1])) return 0;
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
    ok=ok && !strcmp(version,"4REEL_DS1") && !strcmp(c.unit,unit) &&
        !strcmp(orientation,"left=1,right=0,hflip=both,right_sensor_rotation=180") &&
        strlen(c.source_sha256)==64;
    for(unsigned i=0;i<strlen(c.source_sha256);++i)
        if(!strchr("0123456789abcdef",c.source_sha256[i])) ok=0;
    for (int e=0;e<2;++e) for(int i=0;i<6;++i)
        if(fscanf(f,"%lf",&c.eye[e].p[i])!=1) ok=0;
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
    if (!stereo_calibration_valid(c) || ds_ray(&c->eye[0],l[0],l[1],a) ||
        ds_ray(&c->eye[1],r[0],r[1],b)) return -1;
    for(int i=0;i<3;++i) u[i]=dot(c->R+3*i,a);
    double uv=dot(u,b), den=1-uv*uv;
    /* sin(parallax)^2: ~0.57 degree minimum, limits distant noisy anchors. */
    if(den<1e-4) return -1;
    double ut=dot(u,c->t), bt=dot(b,c->t);
    double dl=(uv*bt-ut)/den, dr=(bt-uv*ut)/den;
    if(dl<=0 || dr<=0 || a[2]<=0 || b[2]<=0) return -1;
    double pl[3], pr[3], projected[2];
    for(int i=0;i<3;++i) { pl[i]=a[i]*dl; pr[i]=u[i]*dl+c->t[i]; }
    if(ds_project(&c->eye[1],pr,projected) || hypot(projected[0]-r[0],projected[1]-r[1])>1.0)
        return -1;
    double err=0;
    for(int i=0;i<3;++i) err+=(pr[i]-dr*b[i])*(pr[i]-dr*b[i]);
    if(sqrt(err)>0.01 || pl[2]<0.15 || pl[2]>20 || pr[2]<0.15 || pr[2]>20) return -1;
    z[0]=pl[2]; z[1]=pr[2];
    return 0;
}

static int anchor_valid(const struct StereoAnchor *a)
{
    return isfinite(a->prediction) && isfinite(a->inverse_z) &&
        a->inverse_z>0.05 && a->inverse_z<1/0.15 && isfinite(a->weight) && a->weight>0 &&
        isfinite(a->x) && isfinite(a->y) && a->x>=256 && a->x<=1024 && a->y>=160 && a->y<=640;
}

static int point_order(const void *va,const void *vb)
{
    const double *a=va,*b=vb;
    return a[0]!=b[0] ? (a[0]>b[0] ? 1:-1) : (a[1]>b[1])-(a[1]<b[1]);
}
static double cross(const double *a,const double *b,const double *c)
{ return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]); }

static void make_hull(const struct StereoAnchor *a,unsigned n,struct StereoFit *f)
{
    double p[STEREO_MAX_ANCHORS][2],h[2*STEREO_MAX_ANCHORS][2];
    unsigned m=0,k=0;
    for(unsigned i=0;i<n;++i) if(f->accepted[i]) {
        p[m][0]=a[i].x; p[m++][1]=a[i].y;
    }
    qsort(p,m,sizeof(p[0]),point_order);
    for(unsigned i=0;i<m;++i) {
        while(k>=2 && cross(h[k-2],h[k-1],p[i])<=0) --k;
        memcpy(h[k++],p[i],sizeof(p[i]));
    }
    unsigned t=k+1;
    for(int i=(int)m-2;i>=0;--i) {
        while(k>=t && cross(h[k-2],h[k-1],p[i])<=0) --k;
        memcpy(h[k++],p[i],sizeof(p[i]));
    }
    f->hull_size=k>1 ? k-1:0;
    if(f->hull_size>STEREO_MAX_ANCHORS) f->hull_size=0;
    memcpy(f->hull,h,f->hull_size*sizeof(h[0]));
}

int stereo_fit(const struct StereoAnchor *a, unsigned n, struct StereoFit *f)
{
    memset(f,0,sizeof(*f));
    if(n<12 || n>STEREO_MAX_ANCHORS) return -1;
    unsigned best=0;
    double best_s=0,best_b=0;
    /* Deterministic, bounded RANSAC: all two-point hypotheses, no RNG state. */
    for(unsigned i=0;i<n;++i) for(unsigned j=i+1;j<n;++j) {
        if(!anchor_valid(a+i) || !anchor_valid(a+j)) continue;
        double dx=a[j].prediction-a[i].prediction;
        if(fabs(dx)<1e-6) continue;
        double s=(a[j].inverse_z-a[i].inverse_z)/dx, b=a[i].inverse_z-s*a[i].prediction;
        if(!isfinite(s) || !isfinite(b) || s<=0) continue;
        unsigned count=0;
        for(unsigned k=0;k<n;++k) if(anchor_valid(a+k) &&
            fabs(s*a[k].prediction+b-a[k].inverse_z)<0.03+0.03*a[k].inverse_z) ++count;
        if(count>best) { best=count; best_s=s; best_b=b; }
    }
    if(best<12 || best*5<n*3) return -1;
    double s=best_s,b=best_b;
    for(unsigned it=0;it<3;++it) {
        double sw=0,sx=0,sy=0,sxx=0,sxy=0;
        for(unsigned k=0;k<n;++k) {
            double residual=fabs(s*a[k].prediction+b-a[k].inverse_z);
            int keep=anchor_valid(a+k) && residual<0.03+0.03*a[k].inverse_z;
            f->accepted[k]=keep;
            if(!keep) continue;
            double w=a[k].weight;
            sw+=w; sx+=w*a[k].prediction; sy+=w*a[k].inverse_z;
            sxx+=w*a[k].prediction*a[k].prediction; sxy+=w*a[k].prediction*a[k].inverse_z;
        }
        double den=sw*sxx-sx*sx;
        if(sw<=0 || den<1e-8*sw*sw) return -1;
        s=(sw*sxy-sx*sy)/den; b=(sy-s*sx)/sw;
        if(!isfinite(s) || !isfinite(b) || s<=0) return -1;
    }
    unsigned quadrants=0;
    double error=0,min_y=1e30,max_y=-1e30;
    f->min_prediction=1e30; f->max_prediction=-1e30;
    for(unsigned k=0;k<n;++k) {
        double residual=s*a[k].prediction+b-a[k].inverse_z;
        f->accepted[k]=anchor_valid(a+k) && fabs(residual)<0.03+0.03*a[k].inverse_z;
        if(!f->accepted[k]) continue;
        ++f->inliers; error+=residual*residual;
        quadrants|=1u<<((a[k].x>=640)+2*(a[k].y>=400));
        min_y=fmin(min_y,a[k].inverse_z); max_y=fmax(max_y,a[k].inverse_z);
        f->min_prediction=fmin(f->min_prediction,a[k].prediction);
        f->max_prediction=fmax(f->max_prediction,a[k].prediction);
    }
    unsigned covered=0;
    for(unsigned q=0;q<4;++q) covered+=(quadrants>>q)&1;
    if(f->inliers<12 || f->inliers*5<n*3 || covered<3 || max_y-min_y<0.15) {
        memset(f,0,sizeof(*f)); return -1;
    }
    f->scale=s; f->shift=b; f->rms=sqrt(error/f->inliers);
    f->confidence=(double)f->inliers/n*exp(-f->rms/0.05);
    make_hull(a,n,f);
    if(f->hull_size<3) { memset(f,0,sizeof(*f)); return -1; }
    return 0;
}

int stereo_support(const struct StereoAnchor *a,unsigned n,const struct StereoFit *f,double x,double y)
{
    if(!f->inliers || n>STEREO_MAX_ANCHORS || !isfinite(x) || !isfinite(y)) return 0;
    if(x<256 || x>1024 || y<160 || y>640) return 2;
    (void)a;
    double p[2]={x,y};
    if(f->hull_size<3) return 0;
    for(unsigned i=0;i<f->hull_size;++i)
        if(cross(f->hull[i],f->hull[(i+1)%f->hull_size],p)<-1e-6) return 2;
    return 1;
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
