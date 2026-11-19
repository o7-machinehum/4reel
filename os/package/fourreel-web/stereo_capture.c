#define _POSIX_C_SOURCE 200809L
#include "stereo_capture.h"
#include "stereo_metric.h"
#include <errno.h>
#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define FSIN_CONTROL (V4L2_CID_USER_BASE+0x10f0)
struct Camera {
    int fd, sensor, running, restore;
    struct v4l2_subdev_format original;
    struct v4l2_control trigger;
    void *memory[3]; size_t size[3];
    unsigned count,stride;
};
static struct Camera cams[2]={{.fd=-1,.sensor=-1},{.fd=-1,.sensor=-1}};
static char pwm[128];
static int raw8_mode;
static uint64_t next_id;

static uint64_t now_us(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t);
    return (uint64_t)t.tv_sec*1000000+t.tv_nsec/1000;
}
static int xio(int fd,unsigned long cmd,void *p)
{ int r; do { r=ioctl(fd,cmd,p); } while(r<0 && errno==EINTR); return r; }
static int pwm_write(const char *name,const char *value)
{
    char path[180]; snprintf(path,sizeof(path),"%s/%s",pwm,name);
    int fd=open(path,O_WRONLY|O_CLOEXEC);
    if(fd<0) return -1;
    size_t n=strlen(value); ssize_t k=write(fd,value,n); int e=errno; close(fd); errno=e;
    return k==(ssize_t)n ? 0:-1;
}
static int queue(struct Camera *c,unsigned index)
{
    struct v4l2_plane p={0}; struct v4l2_buffer b={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
        .memory=V4L2_MEMORY_MMAP,.index=index,.length=1,.m.planes=&p};
    return xio(c->fd,VIDIOC_QBUF,&b);
}

void stereo_capture_close(void)
{
    if(pwm[0]) pwm_write("enable","0");
    for(unsigned e=0;e<2;++e) {
        struct Camera *c=&cams[e];
        if(c->running) { enum v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; xio(c->fd,VIDIOC_STREAMOFF,&t); }
        for(unsigned i=0;i<c->count;++i) if(c->memory[i]) munmap(c->memory[i],c->size[i]);
        if(c->fd>=0) close(c->fd);
        if(c->sensor>=0) {
            if(c->restore) { xio(c->sensor,VIDIOC_S_CTRL,&c->trigger); xio(c->sensor,VIDIOC_SUBDEV_S_FMT,&c->original); }
            close(c->sensor);
        }
        memset(c,0,sizeof(*c)); c->fd=c->sensor=-1;
    }
    pwm[0]=0;
}

static int quiet(int ms)
{
    struct pollfd f[2]={{.fd=cams[0].fd,.events=POLLIN},{.fd=cams[1].fd,.events=POLLIN}};
    int r; do { r=poll(f,2,ms); } while(r<0 && errno==EINTR);
    if(r!=0) { errno=EPROTO; return -1; } /* no frames while FSIN is disabled */
    return 0;
}

int stereo_capture_open(int raw8)
{
    stereo_capture_close(); raw8_mode=raw8;
    char videos[2][64]={{0}},sensors[2][64]={{0}};
    /* Physical LEFT is module 1 / address 0x60 / lvds1. RIGHT is module 0 /
     * address 0x10 / lvds0 with the existing sensor-side 180 degree rotation.
     */
    for(unsigned i=0;i<128;++i) {
        char path[128],name[128]={0};
        snprintf(path,sizeof(path),"/sys/class/video4linux/video%u/name",i);
        FILE *f=fopen(path,"r");
        if(f) {
            int ok=fgets(name,sizeof(name),f)!=NULL; fclose(f);
            if(ok && !strcmp(name,"stream_cif_mipi_id0\n")) {
                snprintf(path,sizeof(path),"/dev/video%u",i);
                int fd=open(path,O_RDWR|O_NONBLOCK|O_CLOEXEC); struct v4l2_capability cap={0};
                if(fd>=0) {
                    if(!xio(fd,VIDIOC_QUERYCAP,&cap)) for(int e=0;e<2;++e)
                        if(!strcmp((char *)cap.bus_info,e ? "platform:rkcif-mipi-lvds":"platform:rkcif-mipi-lvds1"))
                            snprintf(videos[e],sizeof(videos[e]),"/dev/video%u",i);
                    close(fd);
                }
            }
        }
        snprintf(path,sizeof(path),"/sys/class/video4linux/v4l-subdev%u/name",i);
        f=fopen(path,"r");
        if(f) {
            int ok=fgets(name,sizeof(name),f)!=NULL; fclose(f);
            if(ok && strstr(name,"ov9281")) for(int e=0;e<2;++e)
                if(strstr(name,e ? "m00_":"m01_") && strstr(name,e ? "4-0010":"4-0060"))
                    snprintf(sensors[e],sizeof(sensors[e]),"/dev/v4l-subdev%u",i);
        }
    }
    for(unsigned i=0;i<32;++i) {
        char path[160],link[256];
        snprintf(path,sizeof(path),"/sys/class/pwm/pwmchip%u/device/of_node",i);
        ssize_t n=readlink(path,link,sizeof(link)-1);
        if(n>0) {
            link[n]=0;
            if(strstr(link,"pwm@ff490000")) {
                snprintf(pwm,sizeof(pwm),"/sys/class/pwm/pwmchip%u",i);
                if(pwm_write("export","0") && errno!=EBUSY) goto fail;
                snprintf(pwm,sizeof(pwm),"/sys/class/pwm/pwmchip%u/pwm0",i);
                break;
            }
        }
    }
    if(!pwm[0] || pwm_write("enable","0") || pwm_write("duty_cycle","0") ||
       pwm_write("period","1000000000") || pwm_write("polarity","normal") ||
       pwm_write("duty_cycle","10000")) goto fail;
    for(unsigned e=0;e<2;++e) {
        struct Camera *c=&cams[e];
        if(!videos[e][0] || !sensors[e][0]) { errno=ENODEV; goto fail; }
        c->sensor=open(sensors[e],O_RDWR|O_CLOEXEC);
        if(c->sensor<0) goto fail;
        c->original=(struct v4l2_subdev_format){.which=V4L2_SUBDEV_FORMAT_ACTIVE,.pad=0};
        c->trigger=(struct v4l2_control){.id=FSIN_CONTROL};
        if(xio(c->sensor,VIDIOC_SUBDEV_G_FMT,&c->original) || xio(c->sensor,VIDIOC_G_CTRL,&c->trigger)) goto fail;
        c->restore=1;
        struct v4l2_subdev_format sf=c->original;
        sf.format.width=STEREO_W; sf.format.height=STEREO_H;
        sf.format.code=raw8 ? MEDIA_BUS_FMT_Y8_1X8:MEDIA_BUS_FMT_Y10_1X10;
        struct v4l2_control trigger={.id=FSIN_CONTROL,.value=1};
        if(xio(c->sensor,VIDIOC_SUBDEV_S_FMT,&sf) || xio(c->sensor,VIDIOC_S_CTRL,&trigger)) goto fail;
        if(sf.format.code!=(raw8 ? MEDIA_BUS_FMT_Y8_1X8:MEDIA_BUS_FMT_Y10_1X10) ||
           sf.format.width!=1280 || sf.format.height!=800) { errno=EPROTO; goto fail; }
        c->fd=open(videos[e],O_RDWR|O_NONBLOCK|O_CLOEXEC);
        if(c->fd<0) goto fail;
        struct v4l2_format fmt={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE};
        fmt.fmt.pix_mp.width=STEREO_W; fmt.fmt.pix_mp.height=STEREO_H;
        fmt.fmt.pix_mp.pixelformat=raw8 ? V4L2_PIX_FMT_GREY:V4L2_PIX_FMT_Y10;
        fmt.fmt.pix_mp.field=V4L2_FIELD_NONE; fmt.fmt.pix_mp.num_planes=1;
        if(xio(c->fd,VIDIOC_S_FMT,&fmt)) goto fail;
        c->stride=fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
        if(fmt.fmt.pix_mp.width!=STEREO_W || fmt.fmt.pix_mp.height!=STEREO_H ||
           fmt.fmt.pix_mp.pixelformat!=(raw8 ? V4L2_PIX_FMT_GREY:V4L2_PIX_FMT_Y10) ||
           fmt.fmt.pix_mp.num_planes!=1 || c->stride<(raw8 ? 1280:1600) ||
           (!raw8 && c->stride>=2560)) { errno=EPROTO; goto fail; }
        struct v4l2_requestbuffers req={.count=3,.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,.memory=V4L2_MEMORY_MMAP};
        if(xio(c->fd,VIDIOC_REQBUFS,&req)) goto fail;
        if(req.count<2 || req.count>3) { errno=ENOMEM; goto fail; }
        c->count=req.count;
        for(unsigned i=0;i<c->count;++i) {
            struct v4l2_plane p={0}; struct v4l2_buffer b={.type=req.type,.memory=req.memory,.index=i,.length=1,.m.planes=&p};
            if(xio(c->fd,VIDIOC_QUERYBUF,&b)) goto fail;
            void *m=mmap(NULL,p.length,PROT_READ|PROT_WRITE,MAP_SHARED,c->fd,p.m.mem_offset);
            if(m==MAP_FAILED) goto fail;
            c->memory[i]=m; c->size[i]=p.length;
            if(queue(c,i)) goto fail;
        }
        enum v4l2_buf_type type=req.type;
        if(xio(c->fd,VIDIOC_STREAMON,&type)) goto fail;
        c->running=1;
    }
    if(quiet(100)) goto fail;
    next_id=now_us();
    return 0;
fail:
    { int e=errno; stereo_capture_close(); errno=e; return -1; }
}

int stereo_capture_pair(uint8_t *left,uint8_t *right,struct CapturePair *pair)
{
    memset(pair,0,sizeof(*pair));
    if(quiet(10)) return -1;
    pair->id=++next_id; pair->trigger_before_us=now_us();
    if(pwm_write("enable","1")) return -1;
    pair->trigger_after_us=now_us();
    unsigned ready=0;
    while(ready!=3 && now_us()-pair->trigger_before_us<200000) {
        struct pollfd f[2];
        for(int e=0;e<2;++e) f[e]=(struct pollfd){.fd=cams[e].fd,.events=(ready&(1u<<e)) ? 0:POLLIN};
        int ret=poll(f,2,10);
        if(ret<0) { if(errno==EINTR) continue; goto fail; }
        for(unsigned e=0;e<2;++e) if(f[e].revents&POLLIN) {
            struct Camera *c=&cams[e]; struct v4l2_plane p={0};
            struct v4l2_buffer b={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,.memory=V4L2_MEMORY_MMAP,.length=1,.m.planes=&p};
            if(xio(c->fd,VIDIOC_DQBUF,&b)) goto fail;
            if(b.index>=c->count || b.flags&V4L2_BUF_FLAG_ERROR ||
               (b.flags&V4L2_BUF_FLAG_TIMESTAMP_MASK)!=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC ||
               p.data_offset || p.bytesused<c->stride*800 || p.bytesused>c->size[b.index]) { errno=EPROTO; goto fail; }
            pair->sequence[e]=b.sequence; pair->flags[e]=b.flags;
            pair->timestamp_us[e]=(int64_t)b.timestamp.tv_sec*1000000+b.timestamp.tv_usec;
            if(pair->timestamp_us[e]<(int64_t)pair->trigger_before_us ||
               pair->timestamp_us[e]>(int64_t)now_us()) { errno=EPROTO; goto fail; }
            const uint8_t *s=c->memory[b.index]; uint8_t *d=e ? right:left;
            for(unsigned y=0;y<800;++y) for(unsigned x=0;x<1280;++x) {
                unsigned value;
                if(raw8_mode) value=s[y*c->stride+x];
                else {
                    /* Rockchip CIF compact RAW10: little-endian 10-bit groups,
                     * matching the existing dashboard's verified unpacker.
                     */
                    const uint8_t *p10=s+y*c->stride+(x/4)*5;
                    unsigned bit=(x%4)*10;
                    unsigned word=p10[bit/8] | ((unsigned)p10[bit/8+1]<<8);
                    value=((word>>(bit%8))&1023)>>2;
                }
                d[y*1280+1279-x]=value; /* existing display/calibration hflip */
            }
            if(queue(c,b.index)) goto fail;
            ready|=1u<<e;
        }
    }
    if(pwm_write("enable","0")) return -1;
    if(ready!=3 || llabs(pair->timestamp_us[0]-pair->timestamp_us[1])>2000 || quiet(20)) {
        errno=EPROTO; return -1;
    }
    return 0;
fail:
    { int e=errno; pwm_write("enable","0"); errno=e; return -1; }
}
