#define _POSIX_C_SOURCE 200809L
#include "stereo_capture.h"
#include "stereo_metric.h"
#include "stereo_trace.h"
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
    unsigned count,stride,exported_count;
    int dma_fd[3];
};
static struct Camera cams[2]={{.fd=-1,.sensor=-1},{.fd=-1,.sensor=-1}};
/* Indices of buffers dequeued by the application and not yet requeued. */
static int held[2]={-1,-1};
static char pwm[128];
static int raw8_mode;
static uint64_t next_id;
static uint32_t previous_sequence[2];
static int sequence_seen[2];

static uint64_t now_us(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t);
    return (uint64_t)t.tv_sec*1000000+t.tv_nsec/1000;
}
static int xio(int fd,unsigned long cmd,void *p)
{ int r; do { r=ioctl(fd,cmd,p); } while(r<0 && errno==EINTR); return r; }
enum { AE_EXPOSURE, AE_GAIN, AE_CONTROLS };
static const unsigned ae_id[AE_CONTROLS]={V4L2_CID_EXPOSURE,V4L2_CID_ANALOGUE_GAIN};
static struct {
    int saved[2][AE_CONTROLS], current[AE_CONTROLS];
    int low[AE_CONTROLS], high[AE_CONTROLS];
    int active, settling, dark_pairs;
} ae;

static int write_setting(unsigned eye,unsigned control,int value)
{
    struct v4l2_control c={.id=ae_id[control],.value=value};
    return xio(cams[eye].sensor,VIDIOC_S_CTRL,&c);
}

int stereo_capture_enable_ae(void)
{
    for(unsigned k=0;k<AE_CONTROLS;k++) {
        for(unsigned e=0;e<2;e++) {
            struct v4l2_queryctrl q={.id=ae_id[k]};
            struct v4l2_control c={.id=ae_id[k]};
            if(xio(cams[e].sensor,VIDIOC_QUERYCTRL,&q) || xio(cams[e].sensor,VIDIOC_G_CTRL,&c)) return -1;
            if(q.step!=1 || q.minimum<=0 || (q.flags&(V4L2_CTRL_FLAG_DISABLED|V4L2_CTRL_FLAG_READ_ONLY))) {
                errno=ENOTSUP; return -1;
            }
            ae.saved[e][k]=c.value;
            if(!e || q.minimum>ae.low[k]) ae.low[k]=q.minimum;
            if(!e || q.maximum<ae.high[k]) ae.high[k]=q.maximum;
        }
        /* At 3652 lines/15.5x this triggered rig returns dark striped images
         * in bright light. 1600 lines/4x passed the native-row sweep. */
        int tested_max=k==AE_EXPOSURE ? 1600:64;
        if(ae.high[k]>tested_max) ae.high[k]=tested_max;
        if(ae.low[k]>ae.high[k]) { errno=ERANGE; return -1; }
        int value=ae.saved[0][k];
        ae.current[k]=value<ae.low[k]?ae.low[k]:value>ae.high[k]?ae.high[k]:value;
    }
    /* All originals are known before the first write. Close restores both
     * cameras even if setup or a later paired update fails part way through. */
    ae.active=1;
    for(unsigned e=0;e<2;e++) for(unsigned k=0;k<AE_CONTROLS;k++)
        if(ae.saved[e][k]!=ae.current[k] && write_setting(e,k,ae.current[k])) return -1;
    return 0;
}

/* 32 x 20 samples from the central field of each eye, before model padding.
 * Outer fisheye corners stay outside the meter. No image allocation or copy. */
static void meter_eye(struct StereoImage eye,int *p75,int *clipped)
{
    unsigned hist[256]={0}, count=0, bright=0;
    for(int gy=0;gy<20;gy++) {
        const uint8_t *row=eye.origin+(172+24*gy)*eye.stride;
        for(int gx=0;gx<32;gx++) {
            /* Interleave odd/even rows and columns within the grid. */
            int x=268+24*gx+(gy&1);
            unsigned v=row[(gx&1)*eye.stride+x*eye.xstep];
            ++hist[v]; ++count; bright+=v>=250;
        }
    }
    unsigned acc=0;
    for(int v=0;v<256;v++) {
        acc+=hist[v];
        if(4*acc>=3*count) { *p75=v; break; }
    }
    *clipped=(int)((1000*bright+count/2)/count);
}
static int bounded(double value,int low,int high)
{
    return value<low?low:value>high?high:(int)(value+.5);
}

int stereo_capture_adjust_ae(const struct StereoImage eyes[2],struct PairTrace *record)
{
    struct ExposureStats *s=&record->exposure;
    s->enabled=1;
    s->lines=ae.current[AE_EXPOSURE]; s->gain=ae.current[AE_GAIN];
    s->next_lines=s->lines; s->next_gain=s->gain;
    struct TraceMark mark=trace_begin(&record->trace);
    for(unsigned e=0;e<2;e++) meter_eye(eyes[e],&s->p75[e],&s->clip_permille[e]);
    int level=s->p75[0]>s->p75[1]?s->p75[0]:s->p75[1];
    int clip=s->clip_permille[0]>s->clip_permille[1]?s->clip_permille[0]:s->clip_permille[1];
    double factor=1;
    if(clip>=200) factor=.25;
    else if(clip>=50 && level>110) factor=.65;
    else if(level>150 || level<105) {
        /* The board's monochrome pipeline has a nominal black level of 16. */
        factor=112.0/(level>16?level-16:1);
        if(factor<.5) factor=.5;
        if(factor>1.6) factor=1.6;
        if(clip>=30 && factor>1) factor=1;
    }
    trace_end(&record->trace,mark,TS_AE_METER,TR_MAIN);
    if(ae.settling) { --ae.settling; return 0; }
    /* Avoid chasing dark phases of a flickering lamp or screen. Bright
     * clipping still acts immediately after the control has settled. */
    if(factor>1) {
        if(++ae.dark_pairs<3) return 0;
    }
    ae.dark_pairs=0;
    if(factor==1) return 0;
    /* Spend the light budget on shutter first up to the board's established
     * motion limit. In dim scenes add gain before allowing longer exposure. */
    double product=(double)s->lines*s->gain*factor;
    int motion=bounded(900,ae.low[AE_EXPOSURE],ae.high[AE_EXPOSURE]);
    int gain_limit=bounded(128,ae.low[AE_GAIN],ae.high[AE_GAIN]);
    int gain=bounded(product/motion,ae.low[AE_GAIN],gain_limit);
    int lines=bounded(product/gain,ae.low[AE_EXPOSURE],ae.high[AE_EXPOSURE]);
    if(lines==ae.high[AE_EXPOSURE]) gain=bounded(product/lines,ae.low[AE_GAIN],ae.high[AE_GAIN]);
    int next[AE_CONTROLS]={lines,gain};
    if(lines==s->lines && gain==s->gain) return 0;
    /* Both images have been dequeued and no new FSIN can fire until return.
     * If an I2C write fails the caller stops capture before another trigger. */
    mark=trace_begin(&record->trace);
    for(unsigned e=0;e<2;e++) for(unsigned k=0;k<AE_CONTROLS;k++)
        if(next[k]!=ae.current[k] && write_setting(e,k,next[k])) return -1;
    trace_end(&record->trace,mark,TS_AE_WRITE,TR_MAIN);
    memcpy(ae.current,next,sizeof(next));
    s->next_lines=lines; s->next_gain=gain;
    ae.settling=1;
    return 0;
}

static int pwm_write(const char *name,const char *value)
{
    char path[180]; snprintf(path,sizeof(path),"%s/%s",pwm,name);
    int fd=open(path,O_WRONLY|O_CLOEXEC);
    if(fd<0) return -1;
    size_t n=strlen(value); ssize_t k=write(fd,value,n); int e=errno; close(fd); errno=e;
    return k==(ssize_t)n ? 0:-1;
}
/* A freshly exported PWM can have a zero period. The kernel rejects even a
 * disable request or a zero duty until a valid period has been established. */
static int pwm_prepare(void)
{
    char path[180];
    snprintf(path,sizeof(path),"%s/period",pwm);
    FILE *file=fopen(path,"r");
    if(!file) return -1;
    unsigned long long period;
    int scanned=fscanf(file,"%llu",&period);
    fclose(file);
    if(scanned!=1) { errno=EPROTO; return -1; }
    if(period==0 && pwm_write("period","1000000000")) return -1;
    if(pwm_write("enable","0") || pwm_write("duty_cycle","0") ||
       pwm_write("period","1000000000") || pwm_write("polarity","normal") ||
       pwm_write("duty_cycle","10000")) return -1;
    return 0;
}

static int queue(struct Camera *c,unsigned index)
{
    struct v4l2_plane p={0}; struct v4l2_buffer b={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
        .memory=V4L2_MEMORY_MMAP,.index=index,.length=1,.m.planes=&p};
    return xio(c->fd,VIDIOC_QBUF,&b);
}

static void close_exports(void)
{
    for(unsigned eye=0;eye<2;eye++) {
        struct Camera *c=&cams[eye];
        while(c->exported_count)
            close(c->dma_fd[--c->exported_count]);
    }
}

int stereo_capture_export(void)
{
    if(!cams[0].running || !cams[1].running) { errno=ENODEV; return -1; }
    if(!raw8_mode) { errno=ENOTSUP; return -1; }
    for(unsigned eye=0;eye<2;eye++) {
        struct Camera *c=&cams[eye];
        while(c->exported_count<c->count) {
            unsigned index=c->exported_count;
            /* No O_CLOEXEC: the private helper inherits these fds before its
             * exec and receives the actual dequeued fd number for each eye. */
            struct v4l2_exportbuffer buf={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                .index=index,.plane=0,.flags=O_RDONLY};
            if(xio(c->fd,VIDIOC_EXPBUF,&buf)) goto fail;
            if(buf.fd<=STDERR_FILENO) {
                int fd=fcntl(buf.fd,F_DUPFD,STDERR_FILENO+1);
                int saved_errno=errno;
                close(buf.fd);
                if(fd<0) { errno=saved_errno; goto fail; }
                buf.fd=fd;
            }
            c->dma_fd[c->exported_count++]=buf.fd;
        }
    }
    return 0;
fail:
    { int saved_errno=errno; close_exports(); errno=saved_errno; return -1; }
}

int stereo_capture_release(void)
{
    int error=0;
    for(unsigned eye=0;eye<2;eye++) {
        if(held[eye]<0) continue;
        if(queue(&cams[eye],(unsigned)held[eye])) {
            if(!error) error=errno;
        } else {
            held[eye]=-1;
        }
    }
    if(error) { errno=error; return -1; }
    return 0;
}

void stereo_capture_close(void)
{
    if(pwm[0]) pwm_write("enable","0");
    close_exports();
    for(unsigned e=0;e<2;++e) {
        struct Camera *c=&cams[e];
        if(c->running) { enum v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; xio(c->fd,VIDIOC_STREAMOFF,&t); }
        for(unsigned i=0;i<c->count;++i) if(c->memory[i]) munmap(c->memory[i],c->size[i]);
        if(c->fd>=0) close(c->fd);
        if(c->sensor>=0) {
            if(c->restore) { xio(c->sensor,VIDIOC_S_CTRL,&c->trigger); xio(c->sensor,VIDIOC_SUBDEV_S_FMT,&c->original); }
            if(ae.active) for(unsigned k=0;k<AE_CONTROLS;k++)
                if(write_setting(e,k,ae.saved[e][k])) perror("restore sensor exposure/gain");
            close(c->sensor);
        }
        memset(c,0,sizeof(*c)); c->fd=c->sensor=-1;
        held[e]=-1;
    }
    memset(&ae,0,sizeof(ae));
    pwm[0]=0;
}

static int quiet(int ms)
{
    struct pollfd f[2]={{.fd=cams[0].fd,.events=POLLIN},{.fd=cams[1].fd,.events=POLLIN}};
    int r; do { r=poll(f,2,ms); } while(r<0 && errno==EINTR);
    if(r!=0) { errno=EPROTO; return -1; } /* no frames while FSIN is disabled */
    return 0;
}

/* CIF can queue startup frames while each sensor is being armed. Nothing
 * discarded here receives a pair ID or reaches clients. Limit the count as
 * well as the time so a free-running sensor cannot pass the startup gate. */
static int discard_startup_frames(unsigned ms,unsigned limit,int require_both)
{
    unsigned counts[2]={0,0};
    const uint64_t deadline=now_us()+ms*1000;
    struct pollfd f[2]={{.fd=cams[0].fd,.events=POLLIN},{.fd=cams[1].fd,.events=POLLIN}};
    for(;;) {
        uint64_t now=now_us();
        if(now>=deadline) break;
        int r=poll(f,2,(int)((deadline-now+999)/1000));
        if(r<0) { if(errno==EINTR) continue; return -1; }
        for(unsigned e=0;e<2;e++) {
            if(f[e].revents & (POLLERR|POLLHUP|POLLNVAL)) { errno=EIO; return -1; }
            if(!(f[e].revents&POLLIN)) continue;
            struct v4l2_plane plane={0};
            struct v4l2_buffer buffer={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
                .memory=V4L2_MEMORY_MMAP,.length=1,.m.planes=&plane};
            if(xio(cams[e].fd,VIDIOC_DQBUF,&buffer)) return -1;
            if(buffer.index>=cams[e].count || ++counts[e]>limit) { errno=EPROTO; return -1; }
            if(queue(&cams[e],buffer.index)) return -1;
        }
    }
    if(require_both && (!counts[0] || !counts[1])) { errno=ETIMEDOUT; return -1; }
    return 0;
}

static int settle_snapshot_mode(void)
{
    if(discard_startup_frames(110,3,0) || quiet(100)) return -1;
    /* Wake the sensor after the extended idle and prime CIF. The second
     * pulse must deliver exactly one frame per eye, with quiet between them.
     * The next pulse is the first that can produce a published pair. */
    for(unsigned i=0;i<2;i++) {
        if(pwm_write("enable","1") || discard_startup_frames(50,1,i==1) ||
           pwm_write("enable","0") || quiet(30)) return -1;
    }
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
    if(!pwm[0]) { errno=ENODEV; goto fail; }
    if(pwm_prepare()) goto fail;
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
            if(p.length/STEREO_H<c->stride) { errno=EPROTO; goto fail; }
            void *m=mmap(NULL,p.length,PROT_READ|PROT_WRITE,MAP_SHARED,c->fd,p.m.mem_offset);
            if(m==MAP_FAILED) goto fail;
            c->memory[i]=m; c->size[i]=p.length;
            if(queue(c,i)) goto fail;
        }
        enum v4l2_buf_type type=req.type;
        if(xio(c->fd,VIDIOC_STREAMON,&type)) goto fail;
        c->running=1;
    }
    if(settle_snapshot_mode()) goto fail;
    next_id=now_us();
    memset(sequence_seen,0,sizeof(sequence_seen));
    return 0;
fail:
    { int e=errno; stereo_capture_close(); errno=e; return -1; }
}

static void copy_oriented(const uint8_t *s,unsigned stride,uint8_t *d)
{
    for(unsigned y=0;y<800;++y) for(unsigned x=0;x<1280;++x) {
        unsigned value;
        if(raw8_mode) value=s[(size_t)y*stride+x];
        else {
            /* Rockchip CIF compact RAW10: little-endian 10-bit groups,
             * matching the existing dashboard's verified unpacker.
             */
            const uint8_t *p10=s+(size_t)y*stride+(x/4)*5;
            unsigned bit=(x%4)*10;
            unsigned word=p10[bit/8] | ((unsigned)p10[bit/8+1]<<8);
            value=((word>>(bit%8))&1023)>>2;
        }
        d[y*1280+1279-x]=value; /* existing display/calibration hflip */
    }
}

static int acquire_pair(uint8_t *left,uint8_t *right,struct CapturePair *pair,struct Trace *trace,int dma)
{
    if(!cams[0].running || !cams[1].running) { errno=ENODEV; return -1; }
    if(held[0]>=0 || held[1]>=0) { errno=EBUSY; return -1; }
    if(!pair || (!dma && (!left || !right))) { errno=EINVAL; return -1; }
    memset(pair,0,sizeof(*pair));
    struct TraceMark m=trace_begin(trace);
    if(quiet(10)) return -1;
    trace_end(trace,m,TS_QUIET_PRE,TR_WAIT);
    m=trace_begin(trace);
    pair->id=++next_id; pair->trigger_before_us=now_us();
    if(pwm_write("enable","1")) return -1;
    pair->trigger_after_us=now_us();
    trace_end(trace,m,TS_TRIGGER,TR_MAIN);
    unsigned ready=0;
    while(ready!=3 && now_us()-pair->trigger_before_us<200000) {
        struct pollfd f[2];
        for(int e=0;e<2;++e) f[e]=(struct pollfd){.fd=cams[e].fd,.events=(ready&(1u<<e)) ? 0:POLLIN};
        m=trace_begin(trace);
        int ret=poll(f,2,10);
        trace_end(trace,m,TS_CAPTURE_WAIT,TR_WAIT);
        if(ret<0) { if(errno==EINTR) continue; goto fail; }
        for(unsigned e=0;e<2;++e) if(f[e].revents&POLLIN) {
            m=trace_begin(trace);
            struct Camera *c=&cams[e]; struct v4l2_plane p={0};
            struct v4l2_buffer b={.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,.memory=V4L2_MEMORY_MMAP,.length=1,.m.planes=&p};
            if(xio(c->fd,VIDIOC_DQBUF,&b)) goto fail;
            pair->dequeued_us[e]=now_us();
            if(b.index>=c->count) { errno=EPROTO; goto fail; }
            held[e]=(int)b.index;
            if(b.flags&V4L2_BUF_FLAG_ERROR ||
               (b.flags&V4L2_BUF_FLAG_TIMESTAMP_MASK)!=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC ||
               p.data_offset || p.bytesused/STEREO_H<c->stride || p.bytesused>c->size[b.index]) { errno=EPROTO; goto fail; }
            pair->sequence[e]=b.sequence; pair->flags[e]=b.flags;
            if(sequence_seen[e]) {
                uint32_t delta=b.sequence-previous_sequence[e];
                if(!delta || delta>0x7fffffffu) { errno=EPROTO; goto fail; }
                pair->sequence_gap[e]=delta-1;
            }
            previous_sequence[e]=b.sequence; sequence_seen[e]=1;
            pair->timestamp_us[e]=(int64_t)b.timestamp.tv_sec*1000000+b.timestamp.tv_usec;
            if(pair->timestamp_us[e]<(int64_t)pair->trigger_before_us ||
               pair->timestamp_us[e]>(int64_t)now_us()) { errno=EPROTO; goto fail; }
            if(!dma) {
                copy_oriented(c->memory[b.index],c->stride,e ? right:left);
                if(queue(c,b.index)) goto fail;
                held[e]=-1;
            }
            ready|=1u<<e;
            unsigned stage=dma ? (e ? TS_DMA_R:TS_DMA_L):(e ? TS_UNPACK_R:TS_UNPACK_L);
            trace_end(trace,m,stage,TR_MAIN);
        }
    }
    if(pwm_write("enable","0")) goto fail;
    m=trace_begin(trace);
    if(ready!=3 || llabs(pair->timestamp_us[0]-pair->timestamp_us[1])>2000 || quiet(20)) {
        errno=EPROTO; goto fail;
    }
    trace_end(trace,m,TS_QUIET_POST,TR_WAIT);
    return 0;
fail:
    { int e=errno; pwm_write("enable","0"); stereo_capture_release(); errno=e; return -1; }
}

int stereo_capture_pair_dma(struct CapturePair *pair,struct Trace *trace,struct CaptureDmaView *view)
{
    if(!view) { errno=EINVAL; return -1; }
    *view=(struct CaptureDmaView){.fd={-1,-1}};
    if(!cams[0].running || !cams[1].running) { errno=ENODEV; return -1; }
    if(!raw8_mode || cams[0].exported_count!=cams[0].count ||
       cams[1].exported_count!=cams[1].count) { errno=ENOTSUP; return -1; }
    if(acquire_pair(NULL,NULL,pair,trace,1)) return -1;
    for(unsigned eye=0;eye<2;eye++) {
        const struct Camera *c=&cams[eye];
        view->fd[eye]=c->dma_fd[held[eye]];
        view->stride[eye]=c->stride;
        view->data[eye]=c->memory[held[eye]];
    }
    return 0;
}

int stereo_capture_pair_timed(uint8_t *left,uint8_t *right,struct CapturePair *pair,struct Trace *trace)
{ return acquire_pair(left,right,pair,trace,0); }

int stereo_capture_pair(uint8_t *l,uint8_t *r,struct CapturePair *p)
{ return stereo_capture_pair_timed(l,r,p,NULL); }
