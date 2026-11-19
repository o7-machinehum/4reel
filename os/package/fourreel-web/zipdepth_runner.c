/* Official librknnmrt runs in a private glibc process; never dlopen it in musl.
 * Versioned fixed-layout request + two native images -> response with pair ID,
 * host timings + two float32 320x224 predictions. One shared context.
 * stdout is reserved for this protocol, diagnostics and SDK logs use stderr.
 */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>
#include "rknn_api.h"
#include "im2d.h"
#include "stereo_trace.h"

#define W 320
#define H 224
#define RAW_BYTES (1280*800)

static int cpu_access(rknn_tensor_mem *m,unsigned flags)
{
    struct dma_buf_sync sync={.flags=flags};
    int ret;
    do { ret=ioctl(m->fd,DMA_BUF_IOCTL_SYNC,&sync); } while(ret<0 && errno==EINTR);
    return ret;
}

static int read_all(int fd,void *p,size_t n)
{
    unsigned char *b=p;
    while(n) { ssize_t k=read(fd,b,n); if(k<0 && errno==EINTR) continue;
        if(k<=0) return -1;
        b+=k; n-=k;
    }
    return 0;
}
static int write_all(int fd,const void *p,size_t n)
{
    const unsigned char *b=p;
    while(n) { ssize_t k=write(fd,b,n); if(k<0 && errno==EINTR) continue;
        if(k<=0) return -1;
        b+=k; n-=k;
    }
    return 0;
}

/* In DMA mode the capture process retains both buffers until this response.
 * Only the main thread accesses the RKNN context or its bound input/output.
 * The worker's RGA destination and padded staging are independent of those. */
struct EyePrep {
    int fd;
    unsigned stride;
    rknn_tensor_mem *mirror;
    unsigned eye;
    rknn_tensor_mem *small;
    uint8_t *tensor;
    unsigned tensor_stride;
    unsigned tensor_bytes;
    struct Trace trace;
};

static int prepare_eye(struct EyePrep *p)
{
    unsigned offset=p->eye ? TS_SOURCE_R-TS_SOURCE_L:0;
    struct TraceMark m=trace_begin(&p->trace);
    rga_buffer_t src=wrapbuffer_fd_t(p->fd,1280,800,p->stride,800,RK_FORMAT_YCbCr_400);
    rga_buffer_t dst=wrapbuffer_fd_t(p->small->fd,W,200,W,200,RK_FORMAT_YCbCr_400);
    trace_end(&p->trace,m,TS_SOURCE_L+offset,TR_HELPER);
    m=trace_begin(&p->trace);
    /* Mirroring after reduction or fusing flip/scale changes the sampling
     * phase on RGA2e. Mirror the native plane first to preserve the model input. */
    if(p->mirror) {
        rga_buffer_t tmp=wrapbuffer_fd_t(p->mirror->fd,1280,800,1280,800,RK_FORMAT_YCbCr_400);
        IM_STATUS flipped=imflip(src,tmp,IM_HAL_TRANSFORM_FLIP_H,1);
        if(flipped!=IM_STATUS_SUCCESS) {
            fprintf(stderr,"RGA eye %u native mirror failed: %s\n",p->eye,imStrError(flipped));
            return -1;
        }
        trace_end(&p->trace,m,TS_RGA_L+offset,TR_RGA);
        m=trace_begin(&p->trace);
        src=tmp;
    }
    IM_STATUS status=imresize_t(src,dst,0,0,IM_INTERP_AVERAGE,1);
    if(status!=IM_STATUS_SUCCESS) {
        fprintf(stderr,"RGA eye %u gray resize failed: %s\n",p->eye,imStrError(status));
        return -1;
    }
    trace_end(&p->trace,m,TS_RGA_L+offset,TR_RGA);
    m=trace_begin(&p->trace);
    if(cpu_access(p->small,DMA_BUF_SYNC_START|DMA_BUF_SYNC_READ)) return -1;
    memset(p->tensor,0,p->tensor_bytes);
    for(unsigned y=0;y<200;++y) {
        uint8_t *d=p->tensor+(y+12)*p->tensor_stride;
        const uint8_t *s=(const uint8_t *)p->small->virt_addr+y*W;
        memcpy(d,s,W);
    }
    if(cpu_access(p->small,DMA_BUF_SYNC_END|DMA_BUF_SYNC_READ)) return -1;
    trace_end(&p->trace,m,TS_TENSOR_L+offset,TR_HELPER);
    return 0;
}

static void append_trace(struct Trace *dest,const struct Trace *src)
{
    for(unsigned i=0;i<src->count;++i) {
        if(dest->count<TRACE_EVENTS) dest->event[dest->count++]=src->event[i];
        else ++dest->overflow;
    }
    dest->overflow+=src->overflow;
}

static int infer_eye(rknn_context ctx,rknn_tensor_mem *out,unsigned stride,float *prediction,
                     unsigned eye,struct Trace *t)
{
    unsigned offset=eye ? TS_SOURCE_R-TS_SOURCE_L:0;
    struct TraceMark m=trace_begin(t);
    if(rknn_run(ctx,NULL)) return -1;
    trace_end(t,m,TS_NPU_L+offset,TR_NPU);
    m=trace_begin(t);
    for(unsigned y=0;y<H;++y)
        memcpy(prediction+y*W,(float *)out->virt_addr+y*stride,W*sizeof(float));
    for(unsigned i=0;i<W*H;++i) if(!isfinite(prediction[i])) return -1;
    trace_end(t,m,TS_OUTPUT_L+offset,TR_HELPER);
    return 0;
}

struct PrepWorker {
    struct EyePrep prep;
    pthread_t tid;
    sem_t request,done;
    uint64_t cpu;
    int stop,ret,started;
};
static void wait_sem(sem_t *s)
{
    while(sem_wait(s)<0 && errno==EINTR) {}
}
static void *prep_loop(void *arg)
{
    struct PrepWorker *w=arg;
    for(;;) {
        wait_sem(&w->request);
        if(w->stop) break;
        uint64_t begin=trace_clock(CLOCK_THREAD_CPUTIME_ID);
        w->ret=prepare_eye(&w->prep);
        w->cpu=trace_clock(CLOCK_THREAD_CPUTIME_ID)-begin;
        sem_post(&w->done);
    }
    return NULL;
}
static int start_worker(struct PrepWorker *w)
{
    if(sem_init(&w->request,0,0)) return -1;
    if(sem_init(&w->done,0,0)) { sem_destroy(&w->request); return -1; }
    if(pthread_create(&w->tid,NULL,prep_loop,w)) {
        sem_destroy(&w->request); sem_destroy(&w->done);
        return -1;
    }
    w->started=1;
    return 0;
}

int main(int argc,char **argv)
{
    if(argc!=2) { fprintf(stderr,"Usage: zipdepth-runner model.rknn < requests > responses\n"); return 2; }
    int protocol=dup(STDOUT_FILENO);
    if(protocol<0 || dup2(STDERR_FILENO,STDOUT_FILENO)<0) return 1;
    rknn_context ctx=0;
    rknn_tensor_mem *in=NULL,*out=NULL,*rga_src=NULL,*rga_dst=NULL;
    uint8_t *native=NULL,*right_tensor=NULL;
    rknn_tensor_mem *right_dst=NULL;
    struct PrepWorker worker={0};
    int result=1;
    /* Model-path initialization avoids holding an extra 6.5 MB userspace copy. */
    if(rknn_init(&ctx,argv[1],0,0,NULL)!=RKNN_SUCC) goto done;
    rknn_sdk_version version={0};
    if(rknn_query(ctx,RKNN_QUERY_SDK_VERSION,&version,sizeof(version))) goto done;
    fprintf(stderr,"RKNN api=%s driver=%s\n",version.api_version,version.drv_version);
    if(!strstr(version.api_version,"2.5.0") || !strstr(version.drv_version,"0.9.8")) {
        fprintf(stderr,"Unvalidated runtime/driver combination; refusing inference\n"); goto done;
    }
    rknn_input_output_num count={0};
    rknn_tensor_attr ia={.index=0},oa={.index=0};
    if(rknn_query(ctx,RKNN_QUERY_IN_OUT_NUM,&count,sizeof(count)) || count.n_input!=1 || count.n_output!=1 ||
       rknn_query(ctx,RKNN_QUERY_INPUT_ATTR,&ia,sizeof(ia)) ||
       rknn_query(ctx,RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR,&oa,sizeof(oa))) goto done;
    if(ia.n_dims!=4 || ia.n_elems!=W*H || oa.n_dims!=4 || oa.n_elems!=W*H ||
       oa.fmt!=RKNN_TENSOR_NHWC || oa.dims[0]!=1 || oa.dims[1]!=H || oa.dims[2]!=W || oa.dims[3]!=1)
        goto done;
    if(ia.fmt==RKNN_TENSOR_NHWC) {
        if(ia.dims[0]!=1 || ia.dims[1]!=H || ia.dims[2]!=W || ia.dims[3]!=1) goto done;
    } else if(ia.fmt==RKNN_TENSOR_NCHW) {
        if(ia.dims[0]!=1 || ia.dims[1]!=1 || ia.dims[2]!=H || ia.dims[3]!=W) goto done;
    } else goto done;
    unsigned is=ia.w_stride ? ia.w_stride:W, os=oa.w_stride ? oa.w_stride:W;
    if(is<W || os<W || is>4096 || os>4096) goto done;
    ia.type=RKNN_TENSOR_UINT8; ia.fmt=RKNN_TENSOR_NHWC; ia.pass_through=0;
    oa.type=RKNN_TENSOR_FLOAT32;
    unsigned ibytes=ia.size_with_stride>is*H ? ia.size_with_stride:is*H;
    unsigned obytes=oa.size_with_stride>os*H*4 ? oa.size_with_stride:os*H*4;
    in=rknn_create_mem(ctx,ibytes); out=rknn_create_mem(ctx,obytes);
    if(!in || !out || rknn_set_io_mem(ctx,in,&ia) || rknn_set_io_mem(ctx,out,&oa)) goto done;
    rga_src=rknn_create_mem(ctx,RAW_BYTES);
    rga_dst=rknn_create_mem(ctx,W*200);
    if(!rga_src || rga_src->fd<0 || !rga_dst || rga_dst->fd<0) goto done;
    fprintf(stderr,"RKNN input stride=%u bytes=%u output stride=%u bytes=%u; shared context\n",is,ibytes,os,obytes);
    float predictions[2][W*H];
    struct RunnerRequest request;
    while(!read_all(STDIN_FILENO,&request,sizeof(request))) {
        struct RunnerResponse response={.id=request.id,.trace={.enabled=request.enabled}};
        uint64_t thread_start=trace_clock(CLOCK_THREAD_CPUTIME_ID);
        struct EyePrep p={.small=rga_dst,.tensor=in->virt_addr,.tensor_stride=is,.tensor_bytes=ibytes};
        if(request.protocol==TRACE_DMA_PROTOCOL) {
            struct RunnerDmaPair pair;
            if(read_all(STDIN_FILENO,&pair,sizeof(pair))) goto done;
            for(unsigned e=0;e<2;++e) {
                if(pair.fd[e]<3 || pair.stride[e]<1280 || pair.stride[e]>8192) goto done;
            }
            if(!worker.started) {
                right_dst=rknn_create_mem(ctx,W*200);
                right_tensor=malloc(ibytes);
                if(!right_dst || right_dst->fd<0 || !right_tensor || start_worker(&worker)) goto done;
            }
            p.mirror=rga_src;
            p.fd=pair.fd[0]; p.stride=pair.stride[0]; p.trace.enabled=request.enabled;
            if(prepare_eye(&p)) goto done;
            append_trace(&response.trace,&p.trace);
            worker.prep=(struct EyePrep){.eye=1,.mirror=rga_src,.fd=pair.fd[1],.stride=pair.stride[1],
                .small=right_dst,.tensor=right_tensor,.tensor_stride=is,.tensor_bytes=ibytes,
                .trace={.enabled=request.enabled}};
            sem_post(&worker.request);
            int ret=infer_eye(ctx,out,os,predictions[0],0,&response.trace);
            response.first_eye_us=trace_clock(CLOCK_MONOTONIC);
            /* Join the in-flight right preparation even if left inference failed. */
            wait_sem(&worker.done);
            if(ret || worker.ret) goto done;
            response.cpu_us=worker.cpu;
            append_trace(&response.trace,&worker.prep.trace);
            struct TraceMark m=trace_begin(&response.trace);
            memcpy(in->virt_addr,right_tensor,ibytes);
            trace_end(&response.trace,m,TS_TENSOR_R,TR_HELPER);
            if(infer_eye(ctx,out,os,predictions[1],1,&response.trace)) goto done;
        } else if(request.protocol==TRACE_PROTOCOL) {
            if(!native) {
                native=malloc(2*RAW_BYTES);
                if(!native) goto done;
            }
            if(read_all(STDIN_FILENO,native,2*RAW_BYTES)) goto done;
            for(unsigned eye=0;eye<2;++eye) {
                struct TraceMark m=trace_begin(&response.trace);
                if(cpu_access(rga_src,DMA_BUF_SYNC_START|DMA_BUF_SYNC_WRITE)) goto done;
                memcpy(rga_src->virt_addr,native+eye*RAW_BYTES,RAW_BYTES);
                if(cpu_access(rga_src,DMA_BUF_SYNC_END|DMA_BUF_SYNC_WRITE)) goto done;
                trace_end(&response.trace,m,eye ? TS_SOURCE_R:TS_SOURCE_L,TR_HELPER);
                p.fd=rga_src->fd; p.stride=1280; p.eye=eye;
                p.trace=(struct Trace){.enabled=request.enabled};
                if(prepare_eye(&p)) goto done;
                append_trace(&response.trace,&p.trace);
                if(infer_eye(ctx,out,os,predictions[eye],eye,&response.trace)) goto done;
                if(!eye) response.first_eye_us=trace_clock(CLOCK_MONOTONIC);
            }
        } else goto done;
        response.complete_us=trace_clock(CLOCK_MONOTONIC);
        response.cpu_us+=trace_clock(CLOCK_THREAD_CPUTIME_ID)-thread_start;
        if(write_all(protocol,&response,sizeof(response)) || write_all(protocol,predictions,sizeof(predictions))) goto done;
    }
    result=0;
done:
    if(result) fprintf(stderr,"ZipDepth inference unavailable; no synthetic/fallback prediction emitted\n");
    if(worker.started) {
        worker.stop=1; sem_post(&worker.request);
        pthread_join(worker.tid,NULL);
        sem_destroy(&worker.request); sem_destroy(&worker.done);
    }
    free(right_tensor);
    if(right_dst) rknn_destroy_mem(ctx,right_dst);
    if(in) rknn_destroy_mem(ctx,in);
    if(out) rknn_destroy_mem(ctx,out);
    if(rga_src) rknn_destroy_mem(ctx,rga_src);
    if(rga_dst) rknn_destroy_mem(ctx,rga_dst);
    free(native);
    if(ctx) rknn_destroy(ctx);
    close(protocol);
    return result;
}
