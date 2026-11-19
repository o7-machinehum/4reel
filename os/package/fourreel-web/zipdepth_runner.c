/* Official librknnmrt runs in a private glibc process; never dlopen it in musl.
 * Pipe protocol (little endian ARM/host): uint64 pair ID, two 1280x800 uint8
 * images -> same ID, two float32 320x224 predictions. One shared context.
 * stdout is reserved for this protocol, diagnostics and SDK logs use stderr.
 */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>
#include "rknn_api.h"
#include "im2d.h"

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

int main(int argc,char **argv)
{
    if(argc!=2) { fprintf(stderr,"Usage: zipdepth-runner model.rknn < requests > responses\n"); return 2; }
    int protocol=dup(STDOUT_FILENO);
    if(protocol<0 || dup2(STDERR_FILENO,STDOUT_FILENO)<0) return 1;
    rknn_context ctx=0;
    rknn_tensor_mem *in=NULL,*out=NULL,*rga_src=NULL,*rga_dst=NULL;
    uint8_t *native=NULL;
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
    native=malloc(2*RAW_BYTES);
    if(!rga_src || !rga_dst || !native || rga_src->fd<0 || rga_dst->fd<0) goto done;
    rga_buffer_t src=wrapbuffer_fd_t(rga_src->fd,1280,800,1280,800,RK_FORMAT_YCbCr_400);
    rga_buffer_t dst=wrapbuffer_fd_t(rga_dst->fd,W,200,W,200,RK_FORMAT_YCbCr_400);
    fprintf(stderr,"RKNN input stride=%u bytes=%u output stride=%u bytes=%u; shared context\n",is,ibytes,os,obytes);
    float predictions[2][W*H];
    uint64_t id;
    while(!read_all(STDIN_FILENO,&id,sizeof(id))) {
        if(read_all(STDIN_FILENO,native,2*RAW_BYTES)) goto done;
        for(unsigned eye=0;eye<2;++eye) {
            if(cpu_access(rga_src,DMA_BUF_SYNC_START|DMA_BUF_SYNC_WRITE)) goto done;
            memcpy(rga_src->virt_addr,native+eye*RAW_BYTES,RAW_BYTES);
            if(cpu_access(rga_src,DMA_BUF_SYNC_END|DMA_BUF_SYNC_WRITE)) goto done;
            if(imresize_t(src,dst,0,0,IM_INTERP_AVERAGE,1)!=IM_STATUS_SUCCESS) {
                fprintf(stderr,"RGA native gray resize failed; no silent CPU substitution\n"); goto done;
            }
            if(cpu_access(rga_dst,DMA_BUF_SYNC_START|DMA_BUF_SYNC_READ)) goto done;
            memset(in->virt_addr,0,ibytes);
            for(unsigned y=0;y<200;++y)
                memcpy((uint8_t *)in->virt_addr+(y+12)*is,(uint8_t *)rga_dst->virt_addr+y*W,W);
            if(cpu_access(rga_dst,DMA_BUF_SYNC_END|DMA_BUF_SYNC_READ)) goto done;
            /* Default rknn_run synchronizes bound input/output caches. Neither
             * RKNN_FLAG_DISABLE_FLUSH_* flag is set. rknn_run is synchronous.
             */
            if(rknn_run(ctx,NULL)) goto done;
            for(unsigned y=0;y<H;++y) memcpy(predictions[eye]+y*W,(float *)out->virt_addr+y*os,W*4);
            for(unsigned i=0;i<W*H;++i) if(!isfinite(predictions[eye][i])) goto done;
        }
        if(write_all(protocol,&id,sizeof(id)) || write_all(protocol,predictions,sizeof(predictions))) goto done;
    }
    result=0;
done:
    if(result) fprintf(stderr,"ZipDepth inference unavailable; no synthetic/fallback prediction emitted\n");
    if(in) rknn_destroy_mem(ctx,in);
    if(out) rknn_destroy_mem(ctx,out);
    if(rga_src) rknn_destroy_mem(ctx,rga_src);
    if(rga_dst) rknn_destroy_mem(ctx,rga_dst);
    free(native);
    if(ctx) rknn_destroy(ctx);
    close(protocol);
    return result;
}
