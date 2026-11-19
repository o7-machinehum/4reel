/* Explicit host-only mock fixture. Never linked into the installed pipeline. */
#define _POSIX_C_SOURCE 200809L
#define STEREO_PREVIEW_PAGE "os/package/fourreel-web/stereo_preview.html"
#include "stereo_preview.c"
#include <assert.h>
#include <sys/time.h>
static char *request(unsigned port,const char *method) {
 int fd=socket(AF_INET,SOCK_STREAM,0); assert(fd>=0);
 struct timeval timeout={.tv_sec=5};
 setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
 struct sockaddr_in addr={.sin_family=AF_INET,.sin_port=htons(port),.sin_addr={.s_addr=htonl(INADDR_LOOPBACK)}};
 assert(!connect(fd,(struct sockaddr *)&addr,sizeof(addr)));
 assert(send(fd,method,strlen(method),MSG_NOSIGNAL)==(ssize_t)strlen(method));
 char *buf=calloc(1,400000); assert(buf); size_t n=0;
 for(;;) { ssize_t k=recv(fd,buf+n,399999-n,0); assert(k>=0); if(!k) break; n+=k; assert(n<399999); }
 close(fd); return buf;
}
int main(int argc,char **argv) {
 int fd=socket(AF_INET,SOCK_STREAM,0); assert(fd>=0);
 struct sockaddr_in a={.sin_family=AF_INET,.sin_addr={.s_addr=htonl(INADDR_LOOPBACK)}};
 assert(!bind(fd,(struct sockaddr *)&a,sizeof(a))); socklen_t len=sizeof(a);
 assert(!getsockname(fd,(struct sockaddr *)&a,&len)); unsigned port=ntohs(a.sin_port); close(fd);
 struct StereoPreview *p=stereo_preview_start(port,1); assert(p);
 struct TraceRing *ring=calloc(1,sizeof(*ring)); uint8_t *native=calloc(2,RAW),*support=calloc(2,64000);
 float *pred=calloc(2*N,sizeof(float)),*metric=calloc(2*N,sizeof(float)); assert(ring&&native&&support&&pred&&metric);
 struct StereoImage eyes[2]={{native,1280,1},{native+RAW,1280,1}};
 memset(native,25,RAW); memset(native+RAW,200,RAW); memset(support,1,128000);
 for(int i=0;i<2*N;++i) { pred[i]=(float)i/N; metric[i]=2; }
 int valid[2]={1,0}; uint64_t now=wall();
 struct PairTrace r={.id=9007199254740993ULL,.start=now-100000,.end=now,.input_us=now-90000,.success=1,.trace={.enabled=1}};
 trace_ring_push(ring,&r);
 char *s=request(port,"GET / HTTP/1.1\r\nHost: localhost\r\n\r\n"); assert(strstr(s,"1 Hz")); free(s);
 s=request(port,"GET /api/frame HTTP/1.1\r\nHost: localhost\r\n\r\n"); assert(strstr(s,"\"ready\":false")); free(s);
 assert(atomic_load(&p->wanted));
 pthread_mutex_lock(&p->lock); uint64_t before=wall();
 stereo_preview_publish(p,&r,ring,eyes,pred,metric,support,valid,0,NULL,0);
 assert(wall()-before<100000 && atomic_load(&p->skipped)==1 && atomic_load(&p->wanted)); pthread_mutex_unlock(&p->lock);
 stereo_preview_publish(p,&r,ring,eyes,pred,metric,support,valid,0,NULL,0); assert(p->have && !atomic_load(&p->wanted));
 /* Render directly to test pair identity, bitmap encoding and metrics without
  * clock-dependent rates. This mock has no sensor/NPU/device measurements. */
 atomic_store(&p->images.depth_deadline,(unsigned)(wall()/1000)+2500);
 atomic_store(&p->images.gray_deadline,(unsigned)(wall()/1000)+2500);
 struct StereoMatch matches[2]={ {.z={1.23,1.25}}, {.z={0,NAN}} };
 publish_images(p,&r,eyes,pred,metric,support,valid,0,matches,2);
 assert(p->images.gray_z_mm[0][0]==1230 && p->images.gray_z_mm[0][1]==1250);
 assert(!p->images.gray_z_mm[1][0] && !p->images.gray_z_mm[1][1]);
 assert(p->images.depth_id==r.id && p->images.depth_flags==1);
 assert(p->images.depth[0]==2 && p->images.depth[DEPTH_PIXELS]==pred[N+12*320]);
 assert(p->images.gray_id==r.id && p->images.gray[0]==25 && p->images.gray[RAW]==200);
 p->images.next_gray=wall()+1000000;
 struct PairTrace next=r; next.id++;
 publish_images(p,&next,eyes,pred,metric,support,valid,0,NULL,0);
 assert(p->images.depth_id==r.id+1 && p->images.gray_id==r.id);
 size_t n=0; s=render(p,&n,1); assert(s && n>250000);
 assert(strstr(s,"\"pair_id\":\"9007199254740993\"")); assert(strstr(s,"\"metric_valid\":[true,false]"));
 assert(strstr(s,"data:image/bmp;base64,Qk")); assert(strstr(s,"\"hardware_timings\":null"));
 if(argc==2) { FILE *f=fopen(argv[1],"w"); assert(f); assert(fwrite(s,1,n,f)==n); assert(!fclose(f)); }
 free(s);
 uint8_t *b=malloc(BMP_BYTES); assert(b); bmp(p,b);
 assert(b[54+199*320*3]==25 && b[54+(199*320+160)*3]==200); free(b);
 s=request(port,"GET /api/frame HTTP/1.1\r\nHost: localhost\r\n\r\n"); free(s); assert(p->rate_limited>=1);
 s=request(port,"POST /api/diagnostics/off HTTP/1.1\r\nHost: localhost\r\nContent-Length: 0\r\n\r\n"); assert(strstr(s,"\"enabled\":false")); free(s); assert(!stereo_preview_enabled(p));
 s=render(p,&n,1); assert(n<1000 && !strstr(s,"data:image")); free(s);
 r.trace.enabled=0; stereo_preview_publish(p,&r,ring,eyes,pred,metric,support,valid,0,NULL,0); assert(p->modes[0].pairs==1);
 s=request(port,"POST /api/diagnostics/on HTTP/1.1\r\nHost: localhost\r\nContent-Length: 0\r\n\r\n"); assert(strstr(s,"\"enabled\":true")); free(s); assert(stereo_preview_enabled(p));
 stereo_preview_stop(p); free(ring); free(native); free(support); free(pred); free(metric);
 puts("PASS mock preview: trigger ID, physical eyes, validity, BMP, demand/rate cap, nonblocking producer, HTTP controls and off baseline");
}
