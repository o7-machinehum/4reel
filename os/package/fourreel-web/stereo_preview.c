#define _POSIX_C_SOURCE 200809L
#include "stereo_preview.h"
#include "stereo_metric.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#ifndef STEREO_PREVIEW_PAGE
#define STEREO_PREVIEW_PAGE "/usr/share/4reel-stereo/preview.html"
#endif
#define RAW (1280*800)
#define N (320*224)
#define BMP_BYTES (54+320*200*3)
struct Mode { uint64_t pairs,elapsed,cpu,helper_cpu; };
struct StereoPreview {
 pthread_t thread; pthread_mutex_t lock;
 atomic_int stop,enabled,wanted;
 atomic_uint skipped;
 int fd,have,capture,valid[2];
 uint64_t last_request,preview_us,preview_cpu,http_us,http_cpu,requests,rate_limited;
 uint64_t snapshot_us,record_overhead_us;
 struct Mode modes[2],local_modes[2];
 struct PairTrace pair;
 struct TraceRing ring;
 uint8_t native[2*RAW],support[2*320*200];
 float depth[2*N];
 char *page,*cached; size_t page_size,cached_size;
};
static uint64_t wall(void) { return trace_clock(CLOCK_MONOTONIC); }
static uint64_t cpu(void) { return trace_clock(CLOCK_THREAD_CPUTIME_ID); }
int stereo_preview_enabled(struct StereoPreview *p) { return p && atomic_load(&p->enabled); }
static void le32(uint8_t *p,uint32_t x) { for(int i=0;i<4;++i) p[i]=x>>(8*i); }
static void bmp(struct StereoPreview *p,uint8_t *b) {
 memset(b,0,BMP_BYTES); b[0]='B'; b[1]='M'; le32(b+2,BMP_BYTES); le32(b+10,54);
 le32(b+14,40); le32(b+18,320); le32(b+22,200); b[26]=1; b[28]=24;
 float lo[2]={INFINITY,INFINITY},hi[2]={-INFINITY,-INFINITY};
 for(int e=0;e<2;++e) for(int y=0;y<200;y+=2) for(int x=0;x<320;x+=2) {
  float v=p->depth[e*N+(y+12)*320+x];
  if(isfinite(v) && (!p->valid[e] || v>0)) { if(v<lo[e]) lo[e]=v; if(v>hi[e]) hi[e]=v; }
 }
 for(int y=0;y<200;++y) for(int x=0;x<320;++x) {
  int e=x/160,px=x%160; uint8_t *d=b+54+((199-y)*320+x)*3;
  if(y<100) { d[0]=d[1]=d[2]=p->native[e*RAW+(y*8+3)*1280+px*8+3]; continue; }
  if(p->capture) continue;
  int iy=(y-100)*2,ix=px*2;
  float v=p->depth[e*N+(iy+12)*320+ix];
  unsigned support=p->support[e*64000+iy*320+ix];
  if(!isfinite(v) || (p->valid[e] && (!support || v<=0))) continue;
  /* Relative independently normalised per eye; metric fixed 0.15..20m. */
  float t=p->valid[e] ? (logf(fmaxf(.15f,v))-logf(.15f))/logf(20/.15f):
   (v-lo[e])/fmaxf(hi[e]-lo[e],1e-9f);
  t=fminf(1,fmaxf(0,t)); d[2]=(uint8_t)(255*t); d[1]=(uint8_t)(255*(1-fabsf(2*t-1))); d[0]=(uint8_t)(255*(1-t));
  if(p->valid[e] && support==2 && ((x+y)&3)==0) d[0]=d[1]=d[2]=180;
 }
}
static void base64(FILE *f,const uint8_t *b,size_t n) {
 static const char a[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
 for(size_t i=0;i<n;i+=3) {
  uint32_t v=(uint32_t)b[i]<<16; if(i+1<n) v|=(uint32_t)b[i+1]<<8; if(i+2<n) v|=b[i+2];
  fputc(a[v>>18],f); fputc(a[(v>>12)&63],f); fputc(i+1<n ? a[(v>>6)&63]:'=',f); fputc(i+2<n ? a[v&63]:'=',f);
 }
}
static double delta(uint64_t end,uint64_t begin) { return end>=begin && begin ? (end-begin)/1000.0:0; }
static void modes_json(FILE *f,const struct Mode *m) {
 fprintf(f,"{\"pairs\":%" PRIu64 ",\"elapsed_ms\":%.3f,\"pairs_s\":%.4f,\"main_cpu_ms_per_pair\":%.3f,\"helper_cpu_ms_per_pair\":%.3f}",
  m->pairs,m->elapsed/1000.,m->elapsed ? m->pairs*1e6/m->elapsed:0.,m->pairs ? m->cpu/1000./m->pairs:0.,m->pairs ? m->helper_cpu/1000./m->pairs:0.);
}
static char *render(struct StereoPreview *p,size_t *size) {
 uint64_t start=wall(),c=cpu();
 FILE *f; char *data=NULL; f=open_memstream(&data,size); if(!f) return NULL;
 pthread_mutex_lock(&p->lock); /* Only HTTP waits; producer uses trylock. */
 if(!atomic_load(&p->enabled)) {
  fputs("{\"enabled\":false,\"ready\":false,\"diagnostic_work\":{\"detail\":\"disabled; minimal comparison counters only\"},\"modes\":{\"off\":",f);
  modes_json(f,&p->modes[0]); fputs(",\"on\":",f); modes_json(f,&p->modes[1]); fputs("}}\n",f);
  pthread_mutex_unlock(&p->lock);
  if(fclose(f)) { free(data); data=NULL; }
  p->preview_us=wall()-start; p->preview_cpu=cpu()-c; return data;
 }
 const struct PairTrace *r=&p->pair;
 fprintf(f,"{\"enabled\":%s,\"ready\":%s,\"pair_id\":\"%" PRIu64 "\",\"sample_end_us\":%" PRIu64 ",\"server_now_us\":%" PRIu64 ",\"input_us\":%" PRIu64 ",",
  atomic_load(&p->enabled) ? "true":"false",p->have ? "true":"false",r->id,r->end,start,r->input_us);
 fprintf(f,"\"metric_valid\":[%s,%s],\"capture_only\":%s,\"hardware_timings\":null,\"rve\":\"unused: CPU sparse fallback\",",
  p->valid[0] ? "true":"false",p->valid[1] ? "true":"false",p->capture ? "true":"false");
 fprintf(f,"\"pair_ms\":%.3f,\"input_age_ms\":%.3f,\"first_capture_ms\":%.3f,\"complete_capture_ms\":%.3f,",
  delta(r->end,r->start),delta(start,r->input_us),delta(r->first_capture_us,r->trigger_us),delta(r->complete_capture_us,r->trigger_us));
 if(r->first_eye_us && r->complete_inference_us)
  fprintf(f,"\"first_eye_ms\":%.3f,\"complete_inference_ms\":%.3f,",delta(r->first_eye_us,r->trigger_us),delta(r->complete_inference_us,r->trigger_us));
 else fputs("\"first_eye_ms\":null,\"complete_inference_ms\":null,",f);
 fprintf(f,"\"main_cpu_ms\":%.3f,\"helper_cpu_ms\":%.3f,\"submit_queue_wait_ms\":%.3f,\"receive_wait_ms\":%.3f,\"sequence_gaps\":[%u,%u],\"snapshot_skips\":%u,\"trace_overflow\":%u,",
  r->main_cpu_us/1000.,r->helper_cpu_us/1000.,r->submit_wait_us/1000.,r->receive_wait_us/1000.,r->sequence_gap[0],r->sequence_gap[1],atomic_load(&p->skipped),r->trace.overflow);
 uint64_t earliest=UINT64_MAX,latest=0; unsigned completed=0; uint64_t gaps[2]={0,0};
 for(unsigned i=0;i<p->ring.count;++i) { const struct PairTrace *q=&p->ring.pair[i];
  if(q->start<earliest) earliest=q->start;
  if(q->end>latest) latest=q->end;
  completed+=q->success; gaps[0]+=q->sequence_gap[0]; gaps[1]+=q->sequence_gap[1];
 }
 fprintf(f,"\"rolling_pairs\":%u,\"rolling_pairs_s\":%.4f,\"rolling_sequence_gaps\":[%" PRIu64 ",%" PRIu64 "],\"queue\":{\"inflight_max\":1,\"pending_pair_queue\":0,\"policy\":\"trigger next pair after completion\",\"unobserved_sensor_drops\":null},",
  p->ring.count,latest>earliest ? completed*1e6/(latest-earliest):0.,gaps[0],gaps[1]);
 fprintf(f,"\"modes\":{\"off\":"); modes_json(f,&p->modes[0]); fputs(",\"on\":",f); modes_json(f,&p->modes[1]); fputs("},\"stages\":[",f);
 for(unsigned s=0;s<TS_COUNT;++s) {
  uint64_t duration=0,thread=0; unsigned calls=0,n=0;
  for(unsigned j=0;j<r->trace.count;++j) if(r->trace.event[j].stage==s) {
   duration+=r->trace.event[j].end-r->trace.event[j].begin; thread+=r->trace.event[j].cpu; ++calls;
  }
  uint64_t p50=trace_percentile(&p->ring,s,50,&n),p95=trace_percentile(&p->ring,s,95,NULL);
  fprintf(f,"%s{\"name\":\"%s\",\"calls\":%u,\"ms\":%.3f,\"cpu_ms\":%.3f,\"samples\":%u,\"p50_ms\":%.3f,\"p95_ms\":%.3f}",s ? ",":"",trace_stage_name(s),calls,duration/1000.,thread/1000.,n,p50/1000.,p95/1000.);
 }
 fputs("],\"events\":[",f);
 for(unsigned i=0;i<r->trace.count;++i) { const struct TraceEvent *e=&r->trace.event[i];
  fprintf(f,"%s{\"stage\":%u,\"resource\":%u,\"start_ms\":%.3f,\"end_ms\":%.3f,\"cpu_ms\":%.3f}",i ? ",":"",e->stage,e->resource,delta(e->begin,r->start),delta(e->end,r->start),e->cpu/1000.);
 }
 fputs("],\"host_call_occupancy_ms\":[",f);
 for(unsigned i=0;i<TR_RESOURCES;++i) fprintf(f,"%s%.3f",i ? ",":"",trace_union_us(r,i)/1000.);
 fprintf(f,"],\"diagnostic_work\":{\"previous_preview_ms\":%.3f,\"previous_preview_cpu_ms\":%.3f,\"previous_http_ms\":%.3f,\"previous_http_cpu_ms\":%.3f,\"snapshot_ms\":%.3f,\"record_us\":%" PRIu64 ",\"requests\":%" PRIu64 ",\"rate_limited\":%" PRIu64 "},",
  p->preview_us/1000.,p->preview_cpu/1000.,p->http_us/1000.,p->http_cpu/1000.,p->snapshot_us/1000.,p->record_overhead_us,p->requests,p->rate_limited);
 fputs("\"bmp\":",f);
 if(p->have && atomic_load(&p->enabled)) {
  uint8_t *b=malloc(BMP_BYTES);
  if(b) { bmp(p,b); fputs("\"data:image/bmp;base64,",f); base64(f,b,BMP_BYTES); fputc('"',f); free(b); }
  else fputs("null",f);
 } else fputs("null",f);
 fputs("}\n",f); pthread_mutex_unlock(&p->lock);
 int bad=ferror(f); if(fclose(f) || bad) { free(data); data=NULL; }
 p->preview_us=wall()-start; p->preview_cpu=cpu()-c; return data;
}
static int send_all(struct StereoPreview *p,int fd,const char *s,size_t n) {
 uint64_t deadline=wall()+2000000;
 while(n && !atomic_load(&p->stop) && wall()<deadline) {
  struct pollfd f={fd,POLLOUT,0}; int r=poll(&f,1,50); if(r<0 && errno==EINTR) continue;
  if(r<0) return -1;
  if(!r) continue;
  ssize_t k=send(fd,s,n,MSG_NOSIGNAL);
  if(k<0 && (errno==EAGAIN || errno==EINTR)) continue;
  if(k<=0) return -1;
  s+=k; n-=k;
 }
 return n ? -1:0;
}
static void reply(struct StereoPreview *p,int fd,int status,const char *type,const char *body,size_t n) {
 char h[320]; int k=snprintf(h,sizeof(h),"HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\nCache-Control: no-store\r\nConnection: close\r\nX-Content-Type-Options: nosniff\r\n\r\n",status,status==200 ? "OK":"Unavailable",type,n);
 if(!send_all(p,fd,h,(size_t)k)) send_all(p,fd,body,n);
}
static void *serve(void *arg) {
 struct StereoPreview *p=arg;
 while(!atomic_load(&p->stop)) {
  struct pollfd f={p->fd,POLLIN,0}; if(poll(&f,1,100)<=0) continue;
  int fd=accept(p->fd,NULL,NULL); if(fd<0) continue; fcntl(fd,F_SETFL,O_NONBLOCK);
  uint64_t start=wall(),c=cpu(); char request[2048]={0}; size_t n=0;
  while(n<sizeof(request)-1 && wall()-start<500000 && !atomic_load(&p->stop)) {
   f=(struct pollfd){fd,POLLIN,0}; if(poll(&f,1,50)<=0) continue;
   ssize_t k=recv(fd,request+n,sizeof(request)-1-n,0);
   if(k<0 && (errno==EAGAIN || errno==EINTR)) continue;
   if(k<=0) break;
   n+=k; request[n]=0; if(strstr(request,"\r\n\r\n")) break;
  }
  ++p->requests;
  if(!strncmp(request,"GET / HTTP/1.",13)) reply(p,fd,200,"text/html; charset=utf-8",p->page,p->page_size);
  else if(!strncmp(request,"GET /api/frame HTTP/1.",21)) {
   uint64_t now=wall();
   if(now-p->last_request>=1000000) {
    p->last_request=now; if(atomic_load(&p->enabled)) atomic_store(&p->wanted,1);
    free(p->cached); p->cached=render(p,&p->cached_size);
   } else ++p->rate_limited;
   if(p->cached) reply(p,fd,200,"application/json",p->cached,p->cached_size);
   else reply(p,fd,503,"application/json","{\"ready\":false}",15);
  } else if(!strncmp(request,"POST /api/diagnostics/on HTTP/1.",31) || !strncmp(request,"POST /api/diagnostics/off HTTP/1.",32)) {
   int on=request[23]=='n'; /* exact path above, no arbitrary commands */
   atomic_store(&p->enabled,on); atomic_store(&p->wanted,on);
   p->last_request=0; /* A mode change must not return the previous mode cache. */
   reply(p,fd,200,"application/json",on ? "{\"enabled\":true}":"{\"enabled\":false}",on ? 16:17);
  } else reply(p,fd,404,"text/plain","Not found\n",10);
  close(fd); p->http_us=wall()-start; p->http_cpu=cpu()-c;
 }
 return NULL;
}
struct StereoPreview *stereo_preview_start(unsigned port,int enabled) {
 if(!port) return NULL;
 struct StereoPreview *p=calloc(1,sizeof(*p)); if(!p) return NULL;
 p->fd=-1; atomic_init(&p->stop,0); atomic_init(&p->enabled,enabled); atomic_init(&p->wanted,0); atomic_init(&p->skipped,0);
 if(pthread_mutex_init(&p->lock,NULL)) { free(p); return NULL; }
 FILE *f=fopen(STEREO_PREVIEW_PAGE,"rb");
 if(!f) goto fail;
 p->page=malloc(32768); if(!p->page) { fclose(f); goto fail; }
 p->page_size=fread(p->page,1,32768,f); int bad=ferror(f) || !feof(f); fclose(f); if(bad) goto fail;
 p->fd=socket(AF_INET,SOCK_STREAM,0); if(p->fd<0) goto fail;
 int one=1; setsockopt(p->fd,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
 fcntl(p->fd,F_SETFD,FD_CLOEXEC); fcntl(p->fd,F_SETFL,O_NONBLOCK);
 struct sockaddr_in addr={.sin_family=AF_INET,.sin_port=htons(port),.sin_addr={.s_addr=htonl(INADDR_ANY)}};
 if(bind(p->fd,(struct sockaddr *)&addr,sizeof(addr)) || listen(p->fd,2)) goto fail;
 if(pthread_create(&p->thread,NULL,serve,p)) goto fail;
 return p;
fail:
 if(p->fd>=0) close(p->fd);
 pthread_mutex_destroy(&p->lock); free(p->page); free(p); return NULL;
}
void stereo_preview_publish(struct StereoPreview *p,struct PairTrace *r,const struct TraceRing *ring,
 const uint8_t *native,const float *prediction,const float *metric,const uint8_t *support,const int valid[2],int capture) {
 if(!p) return;
 uint64_t entry_cpu=cpu(),entry_wall=wall();
 struct Mode *m=&p->local_modes[!!r->trace.enabled];
 ++m->pairs; m->elapsed+=r->end-r->start; m->cpu+=r->main_cpu_us; m->helper_cpu+=r->helper_cpu_us;
 if(pthread_mutex_trylock(&p->lock)) { if(atomic_load(&p->wanted)) atomic_fetch_add(&p->skipped,1); return; }
 memcpy(p->modes,p->local_modes,sizeof(p->modes));
 if(atomic_load(&p->wanted) && r->trace.enabled) {
  struct TraceMark t=trace_begin(&r->trace);
  memcpy(p->native,native,sizeof(p->native));
  for(int e=0;e<2;++e) {
   p->valid[e]=valid[e];
   if(!capture) memcpy(p->depth+e*N,(valid[e] ? metric:prediction)+e*N,N*sizeof(float));
  }
  if(!capture) memcpy(p->support,support,sizeof(p->support));
  p->capture=capture; p->ring=*ring; p->have=1;
  trace_end(&r->trace,t,TS_SNAPSHOT,TR_MAIN);
  p->snapshot_us=wall()-t.wall;
  r->end=wall(); r->main_cpu_us+=cpu()-entry_cpu;
  p->pair=*r;
  for(unsigned j=0;j<r->trace.count;++j) if(r->trace.event[j].stage==TS_TELEMETRY)
   p->record_overhead_us=r->trace.event[j].end-r->trace.event[j].begin;
  if(p->ring.count) p->ring.pair[(p->ring.next+TRACE_RING-1)%TRACE_RING]=*r;
  atomic_store(&p->wanted,0);
 }
 m->elapsed+=wall()-entry_wall; m->cpu+=cpu()-entry_cpu;
 memcpy(p->modes,p->local_modes,sizeof(p->modes));
 pthread_mutex_unlock(&p->lock);
}
void stereo_preview_stop(struct StereoPreview *p) {
 if(!p) return;
 atomic_store(&p->stop,1); pthread_join(p->thread,NULL); close(p->fd);
 pthread_mutex_destroy(&p->lock); free(p->page); free(p->cached); free(p);
}
