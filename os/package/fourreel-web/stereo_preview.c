#define _POSIX_C_SOURCE 200809L
#include "stereo_preview.h"
#include "stereo_metric.h"
#include "stereo_sparse.h"
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
/* The HTTP thread serves depth and telemetry. Native grayscale has its own
 * sender so a 2 MiB transfer cannot hold up a small depth request. */
#define DEPTH_PIXELS (320 * 200)
#define DEPTH_FLOAT_BYTES (2 * DEPTH_PIXELS * sizeof(float))
struct LiveImages {
 pthread_mutex_t depth_lock,gray_lock,job_lock;
 pthread_cond_t job_ready;
 pthread_t gray_worker;
 int gray_fd,gray_flipped,client_flips;
 atomic_uint depth_deadline,gray_deadline;
 uint64_t depth_id,gray_id,next_gray;
 unsigned depth_flags,match_count;
 struct SparseStats depth_scale;
 uint16_t gray_matches[STEREO_MAX_ANCHORS][4];
 uint16_t gray_z_mm[STEREO_MAX_ANCHORS][2];
 uint8_t gray[2 * RAW];
 float depth[2 * DEPTH_PIXELS];
 uint8_t mask[2 * DEPTH_PIXELS];
 /* Private to the HTTP thread once copied out of the producer's slot. */
 uint8_t outgoing[DEPTH_FLOAT_BYTES + 2 * DEPTH_PIXELS];
};

struct Mode { uint64_t pairs,elapsed,cpu,helper_cpu; };
struct StereoPreview {
 struct LiveImages images;
 pthread_t thread; pthread_mutex_t lock;
 atomic_int stop,enabled,wanted;
 atomic_uint skipped;
 int fd,have,capture,valid[2],cached_images,bmp_flipped;
 uint64_t image_id,last_request,preview_us,preview_cpu,http_us,http_cpu,requests,rate_limited;
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
  if(y<100) { d[0]=d[1]=d[2]=p->native[e*RAW+(y*8+3)*1280+(p->bmp_flipped ? 1279-(px*8+3):px*8+3)]; continue; }
  if(p->capture) continue;
  int iy=(y-100)*2,ix=px*2;
  float v=p->depth[e*N+(iy+12)*320+ix];
  unsigned support=p->support[e*64000+iy*320+ix];
  if(!isfinite(v) || (p->valid[e] && (!support || v<=0))) continue;
  /* Relative independently normalised per eye; metric fixed 0.30..20m. */
  float t=p->valid[e] ? 1-(logf(fmaxf(.30f,v))-logf(.30f))/logf(20/.30f):
   (v-lo[e])/fmaxf(hi[e]-lo[e],1e-9f);
  t=fminf(1,fmaxf(0,t)); d[2]=(uint8_t)(255*t); d[1]=(uint8_t)(255*(1-fabsf(2*t-1))); d[0]=(uint8_t)(255*(1-t));
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
static char *render(struct StereoPreview *p,size_t *size,int with_images) {
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
  atomic_load(&p->enabled) ? "true":"false",p->have && (!with_images || p->image_id==r->id) ? "true":"false",r->id,r->end,start,r->input_us);
 fprintf(f,"\"metric_valid\":[%s,%s],\"capture_only\":%s,\"hardware_timings\":null,\"rve\":\"unused: CPU sparse fallback\",",
  p->valid[0] ? "true":"false",p->valid[1] ? "true":"false",p->capture ? "true":"false");
 const struct ExposureStats *ae=&r->exposure;
 fprintf(f,"\"auto_exposure\":{\"enabled\":%s,\"lines\":%d,\"gain\":%d,\"next_lines\":%d,\"next_gain\":%d,\"p75\":[%d,%d],\"clipped_pct\":[%.1f,%.1f]},",
  ae->enabled?"true":"false",ae->lines,ae->gain,ae->next_lines,ae->next_gain,
  ae->p75[0],ae->p75[1],ae->clip_permille[0]/10.,ae->clip_permille[1]/10.);
 const struct SparseStats *s=&r->sparse;
 fprintf(f,"\"stereo\":{\"mode\":\"%s\",\"candidates\":%u,\"forward\":%u,\"mutual\":%u,\"anchors\":%u,\"fit_inliers\":[%u,%u],\"scale_inliers\":%u,\"factor\":%.9g,\"scale_source_pair\":\"%" PRIu64 "\",\"scale_age_ms\":%.3f},",
  p->capture ? "off":s->calibrated ? "calibrated":"image-only",s->candidates,s->forward,s->mutual,s->anchors,
  s->inliers,s->inliers,s->inliers,s->factor,s->source_pair,s->scale_age_us/1000.);
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
 if(with_images && p->have && p->image_id==r->id && atomic_load(&p->enabled)) {
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
static int header(struct StereoPreview *p,int fd,uint64_t id,unsigned flags,size_t bytes,int flip,const struct LiveImages *gray,const struct SparseStats *scale) {
 char buf[256];
 int n=snprintf(buf,sizeof(buf),"HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\nContent-Length: %zu\r\nCache-Control: no-store\r\nConnection: close\r\nX-Pair-Id: %" PRIu64 "\r\nX-Metric-Eyes: %u\r\nX-Horizontal-Flip: %d\r\n",bytes,id,flags,flip);
 if(send_all(p,fd,buf,(size_t)n)) return -1;
 if(scale) {
  n=snprintf(buf,sizeof(buf),"X-Stereo-Scale: %.9g\r\nX-Scale-Source-Pair: %" PRIu64 "\r\nX-Scale-Age-Ms: %.3f\r\nX-Scale-Inliers: %u\r\n",scale->factor,scale->source_pair,scale->scale_age_us/1000.,scale->inliers);
  if(send_all(p,fd,buf,(size_t)n)) return -1;
 }
 if(gray) {
  char points[32*STEREO_MAX_ANCHORS+32]; size_t used=0;
  for(unsigned i=0;i<gray->match_count;i++) {
   const uint16_t *q=gray->gray_matches[i];
   used+=(size_t)snprintf(points+used,sizeof(points)-used,"%s%u,%u,%u,%u",i?" ":"X-Stereo-Matches: ",q[0],q[1],q[2],q[3]);
  }
  if(used) { points[used++]='\r'; points[used++]='\n'; if(send_all(p,fd,points,used)) return -1; }
  used=0;
  for(unsigned i=0;i<gray->match_count;i++) {
   const uint16_t *z=gray->gray_z_mm[i];
   used+=(size_t)snprintf(points+used,sizeof(points)-used,"%s%u,%u",i?" ":"X-Stereo-Z-mm: ",z[0],z[1]);
  }
  if(used) { points[used++]='\r'; points[used++]='\n'; if(send_all(p,fd,points,used)) return -1; }
 }
 return send_all(p,fd,"\r\n",2);
}
static void no_image(struct StereoPreview *p,int fd) {
 static const char response[]="HTTP/1.1 204 No Content\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
 send_all(p,fd,response,sizeof(response)-1);
}
static void serve_depth(struct StereoPreview *p,int fd,uint64_t after) {
 struct LiveImages *v=&p->images;
 atomic_store(&v->depth_deadline,(unsigned)(wall()/1000)+2500);
 pthread_mutex_lock(&v->depth_lock);
 uint64_t id=v->depth_id;
 unsigned flags=v->depth_flags;
 struct SparseStats scale=v->depth_scale;
 size_t n=DEPTH_FLOAT_BYTES+(flags ? sizeof(v->mask):0);
 int fresh=id && id!=after && atomic_load(&p->enabled);
 if(fresh) {
  memcpy(v->outgoing,v->depth,DEPTH_FLOAT_BYTES);
  if(flags) memcpy(v->outgoing+DEPTH_FLOAT_BYTES,v->mask,sizeof(v->mask));
 }
 pthread_mutex_unlock(&v->depth_lock);
 if(!fresh) no_image(p,fd);
 else if(!header(p,fd,id,flags,n,0,NULL,&scale)) send_all(p,fd,(char *)v->outgoing,n);
}
/* Older clients request already-oriented grayscale. The dashboard mirrors in
 * the browser, so its 1 Hz transfer needs no pixel reversal on the board. */
static void send_gray_rows(struct StereoPreview *p,int fd,struct LiveImages *v) {
 if(v->gray_flipped && !v->client_flips) {
  for(int y=0;y<2*STEREO_H;y++) {
   uint8_t *row=v->gray+y*STEREO_W;
   for(int x=0;x<STEREO_W/2;x++) {
    uint8_t tmp=row[x];
    row[x]=row[STEREO_W-1-x];
    row[STEREO_W-1-x]=tmp;
   }
  }
  v->gray_flipped=0;
 }
 if(!header(p,fd,v->gray_id,0,sizeof(v->gray),v->gray_flipped,v,NULL))
  send_all(p,fd,(const char *)v->gray,sizeof(v->gray));
}
static void *send_gray(void *arg) {
 struct StereoPreview *p=arg; struct LiveImages *v=&p->images;
 pthread_mutex_lock(&v->job_lock);
 while(!atomic_load(&p->stop)) {
  while(v->gray_fd<0 && !atomic_load(&p->stop)) pthread_cond_wait(&v->job_ready,&v->job_lock);
  if(v->gray_fd<0) break;
  int fd=v->gray_fd;
  pthread_mutex_unlock(&v->job_lock);
  pthread_mutex_lock(&v->gray_lock);
  if(v->gray_id && atomic_load(&p->enabled)) {
   send_gray_rows(p,fd,v);
  } else no_image(p,fd);
  pthread_mutex_unlock(&v->gray_lock);
  close(fd);
  pthread_mutex_lock(&v->job_lock);
  v->gray_fd=-1;
 }
 if(v->gray_fd>=0) close(v->gray_fd);
 pthread_mutex_unlock(&v->job_lock); return NULL;
}
static int dispatch_gray(struct StereoPreview *p,int fd,uint64_t after,int client_flips) {
 struct LiveImages *v=&p->images;
 atomic_store(&v->gray_deadline,(unsigned)(wall()/1000)+2500);
 pthread_mutex_lock(&v->job_lock);
 int ready=0;
 if(v->gray_fd<0 && atomic_load(&p->enabled) && !pthread_mutex_trylock(&v->gray_lock)) {
  ready=v->gray_id && v->gray_id!=after;
  pthread_mutex_unlock(&v->gray_lock);
 }
 if(ready) { v->client_flips=client_flips; v->gray_fd=fd; pthread_cond_signal(&v->job_ready); }
 pthread_mutex_unlock(&v->job_lock);
 if(!ready) no_image(p,fd);
 return ready;
}
static int viewed(atomic_uint *deadline,uint64_t now) {
 unsigned limit=atomic_load(deadline);
 return limit && (int32_t)(limit-(unsigned)(now/1000))>0;
}
/* Copy only a requested preview, preserving sensor row order and discarding
 * DMA row padding. The inference path never needs this copy. */
static void snapshot_gray(uint8_t *out,const struct StereoImage eyes[2]) {
 for(int e=0;e<2;e++) {
  const uint8_t *start=eyes[e].origin-(eyes[e].xstep<0 ? STEREO_W-1:0);
  if(eyes[e].stride==STEREO_W) memcpy(out+e*RAW,start,RAW);
  else for(int y=0;y<STEREO_H;y++) memcpy(out+e*RAW+y*STEREO_W,start+y*eyes[e].stride,STEREO_W);
 }
}
static void publish_images(struct StereoPreview *p,const struct PairTrace *r,const struct StereoImage native[2],
 const float *prediction,const float *metric,const uint8_t *support,const int valid[2],int capture,
 const struct StereoMatch *matches,unsigned match_count) {
 struct LiveImages *v=&p->images;
 uint64_t now=wall();
 if(!capture && viewed(&v->depth_deadline,now) && !pthread_mutex_trylock(&v->depth_lock)) {
  v->depth_flags=0;
  for(unsigned e=0;e<2;e++) {
   const float *src=(valid[e] ? metric:prediction)+e*N+12*320;
   memcpy(v->depth+e*DEPTH_PIXELS,src,DEPTH_PIXELS*sizeof(float));
   if(valid[e]) v->depth_flags|=1u<<e;
  }
  if(v->depth_flags) memcpy(v->mask,support,sizeof(v->mask));
  v->depth_scale=r->sparse; v->depth_id=r->id;
  pthread_mutex_unlock(&v->depth_lock);
 }
 if(viewed(&v->gray_deadline,now) && now>=v->next_gray && !pthread_mutex_trylock(&v->gray_lock)) {
  snapshot_gray(v->gray,native); v->gray_flipped=native[0].xstep<0; v->gray_id=r->id;
  v->match_count=match_count<STEREO_MAX_ANCHORS ? match_count:STEREO_MAX_ANCHORS;
  for(unsigned i=0;i<v->match_count;i++) for(unsigned e=0;e<2;e++) {
   for(unsigned j=0;j<2;j++) v->gray_matches[i][2*e+j]=(uint16_t)(matches[i].xy[e][j]+0.5);
   double z=matches[i].z[e];
   v->gray_z_mm[i][e]=(z>0 && z<=20) ? (uint16_t)(z*1000+0.5):0;
  }
  v->next_gray=(now/1000000+1)*1000000;
  pthread_mutex_unlock(&v->gray_lock);
 }
}
static int after_id(const char *request,const char *prefix,uint64_t *id) {
 size_t n=strlen(prefix);
 if(strncmp(request,prefix,n) || request[n]<'0' || request[n]>'9') return 0;
 char *end; errno=0; unsigned long long value=strtoull(request+n,&end,10);
 if(errno || strncmp(end," HTTP/1.",8)) return 0;
 *id=(uint64_t)value; return 1;
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
  ++p->requests; uint64_t after; int handed_off=0;
  if(!strncmp(request,"GET / HTTP/1.",13)) reply(p,fd,200,"text/html; charset=utf-8",p->page,p->page_size);
  else if(after_id(request,"GET /api/depth?after=",&after)) serve_depth(p,fd,after);
  else if(after_id(request,"GET /api/gray?after=",&after)) handed_off=dispatch_gray(p,fd,after,0);
  else if(after_id(request,"GET /api/gray?native=1&after=",&after)) handed_off=dispatch_gray(p,fd,after,1);
  else if(!strncmp(request,"GET /api/frame HTTP/1.",21) || !strncmp(request,"GET /api/stats HTTP/1.",21)) {
   int with_images=request[9]=='f';
   uint64_t now=wall();
   if(now-p->last_request>=1000000 || p->cached_images!=with_images) {
    p->last_request=now; p->cached_images=with_images; if(atomic_load(&p->enabled)) atomic_store(&p->wanted,with_images ? 2:1);
    free(p->cached); p->cached=render(p,&p->cached_size,with_images);
   } else ++p->rate_limited;
   if(p->cached) reply(p,fd,200,"application/json",p->cached,p->cached_size);
   else reply(p,fd,503,"application/json","{\"ready\":false}",15);
  } else if(!strncmp(request,"POST /api/diagnostics/on HTTP/1.",31) || !strncmp(request,"POST /api/diagnostics/off HTTP/1.",32)) {
   int on=request[23]=='n'; /* exact path above, no arbitrary commands */
   atomic_store(&p->enabled,on); atomic_store(&p->wanted,on);
   p->last_request=0; /* A mode change must not return the previous mode cache. */
   reply(p,fd,200,"application/json",on ? "{\"enabled\":true}":"{\"enabled\":false}",on ? 16:17);
  } else reply(p,fd,404,"text/plain","Not found\n",10);
  if(!handed_off) close(fd);
  p->http_us=wall()-start; p->http_cpu=cpu()-c;
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
 struct LiveImages *v=&p->images;
 v->gray_fd=-1;
 atomic_init(&v->depth_deadline,0); atomic_init(&v->gray_deadline,0);
 if(pthread_mutex_init(&v->depth_lock,NULL)) goto fail;
 if(pthread_mutex_init(&v->gray_lock,NULL)) goto fail_depth;
 if(pthread_mutex_init(&v->job_lock,NULL)) goto fail_gray;
 if(pthread_cond_init(&v->job_ready,NULL)) goto fail_job;
 if(pthread_create(&v->gray_worker,NULL,send_gray,p)) goto fail_cond;
 if(pthread_create(&p->thread,NULL,serve,p)) {
  pthread_mutex_lock(&v->job_lock); atomic_store(&p->stop,1); pthread_cond_signal(&v->job_ready);
  pthread_mutex_unlock(&v->job_lock); pthread_join(v->gray_worker,NULL); goto fail_cond;
 }
 return p;
fail_cond: pthread_cond_destroy(&v->job_ready);
fail_job: pthread_mutex_destroy(&v->job_lock);
fail_gray: pthread_mutex_destroy(&v->gray_lock);
fail_depth: pthread_mutex_destroy(&v->depth_lock);
fail:
 if(p->fd>=0) close(p->fd);
 pthread_mutex_destroy(&p->lock); free(p->page); free(p); return NULL;
}
void stereo_preview_publish(struct StereoPreview *p,struct PairTrace *r,const struct TraceRing *ring,
 const struct StereoImage native[2],const float *prediction,const float *metric,const uint8_t *support,const int valid[2],int capture,
 const struct StereoMatch *matches,unsigned match_count) {
 if(!p) return;
 uint64_t entry_cpu=cpu(),entry_wall=wall();
 if(r->trace.enabled) publish_images(p,r,native,prediction,metric,support,valid,capture,matches,match_count);
 struct Mode *m=&p->local_modes[!!r->trace.enabled];
 ++m->pairs; m->elapsed+=r->end-r->start; m->cpu+=r->main_cpu_us; m->helper_cpu+=r->helper_cpu_us;
 if(pthread_mutex_trylock(&p->lock)) { if(atomic_load(&p->wanted)) atomic_fetch_add(&p->skipped,1); return; }
 memcpy(p->modes,p->local_modes,sizeof(p->modes));
 if(atomic_load(&p->wanted) && r->trace.enabled) {
  struct TraceMark t=trace_begin(&r->trace);
  int with_images=atomic_load(&p->wanted)==2;
  if(with_images) { snapshot_gray(p->native,native); p->bmp_flipped=native[0].xstep<0; p->image_id=r->id; }
  for(int e=0;e<2;++e) {
   p->valid[e]=valid[e];
   if(with_images && !capture) memcpy(p->depth+e*N,(valid[e] ? metric:prediction)+e*N,N*sizeof(float));
  }
  if(with_images && !capture) memcpy(p->support,support,sizeof(p->support));
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
 struct LiveImages *v=&p->images;
 atomic_store(&p->stop,1); pthread_join(p->thread,NULL); close(p->fd);
 pthread_mutex_lock(&v->job_lock); pthread_cond_signal(&v->job_ready); pthread_mutex_unlock(&v->job_lock);
 pthread_join(v->gray_worker,NULL); pthread_cond_destroy(&v->job_ready);
 pthread_mutex_destroy(&v->job_lock); pthread_mutex_destroy(&v->gray_lock); pthread_mutex_destroy(&v->depth_lock);
 pthread_mutex_destroy(&p->lock); free(p->page); free(p->cached); free(p);
}
