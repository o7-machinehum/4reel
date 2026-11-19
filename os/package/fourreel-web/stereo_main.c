#define _POSIX_C_SOURCE 200809L
#include "stereo_capture.h"
#include "stereo_sparse.h"
#include "stereo_preview.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define RAW (STEREO_W*STEREO_H)
#define N (TENSOR_W*TENSOR_H)
static volatile sig_atomic_t stop;
static void stopping(int s) { (void)s; stop=1; }
static uint64_t now_us(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return (uint64_t)t.tv_sec*1000000+t.tv_nsec/1000; }
static int io_all(int fd,void *p,size_t n,int writing,uint64_t *wait_us)
{
    uint8_t *b=p; uint64_t deadline=now_us()+10000000;
    while(n && !stop) {
        struct pollfd f={.fd=fd,.events=writing ? POLLOUT:POLLIN};
        uint64_t before=wait_us ? now_us():0;
        int ret=poll(&f,1,100);
        if(wait_us) *wait_us+=now_us()-before;
        if(ret<0 && errno==EINTR) continue;
        if(ret<0 || now_us()>deadline) return -1;
        if(!ret) continue;
        ssize_t k=writing ? write(fd,b,n>4096 ? 4096:n):read(fd,b,n>4096 ? 4096:n);
        if(k<0 && (errno==EINTR || errno==EAGAIN)) continue;
        if(k<=0) return -1;
        b+=k; n-=k;
    }
    return n ? -1:0;
}
static int valid_unit(const char *s)
{
    size_t n=strlen(s);
    if(!n || n>63) return 0;
    for(size_t i=0;i<n;++i) if(!((s[i]>='a' && s[i]<='z') || (s[i]>='A' && s[i]<='Z') ||
        (s[i]>='0' && s[i]<='9') || s[i]=='_' || s[i]=='-' || s[i]=='.')) return 0;
    return 1;
}
static int pgm(const char *path,const uint8_t *image,int w,int h)
{
    FILE *f=fopen(path,"wx"); if(!f) return -1;
    int ok=fprintf(f,"P5\n%d %d\n255\n",w,h)>0 && fwrite(image,1,(size_t)w*h,f)==(size_t)w*h;
    if(fclose(f)) ok=0;
    return ok ? 0:-1;
}
static int pfm(const char *path,const float *image)
{
    FILE *f=fopen(path,"wx"); if(!f) return -1;
    int ok=fprintf(f,"Pf\n320 200\n-1.0\n")>0;
    /* PFM rows are bottom-up, IEEE little-endian floats. */
    for(int y=199;y>=0;--y) if(fwrite(image+(y+12)*320,4,320,f)!=320) ok=0;
    if(fclose(f)) ok=0;
    return ok ? 0:-1;
}
static double prediction_at(const float *p,double x,double y)
{
    x=(x+.5)/4-.5; y=(y+.5)/4-.5+12;
    int ix=(int)x,iy=(int)y;
    if(ix<0 || ix>=319 || iy<12 || iy>=211) return NAN;
    double a=x-ix,b=y-iy;
    return (1-b)*((1-a)*p[iy*320+ix]+a*p[iy*320+ix+1])+
           b*((1-a)*p[(iy+1)*320+ix]+a*p[(iy+1)*320+ix+1]);
}

int main(int argc,char **argv)
{
    const char *output=NULL,*unit=NULL,*calfile=NULL;
    unsigned count=1,port=8081; int capture=0,raw8=1,live=0,diagnostics=1;
    for(int i=1;i<argc;++i) {
        if(!strcmp(argv[i],"--capture")) capture=1;
        else if(!strcmp(argv[i],"--live")) live=1;
        else if(!strcmp(argv[i],"--diagnostics-off")) diagnostics=0;
        else if(!strcmp(argv[i],"--preview-port") && i+1<argc) {
            char *end; unsigned long n=strtoul(argv[++i],&end,10);
            if(*end || n>65535) return 2;
            port=(unsigned)n;
        }
        else if(!strcmp(argv[i],"--raw10")) raw8=0;
        else if(!strcmp(argv[i],"--output") && i+1<argc) output=argv[++i];
        else if(!strcmp(argv[i],"--unit") && i+1<argc) unit=argv[++i];
        else if(!strcmp(argv[i],"--calibration") && i+1<argc) calfile=argv[++i];
        else if(!strcmp(argv[i],"--count") && i+1<argc) {
            char *end; unsigned long n=strtoul(argv[++i],&end,10);
            if(*end || !n || n>10000) return 2;
            count=(unsigned)n;
        } else {
            puts("Usage: 4reel-stereo --unit SERIAL (--live | --output DIR) [--count 1..10000] [--capture] [--raw10] [--calibration FILE]\n--preview-port PORT (default 8081, 0 disables HTTP); --diagnostics-off for baseline.\n"
                 "Stop dashboard capture first. --capture records native synchronized calibration pairs without NPU.\n"
                 "Default: RGA + genuine ZipDepth; missing calibration yields relative maps only.\n"
                 "Metric maps are optical Z metres. mask 0=invalid 1=anchor-supported 2=extrapolated.\n"
                 "No hardware-verified accuracy or throughput is implied by this development build.");
            return !strcmp(argv[i],"--help") ? 0:2;
        }
    }
    if((!output && !live) || (output && live) || (capture && live) || !unit || !valid_unit(unit)) {
        fputs("Choose --live or --output DIR; --capture requires --output; valid --unit required\n",stderr); return 2;
    }
    struct StereoCalibration cal={0}; int calibrated=0;
    if(calfile) {
        if(stereo_calibration_load(calfile,unit,&cal)) { fputs("Invalid/mismatched unit calibration; refusing calibrated run\n",stderr); return 1; }
        calibrated=1;
    }
    if(!live && mkdir(output,0755) && errno!=EEXIST) { perror(output); return 1; }
    struct statvfs space;
    if(!live && (statvfs(output,&space) || (uint64_t)space.f_bavail*space.f_frsize < (uint64_t)count*(capture ? 2100000:1200000)+8*1024*1024)) {
        fputs("Insufficient output space; reduce --count or use a larger filesystem\n",stderr); return 1;
    }
    uint8_t *images=malloc(2*RAW),*masks=calloc(2,320*200);
    float *pred=malloc(2*N*sizeof(float)),*metrics=calloc(2*N,sizeof(float));
    struct TraceRing *ring=calloc(1,sizeof(*ring));
    struct StereoPreview *preview=NULL;
    if(!images || !pred || !metrics || !masks || !ring) return 1;
    int tx=-1,rx=-1; pid_t child=-1;
    int result=1;
    signal(SIGINT,stopping); signal(SIGTERM,stopping); signal(SIGPIPE,SIG_IGN);
    if(!capture) {
        int request[2],response[2];
        if(pipe(request) || pipe(response)) goto done;
        child=fork();
        if(child<0) goto done;
        if(!child) {
            if(dup2(request[0],0)<0 || dup2(response[1],1)<0) _exit(126);
            close(request[0]); close(request[1]); close(response[0]); close(response[1]);
            execl("/opt/4reel-rknn/bin/zipdepth-runner","zipdepth-runner",
                  "/usr/share/4reel-stereo/zipdepth-mono-rv1103.rknn",(char *)NULL);
            _exit(127);
        }
        close(request[0]); close(response[1]); tx=request[1]; rx=response[0];
        fcntl(tx,F_SETFL,O_NONBLOCK); fcntl(rx,F_SETFL,O_NONBLOCK);
    }
    if(port) {
        preview=stereo_preview_start(port,diagnostics);
        if(!preview) { perror("stereo preview server"); goto done; }
        fprintf(stderr,"Stereo preview: http://192.168.77.1:%u (1 Hz on demand)\n",port);
    }
    if(stereo_capture_open(raw8)) { perror("stereo capture/FSIN setup"); goto done; }
    int previous_mode=-1;
    for(unsigned index=0;(live || index<count) && !stop;++index) {
        uint64_t started=now_us(),cpu_start=trace_clock(CLOCK_THREAD_CPUTIME_ID);
        struct PairTrace record={.start=started,.trace={.enabled=preview ? stereo_preview_enabled(preview):diagnostics}};
        if(previous_mode!=(int)record.trace.enabled) { memset(ring,0,sizeof(*ring)); previous_mode=record.trace.enabled; }
        struct TraceMark timing=trace_begin(&record.trace);
        struct CapturePair pair;
        if(stereo_capture_pair_timed(images,images+RAW,&pair,&record.trace)) {
            record.id=pair.id; record.end=now_us(); trace_end(&record.trace,timing,TS_CAPTURE,TR_ENVELOPE);
            trace_ring_push(ring,&record);
            fprintf(stderr,"Failed pair %" PRIu64 "; capture/pairing %.3f ms; no result published\n",pair.id,(record.end-started)/1000.);
            perror("ambiguous/failed triggered pair"); goto done;
        }
        trace_end(&record.trace,timing,TS_CAPTURE,TR_ENVELOPE);
        record.id=pair.id; record.trigger_us=pair.trigger_before_us;
        record.input_us=(uint64_t)(pair.timestamp_us[0]<pair.timestamp_us[1] ? pair.timestamp_us[0]:pair.timestamp_us[1]);
        record.first_capture_us=pair.dequeued_us[0]<pair.dequeued_us[1] ? pair.dequeued_us[0]:pair.dequeued_us[1];
        record.complete_capture_us=pair.dequeued_us[0]>pair.dequeued_us[1] ? pair.dequeued_us[0]:pair.dequeued_us[1];
        memcpy(record.sequence_gap,pair.sequence_gap,sizeof(record.sequence_gap));
        struct StereoMatch matches[STEREO_MAX_ANCHORS]; unsigned anchors=0;
        struct StereoFit fits[2]={{0}}; struct StereoAnchor a[2][STEREO_MAX_ANCHORS];
        int valid[2]={0,0};
        uint64_t inference_started=now_us();
        if(!capture) {
            struct RunnerRequest request={.id=pair.id,.protocol=TRACE_PROTOCOL,.enabled=record.trace.enabled};
            timing=trace_begin(&record.trace);
            uint64_t *sw=record.trace.enabled ? &record.submit_wait_us:NULL;
            if(io_all(tx,&request,sizeof(request),1,sw) || io_all(tx,images,2*RAW,1,sw)) goto inference_error;
            trace_end(&record.trace,timing,TS_SUBMIT,TR_WAIT);
            /* CPU stereo overlaps the helper's RGA + sequential left/right NPU. */
            if(calibrated) anchors=stereo_sparse_timed(images,images+RAW,&cal,matches,&record.trace);
            struct RunnerResponse response;
            timing=trace_begin(&record.trace);
            uint64_t *rw=record.trace.enabled ? &record.receive_wait_us:NULL;
            if(io_all(rx,&response,sizeof(response),0,rw) || response.id!=pair.id ||
               response.trace.count>TRACE_EVENTS || io_all(rx,pred,2*N*sizeof(float),0,rw)) goto inference_error;
            trace_end(&record.trace,timing,TS_RECEIVE,TR_WAIT);
            record.first_eye_us=response.first_eye_us; record.complete_inference_us=response.complete_us;
            record.helper_cpu_us=response.cpu_us;
            for(unsigned i=0;i<response.trace.count;++i) {
                if(record.trace.count<TRACE_EVENTS) record.trace.event[record.trace.count++]=response.trace.event[i];
                else ++record.trace.overflow;
            }
            record.trace.overflow+=response.trace.overflow;
            if(calibrated) for(int e=0;e<2;++e) {
                timing=trace_begin(&record.trace);
                for(unsigned k=0;k<anchors;++k) a[e][k]=(struct StereoAnchor){
                    .x=matches[k].xy[e][0],.y=matches[k].xy[e][1],
                    .prediction=prediction_at(pred+e*N,matches[k].xy[e][0],matches[k].xy[e][1]),
                    .inverse_z=1/matches[k].z[e],.weight=matches[k].weight};
                valid[e]=calibrated && !stereo_fit(a[e],anchors,&fits[e]);
                trace_end(&record.trace,timing,e ? TS_FIT_R:TS_FIT_L,TR_MAIN);
            }
        }
        double processing_ms=(now_us()-inference_started)/1000.0;
        char directory[512],final[512],path[600];
        if(!live && (snprintf(directory,sizeof(directory),"%s/.pair-%" PRIu64,output,pair.id)>=(int)sizeof(directory) ||
           snprintf(final,sizeof(final),"%s/pair-%" PRIu64,output,pair.id)>=(int)sizeof(final))) goto done;
        if(!live && mkdir(directory,0755)) { perror(directory); goto done; }
        for(int e=0;e<2;++e) {
            float *metric=metrics+e*N; uint8_t *mask=masks+e*64000;
            const char *eye=e ? "right":"left";
            if(capture) {
                snprintf(path,sizeof(path),"%s/%s.pgm",directory,eye);
                timing=trace_begin(&record.trace);
                if(pgm(path,images+e*RAW,1280,800)) goto done;
                trace_end(&record.trace,timing,TS_FILES,TR_IO);
            } else {
                if(!live) {
                    timing=trace_begin(&record.trace);
                    snprintf(path,sizeof(path),"%s/%s-relative.pfm",directory,eye);
                    if(pfm(path,pred+e*N)) goto done;
                    trace_end(&record.trace,timing,TS_FILES,TR_IO);
                }
                timing=trace_begin(&record.trace);
                memset(metric,0,N*sizeof(float)); memset(mask,0,320*200);
                if(valid[e]) for(int y=0;y<200;++y) for(int x=0;x<320;++x) {
                    unsigned i=(y+12)*320+x;
                    double v=pred[e*N+i],inv=fits[e].scale*v+fits[e].shift;
                    double ray[3];
                    if(!isfinite(inv) || inv<0.05 || inv>1/0.15 ||
                       ds_ray(&cal.eye[e],4*x+1.5,4*y+1.5,ray) || ray[2]<=0) continue;
                    metric[i]=1/inv;
                    int support=stereo_support(a[e],anchors,&fits[e],4*x+1.5,4*y+1.5);
                    if(v<fits[e].min_prediction || v>fits[e].max_prediction) support=2;
                    mask[y*320+x]=support;
                }
                trace_end(&record.trace,timing,e ? TS_MAP_R:TS_MAP_L,TR_MAIN);
                if(!live) {
                    timing=trace_begin(&record.trace);
                    snprintf(path,sizeof(path),"%s/%s-z-metres.pfm",directory,eye);
                    if(pfm(path,metric)) goto done;
                    snprintf(path,sizeof(path),"%s/%s-support.pgm",directory,eye);
                    if(pgm(path,mask,320,200)) goto done;
                    trace_end(&record.trace,timing,TS_FILES,TR_IO);
                }
            }
        }
        if(!live) {
            timing=trace_begin(&record.trace);
            snprintf(path,sizeof(path),"%s/status.json",directory);
            FILE *f=fopen(path,"wx"); if(!f) goto done;
            int ok=fprintf(f,"{\n  \"unit\":\"%s\",\"pair_id\":%" PRIu64 ",\"trigger_identity\":\"one-PWM-burst-with-quiet-and-count-checks\",\n"
                "  \"trigger_before_us\":%" PRIu64 ",\"trigger_after_us\":%" PRIu64 ",\n"
                "  \"left_timestamp_us\":%" PRId64 ",\"right_timestamp_us\":%" PRId64 ",\"left_sequence\":%u,\"right_sequence\":%u,\n"
                "  \"left_v4l2_flags\":%u,\"right_v4l2_flags\":%u,\"sensor_format\":\"%s\",\n"
                "  \"orientation\":\"left=1,right=0,hflip=both,right_sensor_rotation=180\",\n"
                "  \"inference\":\"%s\",\"preprocess\":\"%s\",\"sparse\":\"%s\",\n"
                "  \"calibration_loaded\":%s,\"calibration_sha256\":\"%s\",\"anchors\":%u,\n"
                "  \"metric_valid\":[%s,%s],\"fit_confidence\":[%.6f,%.6f],\"inliers\":[%u,%u],\n"
                "  \"inverse_z_scale\":[%.9g,%.9g],\"inverse_z_shift\":[%.9g,%.9g],\"inverse_z_rms\":[%.9g,%.9g],\n"
                "  \"processing_ms\":%.3f,\"pair_work_ms\":%.3f,\"hardware_accuracy_validated\":false,\n"
                "  \"depth_convention\":\"optical_z_metres;0_invalid;mask1_anchor_hull;mask2_extrapolation\"\n}\n",
                unit,pair.id,pair.trigger_before_us,pair.trigger_after_us,pair.timestamp_us[0],pair.timestamp_us[1],pair.sequence[0],pair.sequence[1],
                pair.flags[0],pair.flags[1],raw8 ? "RAW8":"RAW10-top8",
                capture ? "disabled-calibration-capture":"rknn-2.5.0-rv1103-one-context",
                capture ? "native":"rga-gray-average-4x-pad12",
                calibrated && !capture ? "cpu-grid-corners-epipolar-pyramidal-lk":"disabled-no-calibration",
                calibrated ? "true":"false",calibrated ? cal.source_sha256:"",anchors,
                valid[0] ? "true":"false",valid[1] ? "true":"false",fits[0].confidence,fits[1].confidence,fits[0].inliers,fits[1].inliers,
                fits[0].scale,fits[1].scale,fits[0].shift,fits[1].shift,fits[0].rms,fits[1].rms,
                processing_ms,(now_us()-started)/1000.0)>0;
            if(fclose(f) || !ok || rename(directory,final)) goto done;
            trace_end(&record.trace,timing,TS_FILES,TR_IO);
            printf("%s\n",final); fflush(stdout);
        }
        record.end=now_us(); record.success=1;
        record.main_cpu_us=trace_clock(CLOCK_THREAD_CPUTIME_ID)-cpu_start;
        timing=trace_begin(&record.trace);
        if(record.trace.enabled) {
            trace_ring_push(ring,&record);
            trace_end(&record.trace,timing,TS_TELEMETRY,TR_MAIN);
            ring->pair[(ring->next+TRACE_RING-1)%TRACE_RING]=record;
        }
        record.end=now_us();
        record.main_cpu_us=trace_clock(CLOCK_THREAD_CPUTIME_ID)-cpu_start;
        stereo_preview_publish(preview,&record,ring,images,pred,metrics,masks,valid,capture);
        record.end=now_us(); record.main_cpu_us=trace_clock(CLOCK_THREAD_CPUTIME_ID)-cpu_start;
        if(record.trace.enabled) ring->pair[(ring->next+TRACE_RING-1)%TRACE_RING]=record;
        continue;
inference_error:
        record.end=now_us(); trace_ring_push(ring,&record);
        fputs("Genuine RGA/RKNN inference failed; no relative or metric result published for this pair\n",stderr);
        goto done;
    }
    result=stop ? 1:0;
done:
    stereo_preview_stop(preview);
    stereo_capture_close();
    if(tx>=0) close(tx);
    if(rx>=0) close(rx);
    if(child>0) { kill(child,SIGTERM); waitpid(child,NULL,0); }
    free(images); free(pred); free(metrics); free(masks); free(ring);
    return result;
}
