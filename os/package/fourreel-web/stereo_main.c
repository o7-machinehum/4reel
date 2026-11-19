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
                 "Metric maps are optical Z metres. one shared stereo scale; mask 1=finite scaled pixel.\n"
                 "No hardware-verified accuracy or throughput is implied by this development build.");
            return !strcmp(argv[i],"--help") ? 0:2;
        }
    }
    if((!output && !live) || (output && live) || (capture && live) || !unit || !valid_unit(unit)) {
        fputs("Choose --live or --output DIR; --capture requires --output; valid --unit required\n",stderr); return 2;
    }
    struct StereoCalibration cal={0}; int calibrated=0;
    if(live && !calfile) {
        const char *installed="/etc/4reel/unit.ds";
        if(access(installed,F_OK)==0 || errno!=ENOENT) calfile=installed;
    }
    if(calfile) {
        if(stereo_calibration_load(calfile,unit,&cal)) { fputs("Invalid/mismatched unit calibration; refusing calibrated run\n",stderr); return 1; }
        calibrated=1;
    }
    if(!live && mkdir(output,0755) && errno!=EEXIST) { perror(output); return 1; }
    struct statvfs space;
    if(!live && (statvfs(output,&space) || (uint64_t)space.f_bavail*space.f_frsize < (uint64_t)count*(capture ? 2100000:1200000)+8*1024*1024)) {
        fputs("Insufficient output space; reduce --count or use a larger filesystem\n",stderr); return 1;
    }
    int dma=raw8 && !capture;
    uint8_t *images=dma ? NULL:malloc(2*RAW),*masks=calloc(2,320*200);
    float *pred=malloc(2*N*sizeof(float)),*metrics=calloc(2*N,sizeof(float));
    struct TraceRing *ring=calloc(1,sizeof(*ring));
    struct StereoPreview *preview=NULL;
    if((!dma && !images) || !pred || !metrics || !masks || !ring) return 1;
    int tx=-1,rx=-1; pid_t child=-1;
    int result=1;
    signal(SIGINT,stopping); signal(SIGTERM,stopping); signal(SIGPIPE,SIG_IGN);
    if(stereo_capture_open(raw8)) { perror("stereo capture/FSIN setup"); goto done; }
    if(live && stereo_capture_enable_ae()) { perror("paired auto-exposure setup"); goto done; }
    if(dma && stereo_capture_export()) { perror("export RAW8 camera buffers"); goto done; }
    if(stop) goto done;
    if(!capture) {
        int request[2],response[2];
        if(pipe(request) || pipe(response)) goto done;
        child=fork();
        if(child<0) goto done;
        if(!child) {
            /* The parent handles stop signals. SDK buffers must be released by
             * the helper itself after EOF or a closed response pipe. Ignored
             * dispositions persist across exec, including the inherited SIGPIPE. */
            signal(SIGINT,SIG_IGN); signal(SIGTERM,SIG_IGN);
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
        fprintf(stderr,"Stereo preview on port %u (depth at pair rate, grayscale at 1 Hz)\n",port);
    }
    struct StereoCornerHistory corner_history={0};
    double factor=0;
    uint64_t scale_pair=0,scale_time=0;
    int previous_mode=-1;
    for(unsigned index=0;(live || index<count) && !stop;++index) {
        uint64_t started=now_us(),cpu_start=trace_clock(CLOCK_THREAD_CPUTIME_ID);
        struct PairTrace record={.start=started,.trace={.enabled=preview ? stereo_preview_enabled(preview):diagnostics}};
        if(previous_mode!=(int)record.trace.enabled) { memset(ring,0,sizeof(*ring)); previous_mode=record.trace.enabled; }
        struct TraceMark timing=trace_begin(&record.trace);
        struct CapturePair pair;
        struct CaptureDmaView view={0};
        struct StereoImage eyes[2];
        int acquired=dma ? stereo_capture_pair_dma(&pair,&record.trace,&view)
                         : stereo_capture_pair_timed(images,images+RAW,&pair,&record.trace);
        if(acquired) {
            record.id=pair.id; record.end=now_us(); trace_end(&record.trace,timing,TS_CAPTURE,TR_ENVELOPE);
            trace_ring_push(ring,&record);
            fprintf(stderr,"Failed pair %" PRIu64 "; capture/pairing %.3f ms; no result published\n",pair.id,(record.end-started)/1000.);
            perror("ambiguous/failed triggered pair"); goto done;
        }
        for(int e=0;e<2;e++) {
            if(dma) eyes[e]=(struct StereoImage){view.data[e]+STEREO_W-1,(int)view.stride[e],-1};
            else eyes[e]=(struct StereoImage){images+e*RAW,STEREO_W,1};
        }
        trace_end(&record.trace,timing,TS_CAPTURE,TR_ENVELOPE);
        record.id=pair.id; record.trigger_us=pair.trigger_before_us;
        record.input_us=(uint64_t)(pair.timestamp_us[0]<pair.timestamp_us[1] ? pair.timestamp_us[0]:pair.timestamp_us[1]);
        record.first_capture_us=pair.dequeued_us[0]<pair.dequeued_us[1] ? pair.dequeued_us[0]:pair.dequeued_us[1];
        record.complete_capture_us=pair.dequeued_us[0]>pair.dequeued_us[1] ? pair.dequeued_us[0]:pair.dequeued_us[1];
        memcpy(record.sequence_gap,pair.sequence_gap,sizeof(record.sequence_gap));
        struct StereoMatch matches[STEREO_MAX_ANCHORS]; unsigned anchors=0;
        int valid[2]={0,0};
        uint64_t inference_started=now_us();
        if(!capture) {
            struct RunnerRequest request={.id=pair.id,.protocol=dma ? TRACE_DMA_PROTOCOL:TRACE_PROTOCOL,.enabled=record.trace.enabled};
            timing=trace_begin(&record.trace);
            uint64_t *sw=record.trace.enabled ? &record.submit_wait_us:NULL;
            if(io_all(tx,&request,sizeof(request),1,sw)) goto inference_error;
            if(dma) {
                struct RunnerDmaPair buffers={{view.fd[0],view.fd[1]},{view.stride[0],view.stride[1]}};
                if(io_all(tx,&buffers,sizeof(buffers),1,sw)) goto inference_error;
            } else if(io_all(tx,images,2*RAW,1,sw)) goto inference_error;
            trace_end(&record.trace,timing,TS_SUBMIT,TR_WAIT);
            if(live && stereo_capture_adjust_ae(eyes,&record)) { perror("paired exposure update"); goto done; }
            /* CPU stereo overlaps the helper's RGA + sequential left/right NPU. */
            stereo_match_pair(eyes,calibrated ? &cal:NULL,matches,&record.sparse,&record.trace,&corner_history);
            anchors=record.sparse.anchors;
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
            if(calibrated) {
                timing=trace_begin(&record.trace);
                double votes[STEREO_MAX_ANCHORS][2];
                for(unsigned k=0;k<anchors;k++) for(unsigned e=0;e<2;e++) {
                    const double *xy=matches[k].xy[e];
                    votes[k][e]=matches[k].z[e]*prediction_at(pred+e*N,xy[0],xy[1]);
                }
                record.sparse.inliers=stereo_scale(votes,anchors,&factor);
                if(record.sparse.inliers) { scale_pair=pair.id; scale_time=started; }
                record.sparse.factor=factor;
                record.sparse.source_pair=scale_pair;
                record.sparse.scale_age_us=scale_time ? started-scale_time:0;
                trace_end(&record.trace,timing,TS_FIT_L,TR_MAIN);
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
                if(factor>0) {
                    valid[e]=!stereo_scale_map(pred+e*N+TENSOR_PAD*TENSOR_W,metric+TENSOR_PAD*TENSOR_W,64000,factor);
                    trace_end(&record.trace,timing,e ? TS_MAP_R:TS_MAP_L,TR_MAIN);
                }
                memset(mask,valid[e],64000);
                if(!valid[e] && !live) memset(metric,0,N*sizeof(float));
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
                "  \"metric_valid\":[%s,%s],\"stereo_scale\":%.9g,\"scale_source_pair\":\"%" PRIu64 "\",\"scale_age_ms\":%.3f,\"scale_inliers\":%u,\n"
                "  \"processing_ms\":%.3f,\"pair_work_ms\":%.3f,\"hardware_accuracy_validated\":false,\n"
                "  \"depth_convention\":\"optical_z_metres;0_unavailable;mask1_global_scale\"\n}\n",
                unit,pair.id,pair.trigger_before_us,pair.trigger_after_us,pair.timestamp_us[0],pair.timestamp_us[1],pair.sequence[0],pair.sequence[1],
                pair.flags[0],pair.flags[1],raw8 ? "RAW8":"RAW10-top8",
                capture ? "disabled-calibration-capture":"rknn-2.5.0-rv1103-one-context",
                capture ? "native":"rga-gray-average-4x-pad12",
                capture ? "disabled-capture":calibrated ? "cpu-bidirectional-epipolar-zncc":"central-mutual-patches",
                calibrated ? "true":"false",calibrated ? cal.source_sha256:"",anchors,
                valid[0] ? "true":"false",valid[1] ? "true":"false",factor,scale_pair,record.sparse.scale_age_us/1000.,record.sparse.inliers,
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
        stereo_preview_publish(preview,&record,ring,eyes,pred,metrics,masks,valid,capture,
                               matches,calibrated ? anchors:record.sparse.mutual);
        record.end=now_us(); record.main_cpu_us=trace_clock(CLOCK_THREAD_CPUTIME_ID)-cpu_start;
        if(record.trace.enabled) ring->pair[(ring->next+TRACE_RING-1)%TRACE_RING]=record;
        if(dma && stereo_capture_release()) { perror("release camera buffers"); goto done; }
        continue;
inference_error:
        record.end=now_us(); trace_ring_push(ring,&record);
        fputs("Genuine RGA/RKNN inference failed; no relative or metric result published for this pair\n",stderr);
        goto done;
    }
    result=stop ? 1:0;
done:
    stereo_preview_stop(preview);
    if(tx>=0) close(tx);
    if(rx>=0) close(rx);
    if(child>0) {
        int status;
        pid_t waited;
        do { waited=waitpid(child,&status,0); } while(waited<0 && errno==EINTR);
        if(waited<0 || !WIFEXITED(status) || WEXITSTATUS(status)!=0) result=1;
    }
    stereo_capture_close();
    free(images); free(pred); free(metrics); free(masks); free(ring);
    return result;
}
