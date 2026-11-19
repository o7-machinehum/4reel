#define _POSIX_C_SOURCE 200809L
#include "stereo_trace.h"
#include <stdlib.h>
#include <string.h>
const char *trace_stage_name(unsigned s) {
 static const char *names[]={
#define X(a,b) b,
 TRACE_STAGES(X)
#undef X
 };
 return s<TS_COUNT ? names[s]:"unknown";
}
void trace_ring_push(struct TraceRing *r,const struct PairTrace *p) {
 r->pair[r->next]=*p; r->next=(r->next+1)%TRACE_RING;
 if(r->count<TRACE_RING) ++r->count;
}
static int cmp(const void *a,const void *b) { uint64_t x=*(const uint64_t *)a,y=*(const uint64_t *)b; return (x>y)-(x<y); }
uint64_t trace_percentile(const struct TraceRing *r,unsigned stage,unsigned percentile,unsigned *samples) {
 uint64_t v[TRACE_RING]; unsigned n=0;
 for(unsigned i=0;i<r->count;++i) {
  uint64_t sum=0; int found=0;
  for(unsigned j=0;j<r->pair[i].trace.count;++j) {
   const struct TraceEvent *e=&r->pair[i].trace.event[j];
   if(e->stage==stage && e->end>=e->begin) { sum+=e->end-e->begin; found=1; }
  }
  if(found) v[n++]=sum;
 }
 if(samples) *samples=n;
 if(!n || percentile<1 || percentile>100) return 0;
 qsort(v,n,sizeof(*v),cmp); return v[(n*percentile+99)/100-1];
}
uint64_t trace_union_us(const struct PairTrace *p,unsigned resource) {
 /* Union of intervals, not sum: nested and overlapping calls counted once. */
 uint64_t endpoints[2*TRACE_EVENTS]; unsigned n=0;
 for(unsigned i=0;i<p->trace.count;++i) {
  const struct TraceEvent *e=&p->trace.event[i];
  if(e->resource==resource && e->end>e->begin) { endpoints[n++]=e->begin; endpoints[n++]=e->end; }
 }
 qsort(endpoints,n,sizeof(*endpoints),cmp); uint64_t total=0;
 for(unsigned i=1;i<n;++i) if(endpoints[i]>endpoints[i-1]) {
  for(unsigned j=0;j<p->trace.count;++j) {
   const struct TraceEvent *e=&p->trace.event[j];
   if(e->resource==resource && e->begin<=endpoints[i-1] && e->end>=endpoints[i]) {
    total+=endpoints[i]-endpoints[i-1]; break;
   }
  }
 }
 return total;
}
