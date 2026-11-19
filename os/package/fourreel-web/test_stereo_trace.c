#define _POSIX_C_SOURCE 200809L
#include "stereo_trace.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
int main(void) {
 struct TraceRing *r=calloc(1,sizeof(*r)); assert(r);
 struct PairTrace p={.start=100,.end=300,.success=1,.trace={.enabled=1,.count=4}};
 p.trace.event[0]=(struct TraceEvent){100,200,3,TS_NPU_L,TR_NPU};
 p.trace.event[1]=(struct TraceEvent){150,250,4,TS_NPU_R,TR_NPU};
 p.trace.event[2]=(struct TraceEvent){170,190,5,TS_NPU_L,TR_NPU};
 p.trace.event[3]=(struct TraceEvent){110,130,20,TS_MATCH,TR_MAIN};
 assert(trace_union_us(&p,TR_NPU)==150); /* Not 220; includes nesting. */
 assert(trace_union_us(&p,TR_MAIN)==20);
 assert(trace_union_us(&p,TR_RGA)==0);
 trace_ring_push(r,&p); unsigned n=0;
 assert(trace_percentile(r,TS_NPU_L,50,&n)==120 && n==1);
 assert(trace_percentile(r,TS_CORNERS,95,&n)==0 && n==0);
 *r=(struct TraceRing){0};
 for(unsigned i=1;i<=100;++i) { p.id=i; p.trace.count=1; p.trace.event[0]=(struct TraceEvent){0,i,0,TS_MATCH,TR_MAIN}; trace_ring_push(r,&p); }
 assert(r->count==64); /* Remaining durations 37..100. */
 assert(trace_percentile(r,TS_MATCH,50,&n)==68 && n==64);
 assert(trace_percentile(r,TS_MATCH,95,&n)==97);
 assert(trace_percentile(r,TS_MATCH,100,NULL)==100);
 struct Trace t={.enabled=0}; struct TraceMark m=trace_begin(&t); assert(!m.wall && !m.cpu);
 trace_end(&t,m,TS_MATCH,TR_MAIN); assert(t.count==0);
 t.enabled=1; t.count=TRACE_EVENTS; m=trace_begin(&t); trace_end(&t,m,TS_MATCH,TR_MAIN); assert(t.overflow==1 && t.count==TRACE_EVENTS);
 _Static_assert(sizeof(struct TraceEvent)==32,"Wire event size");
 _Static_assert(sizeof(struct RunnerRequest)==16,"Wire request size");
 _Static_assert(sizeof(struct RunnerResponse)==2096,"Wire response size");
 free(r); puts("PASS mock timings: overlap union, stage sums, nearest-rank quantiles, ring wrap, missing stages, disabled tracing, overflow and wire sizes");
}
