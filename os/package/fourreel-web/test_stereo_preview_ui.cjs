// Host-only DOM smoke test; fixture is emitted by test_stereo_preview, not a device.
const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const source=fs.readFileSync(path.join(__dirname,'stereo_preview.html'),'utf8').split('<script>')[1].split('</script>')[0];
const fixture=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
assert.equal(fixture.pair_id,'9007199254740993'); assert.equal(fixture.hardware_timings,null); assert.equal(fixture.first_eye_ms,null);
const bmp=Buffer.from(fixture.bmp.split(',')[1],'base64'); assert.equal(bmp.readUInt32LE(18),320);assert.equal(bmp.readUInt32LE(22),200);
const elements={}; const context=new Proxy({}, {get:()=>()=>{},set:()=>true});
function element(){return {style:{},dataset:{},textContent:'',checked:true,width:1060,height:245,append(){},replaceChildren(...rows){this.rows=rows},getContext(){return context},getBoundingClientRect(){return {left:0,top:0,width:1060,height:245}}}}
const sandbox={document:{getElementById:id=>elements[id]??=(element()),createElement:element},setInterval(){},fetch:()=>new Promise(()=>{}),fixture,console};
vm.createContext(sandbox);vm.runInContext(source,sandbox);vm.runInContext('update(fixture)',sandbox);
assert.equal(elements.stages.rows.length,fixture.stages.length);assert(elements.warning.textContent.includes(fixture.pair_id));assert(elements.bottleneck.textContent.includes('Hardware busy counters'));
assert.equal(vm.runInContext('views.depth.width',sandbox),320);assert.equal(vm.runInContext('views.gray.width',sandbox),1280);assert(elements.summary.textContent.includes('unavailable'));
vm.runInContext('update({enabled:false,ready:false,modes:{},diagnostic_work:{}})',sandbox);
assert(elements.warning.textContent.includes('frozen')); console.log('PASS frontend mock: strict JSON/BMP, coherent trigger ID, timeline/table rendering, unavailable timing and off-state labels');

for(const flags of [0,1,2,3]){
 vm.runInContext('showDepthUnits('+flags+')',sandbox);
 for(let e=0;e<2;e++) assert.equal(elements['depthMode'+e].textContent,(flags&(1<<e))?'· Metric (m)':'· Relative');
 assert.equal(elements.metricLegend.dataset.active,String(flags!==0));
}
assert.equal(vm.runInContext('metricFraction(.15)',sandbox),0);
assert.equal(vm.runInContext('metricFraction(.30)',sandbox),0);
assert(vm.runInContext('metricFraction(.31)',sandbox)>0);
assert.equal(vm.runInContext('metricFraction(20)',sandbox),1);

for(const [src,pair,age,state] of [['0','10','0','waiting'],['10','10','0','fresh'],['10','14','1750','1.8 s old']]){
 sandbox.scaleHeaders={get:k=>({'X-Scale-Source-Pair':src,'X-Stereo-Scale':src==='0'?'0':'0.1','X-Scale-Age-Ms':age})[k]};
 assert(vm.runInContext('scaleDescription(scaleHeaders,"'+pair+'",3)',sandbox).includes(state));
}
