// Host-only DOM smoke test; fixture is emitted by test_stereo_preview, not a device.
const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const source=fs.readFileSync(path.join(__dirname,'stereo_preview.html'),'utf8').split('<script>')[1].split('</script>')[0];
const fixture=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
assert.equal(fixture.pair_id,'9007199254740993'); assert.equal(fixture.hardware_timings,null); assert.equal(fixture.first_eye_ms,null);
const bmp=Buffer.from(fixture.bmp.split(',')[1],'base64'); assert.equal(bmp.readUInt32LE(18),320);assert.equal(bmp.readUInt32LE(22),200);
const elements={}; const context=new Proxy({}, {get:()=>()=>{},set:()=>true});
function element(){return {textContent:'',checked:true,width:1060,height:245,append(){},replaceChildren(...rows){this.rows=rows},getContext(){return context},getBoundingClientRect(){return {left:0,top:0,width:1060,height:245}}}}
const sandbox={document:{getElementById:id=>elements[id]??=(element()),createElement:element},setInterval(){},fetch:()=>new Promise(()=>{}),fixture,console};
vm.createContext(sandbox);vm.runInContext(source,sandbox);vm.runInContext('update(fixture)',sandbox);
assert.equal(elements.stages.rows.length,fixture.stages.length);assert(elements.warning.textContent.includes(fixture.pair_id));assert(elements.bottleneck.textContent.includes('Hardware busy counters'));
assert.equal(elements.preview.src,fixture.bmp);assert(elements.summary.textContent.includes('unavailable'));
vm.runInContext('update({enabled:false,ready:false,modes:{},diagnostic_work:{}})',sandbox);
assert(elements.warning.textContent.includes('frozen')); console.log('PASS frontend mock: strict JSON/BMP, coherent trigger ID, timeline/table rendering, unavailable timing and off-state labels');
