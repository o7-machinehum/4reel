// Run with node tests/depth-legend.mjs; no browser dependencies needed.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';

const html = fs.readFileSync(new URL('../index.html', import.meta.url), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
new vm.Script(script);
const start = script.indexOf('function formatDistance(');
const end = script.indexOf('let depthLegendKey', start);
const { relative, metric } = vm.runInNewContext(
  `${script.slice(start, end)}; ({ relative: relativeDepthLegend, metric: metricDepthLegend })`);

const ticks = [[5000, '#e12d34'], [2000, '#fac33b'], [500, '#5bcf7e'], [75, '#3653e8']];
const original = relative(ticks);
const shared = metric(ticks);
assert.equal(original.map(entry => entry.label).join(','), 'Far,,,Near');
assert.equal(original.map(entry => entry.color).join(','), shared.map(entry => entry.color).join(','));
assert.equal(shared.map(entry => entry.label).join(','), '≥5 m,2.0 m,50 cm,≤7.5 cm');
assert.equal(shared[0].color, '#e12d34');
assert.equal(shared[3].color, '#3653e8');
console.log('PASS: shared colours for relative and metric depth legends');
