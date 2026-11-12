// Exercise the actual offline dashboard script with a minimal DOM and API.
// Run the native drive-tuning tests first to generate the float32 curve fixture.
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const root = path.resolve(__dirname, '../..');
const source = fs.readFileSync(path.join(root, 'src/main/Session_Logger.ino'), 'utf8');
const html = source.match(/R"__MELTYHTML__\(([\s\S]*?)\)__MELTYHTML__"/)[1];
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
assert(!/<script\b[^>]*\bsrc\s*=/i.test(html), 'Dashboard must work without external scripts');
const fixtureData = JSON.parse(fs.readFileSync(
  path.join(root, 'build/main-drive-tuning-tests/spin-curve-fixture.json'), 'utf8'));
const fixtures = fixtureData.curves;
const elements = new Map();
let viewportWidth = 760;
class Element {
  constructor(id) {
    this.id = id;
    this.attributes = {};
    this.listeners = {};
    this.disabled = false;
    this.className = '';
    this.textContent = '';
    this._html = '';
    this._value = '';
    this.parent = null;
  }
  set value(value) { this._value = String(value); }
  get value() { return this._value; }
  set innerHTML(value) {
    this._html = String(value);
    if (!['config', 'ramp-config'].includes(this.id)) return;
    for (const match of this._html.matchAll(/<input\b([^>]+)>/g)) {
      const attributes = Object.fromEntries([...match[1].matchAll(/([\w-]+)="([^"]*)"/g)]
        .map(attribute => [attribute[1], attribute[2]]));
      const element = new Element(attributes.id);
      element.value = attributes.value || '';
      element.attributes = attributes;
      element.parent = this;
      elements.set('#' + element.id, element);
    }
  }
  get innerHTML() { return this._html; }
  setAttribute(name, value) { this.attributes[name] = String(value); }
  getAttribute(name) { return this.attributes[name] ?? null; }
  addEventListener(name, callback) { this.listeners[name] = callback; }
  querySelectorAll() { return [...elements.values()].filter(element => element.parent === this); }
  getBoundingClientRect() { return {width: viewportWidth, height: 360}; }
  get clientWidth() { return viewportWidth; }
}
function element(selector) {
  if (!elements.has(selector)) elements.set(selector, new Element(selector.slice(1)));
  return elements.get(selector);
}
const configuration = fixtureData.configuration;
const defaults = {...configuration.defaults};
const requests = [];
let rejectNextSave = false;
const windowListeners = {};
const context = vm.createContext({
  document: {querySelector: element},
  window: {addEventListener: (name, callback) => { windowListeners[name] = callback; }},
  console, AbortController, confirm: () => true,
  setTimeout: () => 1, clearTimeout: () => {},
  fetch: async (url, options) => {
    requests.push({url, options});
    assert(url.startsWith('/api/'), 'Preview must use only local controller APIs');
    let body, status = 200;
    if (url === '/api/config.json') body = configuration;
    else {
      assert.equal(options.method, 'POST');
      assert(element('#cfg-acc_rate').disabled && element('#cfg-motor_full_scale_percent').disabled,
        'Saving must disable controls in both configuration groups');
      const parsed = new URL(url, 'http://controller');
      const name = parsed.searchParams.get('name');
      if (rejectNextSave) {
        rejectNextSave = false; status = 500; body = {error: 'Simulated storage failure'};
      } else {
        const value = parsed.pathname.endsWith('/reset') ? defaults[name] : Number(parsed.searchParams.get('value'));
        configuration[name] = value;
        body = {ok: true, name, value, default: defaults[name]};
      }
    }
    return {ok: status === 200, status, text: async () => JSON.stringify(body)};
  },
});
vm.runInContext(script, context, {filename: 'firmware-dashboard.js'});
const run = code => vm.runInContext(code, context);
function pathData(id) {
  const tag = element('#spin-curve-svg').innerHTML.match(new RegExp(`<path\\b[^>]*id="${id}"[^>]*>`));
  return tag ? tag[0].match(/\bd="([^"]*)"/)[1] : null;
}
function edit(name, value) {
  const input = element('#cfg-' + name);
  input.value = value;
  assert(input.parent && input.parent.listeners.input, 'Both groups must redraw the graph on input');
  input.parent.listeners.input({target: input});
}
const posts = () => requests.filter(request => request.options.method === 'POST').length;

(async () => {
  // Graph calculations follow actual compiled C++ across slope/knee combinations.
  for (const fixture of fixtures) {
    const actual = context.spinCurveCommand(fixture.rpm, fixture.start,
      fixture.rate, configuration.spin_curve);
    // C++ uses float32 powf while the browser uses double-precision Math.pow.
    assert(Math.abs(actual - fixture.ceiling) <= 1,
      `Graph/firmware mismatch at ${JSON.stringify(fixture)}: ${actual}`);
    assert(actual >= 200 && actual <= 999);
  }
  assert.equal(context.spinCurveCommand(-1, 2000, .1, configuration.spin_curve), 200);
  assert.equal(context.spinCurveCommand(2000, 2000, .1, configuration.spin_curve), 200);
  assert(Number.isNaN(context.spinCurveCommand(2000, 1000, NaN, configuration.spin_curve)));

  // Values accepted through the API must not be rounded into a different curve.
  configuration.acc_rate = .055;
  configuration.spin_ramp_start_rpm = 1234.567;
  await run('loadConfiguration()');
  const panelPosition = html.indexOf('class="curve-panel"');
  const controlsPosition = html.indexOf('id="ramp-config"');
  const graphPosition = html.indexOf('id="spin-curve-svg"');
  assert(panelPosition >= 0 && controlsPosition > panelPosition && controlsPosition < graphPosition,
    'Ramp controls must appear together immediately beside the graph');
  for (const name of ['acc_rate', 'spin_ramp_start_rpm']) {
    assert.equal(element('#cfg-' + name).parent.id, 'ramp-config');
    assert(!element('#config').innerHTML.includes('id="cfg-' + name + '"'));
  }
  assert.equal(element('#ramp-config').querySelectorAll('input').length, 2);
  assert.equal(element('#config').querySelectorAll('input').length, 5);
  assert.equal(element('#cfg-acc_rate').value, '0.055');
  assert.equal(element('#cfg-spin_ramp_start_rpm').value, '1234.567');
  const baselineDefault = pathData('spin-curve-default');
  const baselinePreview = pathData('spin-curve-preview');
  assert(baselineDefault && baselinePreview);
  assert.equal(baselinePreview,
    context.spinCurvePath(1234.567, .055, configuration.spin_curve));
  assert(element('#spin-curve-svg').innerHTML.includes('stroke-dasharray='));
  assert.match(element('#spin-curve-status').textContent, /Saved/);
  const initialFetchCount = requests.length;
  edit('spin_ramp_start_rpm', '2500');
  assert.notEqual(pathData('spin-curve-preview'), baselinePreview);
  edit('acc_rate', '.08');
  assert.equal(pathData('spin-curve-default'), baselineDefault);
  assert.match(element('#spin-curve-status').textContent, /Unsaved/);
  assert.equal(requests.length, initialFetchCount, 'Draft preview must be offline');
  const draftCurve = pathData('spin-curve-preview');
  edit('motor_full_scale_percent', '50');
  assert.equal(pathData('spin-curve-preview'), draftCurve, 'Graph uses logical command before physical scaling');

  await run("updateConfiguration('acc_rate',false)");
  assert.equal(configuration.acc_rate, .08);
  assert.equal(element('#cfg-spin_ramp_start_rpm').value, '2500');
  assert.equal(element('#cfg-motor_full_scale_percent').value, '50');
  assert.equal(requests.filter(request => request.url === '/api/config.json').length, 1,
    'Saving one control must preserve drafts without replacing the form');
  assert.match(element('#spin-curve-status').textContent, /Unsaved/);
  await run("updateConfiguration('spin_ramp_start_rpm',false)");
  assert.equal(configuration.spin_ramp_start_rpm, 2500);
  assert.match(element('#spin-curve-status').textContent, /Saved/);
  assert.equal(pathData('spin-curve-default'), baselineDefault);

  for (const [name, invalid] of [['acc_rate', '0'], ['acc_rate', 'NaN'],
    ['spin_ramp_start_rpm', ''], ['spin_ramp_start_rpm', '5001']]) {
    const before = posts();
    edit(name, invalid);
    assert.equal(pathData('spin-curve-preview'), null);
    assert.equal(pathData('spin-curve-default'), baselineDefault);
    assert.equal(element('#cfg-' + name).getAttribute('aria-invalid'), 'true');
    await run(`updateConfiguration('${name}',false)`);
    assert.equal(posts(), before, 'Invalid draft must not be sent');
    edit('acc_rate', '.08'); edit('spin_ramp_start_rpm', '2500');
  }
  edit('acc_rate', '.13');
  await run("updateConfiguration('spin_ramp_start_rpm',true)");
  assert.equal(configuration.spin_ramp_start_rpm, defaults.spin_ramp_start_rpm);
  assert.equal(element('#cfg-acc_rate').value, '.13', 'Reset must preserve the other control draft');
  assert.match(element('#spin-curve-status').textContent, /Unsaved/);
  await run("updateConfiguration('acc_rate',true)");
  assert.equal(configuration.acc_rate, defaults.acc_rate);
  assert.match(element('#spin-curve-status').textContent, /Saved/);

  edit('spin_ramp_start_rpm', '4000'); rejectNextSave = true;
  await run("updateConfiguration('spin_ramp_start_rpm',false)");
  assert.equal(configuration.spin_ramp_start_rpm, defaults.spin_ramp_start_rpm);
  assert.equal(element('#cfg-spin_ramp_start_rpm').value, '4000');
  assert.match(element('#config-status').textContent, /Simulated storage failure/);
  assert.equal(element('#cfg-spin_ramp_start_rpm').disabled, false);
  assert.equal(pathData('spin-curve-default'), baselineDefault);

  viewportWidth = 330;
  if (windowListeners.resize) windowListeners.resize();
  assert(pathData('spin-curve-preview') && pathData('spin-curve-default'));
  assert(element('#spin-curve-svg').getAttribute('viewBox'), 'Graph must resize in the offline page');
  console.log(`PASS: ${fixtures.length} compiled C++/JS curve comparisons, offline live preview, dotted defaults, independent save/reset and draft preservation, invalid inputs, failed saves, and mobile graph resize`);
})().catch(error => { console.error(error); process.exitCode = 1; });
