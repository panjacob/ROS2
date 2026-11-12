// Exercise the page's actual script. This fixture does not claim browser rendering
// or native number/range input validation, transport, ROS or hardware validation.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
new vm.Script(script); // syntax check, including initialization and polling code
const elements = new Map();
let focused = null;
function element(id) {
  if (!elements.has(id)) elements.set(id, {
    _value: '', get value() { return this._value; }, set value(v) { this._value = String(v); },
    textContent: '', style: {}, disabled: false, validity: {badInput: false},
    setAttribute(k, v) { this[k] = v; }, focus() { focused = this; }, replaceChildren() {}, append() {},
  });
  return elements.get(id);
}
element('speed').value = '120';
const requests = [];
let pendingFetch = null;
const context = vm.createContext({
  console, document: {getElementById: element, createElement: () => element('option'), hasFocus: () => false},
  window: {addEventListener() {}}, navigator: {}, Date,
  setInterval: () => 1, clearInterval() {},
  fetch: (url, opts) => {
    if (opts) requests.push({url, body: JSON.parse(opts.body)});
    if (pendingFetch) return pendingFetch(url, opts);
    return Promise.resolve({ok: true, json: async () => ({ok: true, plan_id: 'preview'})});
  },
});
// Omit only automatic startup. Tests invoke real handlers/poll explicitly.
vm.runInContext(script.replace('setInterval(poll, 200);\npoll();', ''), context);
const run = code => vm.runInContext(code, context);
const close = (a, b) => assert.ok(Math.abs(a - b) < 1e-10, `${a} != ${b}`);
run(`config = {joint_limits_deg: {joint1: [-180, 180], joint2: [-90, 90]}, home_joints: [], max_jog_speed: 360, execution_available: true, tcps: []};
  connected = true; latestJoints = {joint1: {position_deg: -30}, joint2: {position_deg: 0}};
  latestJointsAge = 0; latestJointsReceived = Date.now();`);
function targets(a, b) {
  element('target_joint1').value = String(a);
  element('target_joint2').value = String(b);
  run('planChanged()');
}
function units(value) { run(`setJointUnits('${value}')`); }
function read() { return JSON.parse(run('JSON.stringify(readTargets())')); }
function ready() { run(`planId = 'ready'; planReady = true; planPending = false; executionPending = false; updatePlanButtons()`); }
async function main() {
  targets(-180, 0);
  ready();
  const revision = run('planRevision');
  for (let i = 0; i < 100; i++) { units('rad'); units('deg'); }
  assert.deepEqual(read(), {joint1: -180, joint2: 0});
  assert.equal(run('planRevision'), revision);
  assert.equal(run('planReady'), true);
  assert.equal(element('executeButton').disabled, false);
  units('rad');
  close(Number(element('target_joint1').value), -Math.PI);
  close(Number(element('target_joint1').min), -Math.PI);
  close(Number(element('target_joint1').max), Math.PI);
  assert.equal(element('target_joint1').step, 'any');
  assert.equal(element('positionHeading').textContent, 'Position [rad]');
  assert.equal(element('target_joint1')['aria-label'], 'joint1 target angle in radians');
  assert.equal(element('pos_joint1').textContent, (-Math.PI / 6).toFixed(4));
  assert.match(element('scale_joint1').textContent, /-3.1416 rad/);
  assert.equal(element('speedUnit').textContent, 'rad/s');
  close(Number(element('speed').value), 2 * Math.PI / 3);
  close(Number(element('speed').max), 2 * Math.PI);
  // Exact configured endpoints and near-limit user input.
  targets(Math.PI, -Math.PI / 2);
  close(read().joint1, 180); close(read().joint2, -90);
  targets(Math.PI - 1e-10, 0); assert.ok(read().joint1 < 180);
  for (const invalid of ['NaN', 'Infinity', '-Infinity', 'abc', '1e309', '4']) {
    targets(invalid, 0); assert.equal(read(), null);
    units('deg'); assert.equal(read(), null); units('rad'); assert.equal(read(), null);
  }
  // Blank + fresh state: the current angle is used and shown in the active unit.
  targets('', 0.5);
  assert.deepEqual(Object.keys(read()), ['joint1', 'joint2']);
  close(read().joint1, -30); close(read().joint2, 0.5 * 180 / Math.PI);
  close(Number(element('target_joint1').value), -Math.PI / 6);
  units('deg'); assert.equal(Number(element('target_joint1').value), -30);
  targets('', ''); assert.deepEqual(read(), {joint1: -30, joint2: 0});
  assert.equal(element('target_joint1').value, '-30'); assert.equal(element('target_joint2').value, '0');
  // Unparsable number-input text ('' with badInput) + fresh state is rejected, not filled.
  targets('', 0); element('target_joint1').validity = {badInput: true}; focused = null;
  const sentBefore = requests.length;
  assert.equal(read(), null);
  assert.match(element('planStatus').textContent, /Enter a finite target for joint1 within -180.00° to 180.00°/);
  assert.equal(focused, element('target_joint1'));
  assert.equal(element('target_joint1').value, '');
  await run('moveDirectly()'); await run('planJoints()');
  assert.equal(requests.length, sentBefore);
  assert.equal(element('target_joint1').value, '');
  element('target_joint1').validity = {badInput: false};
  // Blank + stale state is rejected; non-blank targets do not depend on freshness.
  run('latestJointsAge = 3');
  targets('', 10); assert.equal(read(), null);
  assert.equal(element('planStatus').textContent, 'Enter a target for joint1 or wait for fresh joint state.');
  assert.equal(element('target_joint1').value, '');
  targets(10, 20); assert.deepEqual(read(), {joint1: 10, joint2: 20});
  run('connected = false'); targets(10, ''); assert.equal(read(), null); run('connected = true');
  run('latestJointsAge = 0; latestJointsReceived = Date.now()');
  // Blank + current angle outside limits is rejected, not clamped.
  run('latestJoints.joint2.position_deg = 95');
  targets(10, ''); assert.equal(read(), null);
  assert.match(element('planStatus').textContent, /Enter a finite target for joint2 within -90.00° to 90.00°/);
  assert.equal(element('target_joint2').value, '');
  run('latestJoints.joint2.position_deg = 0');
  // Non-blank out-of-range input keeps the existing limit rejection.
  targets(10, 91); assert.equal(read(), null);
  targets(-45, 0); ready();
  targets(-44, 0); assert.equal(run('planReady'), false);
  units('rad'); run('copyCurrentToPlan()');
  close(Number(element('target_joint1').value), -Math.PI / 6);
  assert.deepEqual(read(), {joint1: -30, joint2: 0});
  // Poll rendering must not overwrite newly edited targets or blank invalid data.
  targets(-0.3, ''); run('renderJointUnits()');
  assert.equal(element('target_joint1').value, '-0.3');
  assert.equal(element('target_joint2').value, '');
  // All motion payloads remain degrees / degrees per second.
  for (const unit of ['deg', 'rad']) {
    units(unit); run('copyCurrentToPlan()');
    await run('moveDirectly()');
    assert.deepEqual(requests.at(-1), {url: '/api/goto', body: {joint1: -30, joint2: 0}});
    await run('planJoints()');
    assert.deepEqual(requests.at(-1), {url: '/api/plan', body: {joint1: -30, joint2: 0}});
    run('startJog("joint1", -1)');
    assert.deepEqual(requests.at(-1), {url: '/api/jog', body: {joint1: -120}});
    units(unit === 'deg' ? 'rad' : 'deg');
    run('startJog("joint1", 1)');
    assert.deepEqual(requests.at(-1), {url: '/api/jog', body: {joint1: 120}});
    run('stopJog()'); assert.equal(requests.at(-1).body.joint1, 0);
  }
  // User-edited radian speed preserves physical value across repeated switching.
  units('rad'); element('speed').value = '1.2345'; run('speedChanged()');
  const physicalSpeed = 1.2345 * 180 / Math.PI;
  for (let i = 0; i < 100; i++) { units('deg'); units('rad'); }
  run('startJog("joint2", 1)'); close(requests.at(-1).body.joint2, physicalSpeed);
  run('stopJog()');
  // A unit switch during in-flight planning does not discard the response.
  let resolvePlan;
  pendingFetch = () => new Promise(resolve => { resolvePlan = resolve; });
  run('copyCurrentToPlan()');
  const planning = run('planJoints()'); units('deg');
  resolvePlan({json: async () => ({ok: true, plan_id: 'unit-switch'})}); await planning;
  assert.equal(run('planId'), 'unit-switch');
  // A target edit during in-flight planning must still reject the late response.
  const superseded = run('planJoints()'); targets(-20, 0);
  resolvePlan({json: async () => ({ok: true, plan_id: 'late'})}); await superseded;
  assert.equal(run('planId'), null);
  pendingFetch = null;
  // Stale state still blocks execution and cannot be copied as valid targets.
  ready(); run('latestJointsAge = 3; updatePlanButtons()');
  assert.equal(element('executeButton').disabled, true);
  run('copyCurrentToPlan()'); assert.equal(read(), null);
  assert.equal(element('target_joint1').value, '');
  // Older state responses cannot undo the newest state; target edits beat ready state.
  const responses = [];
  pendingFetch = () => new Promise(resolve => responses.push(resolve));
  const first = run('poll()'), second = run('poll()');
  const snapshot = run(`({config, joints: latestJoints, joints_age: 0, plan: {state:'ready',plan_id:'ready'}, status: {}, tcp: null})`);
  responses[1]({ok: true, json: async () => snapshot}); await second;
  const applied = run('appliedPoll');
  responses[0]({ok: true, json: async () => ({...snapshot, joints_age: 10})}); await first;
  assert.equal(run('appliedPoll'), applied); assert.equal(run('latestJointsAge'), 0);
  ready(); const pollReady = run('poll()'); targets(10, 0);
  responses[2]({ok: true, json: async () => snapshot}); await pollReady;
  assert.equal(run('planReady'), false); assert.equal(run('planId'), null);
  console.log('PASS: actual GUI script unit conversion, payload, readiness, freshness and race checks');
}
main().catch(error => { console.error(error); process.exitCode = 1; });
