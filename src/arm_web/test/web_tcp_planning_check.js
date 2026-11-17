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
  window: {addEventListener() {}}, navigator: {}, Date, confirm: () => true,
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
run(`config = {joint_limits_deg: {joint1: [-180, 180]}, home_joints: [], max_jog_speed: 360,
  execution_available: true, selected_tcp: 'gripper', tcp_frame: 'tcp_gripper', reference_frame: 'world',
  tcps: [{name: 'gripper', xyz_m: [0,0,0.05], rpy_deg: [0,0,0]}]};
  connected = true; latestJoints = {joint1: {position_deg: 0}};
  latestJointsAge = 0; latestJointsReceived = Date.now();
  latestTcp = {frame: 'tcp_gripper', age: 0, rpy_deg: [0,0,0], position_m: {x: .123456, y: -.05, z: .3}};
  latestTcpName = 'gripper'; updateTcpControls(); updatePlanButtons();`);
const xyz = (x, y, z) => {
  for (const [i, value] of [x, y, z].entries()) element('tcpTarget' + 'XYZ'[i]).value = value;
  run('tcpTargetChanged()');
};
const ready = () => run(`planId = 'tcp-ready'; planKind = 'tcp'; planReady = true; updatePlanButtons()`);
function state(id = 'tcp-ready') {
  return JSON.parse(run(`JSON.stringify({config, joints: latestJoints, joints_age: 0, tcp: latestTcp,
    status: {}, plan: {state: 'ready', plan_id: '${id}'}})`));
}
async function main() {
  assert.equal(element('tcpTargetX').value, '');
  assert.equal(element('tcpPlanButton').disabled, false);
  for (const invalid of ['', 'NaN', 'Infinity', '1e309', 'abc']) {
    xyz(invalid, 0, 10);
    const before = requests.length;
    await run('planTcp()'); assert.equal(requests.length, before);
    assert.equal(run('planId'), null);
  }
  xyz(10, 20, 30); element('tcpTargetX').validity.badInput = true;
  await run('planTcp()'); assert.equal(run('planId'), null);
  element('tcpTargetX').validity.badInput = false;
  run('copyCurrentTcpToPlan()');
  assert.equal(element('tcpTargetX').value, '123.456');
  assert.equal(element('tcpTargetY').value, '-50');
  xyz(0, -42.5, 123.456789);
  for (const units of ['rad', 'deg']) {
    run(`setJointUnits('${units}')`);
    await run('planTcp()');
    assert.deepEqual(requests.at(-1), {url: '/api/plan_tcp', body: {tcp_name: 'gripper', x_mm: 0, y_mm: -42.5, z_mm: 123.456789}});
    assert.equal(run('planReady'), false);
    assert.equal(element('tcpExecuteButton').disabled, true);
    pendingFetch = async () => ({ok: true, json: async () => state('preview')});
    await run('poll()'); pendingFetch = null;
    assert.equal(run('planReady'), true);
    assert.equal(element('tcpExecuteButton').disabled, false);
    assert.equal(element('executeButton').disabled, true);
    assert.equal(element('tcpTargetZ').value, '123.456789');
  }
  // Only the matching card can execute the shared stored plan ID.
  const before = requests.length;
  await run('executeJoints("joints")'); assert.equal(requests.length, before);
  await run('executeJoints("tcp")');
  assert.deepEqual(requests.at(-1), {url: '/api/execute', body: {plan_id: 'preview'}});
  assert.equal(run('planId'), null);
  run(`planKind = 'joints'; planId = 'joint-ready'; planReady = true; updatePlanButtons()`);
  await run('executeJoints("tcp")'); assert.equal(requests.at(-1).body.plan_id, 'preview');
  // Stale or mismatched TF/joints blocks copy, planning and execution.
  for (const stale of ["latestTcp.age = 3", "latestTcp.frame = 'old'", "latestTcpName = 'old'", "latestTcpError = 'missing'", "latestJointsAge = 3", "latestTcp.position_m.x = NaN"]) {
    run(`latestTcp.age = 0; latestTcp.frame = 'tcp_gripper'; latestTcpName = 'gripper'; latestTcpError = null;
      latestTcp.position_m.x = .123456; latestJointsAge = 0; latestJointsReceived = Date.now(); ${stale}`);
    ready(); assert.equal(element('tcpExecuteButton').disabled, true);
    const prior = requests.length; const text = element('tcpTargetX').value;
    await run('planTcp()'); await run('executeJoints("tcp")'); run('copyCurrentTcpToPlan()');
    assert.equal(requests.length, prior); assert.equal(element('tcpTargetX').value, text);
  }
  run(`latestTcp.position_m.x = .123456; latestJointsAge = 0; latestJointsReceived = Date.now()`);
  // Edits, definition edits and TCP selection all defeat a late planning reply.
  for (const action of ['tcpTargetChanged()', 'tcpDefinitionChanged()', 'newTcp()', 'selectTcp("gripper")', 'saveTcp()', 'deleteTcp()']) {
    let resolvePlan;
    pendingFetch = (url) => url === '/api/plan_tcp'
      ? new Promise(resolve => { resolvePlan = resolve; })
      : Promise.resolve({json: async () => ({ok: false, message: 'fixture mutation rejected'})});
    run(`tcpControlsPending = 0; tcpAwaitingState = false; latestJointsReceived = Date.now()`);
    xyz(1, 2, 3);
    element('tcpSelect').value = 'gripper'; element('tcpName').value = 'gripper';
    for (const id of ['tcpOffsetX','tcpOffsetY','tcpOffsetZ','tcpRoll','tcpPitch','tcpYaw']) element(id).value = '0';
    const promise = run('planTcp()'); assert.equal(typeof resolvePlan, 'function');
    await run(action);
    resolvePlan({json: async () => ({ok: true, plan_id: 'late'})}); await promise;
    assert.equal(run('planId'), null, action); assert.equal(element('tcpExecuteButton').disabled, true);
  }
  // Successful mutation waits for a post-response poll, ignoring an earlier in-flight poll.
  pendingFetch = null;
  const polls = [];
  pendingFetch = url => url === '/state' ? new Promise(resolve => polls.push(resolve))
    : Promise.resolve({json: async () => ({ok: true})});
  const oldPoll = run('poll()'); await run('selectTcp("gripper")');
  assert.equal(run('freshTcpState()'), false);
  polls.shift()({ok: true, json: async () => state()}); await oldPoll;
  assert.equal(run('freshTcpState()'), false);
  const newPoll = run('poll()'); polls.shift()({ok: true, json: async () => state()}); await newPoll;
  assert.equal(run('freshTcpState()'), true);
  // Externally changed TCP definitions invalidate the existing Cartesian preview.
  ready(); pendingFetch = async () => {
    const s = state(); s.config.tcps[0].xyz_m[2] = .09;
    return {ok: true, json: async () => s};
  };
  await run('poll()'); assert.equal(run('planId'), null);
  assert.equal(element('tcpTargetZ').value, '3');
  console.log('PASS: actual GUI TCP XYZ payload, shared-plan scope, freshness and mutation races');
}
main().catch(error => { console.error(error); process.exitCode = 1; });
