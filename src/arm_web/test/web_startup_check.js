// Startup regression: getElementById must return null before a row exists.
// This runs the actual page script with a bounded DOM, not a rendering engine.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
const markup = html.slice(0, html.indexOf('<script>'));
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
const elements = new Map();

function makeElement(id = '') {
  const element = {
    id, _value: '', _html: '', descendants: [], style: {}, textContent: '',
    disabled: false, hidden: false, validity: {badInput: false},
    get value() { return this._value; }, set value(value) { this._value = String(value); },
    get innerHTML() { return this._html; },
    set innerHTML(value) {
      for (const child of this.descendants) elements.delete(child);
      this.descendants = [];
      this._html = value;
      for (const match of value.matchAll(/\bid="([^"]+)"/g)) {
        this.descendants.push(match[1]);
        elements.set(match[1], makeElement(match[1]));
      }
    },
    setAttribute(key, value) { this[key] = value; },
    replaceChildren() {}, append() {}, focus() {},
  };
  return element;
}

for (const match of markup.matchAll(/\bid="([^"]+)"/g))
  elements.set(match[1], makeElement(match[1]));
elements.get('speed').value = '120';
const element = id => elements.get(id) || null;
assert.equal(element('jogPlus_joint1'), null);

const state = {
  config: {
    home_joints: [], max_jog_speed: 120, simulation: true, execution_available: true,
    direct_motion_available: false, jog_available: true, jog_active: false,
    scene_ready: false, demo_target_mm: [280, 35, 139], velocity_scaling: .3,
    reference_frame: 'world', selected_tcp: 'tool0', tcp_frame: 'tool0', endstops: [],
    joint_limits_deg: Object.fromEntries([1,2,3,4,5].map(i => ['joint' + i, [-90, 90]])),
    tcps: [{name: 'tool0', frame: 'tool0', xyz_m: [0,0,0], rpy_deg: [0,0,0]}],
  },
  joints: {}, joints_age: null, tcp: null, tcp_error: 'Waiting for TF', status: {},
  plan: {state: 'none', message: 'No plan yet.'},
  cameras: Array.from({length: 8}, () => ({available: false, age_s: null})),
};
let unavailable = false;
const requests = [];
const context = vm.createContext({
  console, document: {getElementById: element, createElement: () => makeElement(), hasFocus: () => false},
  window: {addEventListener() {}}, navigator: {}, Date, confirm: () => true,
  setInterval: () => 1, clearInterval() {},
  fetch: async (url, options) => {
    if (options) {
      requests.push({url, body: JSON.parse(options.body)});
      return {ok: true, json: async () => ({ok: true})};
    }
    if (unavailable) throw new Error('fixture server unavailable');
    return {ok: true, json: async () => JSON.parse(JSON.stringify(state))};
  },
});
vm.runInContext(script.replace('setInterval(poll, 200);\npoll();',
  'setInterval(poll, 200);\nglobalThis.startupPoll = poll();'), context);
const run = code => vm.runInContext(code, context);

async function main() {
  await context.startupPoll;
  assert.equal(element('conn').textContent, 'connected');
  assert.ok(element('target_joint1'));
  assert.equal(element('jogPlus_joint1').disabled, true);
  assert.equal(element('planButton').disabled, true);
  assert.equal(element('camerasCard').hidden, false);
  assert.match(element('cameras').innerHTML, /<h3>Camera 8<\/h3>/);
  assert.equal(element('camera_9'), null);
  assert.match(element('cameraStatus_1').textContent, /Waiting for frames/);
  assert.equal(requests.length, 0);

  state.config.scene_ready = true;
  state.cameras[0] = {available: true, age_s: 0};
  state.cameras[7] = {available: true, age_s: 0};
  state.joints = Object.fromEntries(Object.keys(state.config.joint_limits_deg)
    .map(name => [name, {position_deg: 0, position_rad: 0, velocity: 0}]));
  state.joints_age = 0;
  state.tcp_error = null;
  state.tcp = {frame: 'tool0', age: 0, position_m: {x: 0, y: 0, z: .6}, rpy_deg: [0,0,0]};
  await run('poll()');
  assert.equal(element('conn').textContent, 'connected');
  assert.equal(element('jogPlus_joint1').disabled, false);
  assert.equal(element('tcpPlanButton').disabled, false);
  assert.equal(element('cameraStatus_1').textContent, 'Live');
  run('loadNextCameraFrame(); loadNextCameraFrame()');
  assert.equal(element('camera_8').src, undefined);
  element('camera_1').onload();
  assert.equal(element('camera_1').hidden, false);
  assert.match(element('camera_1').src, /^\/camera_1\.jpg\?/);
  run('loadNextCameraFrame()');
  assert.match(element('camera_8').src, /^\/camera_8\.jpg\?/);
  element('camera_8').onload();
  state.cameras = [];
  await run('poll()');
  assert.equal(element('camerasCard').hidden, true);
  assert.equal(element('camera_1'), null);
  assert.equal(element('directButton').disabled, true);
  assert.equal(Number(element('speed').max), 120);

  // A model change also updates controls before every new row exists.
  state.config.joint_limits_deg.joint6 = [-90, 90];
  state.joints.joint6 = {position_deg: 0, position_rad: 0, velocity: 0};
  assert.equal(element('jogPlus_joint6'), null);
  await run('poll()');
  assert.equal(element('conn').textContent, 'connected');
  assert.equal(element('jogPlus_joint6').disabled, false);

  unavailable = true;
  await run('poll()');
  assert.equal(element('conn').textContent, 'disconnected');
  assert.equal(element('jogPlus_joint1').disabled, true);
  unavailable = false;
  await run('poll()');
  assert.equal(element('conn').textContent, 'connected');
  assert.equal(element('jogPlus_joint1').disabled, false);
  assert.equal(requests.length, 0);
  console.log('PASS: initial empty DOM, delayed ROS state, new rows and HTTP reconnect');
}

main().catch(error => { console.error(error); process.exitCode = 1; });
