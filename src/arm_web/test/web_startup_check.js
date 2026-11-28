// Startup regression: getElementById must return null before a row exists.
// This runs the actual page script with a bounded DOM, not a rendering engine.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
const markup = html.slice(0, html.indexOf('<script>'));
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
const headings = [...markup.matchAll(/<h2(?:\s[^>]*)?>([^<]+)<\/h2>/g)].map(match => match[1]);
assert.deepEqual(headings, ['Joint targets', 'TCP position', 'Cameras', 'Board settings', 'Gamepad']);
assert.match(markup, /label">Link<\/span>[\s\S]*?label">Board<\/span>[\s\S]*?label">Gamepad<\/span>/);
assert.doesNotMatch(markup, /class="(?:primary|direct-move)"/);
assert.doesNotMatch(markup, /<h3>Gamepad mapping<\/h3>|Editing pauses gamepad motion/);
assert.match(markup, /<section class="card right-card">\s*<h2>TCP position<\/h2>/);
assert.match(markup, /<section class="card right-card">\s*<h2>Gamepad<\/h2>/);
assert.match(markup, /<\/div>\s*<section class="card">\s*<h2>Cameras<\/h2>/);
assert.match(markup, /@media \(max-width: 1150px\)[\s\S]*?\.split > \.right-card \{ grid-column: auto; \}/);
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
    setAttribute(key, value) { this[key] = value; }, removeAttribute(key) { delete this[key]; },
    replaceChildren() {}, append() {}, focus() {}, contains(other) { return other === this; },
  };
  return element;
}

for (const match of markup.matchAll(/\bid="([^"]+)"/g))
  elements.set(match[1], makeElement(match[1]));
elements.get('speed').value = '30';
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
  gamepad_settings: {revision: 'initial', settings: {deadzone: .1, joints: [
    {control: 'left_x', invert: false}, {control: 'left_y', invert: true},
    {control: 'right_y', invert: true}, {control: 'right_x', invert: false},
    {control: 'lb_rb', invert: false},
  ]}},
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
      if (url === '/api/gamepad_settings') {
        state.gamepad_settings = {settings: JSON.parse(options.body).settings, revision: 'saved'};
        return {ok: true, json: async () => ({ok: true, ...state.gamepad_settings})};
      }
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
  assert.equal(element('age').textContent, 'not connected');
  assert.equal(element('age').className, 'bad');
  assert.ok(element('target_joint1'));
  assert.equal(element('jogPlus_joint1').disabled, true);
  assert.equal(element('planButton').disabled, true);
  assert.equal(element('motorsToggle').hidden, true);
  assert.equal(element('endstopHeading').hidden, true);
  assert.equal(element('runToggle').textContent, 'STOP');
  assert.equal(element('motorHeading').hidden, true);
  assert.equal(element('motor_joint1'), null);
  assert.equal(element('directButton').hidden, true);
  assert.equal(requests.length, 0);

  state.config.scene_ready = true;
  state.joints = Object.fromEntries(Object.keys(state.config.joint_limits_deg)
    .map(name => [name, {position_deg: 0, position_rad: 0, velocity: 0}]));
  state.joints_age = 0;
  state.tcp_error = null;
  state.tcp = {frame: 'tool0', age: 0, position_m: {x: 0, y: 0, z: .6}, rpy_deg: [0,0,0]};
  await run('poll()');
  assert.equal(element('conn').textContent, 'connected');
  assert.equal(element('jogPlus_joint1').disabled, false);
  assert.equal(element('tcpPlanButton').disabled, false);
  assert.equal(element('directButton').disabled, true);
  assert.match(markup, /id="speed" type="range" min="1" max="100"/);
  assert.equal(element('age').textContent, '0.00 s');
  state.config.joint_limits_deg.joint1 = [-30, 90];
  state.joints.joint1.position_deg = 0;
  await run('poll()');
  assert.equal(element('dot_joint1').style.left, '25%');
  assert.equal(element('scale_joint1').textContent, '-30.00°');
  assert.equal(element('scaleZero_joint1').textContent, '30.00°');
  assert.equal(element('scaleMax_joint1').textContent, '90.00°');
  state.joints.joint1.position_deg = 100;
  await run('poll()');
  assert.equal(element('dot_joint1').className, 'dot out');
  assert.equal(element('dot_joint1').style.left, '100%');
  state.joints.joint1.position_deg = 0;
  state.config.joint_limits_deg.joint1 = [-90, 90];
  state.camera2 = {available: true, age_s: 0};
  await run('poll()');
  assert.match(element('cam2Status').textContent, /phone connection unknown/);
  state.camera2 = {available: false, age_s: 3};
  await run('poll()');
  assert.equal(element('cam2').hidden, true);
  assert.match(element('cam2Status').textContent, /no new frames/);
  await run('startJog("joint1", 1)');
  assert.deepEqual(requests.pop(), {url: '/api/jog', body: {joint1: 36}}); // 30 % of 120 °/s
  await run('stopJog()'); requests.pop();

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
  assert.equal(element('age').textContent, 'not connected');
  assert.equal(element('age').className, 'bad');
  assert.equal(element('jogPlus_joint1').disabled, true);
  unavailable = false;
  await run('poll()');
  assert.equal(element('conn').textContent, 'connected');
  assert.equal(element('jogPlus_joint1').disabled, false);
  assert.equal(requests.length, 0);

  // Real bridge: motor status column, Enable/Disable buttons and Move all to targets.
  state.config.simulation = false;
  state.config.direct_motion_available = true;
  state.config.motors_control_available = true;
  state.status = {motors_enabled: {value: true, age: 0}, motors_powered: {value: 'joint1,joint3', age: 0}};
  await run('poll()');
  assert.equal(element('motorsToggle').hidden, false);
  assert.equal(element('motorHeading').hidden, false);
  assert.ok(element('motorsToggle').innerHTML.includes('Disable motors'));
  assert.equal(element('motor_joint1').textContent, 'ON');
  assert.equal(element('motor_joint2').textContent, 'sleep');
  assert.equal(element('motorsToggle').disabled, false);
  assert.equal(element('directButton').hidden, false);
  assert.equal(element('directButton').disabled, false);
  element('target_joint1').value = '10';
  await run('moveDirectly()');
  assert.equal(requests.at(-1).url, '/api/goto');
  assert.equal(Object.keys(requests.at(-1).body.positions).length, 6);
  assert.equal(requests.at(-1).body.positions.joint1, 10);
  assert.equal(requests.at(-1).body.speed_percent, 30);
  await run('toggleMotors()');
  assert.deepEqual(requests.at(-1), {url: '/api/motors', body: {enable: false}});
  state.status.motors_enabled.value = false;
  state.status.motors_powered.value = '';
  await run('poll()');
  assert.ok(element('motorsToggle').innerHTML.includes('Enable motors'));
  assert.equal(element('motor_joint1').textContent, 'OFF');
  assert.equal(element('motorsToggle').className, 'big motors-off');
  assert.equal(element('directButton').disabled, true);
  await run('toggleMotors()');
  assert.deepEqual(requests.at(-1), {url: '/api/motors', body: {enable: true}});
  state.status.motors_enabled.age = 5;
  await run('poll()');
  assert.ok(element('motorsToggle').innerHTML.includes('Motors unknown'));
  assert.equal(element('motorsToggle').disabled, true);
  assert.equal(element('motor_joint1').textContent, '–');

  // Endstops live in the joint table; STOP latches until START (bridge ~/armed).
  state.config.endstops = ['joint1', 'joint2'];
  state.status = {motors_enabled: {value: true, age: 0}, motors_powered: {value: '', age: 0},
                  limit_switches: {value: 'joint2', age: 0}, armed: {value: true, age: 0}};
  await run('poll()');
  assert.equal(element('endstopHeading').hidden, false);
  assert.equal(element('endstop_joint1').textContent, 'released');
  assert.equal(element('endstop_joint2').textContent, 'PRESSED');
  assert.equal(element('endstop_joint2').className, 'lamp on');
  assert.equal(element('endstop_joint3'), null);
  // Per-joint homing status lives in the joint table (bridge ~/homed + ~/homing_status).
  assert.equal(element('homingHeading').hidden, false);
  state.status.homed = {value: 'joint1', age: 0};
  state.status.homing_status = {value: 'joint2: homing failed: no switch', age: 1};
  await run('poll()');
  assert.equal(element('homed_joint1').textContent, 'homed');
  assert.equal(element('homed_joint2').textContent, 'failed');
  assert.equal(element('homed_joint3').textContent, 'not homed');
  delete state.status.homed;
  delete state.status.homing_status;
  assert.equal(element('runToggle').textContent, 'STOP');
  assert.equal(element('jogPlus_joint1').disabled, false);
  await run('toggleRun()');
  assert.equal(requests.at(-1).url, '/api/stop');
  assert.equal(element('runToggle').textContent, 'START');
  assert.equal(element('runToggle').disabled, true, 'a double click on STOP must not START');
  state.status.armed = {value: false, age: 3};
  run('startGuardUntil = 0');
  await run('poll()');
  assert.equal(element('runToggle').textContent, 'START');
  assert.equal(element('runToggle').className, 'big run-start');
  assert.equal(element('runToggle').disabled, false);
  assert.equal(element('jogPlus_joint1').disabled, true);
  assert.equal(element('directButton').disabled, true);
  const before = requests.length;
  await run("startJog('joint1', 1)");
  await run('moveDirectly()');
  assert.equal(requests.length, before, 'no motion request while stopped');
  await run('toggleRun()');
  assert.equal(requests.at(-1).url, '/api/unlock');
  state.status.armed = {value: true, age: 0};
  await run('poll()');
  assert.equal(element('runToggle').textContent, 'STOP');
  assert.equal(element('jogPlus_joint1').disabled, false);
  context.document.hasFocus = () => true;
  const pad = {id: 'test pad', mapping: 'standard', axes: [0, 0, 0, 0],
               buttons: Array.from({length: 8}, () => ({pressed: false, value: 0}))};
  context.navigator.getGamepads = () => [pad];
  run('padLoop()');
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(element('padStatus').textContent, 'connected');
  assert.equal(element('padStatus').className, 'ok');
  assert.equal(element('padinfo').hidden, true);
  assert.equal(element('padinfo').textContent, '');
  pad.axes[0] = .05; pad.axes[1] = .55; pad.buttons[5] = {pressed: true, value: 1};
  run('padLoop()');
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(requests.at(-1).body.mapped, true);
  assert.equal(requests.at(-1).body.axes[0], 0);
  assert.ok(Math.abs(requests.at(-1).body.axes[1] + .5) < 1e-10);
  assert.equal(requests.at(-1).body.axes[4], 1);
  for (const [i, control] of ['right_x', 'left_y', 'right_y', 'right_x', 'lt_rt'].entries()) {
    element('padControl' + i).value = control;
    element('padInvert' + i).checked = i === 0 || i === 1 || i === 2;
  }
  element('padDeadzone').value = '.2';
  run('padSettingsEdited(); padLoop()');
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(element('padStatus').textContent, 'connected');
  assert.match(element('padinfo').textContent, /mapping paused/);
  assert.deepEqual(requests.at(-1).body.axes, [0, 0, 0, 0, 0]);
  await run('savePadSettings()');
  assert.equal(state.gamepad_settings.settings.joints[0].control, 'right_x');
  assert.equal(state.gamepad_settings.settings.joints[4].control, 'lt_rt');
  run('padLoop()');
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(element('padStatus').textContent, 'connected');
  assert.match(element('padinfo').textContent, /release controls/);
  pad.axes = [0, 0, 0, 0]; pad.buttons[5] = {pressed: false, value: 0};
  run('padLoop()');
  await new Promise(resolve => setImmediate(resolve));
  pad.axes[2] = .6;
  pad.buttons[7] = {pressed: true, value: .6};
  run('padLoop()');
  await new Promise(resolve => setImmediate(resolve));
  assert.ok(Math.abs(requests.at(-1).body.axes[0] + .5) < 1e-10);
  assert.ok(Math.abs(requests.at(-1).body.axes[4] - .5) < 1e-10);
  run('padSettingsEdited()');
  pad.buttons[1] = {pressed: true, value: 1};
  run('padLoop()');
  await new Promise(resolve => setImmediate(resolve));
  assert.deepEqual(requests.at(-1).body.axes, [0, 0, 0, 0, 0]);
  assert.equal(requests.at(-1).body.buttons[1], 1);
  context.navigator.getGamepads = () => [];
  run('padLoop()');
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(element('padStatus').textContent, 'not connected');
  assert.equal(element('padStatus').className, 'bad');
  assert.equal(element('padinfo').hidden, true);
  assert.deepEqual(requests.at(-1).body.axes, [0, 0, 0, 0, 0]);
  console.log('PASS: initial empty DOM, delayed ROS state, new rows and HTTP reconnect');
}

main().catch(error => { console.error(error); process.exitCode = 1; });
