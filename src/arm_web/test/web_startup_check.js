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
  assert.equal(Object.keys(requests.at(-1).body).length, 6);
  assert.equal(requests.at(-1).body.joint1, 10);
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
  console.log('PASS: initial empty DOM, delayed ROS state, new rows and HTTP reconnect');
}

main().catch(error => { console.error(error); process.exitCode = 1; });
