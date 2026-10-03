// End-to-end: ConversationAgent (mocked Claude API) -> tools -> CommandBus ->
// SimRobot with obstacles, plus scan and the voice parser on the same bus.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CommandBus, makeCommand } from '../js/bus.js';
import { SimRobot } from '../js/robot-sim.js';
import { scan, findOpenings, describeScan, driveToward } from '../js/scan.js';
import { TOOLS, createToolExecutor } from '../js/tools.js';
import { ConversationAgent } from '../js/agent.js';
import { parseUtterance } from '../js/voice.js';

const stub = () => {};

async function setup() {
  const sim = new SimRobot({ log: stub, onStatus: stub, timeScale: 20 });
  const bus = new CommandBus({ log: stub });
  bus.setRobot(sim);
  await sim.connect();
  return { sim, bus };
}

function claude(...replies) {
  const requests = [];
  const fetchImpl = async (url, init) => {
    const body = JSON.parse(init.body);
    requests.push(body);
    const r = replies.shift();
    if (!r) throw new Error('unexpected request');
    const content = typeof r === 'function' ? r(body) : r;
    const stop = content.some((b) => b.type === 'tool_use') ? 'tool_use' : 'end_turn';
    const msg = { id: `m${requests.length}`, type: 'message', role: 'assistant', content, stop_reason: stop };
    return { ok: true, status: 200, headers: { get: () => null }, json: async () => msg };
  };
  return { fetchImpl, requests };
}

const use = (id, name, input = {}) => ({ type: 'tool_use', id, name, input });

function makeAgent(bus, fetchImpl, extra = {}) {
  const executor = createToolExecutor({ bus, makeCommand, scan, findOpenings, describeScan, driveToward, ...extra });
  return new ConversationAgent({ apiKey: 'test-key', tools: TOOLS, executor, fetchImpl });
}

test('conversation: scan, pick the opening the model chose, drive there', async () => {
  const { sim, bus } = await setup();
  const scans = [];
  const { fetchImpl, requests } = claude(
    [{ type: 'text', text: 'Ich schaue mich um.' }, use('t1', 'express_emotion', { emotion: 'curious' }), use('t2', 'scan_surroundings', { steps: 8 })],
    (body) => {
      // the model sees the scan result and drives toward the best opening
      const result = body.messages.at(-1).content.find((c) => c.tool_use_id === 't2');
      const data = JSON.parse(typeof result.content === 'string' ? result.content : result.content[0].text);
      const best = data.openings?.[0] ?? { angle: 0 };
      return [use('t3', 'drive_toward', { angle: best.angle, cm: 30 })];
    },
    [{ type: 'text', text: 'Dort ist Platz, ich bin hingefahren.' }],
  );
  const agent = makeAgent(bus, fetchImpl, { onScan: (p) => scans.push(p) });
  const start = { x: sim.state.x, y: sim.state.y };
  const r = await agent.send('Schau dich um und fahr dahin, wo Platz ist');
  await sim.disconnect();

  assert.equal(requests.length, 3);
  assert.equal(scans.length, 1, 'onScan called once');
  assert.equal(scans[0].length, 8);
  assert.match(r.text, /hingefahren/);
  const moved = Math.hypot(sim.state.x - start.x, sim.state.y - start.y);
  assert.ok(moved > 15, `robot moved ${moved.toFixed(1)} cm`);
  // tool results are paired with their tool_use ids
  const ids = requests[1].messages.at(-1).content.map((c) => c.tool_use_id);
  assert.deepEqual(ids, ['t1', 't2']);
});

test('safety: the model cannot drive the robot into an obstacle', async () => {
  const { sim, bus } = await setup();
  // face the sofa (top-left box): start below it, heading up
  sim.state.x = 65; sim.state.y = 90; sim.state.heading = -90;
  const { fetchImpl } = claude(
    [use('a', 'move_straight', { cm: 100 })],
    [{ type: 'text', text: 'ok' }],
  );
  const agent = makeAgent(bus, fetchImpl);
  await agent.send('Fahr einen Meter nach vorne');
  const d = await sim.distance();
  await sim.disconnect();
  assert.ok(d >= 14, `stopped ${d} cm before the sofa`);
});

test('abort during a tool stops the loop and the robot', async () => {
  const { sim, bus } = await setup();
  const { fetchImpl, requests } = claude(
    [use('s', 'scan_surroundings', { steps: 16 })],
    [{ type: 'text', text: 'should not be requested' }],
  );
  const agent = makeAgent(bus, fetchImpl);
  const ctl = new AbortController();
  setTimeout(() => ctl.abort(), 150);
  const r = await agent.send('Schau dich um', { signal: ctl.signal }).catch((e) => ({ error: e }));
  await new Promise((res) => setTimeout(res, 100));
  const motion = sim.motion;
  await sim.disconnect();
  assert.equal(requests.length, 1, 'no further model request after abort');
  assert.ok(r.aborted || r.error, 'reported as aborted');
  assert.equal(motion, null, 'robot is not moving');
});

test('command-mode voice and the AI share one bus', async () => {
  const { sim, bus } = await setup();
  const p = parseUtterance('rechts 90 Grad');
  await bus.submit(makeCommand(p.cmd, { ...p.args, wait: true }, 'voice'));
  await sim.disconnect();
  assert.ok(Math.abs(((sim.state.heading + 360) % 360) - 0) < 2, `heading ${sim.state.heading}`);
});
