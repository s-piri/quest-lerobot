import assert from "node:assert/strict";
import { test } from "node:test";

import { addressProblem, alerts, closeMessage, explainArm, plain, statusLines, wrap } from "./guide.js";

const arm = (state, reason = state, extra = {}) => ({ state, reason, base: [0, 0, 0, 1], measured: null, goal: null, ...extra });

test("every host state has guidance, and none of it is the host's raw wording", () => {
  const cases = [
    arm("disarmed"),
    arm("armed", "ready"),
    arm("armed", "release the grip before pressing it"),
    arm("armed", "no fresh input"),
    arm("engaged", "tracking"),
    arm("engaged", "previewing"),
    arm("engaged", "inverse kinematics rejected the target"),
    arm("stopped", "stop requested"),
    arm("faulted", "measured feedback is stale"),
    arm("demo"),
    arm("something new"),
    undefined,
  ];
  for (const item of cases) {
    const line = explainArm(item);
    assert.ok(["go", "wait", "warn", "stop"].includes(line.tone));
    assert.ok(line.text.length > 10);
    assert.doesNotMatch(line.text, /inverse kinematics|fresh input|disarmed|stale/);
  }
});

test("a disarmed arm says why nothing is drawn and what will change that", () => {
  const line = explainArm(arm("disarmed"));
  assert.equal(line.tone, "wait");
  assert.match(line.text, /nothing will move/);
  assert.match(line.text, /appears here once the host arms it/);
});

test("ready guidance matches how the operator is set up to move", () => {
  assert.match(explainArm(arm("armed", "ready"), { grab: true }).text, /virtual gripper/);
  assert.doesNotMatch(explainArm(arm("armed", "ready"), { grab: false }).text, /virtual gripper/);
  assert.match(explainArm(arm("engaged", "previewing"), { button: "X" }).text, /Hold X /);
  assert.match(explainArm(arm("engaged", "previewing")).text, /Hold A /);
  assert.equal(explainArm(arm("armed", "no fresh input"), { tracked: false }).tone, "warn");
  assert.equal(explainArm(arm("armed", "no fresh input"), { tracked: true }).tone, "wait");
});

test("stops and faults say they need clearing on the host, in plain words", () => {
  const stopped = explainArm(arm("stopped", "operator pressed stop"));
  assert.equal(stopped.tone, "stop");
  assert.match(stopped.text, /^Stopped: operator pressed stop\. .*cleared on the host/);
  const fault = explainArm(arm("faulted", "measured feedback is stale"));
  assert.match(fault.text, /^Fault: the host lost the arm's position readings\./);
  assert.equal(
    plain("right faulted: measured feedback is stale"),
    "the right arm had a fault (the host lost the arm's position readings)",
  );
  assert.equal(plain("left stopped: stop requested"), "the left arm was stopped (a stop was requested)");
  assert.equal(plain("something the host made up"), "something the host made up");
  assert.equal(plain(undefined), "");
});

test("status lines cover the whole journey", () => {
  assert.deepEqual(statusLines({ connection: "connecting" }), [{ tone: "wait", text: "Connecting to the host…" }]);
  const offline = statusLines({ connection: "offline", arms: {} });
  assert.equal(offline[0].tone, "warn");
  assert.match(offline[0].text, /Not connected/);
  // The example arm shown before connecting is not reported as a lost connection.
  const example = statusLines({ connection: "offline", arms: { "": arm("demo") } });
  assert.equal(example.length, 1);
  assert.match(example[0].text, /Example pose/);
  const waiting = statusLines({ connection: "paired", arms: {} });
  assert.match(waiting[0].text, /Waiting for the host to report the arm/);
});

test("two arms are named, and each hand gets its own button and tracking", () => {
  const lines = statusLines({
    connection: "paired",
    mode: "both",
    inHeadset: true,
    tracked: { left: false, right: true },
    arms: { right: arm("engaged", "previewing"), left: arm("armed", "no fresh input") },
  });
  assert.equal(lines.length, 2);
  assert.match(lines[0].text, /^Left arm: The controller is not tracked/);
  assert.match(lines[1].text, /^Right arm: Previewing.*Hold A /);
  const single = statusLines({ connection: "paired", mode: "left", arms: { "": arm("engaged", "previewing") } });
  assert.match(single[0].text, /^Previewing.*Hold X /);
});

test("a stale picture is called out first, and only once paired with an arm", () => {
  const view = { connection: "paired", arms: { "": arm("armed", "ready") } };
  assert.equal(statusLines({ ...view, staleSeconds: 0.4 }).length, 1);
  const stale = statusLines({ ...view, staleSeconds: 3.4 });
  assert.equal(stale.length, 2);
  assert.match(stale[0].text, /No update from the host for 3 s/);
  assert.equal(stale[0].tone, "warn");
});

test("connection failures say what to do", () => {
  assert.match(closeMessage(1008, "authentication required"), /pairing token.*changes every time/);
  assert.match(closeMessage(1013, "input already has an owner"), /already connected/);
  assert.match(closeMessage(1008, "invalid input"), /Reload the page/);
  assert.match(closeMessage(1001, "input shutting down"), /stopped listening/);
  assert.match(closeMessage(1006, ""), /certificate/);
  assert.equal(closeMessage(1000, ""), "Disconnected.");
  assert.equal(closeMessage(4000, "odd"), "Disconnected (code 4000: odd).");
});

test("an address is checked before it is tried", () => {
  assert.match(addressProblem("", true), /Enter the listener address/);
  assert.match(addressProblem("   ", true), /Enter the listener address/);
  assert.match(addressProblem("host:8765", true), /wss:\/\//);
  assert.match(addressProblem("https://host:8765", true), /must start with wss/);
  assert.match(addressProblem("not an address", false), /not a complete address/);
  assert.match(addressProblem("ws://192.168.1.20:8765", true), /only allows a wss/);
  assert.equal(addressProblem("ws://192.168.1.20:8765", false), null);
  assert.equal(addressProblem("ws://127.0.0.1:8765", true), null);
  assert.equal(addressProblem("ws://localhost:8765", true), null);
  assert.equal(addressProblem(" wss://robot.local:8765 ", true), null);
});

test("text wraps on words to the width it is given", () => {
  const measure = (text) => text.length;
  assert.deepEqual(wrap(measure, "one two three four", 9), ["one two", "three", "four"]);
  assert.deepEqual(wrap(measure, "short", 20), ["short"]);
  assert.deepEqual(wrap(measure, "", 20), []);
  assert.deepEqual(wrap(measure, "unbreakablylongword next", 5), ["unbreakablylongword", "next"]);
});

test("only a change worth feeling raises an alert", () => {
  const moving = { "": arm("engaged", "tracking") };
  assert.deepEqual(alerts(moving, moving), []);
  assert.deepEqual(alerts(moving, { "": arm("stopped", "stop requested") }), [{ arm: "", kind: "stop" }]);
  assert.deepEqual(alerts({ "": arm("stopped") }, { "": arm("faulted") }), []); // already halted
  const out = { "": arm("engaged", "inverse kinematics rejected the target") };
  assert.deepEqual(alerts(moving, out), [{ arm: "", kind: "reach" }]);
  assert.deepEqual(alerts(out, out), []);
  assert.deepEqual(alerts({}, { left: arm("faulted"), right: arm("armed", "ready") }), [{ arm: "left", kind: "stop" }]);
  assert.deepEqual(alerts(undefined, undefined), []);
});
