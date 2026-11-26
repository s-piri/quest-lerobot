import assert from "node:assert/strict";
import { test } from "node:test";

import { armFor, auth, expectedVersion, frame, grabbed, hand, message, ping, pose } from "./protocol.js";

const transform = (position = [1, 2, 3], orientation = [0, 0, 0, 1]) => ({
  position: { x: position[0], y: position[1], z: position[2] },
  orientation: { x: orientation[0], y: orientation[1], z: orientation[2], w: orientation[3] },
});

test("a hand carries exactly the documented fields", () => {
  const fields = hand(transform(), true, 0.25, 0.5);
  assert.deepEqual(Object.keys(fields).sort(), ["follow", "grip", "gripper", "orientation", "position", "scale"]);
  assert.deepEqual(fields, {
    position: [1, 2, 3],
    orientation: [0, 0, 0, 1],
    grip: true,
    gripper: 0.75,
    scale: 0.5,
    follow: true,
  });
});

test("follow is a strict boolean that defaults to true", () => {
  assert.equal(hand(transform(), true, 0, 1).follow, true);
  assert.equal(hand(transform(), true, 0, 1, false).follow, false);
  assert.equal(hand(transform(), true, 0, 1, 1).follow, false);
  assert.equal(hand(transform(), true, 0, 1, undefined).follow, true);
});

test("a grab starts only within reach and lasts until the squeeze ends", () => {
  const memory = { held: false, squeezed: false };
  assert.equal(grabbed(memory, true, 0.3), false); // squeezed too far away
  assert.equal(grabbed(memory, true, 0.05), false); // moving closer while squeezing does not grab
  assert.equal(grabbed(memory, false, 0.05), false);
  assert.equal(grabbed(memory, true, 0.05), true); // a fresh squeeze within reach
  assert.equal(grabbed(memory, true, 0.9), true); // it holds wherever the hand goes
  assert.equal(grabbed(memory, false, 0.9), false);
  assert.equal(grabbed(memory, true, NaN), false); // no arm is drawn: nothing to grab
  assert.equal(grabbed({ held: false, squeezed: false }, true, 0.12), true);
});

test("hands map to the host's arm names", () => {
  assert.equal(armFor("both", "left"), "left");
  assert.equal(armFor("both", "right"), "right");
  assert.equal(armFor("left", "left"), "");
  assert.equal(armFor("right", "right"), "");
});

test("the orientation is sent as a unit quaternion", () => {
  const { orientation } = hand(transform([0, 0, 0], [0, 0, 3, 4]), false, 0, 1);
  assert.deepEqual(orientation, [0, 0, 0.6, 0.8]);
});

test("grip is a strict boolean and the gripper and scale stay in range", () => {
  assert.equal(hand(transform(), 1, 0, 1).grip, false);
  assert.equal(hand(transform(), "true", 0, 1).grip, false);
  assert.equal(hand(transform(), true, 1.2, 1).gripper, 0);
  assert.equal(hand(transform(), true, -0.5, 1).gripper, 1);
  assert.equal(hand(transform(), true, 0, 4).scale, 1);
  assert.equal(hand(transform(), true, 0, -1).scale, 0);
});

test("an unusable pose is no hand at all", () => {
  assert.equal(hand(null, true, 0, 1), null);
  assert.equal(hand({ position: { x: 0, y: 0, z: 0 } }, true, 0, 1), null);
  assert.equal(hand(transform([NaN, 0, 0]), true, 0, 1), null);
  assert.equal(hand(transform([0, 0, 0], [0, 0, 0, 0]), true, 0, 1), null);
  assert.equal(hand(transform(), true, undefined, 1), null);
  assert.equal(hand(transform(), true, 0, Infinity), null);
});

test("version 1 is one flat controller message", () => {
  const fields = hand(transform(), false, 0, 0.25);
  assert.deepEqual(pose(7, fields), { type: "pose", version: 1, seq: 7, ...fields });
});

test("version 2 always names both hands", () => {
  const left = hand(transform(), true, 0, 1);
  assert.deepEqual(frame(3, left, undefined), {
    type: "pose",
    version: 2,
    seq: 3,
    left,
    right: null,
  });
  assert.deepEqual(Object.keys(frame(0)).sort(), ["left", "right", "seq", "type", "version"]);
});

test("the mode chooses the message, and a missing single hand sends nothing", () => {
  const left = hand(transform(), true, 0, 1);
  assert.equal(message("left", 1, { left }).version, 1);
  assert.equal(message("right", 1, { left }), null);
  assert.deepEqual(message("both", 1, { left }), frame(1, left, null));
  assert.deepEqual(message("both", 2, {}), frame(2, null, null));
  assert.equal(expectedVersion("both"), 2);
  assert.equal(expectedVersion("left"), 1);
  assert.equal(expectedVersion("right"), 1);
});

test("authentication is the first message", () => {
  assert.deepEqual(auth("secret"), { type: "auth", token: "secret" });
});

test("a ping is exactly what the host accepts as one", () => {
  assert.deepEqual(ping(), { type: "ping" });
});
