// Drives the real listener from Node with the same message builders the page
// uses. Usage: node smoke.mjs <ws-url> <token> <left|right|both>
// It prints one JSON line when a pressed grip has been accepted, then holds the
// connection open until a line arrives on stdin.

import { auth, expectedVersion, hand, message } from "./protocol.js";

const [url, token, mode] = process.argv.slice(2);
const transform = (x) => ({
  position: { x, y: 1.25, z: -0.5 },
  orientation: { x: 0, y: 0, z: 0, w: 1 },
});
const fail = (reason) => {
  console.error(reason);
  process.exit(1);
};

setTimeout(() => fail("timed out"), 15000);

const socket = new WebSocket(url);
const replies = [];
let waiting = null;
socket.addEventListener("message", (event) => {
  const reply = JSON.parse(event.data);
  if (waiting) {
    waiting(reply);
    waiting = null;
  } else {
    replies.push(reply);
  }
});
socket.addEventListener("close", (event) => fail(`closed early: ${event.code} ${event.reason}`));
socket.addEventListener("error", () => fail("socket error"));
const next = () =>
  replies.length ? Promise.resolve(replies.shift()) : new Promise((resolve) => (waiting = resolve));

await new Promise((resolve) => socket.addEventListener("open", resolve));
socket.send(JSON.stringify(auth(token)));
const ready = await next();
if (ready.type !== "ready" || ready.version !== expectedVersion(mode)) {
  fail(`unexpected reply to authentication: ${JSON.stringify(ready)}`);
}

// Released first, then pressed and moved: the only order that can engage.
const frames = [
  { squeezed: false, x: 0.1 },
  { squeezed: true, x: 0.1 },
  { squeezed: true, x: 0.3 },
];
let seq = 0;
for (const { squeezed, x } of frames) {
  const fields = hand(transform(x), squeezed, 0.25, 0.5);
  const hands = mode === "both" ? { left: fields, right: fields } : { [mode]: fields };
  socket.send(JSON.stringify(message(mode, seq, hands)));
  const reply = await next();
  if (reply.type !== "accepted" || reply.seq !== seq) {
    fail(`sample ${seq} was not accepted: ${JSON.stringify(reply)}`);
  }
  seq += 1;
}

console.log(JSON.stringify({ session: ready.session, version: ready.version, sent: seq }));
process.stdin.once("data", () => process.exit(0));
