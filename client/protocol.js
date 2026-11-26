// Message builders for the Quest input listener. No DOM and no WebXR here, so
// Node can test them and check them against the real listener.

const clamp01 = (value) => Math.min(1, Math.max(0, value));

export function auth(token) {
  return { type: "auth", token };
}

// One controller's fields from a WebXR rigid transform and its buttons, or null
// when the pose is unusable. `squeezed` is the deadman; a pulled trigger closes
// the gripper, so the opening sent is 1 - trigger. `follow` false moves only the
// previewed goal: the real arm stays where it is until follow is true.
export function hand(transform, squeezed, trigger, scale, follow = true) {
  const p = transform?.position;
  const o = transform?.orientation;
  if (!p || !o) return null;
  const norm = Math.hypot(o.x, o.y, o.z, o.w);
  const numbers = [p.x, p.y, p.z, o.x, o.y, o.z, o.w, trigger, scale];
  if (!numbers.every(Number.isFinite) || !(norm > 0)) return null;
  return {
    position: [p.x, p.y, p.z],
    orientation: [o.x / norm, o.y / norm, o.z / norm, o.w / norm],
    grip: squeezed === true,
    gripper: clamp01(1 - trigger),
    scale: clamp01(scale),
    follow: follow === true,
  };
}

// Not input: it only lets the host deliver its state while no sample is flowing,
// for example before the headset view starts or while a hand is out of sight.
export function ping() {
  return { type: "ping" };
}

// Version 1: one controller.
export function pose(seq, fields) {
  return { type: "pose", version: 1, seq, ...fields };
}

// Version 2: both controllers. A hand that is not tracked is null.
export function frame(seq, left, right) {
  return { type: "pose", version: 2, seq, left: left ?? null, right: right ?? null };
}

// The message for this animation frame, or null when there is nothing to send.
// `mode` is "left", "right" or "both"; `hands` maps handedness to hand() results.
export function message(mode, seq, hands) {
  if (mode === "both") return frame(seq, hands.left, hands.right);
  const fields = hands[mode];
  return fields ? pose(seq, fields) : null;
}

export function expectedVersion(mode) {
  return mode === "both" ? 2 : 1;
}

// The host names its arms "" for one arm, or "left" and "right" for two.
export function armFor(mode, handedness) {
  return mode === "both" ? handedness : "";
}

// Whether this hand's squeeze counts as a grip when the virtual gripper must be
// grabbed. A grab starts only if the squeeze begins within reach of the gripper,
// and lasts until the squeeze ends. `memory` holds { held, squeezed } per hand.
export function grabbed(memory, squeezed, distance, reach = 0.12) {
  if (!squeezed) {
    memory.held = false;
  } else if (!memory.squeezed) {
    memory.held = Number.isFinite(distance) && distance <= reach;
  }
  memory.squeezed = squeezed;
  return memory.held;
}
