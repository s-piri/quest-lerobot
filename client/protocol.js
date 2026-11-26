// Message builders for the Quest input listener. No DOM and no WebXR here, so
// Node can test them and check them against the real listener.

const clamp01 = (value) => Math.min(1, Math.max(0, value));

export function auth(token) {
  return { type: "auth", token };
}

// One controller's fields from a WebXR rigid transform and its buttons, or null
// when the pose is unusable. `squeezed` is the deadman; a pulled trigger closes
// the gripper, so the opening sent is 1 - trigger.
export function hand(transform, squeezed, trigger, scale) {
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
  };
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
