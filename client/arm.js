// The arm model's maths: where each link sits for given joint values, and the
// matrices to draw it. No DOM and no WebGL here, so Node can test it against the
// same reference poses as the host's kinematics.
//
// Quaternions are [x, y, z, w]. A pose is { position: [x, y, z], orientation }.
// Matrices are 16 numbers in column-major order, as WebGL expects.

export const ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"];
export const JAW_JOINT = "gripper";
export const TOOL_LINK = "gripper_frame_link";
export const IDENTITY = [0, 0, 0, 1];

export function multiply(a, b) {
  const [ax, ay, az, aw] = a;
  const [bx, by, bz, bw] = b;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

export function conjugate(q) {
  return [-q[0], -q[1], -q[2], q[3]];
}

export function rotate(q, v) {
  const [x, y, z] = multiply(multiply(q, [v[0], v[1], v[2], 0]), conjugate(q));
  return [x, y, z];
}

export function about(axis, angle) {
  const s = Math.sin(angle / 2);
  return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(angle / 2)];
}

// The pose `inner` expressed in the frame that `outer` is expressed in.
export function compose(outer, inner) {
  const moved = rotate(outer.orientation, inner.position);
  return {
    position: [
      outer.position[0] + moved[0],
      outer.position[1] + moved[1],
      outer.position[2] + moved[2],
    ],
    orientation: multiply(outer.orientation, inner.orientation),
  };
}

// The host's state lists the five arm joints in radians, then the gripper
// opening from 0 to 1. The jaw angle is a straight-line guess between the
// model's jaw limits: good enough to see open from closed, not a measurement.
export function jointValues(scene, state) {
  const values = {};
  ARM_JOINTS.forEach((name, index) => (values[name] = state[index]));
  const jaw = scene.joints.find((joint) => joint.name === JAW_JOINT);
  if (jaw) values[JAW_JOINT] = jaw.lower + state[5] * (jaw.upper - jaw.lower);
  return values;
}

// Every link's pose in the arm's base frame. The manifest lists joints parents first.
export function linkPoses(scene, values) {
  const poses = { [scene.base]: { position: [0, 0, 0], orientation: IDENTITY } };
  for (const joint of scene.joints) {
    let pose = compose(poses[joint.parent], joint);
    if (joint.axis) {
      const turned = about(joint.axis, values[joint.name] ?? 0);
      pose = { position: pose.position, orientation: multiply(pose.orientation, turned) };
    }
    poses[joint.child] = pose;
  }
  return poses;
}

// Where the arm's base sits in the headset's space. `base` is the host's
// rotation from headset space to the arm's base frame, so the drawn arm turns
// the opposite way: a hand moving right then moves the drawn gripper right.
export function placement(base, position) {
  return { position: [...position], orientation: conjugate(base) };
}

export function toolPosition(scene, values, placed) {
  return compose(placed, linkPoses(scene, values)[TOOL_LINK]).position;
}

// The base position that puts the tool at `wanted` for these joint values.
export function placeToolAt(scene, values, base, wanted) {
  const offset = toolPosition(scene, values, placement(base, [0, 0, 0]));
  return placement(base, [wanted[0] - offset[0], wanted[1] - offset[1], wanted[2] - offset[2]]);
}

export function matrix(pose) {
  const [x, y, z, w] = pose.orientation;
  const [px, py, pz] = pose.position;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w), 0,
    2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w), 0,
    2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y), 0,
    px, py, pz, 1,
  ];
}

export function matMul(a, b) {
  const out = new Array(16);
  for (let column = 0; column < 4; column += 1) {
    for (let row = 0; row < 4; row += 1) {
      let sum = 0;
      for (let k = 0; k < 4; k += 1) sum += a[k * 4 + row] * b[column * 4 + k];
      out[column * 4 + row] = sum;
    }
  }
  return out;
}

// One entry per mesh to draw: its file, its colour and its matrix in headset space.
export function drawList(scene, values, placed) {
  const poses = linkPoses(scene, values);
  return scene.visuals.map((visual) => ({
    mesh: visual.mesh,
    color: visual.color,
    matrix: matrix(compose(placed, compose(poses[visual.link], visual))),
  }));
}

// The matrix `m` followed by a scale along its own axes.
export function scaled(m, sx, sy, sz) {
  const out = [...m];
  for (let row = 0; row < 4; row += 1) {
    out[row] *= sx;
    out[4 + row] *= sy;
    out[8 + row] *= sz;
  }
  return out;
}

// A ball of this radius at a point: for the grab point and the controllers.
export function markerMatrix(position, radius) {
  return scaled(matrix({ position, orientation: IDENTITY }), radius, radius, radius);
}

// A flat panel of this size at `position`, upright and turned to face `target`.
export function facing(position, target, width, height) {
  const yaw = Math.atan2(target[0] - position[0], target[2] - position[2]);
  return scaled(matrix({ position, orientation: about([0, 1, 0], yaw) }), width, height, 1);
}

// A unit ball made by splitting an octahedron's faces. Its normals point outward.
export function sphereMesh(detail = 2) {
  let faces = [];
  const corners = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
  for (const [a, b, c] of [[0, 2, 4], [2, 1, 4], [1, 3, 4], [3, 0, 4], [2, 0, 5], [1, 2, 5], [3, 1, 5], [0, 3, 5]]) {
    faces.push([corners[a], corners[b], corners[c]]);
  }
  const unit = (v) => {
    const length = Math.hypot(v[0], v[1], v[2]);
    return [v[0] / length, v[1] / length, v[2] / length];
  };
  const middle = (a, b) => unit([a[0] + b[0], a[1] + b[1], a[2] + b[2]]);
  for (let level = 0; level < detail; level += 1) {
    const next = [];
    for (const [a, b, c] of faces) {
      const ab = middle(a, b);
      const bc = middle(b, c);
      const ca = middle(c, a);
      next.push([a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]);
    }
    faces = next;
  }
  const flat = new Float32Array(faces.flat(2));
  return { positions: flat, normals: flat.slice(), count: faces.length };
}

// Triangles of a binary STL file as flat arrays of corner positions and normals.
// Normals are recomputed from the corners, so flat shading never depends on the file's.
export function parseStl(buffer) {
  const view = new DataView(buffer);
  if (buffer.byteLength < 84) throw new Error("not a binary STL file");
  const count = view.getUint32(80, true);
  if (buffer.byteLength !== 84 + count * 50) throw new Error("the STL file is truncated");
  const positions = new Float32Array(count * 9);
  const normals = new Float32Array(count * 9);
  for (let triangle = 0; triangle < count; triangle += 1) {
    const offset = 84 + triangle * 50 + 12;
    for (let k = 0; k < 9; k += 1) {
      positions[triangle * 9 + k] = view.getFloat32(offset + k * 4, true);
    }
    const p = positions.subarray(triangle * 9, triangle * 9 + 9);
    const u = [p[3] - p[0], p[4] - p[1], p[5] - p[2]];
    const v = [p[6] - p[0], p[7] - p[1], p[8] - p[2]];
    const n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    const length = Math.hypot(n[0], n[1], n[2]) || 1;
    for (let corner = 0; corner < 3; corner += 1) {
      for (let k = 0; k < 3; k += 1) normals[triangle * 9 + corner * 3 + k] = n[k] / length;
    }
  }
  return { positions, normals, count };
}

// A camera for the flat on-page preview: a perspective projection and a view
// looking from `eye` at `target`. In the headset both come from WebXR instead.
export function perspective(fovY, aspect, near, far) {
  const f = 1 / Math.tan(fovY / 2);
  return [
    f / aspect, 0, 0, 0,
    0, f, 0, 0,
    0, 0, (far + near) / (near - far), -1,
    0, 0, (2 * far * near) / (near - far), 0,
  ];
}

export function lookAt(eye, target, up) {
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const unit = (a) => {
    const length = Math.hypot(a[0], a[1], a[2]);
    return [a[0] / length, a[1] / length, a[2] / length];
  };
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const back = unit(sub(eye, target));
  const right = unit(cross(up, back));
  const above = cross(back, right);
  return [
    right[0], above[0], back[0], 0,
    right[1], above[1], back[1], 0,
    right[2], above[2], back[2], 0,
    -dot(right, eye), -dot(above, eye), -dot(back, eye), 1,
  ];
}
