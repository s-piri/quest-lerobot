import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { test } from "node:test";

import {
  ARM_JOINTS,
  TOOL_LINK,
  about,
  compose,
  drawList,
  facing,
  jointValues,
  linkPoses,
  lookAt,
  markerMatrix,
  matMul,
  matrix,
  multiply,
  parseStl,
  perspective,
  placeToolAt,
  placement,
  rotate,
  sphereMesh,
  toolPosition,
} from "./arm.js";

const here = new URL(".", import.meta.url);
const scene = JSON.parse(readFileSync(new URL("assets/so101.json", here), "utf8"));
const reference = JSON.parse(readFileSync(new URL("../tests/fixtures/so101_fk.json", here), "utf8"));

const close = (actual, expected, tolerance, label) => {
  actual.forEach((value, index) => {
    assert.ok(Math.abs(value - expected[index]) <= tolerance, `${label}[${index}]: ${value} vs ${expected[index]}`);
  });
};
const angleBetween = (a, b) => 2 * Math.acos(Math.min(1, Math.abs(a.reduce((sum, v, i) => sum + v * b[i], 0))));
const apply = (m, p) => [0, 1, 2].map((row) => m[row] * p[0] + m[4 + row] * p[1] + m[8 + row] * p[2] + m[12 + row]);

test("every link sits where an independent URDF library puts it", () => {
  assert.ok(reference.links.length >= 16);
  for (const { values, links } of reference.links) {
    const poses = linkPoses(scene, values);
    assert.deepEqual(Object.keys(poses).sort(), Object.keys(links).sort());
    for (const [name, wanted] of Object.entries(links)) {
      close(poses[name].position, wanted.position, 1e-9, name);
      assert.ok(angleBetween(poses[name].orientation, wanted.orientation) < 1e-6, name);
    }
  }
});

test("the tool link matches the host's forward kinematics cases", () => {
  for (const { joints, position, orientation } of reference.cases) {
    const values = Object.fromEntries(ARM_JOINTS.map((name, index) => [name, joints[index]]));
    const tool = linkPoses(scene, values)[TOOL_LINK];
    close(tool.position, position, 1e-9, "tool");
    assert.ok(angleBetween(tool.orientation, orientation) < 1e-6);
  }
});

test("the host's state becomes joint values, with the jaw between its limits", () => {
  const jaw = scene.joints.find((joint) => joint.name === "gripper");
  const closed = jointValues(scene, [0.1, 0.2, 0.3, 0.4, 0.5, 0]);
  assert.deepEqual(ARM_JOINTS.map((name) => closed[name]), [0.1, 0.2, 0.3, 0.4, 0.5]);
  assert.equal(closed.gripper, jaw.lower);
  assert.equal(jointValues(scene, [0, 0, 0, 0, 0, 1]).gripper, jaw.upper);
  assert.ok(Math.abs(jointValues(scene, [0, 0, 0, 0, 0, 0.5]).gripper - (jaw.lower + jaw.upper) / 2) < 1e-12);
});

test("a matrix moves points exactly as its pose does", () => {
  const pose = { position: [0.3, -0.2, 0.5], orientation: about([0, 0, 1], Math.PI / 2) };
  close(apply(matrix(pose), [1, 0, 0]), [0.3, 0.8, 0.5], 1e-12, "rotated point");
  const other = { position: [0, 0, 1], orientation: about([1, 0, 0], 0.7) };
  const point = [0.2, 0.4, -0.1];
  const viaPoses = compose(compose(pose, other), { position: point, orientation: [0, 0, 0, 1] }).position;
  close(apply(matMul(matrix(pose), matrix(other)), point), viaPoses, 1e-12, "composed");
});

test("the drawn arm turns against the host's base rotation so hand and gripper move together", () => {
  // The host maps headset +x to the arm's +y with this rotation.
  const base = about([0, 0, 1], Math.PI / 2);
  const placed = placement(base, [1, 1, 1]);
  const zero = jointValues(scene, [0, 0, 0, 0, 0, 0.5]);
  const start = toolPosition(scene, zero, placed);
  // Move the tool 2 cm along the arm's +y, as the host would for a hand moving 2 cm along headset +x.
  const tool = linkPoses(scene, zero)[TOOL_LINK].position;
  const shifted = compose(placed, { position: [tool[0], tool[1] + 0.02, tool[2]], orientation: [0, 0, 0, 1] });
  close(shifted.position, [start[0] + 0.02, start[1], start[2]], 1e-12, "moved with the hand");
  close(rotate(multiply(base, placed.orientation), [1, 2, 3]), [1, 2, 3], 1e-12, "inverse rotations");
});

test("the arm can be placed so its gripper starts at a chosen point", () => {
  const base = about([0, 1, 0], 0.4);
  const values = jointValues(scene, [0.2, -0.3, 0.5, 0.4, 0.1, 0.5]);
  const placed = placeToolAt(scene, values, base, [0.1, 1.2, -0.4]);
  close(toolPosition(scene, values, placed), [0.1, 1.2, -0.4], 1e-12, "tool");
});

test("the draw list has one entry per visual, placed in headset space", () => {
  const values = jointValues(scene, [0, 0, 0, 0, 0, 0]);
  const placed = placement([0, 0, 0, 1], [0, 1, -0.5]);
  const list = drawList(scene, values, placed);
  assert.equal(list.length, scene.visuals.length);
  for (const entry of list) {
    assert.ok(entry.mesh in scene.meshes);
    assert.equal(entry.matrix.length, 16);
    assert.ok(entry.matrix.every(Number.isFinite));
    assert.equal(entry.color.length, 4);
  }
  const baseVisual = scene.visuals.findIndex((visual) => visual.link === scene.base);
  close(apply(list[baseVisual].matrix, [0, 0, 0]), compose(placed, scene.visuals[baseVisual]).position, 1e-12, "base visual");
});

test("the shipped meshes parse, with unit normals and the recorded triangle counts", () => {
  for (const [name, count] of Object.entries(scene.meshes)) {
    const bytes = readFileSync(new URL(`assets/${name}`, here));
    const mesh = parseStl(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
    assert.equal(mesh.count, count, name);
    assert.equal(mesh.positions.length, count * 9);
    assert.ok(mesh.positions.every(Number.isFinite), name);
    for (let i = 0; i < mesh.normals.length; i += 3) {
      const length = Math.hypot(mesh.normals[i], mesh.normals[i + 1], mesh.normals[i + 2]);
      assert.ok(Math.abs(length - 1) < 1e-4 || length === 0, `${name} normal ${i}`);
    }
    // Parts are tens of millimetres across: the files are in metres, not millimetres.
    assert.ok(Math.max(...mesh.positions.map(Math.abs)) < 0.5, name);
  }
});

test("the marker ball is closed, unit sized and faces outward", () => {
  const ball = sphereMesh();
  assert.equal(ball.count, 8 * 16);
  assert.equal(ball.positions.length, ball.count * 9);
  for (let i = 0; i < ball.positions.length; i += 9) {
    const p = ball.positions.subarray(i, i + 9);
    for (let corner = 0; corner < 9; corner += 3) {
      assert.ok(Math.abs(Math.hypot(p[corner], p[corner + 1], p[corner + 2]) - 1) < 1e-6);
    }
    // Counter-clockwise seen from outside, so back-face culling keeps the outside.
    const u = [p[3] - p[0], p[4] - p[1], p[5] - p[2]];
    const v = [p[6] - p[0], p[7] - p[1], p[8] - p[2]];
    const n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    assert.ok(n[0] * p[0] + n[1] * p[1] + n[2] * p[2] > 0);
  }
  close(apply(markerMatrix([1, 2, 3], 0.02), [1, 0, 0]), [1.02, 2, 3], 1e-12, "marker");
});

test("a panel is sized, upright and turned toward the viewer", () => {
  const panel = facing([0, 1.4, -1], [0, 1.6, 0], 0.5, 0.25);
  close(apply(panel, [0, 0, 0]), [0, 1.4, -1], 1e-12, "centre");
  close(apply(panel, [0.5, 0.5, 0]), [0.25, 1.525, -1], 1e-12, "top right corner, seen from the viewer");
  // Its front (+z) points at the viewer, level with the floor.
  const front = apply(panel, [0, 0, 1]).map((value, index) => value - [0, 1.4, -1][index]);
  close(front, [0, 0, 1], 1e-12, "front");
  const side = facing([1, 1, 0], [0, 1, 0], 1, 1);
  close(apply(side, [0, 0, 1]), [0, 1, 0], 1e-12, "turned to a viewer on its left");
});

test("a damaged mesh file is refused", () => {
  assert.throws(() => parseStl(new ArrayBuffer(10)), /binary STL/);
  const bad = new ArrayBuffer(84 + 50);
  new DataView(bad).setUint32(80, 2, true);
  assert.throws(() => parseStl(bad), /truncated/);
});

test("the preview camera looks down its own -z at the target", () => {
  const view = lookAt([0, 0, 2], [0, 0, 0], [0, 1, 0]);
  close(apply(view, [0, 0, 0]), [0, 0, -2], 1e-12, "target in view space");
  close(apply(view, [1, 0, 0]), [1, 0, -2], 1e-12, "right stays right");
  const projection = perspective(Math.PI / 2, 1, 0.1, 10);
  const clip = [0, 1, 2, 3].map((row) => projection[8 + row] * -0.1 + projection[12 + row]);
  assert.ok(Math.abs(clip[2] / clip[3] + 1) < 1e-9); // the near plane maps to -1
});
