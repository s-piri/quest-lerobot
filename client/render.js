// Drawing the arm with plain WebGL: one flat-shaded mesh per printed part or motor.
// The maths lives in arm.js; this file only talks to the graphics card.

import { parseStl, sphereMesh } from "./arm.js";

const VERTEX = `
attribute vec3 aPosition;
attribute vec3 aNormal;
uniform mat4 uViewProjection;
uniform mat4 uModel;
varying vec3 vNormal;
void main() {
  vNormal = (uModel * vec4(aNormal, 0.0)).xyz;
  gl_Position = uViewProjection * uModel * vec4(aPosition, 1.0);
}`;

const FRAGMENT = `
precision mediump float;
uniform vec4 uColor;
varying vec3 vNormal;
void main() {
  vec3 n = normalize(vNormal);
  float light = 0.42
    + 0.45 * max(dot(n, normalize(vec3(0.3, 1.0, 0.5))), 0.0)
    + 0.13 * max(dot(n, normalize(vec3(-0.6, -0.3, -0.5))), 0.0);
  // Premultiplied, which is what the headset's compositor expects.
  gl_FragColor = vec4(uColor.rgb * light * uColor.a, uColor.a);
}`;

function compile(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    throw new Error(`shader did not compile: ${gl.getShaderInfoLog(shader)}`);
  }
  return shader;
}

export async function loadModel(base = "./assets/") {
  const reply = await fetch(`${base}so101.json`);
  if (!reply.ok) throw new Error(`the arm model is missing (${reply.status})`);
  const scene = await reply.json();
  const meshes = {};
  await Promise.all(
    Object.keys(scene.meshes).map(async (name) => {
      const file = await fetch(base + name);
      if (!file.ok) throw new Error(`mesh ${name} is missing (${file.status})`);
      meshes[name] = parseStl(await file.arrayBuffer());
    }),
  );
  return { scene, meshes };
}

export class ArmRenderer {
  constructor(gl, meshes) {
    this.gl = gl;
    const program = gl.createProgram();
    gl.attachShader(program, compile(gl, gl.VERTEX_SHADER, VERTEX));
    gl.attachShader(program, compile(gl, gl.FRAGMENT_SHADER, FRAGMENT));
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
      throw new Error(`shaders did not link: ${gl.getProgramInfoLog(program)}`);
    }
    this.program = program;
    this.position = gl.getAttribLocation(program, "aPosition");
    this.normal = gl.getAttribLocation(program, "aNormal");
    this.viewProjection = gl.getUniformLocation(program, "uViewProjection");
    this.model = gl.getUniformLocation(program, "uModel");
    this.color = gl.getUniformLocation(program, "uColor");
    this.buffers = {};
    // "#ball" is a unit sphere for markers; mesh files cannot have that name.
    for (const [name, mesh] of Object.entries({ ...meshes, "#ball": sphereMesh() })) {
      const positions = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, positions);
      gl.bufferData(gl.ARRAY_BUFFER, mesh.positions, gl.STATIC_DRAW);
      const normals = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, normals);
      gl.bufferData(gl.ARRAY_BUFFER, mesh.normals, gl.STATIC_DRAW);
      this.buffers[name] = { positions, normals, vertices: mesh.count * 3 };
    }
  }

  // Draw one arm's meshes. A ghost (alpha below 1) is blended and tinted, and
  // does not hide what is behind it.
  draw(viewProjection, list, alpha = 1, tint = null) {
    const gl = this.gl;
    gl.useProgram(this.program);
    gl.enable(gl.DEPTH_TEST);
    gl.enable(gl.CULL_FACE);
    const ghost = alpha < 1;
    if (ghost) {
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA);
    } else {
      gl.disable(gl.BLEND);
    }
    gl.depthMask(!ghost);
    gl.uniformMatrix4fv(this.viewProjection, false, new Float32Array(viewProjection));
    gl.enableVertexAttribArray(this.position);
    gl.enableVertexAttribArray(this.normal);
    for (const entry of list) {
      const buffer = this.buffers[entry.mesh];
      if (!buffer) continue;
      const [r, g, b] = tint ?? entry.color;
      gl.uniform4f(this.color, r, g, b, alpha);
      gl.uniformMatrix4fv(this.model, false, new Float32Array(entry.matrix));
      gl.bindBuffer(gl.ARRAY_BUFFER, buffer.positions);
      gl.vertexAttribPointer(this.position, 3, gl.FLOAT, false, 0, 0);
      gl.bindBuffer(gl.ARRAY_BUFFER, buffer.normals);
      gl.vertexAttribPointer(this.normal, 3, gl.FLOAT, false, 0, 0);
      gl.drawArrays(gl.TRIANGLES, 0, buffer.vertices);
    }
    gl.depthMask(true);
    gl.disable(gl.BLEND);
  }
}

const PANEL_VERTEX = `
attribute vec2 aCorner;
uniform mat4 uViewProjection;
uniform mat4 uModel;
varying vec2 vTexture;
void main() {
  vTexture = vec2(aCorner.x + 0.5, 0.5 - aCorner.y);
  gl_Position = uViewProjection * uModel * vec4(aCorner, 0.0, 1.0);
}`;

const PANEL_FRAGMENT = `
precision mediump float;
uniform sampler2D uImage;
varying vec2 vTexture;
void main() {
  gl_FragColor = texture2D(uImage, vTexture);
}`;

// A flat panel showing a 2D canvas: the text the operator reads inside the headset.
export class PanelRenderer {
  constructor(gl) {
    this.gl = gl;
    const program = gl.createProgram();
    gl.attachShader(program, compile(gl, gl.VERTEX_SHADER, PANEL_VERTEX));
    gl.attachShader(program, compile(gl, gl.FRAGMENT_SHADER, PANEL_FRAGMENT));
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
      throw new Error(`panel shaders did not link: ${gl.getProgramInfoLog(program)}`);
    }
    this.program = program;
    this.corner = gl.getAttribLocation(program, "aCorner");
    this.viewProjection = gl.getUniformLocation(program, "uViewProjection");
    this.model = gl.getUniformLocation(program, "uModel");
    this.corners = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, this.corners);
    const quad = [-0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, 0.5, -0.5, 0.5];
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(quad), gl.STATIC_DRAW);
    this.texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, this.texture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  }

  setImage(canvas) {
    const gl = this.gl;
    gl.bindTexture(gl.TEXTURE_2D, this.texture);
    gl.pixelStorei(gl.UNPACK_PREMULTIPLY_ALPHA_WEBGL, true);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, canvas);
    gl.pixelStorei(gl.UNPACK_PREMULTIPLY_ALPHA_WEBGL, false);
  }

  draw(viewProjection, matrix) {
    const gl = this.gl;
    gl.useProgram(this.program);
    gl.disable(gl.CULL_FACE);
    gl.enable(gl.DEPTH_TEST);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA);
    gl.uniformMatrix4fv(this.viewProjection, false, new Float32Array(viewProjection));
    gl.uniformMatrix4fv(this.model, false, new Float32Array(matrix));
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.texture);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.corners);
    gl.enableVertexAttribArray(this.corner);
    gl.vertexAttribPointer(this.corner, 2, gl.FLOAT, false, 0, 0);
    gl.drawArrays(gl.TRIANGLES, 0, 6);
    gl.disableVertexAttribArray(this.corner);
    gl.disable(gl.BLEND);
  }
}

