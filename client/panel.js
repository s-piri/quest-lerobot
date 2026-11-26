// Paints the operator's guidance onto a 2D canvas, which the headset view then
// shows as a floating panel. Browser only: it needs a canvas.

import { wrap } from "./guide.js";

const COLORS = { go: "#5ad17a", wait: "#c8ccd4", warn: "#f2b94b", stop: "#ff6b5e" };
const PAD = 44;

function layout(context, lines, size, width) {
  context.font = `${size}px system-ui, sans-serif`;
  const rows = lines.map((line) => ({
    tone: line.tone,
    texts: wrap((text) => context.measureText(text).width, line.text, width - 2 * PAD - size),
  }));
  const height = rows.reduce((sum, row) => sum + row.texts.length * size * 1.3 + size * 0.55, 0);
  return { rows, height };
}

export function paintPanel(canvas, lines) {
  const context = canvas.getContext("2d");
  const { width, height } = canvas;
  context.clearRect(0, 0, width, height);
  context.fillStyle = "rgba(18, 20, 26, 0.9)";
  context.beginPath();
  context.roundRect(0, 0, width, height, 36);
  context.fill();

  // The largest text that fits: a stop must not be cut off by a long message.
  let size = 46;
  let plan = layout(context, lines, size, width);
  while (plan.height > height - 2 * PAD && size > 26) {
    size -= 3;
    plan = layout(context, lines, size, width);
  }
  context.textBaseline = "top";
  let y = PAD;
  for (const row of plan.rows) {
    context.fillStyle = COLORS[row.tone] ?? COLORS.wait;
    context.beginPath();
    context.arc(PAD + size * 0.3, y + size * 0.6, size * 0.3, 0, 2 * Math.PI);
    context.fill();
    context.fillStyle = row.tone === "stop" ? COLORS.stop : "#f2f4f8";
    for (const text of row.texts) {
      context.fillText(text, PAD + size, y);
      y += size * 1.3;
    }
    y += size * 0.55;
  }
}
