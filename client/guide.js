// Plain-language guidance for the operator: what is happening and what to do
// next. No DOM and no WebXR here, so Node can test every sentence.
//
// A line is { tone, text }. Tones: "go" (all well), "wait" (nothing to do yet),
// "warn" (do something), "stop" (the host has stopped and must be cleared).

const HOST_REASONS = [
  ["measured feedback is stale", "the host lost the arm's position readings"],
  ["forward kinematics did not return a pose", "the host could not work out where the arm is"],
  ["stop requested", "a stop was requested"],
];

// The host's own wording, made readable. Unknown reasons pass through unchanged.
export function plain(reason) {
  let text = String(reason ?? "");
  const other = /^(left|right|arm) (stopped|faulted): (.*)$/.exec(text);
  if (other) {
    const who = other[1] === "arm" ? "the arm" : `the ${other[1]} arm`;
    return `${who} ${other[2] === "faulted" ? "had a fault" : "was stopped"} (${plain(other[3])})`;
  }
  for (const [from, to] of HOST_REASONS) text = text.replace(from, to);
  return text;
}

// One arm's state from the host, as guidance. `options`: grab (the virtual
// gripper must be grabbed), button ("A" or "X"), tracked (its controller is in view).
export function explainArm(arm, options = {}) {
  const { grab = true, button = "A", tracked = true } = options;
  switch (arm?.state) {
    case "demo":
      return { tone: "wait", text: "Example pose. Connect to a host to see the real arm." };
    case "disarmed":
      return {
        tone: "wait",
        text: "Motion is off on the host, so nothing will move. The arm appears here once the host arms it.",
      };
    case "armed":
      if (arm.reason === "release the grip before pressing it") {
        return { tone: "warn", text: "Let go of the grip, then squeeze again." };
      }
      if (arm.reason === "no fresh input") {
        return tracked
          ? { tone: "wait", text: "Waiting for your controller. Enter the headset view to start." }
          : { tone: "warn", text: "The controller is not tracked. Bring it into view." };
      }
      return grab
        ? { tone: "go", text: "Ready. Reach for the virtual gripper and squeeze the grip to take hold." }
        : { tone: "go", text: "Ready. Squeeze the grip to move." };
    case "engaged":
      if (arm.reason === "previewing") {
        return {
          tone: "go",
          text: `Previewing: only the virtual arm is moving. Hold ${button} to move the real arm.`,
        };
      }
      if (arm.reason === "inverse kinematics rejected the target") {
        return { tone: "warn", text: "Out of reach. Bring your hand back toward the arm." };
      }
      return { tone: "go", text: "Moving with your hand. Let go of the grip to hold." };
    case "stopped":
      return {
        tone: "stop",
        text: `Stopped: ${plain(arm.reason)}. It stays stopped until it is cleared on the host.`,
      };
    case "faulted":
      return {
        tone: "stop",
        text: `Fault: ${plain(arm.reason)}. It stays stopped until it is cleared on the host.`,
      };
    default:
      return { tone: "wait", text: "Waiting for the host to report this arm." };
  }
}

const handFor = (mode, name) => (mode === "both" ? name : mode);
const label = (name) => (name ? `${name[0].toUpperCase()}${name.slice(1)} arm: ` : "");

// Everything the operator should know right now, most important first.
// `view`: connection ("offline" | "connecting" | "paired"), arms, mode, grab,
// tracked ({ left, right }), staleSeconds (since the host's last update), inHeadset.
export function statusLines(view) {
  const { connection, arms = {}, mode = "right", grab = true, tracked = {}, staleSeconds = 0 } = view;
  if (connection === "connecting") return [{ tone: "wait", text: "Connecting to the host…" }];
  const lines = [];
  const names = Object.keys(arms).sort();
  if (connection === "paired" && names.length === 0) {
    lines.push({
      tone: "wait",
      text: "Paired. Waiting for the host to report the arm. Your controller input is being sent meanwhile.",
    });
  }
  for (const name of names) {
    const hand = handFor(mode, name);
    const line = explainArm(arms[name], {
      grab,
      button: hand === "left" ? "X" : "A",
      tracked: view.inHeadset ? tracked[hand] === true : true,
    });
    lines.push({ tone: line.tone, text: label(name) + line.text });
  }
  if (connection === "paired" && names.length > 0 && staleSeconds >= 1) {
    lines.unshift({
      tone: "warn",
      text: `No update from the host for ${Math.round(staleSeconds)} s. The picture may be out of date.`,
    });
  }
  if (connection === "offline" && !names.some((name) => arms[name].state === "demo")) {
    lines.unshift({ tone: "warn", text: "Not connected. The host is not following your hands." });
  }
  return lines;
}

// Why the connection closed, and what to do about it.
export function closeMessage(code, reason) {
  if (reason === "authentication required") {
    return "The host did not accept the pairing token. Use the one the host shows for this run: it changes every time the host starts.";
  }
  if (reason === "input already has an owner") {
    return "Another headset or page is already connected to this host. Close that one first.";
  }
  if (reason === "invalid input") {
    return "The host rejected a message from this page and closed the connection. Reload the page so it matches the host, then connect again.";
  }
  if (reason === "input shutting down" || code === 1001) {
    return "The host stopped listening.";
  }
  if (code === 1006) {
    return "Could not reach the host, or the connection dropped. Check the address, that the host is running, and that this device trusts the host's certificate.";
  }
  if (code === 1000 || code === 1005) return "Disconnected.";
  return `Disconnected (code ${code}${reason ? `: ${reason}` : ""}).`;
}

// A problem with the typed address that can be explained before trying it, or null.
export function addressProblem(text, pageIsSecure) {
  const address = String(text ?? "").trim();
  if (!address) return "Enter the listener address the host shows, for example wss://host-name:8765.";
  let url;
  try {
    url = new URL(address);
  } catch {
    return "That is not a complete address. It should look like wss://host-name:8765.";
  }
  if (url.protocol !== "ws:" && url.protocol !== "wss:") {
    return "The address must start with wss:// (or ws:// on this same machine).";
  }
  const local = ["localhost", "127.0.0.1", "[::1]"].includes(url.hostname);
  if (url.protocol === "ws:" && pageIsSecure && !local) {
    return "This page was opened over https, so the browser only allows a wss:// address.";
  }
  return null;
}

// Break text into lines no wider than `width`, measuring with `measure(text)`.
export function wrap(measure, text, width) {
  const lines = [];
  let line = "";
  for (const word of String(text).split(/\s+/).filter(Boolean)) {
    const longer = line ? `${line} ${word}` : word;
    if (line && measure(longer) > width) {
      lines.push(line);
      line = word;
    } else {
      line = longer;
    }
  }
  if (line) lines.push(line);
  return lines;
}

// What changed between two host states that is worth a buzz in the hand:
// "stop" when an arm stops or faults, "reach" when it first goes out of reach.
export function alerts(before = {}, after = {}) {
  const events = [];
  for (const [name, arm] of Object.entries(after)) {
    const was = before[name] ?? {};
    const halted = (state) => state === "stopped" || state === "faulted";
    if (halted(arm.state) && !halted(was.state)) events.push({ arm: name, kind: "stop" });
    const rejected = (reason) => reason === "inverse kinematics rejected the target";
    if (rejected(arm.reason) && !rejected(was.reason)) events.push({ arm: name, kind: "reach" });
  }
  return events;
}
