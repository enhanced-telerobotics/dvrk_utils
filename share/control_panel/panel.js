/* Standalone web UI: no CDNs, ROS libraries, or commands queued for replay. */
"use strict";

const byId = (id) => document.getElementById(id);
let socket = null;
let retryTimer = null;
let lastSnapshot = 0;
let snapshot = null;
let control = "available";
let sequence = 0;
let rejectedToken = false;
const pending = new Map();
const armRows = new Map();
const teleopRows = new Map();
const sliderTouched = {scale: 0, volume: 0};
let token = "";
let session = "";

function storageRead(storage, key) {
  try { return storage.getItem(key) || ""; } catch (_) { return ""; }
}
function storageWrite(storage, key, value) {
  try { storage.setItem(key, value); } catch (_) { /* Private browsing is supported. */ }
}
const fragmentToken = new URLSearchParams(location.hash.slice(1)).get("token");
const savedToken = storageRead(sessionStorage, "dvrk-pairing");
token = fragmentToken || savedToken;
session = token === savedToken ? storageRead(sessionStorage, "dvrk-control-session") : "";
if (fragmentToken) {
  storageWrite(sessionStorage, "dvrk-pairing", token);
  storageWrite(sessionStorage, "dvrk-control-session", session);
  history.replaceState(null, "", location.pathname + location.search);
}
document.documentElement.dataset.theme = storageRead(localStorage, "dvrk-theme") || (
  matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light");
byId("theme").addEventListener("click", () => {
  const value = document.documentElement.dataset.theme === "dark" ? "light" : "dark";
  document.documentElement.dataset.theme = value;
  storageWrite(localStorage, "dvrk-theme", value);
});

function notice(text, error = false) {
  byId("notice").textContent = text;
  byId("notice").classList.toggle("error", error);
}
function connected() {
  return socket && socket.readyState === WebSocket.OPEN && lastSnapshot > 0
    && Date.now() - lastSnapshot < 3000 && !document.hidden;
}
function canControl() { return connected() && control === "mine"; }

function request(type, payload = {}) {
  if (!connected() || (type === "command" && !canControl())) {
    notice("Connect and take control before sending a request.", true);
    return;
  }
  const id = `${Date.now()}-${++sequence}`;
  pending.set(id, Date.now());
  socket.send(JSON.stringify({type, id, ...payload}));
  if (type === "command") notice("Sending request…");
}

function badge(element, text, color) {
  element.textContent = text;
  element.className = `badge ${color}`;
}
function stateColor(state) {
  if (state === "ENABLED" || state === "following") return "green";
  if (state === "DISABLED") return "red";
  if (state === "OFFLINE") return "neutral";
  return "orange";
}

function makeRow(parent, name, action) {
  const row = document.createElement("div"); row.className = "row";
  const info = document.createElement("div"); info.className = "row-info";
  const title = document.createElement("div"); title.className = "row-title";
  const label = document.createElement("strong"); label.textContent = name;
  const status = document.createElement("span");
  title.append(label, status);
  const detail = document.createElement("p"); detail.className = "row-detail";
  const tool = document.createElement("p"); tool.className = "row-tool";
  const reset = document.createElement("p"); reset.className = "row-detail";
  const button = document.createElement("button"); button.className = "row-action";
  button.addEventListener("click", action);
  info.append(title, detail, tool, reset);
  row.append(info, button); parent.append(row);
  return {row, status, detail, tool, reset, button};
}

function render() {
  const live = connected();
  const writable = canControl();
  byId("connection").textContent = live ? "Connected" : "Disconnected";
  byId("connection").classList.toggle("offline", !live);
  byId("panel").classList.toggle("stale", !live);
  byId("control-button").disabled = !live || control === "busy";
  byId("control-button").textContent = control === "mine" ? "Release control" : "Take control";
  byId("control-title").textContent = !live ? "Connection unavailable" : (
    control === "mine" ? "You have control" : control === "busy" ? "Viewing only" : "Ready to connect controls");
  byId("control-description").textContent = !live ? "Controls are disabled. Reconnect to refresh the robot state." : (
    control === "mine" ? "Requests are sent to the PC. Status reflects ROS feedback." :
    control === "busy" ? "The owner must release control, or regenerate the QR on the PC." : "Take control to operate from this device.");
  if (!snapshot) return;
  byId("console-name").textContent = `Console /${snapshot.console}`;
  const arms = Object.values(snapshot.arms);
  const anyOnline = arms.some((arm) => arm.online);
  badge(byId("power-status"), anyOnline ? (snapshot.powered ? "ENABLED" : "DISABLED") : "OFFLINE",
    anyOnline ? (snapshot.powered ? "green" : "red") : "neutral");
  byId("power-text").textContent = snapshot.powered ? "Power off system" : "Home system";
  byId("power-button").disabled = !writable || !anyOnline;
  byId("arms-empty").classList.toggle("hidden", arms.some((arm) => arm.seen));
  for (const arm of arms) {
    if (!arm.seen) continue;
    if (!armRows.has(arm.name)) {
      armRows.set(arm.name, makeRow(byId("arms"), arm.name,
        () => request("command", {action: "reset", name: arm.name})));
      // Always show the standard arm order, regardless of ROS arrival order.
      for (const ordered of arms) {
        if (armRows.has(ordered.name)) byId("arms").append(armRows.get(ordered.name).row);
      }
    }
    const row = armRows.get(arm.name);
    badge(row.status, arm.online ? arm.state : "OFFLINE", arm.online ? stateColor(arm.state) : "neutral");
    row.detail.textContent = `${arm.is_homed ? "HOMED" : "NOT HOMED"}${arm.is_busy ? " · BUSY" : ""}`;
    row.tool.textContent = arm.name.startsWith("PSM") ? (arm.tool_type || "No instrument") : "";
    const resetting = arm.reset && !["done", "failed"].includes(arm.reset.stage);
    row.reset.textContent = arm.reset ? arm.reset.message : "";
    row.reset.className = `row-detail reset-${arm.reset ? arm.reset.stage : "idle"}`;
    row.button.textContent = resetting ? "Resetting" : "Reset";
    row.button.setAttribute("aria-label", `Reset ${arm.name}`);
    row.button.disabled = !writable || !arm.online || Boolean(resetting);
  }
  const enabledAvailable = snapshot.available.enabled;
  const enabledKnown = snapshot.enabled !== null;
  badge(byId("teleop-status"), !enabledAvailable ? "OFFLINE" : !enabledKnown ? "UNKNOWN" : (
    snapshot.enabled ? "ENABLED" : "DISABLED"),
    !enabledAvailable ? "neutral" : !enabledKnown ? "orange" : snapshot.enabled ? "green" : "red");
  byId("enable-text").textContent = snapshot.enabled ? "Disable teleoperation" : "Enable teleoperation";
  byId("enable-button").disabled = !writable || !enabledAvailable;
  const teleops = Object.values(snapshot.teleops).sort((a, b) =>
    Number(a.name.includes("ECM")) - Number(b.name.includes("ECM")) || a.name.localeCompare(b.name));
  byId("teleops-empty").classList.toggle("hidden", teleops.length > 0);
  for (const [index, pair] of teleops.entries()) {
    if (!teleopRows.has(pair.name)) {
      teleopRows.set(pair.name, makeRow(byId("teleops"), pair.name, () => {
        const current = snapshot.teleops[pair.name];
        request("command", {action: current.selected ? "unselect" : "select", name: pair.name});
      }));
    }
    const row = teleopRows.get(pair.name);
    badge(row.status, pair.selected ? "SELECTED" : "UNSELECTED", pair.selected ? "green" : "orange");
    row.detail.textContent = pair.online ? pair.state : "OFFLINE";
    row.button.textContent = pair.selected ? "−" : "+";
    row.button.setAttribute("aria-label", `${pair.selected ? "Unselect" : "Select"} ${pair.name}`);
    row.button.disabled = !writable || !pair.online;
    const parent = byId("teleops");
    if (parent.children[index] !== row.row) parent.insertBefore(row.row, parent.children[index] || null);
  }
  for (const field of ["scale", "volume"]) {
    const input = byId(field);
    input.disabled = !writable || !snapshot.available[field];
    if (Date.now() - sliderTouched[field] > 1000) {
      if (snapshot[field] !== null) {
        input.value = snapshot[field];
        showSlider(field, Number(input.value));
      } else byId(`${field}-value`).textContent = "—";
    }
  }
}

function showSlider(field, value) {
  byId(`${field}-value`).textContent = field === "scale" ? value.toFixed(1) : `${Math.round(value * 100)}%`;
}
for (const field of ["scale", "volume"]) {
  byId(field).addEventListener("input", () => {
    sliderTouched[field] = Date.now();
    showSlider(field, Number(byId(field).value));
  });
  byId(field).addEventListener("change", () => {
    sliderTouched[field] = Date.now();
    request("command", {action: field, value: Number(byId(field).value)});
  });
}
byId("control-button").addEventListener("click", () => request(control === "mine" ? "release" : "acquire"));
byId("power-button").addEventListener("click", () => request("command", {action: snapshot.powered ? "power_off" : "home"}));
byId("enable-button").addEventListener("click", () => request("command", {action: "enable", value: !snapshot.enabled}));

function connect() {
  clearTimeout(retryTimer);
  if (document.hidden || rejectedToken) return;
  if (!token) {
    byId("pairing").classList.remove("hidden");
    notice("Use the connection link shown on the PC.");
    render(); return;
  }
  if (socket && socket.readyState <= WebSocket.OPEN) return;
  const ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  socket = ws;
  ws.addEventListener("open", () => {
    ws.send(JSON.stringify({token, session}));
  });
  ws.addEventListener("message", (event) => {
    if (socket !== ws) return;
    const message = JSON.parse(event.data);
    if (message.type === "state") {
      session = message.session;
      storageWrite(sessionStorage, "dvrk-control-session", session);
      const first = lastSnapshot === 0;
      snapshot = message.state; control = message.control; lastSnapshot = Date.now();
      byId("devices").textContent = `${message.clients} device${message.clients === 1 ? "" : "s"} connected`;
      byId("pairing").classList.add("hidden");
      if (first) notice(control === "mine" ? "Reconnected. Your exclusive control is restored." :
        "Connected. Take control when you are ready.");
      render();
    } else if (message.type === "ack") {
      pending.delete(message.id);
      notice(message.message, !message.ok);
    }
  });
  ws.addEventListener("close", (event) => {
    if (socket !== ws) return;
    socket = null; lastSnapshot = 0; control = "available"; pending.clear();
    if (event.code === 1008) {
      rejectedToken = true;
      token = ""; session = "";
      storageWrite(sessionStorage, "dvrk-pairing", "");
      storageWrite(sessionStorage, "dvrk-control-session", "");
      byId("pairing").classList.remove("hidden");
      notice("This connection link has expired. Scan the current PC QR code.", true);
    } else {
      notice("Disconnected. Requests are not replayed. Reconnecting…", true);
      retryTimer = setTimeout(connect, 1500);
    }
    render();
  });
  ws.addEventListener("error", () => ws.close());
}

byId("pairing").addEventListener("submit", (event) => {
  event.preventDefault();
  try {
    const url = new URL(byId("pairing-link").value);
    if (url.origin !== location.origin) {
      if (!["http:", "https:"].includes(url.protocol)) throw new Error("Invalid URL");
      location.assign(url.href); return;
    }
    const value = new URLSearchParams(url.hash.slice(1)).get("token");
    if (!value) throw new Error("Missing token");
    if (token !== value) session = "";
    token = value; rejectedToken = false;
    storageWrite(sessionStorage, "dvrk-pairing", token);
    storageWrite(sessionStorage, "dvrk-control-session", session);
    connect();
  } catch (_) { notice("Paste the complete connection link, including #token=…", true); }
});

document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    clearTimeout(retryTimer);
    if (socket) socket.close();
    lastSnapshot = 0; control = "available"; render();
  } else connect();
});
setInterval(() => {
  if (socket && socket.readyState === WebSocket.OPEN && !document.hidden) {
    socket.send(JSON.stringify({type: "ping"}));
    if (lastSnapshot && Date.now() - lastSnapshot > 6000) socket.close();
  }
  for (const [id, started] of pending) {
    if (Date.now() - started > 5000) {
      pending.delete(id); notice("No request acknowledgement received. Check robot status before retrying.", true);
    }
  }
  render();
}, 1000);
render();
connect();
