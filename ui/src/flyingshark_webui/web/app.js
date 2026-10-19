let teleopV = 0.25;
let teleopW = 0.8;
let viewer = null;
try { viewer = View3D; } catch (err) { viewer = null; }

const cam = document.getElementById("cam");
const statusEl = document.getElementById("status");
const modeLabel = document.getElementById("modeLabel");
const bwEl = document.getElementById("bw");
const fpsEl = document.getElementById("fps");
const audioLeft = document.getElementById("audioLeft");
const audioRight = document.getElementById("audioRight");
const mapInfo = document.getElementById("mapInfo");
const mapPick = document.getElementById("mapPick");
const deleteMapBtn = document.getElementById("btnDeleteMap");
const followBtn = document.getElementById("follow");
const stick = document.getElementById("stick");
const knob = document.getElementById("knob");

const buttons = {
  init: document.getElementById("btnInit"),
  blank: document.getElementById("btnBlank"),
  mapping: document.getElementById("btnMapping"),
  save: document.getElementById("btnSave"),
  nav: document.getElementById("btnNav"),
  reloc: document.getElementById("btnReloc"),
  goal: document.getElementById("btnGoal"),
};

const state = {
  mode: "idle",
  busy: false,
  message: "",
  maps: [],
  map_name: "",
  robot_radius: 0.108,
  lidar_x: 0.06,
  camera_x: 0.055,
  wheel_y: 0.082,
  wheel_radius: 0.037,
};
const pending = new Set();
let tool = "";
let follow = true;
let pose = null;
let lastGoal = null;
let mapToken = 0;
let rgbToken = 0;
const costToken = { global: 0, local: 0 };
let poseCount = 0;
let drawCount = 0;
let meterT = performance.now();

if (viewer) {
  viewer.setHooks({
    tool: function () { return tool; },
    onFollowOff: function () {
      follow = false;
      viewer.setFollow(false);
      syncButtons();
    },
    onPick: function (start, end) {
      const dx = end.x - start.x;
      const dy = end.y - start.y;
      const dragged = dx * dx + dy * dy > 0.05 * 0.05;
      let yaw = Math.atan2(dy, dx);
      if (!dragged && pose && pose.ok) yaw = pose.yaw || 0;
      const body = { x: start.x, y: start.y, yaw: yaw };
      if (tool === "goal") {
        lastGoal = body;
        viewer.setGoal(body);
        setStatus("已发送导航点");
        call("goal", body);
      } else if (tool === "reloc") {
        setStatus("已发送重定位");
        call("reloc", body);
      }
    },
    onFrame: function () {
      drawCount += 1;
      const now = performance.now();
      if (now - meterT >= 1000) {
        const drawHz = Math.round(drawCount * 1000 / (now - meterT));
        const poseHz = Math.round(poseCount * 1000 / (now - meterT));
        drawCount = 0;
        poseCount = 0;
        meterT = now;
        fpsEl.textContent = "画面 " + drawHz + " Hz · 位姿 " + poseHz + " Hz";
      }
    },
  });
  viewer.setFollow(true);
}

function modeName(mode) {
  return { idle: "未启动", init: "已初始化", blank: "无地图导航", mapping: "手动建图", nav: "已知地图导航" }[mode] || mode;
}

function setStatus(text) {
  statusEl.textContent = text || "";
}

function clearScene() {
  lastGoal = null;
  if (viewer) {
    viewer.clearScene();
    viewer.setGoal(null);
  }
  mapInfo.textContent = "等待地图";
}

function syncButtons() {
  const busy = state.busy || pending.size > 0;
  const ready = state.mode !== "idle";
  buttons.init.classList.toggle("on", ready);
  buttons.blank.classList.toggle("on", state.mode === "blank");
  buttons.mapping.classList.toggle("on", state.mode === "mapping");
  buttons.nav.classList.toggle("on", state.mode === "nav");
  buttons.goal.classList.toggle("on", tool === "goal");
  buttons.reloc.classList.toggle("on", tool === "reloc");
  followBtn.classList.toggle("on", follow);
  buttons.init.disabled = busy;
  buttons.blank.disabled = busy || !ready;
  buttons.mapping.disabled = busy || !ready;
  buttons.nav.disabled = busy || !ready || (!mapPick.value && state.mode !== "nav");
  buttons.save.disabled = busy || state.mode !== "mapping";
  buttons.reloc.disabled = busy;
  buttons.goal.disabled = busy || (state.mode !== "blank" && state.mode !== "nav");
  mapPick.disabled = busy || state.mode === "nav";
  const selectedMap = mapPick.value;
  const usingSelected = state.mode === "nav" && selectedMap && selectedMap === state.map_name;
  deleteMapBtn.disabled = busy || !selectedMap || usingSelected;
  if (deleteArmed && deleteArmed !== selectedMap) {
    deleteArmed = "";
    clearTimeout(deleteTimer);
  }
  deleteMapBtn.classList.toggle("armed", deleteArmed === selectedMap && !!selectedMap);
  modeLabel.textContent = state.busy ? "切换中" : modeName(state.mode);
  setAvailability();
}

function setAvailability() {
  const closed = state.mode === "idle" && !state.busy;
  document.body.classList.toggle("is-idle", closed);
  if (!closed) {
    if (!cam.getAttribute("src")) cam.src = "camera.mjpg?t=" + Date.now();
    return;
  }
  stickVec = null;
  keys.clear();
  placeKnob(null);
  audioLeft.style.width = "0%";
  audioRight.style.width = "0%";
  if (cam.getAttribute("src")) cam.removeAttribute("src");
  if (teleopOn) {
    teleopOn = false;
    fetch("/api/cmd", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ op: "teleop", linear: 0, angular: 0 }),
    }).catch(function () {});
  }
}

function applyState(next) {
  const was = state.mode;
  Object.assign(state, next);
  const selected = mapPick.value;
  mapPick.innerHTML = "";
  (state.maps || []).forEach((name) => {
    const opt = document.createElement("option");
    opt.value = name;
    opt.textContent = name;
    mapPick.appendChild(opt);
  });
  if (state.map_name && [...mapPick.options].some((opt) => opt.value === state.map_name)) {
    mapPick.value = state.map_name;
  } else if ([...mapPick.options].some((opt) => opt.value === selected)) {
    mapPick.value = selected;
  }
  if (next.message) setStatus(next.message);
  if (state.mode !== "nav" && tool === "reloc") tool = "";
  if (state.mode !== "blank" && state.mode !== "nav" && tool === "goal") tool = "";
  const switching = next.busy && next.busy_action && next.busy_action !== "save";
  if (switching || (state.mode === "idle" && was !== "idle")) clearScene();
  syncButtons();
}

async function call(op, extra) {
  pending.add(op);
  syncButtons();
  try {
    const res = await fetch("/api/cmd", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(Object.assign({ op: op }, extra || {})),
    });
    const data = await res.json();
    if (data.state) applyState(data.state);
    if (!data.ok) setStatus(data.error || "操作失败");
    return data;
  } catch (err) {
    setStatus("和香橙派的连接中断了");
    return null;
  } finally {
    pending.delete(op);
    syncButtons();
  }
}

buttons.init.addEventListener("click", function () { call("init", { on: state.mode === "idle" }); });
buttons.blank.addEventListener("click", function () { call("blank", { on: state.mode !== "blank" }); });
buttons.mapping.addEventListener("click", function () { call("mapping", { on: state.mode !== "mapping" }); });
buttons.save.addEventListener("click", function () { call("save"); });
buttons.nav.addEventListener("click", function () { call("nav", { on: state.mode !== "nav", map: mapPick.value }); });

let deleteArmed = "";
let deleteTimer = 0;
deleteMapBtn.addEventListener("click", function () {
  const name = mapPick.value;
  if (!name || deleteMapBtn.disabled) return;
  if (deleteArmed !== name) {
    deleteArmed = name;
    setStatus("再点一次确认删除 " + name);
    clearTimeout(deleteTimer);
    deleteTimer = setTimeout(function () {
      deleteArmed = "";
      syncButtons();
    }, 4000);
    syncButtons();
    return;
  }
  deleteArmed = "";
  clearTimeout(deleteTimer);
  call("delete_map", { map: name });
});
mapPick.addEventListener("change", function () {
  deleteArmed = "";
  clearTimeout(deleteTimer);
  syncButtons();
});

buttons.goal.addEventListener("click", function () {
  if (buttons.goal.disabled) return;
  buttons.goal.disabled = true;
  if (tool === "reloc") {
    fetch("/api/cmd", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ op: "reloc_stop" }),
    }).catch(function () {});
  }
  tool = tool === "goal" ? "" : "goal";
  setStatus(tool === "goal" ? "左键在地面上按住，拖出导航方向" : "已取消导航点选择");
  syncButtons();
});

buttons.reloc.addEventListener("click", function () {
  if (buttons.reloc.disabled) return;
  if (state.mode !== "nav") {
    setStatus("重定位只在已知地图导航时可用");
    return;
  }
  if (tool === "reloc") {
    tool = "";
    setStatus("已停止转动，可以发导航点");
    syncButtons();
    fetch("/api/cmd", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ op: "reloc_stop" }),
    }).catch(function () {});
    return;
  }
  tool = "reloc";
  syncButtons();
  setStatus("正在用雷达自动对齐地图…");
  call("reloc_global");
});

followBtn.addEventListener("click", function () {
  follow = !follow;
  if (viewer) viewer.setFollow(follow);
  syncButtons();
});
document.getElementById("zin").addEventListener("click", function () { if (viewer) viewer.zoom(0.8); });
document.getElementById("zout").addEventListener("click", function () { if (viewer) viewer.zoom(1.25); });

function loadImage(blob) {
  return new Promise(function (resolve, reject) {
    const url = URL.createObjectURL(blob);
    const img = new Image();
    img.onload = function () {
      URL.revokeObjectURL(url);
      resolve(img);
    };
    img.onerror = function () {
      URL.revokeObjectURL(url);
      reject(new Error("image"));
    };
    img.src = url;
  });
}

async function loadCost(kind, meta) {
  if (!viewer) return;
  if (meta.clear) {
    viewer.clearCost(kind);
    return;
  }
  const token = ++costToken[kind];
  const res = await fetch("costmap/" + kind + ".png?rev=" + meta.rev);
  if (!res.ok || token !== costToken[kind]) return;
  const blob = await res.blob();
  if (token !== costToken[kind]) return;
  viewer.setCost(kind, await loadImage(blob), meta);
}

async function loadRgb(meta) {
  if (!viewer) return;
  if (meta.clear) {
    viewer.setRgb(null);
    return;
  }
  const token = ++rgbToken;
  const res = await fetch("cloud/rgb.bin?rev=" + meta.rev);
  if (!res.ok || token !== rgbToken) return;
  const buf = await res.arrayBuffer();
  if (token !== rgbToken || buf.byteLength < 4) return;
  const count = new DataView(buf).getUint32(0, true);
  if (count <= 0 || buf.byteLength < 4 + count * 24) {
    viewer.setRgb(null);
    return;
  }
  viewer.setRgb(new Float32Array(buf, 4, count * 6));
}

async function loadMap(meta) {
  if (!viewer) return;
  if (meta.clear) {
    viewer.clearMap();
    mapInfo.textContent = "等待地图";
    return;
  }
  const token = ++mapToken;
  const res = await fetch("map.png?rev=" + meta.rev);
  if (!res.ok || token !== mapToken) return;
  const blob = await res.blob();
  if (token !== mapToken) return;
  viewer.setMap(await loadImage(blob), meta);
  mapInfo.textContent = meta.width + "×" + meta.height + " · " + Math.round(meta.resolution * 100) + " cm/格";
}

function connect() {
  const es = new EventSource("/events");
  es.addEventListener("pose", function (ev) {
    const data = JSON.parse(ev.data);
    poseCount += 1;
    if (typeof data.bandwidth === "number") bwEl.textContent = Math.round(data.bandwidth / 1024) + " KB/s";
    audioLeft.style.width = (Math.max(0, Math.min(1, data.audio_l || 0)) * 100).toFixed(1) + "%";
    audioRight.style.width = (Math.max(0, Math.min(1, data.audio_r || 0)) * 100).toFixed(1) + "%";
    pose = data;
    if (viewer) {
      viewer.setPose(data);
      viewer.setRobot({
        radius: state.robot_radius,
        lidarX: state.lidar_x,
        wheelY: state.wheel_y,
        wheelRadius: state.wheel_radius,
      });
    }
  });
  es.addEventListener("state", function (ev) { applyState(JSON.parse(ev.data)); });
  es.addEventListener("map", function (ev) {
    loadMap(JSON.parse(ev.data)).catch(function () { setStatus("地图刷新失败"); });
  });
  es.addEventListener("cloud", function (ev) {
    const data = JSON.parse(ev.data);
    if (!viewer) return;
    if (Array.isArray(data.laser)) viewer.setLaser(data.laser);
    if (Array.isArray(data.depth)) viewer.setDepth(data.depth);
  });
  es.addEventListener("rgb", function (ev) {
    loadRgb(JSON.parse(ev.data)).catch(function () {});
  });
  es.addEventListener("paths", function (ev) {
    const data = JSON.parse(ev.data);
    if (viewer) viewer.setPaths(data.global || [], data.local || []);
  });
  es.addEventListener("costmap", function (ev) {
    const meta = JSON.parse(ev.data);
    loadCost(meta.kind, meta).catch(function () {});
  });
  es.onerror = function () {
    es.close();
    setTimeout(connect, 1000);
  };
}

const keys = new Set();
let stickVec = null;
let teleopOn = false;

function drivingAllowed() {
  return !document.body.classList.contains("is-idle");
}

let keyboardEnabled = true;

function twistFromInput() {
  if (!drivingAllowed()) {
    return { linear: 0, angular: 0, active: false };
  }
  let linear = 0;
  let angular = 0;
  if (keyboardEnabled && (keys.has("w") || keys.has("arrowup"))) linear += teleopV;
  if (keyboardEnabled && (keys.has("s") || keys.has("arrowdown"))) linear -= teleopV;
  if (keyboardEnabled && (keys.has("a") || keys.has("arrowleft"))) angular += teleopW;
  if (keyboardEnabled && (keys.has("d") || keys.has("arrowright"))) angular -= teleopW;
  if (stickVec) {
    linear = -stickVec.y * teleopV;
    angular = -stickVec.x * teleopW;
  }
  return {
    linear: Math.max(-teleopV, Math.min(teleopV, linear)),
    angular: Math.max(-teleopW, Math.min(teleopW, angular)),
    active: stickVec !== null || keys.size > 0,
  };
}

function placeKnob(vec) {
  const radius = stick.clientWidth / 2;
  const knobR = knob.clientWidth / 2;
  const x = vec ? vec.x * (radius - knobR) : 0;
  const y = vec ? vec.y * (radius - knobR) : 0;
  knob.style.left = (radius - knobR + x) + "px";
  knob.style.top = (radius - knobR + y) + "px";
}

function stickPointer(event) {
  const rect = stick.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  let dx = event.clientX - cx;
  let dy = event.clientY - cy;
  const limit = rect.width / 2;
  const dist = Math.hypot(dx, dy) || 1;
  if (dist > limit) {
    dx *= limit / dist;
    dy *= limit / dist;
  }
  stickVec = { x: dx / limit, y: dy / limit };
  placeKnob(stickVec);
}

stick.addEventListener("pointerdown", function (event) {
  if (!drivingAllowed()) return;
  stick.setPointerCapture(event.pointerId);
  stickPointer(event);
});
stick.addEventListener("pointermove", function (event) {
  if (stickVec) stickPointer(event);
});
function stickUp() {
  stickVec = null;
  placeKnob(null);
}
stick.addEventListener("pointerup", stickUp);
stick.addEventListener("pointercancel", stickUp);

window.addEventListener("keydown", function (event) {
  if (event.target.closest("select, input, textarea")) return;
  if (!drivingAllowed() || !keyboardEnabled) return;
  const key = event.key.toLowerCase();
  if (["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright", " "].indexOf(key) >= 0) {
    event.preventDefault();
  }
  if (key === " ") {
    keys.clear();
    stickVec = null;
    placeKnob(null);
    return;
  }
  keys.add(key);
});
window.addEventListener("keyup", function (event) { keys.delete(event.key.toLowerCase()); });
window.addEventListener("blur", function () {
  keys.clear();
  stickVec = null;
  placeKnob(null);
});

setInterval(function () {
  const twist = twistFromInput();
  if (twist.active) {
    teleopOn = true;
    fetch("/api/cmd", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ op: "teleop", linear: twist.linear, angular: twist.angular }),
    }).catch(function () {});
  } else if (teleopOn) {
    teleopOn = false;
    fetch("/api/cmd", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ op: "teleop", linear: 0, angular: 0 }),
    }).catch(function () {});
  }
}, 50);

cam.addEventListener("error", function () {
  setTimeout(function () { cam.src = "camera.mjpg?t=" + Date.now(); }, 1000);
});

window.addEventListener("resize", function () { if (viewer) viewer.resize(); });

(function () {
  const splitter = document.getElementById("splitter");
  const stage = document.getElementById("stage");
  const slotMain = document.getElementById("slotMain");
  const pipFrame = document.getElementById("pipFrame");
  const view3d = document.getElementById("view3d");
  const viewRgb = document.getElementById("viewRgb");
  const swap = document.getElementById("swap");
  const pip = document.getElementById("pip");
  const pipResize = document.getElementById("pipResize");
  let dragging = false;
  let pipDrag = null;

  function pipMetrics() {
    const stageRect = stage.getBoundingClientRect();
    const pipRect = pip.getBoundingClientRect();
    const frameRect = pipFrame.getBoundingClientRect();
    return {
      left: pipRect.left - stageRect.left,
      top: pipRect.top - stageRect.top,
      width: pipRect.width,
      height: frameRect.height,
      chrome: pipRect.height - frameRect.height,
    };
  }

  function placePip(box) {
    const bounds = stage.getBoundingClientRect();
    const chrome = box.chrome != null ? box.chrome : pipMetrics().chrome;
    const width = Math.max(180, Math.min(bounds.width - 8, box.width));
    let height = box.lockRatio ? width * 3 / 4 : Math.max(110, box.height);
    const maxH = Math.max(110, bounds.height - chrome - 8);
    if (height > maxH) {
      height = maxH;
      if (box.lockRatio) {
        const fitted = height * 4 / 3;
        pip.style.width = Math.max(180, fitted) + "px";
      }
    }
    const usedWidth = box.lockRatio && height === maxH ? Math.max(180, height * 4 / 3) : width;
    const left = Math.max(0, Math.min(bounds.width - usedWidth, box.left));
    const top = Math.max(0, Math.min(bounds.height - height - chrome, box.top));
    pip.style.left = left + "px";
    pip.style.top = top + "px";
    pip.style.bottom = "auto";
    if (!(box.lockRatio && height === maxH)) pip.style.width = usedWidth + "px";
    pipFrame.style.height = height + "px";
    if (viewer) viewer.resize();
  }

  viewRgb.addEventListener("pointerdown", function (event) {
    if (!pipFrame.contains(viewRgb) || event.target.closest("#swap, #pipResize")) return;
    const box = pipMetrics();
    pipDrag = { kind: "move", id: event.pointerId, x: event.clientX, y: event.clientY, box: box };
    viewRgb.setPointerCapture(event.pointerId);
    event.preventDefault();
  });
  pipResize.addEventListener("pointerdown", function (event) {
    const box = pipMetrics();
    pipDrag = { kind: "resize", id: event.pointerId, x: event.clientX, y: event.clientY, box: box };
    pipResize.setPointerCapture(event.pointerId);
    event.stopPropagation();
    event.preventDefault();
  });
  window.addEventListener("pointermove", function (event) {
    if (!pipDrag || event.pointerId !== pipDrag.id) return;
    const dx = event.clientX - pipDrag.x;
    const dy = event.clientY - pipDrag.y;
    if (pipDrag.kind === "move") {
      placePip({
        left: pipDrag.box.left + dx,
        top: pipDrag.box.top + dy,
        width: pipDrag.box.width,
        height: pipDrag.box.height,
        chrome: pipDrag.box.chrome,
      });
    } else {
      placePip({
        left: pipDrag.box.left,
        top: pipDrag.box.top + pipDrag.box.height - (pipDrag.box.width + dx) * 3 / 4,
        width: pipDrag.box.width + dx,
        height: (pipDrag.box.width + dx) * 3 / 4,
        chrome: pipDrag.box.chrome,
        lockRatio: true,
      });
    }
  });
  function endPip(event) {
    if (pipDrag && event.pointerId === pipDrag.id) pipDrag = null;
  }
  window.addEventListener("pointerup", endPip);
  window.addEventListener("pointercancel", endPip);

  splitter.addEventListener("pointerdown", function (event) {
    dragging = true;
    splitter.setPointerCapture(event.pointerId);
  });
  splitter.addEventListener("pointermove", function (event) {
    if (!dragging) return;
    const width = Math.max(220, Math.min(window.innerWidth * 0.55, event.clientX));
    document.documentElement.style.setProperty("--side", width + "px");
    if (viewer) viewer.resize();
    placeKnob(stickVec);
  });
  function endDrag() { dragging = false; }
  splitter.addEventListener("pointerup", endDrag);
  splitter.addEventListener("pointercancel", endDrag);

  swap.addEventListener("click", function () {
    const rgbLarge = stage.classList.toggle("swapped");
    if (rgbLarge) {
      slotMain.appendChild(viewRgb);
      pipFrame.appendChild(view3d);
    } else {
      slotMain.appendChild(view3d);
      pipFrame.appendChild(viewRgb);
    }
    if (viewer) viewer.resize();
  });
})();

syncButtons();
connect();
bindSpeeds();
bindSystem();
fetch("/health").then(function (res) { return res.json(); }).then(function (data) {
  applyState(data);
  applySavedSettings(data.settings);
}).catch(function () {});

function bindSystem() {
  const rebootBtn = document.getElementById("btnReboot");
  const shutdownBtn = document.getElementById("btnShutdown");
  let armed = "";
  let timer = 0;

  function labels() {
    rebootBtn.textContent = armed === "reboot" ? "确认重启" : "重启";
    shutdownBtn.textContent = armed === "shutdown" ? "确认关机" : "关机";
  }

  function arm(action) {
    if (armed === action) {
      armed = "";
      clearTimeout(timer);
      labels();
      runSystem(action);
      return;
    }
    armed = action;
    labels();
    setStatus(action === "reboot" ? "再点一次确认重启整台香橙派" : "再点一次确认关机");
    clearTimeout(timer);
    timer = setTimeout(function () {
      armed = "";
      labels();
    }, 4000);
  }

  rebootBtn.addEventListener("click", function () { arm("reboot"); });
  shutdownBtn.addEventListener("click", function () { arm("shutdown"); });
}

function runSystem(action) {
  const rebootBtn = document.getElementById("btnReboot");
  const shutdownBtn = document.getElementById("btnShutdown");
  rebootBtn.disabled = true;
  shutdownBtn.disabled = true;
  setStatus(action === "reboot" ? "正在重启" : "正在关机");
  fetch("/api/cmd", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ op: "system", action: action, confirm: true }),
  }).then(function (res) { return res.json(); }).then(function (data) {
    if (!data.ok) {
      rebootBtn.disabled = false;
      shutdownBtn.disabled = false;
      setStatus(data.error || "系统操作失败");
    }
  }).catch(function () {
    setStatus(action === "reboot" ? "正在重启，请稍候再打开页面" : "正在关机");
  });
}

function bindSpeeds() {
  const settings = document.getElementById("settings");
  document.getElementById("openSettings").addEventListener("click", function () {
    settings.hidden = !settings.hidden;
  });
  document.getElementById("closeSettings").addEventListener("click", function () {
    settings.hidden = true;
  });
  document.getElementById("keyEnable").addEventListener("change", function () {
    keyboardEnabled = this.checked;
    if (!keyboardEnabled) keys.clear();
    postSpeeds();
  });
  const fields = [
    ["teleopV", "teleopVVal", function (v) { teleopV = v; }, 2],
    ["teleopW", "teleopWVal", function (v) { teleopW = v; }, 2],
    ["navV", "navVVal", function () {}, 2],
    ["navW", "navWVal", function () {}, 2],
  ];
  fields.forEach(function (item) {
    const input = document.getElementById(item[0]);
    const label = document.getElementById(item[1]);
    input.addEventListener("input", function () {
      const value = parseFloat(input.value);
      label.textContent = value.toFixed(item[3]);
      item[2](value);
      postSpeeds();
    });
  });
}

function applySavedSettings(settings) {
  if (!settings) return;
  const rows = [
    ["teleopV", "teleopVVal", "teleop_v", function (v) { teleopV = v; }],
    ["teleopW", "teleopWVal", "teleop_w", function (v) { teleopW = v; }],
    ["navV", "navVVal", "nav_v", function () {}],
    ["navW", "navWVal", "nav_w", function () {}],
  ];
  rows.forEach(function (row) {
    const value = Number(settings[row[2]]);
    if (!isFinite(value)) return;
    const input = document.getElementById(row[0]);
    const label = document.getElementById(row[1]);
    input.value = String(value);
    label.textContent = value.toFixed(2);
    row[3](value);
  });
  if (typeof settings.keyboard === "boolean") {
    keyboardEnabled = settings.keyboard;
    document.getElementById("keyEnable").checked = settings.keyboard;
  }
}

let speedTimer = 0;
function postSpeeds() {
  clearTimeout(speedTimer);
  speedTimer = setTimeout(function () {
    fetch("/api/cmd", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        op: "speeds",
        teleop_v: teleopV,
        teleop_w: teleopW,
        nav_v: parseFloat(document.getElementById("navV").value),
        nav_w: parseFloat(document.getElementById("navW").value),
        keyboard: keyboardEnabled,
      }),
    }).catch(function () {});
  }, 120);
}
