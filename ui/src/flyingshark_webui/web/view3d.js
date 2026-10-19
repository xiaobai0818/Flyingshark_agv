/* Orbit view: left drag rotates, right drag pans, wheel zooms. */
const View3D = (function () {
  const canvas = document.getElementById("map");
  const gl = canvas.getContext("webgl", { antialias: true, alpha: false });
  if (!gl) {
    return null;
  }

  const cam = { target: [0, 0, 0], yaw: 0.9, pitch: 1.05, dist: 8 };
  let follow = true;
  let pose = null;
  let robot = { radius: 0.108, lidarX: 0.06, wheelY: 0.082, wheelRadius: 0.037 };
  let mapTex = null;
  let mapMeta = null;
  const costTex = { global: null, local: null };
  const costMeta = { global: null, local: null };
  let laser = null;
  let depth = null;
  let rgbCloud = null;
  let globalPath = null;
  let localPath = null;
  let goal = null;
  let drag = null;
  let initialPose = null;
  let cssW = 1;
  let cssH = 1;
  let mvp = new Float32Array(16);
  const hooks = { tool: function () { return ""; }, onPick: function () {}, onFollowOff: function () {}, onFrame: function () {} };

  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
  function scale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function len(a) { return Math.hypot(a[0], a[1], a[2]); }
  function norm(a) {
    const l = len(a) || 1;
    return [a[0] / l, a[1] / l, a[2] / l];
  }
  function cross(a, b) {
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  }
  function mul(a, b) {
    const o = new Float32Array(16);
    for (let c = 0; c < 4; c++) {
      for (let r = 0; r < 4; r++) {
        o[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] + a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
      }
    }
    return o;
  }
  function perspective(fovy, aspect, near, far) {
    const f = 1 / Math.tan(fovy / 2);
    const nf = 1 / (near - far);
    const o = new Float32Array(16);
    o[0] = f / aspect;
    o[5] = f;
    o[10] = (far + near) * nf;
    o[11] = -1;
    o[14] = 2 * far * near * nf;
    return o;
  }
  function lookAt(eye, center, up) {
    const z = norm(sub(eye, center));
    const x = norm(cross(up, z));
    const y = cross(z, x);
    const o = new Float32Array(16);
    o[0] = x[0]; o[1] = y[0]; o[2] = z[0];
    o[4] = x[1]; o[5] = y[1]; o[6] = z[1];
    o[8] = x[2]; o[9] = y[2]; o[10] = z[2];
    o[12] = -dot(x, eye); o[13] = -dot(y, eye); o[14] = -dot(z, eye); o[15] = 1;
    return o;
  }
  function eyePos() {
    const cp = Math.cos(cam.pitch);
    const sp = Math.sin(cam.pitch);
    const cy = Math.cos(cam.yaw);
    const sy = Math.sin(cam.yaw);
    return [
      cam.target[0] + cam.dist * cp * cy,
      cam.target[1] + cam.dist * cp * sy,
      cam.target[2] + cam.dist * sp,
    ];
  }
  function basis() {
    const e = eyePos();
    const f = norm(sub(cam.target, e));
    let r = cross(f, [0, 0, 1]);
    if (len(r) < 1e-4) r = [1, 0, 0];
    r = norm(r);
    const u = norm(cross(r, f));
    return { e: e, f: f, r: r, u: u };
  }

  function compile(type, src) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, src);
    gl.compileShader(shader);
    return shader;
  }
  function program(vs, fs) {
    const p = gl.createProgram();
    gl.attachShader(p, compile(gl.VERTEX_SHADER, vs));
    gl.attachShader(p, compile(gl.FRAGMENT_SHADER, fs));
    gl.linkProgram(p);
    return p;
  }
  const colorProg = program(
    "attribute vec3 aPos;uniform mat4 uMvp;uniform float uPoint;void main(){gl_Position=uMvp*vec4(aPos,1.0);gl_PointSize=uPoint;}",
    "precision mediump float;uniform vec4 uColor;void main(){gl_FragColor=uColor;}"
  );
  const rgbProg = program(
    "attribute vec3 aPos;attribute vec3 aColor;uniform mat4 uMvp;uniform float uPoint;varying vec3 vColor;void main(){vColor=aColor;gl_Position=uMvp*vec4(aPos,1.0);gl_PointSize=uPoint;}",
    "precision mediump float;varying vec3 vColor;void main(){gl_FragColor=vec4(vColor,1.0);}"
  );
  const texProg = program(
    "attribute vec3 aPos;attribute vec2 aUv;uniform mat4 uMvp;varying vec2 vUv;void main(){vUv=aUv;gl_Position=uMvp*vec4(aPos,1.0);}",
    "precision mediump float;varying vec2 vUv;uniform sampler2D uTex;uniform float uAlpha;void main(){vec4 c=texture2D(uTex,vUv);gl_FragColor=vec4(c.rgb,c.a*uAlpha);}"
  );
  const buf = gl.createBuffer();

  function uploadTexture(image) {
    const tex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, tex);
    gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, true);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, image);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    return tex;
  }
  function dropTex(tex) {
    if (tex) gl.deleteTexture(tex);
  }

  function gridLines() {
    const lines = [];
    for (let i = -40; i <= 40; i++) {
      const major = i % 5 === 0 ? 1 : 0;
      lines.push(-40, i, 0, major, 40, i, 0, major);
      lines.push(i, -40, 0, major, i, 40, 0, major);
    }
    return new Float32Array(lines);
  }
  const gridData = gridLines();

  function drawArrays(prog, data, mode, color, pointSize) {
    gl.useProgram(prog);
    gl.uniformMatrix4fv(gl.getUniformLocation(prog, "uMvp"), false, mvp);
    gl.uniform4fv(gl.getUniformLocation(prog, "uColor"), color);
    const loc = gl.getAttribLocation(prog, "aPos");
    const pLoc = gl.getUniformLocation(prog, "uPoint");
    if (pLoc) gl.uniform1f(pLoc, pointSize || 1);
    gl.bindBuffer(gl.ARRAY_BUFFER, buf);
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.DYNAMIC_DRAW);
    gl.enableVertexAttribArray(loc);
    gl.vertexAttribPointer(loc, 3, gl.FLOAT, false, 0, 0);
    gl.drawArrays(mode, 0, data.length / 3);
  }

  function quadCorners(meta) {
    const w = meta.width * meta.resolution;
    const h = meta.height * meta.resolution;
    const yaw = meta.yaw || 0;
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    function at(gx, gy) {
      return [meta.origin_x + c * gx - s * gy, meta.origin_y + s * gx + c * gy];
    }
    return [at(0, h), at(w, h), at(w, 0), at(0, 0)];
  }

  function drawTextured(tex, meta, z, alpha) {
    if (!tex || !meta || meta.clear) return;
    const c = quadCorners(meta);
    const data = new Float32Array([
      c[0][0], c[0][1], z, 0, 1,
      c[1][0], c[1][1], z, 1, 1,
      c[2][0], c[2][1], z, 1, 0,
      c[0][0], c[0][1], z, 0, 1,
      c[2][0], c[2][1], z, 1, 0,
      c[3][0], c[3][1], z, 0, 0,
    ]);
    gl.useProgram(texProg);
    gl.uniformMatrix4fv(gl.getUniformLocation(texProg, "uMvp"), false, mvp);
    gl.uniform1f(gl.getUniformLocation(texProg, "uAlpha"), alpha);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, tex);
    gl.uniform1i(gl.getUniformLocation(texProg, "uTex"), 0);
    const stride = 20;
    gl.bindBuffer(gl.ARRAY_BUFFER, buf);
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.DYNAMIC_DRAW);
    const p = gl.getAttribLocation(texProg, "aPos");
    const u = gl.getAttribLocation(texProg, "aUv");
    gl.enableVertexAttribArray(p);
    gl.enableVertexAttribArray(u);
    gl.vertexAttribPointer(p, 3, gl.FLOAT, false, stride, 0);
    gl.vertexAttribPointer(u, 2, gl.FLOAT, false, stride, 12);
    gl.drawArrays(gl.TRIANGLES, 0, 6);
    gl.disableVertexAttribArray(u);
  }

  function pointsToArray(points, zLift) {
    if (!points || !points.length) return null;
    const data = new Float32Array(points.length * 3);
    for (let i = 0; i < points.length; i++) {
      data[i * 3] = points[i][0];
      data[i * 3 + 1] = points[i][1];
      data[i * 3 + 2] = (points[i].length > 2 ? points[i][2] : 0) + (zLift || 0);
    }
    return data;
  }

  function robotParts() {
    if (!pose || !pose.ok) return null;
    const c = Math.cos(pose.yaw);
    const s = Math.sin(pose.yaw);
    const local = [];
    function tri(x0, y0, z0, x1, y1, z1, x2, y2, z2) {
      local.push(x0, y0, z0, x1, y1, z1, x2, y2, z2);
    }
    function cylinderZ(cx, cy, cz, radius, height, n) {
      const z0 = cz - height / 2;
      const z1 = cz + height / 2;
      for (let i = 0; i < n; i++) {
        const a0 = (i / n) * Math.PI * 2;
        const a1 = ((i + 1) / n) * Math.PI * 2;
        const x0 = Math.cos(a0) * radius;
        const y0 = Math.sin(a0) * radius;
        const x1 = Math.cos(a1) * radius;
        const y1 = Math.sin(a1) * radius;
        tri(cx + x0, cy + y0, z0, cx + x1, cy + y1, z0, cx + x1, cy + y1, z1);
        tri(cx + x0, cy + y0, z0, cx + x1, cy + y1, z1, cx + x0, cy + y0, z1);
        tri(cx, cy, z0, cx + x1, cy + y1, z0, cx + x0, cy + y0, z0);
        tri(cx, cy, z1, cx + x0, cy + y0, z1, cx + x1, cy + y1, z1);
      }
    }
    function cylinderY(cx, cy, cz, radius, length, n) {
      const y0 = cy - length / 2;
      const y1 = cy + length / 2;
      for (let i = 0; i < n; i++) {
        const a0 = (i / n) * Math.PI * 2;
        const a1 = ((i + 1) / n) * Math.PI * 2;
        const x0 = Math.cos(a0) * radius;
        const z0 = Math.sin(a0) * radius;
        const x1 = Math.cos(a1) * radius;
        const z1 = Math.sin(a1) * radius;
        tri(cx + x0, y0, cz + z0, cx + x1, y0, cz + z1, cx + x1, y1, cz + z1);
        tri(cx + x0, y0, cz + z0, cx + x1, y1, cz + z1, cx + x0, y1, cz + z0);
        tri(cx, y0, cz, cx + x1, y0, cz + z1, cx + x0, y0, cz + z0);
        tri(cx, y1, cz, cx + x0, y1, cz + z0, cx + x1, y1, cz + z1);
      }
    }
    function box(cx, cy, cz, sx, sy, sz) {
      const dx = sx / 2;
      const dy = sy / 2;
      const dz = sz / 2;
      const p = [
        [cx - dx, cy - dy, cz - dz], [cx + dx, cy - dy, cz - dz],
        [cx + dx, cy + dy, cz - dz], [cx - dx, cy + dy, cz - dz],
        [cx - dx, cy - dy, cz + dz], [cx + dx, cy - dy, cz + dz],
        [cx + dx, cy + dy, cz + dz], [cx - dx, cy + dy, cz + dz],
      ];
      const faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [1, 2, 6, 5], [0, 3, 7, 4]];
      faces.forEach(function (f) {
        tri(p[f[0]][0], p[f[0]][1], p[f[0]][2], p[f[1]][0], p[f[1]][1], p[f[1]][2], p[f[2]][0], p[f[2]][1], p[f[2]][2]);
        tri(p[f[0]][0], p[f[0]][1], p[f[0]][2], p[f[2]][0], p[f[2]][1], p[f[2]][2], p[f[3]][0], p[f[3]][1], p[f[3]][2]);
      });
    }
    const wheelCount = local.length;
    const wheelY = robot.wheelY || 0.082;
    const wheelRadius = robot.wheelRadius || 0.037;
    cylinderY(0, wheelY, wheelRadius, wheelRadius, 0.03, 16);
    cylinderY(0, -wheelY, wheelRadius, wheelRadius, 0.03, 16);
    const bodyCount = local.length;
    cylinderZ(0, 0, wheelRadius + 0.02, robot.radius || 0.108, 0.04, 28);
    const lidarCount = local.length;
    cylinderZ(0.06, 0, 0.105, 0.035, 0.04, 16);
    const cameraCount = local.length;
    box(0.055, 0, 0.16, 0.09, 0.03, 0.025);
    const world = new Float32Array(local.length);
    for (let i = 0; i < local.length; i += 3) {
      const x = local[i];
      const y = local[i + 1];
      world[i] = pose.x + c * x - s * y;
      world[i + 1] = pose.y + s * x + c * y;
      world[i + 2] = local[i + 2];
    }
    return {
      wheels: world.subarray(wheelCount, bodyCount),
      body: world.subarray(bodyCount, lidarCount),
      lidar: world.subarray(lidarCount, cameraCount),
      camera: world.subarray(cameraCount),
    };
  }

  function axisLines(x, y, z, yaw, length) {
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    return {
      x: new Float32Array([x, y, z, x + c * length, y + s * length, z]),
      y: new Float32Array([x, y, z, x - s * length, y + c * length, z]),
      z: new Float32Array([x, y, z, x, y, z + length]),
    };
  }

  function drawAxes(framePose, length) {
    const axes = axisLines(framePose.x, framePose.y, 0.02, framePose.yaw || 0, length);
    drawArrays(colorProg, axes.x, gl.LINES, [0.9, 0.25, 0.2, 1], 1);
    drawArrays(colorProg, axes.y, gl.LINES, [0.25, 0.75, 0.3, 1], 1);
    drawArrays(colorProg, axes.z, gl.LINES, [0.25, 0.45, 0.95, 1], 1);
  }

  function drawRgbCloud() {
    if (!rgbCloud || rgbCloud.length < 6) return;
    gl.useProgram(rgbProg);
    gl.uniformMatrix4fv(gl.getUniformLocation(rgbProg, "uMvp"), false, mvp);
    gl.uniform1f(gl.getUniformLocation(rgbProg, "uPoint"), 4);
    gl.bindBuffer(gl.ARRAY_BUFFER, buf);
    gl.bufferData(gl.ARRAY_BUFFER, rgbCloud, gl.DYNAMIC_DRAW);
    const pos = gl.getAttribLocation(rgbProg, "aPos");
    const color = gl.getAttribLocation(rgbProg, "aColor");
    gl.enableVertexAttribArray(pos);
    gl.enableVertexAttribArray(color);
    gl.vertexAttribPointer(pos, 3, gl.FLOAT, false, 24, 0);
    gl.vertexAttribPointer(color, 3, gl.FLOAT, false, 24, 12);
    gl.drawArrays(gl.POINTS, 0, rgbCloud.length / 6);
    gl.disableVertexAttribArray(color);
  }

  function arrowLine(item, z) {
    if (!item) return null;
    const x2 = item.x + Math.cos(item.yaw) * 0.7;
    const y2 = item.y + Math.sin(item.yaw) * 0.7;
    return new Float32Array([item.x, item.y, z, x2, y2, z]);
  }

  function drawColoredGrid() {
    gl.useProgram(colorProg);
    gl.uniformMatrix4fv(gl.getUniformLocation(colorProg, "uMvp"), false, mvp);
    gl.bindBuffer(gl.ARRAY_BUFFER, buf);
    gl.bufferData(gl.ARRAY_BUFFER, gridData, gl.DYNAMIC_DRAW);
    const loc = gl.getAttribLocation(colorProg, "aPos");
    gl.enableVertexAttribArray(loc);
    gl.vertexAttribPointer(loc, 3, gl.FLOAT, false, 16, 0);
    const major = [];
    const minor = [];
    for (let i = 0; i < gridData.length; i += 8) {
      const dest = gridData[i + 3] ? major : minor;
      dest.push(gridData[i], gridData[i + 1], gridData[i + 2], gridData[i + 4], gridData[i + 5], gridData[i + 6]);
    }
    drawArrays(colorProg, new Float32Array(minor), gl.LINES, [0.18, 0.2, 0.22, 1], 1);
    drawArrays(colorProg, new Float32Array(major), gl.LINES, [0.32, 0.36, 0.38, 1], 1);
    drawArrays(colorProg, new Float32Array([-40, 0, 0.001, 40, 0, 0.001]), gl.LINES, [0.75, 0.28, 0.25, 1], 1);
    drawArrays(colorProg, new Float32Array([0, -40, 0.001, 0, 40, 0.001]), gl.LINES, [0.3, 0.65, 0.35, 1], 1);
  }

  function resize() {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    cssW = Math.max(1, rect.width);
    cssH = Math.max(1, rect.height);
    canvas.width = Math.round(cssW * dpr);
    canvas.height = Math.round(cssH * dpr);
    gl.viewport(0, 0, canvas.width, canvas.height);
  }

  function frame() {
    hooks.onFrame();
    if (follow && pose && pose.ok && !panning) {
      cam.target[0] = pose.x;
      cam.target[1] = pose.y;
    }
    const b = basis();
    const proj = perspective(0.9, cssW / cssH, 0.05, 200);
    const view = lookAt(b.e, cam.target, [0, 0, 1]);
    mvp = mul(proj, view);
    gl.enable(gl.DEPTH_TEST);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.clearColor(0.055, 0.07, 0.065, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    drawColoredGrid();
    drawTextured(mapTex, mapMeta, 0.01, 1);
    drawTextured(costTex.global, costMeta.global, 0.02, 0.85);
    drawTextured(costTex.local, costMeta.local, 0.03, 0.9);
    const laserData = pointsToArray(laser, 0);
    if (laserData) drawArrays(colorProg, laserData, gl.POINTS, [0.35, 0.85, 0.82, 1], 4);
    const depthData = pointsToArray(depth, 0);
    if (depthData) drawArrays(colorProg, depthData, gl.POINTS, [0.82, 0.48, 1, 0.35], 3);
    drawRgbCloud();
    const gPath = pointsToArray(globalPath, 0.06);
    if (gPath) drawArrays(colorProg, gPath, gl.LINE_STRIP, [0.9, 0.64, 0.24, 1], 1);
    const lPath = pointsToArray(localPath, 0.07);
    if (lPath) drawArrays(colorProg, lPath, gl.LINE_STRIP, [0.49, 0.81, 0.63, 1], 1);
    if (drag && drag.moved) {
      const line = arrowLine({ x: drag.start.x, y: drag.start.y, yaw: Math.atan2(drag.end.y - drag.start.y, drag.end.x - drag.start.x) }, 0.08);
      if (line) drawArrays(colorProg, line, gl.LINES, hooks.tool() === "reloc" ? [0.49, 0.81, 0.63, 1] : [0.9, 0.64, 0.24, 1], 1);
    } else if (goal) {
      const line = arrowLine(goal, 0.08);
      if (line) drawArrays(colorProg, line, gl.LINES, [0.96, 0.94, 0.9, 1], 1);
    }
    drawAxes({ x: 0, y: 0, yaw: 0 }, 1.2);
    if (initialPose) drawAxes(initialPose, 0.55);
    const parts = robotParts();
    if (parts) {
      drawArrays(colorProg, parts.wheels, gl.TRIANGLES, [0.12, 0.12, 0.12, 1], 1);
      drawArrays(colorProg, parts.body, gl.TRIANGLES, [0.16, 0.38, 0.58, 1], 1);
      drawArrays(colorProg, parts.lidar, gl.TRIANGLES, [0.8, 0.15, 0.15, 1], 1);
      drawArrays(colorProg, parts.camera, gl.TRIANGLES, [0.05, 0.05, 0.05, 1], 1);
    }
    requestAnimationFrame(frame);
  }

  function groundPoint(clientX, clientY) {
    const rect = canvas.getBoundingClientRect();
    const ndcX = ((clientX - rect.left) / rect.width) * 2 - 1;
    const ndcY = 1 - ((clientY - rect.top) / rect.height) * 2;
    const b = basis();
    const tan = Math.tan(0.45);
    const dir = norm(add(b.f, add(scale(b.r, ndcX * tan * (cssW / cssH)), scale(b.u, ndcY * tan))));
    if (Math.abs(dir[2]) < 1e-5) return null;
    const t = -b.e[2] / dir[2];
    if (t < 0) return null;
    return { x: b.e[0] + dir[0] * t, y: b.e[1] + dir[1] * t };
  }

  let panning = false;
  const pointers = new Map();
  let gesture = null;

  canvas.addEventListener("contextmenu", function (event) { event.preventDefault(); });
  canvas.addEventListener("pointerdown", function (event) {
    canvas.setPointerCapture(event.pointerId);
    pointers.set(event.pointerId, { x: event.clientX, y: event.clientY, button: event.button });
    if (pointers.size >= 2) {
      gesture = { kind: "pinch", dist: pinchDist(), dist0: cam.dist, pan: averagePointer() };
      drag = null;
      return;
    }
    const picking = (hooks.tool() === "goal" || hooks.tool() === "reloc") && event.button === 0;
    if (picking) {
      const world = groundPoint(event.clientX, event.clientY);
      if (world) drag = { id: event.pointerId, start: world, end: world, moved: false };
      gesture = null;
      return;
    }
    if (event.button === 2 || event.button === 1) {
      gesture = { kind: "pan", id: event.pointerId, x: event.clientX, y: event.clientY };
      panning = true;
      follow = false;
      hooks.onFollowOff();
      return;
    }
    gesture = { kind: "orbit", id: event.pointerId, x: event.clientX, y: event.clientY };
  });
  function pinchDist() {
    const pts = Array.from(pointers.values());
    if (pts.length < 2) return 0;
    return Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y);
  }
  function averagePointer() {
    const pts = Array.from(pointers.values());
    return { x: (pts[0].x + pts[1].x) / 2, y: (pts[0].y + pts[1].y) / 2 };
  }
  canvas.addEventListener("pointermove", function (event) {
    if (!pointers.has(event.pointerId)) return;
    const prev = pointers.get(event.pointerId);
    pointers.set(event.pointerId, { x: event.clientX, y: event.clientY, button: prev.button });
    if (pointers.size >= 2 && gesture && gesture.kind === "pinch") {
      const dist = pinchDist();
      if (gesture.dist > 0) cam.dist = Math.max(1.2, Math.min(80, gesture.dist0 * gesture.dist / dist));
      const mid = averagePointer();
      panBy(mid.x - gesture.pan.x, mid.y - gesture.pan.y);
      gesture.pan = mid;
      return;
    }
    const dx = event.clientX - prev.x;
    const dy = event.clientY - prev.y;
    if (drag && event.pointerId === drag.id) {
      const world = groundPoint(event.clientX, event.clientY);
      if (world) {
        drag.end = world;
        const ddx = drag.end.x - drag.start.x;
        const ddy = drag.end.y - drag.start.y;
        drag.moved = ddx * ddx + ddy * ddy > 0.08 * 0.08;
      }
      return;
    }
    if (!gesture || gesture.id !== event.pointerId) return;
    if (gesture.kind === "orbit") {
      cam.yaw -= dx * 0.005;
      cam.pitch = Math.max(0.12, Math.min(1.45, cam.pitch + dy * 0.004));
    } else if (gesture.kind === "pan") {
      panBy(dx, dy);
    }
  });
  function panBy(dx, dy) {
    const b = basis();
    const scaleN = cam.dist * 0.0016;
    cam.target[0] += -dx * b.r[0] * scaleN + dy * b.f[0] * scaleN;
    cam.target[1] += -dx * b.r[1] * scaleN + dy * b.f[1] * scaleN;
  }
  function endPointer(event) {
    pointers.delete(event.pointerId);
    if (drag && event.pointerId === drag.id) {
      if (drag.moved || hooks.tool() === "reloc") hooks.onPick(drag.start, drag.end);
      drag = null;
    }
    if (gesture && gesture.id === event.pointerId) gesture = null;
    if (pointers.size < 2) panning = false;
  }
  canvas.addEventListener("pointerup", endPointer);
  canvas.addEventListener("pointercancel", endPointer);
  canvas.addEventListener("wheel", function (event) {
    event.preventDefault();
    cam.dist = Math.max(1.2, Math.min(80, cam.dist * (event.deltaY > 0 ? 1.1 : 0.9)));
  }, { passive: false });

  gl.clearColor(0.055, 0.07, 0.065, 1);
  window.addEventListener("resize", resize);
  if (window.ResizeObserver) {
    new ResizeObserver(function () { resize(); }).observe(canvas.parentElement || canvas);
  }
  resize();
  requestAnimationFrame(frame);

  return {
    setHooks: function (next) { Object.assign(hooks, next); },
    setFollow: function (on) { follow = on; },
    zoom: function (factor) { cam.dist = Math.max(1.2, Math.min(80, cam.dist * factor)); },
    setPose: function (next) {
      pose = next;
      if (next && next.ok && next.frame === "map" && !initialPose) {
        initialPose = { x: next.x, y: next.y, yaw: next.yaw || 0 };
      }
    },
    setRobot: function (spec) { robot = Object.assign(robot, spec); },
    setGoal: function (next) { goal = next; },
    setLaser: function (points) { laser = points; },
    setDepth: function (points) { depth = points; },
    setRgb: function (points) { rgbCloud = points; },
    setPaths: function (globalPts, localPts) { globalPath = globalPts; localPath = localPts; },
    setMap: function (bitmap, meta) {
      dropTex(mapTex);
      mapTex = bitmap ? uploadTexture(bitmap) : null;
      mapMeta = meta;
    },
    clearMap: function () { dropTex(mapTex); mapTex = null; mapMeta = null; },
    setCost: function (kind, bitmap, meta) {
      dropTex(costTex[kind]);
      costTex[kind] = bitmap ? uploadTexture(bitmap) : null;
      costMeta[kind] = meta;
    },
    clearCost: function (kind) { dropTex(costTex[kind]); costTex[kind] = null; costMeta[kind] = null; },
    clearScene: function () {
      dropTex(mapTex); mapTex = null; mapMeta = null;
      this.clearCost("global");
      this.clearCost("local");
      laser = null; depth = null; rgbCloud = null; globalPath = null; localPath = null; goal = null; drag = null;
      initialPose = null;
    },
    resize: resize,
  };
})();
