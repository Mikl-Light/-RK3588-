#!/usr/bin/env python3
"""
robot_gui.py — 麦轮小车可视化控制中心
HTTP 页面默认端口 8766，WebSocket 默认端口 8767（可通过参数 ws_host/http_port/ws_port 配置）。
浏览器访问: http://<RK3588_IP>:8766/
所以局域网里只需要保证 8766/8767 两个端口都放行。

功能：
  - 实时地图显示（OccupancyGrid）
  - 激光扫描点叠加
  - 机器人位姿箭头
  - nav2 规划路径显示
  - 虚拟摇杆遥控（/cmd_vel）
  - 地图点击设置导航目标（NavigateToPose action）
  - 实时数据面板（速度/位置/航向/激光有效点）
  - 模式切换按钮（建图导航一体化 + 待开发占位）
"""

import asyncio
import base64
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

try:
    from nav2_msgs.action import NavigateToPose
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False

try:
    # websockets >= 13 新 API
    from websockets.asyncio.server import serve as ws_serve
except ImportError:
    # websockets 12 及更早的旧 API
    from websockets.server import serve as ws_serve

# ============================================================
#  HTML 前端（内嵌，一个文件搞定部署）
# ============================================================
HTML_PAGE = r"""
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>麦轮小车控制中心</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { font-family: 'Segoe UI', 'PingFang SC', sans-serif; background: #0f0f1a; color: #e0e0e0; overflow: hidden; height: 100vh; }

/* 顶栏 */
#topbar { height: 48px; background: #16213e; display: flex; align-items: center; padding: 0 14px; gap: 6px; border-bottom: 1px solid #2a2a4a; flex-wrap: nowrap; }
#topbar .title { font-weight: bold; font-size: 15px; color: #4fc3f7; margin-right: 10px; white-space: nowrap; }
.mode-btn { padding: 5px 10px; border-radius: 5px; border: 1px solid #4fc3f7; background: transparent; color: #4fc3f7; cursor: pointer; font-size: 12px; white-space: nowrap; transition: all 0.15s; }
.mode-btn:hover { background: rgba(79,195,247,0.15); }
.mode-btn.active { background: #4fc3f7; color: #0f0f1a; font-weight: bold; }
.mode-btn.todo { opacity: 0.35; cursor: not-allowed; border-color: #555; color: #888; }
.mode-btn.todo:hover { background: transparent; }
#topbar .spacer { flex: 1; }
#conn-indicator { display: flex; align-items: center; gap: 5px; font-size: 12px; }
.conn-dot { width: 9px; height: 9px; border-radius: 50%; background: #f44336; transition: background 0.3s; }
.conn-dot.online { background: #4caf50; box-shadow: 0 0 6px #4caf50; }
#nav-status-text { font-size: 12px; color: #aaa; min-width: 60px; }

/* 主区域 */
#main { display: flex; height: calc(100vh - 48px); }
#map-container { flex: 1; position: relative; background: #0a0a14; overflow: hidden; }
#map-canvas { display: block; width: 100%; height: 100%; cursor: crosshair; }

/* 侧边数据面板 */
#sidebar { width: 220px; background: #16213e; padding: 12px; overflow-y: auto; border-left: 1px solid #2a2a4a; flex-shrink: 0; }
.panel-title { font-weight: bold; font-size: 13px; color: #4fc3f7; margin: 12px 0 6px; padding-bottom: 4px; border-bottom: 1px solid #2a2a4a; }
.panel-title:first-child { margin-top: 0; }
.stat-row { display: flex; justify-content: space-between; padding: 4px 0; font-size: 12px; }
.stat-label { color: #888; }
.stat-value { color: #4fc3f7; font-weight: bold; font-family: monospace; }
.stat-value.warn { color: #ff9800; }
.stat-value.error { color: #f44336; }
.help-text { font-size: 11px; color: #666; line-height: 1.7; }

/* 摇杆 */
#joystick-wrap { position: absolute; bottom: 18px; left: 18px; width: 130px; height: 130px; user-select: none; -webkit-user-select: none; touch-action: none; }
#joystick-canvas { width: 130px; height: 130px; }

/* 底部控制按钮 */
#bottom-controls { position: absolute; bottom: 18px; right: 18px; display: flex; gap: 8px; align-items: center; }
.ctrl-btn { width: 52px; height: 44px; font-size: 18px; border: 1px solid #4fc3f7; background: rgba(79,195,247,0.1); color: #4fc3f7; border-radius: 6px; cursor: pointer; display: flex; align-items: center; justify-content: center; transition: all 0.1s; user-select: none; -webkit-user-select: none; }
.ctrl-btn:active, .ctrl-btn.pressed { background: #4fc3f7; color: #0f0f1a; }
.ctrl-btn.stop { background: #f44336; border-color: #f44336; color: white; font-size: 13px; font-weight: bold; width: 60px; }
.ctrl-btn.stop:active { background: #d32f2f; }

/* 导航信息浮层 */
#nav-overlay { position: absolute; top: 12px; left: 12px; background: rgba(22,33,62,0.92); padding: 8px 12px; border-radius: 6px; font-size: 12px; display: none; border: 1px solid #2a2a4a; max-width: 280px; }
#nav-overlay .goal-text { color: #4caf50; }
#nav-overlay .cancel-btn { margin-left: 10px; background: #f44336; color: white; border: none; padding: 3px 8px; border-radius: 4px; cursor: pointer; font-size: 11px; }

/* 缩放提示 */
#zoom-hint { position: absolute; bottom: 8px; left: 50%; transform: translateX(-50%); font-size: 10px; color: #444; pointer-events: none; }
</style>
</head>
<body>

<div id="topbar">
  <span class="title">麦轮小车控制中心</span>
  <button class="mode-btn active" data-mode="mapping_nav">建图导航一体化</button>
  <button class="mode-btn" data-mode="line_follow" id="btn-line-follow">视觉巡线</button>
  <button class="mode-btn todo" data-mode="pure_nav">纯导航</button>
  <button class="mode-btn todo" data-mode="debug">调试模式</button>
  <div class="spacer"></div>
  <span id="nav-status-text">待机</span>
  <div id="conn-indicator"><span class="conn-dot" id="conn-dot"></span><span id="conn-text">未连接</span></div>
</div>

<div id="main">
  <div id="map-container">
    <canvas id="map-canvas"></canvas>
    <div id="joystick-wrap"><canvas id="joystick-canvas" width="130" height="130"></canvas></div>
    <div id="bottom-controls">
      <button class="ctrl-btn" id="btn-left" title="左移">◀</button>
      <button class="ctrl-btn stop" id="btn-stop">STOP</button>
      <button class="ctrl-btn" id="btn-right" title="右移">▶</button>
    </div>
    <div id="nav-overlay">
      <span class="goal-text" id="nav-goal-text"></span>
      <button class="cancel-btn" id="btn-cancel-nav">取消导航</button>
    </div>
    <div id="zoom-hint">滚轮缩放 · 右键拖动平移 · 左键点击设置导航目标</div>
  </div>
  <div id="sidebar">
    <div class="panel-title">运动状态</div>
    <div class="stat-row"><span class="stat-label">线速度 Vx</span><span class="stat-value" id="s-vx">0.00 m/s</span></div>
    <div class="stat-row"><span class="stat-label">线速度 Vy</span><span class="stat-value" id="s-vy">0.00 m/s</span></div>
    <div class="stat-row"><span class="stat-label">角速度 Wz</span><span class="stat-value" id="s-wz">0.00 rad/s</span></div>
    <div class="panel-title">位姿</div>
    <div class="stat-row"><span class="stat-label">位置 X</span><span class="stat-value" id="s-x">0.000 m</span></div>
    <div class="stat-row"><span class="stat-label">位置 Y</span><span class="stat-value" id="s-y">0.000 m</span></div>
    <div class="stat-row"><span class="stat-label">航向 Yaw</span><span class="stat-value" id="s-yaw">0.0°</span></div>
    <div class="panel-title">传感器</div>
    <div class="stat-row"><span class="stat-label">激光有效点</span><span class="stat-value" id="s-scan">0</span></div>
    <div class="stat-row"><span class="stat-label">地图尺寸</span><span class="stat-value" id="s-mapsize">--</span></div>
    <div class="panel-title">操作说明</div>
    <div class="help-text">
      • 左键点击地图 → 设置导航目标<br>
      • 滚轮 → 缩放地图<br>
      • 右键拖动 → 平移地图<br>
      • 左下角摇杆 → 手动遥控<br>
      • ◀ ▶ 按钮 → 麦轮横向移动<br>
      • STOP → 紧急停止<br>
      • 待开发模式仅为占位
    </div>
  </div>
</div>

<script>
// ============================================================
//  WebSocket 连接
// ============================================================
let ws = null;
let wsConnected = false;
function connectWS() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.hostname}:__WS_PORT__`);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => {
    wsConnected = true;
    pendingCmd = null;
    updateConnIndicator(true);
  };
  ws.onclose = () => {
    wsConnected = false;
    // 清除本地遥控状态，避免重连后把断线前的旧速度发给机器人
    pendingCmd = null;
    lateralDir = 0;
    document.querySelectorAll('.ctrl-btn').forEach(b => b.classList.remove('pressed'));
    if (jsActive) {
      jsActive = false;
      jsKnobX = jsCenter; jsKnobY = jsCenter;
      drawJoystick();
    }
    updateConnIndicator(false);
    setTimeout(connectWS, 2000);
  };
  ws.onerror = () => { ws.close(); };
  ws.onmessage = (evt) => {
    try { handleMessage(JSON.parse(evt.data)); } catch(e) {}
  };
}
function updateConnIndicator(online) {
  document.getElementById('conn-dot').className = 'conn-dot' + (online ? ' online' : '');
  document.getElementById('conn-text').textContent = online ? '已连接' : '重连中...';
}

// ============================================================
//  数据存储
// ============================================================
let mapData = null;       // {width,height,resolution,origin_x,origin_y,data:Uint8Array}
let mapDirty = true;
let scanData = null;      // {angle_min,angle_max,angle_increment,ranges:[]}
let poseData = null;      // {x,y,yaw,vx,vy,wz}
let pathData = null;      // {points:[[x,y],...]}
let navGoal = null;       // {x,y,yaw}

// ============================================================
//  地图 Canvas 渲染
// ============================================================
const mapCanvas = document.getElementById('map-canvas');
const mapCtx = mapCanvas.getContext('2d');
const offscreen = document.createElement('canvas');
const offCtx = offscreen.getContext('2d');
let viewScale = 20;       // 像素/米
let viewOffsetX = 0, viewOffsetY = 0;
let viewInitialized = false;  // 首次收到位姿后自动居中
let isPanning = false, panStartX = 0, panStartY = 0, panStartOX = 0, panStartOY = 0;

function resizeCanvas() {
  const rect = mapCanvas.parentElement.getBoundingClientRect();
  mapCanvas.width = rect.width;
  mapCanvas.height = rect.height;
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

function worldToCanvas(wx, wy) {
  if (!mapData) return [0, 0];
  const cx = (wx - mapData.origin_x) / mapData.resolution * viewScale + viewOffsetX + mapCanvas.width / 2;
  const cy = mapCanvas.height / 2 - (wy - mapData.origin_y) / mapData.resolution * viewScale + viewOffsetY;
  return [cx, cy];
}
function canvasToWorld(cx, cy) {
  if (!mapData) return [0, 0];
  const wx = (cx - mapCanvas.width / 2 - viewOffsetX) / viewScale * mapData.resolution + mapData.origin_x;
  const wy = (mapCanvas.height / 2 - cy + viewOffsetY) / viewScale * mapData.resolution + mapData.origin_y;
  return [wx, wy];
}

function renderMapToOffscreen() {
  if (!mapData) return;
  offscreen.width = mapData.width;
  offscreen.height = mapData.height;
  const imgData = offCtx.createImageData(mapData.width, mapData.height);
  const d = imgData.data;
  const raw = mapData.data;
  for (let i = 0; i < raw.length; i++) {
    const v = raw[i];
    const p = i * 4;
    if (v === -1 || v === 255) { // 未知
      d[p]=20; d[p+1]=20; d[p+2]=30; d[p+3]=255;
    } else if (v === 0) { // 空闲
      d[p]=30; d[p+1]=40; d[p+2]=60; d[p+3]=255;
    } else { // 占用 (1-100)
      const intensity = Math.min(255, v * 2 + 55);
      d[p]=intensity; d[p+1]=intensity*0.9; d[p+2]=intensity*0.7; d[p+3]=255;
    }
  }
  offCtx.putImageData(imgData, 0, 0);
  mapDirty = false;
}

function render() {
  mapCtx.fillStyle = '#0a0a14';
  mapCtx.fillRect(0, 0, mapCanvas.width, mapCanvas.height);
  if (mapData) {
    if (mapDirty) renderMapToOffscreen();
    // 绘制地图（以 origin 为中心）
    const [ox, oy] = worldToCanvas(mapData.origin_x, mapData.origin_y);
    const mapW = mapData.width / mapData.resolution * viewScale;
    const mapH = mapData.height / mapData.resolution * viewScale;
    mapCtx.drawImage(offscreen, ox, oy - mapH, mapW, mapH);
  }
  // 绘制规划路径
  if (pathData && pathData.points && pathData.points.length > 1) {
    mapCtx.strokeStyle = 'rgba(76,175,80,0.8)';
    mapCtx.lineWidth = 2;
    mapCtx.beginPath();
    for (let i = 0; i < pathData.points.length; i++) {
      const [cx, cy] = worldToCanvas(pathData.points[i][0], pathData.points[i][1]);
      if (i === 0) mapCtx.moveTo(cx, cy); else mapCtx.lineTo(cx, cy);
    }
    mapCtx.stroke();
  }
  // 绘制激光扫描
  if (scanData && poseData) {
    const [rx, ry] = worldToCanvas(poseData.x, poseData.y);
    mapCtx.fillStyle = 'rgba(79,195,247,0.6)';
    const inc = scanData.angle_increment;
    let angle = scanData.angle_min + poseData.yaw;
    for (let i = 0; i < scanData.ranges.length; i++) {
      const r = scanData.ranges[i];
      if (r > 0.05 && r < 15) {
        const ex = rx + Math.cos(angle) * r / mapData.resolution * viewScale;
        const ey = ry - Math.sin(angle) * r / mapData.resolution * viewScale;
        mapCtx.beginPath();
        mapCtx.arc(ex, ey, 1.5, 0, Math.PI * 2);
        mapCtx.fill();
      }
      angle += inc;
    }
  }
  // 绘制导航目标
  if (navGoal) {
    const [gx, gy] = worldToCanvas(navGoal.x, navGoal.y);
    mapCtx.strokeStyle = '#ff9800';
    mapCtx.lineWidth = 2;
    mapCtx.beginPath();
    mapCtx.arc(gx, gy, 10, 0, Math.PI * 2);
    mapCtx.stroke();
    mapCtx.beginPath();
    mapCtx.moveTo(gx, gy);
    mapCtx.lineTo(gx + Math.cos(navGoal.yaw) * 18, gy - Math.sin(navGoal.yaw) * 18);
    mapCtx.stroke();
  }
  // 绘制机器人位姿箭头
  if (poseData) {
    const [rx, ry] = worldToCanvas(poseData.x, poseData.y);
    mapCtx.save();
    mapCtx.translate(rx, ry);
    mapCtx.rotate(-poseData.yaw);
    // 箭头
    mapCtx.fillStyle = '#4caf50';
    mapCtx.beginPath();
    mapCtx.moveTo(14, 0);
    mapCtx.lineTo(-8, -7);
    mapCtx.lineTo(-4, 0);
    mapCtx.lineTo(-8, 7);
    mapCtx.closePath();
    mapCtx.fill();
    mapCtx.restore();
    // 外框
    mapCtx.strokeStyle = 'rgba(76,175,80,0.4)';
    mapCtx.lineWidth = 1;
    mapCtx.beginPath();
    mapCtx.arc(rx, ry, 16, 0, Math.PI * 2);
    mapCtx.stroke();
  }
  requestAnimationFrame(render);
}
render();

// 地图交互
mapCanvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const factor = e.deltaY > 0 ? 0.85 : 1.18;
  const [wx, wy] = canvasToWorld(e.offsetX, e.offsetY);
  viewScale *= factor;
  viewScale = Math.max(2, Math.min(200, viewScale));
  const [cx, cy] = worldToCanvas(wx, wy);
  viewOffsetX += e.offsetX - cx;
  viewOffsetY += e.offsetY - cy;
}, { passive: false });

mapCanvas.addEventListener('mousedown', (e) => {
  if (e.button === 2) {
    isPanning = true;
    panStartX = e.offsetX; panStartY = e.offsetY;
    panStartOX = viewOffsetX; panStartOY = viewOffsetY;
    e.preventDefault();
  }
});
mapCanvas.addEventListener('mousemove', (e) => {
  if (isPanning) {
    viewOffsetX = panStartOX + (e.offsetX - panStartX);
    viewOffsetY = panStartOY + (e.offsetY - panStartY);
  }
});
mapCanvas.addEventListener('mouseup', (e) => {
  if (e.button === 2) isPanning = false;
});
mapCanvas.addEventListener('contextmenu', (e) => e.preventDefault());

// 左键点击设置导航目标
mapCanvas.addEventListener('click', (e) => {
  if (isPanning) return;
  const [wx, wy] = canvasToWorld(e.offsetX, e.offsetY);
  const yaw = poseData ? poseData.yaw : 0;
  sendNavGoal(wx, wy, yaw);
});

// 触摸支持（单指拖动平移，点击选点）
let touchStartX = 0, touchStartY = 0, touchMoved = false;
mapCanvas.addEventListener('touchstart', (e) => {
  if (e.touches.length === 1) {
    touchStartX = e.touches[0].clientX;
    touchStartY = e.touches[0].clientY;
    panStartOX = viewOffsetX; panStartOY = viewOffsetY;
    touchMoved = false;
  }
}, { passive: true });
mapCanvas.addEventListener('touchmove', (e) => {
  if (e.touches.length === 1) {
    const dx = e.touches[0].clientX - touchStartX;
    const dy = e.touches[0].clientY - touchStartY;
    if (Math.abs(dx) > 5 || Math.abs(dy) > 5) touchMoved = true;
    viewOffsetX = panStartOX + dx;
    viewOffsetY = panStartOY + dy;
  }
}, { passive: true });
mapCanvas.addEventListener('touchend', (e) => {
  if (!touchMoved && e.changedTouches.length === 1) {
    const rect = mapCanvas.getBoundingClientRect();
    const cx = e.changedTouches[0].clientX - rect.left;
    const cy = e.changedTouches[0].clientY - rect.top;
    const [wx, wy] = canvasToWorld(cx, cy);
    const yaw = poseData ? poseData.yaw : 0;
    sendNavGoal(wx, wy, yaw);
  }
});

// ============================================================
//  虚拟摇杆
// ============================================================
const jsCanvas = document.getElementById('joystick-canvas');
const jsCtx = jsCanvas.getContext('2d');
const jsCenter = 65, jsBaseR = 55, jsKnobR = 22;
let jsActive = false, jsKnobX = 65, jsKnobY = 65;
let jsTouchId = null;

function drawJoystick() {
  jsCtx.clearRect(0, 0, 130, 130);
  // 底座
  jsCtx.fillStyle = 'rgba(79,195,247,0.1)';
  jsCtx.beginPath(); jsCtx.arc(jsCenter, jsCenter, jsBaseR, 0, Math.PI*2); jsCtx.fill();
  jsCtx.strokeStyle = 'rgba(79,195,247,0.4)';
  jsCtx.lineWidth = 2;
  jsCtx.stroke();
  // 十字线
  jsCtx.strokeStyle = 'rgba(79,195,247,0.15)';
  jsCtx.lineWidth = 1;
  jsCtx.beginPath();
  jsCtx.moveTo(jsCenter-jsBaseR, jsCenter); jsCtx.lineTo(jsCenter+jsBaseR, jsCenter);
  jsCtx.moveTo(jsCenter, jsCenter-jsBaseR); jsCtx.lineTo(jsCenter, jsCenter+jsBaseR);
  jsCtx.stroke();
  // 摇杆球
  jsCtx.fillStyle = jsActive ? '#4fc3f7' : 'rgba(79,195,247,0.6)';
  jsCtx.beginPath(); jsCtx.arc(jsKnobX, jsKnobY, jsKnobR, 0, Math.PI*2); jsCtx.fill();
}
drawJoystick();

function jsStart(clientX, clientY) {
  jsActive = true;
  jsMove(clientX, clientY);
}
function jsMove(clientX, clientY) {
  if (!jsActive) return;
  const rect = jsCanvas.getBoundingClientRect();
  let dx = (clientX - rect.left) * (130 / rect.width) - jsCenter;
  let dy = (clientY - rect.top) * (130 / rect.height) - jsCenter;
  const dist = Math.sqrt(dx*dx + dy*dy);
  if (dist > jsBaseR) { dx = dx/dist*jsBaseR; dy = dy/dist*jsBaseR; }
  jsKnobX = jsCenter + dx; jsKnobY = jsCenter + dy;
  drawJoystick();
  // 输出速度：向上=正Vx，向左=正Wz
  const vx = -dy / jsBaseR * 0.5;   // 最大 0.5 m/s
  const wz = -dx / jsBaseR * 1.5;   // 最大 1.5 rad/s
  sendCmdVel(vx, 0, wz);
}
function jsEnd() {
  jsActive = false;
  jsKnobX = jsCenter; jsKnobY = jsCenter;
  drawJoystick();
  sendCmdVel(0, 0, 0);
}

jsCanvas.addEventListener('mousedown', (e) => { jsStart(e.clientX, e.clientY); });
window.addEventListener('mousemove', (e) => { if (jsActive) jsMove(e.clientX, e.clientY); });
window.addEventListener('mouseup', () => { if (jsActive) jsEnd(); });
jsCanvas.addEventListener('touchstart', (e) => {
  if (jsTouchId === null && e.touches.length > 0) {
    jsTouchId = e.touches[0].identifier;
    jsStart(e.touches[0].clientX, e.touches[0].clientY);
  }
  e.preventDefault();
}, { passive: false });
jsCanvas.addEventListener('touchmove', (e) => {
  for (let t of e.touches) { if (t.identifier === jsTouchId) jsMove(t.clientX, t.clientY); }
  e.preventDefault();
}, { passive: false });
jsCanvas.addEventListener('touchend', (e) => {
  for (let t of e.changedTouches) { if (t.identifier === jsTouchId) { jsTouchId = null; jsEnd(); } }
});

// ============================================================
//  横向移动按钮 + STOP
// ============================================================
let lateralDir = 0; // -1 左, 0 停, 1 右
function setupHoldButton(id, dir) {
  const btn = document.getElementById(id);
  const start = (e) => { e.preventDefault(); lateralDir = dir; btn.classList.add('pressed'); };
  const end = () => { if (lateralDir === dir) { lateralDir = 0; btn.classList.remove('pressed'); } };
  btn.addEventListener('mousedown', start);
  btn.addEventListener('touchstart', start, { passive: false });
  window.addEventListener('mouseup', end);
  window.addEventListener('touchend', end);
  window.addEventListener('touchcancel', end);
}
setupHoldButton('btn-left', -1);
setupHoldButton('btn-right', 1);

document.getElementById('btn-stop').addEventListener('click', () => {
  sendCmdVel(0, 0, 0);
  if (navGoal) cancelNav();
});

// 横向移动速度发送（50ms 周期）
setInterval(() => {
  if (lateralDir !== 0 && !jsActive) {
    sendCmdVel(0, lateralDir * 0.3, 0);
  }
}, 50);

// ============================================================
//  指令发送（节流）
// ============================================================
let lastCmdSend = 0;
let pendingCmd = null;
function sendCmdVel(vx, vy, wz) {
  pendingCmd = { vx, vy, wz };
  const now = Date.now();
  if (now - lastCmdSend >= 50) {
    flushCmd();
  }
}
function flushCmd() {
  if (pendingCmd && wsConnected) {
    ws.send(JSON.stringify({ type: 'cmd_vel', ...pendingCmd }));
    lastCmdSend = Date.now();
  }
}
setInterval(flushCmd, 60);

function sendNavGoal(x, y, yaw) {
  if (!wsConnected) return;
  navGoal = { x, y, yaw };
  ws.send(JSON.stringify({ type: 'nav_goal', x, y, yaw }));
  document.getElementById('nav-overlay').style.display = 'block';
  document.getElementById('nav-goal-text').textContent =
    `导航目标: (${x.toFixed(2)}, ${y.toFixed(2)})`;
  document.getElementById('nav-status-text').textContent = '导航中';
}
function cancelNav() {
  // 即使 WebSocket 暂时断开也先清除本地导航状态，避免界面卡死
  navGoal = null;
  document.getElementById('nav-overlay').style.display = 'none';
  document.getElementById('nav-status-text').textContent = '已取消';
  setTimeout(() => { document.getElementById('nav-status-text').textContent = '待机'; }, 1500);
  if (wsConnected) {
    ws.send(JSON.stringify({ type: 'cancel_nav' }));
  }
}
document.getElementById('btn-cancel-nav').addEventListener('click', cancelNav);

// ============================================================
//  消息处理
// ============================================================
function handleMessage(msg) {
  switch (msg.type) {
    case 'map':
      const raw = Uint8Array.from(atob(msg.data), c => c.charCodeAt(0));
      mapData = { width: msg.width, height: msg.height, resolution: msg.resolution,
                   origin_x: msg.origin_x, origin_y: msg.origin_y, data: raw };
      mapDirty = true;
      document.getElementById('s-mapsize').textContent = `${msg.width}x${msg.height}`;
      break;
    case 'scan':
      scanData = msg;
      let valid = 0;
      for (let r of msg.ranges) if (r > 0.05) valid++;
      document.getElementById('s-scan').textContent = valid;
      break;
    case 'pose':
      poseData = msg;
      // 首次收到位姿时自动把画面居中到机器人位置
      if (!viewInitialized && mapData) {
        const [rx, ry] = worldToCanvas(msg.x, msg.y);
        viewOffsetX += mapCanvas.width / 2 - rx;
        viewOffsetY += mapCanvas.height / 2 - ry;
        viewInitialized = true;
      }
      document.getElementById('s-vx').textContent = msg.vx.toFixed(2) + ' m/s';
      document.getElementById('s-vy').textContent = msg.vy.toFixed(2) + ' m/s';
      document.getElementById('s-wz').textContent = msg.wz.toFixed(2) + ' rad/s';
      document.getElementById('s-x').textContent = msg.x.toFixed(3) + ' m';
      document.getElementById('s-y').textContent = msg.y.toFixed(3) + ' m';
      document.getElementById('s-yaw').textContent = (msg.yaw * 180 / Math.PI).toFixed(1) + '°';
      break;
    case 'path':
      pathData = msg;
      break;
    case 'nav_result':
      if (msg.success) {
        document.getElementById('nav-status-text').textContent = '导航完成';
        document.getElementById('nav-goal-text').textContent = '已到达目标';
      } else {
        document.getElementById('nav-status-text').textContent = '导航失败';
        document.getElementById('nav-goal-text').textContent = '失败: ' + (msg.reason || '未知');
      }
      navGoal = null;
      setTimeout(() => {
        document.getElementById('nav-overlay').style.display = 'none';
        document.getElementById('nav-status-text').textContent = '待机';
      }, 3000);
      break;
        case 'mode_result':
      if (msg.success) {
        const lineBtn = document.getElementById('btn-line-follow');
        const navBtn = document.querySelector('[data-mode="mapping_nav"]');
        if (msg.mode === 'line_follow') {
          lineBtn.classList.add('active');
          navBtn.classList.remove('active');
        } else {
          lineBtn.classList.remove('active');
          navBtn.classList.add('active');
        }
      } else {
        console.warn('模式切换失败:', msg.reason);
        alert('模式切换失败: ' + (msg.reason || 'unknown'));
        // 回退按钮状态
        document.getElementById('btn-line-follow').classList.remove('active');
        document.querySelector('[data-mode="mapping_nav"]').classList.add('active');
      }
      break;
case 'status':
      // 预留：底盘状态扩展
      break;
  }
}

// 模式按钮：建图导航 / 视觉巡线 切换
document.querySelectorAll('.mode-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    if (btn.classList.contains('todo')) return;
    const mode = btn.dataset.mode;
    // 视觉巡线模式切换
    if (mode === 'line_follow') {
      const enable = !btn.classList.contains('active');
      if (wsConnected) {
        ws.send(JSON.stringify({ type: 'switch_mode', enable: enable }));
      }
      if (enable) {
        document.querySelector('[data-mode="mapping_nav"]').classList.remove('active');
        btn.classList.add('active');
      } else {
        btn.classList.remove('active');
        document.querySelector('[data-mode="mapping_nav"]').classList.add('active');
      }
      return;
    }
    // 建图导航模式(默认)
    if (mode === 'mapping_nav') {
      const lineBtn = document.getElementById('btn-line-follow');
      if (wsConnected && lineBtn.classList.contains('active')) {
        ws.send(JSON.stringify({ type: 'switch_mode', enable: false }));
      }
      lineBtn.classList.remove('active');
    }
    document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
  });
});

// 启动
connectWS();
</script>
</body>
</html>
"""

# ============================================================
#  Python 后端节点
# ============================================================
class RobotGuiNode(Node):
    def __init__(self):
        super().__init__('robot_gui')

        # ---------------- 可配置参数（默认保持原有行为） ----------------
        self.declare_parameter('ws_host', '0.0.0.0')
        # HTTP 页面端口和 WebSocket 端口分离，避免旧 websockets 版本同时处理 HTTP/WS 握手导致局域网打不开
        self.declare_parameter('http_port', 8766)
        self.declare_parameter('ws_port', 8767)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom_laser')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('cmd_timeout', 0.5)

        # ---------------- 数据存储（线程间共享，用锁保护） ----------------
        self._lock = threading.Lock()
        self.latest_map = None
        self.latest_scan = None
        self.latest_pose = None
        self.latest_path = None
        self.ws_clients = set()
        self.loop = None  # asyncio event loop，由 main() 设置

        # ---- 遥控状态 ----
        self.last_cmd_time = 0.0
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # ---- 导航 action ----
        self.nav_action = None
        self.nav_goal_handle = None
        self._nav_token = 0  # 递增以丢弃旧导航目标的过期回调

        # ---- 巡线模式切换服务 ----
        self.switch_mode_client = self.create_client(SetBool, "/switch_mode")
        self.current_mode = "nav"  # nav / line_follow
        nav_action_name = self.get_parameter('nav_action_name').value
        if NAV2_AVAILABLE:
            try:
                self.nav_action = ActionClient(self, NavigateToPose, nav_action_name)
                self.get_logger().info(f'{nav_action_name} action client 已创建')
            except Exception as e:
                self.get_logger().warn(f'创建导航 action client 失败: {e}')

        # ---- QoS ----
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)
        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        # ---- 订阅 ----
        self.create_subscription(OccupancyGrid,
                                 self.get_parameter('map_topic').value,
                                 self._map_cb, map_qos)
        self.create_subscription(LaserScan,
                                 self.get_parameter('scan_topic').value,
                                 self._scan_cb, sensor_qos)
        self.create_subscription(Odometry,
                                 self.get_parameter('odom_topic').value,
                                 self._pose_cb, 1)
        self.create_subscription(Path,
                                 self.get_parameter('path_topic').value,
                                 self._path_cb, 1)

        # ---- 发布 ----
        self.cmd_vel_pub = self.create_publisher(Twist,
                                                 self.get_parameter('cmd_vel_topic').value,
                                                 1)

        # ---- 定时器：推送数据到 WebSocket ----
        self.create_timer(1.0, self._push_map)      # 地图 1Hz
        self.create_timer(0.1, self._push_scan)      # 激光 10Hz
        self.create_timer(0.1, self._push_pose)      # 位姿 10Hz
        self.create_timer(1.0, self._push_path)      # 路径 1Hz
        self.create_timer(0.1, self._check_cmd_timeout)  # 遥控超时检测

        http_port = self.get_parameter('http_port').value
        ws_port = self.get_parameter('ws_port').value
        host = self.get_parameter('ws_host').value
        self.get_logger().info(
            f'robot_gui 节点已启动: HTTP {host}:{http_port}, WebSocket {host}:{ws_port}')

    # ============================================================
    #  订阅回调
    # ============================================================
    def _map_cb(self, msg):
        try:
            raw = bytes(msg.data)
            encoded = base64.b64encode(raw).decode('ascii')
            with self._lock:
                self.latest_map = {
                    'width': msg.info.width,
                    'height': msg.info.height,
                    'resolution': msg.info.resolution,
                    'origin_x': msg.info.origin.position.x,
                    'origin_y': msg.info.origin.position.y,
                    'data': encoded,
                }
        except Exception as e:
            self.get_logger().warn(f'地图消息处理失败: {e}')

    def _scan_cb(self, msg):
        try:
            ranges = []
            for r in msg.ranges:
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    ranges.append(round(r, 3))
                else:
                    ranges.append(0.0)
            with self._lock:
                self.latest_scan = {
                    'angle_min': msg.angle_min,
                    'angle_max': msg.angle_max,
                    'angle_increment': msg.angle_increment,
                    'ranges': ranges,
                }
        except Exception as e:
            self.get_logger().warn(f'激光消息处理失败: {e}')

    def _pose_cb(self, msg):
        try:
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self._lock:
                self.latest_pose = {
                    'x': round(msg.pose.pose.position.x, 4),
                    'y': round(msg.pose.pose.position.y, 4),
                    'yaw': round(yaw, 4),
                    'vx': round(msg.twist.twist.linear.x, 3),
                    'vy': round(msg.twist.twist.linear.y, 3),
                    'wz': round(msg.twist.twist.angular.z, 3),
                }
        except Exception as e:
            self.get_logger().warn(f'位姿消息处理失败: {e}')

    def _path_cb(self, msg):
        try:
            points = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
                      for p in msg.poses]
            with self._lock:
                self.latest_path = {'points': points}
        except Exception as e:
            self.get_logger().warn(f'路径消息处理失败: {e}')

    # ============================================================
    #  数据推送（rclpy 定时器线程 → asyncio 事件循环）
    # ============================================================
    def _handle_switch_mode(self, msg):
        """切换巡线/导航模式, 调用 /switch_mode 服务"""
        try:
            enable = bool(msg.get("enable", False))
            if not self.switch_mode_client.wait_for_service(timeout_sec=1.0):
                self._broadcast({"type": "mode_result", "success": False,
                                  "reason": "switch_mode 服务不可用 (cmd_vel_arbiter 未启动?)"})
                return
            req = SetBool.Request()
            req.data = enable
            future = self.switch_mode_client.call_async(req)
            future.add_done_callback(lambda f: self._on_switch_mode_done(f, enable))
        except Exception as e:
            self.get_logger().warn(f"模式切换失败: {e}")
            self._broadcast({"type": "mode_result", "success": False, "reason": str(e)})

    def _on_switch_mode_done(self, future, enable):
        try:
            res = future.result()
            if res is not None and res.success:
                self.current_mode = "line_follow" if enable else "nav"
                self._broadcast({"type": "mode_result", "success": True,
                                  "mode": self.current_mode})
            else:
                self._broadcast({"type": "mode_result", "success": False,
                                  "reason": "服务返回失败"})
        except Exception as e:
            self._broadcast({"type": "mode_result", "success": False, "reason": str(e)})

    def _broadcast(self, msg_dict):
        if self.loop is None:
            return
        with self._lock:
            clients = list(self.ws_clients)
        if not clients:
            return
        text = json.dumps(msg_dict)
        for ws in clients:
            try:
                asyncio.run_coroutine_threadsafe(_ws_send(ws, text), self.loop)
            except Exception:
                pass

    def _push_map(self):
        with self._lock:
            data = self.latest_map
        if data:
            self._broadcast({'type': 'map', **data})

    def _push_scan(self):
        with self._lock:
            data = self.latest_scan
        if data:
            self._broadcast({'type': 'scan', **data})

    def _push_pose(self):
        with self._lock:
            data = self.latest_pose
        if data:
            self._broadcast({'type': 'pose', **data})

    def _push_path(self):
        with self._lock:
            data = self.latest_path
        if data:
            self._broadcast({'type': 'path', **data})

    # ============================================================
    #  遥控指令处理
    # ============================================================
    def handle_client_message(self, raw_text):
        try:
            msg = json.loads(raw_text)
        except Exception:
            return
        mtype = msg.get('type')
        if mtype == 'cmd_vel':
            self._handle_cmd_vel(msg)
        elif mtype == 'nav_goal':
            self._handle_nav_goal(msg)
        elif mtype == 'cancel_nav':
            self._handle_cancel_nav()

    def _handle_cmd_vel(self, msg):
        try:
            twist = Twist()
            twist.linear.x = float(msg.get('vx', 0.0))
            twist.linear.y = float(msg.get('vy', 0.0))
            twist.angular.z = float(msg.get('wz', 0.0))
            self.cmd_vel_pub.publish(twist)
            self.last_cmd_time = time.time()
        except Exception as e:
            self.get_logger().warn(f'遥控指令处理失败: {e}')

    def _check_cmd_timeout(self):
        # 如果 500ms 没收到遥控指令，自动停止（防止断连后小车乱跑）
        if self.last_cmd_time > 0 and (time.time() - self.last_cmd_time) > self.cmd_timeout:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.last_cmd_time = 0

    # ============================================================
    #  导航 action 处理
    # ============================================================
    def _handle_nav_goal(self, msg):
        if self.nav_action is None:
            self._broadcast({'type': 'nav_result', 'success': False,
                              'reason': 'nav2 action 不可用'})
            return
        try:
            x = float(msg.get('x', 0.0))
            y = float(msg.get('y', 0.0))
            yaw = float(msg.get('yaw', 0.0))
            self.get_logger().info(f'导航目标: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
            # 新目标会使旧目标回调失效；wait_for_server 阻塞调用放到后台线程，
            # 避免卡住 asyncio/WebSocket。
            self._nav_token += 1
            token = self._nav_token
            threading.Thread(target=self._send_nav_goal_worker,
                             args=(x, y, yaw, token),
                             daemon=True).start()
        except Exception as e:
            self.get_logger().error(f'导航目标发送失败: {e}')
            self._broadcast({'type': 'nav_result', 'success': False, 'reason': str(e)})

    def _send_nav_goal_worker(self, x, y, yaw, token):
        try:
            # 等待 action server 可用
            if token != self._nav_token:
                return
            if not self.nav_action.wait_for_server(timeout_sec=3.0):
                if token == self._nav_token:
                    self._broadcast({'type': 'nav_result', 'success': False,
                                      'reason': 'nav2 action server 未响应'})
                return
            # 等待期间用户可能取消了导航或发送了新目标
            if token != self._nav_token:
                return

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            # yaw → quaternion
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw

            future = self.nav_action.send_goal_async(goal_msg,
                                                     feedback_callback=self._nav_feedback)
            future.add_done_callback(
                lambda fut: self._nav_goal_accepted(fut, token))
        except Exception as e:
            self.get_logger().error(f'导航目标发送失败: {e}')
            if token == self._nav_token:
                self._broadcast({'type': 'nav_result', 'success': False,
                                  'reason': str(e)})

    def _nav_feedback(self, feedback_msg):
        # 导航进度反馈（可选，暂不推送到前端）
        pass

    def _nav_goal_accepted(self, future, token):
        if token != self._nav_token:
            return
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._broadcast({'type': 'nav_result', 'success': False,
                                  'reason': '目标被拒绝'})
                return
            self.nav_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda fut: self._nav_result(fut, token))
        except Exception as e:
            if token == self._nav_token:
                self._broadcast({'type': 'nav_result', 'success': False,
                                  'reason': str(e)})

    def _nav_result(self, future, token):
        if token != self._nav_token:
            return
        try:
            result = future.result()
            # rclpy 的 get_result_async() future 返回 GetResult.Response，
            # 其中 .status 是 action_msgs/GoalStatus（STATUS_SUCCEEDED == 4）。
            # 有些 nav2 版本把 action 具体结果放在 .result 中，因此做兼容回退。
            if result is None:
                status = None
            else:
                status = getattr(result, 'status', None)
                if status is None:
                    status = getattr(getattr(result, 'result', None), 'status', None)
            # 用户主动取消时前端已经清掉导航状态，这里不再广播“失败”造成误报
            if status == GoalStatus.STATUS_CANCELED:
                return
            success = (status == GoalStatus.STATUS_SUCCEEDED)
            reason = '' if success else f'status={status}'
            self._broadcast({'type': 'nav_result', 'success': success, 'reason': reason})
        except Exception as e:
            self._broadcast({'type': 'nav_result', 'success': False, 'reason': str(e)})
        finally:
            self.nav_goal_handle = None

    def _handle_cancel_nav(self):
        # 使旧目标的后续回调全部失效；前端已经自行更新界面
        self._nav_token += 1
        if self.nav_goal_handle is not None:
            try:
                self.nav_goal_handle.cancel_goal_async()
                self.get_logger().info('导航已取消')
            except Exception as e:
                self.get_logger().warn(f'取消导航失败: {e}')
            self.nav_goal_handle = None


# ============================================================
#  HTTP + WebSocket 服务器
#  HTTP 用标准库 ThreadingHTTPServer；WebSocket 单独端口。
#  这样局域网浏览器一定能打开页面，不依赖 websockets 的 HTTP 握手。
# ============================================================
_NODE = None


class RobotGuiHTTPHandler(BaseHTTPRequestHandler):
    """只负责返回内嵌 HTML 页面。"""
    page = b''

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ('/', '/index.html'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(self.page)))
            self.end_headers()
            self.wfile.write(self.page)
        else:
            self.send_response(404)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.end_headers()
            self.wfile.write(b'Not Found')

    def log_message(self, fmt, *args):
        pass


async def _ws_send(websocket, text):
    """向单个客户端发送消息；失败时自动清理该连接。"""
    try:
        await websocket.send(text)
    except Exception:
        node = _NODE
        if node is not None:
            try:
                with node._lock:
                    node.ws_clients.discard(websocket)
            except Exception:
                pass


async def _ws_handler(websocket, path=None):
    """处理 WebSocket 连接（兼容 websockets 12 旧 API 和 13+ 新 API）。"""
    node = _NODE
    if node is None:
        return
    with node._lock:
        node.ws_clients.add(websocket)
    try:
        async for message in websocket:
            node.handle_client_message(message)
    except Exception:
        pass
    finally:
        with node._lock:
            node.ws_clients.discard(websocket)


def main():
    global _NODE
    rclpy.init(args=None)
    node = RobotGuiNode()
    _NODE = node

    # 用多线程执行器驱动 rclpy 回调（订阅/定时/action），避免阻塞 asyncio
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    host = str(node.get_parameter('ws_host').value)
    http_port = int(node.get_parameter('http_port').value)
    ws_port = int(node.get_parameter('ws_port').value)

    # 1. 标准 HTTP 服务：返回 HTML 页面
    RobotGuiHTTPHandler.page = HTML_PAGE.replace('__WS_PORT__', str(ws_port)).encode('utf-8')
    httpd = ThreadingHTTPServer((host, http_port), RobotGuiHTTPHandler)
    http_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    http_thread.start()
    node.get_logger().info(
        f'robot_gui HTTP 页面: http://<RK3588_IP>:{http_port}/  (WebSocket 端口 {ws_port})')

    # 2. WebSocket 服务：只负责实时数据推送/控制
    async def main_async():
        node.loop = asyncio.get_running_loop()
        async with ws_serve(_ws_handler, host, ws_port,
                            max_size=20 * 1024 * 1024):
            node.get_logger().info(f'WebSocket 服务已启动: ws://{host}:{ws_port}')
            while rclpy.ok():
                await asyncio.sleep(0.1)

    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        pass
    finally:
        try:
            httpd.shutdown()
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        spin_thread.join(timeout=3.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
