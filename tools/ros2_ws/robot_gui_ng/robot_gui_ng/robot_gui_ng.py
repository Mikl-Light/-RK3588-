#!/usr/bin/env python3
"""
robot_gui_ng.py — 新一代麦轮小车 Web 可视化控制中心

与 odom_compare_web 一样使用 Python 标准库 ThreadingHTTPServer 提供网页，
不依赖 websockets 的 HTTP 握手，因此局域网浏览器可直接打开。

默认地址: http://0.0.0.0:8766/
"""

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
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Quaternion
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from nav2_msgs.action import NavigateToPose
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False

try:
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    TF2_AVAILABLE = True
except Exception:
    TF2_AVAILABLE = False

# ============================================================
#  前端页面
# ============================================================
HTML_PAGE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>麦轮小车控制中心 [预览版]</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { font-family: 'Segoe UI', 'PingFang SC', sans-serif; background: #0f0f1a; color: #e0e0e0; overflow: hidden; height: 100vh; }
#topbar { height: 48px; background: #16213e; display: flex; align-items: center; padding: 0 14px; gap: 6px; border-bottom: 1px solid #2a2a4a; flex-wrap: nowrap; }
#topbar .title { font-weight: bold; font-size: 15px; color: #4fc3f7; margin-right: 10px; white-space: nowrap; }
.preview-badge { background: #ff9800; color: #0f0f1a; font-size: 10px; padding: 2px 6px; border-radius: 3px; font-weight: bold; margin-left: -4px; }
.mode-btn { padding: 5px 10px; border-radius: 5px; border: 1px solid #4fc3f7; background: transparent; color: #4fc3f7; cursor: pointer; font-size: 12px; white-space: nowrap; transition: all 0.15s; }
.mode-btn:hover { background: rgba(79,195,247,0.15); }
.mode-btn.active { background: #4fc3f7; color: #0f0f1a; font-weight: bold; }
.mode-btn.todo { opacity: 0.35; cursor: not-allowed; border-color: #555; color: #888; }
.mode-btn.todo:hover { background: transparent; }
#topbar .spacer { flex: 1; }
#conn-indicator { display: flex; align-items: center; gap: 5px; font-size: 12px; }
.conn-dot { width: 9px; height: 9px; border-radius: 50%; background: #4caf50; box-shadow: 0 0 6px #4caf50; }
#nav-status-text { font-size: 12px; color: #aaa; min-width: 60px; }
#main { display: flex; height: calc(100vh - 48px); }
#map-container { flex: 1; position: relative; background: #0a0a14; overflow: hidden; }
#map-canvas { display: block; width: 100%; height: 100%; cursor: crosshair; }
#sidebar { width: 220px; background: #16213e; padding: 12px; overflow-y: auto; border-left: 1px solid #2a2a4a; flex-shrink: 0; }
.panel-title { font-weight: bold; font-size: 13px; color: #4fc3f7; margin: 12px 0 6px; padding-bottom: 4px; border-bottom: 1px solid #2a2a4a; }
.panel-title:first-child { margin-top: 0; }
.stat-row { display: flex; justify-content: space-between; padding: 4px 0; font-size: 12px; }
.stat-label { color: #888; }
.stat-value { color: #4fc3f7; font-weight: bold; font-family: monospace; }
.help-text { font-size: 11px; color: #666; line-height: 1.7; }
#joystick-wrap { position: absolute; bottom: 18px; left: 18px; width: 130px; height: 130px; user-select: none; -webkit-user-select: none; touch-action: none; }
#joystick-canvas { width: 130px; height: 130px; }
#bottom-controls { position: absolute; bottom: 18px; right: 18px; display: flex; gap: 8px; align-items: center; }
.ctrl-btn { width: 52px; height: 44px; font-size: 18px; border: 1px solid #4fc3f7; background: rgba(79,195,247,0.1); color: #4fc3f7; border-radius: 6px; cursor: pointer; display: flex; align-items: center; justify-content: center; transition: all 0.1s; user-select: none; -webkit-user-select: none; }
.ctrl-btn:active, .ctrl-btn.pressed { background: #4fc3f7; color: #0f0f1a; }
.ctrl-btn.stop { background: #f44336; border-color: #f44336; color: white; font-size: 13px; font-weight: bold; width: 60px; }
.ctrl-btn.stop:active { background: #d32f2f; }
#nav-overlay { position: absolute; top: 12px; left: 12px; background: rgba(22,33,62,0.92); padding: 8px 12px; border-radius: 6px; font-size: 12px; display: none; border: 1px solid #2a2a4a; max-width: 280px; }
#nav-overlay .goal-text { color: #4caf50; }
#nav-overlay .cancel-btn { margin-left: 10px; background: #f44336; color: white; border: none; padding: 3px 8px; border-radius: 4px; cursor: pointer; font-size: 11px; }
#zoom-hint { position: absolute; bottom: 8px; left: 50%; transform: translateX(-50%); font-size: 10px; color: #444; pointer-events: none; }
.path-tool-btn { padding: 5px 10px; border-radius: 5px; border: 1px solid #ff9800; background: transparent; color: #ff9800; cursor: pointer; font-size: 12px; white-space: nowrap; transition: all 0.15s; margin-left: 8px; }
.path-tool-btn:hover { background: rgba(255,152,0,0.15); }
.path-tool-btn.active { background: #ff9800; color: #0f0f1a; font-weight: bold; }
.waypoint-tool-btn { padding: 5px 10px; border-radius: 5px; border: 1px solid #4fc3f7; background: transparent; color: #4fc3f7; cursor: pointer; font-size: 12px; white-space: nowrap; transition: all 0.15s; margin-left: 6px; }
.waypoint-tool-btn:hover { background: rgba(79,195,247,0.15); }
.waypoint-tool-btn.active { background: #4fc3f7; color: #0f0f1a; font-weight: bold; }
#path-actions { position: absolute; top: 12px; left: 50%; transform: translateX(-50%); background: rgba(22,33,62,0.95); padding: 8px 14px; border-radius: 6px; font-size: 12px; display: none; border: 1px solid #ff9800; z-index: 10; }
#path-actions .path-info { color: #ff9800; margin-right: 12px; }
#path-actions button { margin-left: 6px; padding: 4px 10px; border-radius: 4px; border: none; cursor: pointer; font-size: 11px; }
#path-actions .btn-exec { background: #4caf50; color: white; }
#path-actions .btn-clear { background: #666; color: white; }
#path-actions .btn-cancel { background: #f44336; color: white; }
#mock-toast { position: absolute; top: 12px; right: 12px; background: rgba(255,152,0,0.15); border: 1px solid #ff9800; color: #ff9800; padding: 6px 12px; border-radius: 6px; font-size: 11px; }
</style>
</head>
<body>

<div id="topbar">
  <span class="title">麦轮小车控制中心</span>
  <span class="preview-badge">预览版</span>
  <button class="mode-btn active" data-mode="mapping_nav">建图导航一体化</button>
  <button class="mode-btn todo" data-mode="pure_mapping">纯建图</button>
  <button class="mode-btn todo" data-mode="pure_nav">纯导航</button>
  <button class="mode-btn todo" data-mode="debug">调试模式</button>
  <button class="path-tool-btn" id="btn-draw-path">✏️ 画路径</button>
  <button class="waypoint-tool-btn" id="btn-waypoint">📍 航点导航</button>
  <div class="spacer"></div>
  <span id="nav-status-text">待机</span>
  <div id="conn-indicator"><span class="conn-dot"></span><span id="conn-text">模拟数据</span></div>
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
    <div id="path-actions">
      <span class="path-info" id="path-info">已添加 0 个路径点</span>
      <button class="btn-exec" id="btn-exec-path">▶ 执行路径</button>
      <button class="btn-clear" id="btn-clear-path">清除</button>
      <button class="btn-cancel" id="btn-cancel-path" style="display:none;">取消执行</button>
    </div>
    <div id="waypoint-actions" style="display:none; position:absolute; top:12px; left:50%; transform:translateX(-50%); background:rgba(22,33,62,0.95); padding:8px 14px; border-radius:6px; font-size:12px; border:1px solid #4fc3f7; z-index:10;">
      <span class="path-info" id="waypoint-info" style="color:#4fc3f7;">已添加 0 个航点</span>
      <button class="btn-exec" id="btn-exec-waypoint" style="background:#2196f3;">▶ 开始导航</button>
      <button class="btn-clear" id="btn-clear-waypoint">清除</button>
      <button class="btn-cancel" id="btn-cancel-waypoint" style="display:none;">取消导航</button>
    </div>
    <div id="mock-toast">🖥️ 预览模式 — 数据为模拟，非实车</div>
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
//  数据存储
// ============================================================
let mapData = null;
let mapDirty = true;
let scanData = null;
let poseData = null;
let pathData = null;
let navGoal = null;
let pathDrawMode = false;
let customPath = [];
let pathExecuting = false;
let pathExecIndex = 0;
let isDrawingPath = false;
let waypointMode = false;
let waypoints = [];
let waypointExecuting = false;
let waypointExecIndex = 0;
let mockTime = 0;

let routeStatus = '';
async function fetchJSON(url, options) {
  const r = await fetch(url, options);
  if (!r.ok) throw new Error('HTTP ' + r.status);
  return r.json();
}
function sendCmdVel(vx, vy, wz) {
  fetch('/api/cmd_vel', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({vx, vy, wz}),
  }).catch(() => {});
}
function sendNavGoal(x, y, yaw) {
  navGoal = {x, y, yaw};
  fetch('/api/nav_goal', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({x, y, yaw}),
  }).catch(() => {});
  document.getElementById('nav-overlay').style.display = 'block';
  document.getElementById('nav-goal-text').textContent =
    '导航目标: (' + x.toFixed(2) + ', ' + y.toFixed(2) + ')';
  document.getElementById('nav-status-text').textContent = '导航中';
}
function cancelNav() {
  navGoal = null;
  document.getElementById('nav-overlay').style.display = 'none';
  document.getElementById('nav-status-text').textContent = '已取消';
  setTimeout(() => { document.getElementById('nav-status-text').textContent = '待机'; }, 1000);
  fetch('/api/cancel_nav', {method: 'POST'}).catch(() => {});
}
function cancelRoute() {
  fetch('/api/cancel_route', {method: 'POST'}).catch(() => {});
}
function applyRouteStatus(st) {
  routeStatus = st || '';
  if (!routeStatus) return;
  document.getElementById('nav-status-text').textContent = routeStatus;
  const up = routeStatus.toUpperCase();
  if (up.includes('SUCCEEDED') || up.includes('ALL DONE') ||
      up.includes('CANCELED') || up.includes('REJECTED') || up.includes('STOPPED')) {
    pathExecuting = false;
    waypointExecuting = false;
    const pe = document.getElementById('btn-exec-path');
    const pc = document.getElementById('btn-clear-path');
    const px = document.getElementById('btn-cancel-path');
    if (pe) pe.style.display = 'inline';
    if (pc) pc.style.display = 'inline';
    if (px) px.style.display = 'none';
    const we = document.getElementById('btn-exec-waypoint');
    const wc = document.getElementById('btn-clear-waypoint');
    const wx = document.getElementById('btn-cancel-waypoint');
    if (we) we.style.display = 'inline';
    if (wc) wc.style.display = 'inline';
    if (wx) wx.style.display = 'none';
  }
}
async function pollOnce() {
  try {
    const map = await fetchJSON('/api/map');
    if (map && map.data) {
      const raw = Uint8Array.from(atob(map.data), c => c.charCodeAt(0));
      mapData = {
        width: map.width, height: map.height, resolution: map.resolution,
        origin_x: map.origin_x, origin_y: map.origin_y, data: raw,
      };
      mapDirty = true;
      document.getElementById('s-mapsize').textContent = map.width + 'x' + map.height;
    }
    const data = await fetchJSON('/api/data');
    if (data.pose) {
      poseData = data.pose;
      if (!viewInitialized && mapData) {
        const [rx, ry] = worldToCanvas(poseData.x, poseData.y);
        viewOffsetX += mapCanvas.width / 2 - rx;
        viewOffsetY += mapCanvas.height / 2 - ry;
        viewInitialized = true;
      }
      document.getElementById('s-vx').textContent = poseData.vx.toFixed(2) + ' m/s';
      document.getElementById('s-vy').textContent = poseData.vy.toFixed(2) + ' m/s';
      document.getElementById('s-wz').textContent = poseData.wz.toFixed(2) + ' rad/s';
      document.getElementById('s-x').textContent = poseData.x.toFixed(3) + ' m';
      document.getElementById('s-y').textContent = poseData.y.toFixed(3) + ' m';
      document.getElementById('s-yaw').textContent = (poseData.yaw * 180 / Math.PI).toFixed(1) + '°';
    }
    if (data.scan) {
      scanData = data.scan;
      let valid = 0;
      for (const r of scanData.ranges) if (r > 0.05) valid++;
      document.getElementById('s-scan').textContent = valid;
    }
    if (data.path) pathData = data.path;
    if (data.nav_result) {
      if (data.nav_result.success) {
        document.getElementById('nav-status-text').textContent = '导航完成';
        document.getElementById('nav-goal-text').textContent = '已到达目标';
        navGoal = null;
      } else {
        document.getElementById('nav-status-text').textContent = '导航失败';
        document.getElementById('nav-goal-text').textContent = '失败: ' + (data.nav_result.reason || '未知');
      }
    }
    if (data.route_status) applyRouteStatus(data.route_status);
    document.getElementById('conn-text').textContent = '已连接';
  } catch (e) {
    document.getElementById('conn-text').textContent = '连接中...';
  }
}
function startPolling() {
  setInterval(pollOnce, 100);
  pollOnce();
}


// ============================================================
//  地图 Canvas 渲染
// ============================================================
const mapCanvas = document.getElementById('map-canvas');
const mapCtx = mapCanvas.getContext('2d');
const offscreen = document.createElement('canvas');
const offCtx = offscreen.getContext('2d');
let viewScale = 30;
let viewOffsetX = 0, viewOffsetY = 0;
let viewInitialized = false;
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
  const cx = (wx - mapData.origin_x) * viewScale + viewOffsetX + mapCanvas.width / 2;
  const cy = mapCanvas.height / 2 - (wy - mapData.origin_y) * viewScale + viewOffsetY;
  return [cx, cy];
}
function canvasToWorld(cx, cy) {
  if (!mapData) return [0, 0];
  const wx = (cx - mapCanvas.width / 2 - viewOffsetX) / viewScale + mapData.origin_x;
  const wy = (mapCanvas.height / 2 - cy + viewOffsetY) / viewScale + mapData.origin_y;
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
    if (v === -1 || v === 255) { d[p]=48; d[p+1]=48; d[p+2]=48; d[p+3]=255; }
    else if (v === 0) { d[p]=205; d[p+1]=205; d[p+2]=205; d[p+3]=255; }
    else { d[p]=0; d[p+1]=0; d[p+2]=0; d[p+3]=255; }
  }
  offCtx.putImageData(imgData, 0, 0);
  mapDirty = false;
}

function render() {
  mapCtx.fillStyle = '#1a1a1a';
  mapCtx.fillRect(0, 0, mapCanvas.width, mapCanvas.height);
  if (mapData) {
    if (mapDirty) renderMapToOffscreen();
    // OccupancyGrid 的 data[0] 是地图左下角 (origin)，而 canvas 图像 row 0 在顶部。
    // 直接 drawImage 会把地图上下翻转；这里 translate 到 origin 后 scale(1,-1)，
    // 让地图和世界坐标/点云保持同向。
    const [ox, oy] = worldToCanvas(mapData.origin_x, mapData.origin_y);
    const mapW = mapData.width * mapData.resolution * viewScale;
    const mapH = mapData.height * mapData.resolution * viewScale;
    mapCtx.save();
    mapCtx.translate(ox, oy);
    mapCtx.scale(1, -1);
    mapCtx.drawImage(offscreen, 0, 0, mapW, mapH);
    mapCtx.restore();
  }
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
  if (scanData && poseData) {
    // 静态 TF: base_link -> laser = x:0.13, y:0, yaw:pi
    // 点云必须先转到 base_link，再从 base_link 转到 map，否则会整体偏 0.13m 并反 180°。
    const LASER_X = 0.13, LASER_Y = 0.0, LASER_YAW = Math.PI;
    mapCtx.fillStyle = 'rgba(79,195,247,0.45)';
    const inc = scanData.angle_increment;
    for (let i = 0; i < scanData.ranges.length; i++) {
      const r = scanData.ranges[i];
      if (r > 0.05 && r < 15) {
        const laserAngle = scanData.angle_min + i * inc;
        const lx = r * Math.cos(laserAngle);
        const ly = r * Math.sin(laserAngle);
        // laser -> base_link
        const bx = Math.cos(LASER_YAW) * lx - Math.sin(LASER_YAW) * ly + LASER_X;
        const by = Math.sin(LASER_YAW) * lx + Math.cos(LASER_YAW) * ly + LASER_Y;
        // base_link -> map
        const mx = Math.cos(poseData.yaw) * bx - Math.sin(poseData.yaw) * by + poseData.x;
        const my = Math.sin(poseData.yaw) * bx + Math.cos(poseData.yaw) * by + poseData.y;
        const [ex, ey] = worldToCanvas(mx, my);
        mapCtx.beginPath();
        mapCtx.arc(ex, ey, 1.2, 0, Math.PI * 2);
        mapCtx.fill();
      }
    }
  }
  // 绘制用户画的路径（连续曲线）
  if (customPath.length > 1) {
    // 路径线
    mapCtx.strokeStyle = pathExecuting ? 'rgba(255,87,34,0.95)' : 'rgba(255,152,0,0.85)';
    mapCtx.lineWidth = 3;
    mapCtx.lineCap = 'round';
    mapCtx.lineJoin = 'round';
    mapCtx.beginPath();
    for (let i = 0; i < customPath.length; i++) {
      const [cx, cy] = worldToCanvas(customPath[i].x, customPath[i].y);
      if (i === 0) mapCtx.moveTo(cx, cy); else mapCtx.lineTo(cx, cy);
    }
    mapCtx.stroke();
    // 起点标记（绿色）
    const [sx, sy] = worldToCanvas(customPath[0].x, customPath[0].y);
    mapCtx.fillStyle = '#4caf50';
    mapCtx.beginPath();
    mapCtx.arc(sx, sy, 7, 0, Math.PI * 2);
    mapCtx.fill();
    mapCtx.fillStyle = '#fff';
    mapCtx.font = 'bold 9px sans-serif';
    mapCtx.textAlign = 'center';
    mapCtx.textBaseline = 'middle';
    mapCtx.fillText('起', sx, sy);
    // 终点标记（红色）
    const [ex, ey] = worldToCanvas(customPath[customPath.length-1].x, customPath[customPath.length-1].y);
    mapCtx.fillStyle = '#f44336';
    mapCtx.beginPath();
    mapCtx.arc(ex, ey, 7, 0, Math.PI * 2);
    mapCtx.fill();
    mapCtx.fillStyle = '#fff';
    mapCtx.fillText('终', ex, ey);
    // 执行中当前位置高亮
    if (pathExecuting && pathExecIndex < customPath.length) {
      const [cx, cy] = worldToCanvas(customPath[pathExecIndex].x, customPath[pathExecIndex].y);
      mapCtx.strokeStyle = '#ffeb3b';
      mapCtx.lineWidth = 2;
      mapCtx.beginPath();
      mapCtx.arc(cx, cy, 10, 0, Math.PI * 2);
      mapCtx.stroke();
    }
  }
  // 绘制航点（方案二）
  if (waypoints.length > 0 && poseData) {
    // 航点之间的规划路径
    mapCtx.strokeStyle = waypointExecuting ? 'rgba(76,175,80,0.9)' : 'rgba(76,175,80,0.5)';
    mapCtx.lineWidth = 2;
    mapCtx.setLineDash(waypointExecuting ? [] : [8, 4]);
    mapCtx.beginPath();
    const [rx0, ry0] = worldToCanvas(poseData.x, poseData.y);
    mapCtx.moveTo(rx0, ry0);
    for (let i = 0; i < waypoints.length; i++) {
      const [cx, cy] = worldToCanvas(waypoints[i].x, waypoints[i].y);
      mapCtx.lineTo(cx, cy);
    }
    mapCtx.stroke();
    mapCtx.setLineDash([]);
    // 航点标记
    for (let i = 0; i < waypoints.length; i++) {
      const [cx, cy] = worldToCanvas(waypoints[i].x, waypoints[i].y);
      const isCurrent = waypointExecuting && i === waypointExecIndex;
      const isPassed = waypointExecuting && i < waypointExecIndex;
      mapCtx.fillStyle = isCurrent ? '#2196f3' : (isPassed ? '#4caf50' : '#4fc3f7');
      mapCtx.beginPath();
      mapCtx.arc(cx, cy, isCurrent ? 8 : 6, 0, Math.PI * 2);
      mapCtx.fill();
      mapCtx.fillStyle = '#fff';
      mapCtx.font = 'bold 10px monospace';
      mapCtx.textAlign = 'center';
      mapCtx.textBaseline = 'middle';
      mapCtx.fillText(String(i + 1), cx, cy);
      if (isPassed) {
        mapCtx.strokeStyle = '#fff';
        mapCtx.lineWidth = 2;
        mapCtx.beginPath();
        mapCtx.moveTo(cx - 3, cy);
        mapCtx.lineTo(cx - 1, cy + 2);
        mapCtx.lineTo(cx + 3, cy - 2);
        mapCtx.stroke();
      }
    }
  }
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
  if (poseData) {
    const [rx, ry] = worldToCanvas(poseData.x, poseData.y);
    mapCtx.save();
    mapCtx.translate(rx, ry);
    mapCtx.rotate(-poseData.yaw);
    mapCtx.fillStyle = '#4caf50';
    mapCtx.beginPath();
    mapCtx.moveTo(14, 0);
    mapCtx.lineTo(-8, -7);
    mapCtx.lineTo(-4, 0);
    mapCtx.lineTo(-8, 7);
    mapCtx.closePath();
    mapCtx.fill();
    mapCtx.restore();
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

function finishPathDrawing() {
  if (!isDrawingPath) return;
  isDrawingPath = false;
  if (customPath.length >= 3) {
    document.getElementById('path-info').textContent = '路径完成: ' + customPath.length + ' 个点';
  } else {
    customPath = [];
    document.getElementById('path-info').textContent = '路径太短，请重新拖拽绘制';
  }
}
mapCanvas.addEventListener('pointerdown', (e) => {
  try { mapCanvas.setPointerCapture(e.pointerId); } catch (err) {}
  if (e.button === 2) {
    isPanning = true;
    panStartX = e.offsetX; panStartY = e.offsetY;
    panStartOX = viewOffsetX; panStartOY = viewOffsetY;
    e.preventDefault();
  } else if (e.button === 0 && pathDrawMode && !pathExecuting) {
    isDrawingPath = true;
    customPath = [];
    const [wx, wy] = canvasToWorld(e.offsetX, e.offsetY);
    customPath.push({ x: wx, y: wy });
    document.getElementById('path-info').textContent = '绘制中... 1 个点';
    e.preventDefault();
  }
});
mapCanvas.addEventListener('pointermove', (e) => {
  if (isPanning) {
    viewOffsetX = panStartOX + (e.offsetX - panStartX);
    viewOffsetY = panStartOY + (e.offsetY - panStartY);
  }
  if (isDrawingPath) {
    const [wx, wy] = canvasToWorld(e.offsetX, e.offsetY);
    const last = customPath[customPath.length - 1];
    const dx = wx - last.x, dy = wy - last.y;
    if (Math.sqrt(dx*dx + dy*dy) > 0.06) {
      customPath.push({ x: wx, y: wy });
      document.getElementById('path-info').textContent = '绘制中... ' + customPath.length + ' 个点';
    }
  }
});
mapCanvas.addEventListener('pointerup', (e) => {
  if (e.button === 2) isPanning = false;
  if (e.button === 0) finishPathDrawing();
});
window.addEventListener('pointerup', (e) => {
  if (e.button === 0) finishPathDrawing();
});
window.addEventListener('pointercancel', () => { isPanning = false; finishPathDrawing(); });
mapCanvas.addEventListener('contextmenu', (e) => e.preventDefault());

mapCanvas.addEventListener('click', (e) => {
  if (isPanning) return;
  const [wx, wy] = canvasToWorld(e.offsetX, e.offsetY);
  if (pathDrawMode) return;
  if (waypointMode) {
    waypoints.push({x: wx, y: wy});
    document.getElementById('waypoint-info').textContent = '已添加 ' + waypoints.length + ' 个航点';
    return;
  }
  if (pathExecuting || waypointExecuting) return;
  sendNavGoal(wx, wy, poseData ? poseData.yaw : 0);
});

// ============================================================
//  虚拟摇杆
// ============================================================
const jsCanvas = document.getElementById('joystick-canvas');
const jsCtx = jsCanvas.getContext('2d');
const jsCenter = 65, jsBaseR = 55, jsKnobR = 22;
let jsActive = false, jsKnobX = 65, jsKnobY = 65;

function drawJoystick() {
  jsCtx.clearRect(0, 0, 130, 130);
  jsCtx.fillStyle = 'rgba(79,195,247,0.1)';
  jsCtx.beginPath(); jsCtx.arc(jsCenter, jsCenter, jsBaseR, 0, Math.PI*2); jsCtx.fill();
  jsCtx.strokeStyle = 'rgba(79,195,247,0.4)';
  jsCtx.lineWidth = 2;
  jsCtx.stroke();
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
  const vx = -dy / jsBaseR * 0.5;
  const wz = -dx / jsBaseR * 1.5;
  sendCmdVel(vx, 0, wz);
}
function jsEnd() {
  jsActive = false;
  jsKnobX = jsCenter; jsKnobY = jsCenter;
  drawJoystick();
  sendCmdVel(0, 0, 0);
}
jsCanvas.addEventListener('pointerdown', (e) => {
  e.preventDefault();
  try { jsCanvas.setPointerCapture(e.pointerId); } catch (err) {}
  jsStart(e.clientX, e.clientY);
});
window.addEventListener('pointermove', (e) => { if (jsActive) jsMove(e.clientX, e.clientY); });
window.addEventListener('pointerup', () => { if (jsActive) jsEnd(); });
window.addEventListener('pointercancel', () => { if (jsActive) jsEnd(); });


// 横向按钮 + STOP
let lateralDir = 0;
function setupHoldButton(id, dir) {
  const btn = document.getElementById(id);
  const start = (e) => { e.preventDefault(); lateralDir = dir; btn.classList.add('pressed'); };
  const end = () => { if (lateralDir === dir) { lateralDir = 0; btn.classList.remove('pressed'); } };
  btn.addEventListener('mousedown', start);
  btn.addEventListener('touchstart', start, {passive: false});
  window.addEventListener('mouseup', end);
  window.addEventListener('touchend', end);
  window.addEventListener('touchcancel', end);
}
setupHoldButton('btn-left', -1);
setupHoldButton('btn-right', 1);
setInterval(() => { if (lateralDir !== 0 && !jsActive) sendCmdVel(0, lateralDir * 0.3, 0); }, 50);

document.getElementById('btn-stop').addEventListener('click', () => {
  sendCmdVel(0, 0, 0);
  cancelNav();
  cancelRoute();
  pathExecuting = false;
  waypointExecuting = false;
  document.getElementById('nav-overlay').style.display = 'none';
  document.getElementById('nav-status-text').textContent = '已停止';
  setTimeout(() => { document.getElementById('nav-status-text').textContent = '待机'; }, 1000);
});
document.getElementById('btn-cancel-nav').addEventListener('click', () => {
  cancelNav();
});

// 模式按钮
document.querySelectorAll('.mode-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    if (btn.classList.contains('todo')) return;
    document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
  });
});

// 画路径功能
const btnDrawPath = document.getElementById('btn-draw-path');
const pathActions = document.getElementById('path-actions');
const btnExecPath = document.getElementById('btn-exec-path');
const btnClearPath = document.getElementById('btn-clear-path');
const btnCancelPath = document.getElementById('btn-cancel-path');

btnDrawPath.addEventListener('click', () => {
  if (pathExecuting) return;
  pathDrawMode = !pathDrawMode;
  btnDrawPath.classList.toggle('active', pathDrawMode);
  pathActions.style.display = pathDrawMode ? 'block' : 'none';
  if (pathDrawMode) {
    waypointMode = false;
    btnWaypoint.classList.remove('active');
    waypointActions.style.display = 'none';
  }
});

btnClearPath.addEventListener('click', () => {
  customPath = [];
  document.getElementById('path-info').textContent = '按住左键拖拽绘制路径';
});

btnExecPath.addEventListener('click', () => {
  if (customPath.length < 3) {
    alert('请先拖拽绘制一条路径（至少 3 个点）');
    return;
  }
  const points = customPath.map(p => ({x: p.x, y: p.y}));
  fetch('/api/follow_path', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({frame: 'map', points: points}),
  }).catch(() => {});
  pathDrawMode = false;
  btnDrawPath.classList.remove('active');
  pathExecuting = true;
  navGoal = null;
  pathData = null;
  document.getElementById('nav-overlay').style.display = 'none';
  document.getElementById('nav-status-text').textContent = '路径已发送，等待 route_follower 执行...';
  btnExecPath.style.display = 'none';
  btnClearPath.style.display = 'none';
  btnCancelPath.style.display = 'inline';
});

btnCancelPath.addEventListener('click', () => {
  cancelRoute();
  pathExecuting = false;
  customPath = [];
  pathActions.style.display = 'none';
  btnExecPath.style.display = 'inline';
  btnClearPath.style.display = 'inline';
  btnCancelPath.style.display = 'none';
  document.getElementById('nav-status-text').textContent = '已取消';
  setTimeout(() => { document.getElementById('nav-status-text').textContent = '待机'; }, 1000);
});

// 航点导航
const btnWaypoint = document.getElementById('btn-waypoint');
const waypointActions = document.getElementById('waypoint-actions');
const btnExecWaypoint = document.getElementById('btn-exec-waypoint');
const btnClearWaypoint = document.getElementById('btn-clear-waypoint');
const btnCancelWaypoint = document.getElementById('btn-cancel-waypoint');

btnWaypoint.addEventListener('click', () => {
  if (waypointExecuting) return;
  waypointMode = !waypointMode;
  btnWaypoint.classList.toggle('active', waypointMode);
  waypointActions.style.display = waypointMode ? 'block' : 'none';
  if (waypointMode) {
    pathDrawMode = false;
    btnDrawPath.classList.remove('active');
    pathActions.style.display = 'none';
  }
});

btnClearWaypoint.addEventListener('click', () => {
  waypoints = [];
  document.getElementById('waypoint-info').textContent = '已添加 0 个航点';
});

btnExecWaypoint.addEventListener('click', () => {
  if (waypoints.length < 1) {
    alert('至少需要 1 个航点');
    return;
  }
  const points = waypoints.map(p => ({x: p.x, y: p.y}));
  fetch('/api/waypoints', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({frame: 'map', points: points}),
  }).catch(() => {});
  waypointMode = false;
  btnWaypoint.classList.remove('active');
  waypointExecuting = true;
  navGoal = null;
  pathData = null;
  document.getElementById('nav-overlay').style.display = 'none';
  document.getElementById('nav-status-text').textContent = '航点已发送，等待 route_follower 依次导航...';
  btnExecWaypoint.style.display = 'none';
  btnClearWaypoint.style.display = 'none';
  btnCancelWaypoint.style.display = 'inline';
});

btnCancelWaypoint.addEventListener('click', () => {
  cancelRoute();
  waypointExecuting = false;
  waypoints = [];
  waypointActions.style.display = 'none';
  btnExecWaypoint.style.display = 'inline';
  btnClearWaypoint.style.display = 'inline';
  btnCancelWaypoint.style.display = 'none';
  document.getElementById('nav-status-text').textContent = '已取消';
  setTimeout(() => { document.getElementById('nav-status-text').textContent = '待机'; }, 1000);
});

// ============================================================
//  启动：HTTP 轮询 + 渲染循环
// ============================================================
startPolling();
requestAnimationFrame(render);
</script>
</body>
</html>

"""


# ============================================================
#  ROS2 节点
# ============================================================
class RobotGuiNgNode(Node):
    def __init__(self):
        super().__init__('robot_gui_ng')

        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8766)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom_laser')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('slam_pose_topic', '/slam_out_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('cmd_timeout', 0.5)

        self._lock = threading.Lock()
        self.latest_map = None
        self.latest_scan = None
        self.latest_path = None
        # odom_laser / odom 中的速度和相对里程计位姿
        self.odom_pose = None
        # 地图坐标系下的机器人位姿（来自 TF / AMCL / slam_out_pose）
        self.map_pose = None
        self.last_nav_result = None

        self.last_cmd_time = 0.0
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.nav_action = None
        self.nav_goal_handle = None
        self._nav_token = 0

        nav_action_name = self.get_parameter('nav_action_name').value
        if NAV2_AVAILABLE:
            try:
                self.nav_action = ActionClient(self, NavigateToPose, nav_action_name)
                self.get_logger().info(f'{nav_action_name} action client 已创建')
            except Exception as e:
                self.get_logger().warn(f'创建导航 action client 失败: {e}')

        # ---- 画线/航点巡航 topic 发布 ----
        self.route_path_pub = self.create_publisher(Path, '/route/follow_path', 10)
        self.route_waypoints_pub = self.create_publisher(Path, '/route/waypoints', 10)
        self.route_status = None
        self.create_subscription(String, '/route/status', self._route_status_cb, 10)
        self.route_stop_client = self.create_client(Trigger, '/route/stop')

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # ---- 建图/定位相关订阅 ----
        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value,
                                 self._map_cb, map_qos)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self._scan_cb, sensor_qos)
        # 激光里程计 /odom_laser（online_slam / navigation 常用）
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self._odom_cb, 5)
        # 轮式里程计 /odom（serial_bridge / hector_mapping 常用）
        self.create_subscription(Odometry, self.get_parameter('wheel_odom_topic').value,
                                 self._odom_cb, 5)
        # AMCL 地图位姿（纯导航模式）
        self.create_subscription(PoseWithCovarianceStamped,
                                 self.get_parameter('amcl_topic').value,
                                 self._amcl_cb, 5)
        # slam_toolbox 的 /slam_out_pose（如果存在）
        self.create_subscription(PoseStamped,
                                 self.get_parameter('slam_pose_topic').value,
                                 self._slam_pose_cb, 5)
        # Nav2 规划路径
        self.create_subscription(Path, self.get_parameter('path_topic').value,
                                 self._path_cb, 5)

        self.cmd_vel_pub = self.create_publisher(Twist,
                                                 self.get_parameter('cmd_vel_topic').value,
                                                 1)

        # TF 监听：map -> base_link，用于在 SLAM 时把机器人画到地图上
        self.tf_buffer = None
        if TF2_AVAILABLE:
            try:
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
                self.get_logger().info('TF listener 已创建: map -> base_link')
            except Exception as e:
                self.tf_buffer = None
                self.get_logger().warn(f'TF listener 创建失败: {e}')

        self.create_timer(0.1, self._check_cmd_timeout)
        self.create_timer(0.1, self._update_tf_pose)

        port = int(self.get_parameter('ws_port').value)
        host = str(self.get_parameter('ws_host').value)
        self.get_logger().info(f'robot_gui_ng 节点已启动，HTTP 端口 {port} ({host})')

    # ---------------- 订阅回调 ----------------
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

    def _odom_cb(self, msg):
        # /odom_laser 和 /odom 都会进这里；保存相对里程计位姿和底盘速度。
        try:
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self._lock:
                self.odom_pose = {
                    'x': round(msg.pose.pose.position.x, 4),
                    'y': round(msg.pose.pose.position.y, 4),
                    'yaw': round(yaw, 4),
                    'vx': round(msg.twist.twist.linear.x, 3),
                    'vy': round(msg.twist.twist.linear.y, 3),
                    'wz': round(msg.twist.twist.angular.z, 3),
                    'frame': msg.header.frame_id,
                    'child_frame': msg.child_frame_id,
                }
        except Exception as e:
            self.get_logger().warn(f'里程计消息处理失败: {e}')

    def _amcl_cb(self, msg):
        # AMCL 定位结果：直接在 map 坐标系
        try:
            p = msg.pose.pose
            q = p.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self._lock:
                self.map_pose = {
                    'x': round(p.position.x, 4),
                    'y': round(p.position.y, 4),
                    'yaw': round(yaw, 4),
                    'source': 'amcl',
                }
        except Exception as e:
            self.get_logger().warn(f'AMCL 位姿处理失败: {e}')

    def _slam_pose_cb(self, msg):
        # slam_toolbox 若有 /slam_out_pose，通常也是 map 坐标系
        try:
            q = msg.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self._lock:
                self.map_pose = {
                    'x': round(msg.pose.position.x, 4),
                    'y': round(msg.pose.position.y, 4),
                    'yaw': round(yaw, 4),
                    'source': 'slam',
                }
        except Exception as e:
            self.get_logger().warn(f'SLAM 位姿处理失败: {e}')

    def _update_tf_pose(self):
        # 从 TF 读取 map -> base_link，作为机器人在地图上的位置。
        # 这个在 slam_toolbox / hector / AMCL 下都能工作。
        if self.tf_buffer is None:
            return
        try:
            map_frame = str(self.get_parameter('map_frame').value)
            base_frame = str(self.get_parameter('base_frame').value)
            trans = self.tf_buffer.lookup_transform(
                map_frame, base_frame, Time(),
                timeout=Duration(seconds=0.05))
            t = trans.transform.translation
            q = trans.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self._lock:
                self.map_pose = {
                    'x': round(t.x, 4),
                    'y': round(t.y, 4),
                    'yaw': round(yaw, 4),
                    'source': 'tf',
                }
        except Exception:
            # map -> base_link 还没发布时先不画，等 SLAM/定位启动
            pass

    def _path_cb(self, msg):
        try:
            points = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
                      for p in msg.poses]
            with self._lock:
                self.latest_path = {'points': points}
        except Exception as e:
            self.get_logger().warn(f'路径消息处理失败: {e}')


    # ---------------- 画线 / 航点巡航 ----------------
    def _route_status_cb(self, msg: String):
        with self._lock:
            self.route_status = msg.data

    @staticmethod
    def _yaw_to_quat(yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _make_route_path(self, points, frame='map'):
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = frame or 'map'
        xs = [float(p.get('x', 0.0)) for p in points]
        ys = [float(p.get('y', 0.0)) for p in points]
        for i, (x, y) in enumerate(zip(xs, ys)):
            if i < len(xs) - 1:
                yaw = math.atan2(ys[i + 1] - y, xs[i + 1] - x)
            else:
                yaw = math.atan2(y - ys[i - 1], x - xs[i - 1]) if i > 0 else 0.0
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = path.header.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation = self._yaw_to_quat(yaw)
            path.poses.append(pose)
        return path

    def handle_follow_path(self, points, frame='map'):
        if not points or len(points) < 2:
            self._set_nav_result(False, '路径点不足')
            return
        self.route_path_pub.publish(self._make_route_path(points, frame))
        self.get_logger().info(f'画线路径已发送: {len(points)} 个点, frame={frame}')

    def handle_waypoints(self, points, frame='map'):
        if not points or len(points) < 1:
            self._set_nav_result(False, '航点不足')
            return
        self.route_waypoints_pub.publish(self._make_route_path(points, frame))
        self.get_logger().info(f'航点已发送: {len(points)} 个点, frame={frame}')

    def handle_cancel_route(self):
        if self.route_stop_client.service_is_ready():
            self.route_stop_client.call_async(Trigger.Request())
        else:
            self.get_logger().warn('/route/stop 服务未就绪')

    # ---------------- 数据快照 ----------------
    def snapshot_map(self):
        with self._lock:
            return self.latest_map

    def snapshot_data(self):
        with self._lock:
            odom = self.odom_pose
            map_pose = self.map_pose
            scan = self.latest_scan
            path = self.latest_path
            nav_result = self.last_nav_result
            if nav_result is not None:
                self.last_nav_result = None

        # 优先使用地图系位姿显示小车；速度从里程计 twist 补上。
        if map_pose is not None:
            pose = dict(map_pose)
            if odom is not None:
                pose['vx'] = odom.get('vx', 0.0)
                pose['vy'] = odom.get('vy', 0.0)
                pose['wz'] = odom.get('wz', 0.0)
            else:
                pose['vx'] = 0.0
                pose['vy'] = 0.0
                pose['wz'] = 0.0
        elif odom is not None:
            pose = dict(odom)
        else:
            pose = None

        return {
            'pose': pose,
            'scan': scan,
            'path': path,
            'nav_result': nav_result,
            'route_status': self.route_status,
        }

    # ---------------- 遥控 ----------------
    def handle_cmd_vel(self, vx, vy, wz):
        try:
            twist = Twist()
            twist.linear.x = float(vx)
            twist.linear.y = float(vy)
            twist.angular.z = float(wz)
            self.cmd_vel_pub.publish(twist)
            self.last_cmd_time = time.time()
        except Exception as e:
            self.get_logger().warn(f'遥控指令处理失败: {e}')

    def _check_cmd_timeout(self):
        if self.last_cmd_time > 0 and (time.time() - self.last_cmd_time) > self.cmd_timeout:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.last_cmd_time = 0.0

    # ---------------- 导航 ----------------
    def handle_nav_goal(self, x, y, yaw):
        if self.nav_action is None:
            self._set_nav_result(False, 'nav2 action 不可用')
            return
        try:
            self.get_logger().info(f'导航目标: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
            self._nav_token += 1
            token = self._nav_token
            threading.Thread(target=self._send_nav_goal_worker,
                             args=(float(x), float(y), float(yaw), token),
                             daemon=True).start()
        except Exception as e:
            self._set_nav_result(False, str(e))

    def _send_nav_goal_worker(self, x, y, yaw, token):
        try:
            if token != self._nav_token:
                return
            if not self.nav_action.wait_for_server(timeout_sec=3.0):
                if token == self._nav_token:
                    self._set_nav_result(False, 'nav2 action server 未响应')
                return
            if token != self._nav_token:
                return

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw

            future = self.nav_action.send_goal_async(goal_msg,
                                                     feedback_callback=self._nav_feedback)
            future.add_done_callback(lambda fut: self._nav_goal_accepted(fut, token))
        except Exception as e:
            if token == self._nav_token:
                self._set_nav_result(False, str(e))

    def _nav_feedback(self, feedback_msg):
        pass

    def _nav_goal_accepted(self, future, token):
        if token != self._nav_token:
            return
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._set_nav_result(False, '目标被拒绝')
                return
            self.nav_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda fut: self._nav_result(fut, token))
        except Exception as e:
            if token == self._nav_token:
                self._set_nav_result(False, str(e))

    def _nav_result(self, future, token):
        if token != self._nav_token:
            return
        try:
            result = future.result()
            if result is None:
                status = None
            else:
                status = getattr(result, 'status', None)
                if status is None:
                    status = getattr(getattr(result, 'result', None), 'status', None)
            if status == GoalStatus.STATUS_CANCELED:
                return
            success = (status == GoalStatus.STATUS_SUCCEEDED)
            reason = '' if success else f'status={status}'
            self._set_nav_result(success, reason)
        except Exception as e:
            self._set_nav_result(False, str(e))
        finally:
            self.nav_goal_handle = None

    def _set_nav_result(self, success, reason):
        with self._lock:
            self.last_nav_result = {'success': success, 'reason': reason}

    def handle_cancel_nav(self):
        self._nav_token += 1
        if self.nav_goal_handle is not None:
            try:
                self.nav_goal_handle.cancel_goal_async()
                self.get_logger().info('导航已取消')
            except Exception as e:
                self.get_logger().warn(f'取消导航失败: {e}')
            self.nav_goal_handle = None


# ============================================================
#  HTTP Handler
# ============================================================
class Handler(BaseHTTPRequestHandler):
    node = None

    def log_message(self, fmt, *args):
        pass

    def _send_json(self, obj, status=200):
        data = json.dumps(obj).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_text(self, text, status=200):
        data = text.encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self):
        try:
            length = int(self.headers.get('Content-Length', 0))
            if length <= 0:
                return {}
            body = self.rfile.read(length)
            return json.loads(body.decode('utf-8'))
        except Exception:
            return {}

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path in ('/', '/index.html'):
            self._send_text(HTML_PAGE)
        elif path == '/api/map':
            data = self.node.snapshot_map()
            if data is None:
                self._send_json({'ok': False, 'reason': 'no map yet'})
            else:
                self._send_json({'ok': True, **data})
        elif path == '/api/data':
            self._send_json({'ok': True, **self.node.snapshot_data()})
        else:
            self._send_json({'ok': False, 'reason': 'not found'}, 404)

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path == '/api/cmd_vel':
            msg = self._read_json()
            self.node.handle_cmd_vel(msg.get('vx', 0.0), msg.get('vy', 0.0), msg.get('wz', 0.0))
            self._send_json({'ok': True})
        elif path == '/api/nav_goal':
            msg = self._read_json()
            self.node.handle_nav_goal(msg.get('x', 0.0), msg.get('y', 0.0), msg.get('yaw', 0.0))
            self._send_json({'ok': True})
        elif path == '/api/cancel_nav':
            self.node.handle_cancel_nav()
            self._send_json({'ok': True})
        elif path == '/api/follow_path':
            msg = self._read_json()
            self.node.handle_follow_path(msg.get('points', []), msg.get('frame', 'map'))
            self._send_json({'ok': True})
        elif path == '/api/waypoints':
            msg = self._read_json()
            self.node.handle_waypoints(msg.get('points', []), msg.get('frame', 'map'))
            self._send_json({'ok': True})
        elif path == '/api/cancel_route':
            self.node.handle_cancel_route()
            self._send_json({'ok': True})
        else:
            self._send_json({'ok': False, 'reason': 'not found'}, 404)


# ============================================================
#  main
# ============================================================
def _spin_executor_safely(executor):
    # SIGINT/Ctrl-C 时 rclpy 会先关闭 context，后台 spin 线程可能因 context 无效
    # 而抛 RCLError/ExternalShutdownException，这里统一吞掉，保证进程干净退出。
    try:
        executor.spin()
    except Exception:
        pass


def main():
    rclpy.init(args=None)
    node = RobotGuiNgNode()

    Handler.node = node
    host = str(node.get_parameter('ws_host').value)
    port = int(node.get_parameter('ws_port').value)
    httpd = ThreadingHTTPServer((host, port), Handler)
    web_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    web_thread.start()
    node.get_logger().info(f'HTTP 服务器已启动: http://{host}:{port}/')

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=_spin_executor_safely, args=(executor,),
                                   daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            time.sleep(0.1)
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
        web_thread.join(timeout=3.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
