#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""odom_compare_bridge.py — 串口桥 + 里程计对比 + 串口命令 Web 面板

复用 mecanum_bringup.serial_bridge.SerialBridge，不额外打开串口。
网页端口 8765。
"""
import json
import math
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField
from mecanum_bringup.serial_bridge import SerialBridge


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


HTML_HEAD = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Odom Compare + Serial Control</title>
<style>
body{font-family:Arial,sans-serif;background:#111;color:#eee;margin:0;padding:16px}
h1{font-size:20px}.flex{display:flex;flex-wrap:wrap;gap:16px}
.box{background:#1a1a1a;border:1px solid #333;border-radius:8px;padding:12px;margin:8px 0}
canvas{background:#000;border:1px solid #444;border-radius:8px;width:100%;max-width:760px}
table{border-collapse:collapse;min-width:540px}
th,td{border:1px solid #444;padding:5px 8px;text-align:right}
th{background:#222}.wheel{color:#ff5555}.laser{color:#55aaff}.err{color:#ffcc00}
button{background:#2a2a2a;color:#eee;border:1px solid #555;padding:7px 11px;margin:3px;border-radius:6px;cursor:pointer;touch-action:none;user-select:none}
.ctrl button{width:58px;height:42px;font-size:17px}
input{background:#222;color:#eee;border:1px solid #555;padding:8px;border-radius:6px;width:260px}
.note{color:#aaa;font-size:13px;margin-top:6px}
</style>
</head>
<body>
<h1>Odom Compare + Serial Web Control</h1>
<div class="flex">
<div class="box">
<canvas id="map" width="760" height="460"></canvas>
<div><button onclick="setBaseline()">设置基准点</button><button onclick="clearHistory()">清空轨迹</button><button onclick="sendCmd('STOP')">STOP</button></div>
<div class="note">红色=轮式 /odom，蓝色=激光 /odom_laser。轨迹相对基准点显示。</div>
</div>
<div class="box ctrl">
<div>
<button style="visibility:hidden"></button><button data-cmd="w" class="mov">W</button><button style="visibility:hidden"></button><br>
<button data-cmd="a" class="mov">A</button><button data-cmd="s" class="mov">S</button><button data-cmd="d" class="mov">D</button><br>
<button data-cmd="q" class="mov">Q</button><button style="visibility:hidden"></button><button data-cmd="e" class="mov">E</button>
</div>
<div class="note">W/S 前后，Q/E 横移，A/D 旋转。按住移动，松开停。</div>
</div>
<div class="box">
<table>
<tr><th></th><th class="wheel">轮式 /odom</th><th class="laser">激光 /odom_laser</th><th class="err">误差</th></tr>
<tr><td>X(m)</td><td id="wx">-</td><td id="lx">-</td><td id="ex">-</td></tr>
<tr><td>Y(m)</td><td id="wy">-</td><td id="ly">-</td><td id="ey">-</td></tr>
<tr><td>Yaw(°)</td><td id="wyaw">-</td><td id="lyaw">-</td><td id="eyaw">-</td></tr>
<tr><td>ΔX(m)</td><td id="wdx">-</td><td id="ldx">-</td><td>-</td></tr>
<tr><td>ΔY(m)</td><td id="wdy">-</td><td id="ldy">-</td><td>-</td></tr>
<tr><td>ΔYaw(°)</td><td id="wdyaw">-</td><td id="ldyaw">-</td><td>-</td></tr>
<tr><td>Vx(m/s)</td><td id="wvx">-</td><td id="lvx">-</td><td>-</td></tr>
<tr><td>Vy(m/s)</td><td id="wvy">-</td><td id="lvy">-</td><td>-</td></tr>
<tr><td>Wz(rad/s)</td><td id="wwz">-</td><td id="lwz">-</td><td>-</td></tr>
<tr><td>距离误差(m)</td><td colspan="3" id="edist">-</td></tr>
</table>
</div>
<div class="box">
<b>磁力计原始（TELEM BIN 模式）</b><br>
<span id="magx">X:-</span>&nbsp;&nbsp;<span id="magy">Y:-</span>&nbsp;&nbsp;<span id="magz">Z:-</span>
</div>
</div>
"""

HTML_CMD = """
<div class="box">
<b>自定义串口命令</b><br>
<input id="cmd" placeholder="V,0.3,0,0 / MAGCAL,10000 / STATUS / ODORST">
<button onclick="sendCustom()">发送</button>
<div class="note" id="status"></div>
</div>
<div class="box">
<b>运动串口命令</b><br>
<button onclick="sendCmd('V,0.30,0,0')">前进 0.3</button>
<button onclick="sendCmd('V,-0.30,0,0')">后退 0.3</button>
<button onclick="sendCmd('V,0,0.30,0')">左移 0.3</button>
<button onclick="sendCmd('V,0,-0.30,0')">右移 0.3</button>
<button onclick="sendCmd('V,0,0,0.8')">左转 0.8</button>
<button onclick="sendCmd('V,0,0,-0.8')">右转 0.8</button>
<button onclick="sendCmd('STOP')">STOP</button>
</div>
<div class="box">
<b>系统</b><br>
<button onclick="sendCmd('STATUS')">STATUS</button>
<button onclick="sendCmd('VERSION')">VERSION</button>
<button onclick="sendCmd('DEBUG,4')">DEBUG 4</button>
<button onclick="sendCmd('DEBUG,3')">DEBUG 3</button>
<button onclick="sendCmd('ENCVAL')">ENCVAL</button>
<button onclick="sendCmd('PIDTUNE')">PIDTUNE</button>
</div>
<div class="box">
<b>遥测</b><br>
<button onclick="sendCmd('TELEM,ON')">TELEM ON</button>
<button onclick="sendCmd('TELEM,OFF')">TELEM OFF</button>
<button onclick="sendCmd('TELEM,ASCII')">TELEM ASCII</button>
<button onclick="sendCmd('TELEM,BIN')">TELEM BIN</button>
<button onclick="sendCmd('TELEM,NOW')">TELEM NOW</button>
</div>
<div class="box">
<b>磁力计 / 航向</b><br>
<button onclick="sendCmd('AHRSMAG,ON')">AHRSMAG ON</button>
<button onclick="sendCmd('AHRSMAG,OFF')">AHRSMAG OFF</button>
<button onclick="sendCmd('MAGCAL,10000')">MAGCAL 10s</button>
<button onclick="sendCmd('GYROCAL,1000')">GYROCAL 1000</button>
<button onclick="sendCmd('GYROCAL,2000')">GYROCAL 2000</button>
<button onclick="sendCmd('MAGCAL,20000')">MAGCAL 20s</button>
</div>
<div class="box">
<b>里程计</b><br>
<button onclick="sendCmd('ODORST')">ODORST</button>
<button onclick="sendCmd('ODOSET,1.0,2.0,90')">ODOSET 示例</button>
</div>
<div class="box">
<b>单轮</b><br>
<button onclick="sendCmd('WALL,60')">WALL 60</button>
<button onclick="sendCmd('WALL,STOP')">WALL STOP</button>
<button onclick="sendCmd('W1,60')">W1 60</button>
<button onclick="sendCmd('W2,60')">W2 60</button>
<button onclick="sendCmd('W3,60')">W3 60</button>
<button onclick="sendCmd('W4,60')">W4 60</button>
</div>
<script>
const canvas=document.getElementById('map');
const ctx=canvas.getContext('2d');
let wheel=[],laser=[],baseline=null,autoScale=true;
const MOV={w:{vx:0.30,vy:0,wz:0},s:{vx:-0.30,vy:0,wz:0},a:{vx:0,vy:0,wz:0.8},d:{vx:0,vy:0,wz:-0.8},q:{vx:0,vy:0.30,wz:0},e:{vx:0,vy:-0.30,wz:0},stop:{vx:0,vy:0,wz:0}};
async function sendCmd(cmd){const st=document.getElementById('status');try{const r=await fetch('/send?cmd='+encodeURIComponent(cmd));const d=await r.json();st.textContent=d.ok?('已发送: '+cmd):('失败');st.style.color=d.ok?'#8f8':'#f88';}catch(e){st.textContent='网络错误';st.style.color='#f88';}}
async function sendMove(cmd){const m=MOV[cmd];if(!m)return;if(m.vx===0&&m.vy===0&&m.wz===0)sendCmd('STOP');else sendCmd('V,'+m.vx+','+m.vy+','+m.wz);}
function bindControls(){document.querySelectorAll('.mov').forEach(btn=>{const cmd=btn.dataset.cmd;btn.addEventListener('pointerdown',e=>{e.preventDefault();sendMove(cmd);});btn.addEventListener('pointerup',e=>{e.preventDefault();sendMove('stop');});btn.addEventListener('pointerleave',e=>{sendMove('stop');});});}
bindControls();
function sendCustom(){const v=document.getElementById('cmd').value.trim();if(v)sendCmd(v);}
async function setBaseline(){await fetch('/baseline',{method:'POST'});autoScale=true;}
async function clearHistory(){await fetch('/clear',{method:'POST'});wheel=[];laser=[];baseline=null;autoScale=true;draw();}
function drawArrow(x,y,yaw,color){const len=22;ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+len*Math.cos(yaw),y+len*Math.sin(yaw));ctx.stroke();}
function rel(p,kind){if(!baseline||!baseline[kind])return{x:p.x,y:p.y,yaw:p.yaw};const b=baseline[kind];return{x:p.x-b.x,y:p.y-b.y,yaw:p.yaw-b.yaw};}
function draw(){ctx.clearRect(0,0,canvas.width,canvas.height);const w=wheel.map(p=>rel(p,'wheel')),l=laser.map(p=>rel(p,'laser')),all=w.concat(l);if(all.length===0)return;let minx=Infinity,maxx=-Infinity,miny=Infinity,maxy=-Infinity;all.forEach(p=>{minx=Math.min(minx,p.x);maxx=Math.max(maxx,p.x);miny=Math.min(miny,p.y);maxy=Math.max(maxy,p.y);});if(autoScale){const cx=(minx+maxx)/2,cy=(miny+maxy)/2,span=Math.max(maxx-minx,maxy-miny,1.0)*0.62;minx=cx-span;maxx=cx+span;miny=cy-span;maxy=cy+span;}const scale=Math.min(canvas.width/(maxx-minx),canvas.height/(maxy-miny));const tx=x=>(x-minx)*scale,ty=y=>canvas.height-(y-miny)*scale;ctx.strokeStyle='#333';ctx.lineWidth=1;for(let gx=Math.ceil(minx);gx<=maxx;gx++){ctx.beginPath();ctx.moveTo(tx(gx),0);ctx.lineTo(tx(gx),canvas.height);ctx.stroke();}for(let gy=Math.ceil(miny);gy<=maxy;gy++){ctx.beginPath();ctx.moveTo(0,ty(gy));ctx.lineTo(canvas.width,ty(gy));ctx.stroke();}function trace(pts,color){ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();for(let i=0;i<pts.length;i++){const x=tx(pts[i].x),y=ty(pts[i].y);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}ctx.stroke();if(pts.length){const last=pts[pts.length-1];drawArrow(tx(last.x),ty(last.y),last.yaw,color);ctx.beginPath();ctx.arc(tx(last.x),ty(last.y),5,0,2*Math.PI);ctx.stroke();}}trace(w,'#ff5555');trace(l,'#55aaff');}
function setText(id,v,cls){const el=document.getElementById(id);el.textContent=v;el.className=cls||'';}
function fmt(v,n=3){return v.toFixed(n);}
function ang180(d){while(d>180)d-=360;while(d<-180)d+=360;return d;}
async function poll(){try{const res=await fetch('/data');const d=await res.json();wheel=d.wheel||[];laser=d.laser||[];baseline=d.baseline||null;const w=d.latest.wheel,l=d.latest.laser;if(w){setText('wx',fmt(w.x),'wheel');setText('wy',fmt(w.y),'wheel');setText('wyaw',fmt(w.yaw*180/Math.PI,1),'wheel');setText('wvx',fmt(w.vx),'wheel');setText('wvy',fmt(w.vy),'wheel');setText('wwz',fmt(w.wz),'wheel');}if(l){setText('lx',fmt(l.x),'laser');setText('ly',fmt(l.y),'laser');setText('lyaw',fmt(l.yaw*180/Math.PI,1),'laser');setText('lvx',fmt(l.vx),'laser');setText('lvy',fmt(l.vy),'laser');setText('lwz',fmt(l.wz),'laser');}if(w&&l){setText('ex',fmt(w.x-l.x),'err');setText('ey',fmt(w.y-l.y),'err');setText('eyaw',fmt(ang180((w.yaw-l.yaw)*180/Math.PI),1),'err');setText('edist',fmt(Math.hypot(w.x-l.x,w.y-l.y)),'err');}if(baseline){if(w){setText('wdx',fmt(w.x-baseline.wheel.x),'wheel');setText('wdy',fmt(w.y-baseline.wheel.y),'wheel');setText('wdyaw',fmt(ang180((w.yaw-baseline.wheel.yaw)*180/Math.PI),1),'wheel');}if(l){setText('ldx',fmt(l.x-baseline.laser.x),'laser');setText('ldy',fmt(l.y-baseline.laser.y),'laser');setText('ldyaw',fmt(ang180((l.yaw-baseline.laser.yaw)*180/Math.PI),1),'laser');}}if(d.mag){document.getElementById('magx').textContent='X:'+d.mag.x.toFixed(1);document.getElementById('magy').textContent='Y:'+d.mag.y.toFixed(1);document.getElementById('magz').textContent='Z:'+d.mag.z.toFixed(1);}draw();}catch(e){console.error(e);}}
setInterval(poll,200);poll();
</script>
</body>
</html>"""

HTML = HTML_HEAD + HTML_CMD

class OdomCompareBridge(SerialBridge):
    def __init__(self):
        super().__init__()
        self.wheel_history = deque(maxlen=4000)
        self.laser_history = deque(maxlen=4000)
        self.wheel_latest = None
        self.laser_latest = None
        self.baseline = None

        # 订阅自己发布的 /odom 用于对比显示
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.wheel_cb, 10)
        self.laser_sub = self.create_subscription(Odometry, '/odom_laser', self.laser_cb, 10)
        self.mag_sub = self.create_subscription(MagneticField, '/imu/imu407/mag', self.mag_cb, 10)
        self.mag_latest = None

        self.server = ThreadingHTTPServer(('0.0.0.0', 8765), Handler)
        Handler.bridge = self
        self.web_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.web_thread.start()
        self.get_logger().info('odom_compare_bridge web ready: http://<ip>:8765/')

    @staticmethod
    def extract(msg: Odometry):
        p = msg.pose.pose
        t = msg.twist.twist
        yaw = yaw_from_quat(p.orientation)
        return {
            'x': p.position.x,
            'y': p.position.y,
            'yaw': yaw,
            'vx': t.linear.x,
            'vy': t.linear.y,
            'wz': t.angular.z,
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }

    def wheel_cb(self, msg):
        self.wheel_latest = self.extract(msg)
        self.wheel_history.append(self.wheel_latest)

    def laser_cb(self, msg):
        self.laser_latest = self.extract(msg)
        self.laser_history.append(self.laser_latest)

    def mag_cb(self, msg):
        self.mag_latest = {
            'x': msg.magnetic_field.x,
            'y': msg.magnetic_field.y,
            'z': msg.magnetic_field.z,
        }

    def set_baseline(self):
        if self.wheel_latest and self.laser_latest:
            self.baseline = {
                'wheel': dict(self.wheel_latest),
                'laser': dict(self.laser_latest),
            }
            return True
        return False

    def clear(self):
        self.wheel_history.clear()
        self.laser_history.clear()
        self.baseline = None

    def send_serial(self, cmd: str):
        cmd = cmd.strip()
        if not cmd:
            return False, 'empty command'
        self.send(cmd)
        return True, ''

    def snapshot(self):
        return {
            'wheel': list(self.wheel_history),
            'laser': list(self.laser_history),
            'latest': {'wheel': self.wheel_latest, 'laser': self.laser_latest},
            'mag': self.mag_latest,
            'baseline': self.baseline,
        }

    def destroy_node(self):
        try:
            self.server.shutdown()
        except Exception:
            pass
        super().destroy_node()


class Handler(BaseHTTPRequestHandler):
    bridge = None

    def _send_json(self, obj, status=200):
        data = json.dumps(obj).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML.encode('utf-8'))
        elif parsed.path == '/data':
            self._send_json(self.bridge.snapshot())
        elif parsed.path == '/send':
            q = parse_qs(parsed.query)
            cmd = q.get('cmd', [''])[0]
            ok, err = self.bridge.send_serial(cmd)
            self._send_json({'ok': ok, 'error': err})
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/baseline':
            self._send_json({'ok': self.bridge.set_baseline()})
        elif self.path == '/clear':
            self.bridge.clear()
            self._send_json({'ok': True})
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, fmt, *args):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = OdomCompareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
