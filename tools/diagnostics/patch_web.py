#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一次性修改 odom_compare_bridge.py，加入融合诊断可视化。"""
import re

PATH = r'C:\dosk\STM32\ODOM\tools\odom_compare_web\odom_compare_web\odom_compare_bridge.py'

with open(PATH, 'r', encoding='utf-8') as f:
    src = f.read()

# ── 1. 在 __init__ 中加入 fusion 订阅和状态 ──
old1 = "        self.diag_sub = self.create_subscription(String, '/odom_diag', self.diag_cb, 10)\n        self.mag_latest = None\n        self.diag_latest = None\n"
new1 = ("        self.diag_sub = self.create_subscription(String, '/odom_diag', self.diag_cb, 10)\n"
        "        self.fusion_sub = self.create_subscription(String, '/fusion_diag', self.fusion_cb, 10)\n"
        "        self.mag_latest = None\n"
        "        self.diag_latest = None\n"
        "        self.fusion_latest = None\n")
assert old1 in src, "patch 1 failed"
src = src.replace(old1, new1, 1)

# ── 2. 在 diag_cb 后加入 fusion_cb ──
old2 = ("    def diag_cb(self, msg):\n"
        "        try:\n"
        "            self.diag_latest = json.loads(msg.data)\n"
        "        except Exception:\n"
        "            self.diag_latest = None\n")
new2 = (old2 +
        "\n"
        "    def fusion_cb(self, msg):\n"
        "        try:\n"
        "            self.fusion_latest = json.loads(msg.data)\n"
        "        except Exception:\n"
        "            self.fusion_latest = None\n")
assert old2 in src, "patch 2 failed"
src = src.replace(old2, new2, 1)

# ── 3. 在 snapshot() 中加入 fusion ──
old3 = ("            'mag': self.mag_latest,\n"
        "            'diag': self.diag_latest,\n"
        "            'baseline': self.baseline,\n")
new3 = ("            'mag': self.mag_latest,\n"
        "            'diag': self.diag_latest,\n"
        "            'fusion': self.fusion_latest,\n"
        "            'baseline': self.baseline,\n")
assert old3 in src, "patch 3 failed"
src = src.replace(old3, new3, 1)

# ── 4. 在 HTML 中加入融合诊断面板（在 ODOM 诊断 box 后）──
old4 = ('<div class="box">\n'
        '<b>ODOM 可靠里程计诊断（ODOM TEST）</b><br>\n'
        '<span id="odomdiag">等待 /odom_diag 数据...</span>\n'
        '</div>')
new4 = (old4 + '\n' +
'''<div class="box">
<b>融合诊断（odom_fusion）</b><br>
<table style="min-width:560px;font-size:13px;">
<tr><th>指标</th><th>数值</th><th>说明</th></tr>
<tr><td>激光可信度 w</td><td id="f_w" style="color:#55aaff;">-</td><td>1=特征区信激光, 0=开阔区信轮式</td></tr>
<tr><td>Scan 有效点占比</td><td id="f_ratio">-</td><td>低于25%判定激光退化</td></tr>
<tr><td>陀螺零偏 b (°/s)</td><td id="f_b" style="color:#ffcc00;font-weight:bold;">-</td><td>激光估计的漂移率（核心数值）</td></tr>
<tr><td>零偏原始值 (°/s)</td><td id="f_braw">-</td><td>本次回归结果（未平滑）</td></tr>
<tr><td>累计补偿 (°)</td><td id="f_bcum">-</td><td>b 从估计开始累计的漂移量</td></tr>
<tr><td>有效估计次数</td><td id="f_samples">-</td><td>特征区累计的零偏估计样本数</td></tr>
<tr><td>轮式 yaw (°)</td><td id="f_yw" class="wheel">-</td><td>原始（漂移中）</td></tr>
<tr><td>补偿后轮式 yaw (°)</td><td id="f_ywc" style="color:#52C41A;">-</td><td>减去累计漂移后</td></tr>
<tr><td>激光 yaw (°)</td><td id="f_yl" class="laser">-</td><td>真值参考</td></tr>
<tr><td>融合 yaw (°)</td><td id="f_yf" style="color:#c084fc;font-weight:bold;">-</td><td>最终输出 /odom_fused</td></tr>
<tr><td>陀螺 wz (rad/s)</td><td id="f_gz">-</td><td>IMU 原始角速度</td></tr>
</table>
<div class="note" id="f_status">等待 /fusion_diag 数据... 启动 odom_fusion 节点后自动显示。</div>
</div>''')
assert old4 in src, "patch 4 failed"
src = src.replace(old4, new4, 1)

# ── 5. 在 JS poll() 中加入融合数据更新（在 odomdiag 更新后）──
old5 = "document.getElementById('odomdiag').textContent=txt;}draw();"
new5 = ("document.getElementById('odomdiag').textContent=txt;}"
        "if(d.fusion){const f=d.fusion;"
        "document.getElementById('f_w').textContent=f.trust_w!==undefined?f.trust_w.toFixed(3):'-';"
        "document.getElementById('f_ratio').textContent=f.scan_valid_ratio!==undefined?f.scan_valid_ratio.toFixed(3):'-';"
        "document.getElementById('f_b').textContent=f.bias_b_deg_s!==undefined?f.bias_b_deg_s.toFixed(4):'-';"
        "document.getElementById('f_braw').textContent=f.bias_b_raw!==undefined?f.bias_b_raw.toFixed(4):'-';"
        "document.getElementById('f_bcum').textContent=f.bias_cumulative_deg!==undefined?f.bias_cumulative_deg.toFixed(2):'-';"
        "document.getElementById('f_samples').textContent=f.bias_samples!==undefined?f.bias_samples:'-';"
        "document.getElementById('f_yw').textContent=f.yaw_wheel_deg!==undefined?f.yaw_wheel_deg.toFixed(2):'-';"
        "document.getElementById('f_ywc').textContent=f.yaw_wheel_corrected_deg!==undefined?f.yaw_wheel_corrected_deg.toFixed(2):'-';"
        "document.getElementById('f_yl').textContent=f.yaw_laser_deg!==undefined?f.yaw_laser_deg.toFixed(2):'-';"
        "document.getElementById('f_yf').textContent=f.yaw_fused_deg!==undefined?f.yaw_fused_deg.toFixed(2):'-';"
        "document.getElementById('f_gz').textContent=f.gyro_z_rad_s!==undefined?f.gyro_z_rad_s.toFixed(4):'-';"
        "const st=f.trust_w>0.5?'特征区：激光可信，正在估计零偏':(f.trust_w>0.1?'过渡区':'开阔区：激光退化，使用补偿后轮式+陀螺');"
        "document.getElementById('f_status').textContent='状态: '+st+' | 位置: ('+(f.x_fused!==undefined?f.x_fused.toFixed(3):'-')+', '+(f.y_fused!==undefined?f.y_fused.toFixed(3):'-')+')';"
        "}draw();")
assert old5 in src, "patch 5 failed"
src = src.replace(old5, new5, 1)

with open(PATH, 'w', encoding='utf-8') as f:
    f.write(src)

print("All 5 patches applied successfully.")
