# 麦轮巡检车 — 微信小程序

麦轮巡检车的手机端。后端是机器人（RK3588）上跑的两个 ROS 2 包：

```
warehouse_inspector   ← 现场栈：导航 / 巡检 / 绕圆 / 相机 / 底盘桥
inspector_web         ← 手机端出口：只做 ROS↔HTTP 翻译，不开串口
```

> 历史：本小程序原本对接 `odom_compare_web`（发 `V,..` / `STOP` / `ODORST` / `TELEM,ON` 等串口命令）。
> 现在对接 `inspector_web`（`POST /cmd_vel` + `/inspection/*` + `/circle/*`）。
> **`ODORST`（里程归零）、`TELEM,ON`（遥测开关）、`LINE_FOLLOW_ON/OFF`（视觉巡线）在这个后端里已不存在**，
> 发过去只会得到一条明确的拒绝说明 —— 因为新桥根本没有串口，那三件事分别归固件 / 里程计设置 / 其它节点管。

## 功能

| 页面 | 干什么 |
|------|--------|
| **遥控** | 虚拟摇杆（上推前进 / 左右推转向）、按住左移右移、点动按钮、STOP 急停、速度滑杆、自定义命令 |
| **遥测** | 地图画布（底图 + 轨迹 + 航点 + 位姿 + 圆）、真罗盘航向、Vx/Vy/Wz、雷达统计、底盘状态、相机预览 |
| **巡检** | 开始 / 停止 / 清空航点、当前进度（第几个 / 共几个 / 尝试次数 / 等障碍多久）、最近报告、照片墙 |
| **绕圆** | 在地图上点两下圈定圆周（圆心 + 圆周点）、开始 / 停止 / 清圆 |
| **设置** | 机器人 IP / 端口 / 轮询间隔 / 默认速度、连接自检、复制调试页地址 |

## 航向显示的是「真罗盘角」

- `yaw` 是 ROS 的数学角（逆时针为正）。
- 罗盘角是顺时针为正，所以 `compass_deg = (−yaw·180/π + 360) % 360`（见 `utils/robot.js` 的 `compassDeg`）。
- 效果：**左转 → 罗盘角减小**（0°=北，90°=东，180°=南，270°=西），和手机指南针一致。

## 目录结构

```
wechat-miniprogram/
├── app.js / app.json / app.wxss          # app.wxss 是公共设计系统，页面只写自己特有的规则
├── project.config.json / sitemap.json
├── utils/
│   ├── robot.js      # HTTP 封装 + 配置 + 限幅常量 + compassDeg
│   ├── format.js     # 数值/时间格式化 + 状态字典（巡检/底盘/航点来源）
│   ├── view.js       # /data -> 视图模型（把原始 JSON 摊平成页面字段）
│   ├── mapdraw.js    # canvas 2d 渲染器：世界坐标 <-> 画布坐标
│   └── mapview.js    # initCanvas + 底图下载 + 渲染 的粘合层
├── components/joystick/
└── pages/{control,telemetry,inspection,circle,settings}/
```

## 快速开始

1. **机器人上起现场栈**
   ```bash
   ros2 launch warehouse_inspector inspection.launch.py
   # 需要照片就加 start_camera:=true
   ```
2. **再起手机端出口**
   ```bash
   ros2 launch inspector_web web.launch.py          # 默认 8765 端口
   ```
3. **确认网络**：手机与机器人同网段（或都连 Tailscale），浏览器打开 `http://<机器人IP>:8765/` 能看到调试页。
4. **导入项目**：微信开发者工具 → 导入 → 选本目录（`urlCheck` 已关）。
5. **开发者工具**：详情 → 本地设置 → 勾选 **「不校验合法域名」**（后端是局域网 HTTP）。
6. **设置页**填 IP、点「连接测试」，能看到位姿/地图/雷达/相机状态即通。

## 安全策略（已内置）

- 松开摇杆 / 横移按钮 → 立刻发零速。
- 有速度时每 300 ms 保活重发（桥的看门狗 0.8 s，留足余量）。
- 离开页面 / 连续 3 次请求失败 → 判为断线并急停。
- 指令节流 100 ms + 相同指令去重，避免摇杆的 `touchmove` 刷爆局域网。
- 同一时刻只允许一个速度请求在飞（`_pending` 存最新值，不堆积）。
- 速度滑杆的上限 = 后端限幅（Vx/Vy ≤ 0.50 m/s，Wz ≤ 1.50 rad/s），滑杆给不出后端不执行的数；
  实测桥还会再夹一次并把真正采用的值回在响应里。

## 后端接口（与 `inspector_web/inspector_web/web_bridge.py` 一一对应）

| 方法 | 路径 | 用途 |
|------|------|------|
| GET | `/data` | 全局快照 JSON（本小程序的主数据源） |
| POST | `/cmd_vel` `{vx,vy,wz}` | 发速度；响应里 `vx/vy/wz` 是**实际采用**的值 |
| POST | `/stop` | 零速（同时通知 `chassis_bridge`） |
| GET | `/send?cmd=` | 旧桥兼容；只认 `V,x,y,z` 与 `STOP` |
| POST | `/inspection/{start,stop,clear}` | 巡检 |
| POST | `/circle/{start,stop,clear}` | 绕圆 |
| POST | `/circle/point?x=&y=` | 喂点（= RViz 的 Publish Point，先圆心后圆周） |
| GET | `/map.png` | 地图 PNG（`/data.map` 给元信息） |
| GET | `/snapshot.jpg` | 相机当前帧 |
| GET | `/photos`、`/photo?name=` | 照片列表 / 单张 |
| GET | `/report`、`/waypoints` | 最近报告 / 航点 |

## `/data` 里本小程序用到的字段

```
pose      {x,y,yaw,frame,source,stamp} | null   ← source: tf(SLAM) / odom(回落)
twist     {vx,vy,wz}
scan      {min,valid,total} | null
traj      [{x,y,yaw,stamp}]  (最近 200 点)
map       {w,h,res,ox,oy} | null
inspection{state,index,total,name,attempt,max_attempts,waited_s,report,note,…}
circle    {executing,defined,external_active,geo{cx,cy,r}}
waypoints {markers:[[x,y]],files:{source,file,count}}
chassis   {state,raw}
camera    {ok,age}
photos    {count,latest}
frames    {map,base}
```

⚠ `inspection.index` 后端发的**已经是 1 起**（`payload.update({'index': index + 1, …})`），前端直接用。

## 离线自检

没有任何真机也能跑：

```bash
node tools/verify_miniprogram.js
```

覆盖：页面/组件文件齐全、tabBar 一致、wxml 里的事件处理函数真的存在、
wxml 用到的标识符都在 `data` 里声明过、js 语法、
**小程序调用的每条 HTTP 路径都在 `web_bridge.py` 里存在**、
`/data`→视图模型映射与罗盘角换算、地图画布的「世界↔画布」变换（含上下翻转方向）。
