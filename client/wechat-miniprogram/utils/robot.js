/**
 * robot.js — warehouse_inspector 手机端通信封装
 *
 * 后端：机器人（RK3588）上运行的 inspector_web / web_bridge 节点
 *       （HTTP，默认端口 8765；`ros2 launch inspector_web web.launch.py`）
 *
 * 接口一览（实现在 web_bridge.py 的 Handler 里）：
 *   GET  /data                        -> 全局快照 JSON（位姿/速度/雷达/地图/巡检/圆/相机/照片…）
 *   POST /cmd_vel  {vx,vy,wz}         -> 发速度（桥内部限幅 max_vx/vy/max_wz）
 *   POST /stop                        -> 零速度（并顺带通知 chassis_bridge）
 *   GET  /send?cmd=V,x,y,z|STOP       -> 旧桥兼容；**其它固件命令会被明确拒绝**
 *   POST /inspection/{start,stop,clear}
 *   POST /circle/{start,stop,clear}
 *   POST /circle/point?x=&y=          -> 给 path_follower 喂一个点（= RViz Publish Point）
 *   GET  /map.png /snapshot.jpg /photo?name= /photos /report /waypoints
 *
 * ⚠ 本桥**没有串口**：它是纯粹的 ROS↔HTTP 翻译层，不碰 /dev/ttyS1。
 *   所以「里程归零 / TELEM,ON」这类固件命令发过去只会得到一条拒绝说明。
 */

const STORAGE_KEY = 'robot_config'

const DEFAULT_CONFIG = {
  ip: '192.168.1.100',   // 机器人局域网 IP（Tailscale 地址也可以）
  port: 8765,            // web_bridge 监听端口
  pollInterval: 500,     // 遥测轮询间隔 (ms)
  linearSpeed: 0.30,     // 默认线速度 (m/s)
  rotationSpeed: 1.00    // 默认旋转速度 (rad/s)
}

/**
 * 与 web_bridge 的限幅保持一致（桥参数 max_vx / max_vy / max_wz）。
 * 手机端先夹一次，滑块就不会给出一个后端根本不执行的数。
 * 桥内部还会再夹一次并把它真正采用的值回在响应里（vx/vy/wz 字段）。
 */
const LIMITS = {
  vx: 0.50,
  vy: 0.50,
  wz: 1.50
}

/** 巡检/圆的 Trigger 服务名片段（拼在 /inspection/ 或 /circle/ 后面） */
const ACTIONS = ['start', 'stop', 'clear']

/* ---------------------------------------------------------------- 配置 */

/** 读取配置（合并默认值，剔除未知键） */
function loadConfig () {
  const cfg = Object.assign({}, DEFAULT_CONFIG)
  try {
    const saved = wx.getStorageSync(STORAGE_KEY)
    if (saved && typeof saved === 'object') {
      Object.keys(DEFAULT_CONFIG).forEach(function (k) {
        if (saved[k] !== undefined && saved[k] !== null && saved[k] !== '') {
          cfg[k] = saved[k]
        }
      })
    }
  } catch (e) {
    /* 忽略：读不到就用默认值 */
  }
  return cfg
}

/** 保存配置（只保留已知键） */
function saveConfig (cfg) {
  const clean = {}
  Object.keys(DEFAULT_CONFIG).forEach(function (k) {
    if (cfg && cfg[k] !== undefined) clean[k] = cfg[k]
  })
  wx.setStorageSync(STORAGE_KEY, clean)
}

/** 后端基础地址，例如 http://100.94.102.64:8765 */
function getBaseUrl () {
  const cfg = loadConfig()
  return 'http://' + cfg.ip + ':' + cfg.port
}

/**
 * 构造带缓存的图片/文件地址。
 * @param {string} path  以 / 开头，如 /map.png
 * @param {object} query 额外查询参数
 * @param {boolean} bust 是否加 ?t=时间戳 破缓存
 */
function getUrl (path, query, bust) {
  const parts = []
  const q = Object.assign({}, query || {})
  if (bust) q.t = Date.now()
  Object.keys(q).forEach(function (k) {
    parts.push(encodeURIComponent(k) + '=' + encodeURIComponent(q[k]))
  })
  const qs = parts.length ? ('?' + parts.join('&')) : ''
  return getBaseUrl() + path + qs
}

/* ---------------------------------------------------------------- 底层请求 */

/**
 * 统一请求封装。
 * - 超时 3s（局域网足够；避免页面卡住）
 * - 成功判定：HTTP 2xx **并且** 业务 ok!==false
 *   （桥在“服务不可用/参数错”时也回 200 + {ok:false,message}）
 * @returns {Promise<object>} 解析后的对象
 */
function request (path, options) {
  const opt = options || {}
  return new Promise(function (resolve, reject) {
    wx.request({
      url: getUrl(path, opt.query),
      method: opt.method || 'GET',
      data: opt.data,
      header: opt.data ? { 'content-type': 'application/json' } : {},
      timeout: opt.timeout || 3000,
      success: function (res) {
        const body = res.data
        if (res.statusCode < 200 || res.statusCode >= 300) {
          reject(new Error('HTTP ' + res.statusCode +
            (body && body.message ? '：' + body.message : '')))
          return
        }
        if (body && body.ok === false) {
          reject(new Error(body.message || '后端返回失败'))
          return
        }
        resolve(body || {})
      },
      fail: function (err) {
        reject(new Error(friendly(err)))
      }
    })
  })
}

/** 把 wx.request 的原始错误翻译成人能看懂的一句话 */
function friendly (err) {
  const msg = (err && (err.errMsg || err.message)) || '未知错误'
  if (/timeout/i.test(msg)) return '请求超时（机器人没响应）'
  if (/fail.*abort/i.test(msg)) return '连接被中止'
  if (/domain|合法域名|not in domain/i.test(msg)) return '域名校验未关闭（开发者工具需勾选“不校验合法域名”）'
  if (/fail/i.test(msg)) return '连不上 ' + getBaseUrl() + '（IP/端口 / 同一局域网 / 桥是否在跑）'
  return msg
}

/* ---------------------------------------------------------------- 遥测 */

/** 拉取全局快照：/data */
function fetchData () {
  return request('/data')
}

/* ---------------------------------------------------------------- 控制 */

/** 发速度矢量（POST /cmd_vel）。桥会限幅，返回值里是它真正采用的值。 */
function sendMove (vx, vy, wz) {
  return request('/cmd_vel', {
    method: 'POST',
    data: { vx: clamp(vx, LIMITS.vx), vy: clamp(vy, LIMITS.vy), wz: clamp(wz, LIMITS.wz) }
  })
}

/** 急停（POST /stop） */
function stop () {
  return request('/stop', { method: 'POST' })
}

/**
 * 旧桥兼容入口（GET /send?cmd=）。
 * 本桥只认 `STOP` 与 `V,<vx>,<vy>,<wz>`，其余会被拒绝 —— 拒绝原因由后端给出。
 */
function sendCmd (cmd) {
  return request('/send', { query: { cmd: cmd } })
}

/* ---------------------------------------------------------------- 巡检 / 绕圆 */

/** @param {'start'|'stop'|'clear'} action */
function inspection (action) {
  return request('/inspection/' + action, { method: 'POST', timeout: 6000 })
}

/** @param {'start'|'stop'|'clear'} action */
function circle (action) {
  return request('/circle/' + action, { method: 'POST', timeout: 6000 })
}

/**
 * 给 path_follower 喂一个点（圆心 or 圆周上一点）。
 * 顺序与 RViz 一致：先圆心，再圆周上一点。
 */
function circlePoint (x, y) {
  return request('/circle/point', { method: 'POST', query: { x: x, y: y } })
}

/** 批量步骤执行：串行，任一步失败即中断并抛出该步的错误 */
function runSteps (steps) {
  let p = Promise.resolve()
  steps.forEach(function (fn) {
    p = p.then(fn)
  })
  return p
}

/* ---------------------------------------------------------------- 文件 */

/** 照片列表：/photos -> {photos:[{name,size,mtime}]} */
function fetchPhotos () {
  return request('/photos')
}

/** 最近一份巡检报告：/report（没有报告时后端也回 200，body.ok=false -> 会被 reject） */
function fetchReport () {
  return request('/report')
}

/** 航点（收集的优先）：/waypoints */
function fetchWaypoints () {
  return request('/waypoints')
}

/* ---------------------------------------------------------------- 工具 */

function clamp (v, limit) {
  const n = Number(v)
  if (!isFinite(n)) return 0
  return Math.max(-limit, Math.min(limit, n))
}

/**
 * ROS yaw（弧度，逆时针为正） -> 真罗盘角（度，0=北，顺时针增，[0,360)）。
 * 数学 yaw 逆时针为正，罗盘角顺时针为正，所以是取负。
 */
function compassDeg (yaw) {
  const n = Number(yaw)
  if (!isFinite(n)) return null
  return ((-n * 180 / Math.PI) % 360 + 360) % 360
}

/** 罗盘角 -> 中文方位 */
const SECTORS = ['北', '东北', '东', '东南', '南', '西南', '西', '西北']
function sectorText (deg) {
  if (deg === null || deg === undefined) return '--'
  return SECTORS[Math.round(((Number(deg) % 360) + 360) % 360 / 45) % 8]
}

module.exports = {
  STORAGE_KEY,
  DEFAULT_CONFIG,
  LIMITS,
  ACTIONS,
  loadConfig,
  saveConfig,
  getBaseUrl,
  getUrl,
  request,
  fetchData,
  sendMove,
  stop,
  sendCmd,
  inspection,
  circle,
  circlePoint,
  runSteps,
  fetchPhotos,
  fetchReport,
  fetchWaypoints,
  clamp,
  compassDeg,
  sectorText
}
