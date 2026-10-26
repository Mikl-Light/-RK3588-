/**
 * format.js — 展示层格式化与状态字典
 *
 * 这里只放"纯函数 + 常量"，不依赖 wx API，方便离线自测。
 */

/** 数值格式化：非数字一律回落为占位符，避免 "NaN"/"undefined" 上屏 */
function num (v, digits, fallback) {
  const fb = fallback === undefined ? '--' : fallback
  const n = Number(v)
  if (v === null || v === undefined || !isFinite(n)) return fb
  const d = digits === undefined ? 2 : digits
  return n.toFixed(d)
}

/** 带符号显示（速度用）：+0.30 / -0.30 / 0.00 */
function signed (v, digits) {
  const n = Number(v)
  if (!isFinite(n)) return '--'
  const d = digits === undefined ? 2 : digits
  const s = n.toFixed(d)
  return n > 0 ? ('+' + s) : s
}

/** 0.3 -> "0.30 m/s" 之类；单位为空则只给数字 */
function withUnit (v, digits, unit) {
  const s = num(v, digits)
  return unit ? (s + ' ' + unit) : s
}

/** 空值占位 */
function orDash (v) {
  return (v === null || v === undefined || v === '') ? '--' : v
}

/** 字节数 -> 人类可读 */
function bytes (n) {
  const v = Number(n)
  if (!isFinite(v) || v < 0) return '--'
  if (v < 1024) return v + ' B'
  if (v < 1024 * 1024) return (v / 1024).toFixed(0) + ' KB'
  return (v / 1024 / 1024).toFixed(1) + ' MB'
}

/** 秒 -> "1m23s" / "45s" */
function duration (sec) {
  const v = Number(sec)
  if (!isFinite(v) || v < 0) return '--'
  const s = Math.round(v)
  if (s < 60) return s + 's'
  const m = Math.floor(s / 60)
  const r = s % 60
  if (m < 60) return m + 'm' + r + 's'
  return Math.floor(m / 60) + 'h' + (m % 60) + 'm'
}

/* ---------------------------------------------------------- 状态字典 */

/**
 * /inspection_route/status 里 state 的取值 -> 文案 + 色调
 * 色调：ok（正常/进行中）/ warn（需要人看一眼）/ err（异常）/ idle（空闲）
 */
const INSPECTION_STATES = {
  idle: { text: '空闲', tone: 'idle' },
  started: { text: '已启动', tone: 'ok' },
  navigating: { text: '前往航点中', tone: 'ok' },
  capturing: { text: '拍照中', tone: 'ok' },
  waiting_obstacle: { text: '停车等障碍', tone: 'warn' },
  blocked_by_obstacle: { text: '被障碍阻挡', tone: 'err' },
  collected: { text: '已记录航点', tone: 'ok' },
  cleared: { text: '已清空航点', tone: 'idle' },
  finished: { text: '本轮完成', tone: 'ok' },
  aborted_unexpected: { text: '异常中止（已复位）', tone: 'err' },
  aborted: { text: '已中止', tone: 'warn' },
  stopping: { text: '正在停止', tone: 'warn' }
}

function inspectionText (state) {
  const e = INSPECTION_STATES[state]
  return e ? e.text : orDash(state)
}

function inspectionTone (state) {
  const e = INSPECTION_STATES[state]
  return e ? e.tone : 'idle'
}

/** /chassis_bridge/status 的 state=... -> 文案 */
const CHASSIS_STATES = {
  idle: '空闲',
  moving: '行驶中',
  stopped: '已停止',
  watchdog: '看门狗停车',
  unknown: '未知'
}

function chassisText (state) {
  return CHASSIS_STATES[state] || orDash(state)
}

/** /waypoints 的 source -> 文案 */
const WAYPOINT_SOURCES = {
  collected: '手机/RViz 收集',
  route_file: '配置文件',
  none: '无'
}

function waypointSourceText (src) {
  return WAYPOINT_SOURCES[src] || orDash(src)
}

/** 航点文件里 source: 'rviz_collect' 也用同一套词 */
function sourceText (src) {
  if (src === 'rviz_collect') return WAYPOINT_SOURCES.collected
  return WAYPOINT_SOURCES[src] || orDash(src)
}

/** JSON 里可能缺字段，统一补上占位符，避免 wxml 里到处写三元 */
function safe (obj, key, fallback) {
  const v = obj ? obj[key] : undefined
  return (v === null || v === undefined) ? (fallback === undefined ? '--' : fallback) : v
}

module.exports = {
  num,
  signed,
  withUnit,
  orDash,
  bytes,
  duration,
  safe,
  INSPECTION_STATES,
  CHASSIS_STATES,
  WAYPOINT_SOURCES,
  inspectionText,
  inspectionTone,
  chassisText,
  waypointSourceText,
  sourceText
}
