/**
 * view.js — 把 /data 的原始快照摊平成页面直接可用的视图模型
 *
 * 目的：四个页面都要用同一套字段和同一套格式化，集中在这里做一次，
 * 页面里就只剩 setData + 渲染，不再各写一份 toFixed。
 */
const fmt = require('./format')
const robot = require('./robot')

function num2 (v) { return fmt.num(v, 2) }

/**
 * @param {object} d /data 返回的 JSON
 * @returns {object} 视图模型
 */
function build (d) {
  const out = {
    ok: !!(d && d.ok),
    ts: d ? d.ts : null,
    empty: !d || (!d.pose && !d.twist)
  }
  if (!d) return out

  /* ---- 位姿 ---- */
  const p = d.pose
  if (p && p.x !== undefined && p.x !== null) {
    const deg = robot.compassDeg(p.yaw)
    out.pose = { x: p.x, y: p.y, yaw: p.yaw, frame: p.frame, source: p.source }
    out.x = fmt.num(p.x, 3)
    out.y = fmt.num(p.y, 3)
    out.heading = fmt.num(deg, 1)
    out.headingText = fmt.num(deg, 1) + '°'
    out.sector = robot.sectorText(deg)
    out.poseSource = p.source === 'tf' ? 'SLAM(TF)' : '里程计'
  } else {
    out.pose = null
    out.x = out.y = '--'
    out.heading = null
    out.headingText = '--'
    out.sector = '--'
    out.poseSource = '无（TF/里程计都断）'
  }

  /* ---- 速度 ---- */
  const t = d.twist || {}
  out.vx = fmt.signed(t.vx, 3)
  out.vy = fmt.signed(t.vy, 3)
  out.wz = fmt.signed(t.wz, 3)
  out.moving = Math.abs(t.vx || 0) + Math.abs(t.vy || 0) + Math.abs(t.wz || 0) > 1e-3

  /* ---- 雷达 ---- */
  const s = d.scan
  if (s && s.total) {
    out.scanValid = s.valid
    out.scanTotal = s.total
    out.scanMin = (s.min === null || s.min === undefined) ? '--' : fmt.num(s.min, 2)
    out.scanText = out.scanValid + '/' + out.scanTotal +
      ' 有效 · 最近 ' + out.scanMin + ' m'
    out.scanOk = s.valid > 0
  } else {
    out.scanText = '无扫描数据（雷达没起来？）'
    out.scanOk = false
  }

  /* ---- 底盘 ---- */
  const ch = d.chassis || {}
  out.chassisState = ch.state || 'unknown'
  out.chassisText = fmt.chassisText(ch.state)

  /* ---- 相机 ---- */
  const c = d.camera || {}
  out.cameraOk = !!c.ok
  out.cameraAge = (c.age === null || c.age === undefined) ? '--' : fmt.num(c.age, 1)
  out.cameraText = c.ok ? ('画面 ' + out.cameraAge + ' s 前') : '无画面（start_camera 关着？）'

  /* ---- 地图 ---- */
  const m = d.map
  if (m && m.w) {
    out.map = m
    out.mapText = m.w + '×' + m.h + ' @ ' + m.res + ' m/px'
  } else {
    out.map = null
    out.mapText = '无地图（SLAM 没起？）'
  }

  /* ---- 航点 ---- */
  const wpm = (d.waypoints && d.waypoints.markers) || []
  const wpf = (d.waypoints && d.waypoints.files) || {}
  out.waypoints = wpm
  out.wpSource = wpf.source || 'none'
  out.wpSourceText = fmt.waypointSourceText(out.wpSource)
  out.wpCount = wpf.count || 0
  out.wpFile = wpf.file || ''

  /* ---- 巡检 ---- */
  const ins = d.inspection || { state: 'idle' }
  out.inspState = ins.state || 'idle'
  out.inspText = fmt.inspectionText(out.inspState)
  out.inspTone = fmt.inspectionTone(out.inspState)
  out.inspRunning = ['started', 'navigating', 'capturing', 'waiting_obstacle'].indexOf(out.inspState) >= 0
  out.inspIndex = ins.index || 0
  out.inspTotal = ins.total || 0
  out.inspName = ins.name || ''
  out.inspProgress = out.inspTotal ? (out.inspIndex + '/' + out.inspTotal) : '--'
  out.inspAttempt = ins.attempt || 0
  out.inspMaxAttempts = ins.max_attempts || 0
  out.inspAttemptText = out.inspMaxAttempts ? (out.inspAttempt + '/' + out.inspMaxAttempts) : '--'
  out.inspWaitedS = (ins.waited_s === undefined || ins.waited_s === null) ? null : ins.waited_s
  out.inspOk = ins.ok
  out.inspNote = ins.note || ''
  out.inspAborted = ins.aborted_reason || null
  out.reportFile = (typeof ins.report === 'string') ? ins.report : ''

  /* ---- 绕圆 ---- */
  const ci = d.circle || {}
  out.circleDefined = !!ci.defined
  out.circleExecuting = !!ci.executing
  out.circleExternal = !!ci.external_active
  const g = ci.geo
  if (g) {
    out.circle = { cx: g.cx, cy: g.cy, r: g.r }
    out.circleText = '圆心 (' + fmt.num(g.cx, 2) + ', ' + fmt.num(g.cy, 2) +
      ') · R = ' + fmt.num(g.r, 2) + ' m'
    out.circleR = fmt.num(g.r, 2)
  } else {
    out.circle = null
    out.circleText = '还没定义（先点圆心，再点圆周上一点）'
    out.circleR = '--'
  }
  out.circleStateText = out.circleExecuting
    ? (out.circleExternal ? '正在跑圆' : '执行中')
    : (out.circleDefined ? '已定义，未执行' : '未定义')

  /* ---- 照片 ---- */
  const ph = d.photos || {}
  out.photoCount = ph.count || 0
  out.latestPhoto = ph.latest || null
  out.latestPhotoUrl = ph.latest ? robot.getUrl('/photo', { name: ph.latest }, true) : ''

  /* ---- 轨迹 ---- */
  out.traj = (d.traj || []).filter(function (q) {
    return q && typeof q.x === 'number' && typeof q.y === 'number'
  })

  return out
}

/** 数据"指纹"：只有它变了才值得重绘 canvas / setData */
function fingerprint (view) {
  const last = view.traj && view.traj.length ? view.traj[view.traj.length - 1] : null
  const p = view.pose
  return [
    view.traj ? view.traj.length : 0,
    last ? last.x.toFixed(3) + ',' + last.y.toFixed(3) : '-',
    p ? p.x.toFixed(3) + ',' + p.y.toFixed(3) : '-',
    view.circle ? (view.circle.cx + ',' + view.circle.cy + ',' + view.circle.r) : '-',
    view.waypoints ? view.waypoints.length : 0,
    view.map ? (view.map.w + 'x' + view.map.h) : '-'
  ].join('|')
}

module.exports = {
  build: build,
  fingerprint: fingerprint,
  num2: num2
}
