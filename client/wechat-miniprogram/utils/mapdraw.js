/**
 * mapdraw.js — 地图画布渲染器（canvas 2d）
 *
 * 一个画布同时画：地图底图（/map.png）+ 轨迹 + 航点 + 当前位姿 + 圆形几何。
 * 之所以自己画底图而不是「<image> 叠一个 canvas」，是因为叠加对齐一旦有几像素
 * 误差，用户就会以为车跑偏了；统一在一个 canvas 里用同一套变换绘制最可靠。
 *
 * 坐标系（两者都是 ROS 右手系：x 右、y 上）：
 *   世界 (x, y)  <-变换->  画布 (u, v)
 *   u = offX + (x - x0) * s
 *   v = H - offY - (y - y0) * s
 * 地图底图是 OccupancyGrid（原点在左下、y 向上），web_bridge 渲染 PNG 时做过
 * flipud，所以 PNG 的第 0 行是世界的最大 y —— 上面这个式子正好对上。
 */

/** 一组好看的世界网格步长 */
const GRID_STEPS = [0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20]

function pickGrid (scale) {
  for (let i = 0; i < GRID_STEPS.length; i++) {
    if (GRID_STEPS[i] * scale >= 44) return GRID_STEPS[i]
  }
  return GRID_STEPS[GRID_STEPS.length - 1]
}

/**
 * 初始化一个 type="2d" 的 canvas。
 * @param {object} holder 页面/组件实例（this）
 * @param {string} selector 如 '#map'
 * @returns {Promise<{canvas, ctx, width, height, dpr}>}
 */
function initCanvas (holder, selector) {
  return new Promise(function (resolve, reject) {
    // 页面/组件自带 createSelectorQuery()，作用域天然正确；
    // 只有拿不到时才退回 wx.createSelectorQuery().in(holder)
    const query = (holder && typeof holder.createSelectorQuery === 'function')
      ? holder.createSelectorQuery()
      : wx.createSelectorQuery().in(holder)
    query.select(selector)
      .fields({ node: true, size: true })
      .exec(function (res) {
        const info = res && res[0]
        if (!info || !info.node) {
          reject(new Error('找不到画布 ' + selector))
          return
        }
        const canvas = info.node
        const ctx = canvas.getContext('2d')
        let dpr = 2
        try {
          dpr = (wx.getWindowInfo ? wx.getWindowInfo().pixelRatio
            : wx.getSystemInfoSync().pixelRatio) || 2
        } catch (e) { /* 用默认值 */ }
        canvas.width = Math.max(1, Math.round(info.width * dpr))
        canvas.height = Math.max(1, Math.round(info.height * dpr))
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
        resolve({
          canvas: canvas,
          ctx: ctx,
          width: info.width,
          height: info.height,
          dpr: dpr
        })
      })
  })
}

/**
 * 创建渲染器。
 * @param {object} c  initCanvas 的返回值
 * @param {object} o  选项 { theme: 'light'|'dark' }
 */
function createRenderer (c, o) {
  const opt = o || {}
  const theme = opt.theme === 'dark' ? 'dark' : 'light'
  const COLORS = theme === 'dark'
    ? {
      bg: '#111827', grid: '#1f2937', gridText: '#6b7280',
      traj: '#60a5fa', start: '#34d399', pose: '#f87171',
      wp: '#fbbf24', wpText: '#fde68a', circle: '#a78bfa', text: '#9ca3af'
    }
    : {
      bg: '#ffffff', grid: '#eceff3', gridText: '#9ca3af',
      traj: '#2563eb', start: '#16a34a', pose: '#dc2626',
      wp: '#f59e0b', wpText: '#b45309', circle: '#7c3aed', text: '#6b7280'
    }

  const W = c.width
  const H = c.height
  let img = null
  let info = null
  const tf = { s: 1, x0: 0, y0: 0, offX: 0, offY: 0, valid: false }

  function u (x) { return tf.offX + (x - tf.x0) * tf.s }
  function v (y) { return H - tf.offY - (y - tf.y0) * tf.s }

  /** 反向：画布像素 -> 世界坐标（点选地图时用） */
  function toWorld (pu, pv) {
    if (!tf.valid) return null
    return {
      x: tf.x0 + (pu - tf.offX) / tf.s,
      y: tf.y0 + (H - tf.offY - pv) / tf.s
    }
  }

  function setInfo (m) {
    info = (m && m.w && m.h) ? m : null
  }

  function setImage (image) {
    img = image || null
  }

  function computeTransform (state) {
    let x0, x1, y0, y1
    if (info) {
      x0 = info.ox
      y0 = info.oy
      x1 = info.ox + info.w * info.res
      y1 = info.oy + info.h * info.res
    } else {
      // 没有地图就按叠加数据的包围盒自适应（至少 0.6 m 见方）
      const pts = []
      if (state && state.traj) pts.push.apply(pts, state.traj)
      if (state && state.pose) pts.push(state.pose)
      if (state && state.waypoints) {
        state.waypoints.forEach(function (p) { pts.push({ x: p[0], y: p[1] }) })
      }
      if (state && state.circle && state.circle.cx !== undefined) {
        pts.push({ x: state.circle.cx, y: state.circle.cy })
      }
      if (!pts.length) {
        tf.valid = false
        return false
      }
      x0 = x1 = pts[0].x
      y0 = y1 = pts[0].y
      pts.forEach(function (p) {
        if (p.x < x0) x0 = p.x; if (p.x > x1) x1 = p.x
        if (p.y < y0) y0 = p.y; if (p.y > y1) y1 = p.y
      })
      if (x1 - x0 < 0.6) { const m = (x0 + x1) / 2; x0 = m - 0.3; x1 = m + 0.3 }
      if (y1 - y0 < 0.6) { const m = (y0 + y1) / 2; y0 = m - 0.3; y1 = m + 0.3 }
    }

    const pad = 14
    const s = Math.min((W - pad * 2) / (x1 - x0), (H - pad * 2) / (y1 - y0))
    tf.s = s
    tf.x0 = x0
    tf.y0 = y0
    tf.offX = pad + (W - pad * 2 - (x1 - x0) * s) / 2
    tf.offY = pad + (H - pad * 2 - (y1 - y0) * s) / 2
    tf.valid = true
    return true
  }

  function drawGrid () {
    const step = pickGrid(tf.s)
    const x1 = tf.x0 + (W - tf.offX * 2) / tf.s
    const y1 = tf.y0 + (H - tf.offY * 2) / tf.s
    c.ctx.save()
    c.ctx.strokeStyle = COLORS.grid
    c.ctx.lineWidth = 1
    c.ctx.fillStyle = COLORS.gridText
    c.ctx.font = '10px sans-serif'
    for (let gx = Math.ceil(tf.x0 / step) * step; gx <= x1; gx += step) {
      const px = Math.round(u(gx)) + 0.5
      c.ctx.beginPath(); c.ctx.moveTo(px, 0); c.ctx.lineTo(px, H); c.ctx.stroke()
      c.ctx.fillText(gx.toFixed(step < 1 ? 1 : 0), px + 2, 11)
    }
    for (let gy = Math.ceil(tf.y0 / step) * step; gy <= y1; gy += step) {
      const py = Math.round(v(gy)) + 0.5
      c.ctx.beginPath(); c.ctx.moveTo(0, py); c.ctx.lineTo(W, py); c.ctx.stroke()
      c.ctx.fillText(gy.toFixed(step < 1 ? 1 : 0), 2, py - 2)
    }
    c.ctx.restore()
  }

  function drawMapImage () {
    if (!img || !info) return
    const dw = info.w * info.res * tf.s
    const dh = info.h * info.res * tf.s
    const left = u(info.ox)
    const top = v(info.oy + info.h * info.res)
    c.ctx.save()
    c.ctx.imageSmoothingEnabled = false
    try {
      c.ctx.drawImage(img, left, top, dw, dh)
    } catch (e) { /* 图还没解码好就跳过这一帧 */ }
    c.ctx.restore()
  }

  function line (pts, color, width, close) {
    if (!pts || pts.length < 1) return
    c.ctx.save()
    c.ctx.strokeStyle = color
    c.ctx.lineWidth = width
    c.ctx.lineCap = 'round'
    c.ctx.lineJoin = 'round'
    c.ctx.beginPath()
    pts.forEach(function (p, i) {
      const px = u(p.x)
      const py = v(p.y)
      if (i === 0) c.ctx.moveTo(px, py)
      else c.ctx.lineTo(px, py)
    })
    if (close) c.ctx.closePath()
    c.ctx.stroke()
    c.ctx.restore()
  }

  function dot (x, y, r, color, stroke) {
    c.ctx.save()
    c.ctx.beginPath()
    c.ctx.arc(u(x), v(y), r, 0, Math.PI * 2)
    c.ctx.fillStyle = color
    c.ctx.fill()
    if (stroke) {
      c.ctx.lineWidth = 1.5
      c.ctx.strokeStyle = stroke
      c.ctx.stroke()
    }
    c.ctx.restore()
  }

  function drawCircle (geo) {
    if (!geo || geo.cx === undefined || !(geo.r > 0)) return
    const cx = u(geo.cx)
    const cy = v(geo.cy)
    const r = geo.r * tf.s
    c.ctx.save()
    c.ctx.strokeStyle = COLORS.circle
    c.ctx.lineWidth = 2
    c.ctx.setLineDash([6, 4])
    c.ctx.beginPath()
    c.ctx.arc(cx, cy, r, 0, Math.PI * 2)
    c.ctx.stroke()
    c.ctx.setLineDash([])
    // 圆心十字
    c.ctx.beginPath()
    c.ctx.moveTo(cx - 6, cy); c.ctx.lineTo(cx + 6, cy)
    c.ctx.moveTo(cx, cy - 6); c.ctx.lineTo(cx, cy + 6)
    c.ctx.stroke()
    c.ctx.restore()
  }

  function drawPose (pose) {
    if (!pose || pose.x === undefined || pose.x === null) return
    const px = u(pose.x)
    const py = v(pose.y)
    if (typeof pose.yaw === 'number' && isFinite(pose.yaw)) {
      const L = 22
      const ax = px + L * Math.cos(pose.yaw)
      const ay = py - L * Math.sin(pose.yaw)
      c.ctx.save()
      c.ctx.strokeStyle = COLORS.pose
      c.ctx.lineWidth = 2.5
      c.ctx.beginPath()
      c.ctx.moveTo(px, py)
      c.ctx.lineTo(ax, ay)
      c.ctx.stroke()
      c.ctx.restore()
    }
    dot(pose.x, pose.y, 5, COLORS.pose, '#ffffff')
  }

  function drawWaypoints (wps) {
    if (!wps || !wps.length) return
    c.ctx.save()
    c.ctx.font = '10px sans-serif'
    c.ctx.fillStyle = COLORS.wpText
    wps.forEach(function (p, i) {
      dot(p[0], p[1], 4, COLORS.wp, '#ffffff')
      c.ctx.fillText(String(i + 1), u(p[0]) + 5, v(p[1]) - 4)
    })
    c.ctx.restore()
  }

  function drawScaleBar () {
    if (!tf.valid) return
    const target = 90
    let m = 1
    for (let i = 0; i < GRID_STEPS.length; i++) {
      if (GRID_STEPS[i] * tf.s >= target) { m = GRID_STEPS[i]; break }
    }
    const px = m * tf.s
    const x0 = W - 12 - px
    const y0 = H - 14
    c.ctx.save()
    c.ctx.strokeStyle = COLORS.text
    c.ctx.fillStyle = COLORS.text
    c.ctx.lineWidth = 2
    c.ctx.beginPath()
    c.ctx.moveTo(x0, y0 - 4); c.ctx.lineTo(x0, y0); c.ctx.lineTo(x0 + px, y0)
    c.ctx.lineTo(x0 + px, y0 - 4)
    c.ctx.stroke()
    c.ctx.font = '10px sans-serif'
    c.ctx.fillText(m + ' m', x0 + px / 2 - 10, y0 - 6)
    c.ctx.restore()
  }

  /**
   * 渲染一帧。
   * @param {object} state {traj, pose, waypoints, circle:{cx,cy,r}}
   */
  function render (state) {
    const st = state || {}
    c.ctx.save()
    c.ctx.fillStyle = COLORS.bg
    c.ctx.fillRect(0, 0, W, H)
    c.ctx.restore()

    if (!computeTransform(st)) {
      c.ctx.save()
      c.ctx.fillStyle = COLORS.text
      c.ctx.font = '13px sans-serif'
      c.ctx.fillText('暂无数据', 14, 24)
      c.ctx.restore()
      return
    }

    drawGrid()
    drawMapImage()

    if (st.traj && st.traj.length > 1) {
      line(st.traj, COLORS.traj, 2.5, false)
      const s0 = st.traj[0]
      dot(s0.x, s0.y, 4, COLORS.start, '#ffffff')
    }
    drawWaypoints(st.waypoints)
    drawCircle(st.circle)
    drawPose(st.pose)
    drawScaleBar()
  }

  return {
    render: render,
    toWorld: toWorld,
    setInfo: setInfo,
    setImage: setImage,
    hasTransform: function () { return tf.valid },
    colors: COLORS
  }
}

module.exports = {
  initCanvas: initCanvas,
  createRenderer: createRenderer
}
