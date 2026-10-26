/**
 * circle — 绕圆页
 *
 * path_follower 的协议就是 RViz 上「Publish Point」的顺序：
 *   第 1 个点 = 圆心，第 2 个点 = 圆周上一点。
 * 手机端做的是同一件事 —— 把点 POST 到 /circle/point，几何由后端算，
 * 算好后通过 /data 的 circle.geo 回显（所以这里不自己算半径，避免两套真相）。
 *
 * 两种给点方式：
 *   ① 直接点地图（推荐，所见即所得）
 *   ② 手输 x/y，或"用当前位姿"
 */
const robot = require('../../utils/robot')
const view = require('../../utils/view')
const fmt = require('../../utils/format')
const mapview = require('../../utils/mapview')

Page({
  data: {
    connected: false,
    baseUrl: '',
    circleText: '--', circleR: '--',
    circleStateText: '未定义',
    circleDefined: false, circleExecuting: false, circleExternal: false,
    poseText: '--',
    manualX: '', manualY: '',
    sent: [],
    tapHint: '在地图上点两下：第 1 下 = 圆心，第 2 下 = 圆周上一点',
    poseSource: '--',
    busy: ''
  },

  onLoad: function () {
    this._fails = 0
    this._pollBusy = false
    this._lastFp = ''
    this._view = null
    this._sent = []
  },

  onReady: function () {
    const self = this
    const api = mapview.attach(this, '#map', { imageInterval: 5000 })
    this._map = api
    api.init().then(function () {
      if (!api.ready) {
        self.setData({ tapHint: '画布初始化失败：' + (api.error || '未知原因') })
        return
      }
      self.measure()
    })
  },

  measure: function () {
    const self = this
    this.createSelectorQuery().select('#map').boundingClientRect(function (r) {
      if (r) self._rect = r
    }).exec()
  },

  onShow: function () {
    this.setData({ baseUrl: robot.getBaseUrl() })
    const cfg = robot.loadConfig()
    this._pollTimer = setInterval(this.poll.bind(this), cfg.pollInterval || 500)
    this._mapTimer = setInterval(this.refreshMap.bind(this), 5000)
    this.poll()
    this.measure()
  },

  onHide: function () { this.clearTimers() },
  onUnload: function () { this.clearTimers() },

  clearTimers: function () {
    if (this._pollTimer) { clearInterval(this._pollTimer); this._pollTimer = null }
    if (this._mapTimer) { clearInterval(this._mapTimer); this._mapTimer = null }
  },

  refreshMap: function () {
    if (this._map && this._map.ready) this._map.refreshImage()
  },

  /* ------------------------------------------------------------ 轮询 */

  poll: function () {
    if (this._pollBusy) return
    this._pollBusy = true
    const self = this
    robot.fetchData().then(function (d) {
      const v = view.build(d)
      self._view = v
      self._fails = 0

      if (self._map && self._map.ready) {
        self._map.setMeta(v.map)
        self._map.refreshImage()
      }

      self.setData({
        connected: true,
        circleText: v.circleText,
        circleR: v.circleR,
        circleStateText: v.circleStateText,
        circleDefined: v.circleDefined,
        circleExecuting: v.circleExecuting,
        circleExternal: v.circleExternal,
        poseText: v.pose ? ('(' + v.x + ', ' + v.y + ')') : '无位姿',
        poseSource: v.poseSource
      })

      const fp = view.fingerprint(v)
      if (fp !== self._lastFp) {
        self._lastFp = fp
        if (self._map && self._map.ready) {
          self._map.render({
            traj: v.traj,
            pose: v.pose,
            waypoints: v.waypoints,
            circle: v.circle
          })
        }
      }
    }).catch(function () {
      self._fails++
      if (self._fails >= 3) self.setData({ connected: false })
    }).then(function () {
      self._pollBusy = false
    })
  },

  /* ------------------------------------------------------------ 给点 */

  onMapTap: function (e) {
    const t = (e.changedTouches && e.changedTouches[0]) || (e.touches && e.touches[0])
    if (!t) return
    let u = null
    let v = null
    const rect = this._rect
    // CanvasTouch.x/y 就是画布内坐标；没有才退回 clientX/Y 减矩形
    if (typeof t.x === 'number' && typeof t.y === 'number' &&
        (!rect || (t.x >= -2 && t.x <= rect.width + 2))) {
      u = t.x
      v = t.y
    } else if (typeof t.clientX === 'number' && rect) {
      u = t.clientX - rect.left
      v = t.clientY - rect.top
    }
    if (u === null) return
    const w = this._map ? this._map.toWorld(u, v) : null
    if (!w) {
      wx.showToast({ title: '地图还没就绪', icon: 'none' })
      return
    }
    this.sendPoint(w.x, w.y)
  },

  onManualX: function (e) { this.setData({ manualX: e.detail.value }) },
  onManualY: function (e) { this.setData({ manualY: e.detail.value }) },

  onSendManual: function () {
    const x = parseFloat(this.data.manualX)
    const y = parseFloat(this.data.manualY)
    if (!isFinite(x) || !isFinite(y)) {
      wx.showToast({ title: 'x / y 要填数字', icon: 'none' })
      return
    }
    this.sendPoint(x, y)
  },

  onUsePose: function () {
    const v = this._view
    if (!v || !v.pose) {
      wx.showModal({
        title: '拿不到当前位姿',
        content: '模型里 pose 为空：TF（map→base_link）和轮式里程计都没有数据。\n' +
          '常见原因：rf2o 没输出（雷达没出扫描）/ SLAM 没起。',
        showCancel: false
      })
      return
    }
    this.sendPoint(v.pose.x, v.pose.y)
  },

  sendPoint: function (x, y) {
    const self = this
    if (this.data.busy) return
    this.setData({ busy: 'point' })
    robot.circlePoint(x.toFixed(3), y.toFixed(3)).then(function (res) {
      const sent = self._sent.concat([fmt.num(x, 2) + ', ' + fmt.num(y, 2)])
      if (sent.length > 4) sent.shift()
      self._sent = sent
      self.setData({
        sent: sent,
        tapHint: sent.length === 1
          ? '已给圆心，再点一下圆周上一点即可成圆'
          : '已给 ' + sent.length + ' 个点（每次成圆都用最近两个）'
      })
      wx.showToast({ title: '已发送点', icon: 'none', duration: 700 })
      setTimeout(function () { self.poll() }, 300)
    }).catch(function (err) {
      wx.showModal({ title: '发送点失败', content: err.message || '未知原因', showCancel: false })
    }).then(function () {
      self.setData({ busy: '' })
    })
  },

  /* ------------------------------------------------------------ 动作 */

  onStart: function () {
    this.act('start', '开始走圆', function (res) { return res.message || '已开始' })
  },

  onStop: function () {
    this.act('stop', '停止', function (res) { return res.message || '已请求停止' })
  },

  onClear: function () {
    const self = this
    wx.showModal({
      title: '清除圆周？',
      content: '会清掉 path_follower 已定义的圆心/半径（不会动车）。',
      success: function (r) {
        if (r.confirm) self.act('clear', '清除', null)
      }
    })
  },

  act: function (action, title, msgFn) {
    const self = this
    if (this.data.busy) return
    this.setData({ busy: action })
    robot.circle(action).then(function (res) {
      wx.showToast({
        title: (msgFn ? msgFn(res) : (res.message || '完成')),
        icon: 'none',
        duration: 1500
      })
      self._sent = []
      self.setData({ sent: [], tapHint: '在地图上点两下：第 1 下 = 圆心，第 2 下 = 圆周上一点' })
      setTimeout(function () { self.poll() }, 300)
    }).catch(function (err) {
      wx.showModal({ title: title + '失败', content: err.message || '未知原因', showCancel: false })
    }).then(function () {
      self.setData({ busy: '' })
    })
  }
})
