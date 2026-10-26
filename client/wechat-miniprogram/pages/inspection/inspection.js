/**
 * inspection — 巡检页
 *
 * 对接（web_bridge）：
 *   POST /inspection/{start,stop,clear}   -> 转发到 inspection_route 的 Trigger 服务
 *   GET  /data                            -> inspection 字段（state/index/total/name/attempt…）
 *   GET  /photos                          -> 照片列表
 *   GET  /photo?name=                     -> 单张照片（<image> 直接用）
 *   GET  /report                          -> 最近一份报告（JSON）
 *
 * 状态字典见 utils/format.js 的 INSPECTION_STATES。
 */
const robot = require('../../utils/robot')
const view = require('../../utils/view')
const fmt = require('../../utils/format')

const PHOTO_POLL_MS = 3000

Page({
  data: {
    connected: false,
    baseUrl: '',
    // 状态
    inspText: '空闲', inspTone: 'idle', inspRunning: false,
    inspProgress: '--', inspName: '', inspAttemptText: '--',
    inspWaitedText: '', inspNote: '', inspSourceText: '--',
    inspOk: null, inspTotal: 0,
    // 航点
    wpCount: 0, wpSourceText: '--', wpFile: '',
    // 照片
    photos: [], photoCount: 0,
    // 报告
    report: null, reportRows: [], reportBusy: false, reportMsg: '',
    busy: ''
  },

  onLoad: function () {
    this._fails = 0
    this._pollBusy = false
    this._lastInspState = ''
  },

  onShow: function () {
    this.setData({ baseUrl: robot.getBaseUrl() })
    const cfg = robot.loadConfig()
    this._pollTimer = setInterval(this.poll.bind(this), cfg.pollInterval || 500)
    this._photoTimer = setInterval(this.pollPhotos.bind(this), PHOTO_POLL_MS)
    this.poll()
    this.pollPhotos()
    this.loadReport()
  },

  onHide: function () { this.clearTimers() },
  onUnload: function () { this.clearTimers() },

  clearTimers: function () {
    if (this._pollTimer) { clearInterval(this._pollTimer); this._pollTimer = null }
    if (this._photoTimer) { clearInterval(this._photoTimer); this._photoTimer = null }
  },

  /* ------------------------------------------------------------ 轮询 */

  poll: function () {
    if (this._pollBusy) return
    this._pollBusy = true
    const self = this
    robot.fetchData().then(function (d) {
      const v = view.build(d)
      self._fails = 0
      const waited = (v.inspWaitedS === null || v.inspWaitedS === undefined)
        ? ''
        : ('已等待 ' + fmt.duration(v.inspWaitedS))

      self.setData({
        connected: true,
        inspText: v.inspText, inspTone: v.inspTone, inspRunning: v.inspRunning,
        inspProgress: v.inspProgress, inspName: v.inspName,
        inspAttemptText: v.inspAttemptText,
        inspWaitedText: waited,
        inspNote: v.inspNote,
        wpCount: v.wpCount, wpSourceText: v.wpSourceText, wpFile: v.wpFile,
        photoCount: v.photoCount
      })

      // 一轮刚结束 -> 自动把新报告拉回来
      if (self._lastInspState !== 'finished' && v.inspState === 'finished') {
        self.loadReport()
        self.pollPhotos()
      }
      self._lastInspState = v.inspState
    }).catch(function () {
      self._fails++
      if (self._fails >= 3) self.setData({ connected: false })
    }).then(function () {
      self._pollBusy = false
    })
  },

  pollPhotos: function () {
    if (this._photoBusy) return
    this._photoBusy = true
    const self = this
    robot.fetchPhotos().then(function (res) {
      const list = (res.photos || []).map(function (p) {
        return {
          name: p.name,
          size: fmt.bytes(p.size),
          mtime: p.mtime,
          url: robot.getUrl('/photo', { name: p.name })
        }
      })
      self.setData({ photos: list, photoCount: list.length })
    }).catch(function () { /* 拉不到就保留上一次 */ }).then(function () {
      self._photoBusy = false
    })
  },

  loadReport: function () {
    const self = this
    this.setData({ reportBusy: true })
    robot.fetchReport().then(function (rep) {
      const rows = (rep.results || []).map(function (r) {
        const file = r.image ? String(r.image).split(/[\\/]/).pop() : ''
        return {
          index: r.index,
          name: r.name,
          ok: !!r.ok,
          mark: r.ok ? '成功' : '失败',
          status: r.status || '-',
          duration: fmt.duration(r.duration_s),
          attempts: r.attempts,
          obstacle: r.obstacle_waits ? (r.obstacle_waits + ' 次/' + fmt.duration(r.obstacle_wait_s)) : '',
          file: file
        }
      })
      self.setData({
        report: {
          file: rep._file || '',
          startedAt: rep.started_at || '--',
          finishedAt: rep.finished_at || '--',
          duration: fmt.duration(rep.duration_s),
          total: rep.total,
          succeeded: rep.succeeded,
          frame: rep.frame_id || '--',
          segmentMode: rep.segment_mode || '--',
          onObstacle: rep.on_obstacle || '--',
          onFailure: rep.on_failure || '--',
          aborted: rep.aborted_reason || '无',
          waitsTotal: rep.obstacle_waits_total || 0,
          waitsTime: fmt.duration(rep.obstacle_wait_s_total || 0),
          costmapClears: rep.costmap_clears_total || 0
        },
        reportRows: rows,
        reportBusy: false
      })
    }).catch(function (err) {
      self.setData({ report: null, reportRows: [], reportBusy: false, reportMsg: err.message || '还没有报告' })
    })
  },

  /* ------------------------------------------------------------ 动作 */

  onStart: function () {
    const self = this
    if (this.data.busy) return
    this.setData({ busy: 'start' })
    robot.inspection('start').then(function (res) {
      wx.showToast({ title: res.message || '已启动', icon: 'none', duration: 1500 })
      self.poll()
    }).catch(function (err) {
      // 最常见的两种：航点为空、已经在跑 —— 后端会把原因写在 message 里
      wx.showModal({
        title: '启动失败',
        content: err.message || '未知原因',
        showCancel: false
      })
    }).then(function () {
      self.setData({ busy: '' })
    })
  },

  onStop: function () {
    const self = this
    if (this.data.busy) return
    this.setData({ busy: 'stop' })
    robot.inspection('stop').then(function (res) {
      wx.showToast({ title: res.message || '已请求停止', icon: 'none', duration: 1500 })
      self.poll()
    }).catch(function (err) {
      wx.showToast({ title: err.message || '停止失败', icon: 'none', duration: 2000 })
    }).then(function () {
      self.setData({ busy: '' })
    })
  },

  onClear: function () {
    const self = this
    wx.showModal({
      title: '清空收集的航点？',
      content: '只清 collected_waypoints.yaml，配置文件（route_file）不受影响。',
      success: function (r) {
        if (!r.confirm) return
        self.setData({ busy: 'clear' })
        robot.inspection('clear').then(function (res) {
          wx.showToast({ title: res.message || '已清空', icon: 'none', duration: 1500 })
          self.poll()
        }).catch(function (err) {
          wx.showToast({ title: err.message || '清空失败', icon: 'none' })
        }).then(function () {
          self.setData({ busy: '' })
        })
      }
    })
  },

  /* ------------------------------------------------------------ 照片 */

  onPhotoTap: function (e) {
    const idx = Number(e.currentTarget.dataset.index)
    const urls = this.data.photos.map(function (p) { return p.url })
    if (!urls.length) return
    wx.previewImage({
      current: urls[idx] || urls[0],
      urls: urls
    })
  },

  onRefreshPhotos: function () {
    this.pollPhotos()
    wx.showToast({ title: '已刷新', icon: 'none', duration: 600 })
  },

  onRefreshReport: function () {
    this.loadReport()
  }
})
