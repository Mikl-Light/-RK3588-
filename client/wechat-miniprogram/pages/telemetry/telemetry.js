/**
 * telemetry — 遥测页（地图 + 实时数据）
 *
 * 数据源：GET /data（轮询）
 * 画面：  GET /map.png（默认每 5s 重新拉一次，SLAM 长出新区域能跟上）
 *         GET /snapshot.jpg（可选，打开相机预览后才拉，省流量）
 *
 * 画布把「地图底图 + 轨迹 + 航点 + 当前位姿 + 圆形几何」画在同一套变换里，
 * 所以不会出现"图叠 canvas 对不齐"的问题。
 */
const robot = require('../../utils/robot')
const view = require('../../utils/view')
const fmt = require('../../utils/format')
const mapview = require('../../utils/mapview')

Page({
  data: {
    connected: false,
    baseUrl: '',
    // 位姿
    x: '--', y: '--', headingText: '--', sector: '--',
    poseSource: '--',
    vx: '--', vy: '--', wz: '--',
    // 环境
    scanText: '--', scanOk: false,
    chassisText: '未知',
    cameraText: '--', cameraOk: false,
    mapText: '--', mapOk: false,
    wpText: '--', photoText: '--',
    circleText: '--', circleStateText: '--',
    inspText: '空闲', inspTone: 'idle',
    trajText: '0 点',
    // 相机预览开关
    showCamera: false,
    camUrl: ''
  },

  onLoad: function () {
    this._lastFp = ''
    this._fails = 0
    this._pollBusy = false
    this._view = null
  },

  onReady: function () {
    const self = this
    // canvas 2d 需要节点渲染完才能拿到节点句柄，所以放在 onReady
    const api = mapview.attach(this, '#map', { imageInterval: 5000 })
    this._map = api
    api.init().then(function () {
      if (!api.ready) {
        self.setData({ mapText: '画布初始化失败：' + (api.error || '未知原因') })
        return
      }
      if (self._view) self.refreshMap()      // onShow 可能已经先跑到了一次
    })
  },

  onShow: function () {
    this.setData({ baseUrl: robot.getBaseUrl() })
    const cfg = robot.loadConfig()
    const iv = cfg.pollInterval || 500
    this._pollTimer = setInterval(this.poll.bind(this), iv)
    // 底图定期刷新（画布就绪后才真的下载）
    this._mapTimer = setInterval(this.refreshMap.bind(this), 5000)
    this.poll()
  },

  onHide: function () {
    this.clearTimers()
  },

  onUnload: function () {
    this.clearTimers()
  },

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
        x: v.x, y: v.y,
        headingText: v.headingText, sector: v.sector,
        poseSource: v.poseSource,
        vx: v.vx, vy: v.vy, wz: v.wz,
        scanText: v.scanText, scanOk: v.scanOk,
        chassisText: v.chassisText,
        cameraText: v.cameraText, cameraOk: v.cameraOk,
        mapText: v.mapText, mapOk: !!v.map,
        wpText: v.wpCount + ' 个 · ' + v.wpSourceText,
        photoText: v.photoCount + ' 张',
        circleText: v.circleText, circleStateText: v.circleStateText,
        inspText: v.inspText, inspTone: v.inspTone,
        trajText: (v.traj ? v.traj.length : 0) + ' 点',
        camUrl: self.data.showCamera
          ? robot.getUrl('/snapshot.jpg', null, true)
          : self.data.camUrl
      })

      // 只有几何真的变了才重绘（轮询 2 Hz，重绘没必要跟着跑）
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

  /* ------------------------------------------------------------ 相机 */

  onToggleCamera: function (e) {
    const on = e.detail.value
    this.setData({
      showCamera: on,
      camUrl: on ? robot.getUrl('/snapshot.jpg', null, true) : ''
    })
    if (on) this.startCamTimer()
    else this.stopCamTimer()
  },

  startCamTimer: function () {
    this.stopCamTimer()
    const self = this
    // 相机 15 fps，手机上 1.5s 一帧足够"是不是对着要看的地方"
    this._camTimer = setInterval(function () {
      self.setData({ camUrl: robot.getUrl('/snapshot.jpg', null, true) })
    }, 1500)
  },

  stopCamTimer: function () {
    if (this._camTimer) { clearInterval(this._camTimer); this._camTimer = null }
  },

  onCamError: function () {
    if (this.data.showCamera) {
      this.setData({ showCamera: false, cameraText: '取不到画面（相机没起？start_camera=false）' })
      this.stopCamTimer()
    }
  },

  onReset: function () {
    this._lastFp = ''
    this.refreshMap()
    this.poll()
  }
})
