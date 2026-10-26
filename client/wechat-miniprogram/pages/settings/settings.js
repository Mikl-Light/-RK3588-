/**
 * settings — 设置页
 *
 * 配置项（本地 storage）：
 *  - ip           机器人局域网 IP（Tailscale 地址也可以）
 *  - port         web_bridge 端口，默认 8765
 *  - pollInterval 遥测轮询间隔 ms
 *  - linearSpeed / rotationSpeed 默认速度（上限对齐后端限幅）
 */
const robot = require('../../utils/robot')
const view = require('../../utils/view')
const fmt = require('../../utils/format')

Page({
  data: {
    ip: '',
    port: '8765',
    pollInterval: '500',
    linearSpeed: '0.3',
    rotationSpeed: '1.0',
    limits: robot.LIMITS,
    testResult: '',
    testOk: false,
    busy: false
  },

  onLoad: function () {
    this.fill()
  },

  fill: function () {
    const cfg = robot.loadConfig()
    this.setData({
      ip: cfg.ip,
      port: String(cfg.port),
      pollInterval: String(cfg.pollInterval),
      linearSpeed: String(cfg.linearSpeed),
      rotationSpeed: String(cfg.rotationSpeed)
    })
  },

  onIpInput: function (e) { this.setData({ ip: e.detail.value }) },
  onPortInput: function (e) { this.setData({ port: e.detail.value }) },
  onPollInput: function (e) { this.setData({ pollInterval: e.detail.value }) },
  onLinearInput: function (e) { this.setData({ linearSpeed: e.detail.value }) },
  onRotationInput: function (e) { this.setData({ rotationSpeed: e.detail.value }) },

  validate: function () {
    const cfg = {
      ip: (this.data.ip || '').trim(),
      port: parseInt(this.data.port, 10),
      pollInterval: parseInt(this.data.pollInterval, 10),
      linearSpeed: parseFloat(this.data.linearSpeed),
      rotationSpeed: parseFloat(this.data.rotationSpeed)
    }
    const toast = function (t) { wx.showToast({ title: t, icon: 'none' }) }

    // 允许 IPv4 或主机名（Tailscale 的 100.x.x.x 也是 IPv4，能过）
    if (!/^[A-Za-z0-9][A-Za-z0-9._-]*$/.test(cfg.ip) &&
        !/^(\d{1,3}\.){3}\d{1,3}$/.test(cfg.ip)) {
      toast('IP/主机名格式不正确'); return null
    }
    if (!(cfg.port >= 1 && cfg.port <= 65535)) { toast('端口需在 1~65535'); return null }
    if (!(cfg.pollInterval >= 100 && cfg.pollInterval <= 10000)) {
      toast('轮询间隔需在 100~10000 ms'); return null
    }
    if (!(cfg.linearSpeed > 0 && cfg.linearSpeed <= robot.LIMITS.vx)) {
      toast('线速度需在 0~' + robot.LIMITS.vx + ' m/s'); return null
    }
    if (!(cfg.rotationSpeed > 0 && cfg.rotationSpeed <= robot.LIMITS.wz)) {
      toast('旋转速度需在 0~' + robot.LIMITS.wz + ' rad/s'); return null
    }
    return cfg
  },

  save: function (silent) {
    const cfg = this.validate()
    if (!cfg) return null
    robot.saveConfig(cfg)
    if (!silent) wx.showToast({ title: '已保存', icon: 'success' })
    return cfg
  },

  onSave: function () {
    this.save(false)
  },

  onReset: function () {
    robot.saveConfig(robot.DEFAULT_CONFIG)
    this.fill()
    wx.showToast({ title: '已恢复默认', icon: 'none' })
  },

  onTest: function () {
    const cfg = this.save(true)
    if (!cfg) return
    const self = this
    this.setData({ busy: true, testOk: false, testResult: '正在测试 ' + robot.getBaseUrl() + ' …' })
    robot.fetchData().then(function (d) {
      const v = view.build(d)
      const parts = [
        '连接成功',
        '底盘 ' + v.chassisText,
        '位姿 ' + (v.pose ? ('(' + v.x + ', ' + v.y + ') @ ' + v.headingText) : '无（TF/里程计都断）'),
        '地图 ' + (v.map ? v.mapText : '无'),
        '雷达 ' + (v.scanOk ? v.scanText : '无数据'),
        '相机 ' + (v.cameraOk ? '有画面' : '无画面'),
        '照片 ' + v.photoCount + ' 张',
        '航点 ' + v.wpCount + ' 个（' + v.wpSourceText + '）'
      ]
      self.setData({ busy: false, testOk: true, testResult: parts.join(' · ') })
    }).catch(function (err) {
      self.setData({
        busy: false,
        testOk: false,
        testResult: '连接失败：' + (err.message || '未知错误') +
          '\n排查：① 手机和机器人在同一网段（或都已连 Tailscale）' +
          '② 机器人上 `ros2 launch inspector_web web.launch.py` 是否在跑' +
          '③ 开发者工具「详情 → 本地设置 → 不校验合法域名」是否勾选'
      })
    })
  },

  /** 把桥自带的网页调试页地址复制到剪贴板，方便用电脑浏览器打开 */
  onCopyUrl: function () {
    const cfg = robot.loadConfig()
    const url = 'http://' + cfg.ip + ':' + cfg.port + '/'
    wx.setClipboardData({
      data: url,
      success: function () { wx.showToast({ title: '已复制 ' + url, icon: 'none', duration: 1500 }) }
    })
  },

  onClearCache: function () {
    try {
      wx.removeStorageSync(robot.STORAGE_KEY)
      wx.showToast({ title: '本地配置已清除', icon: 'none' })
      this.fill()
    } catch (e) {
      wx.showToast({ title: '清除失败', icon: 'none' })
    }
  }
})
