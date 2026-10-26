/**
 * control — 遥控页
 *
 * 对接：POST /cmd_vel {vx,vy,wz}（web_bridge 内部限幅 0.50 / 0.50 / 1.50）
 *
 * 交互：
 *  - 虚拟摇杆：上推前进(Vx+)、下推后退、左推左转(Wz+)、右推右转
 *  - 左移 / 右移：按住持续横移（Vy±），松开即停
 *  - 点动按钮：按一下走一小段（默认 800ms）后自动停
 *  - STOP：急停（同时通知 chassis_bridge）
 *  - 速度滑杆：上限 = 桥的限幅，滑杆给不出后端不执行的数
 *
 * 安全策略：
 *  - 有速度时每 300ms 保活重发（桥的看门狗 0.8s，留足余量）
 *  - 离开页面 / 连续请求失败 → 急停
 *  - 指令节流 100ms + 相同指令去重，避免摇杆事件刷爆局域网
 */
const robot = require('../../utils/robot')
const view = require('../../utils/view')
const fmt = require('../../utils/format')

const SEND_THROTTLE = 100     // 指令节流 (ms)
const KEEPALIVE_MS = 300      // 运动中保活重发间隔
const PULSE_MS = 800          // 点动持续时长
const FAIL_LIMIT = 3          // 连续失败多少次判为断线

Page({
  data: {
    connected: false,
    baseUrl: '',
    limits: robot.LIMITS,
    pulseMs: PULSE_MS,
    speed: 0.30,               // 线速度 (m/s)
    rotationSpeed: 1.00,       // 旋转速度 (rad/s)
    // 遥测摘要
    x: '--', y: '--', headingText: '--', sector: '--',
    vx: '--', vy: '--', wz: '--',
    chassisText: '未知',
    inspRunning: false, inspText: '空闲',
    // 自定义命令
    rawCmd: '',
    rawHint: ''
  },

  onLoad: function () {
    const cfg = robot.loadConfig()
    this.setData({
      baseUrl: robot.getBaseUrl(),
      speed: Math.min(cfg.linearSpeed || 0.30, robot.LIMITS.vx),
      rotationSpeed: Math.min(cfg.rotationSpeed || 1.00, robot.LIMITS.wz)
    })
    this._joy = { x: 0, y: 0 }
    this._vy = 0
    this._pulse = null
    this._current = null
    this._lastSent = ''
    this._lastSendAt = 0
    this._fails = 0
    this._sendBusy = false
    this._shiftTimer = null
    this._pulseTimer = null
    this._keepTimer = null
    this._pollTimer = null
  },

  onShow: function () {
    this.setData({ baseUrl: robot.getBaseUrl() })
    const cfg = robot.loadConfig()
    this._pollTimer = setInterval(this.pollTelemetry.bind(this), cfg.pollInterval || 500)
    this._keepTimer = setInterval(this.keepAlive.bind(this), KEEPALIVE_MS)
    this.pollTelemetry()
  },

  onHide: function () {
    this.emergencyStop()
    this.clearTimers()
  },

  onUnload: function () {
    this.emergencyStop()
    this.clearTimers()
  },

  clearTimers: function () {
    if (this._pollTimer) { clearInterval(this._pollTimer); this._pollTimer = null }
    if (this._keepTimer) { clearInterval(this._keepTimer); this._keepTimer = null }
    if (this._shiftTimer) { clearInterval(this._shiftTimer); this._shiftTimer = null }
    if (this._pulseTimer) { clearTimeout(this._pulseTimer); this._pulseTimer = null }
  },

  /* ------------------------------------------------------------ 摇杆 */

  onJoyMove: function (e) {
    this._joy.x = e.detail.x || 0
    this._joy.y = e.detail.y || 0
    this.sendCurrent()
  },

  onJoyEnd: function () {
    this._joy.x = 0
    this._joy.y = 0
    this.sendCurrent(true)
  },

  /* ------------------------------------------------ 横移按钮（按住走） */

  onShiftStart: function (e) {
    const dir = Number(e.currentTarget.dataset.dir) || 1   // 1=左移(Vy+), -1=右移(Vy-)
    this._vy = dir * this.data.speed
    this.sendCurrent(true)
    if (this._shiftTimer) clearInterval(this._shiftTimer)
    this._shiftTimer = setInterval(this.sendCurrent.bind(this, true), 150)
  },

  onShiftEnd: function () {
    if (this._shiftTimer) { clearInterval(this._shiftTimer); this._shiftTimer = null }
    this._vy = 0
    this.sendCurrent(true)
  },

  /* ------------------------------------------------------ 点动按钮 */

  onPulse: function (e) {
    const dir = e.currentTarget.dataset.dir
    const spd = this.data.speed
    const rot = this.data.rotationSpeed
    const table = {
      fwd: { vx: spd, vy: 0, wz: 0 },
      back: { vx: -spd, vy: 0, wz: 0 },
      left: { vx: 0, vy: spd, wz: 0 },
      right: { vx: 0, vy: -spd, wz: 0 },
      ccw: { vx: 0, vy: 0, wz: rot },
      cw: { vx: 0, vy: 0, wz: -rot }
    }
    const mv = table[dir]
    if (!mv) return
    this._pulse = mv
    if (this._pulseTimer) clearTimeout(this._pulseTimer)
    this.sendCurrent(true)
    const self = this
    this._pulseTimer = setTimeout(function () {
      self._pulse = null
      self.sendCurrent(true)
    }, PULSE_MS)
  },

  /* ------------------------------------------------------------ 发送 */

  /** 按当前（点动 | 摇杆 | 横移）状态组包并发送，带节流与去重 */
  sendCurrent: function (force) {
    const now = Date.now()
    if (!force && now - this._lastSendAt < SEND_THROTTLE) return

    const spd = this.data.speed
    const rot = this.data.rotationSpeed
    let vx, vy, wz
    if (this._pulse) {
      vx = this._pulse.vx
      vy = this._pulse.vy
      wz = this._pulse.wz
    } else {
      vx = this._joy.y * spd
      wz = -this._joy.x * rot
      vy = this._vy
    }
    vx = robot.clamp(vx, robot.LIMITS.vx)
    vy = robot.clamp(vy, robot.LIMITS.vy)
    wz = robot.clamp(wz, robot.LIMITS.wz)

    const key = vx.toFixed(3) + ',' + vy.toFixed(3) + ',' + wz.toFixed(3)
    const zero = (Math.abs(vx) + Math.abs(vy) + Math.abs(wz)) < 1e-6
    if (!force && key === this._lastSent && !zero) return   // 值没变就不重发（零速仍要发，见下）

    this._lastSendAt = now
    this._lastSent = key
    this._current = zero ? null : { vx: vx, vy: vy, wz: wz }
    this._post(vx, vy, wz)
  },

  /** 真正发出去；同一时刻只允许一个在飞，避免请求堆积 */
  _post: function (vx, vy, wz) {
    if (this._sendBusy) {
      this._pending = { vx: vx, vy: vy, wz: wz }
      return
    }
    this._sendBusy = true
    const self = this
    robot.sendMove(vx, vy, wz).then(function () {
      self._fails = 0
      if (!self.data.connected) self.setData({ connected: true })
    }).catch(function () {
      self._fails++
      if (self._fails >= FAIL_LIMIT && self.data.connected) {
        self.setData({ connected: false })
      }
    }).then(function () {
      self._sendBusy = false
      if (self._pending) {
        const p = self._pending
        self._pending = null
        self._post(p.vx, p.vy, p.wz)
      }
    })
  },

  /** 运动中保活：有目标速度就重发，避免断连后车一直跑 */
  keepAlive: function () {
    const cur = this._current
    if (cur) this._post(cur.vx, cur.vy, cur.wz)
  },

  /** 急停 */
  emergencyStop: function () {
    this._joy = { x: 0, y: 0 }
    this._vy = 0
    this._pulse = null
    this._current = null
    this._lastSent = ''
    if (this._pulseTimer) { clearTimeout(this._pulseTimer); this._pulseTimer = null }
    this._post(0, 0, 0)
  },

  onStop: function () {
    this.emergencyStop()
    if (this._shiftTimer) { clearInterval(this._shiftTimer); this._shiftTimer = null }
    try { wx.vibrateShort({ type: 'medium' }) } catch (e) { /* 部分机型不支持 */ }
    wx.showToast({ title: '已急停', icon: 'none', duration: 800 })
  },

  /* ---------------------------------------------------------- 速度 */

  onSpeedChange: function (e) {
    const v = Number(e.detail.value)
    this.setData({ speed: v })
    const cfg = robot.loadConfig()
    cfg.linearSpeed = v
    robot.saveConfig(cfg)
    this.sendCurrent(true)
  },

  onRotationChange: function (e) {
    const v = Number(e.detail.value)
    this.setData({ rotationSpeed: v })
    const cfg = robot.loadConfig()
    cfg.rotationSpeed = v
    robot.saveConfig(cfg)
    this.sendCurrent(true)
  },

  /* ------------------------------------------------ 自定义 / 快捷命令 */

  onRawInput: function (e) {
    this.setData({ rawCmd: e.detail.value, rawHint: '' })
  },

  onRawSend: function () {
    const cmd = (this.data.rawCmd || '').trim()
    if (!cmd) return
    const self = this
    // 本地先拦一道：本桥没有串口，只认 V 与 STOP，早点告诉用户别白等
    const head = cmd.split(',')[0].trim().toUpperCase()
    if (head !== 'V' && head !== 'STOP') {
      this.setData({
        rawHint: '本桥只支持 V,<vx>,<vy>,<wz> 与 STOP（它是 ROS↔HTTP 翻译层，没有串口）'
      })
      return
    }
    robot.sendCmd(cmd).then(function (res) {
      self.setData({ rawHint: res && res.message ? res.message : '已发送' })
      self._fails = 0
      if (!self.data.connected) self.setData({ connected: true })
    }).catch(function (err) {
      self._fails++
      self.setData({ rawHint: '失败：' + (err.message || '未知错误') })
      if (self._fails >= FAIL_LIMIT) self.setData({ connected: false })
    })
  },

  onGotoInspection: function () {
    wx.switchTab({ url: '/pages/inspection/inspection' })
  },

  /* ---------------------------------------------------------- 遥测 */

  pollTelemetry: function () {
    const self = this
    if (this._pollBusy) return
    this._pollBusy = true
    robot.fetchData().then(function (d) {
      const v = view.build(d)
      // 命令被限幅后，用后端真正采用的值回显，用户能看到"我发的到底是多少"
      self._fails = 0
      self.setData({
        connected: true,
        x: v.x, y: v.y, headingText: v.headingText, sector: v.sector,
        vx: v.vx, vy: v.vy, wz: v.wz,
        chassisText: v.chassisText,
        inspRunning: v.inspRunning, inspText: v.inspText
      })
    }).catch(function () {
      self._fails++
      if (self._fails >= FAIL_LIMIT) self.setData({ connected: false })
    }).then(function () {
      self._pollBusy = false
    })
  },

  onShareAppMessage: function () {
    return { title: '麦轮巡检车 · 遥控', path: '/pages/control/control' }
  }
})
