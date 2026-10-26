/**
 * joystick — 虚拟摇杆组件
 *
 * 用法:
 *   <joystick size="{{260}}" bind:move="onJoyMove" bind:end="onJoyEnd"></joystick>
 *
 * 事件:
 *   start -> detail: {}
 *   move  -> detail: { x: -1~1, y: -1~1 }   (屏幕坐标: 上=+y, 右=+x)
 *   end   -> detail: {}
 *
 * 实现要点:
 *  - 用 touch.identifier 只跟同一根手指，避免和横移/STOP 按钮的多指操作串味
 *  - 每次 touchstart 重新测一次中心（键盘弹出/滚动会让旧矩形失效）
 *  - radius 默认按尺寸算（size 的 26%），保证滑块不会顶出圆盘被裁掉
 */
Component({
  properties: {
    /** 组件边长 (px) */
    size: { type: Number, value: 240 },
    /** 摇杆最大行程半径 (px)；<=0 表示按尺寸自动算 */
    radius: { type: Number, value: 0 }
  },

  data: {
    knobX: 0,
    knobY: 0,
    active: false
  },

  lifetimes: {
    ready: function () {
      this._measure()
    }
  },

  methods: {
    _travel: function () {
      const r = Number(this.data.radius)
      if (r > 0) return r
      return Math.max(40, Number(this.data.size) * 0.26)
    },

    _measure: function () {
      const self = this
      this.createSelectorQuery().select('.joy').boundingClientRect(function (rect) {
        if (rect && rect.width > 0) {
          self._originX = rect.left + rect.width / 2
          self._originY = rect.top + rect.height / 2
        }
      }).exec()
    },

    onTouchStart: function (e) {
      const touch = e.touches && e.touches[0]
      if (!touch) return
      this._touchId = touch.identifier
      this.setData({ active: true, knobX: 0, knobY: 0 })
      this._measure()
      // 中心还没测出来时先把原点设成触点位置，至少不会因为 undefined 直接跳变
      if (this._originX === undefined) {
        this._originX = touch.clientX
        this._originY = touch.clientY
      }
      this.triggerEvent('start', {})
    },

    onTouchMove: function (e) {
      const touch = this._findTouch(e)
      if (!touch) return
      if (this._originX === undefined) { this._measure(); return }
      const travel = this._travel()
      let dx = touch.clientX - this._originX
      let dy = touch.clientY - this._originY
      const len = Math.sqrt(dx * dx + dy * dy)
      if (len > travel) {
        dx = dx / len * travel
        dy = dy / len * travel
      }
      this.setData({ knobX: dx, knobY: dy })
      // 屏幕坐标: 上为正 y（返回 +y 表示前进方向）
      this.triggerEvent('move', { x: dx / travel, y: -dy / travel })
    },

    onTouchEnd: function (e) {
      // 只有"我认领的那根手指"抬起来才算结束，其它手指抬起忽略
      const touch = (e.changedTouches && e.changedTouches[0]) || null
      if (touch && this._touchId !== null && this._touchId !== undefined &&
          touch.identifier !== this._touchId) {
        return
      }
      this._touchId = null
      this.setData({ active: false, knobX: 0, knobY: 0 })
      this.triggerEvent('end', {})
    },

    onTouchCancel: function () {
      this._touchId = null
      this.setData({ active: false, knobX: 0, knobY: 0 })
      this.triggerEvent('end', {})
    },

    _findTouch: function (e) {
      const id = this._touchId
      if (id === null || id === undefined) return null
      const lists = [e.touches, e.changedTouches]
      for (let k = 0; k < lists.length; k++) {
        const list = lists[k]
        if (!list) continue
        for (let i = 0; i < list.length; i++) {
          if (list[i].identifier === id) return list[i]
        }
      }
      return null
    }
  }
})
