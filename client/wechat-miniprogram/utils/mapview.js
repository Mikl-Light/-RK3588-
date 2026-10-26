/**
 * mapview.js — 把「canvas 初始化 + 底图下载 + 渲染」串起来
 *
 * telemetry 与 circle 两个页面共用：区别只在 circle 会额外把点击坐标
 * 反算成世界坐标（toWorld）。
 */
const md = require('./mapdraw')
const robot = require('./robot')

function attach (holder, selector, opts) {
  const o = opts || {}
  const imageInterval = o.imageInterval || 5000

  let canvas = null
  let renderer = null
  let lastImageAt = 0
  let hasImage = false

  const api = {
    ready: false,
    error: null,

    /** 在 onReady 里调用（selectorQuery 需要节点已渲染） */
    init: function () {
      return md.initCanvas(holder, selector).then(function (c) {
        canvas = c.canvas
        renderer = md.createRenderer(c, { theme: o.theme })
        api.ready = true
        return c
      }).catch(function (e) {
        api.error = e.message || String(e)
        return null
      })
    },

    /** /data 里的 map 元信息（拿不到就传 null，画布会按叠加数据自适应） */
    setMeta: function (m) {
      if (renderer) renderer.setInfo(m)
    },

    /** 按需重新下载底图（默认 5s 一次；force 强制） */
    refreshImage: function (force) {
      if (!canvas || !renderer) return
      const now = Date.now()
      if (!force && now - lastImageAt < imageInterval) return
      lastImageAt = now
      const url = robot.getUrl('/map.png', null, true)
      // 用 downloadFile 而不是直接把网络地址交给 createImage：
      // 前者有明确的 statusCode/失败回调，现场排障时能说清"是 204 无地图"还是"网不通"
      wx.downloadFile({
        url: url,
        success: function (res) {
          if (res.statusCode !== 200) return         // 204 = 暂无地图
          const image = canvas.createImage()
          image.onload = function () {
            hasImage = true
            renderer.setImage(image)
          }
          image.onerror = function () { /* 这一帧算了，下个周期再试 */ }
          image.src = res.tempFilePath
        },
        fail: function () { /* 网络抖动忽略 */ }
      })
    },

    render: function (state) {
      if (renderer) renderer.render(state)
    },

    /** 画布像素 -> 世界坐标；未就绪返回 null */
    toWorld: function (u, v) {
      return renderer ? renderer.toWorld(u, v) : null
    },

    hasImage: function () { return hasImage }
  }

  return api
}

module.exports = { attach: attach }
