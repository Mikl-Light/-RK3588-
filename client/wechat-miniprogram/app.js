const robot = require('./utils/robot')

App({
  globalData: {},

  onLaunch() {
    // 启动即读取本地配置（IP/端口/速度），未配置时使用默认值
    robot.loadConfig()
  }
})
