# legacy/ — 已停用的旧方案

**这里的代码不再用于当前链路。新代码不要 import、不要 launch。**

保留目的：查历史设计、对照旧实现、出问题时回溯"以前是怎么做的"。

| 包 | 停用时间 | 被谁取代 | 停用原因 |
|---|---|---|---|
| `mecanum_bringup` | 2026-09 | `warehouse_inspector/chassis_bridge` | 旧串口桥 `serial_bridge` 没有失控看门狗（上层崩了底盘会保持最后速度继续跑）；新桥带 0.5 s 超时主动停车 |
| `mecanum_bringup_hector` | 2026-10 | `warehouse_inspector` | 旧 SLAM/导航 launch 集合。**与 warehouse_inspector 冲突，同时起会抢 tf 与 costmap** |
| `robot_gui` | 2026-09 | `robot_gui_ng` | 前端与交互已合并进 ng，ng 改用单端口 ThreadingHTTPServer |
| `odom_compare_web` | 2026-10 | `inspector_web` | 旧里程计对比 + Web 控制（8765）。当前手机端出口是 inspector_web |

## 这些包里的 README 都写在停用之前

看的时候记得：**文档描述的是当时的架构，不是现在的**。当前架构见：

- 项目根 `README.md`
- `tools/README.md`
- `tools/ros2_ws/*/README.md`

具体已确认过时的说法（读到别再信）：

- `mecanum_bringup/README.md` 把自己写成主串口桥 → 现在是 `chassis_bridge`
- `odom_compare_web/README.md` 说 8765 是"小程序首选后端" → 现在是 `inspector_web`
- 任何提到 "rf2o 发 TF" 的地方 → 现在 rf2o **不发 TF**，由 `fusion_node` 统一广播
- 任何提到 "/odom 是主里程计" 的地方 → 现在是 `/odom_fused`
