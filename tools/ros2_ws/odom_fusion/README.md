# odom_fusion — 融合里程计（全车唯一 odom 广播者）

## 这是什么

把**激光里程计（rf2o）**、**固件轮式里程计**、**IMU** 三路揉成一路自洽的 odom，
发布 `/odom_fused` 和 TF `odom_fused → base_link`。slam_toolbox 与 nav2 都只用它。

一句话概括设计：
> **位置信激光，航向信"增量"** —— 位置用激光的绝对位姿（激光在平移上最准），
> 航向只用各路传感器的**帧间增量**加权（增量与坐标系无关，能绕开"磁绝对不可信、激光有基准差"这个死结）。

## ⚠️ 改这个包之前必须知道的三件事

1. **位置必须和航向在同一个坐标系里。** 上一版的 bug 就出在这里：
   位置抄 `/odom_laser` 的增量（激光系），航向却用 `/odom` 的磁绝对航向（固件系）做首帧基准
   —— 两个基准不一致，实测位移方向与自报航向差了 **119°**，建图里墙全是平行重影。
   现在首帧基准统一取 `yaw_laser`。
2. **`/odom` 的位置不可用。** `chassis_bridge` 只把航向转成了 ROS 口径，位置仍是固件世界系
   （X=北 / Y=西），与 ROS 系（北/东）差 90°。所以融合**只取 `/odom` 的 yaw，不取它的 x/y**。
3. **动完必须跑自洽性验证**：`tools/diagnostics/probe_dir.py --dist 0.30`。
   它让车走 0.30 m，比较"位移方向"与"自身航向"，偏差进 ±3° 才算过。
   原地旋转测不出这个问题（位置不参与），**只有平移才暴露**。

## 算法

### 位置（`_fuse_and_publish` 第 6 步）

```
x_fused = laser.x − laser_origin.x        # 绝对位姿减首帧原点，不是增量累加
y_fused = laser.y − laser_origin.y
```

- **为什么不用增量累加**：某一帧被判跳变丢弃后，累加值和激光绝对值会**永久分叉**，
  之后越走越偏且拉不回来。用绝对位姿则每帧自动归位。
- **跳变拒绝**：单帧增量 > `pos_jump_reject`（0.30 m）→ 判 rf2o 跳变，本帧位置冻结。
- **静止 / 激光退化时冻结**：车不动位置本来就不该变。rf2o 在纯旋转和静止时会算假位移
  （实测原地旋转漂 29 mm，被抬起晃动时"走"了 0.49 m）。

### 航向（第 5 步）

```
首帧：  yaw_fused = yaw_laser                      # 基准统一到激光系
之后：  yaw_fused += (1−w)·dyaw_wheel + w·dyaw_laser
```

- `w` = 激光可信度（由扫描有效点占比、激光 odom 是否新鲜算出）。
- **轮式至少保留 `wheel_weight_floor = 0.05`**：保证激光完全退化时仍有一点点磁绝对兜底。
- **为什么只加权增量**：轮式是磁绝对航向（不漂但有环境磁场梯度误差），激光是相对初始帧（短期准但有基准差）。
  两者基准差一大段常数，直接用绝对差修正会把磁绝对拉到激光基准上。增量相加时基准常数自动消掉。

### 静止冻结（`still_anchor_freeze = True`）

判据：轮式线速度 < `still_lin_thresh`(0.01) 且 IMU 角速度 < `still_ang_thresh`(0.02)，
持续 `still_hold_sec`(0.5s)。

冻结后**不再向磁绝对航向收敛**。原因：静止时 rf2o 的 yaw 只是噪声（实测 110 s 漂 8.5°），
而磁绝对航向受环境磁场梯度影响（实测 **21 µT/m**，地磁水平分量才 18.83 µT），
停车位置一换读数就变。两者都不可信时，什么都不做最安全。

> 旧配置（floor = 0.3 + 静止向磁收敛）实测让航向偏 −5.53°；
> 改成 0.05 + 冻结后为 **+0.09°**，改善 6 倍。

### 磁野值剔除（第 5.5 步）

用**激光 dyaw 当裁判**：`|dyaw_wheel − dyaw_laser| > mag_outlier_thresh_rad`(0.35 rad ≈ 20°)
→ 判磁被扰动，本帧丢弃轮式增量、纯激光兜底。
连续超 `mag_outlier_max_gate`(0.5 s) 则反过来怀疑激光自己漂了，强制恢复采纳轮式
（防止激光退化时把好的轮式全杀掉）。

> 原理：手机等外部铁磁体只扰磁场、不扰光学测距。所以激光 dyaw 是干净的"真实旋转"参考。

---

## 参数速查

| 参数 | 默认 | 说明 |
|---|---|---|
| `wheel_topic` / `laser_topic` | `/odom` / `/odom_laser` | 输入 |
| `out_topic` / `out_frame` | `/odom_fused` / `odom_fused` | 输出 |
| `publish_tf` | `True` | **只允许本节点发 odom→base_link 的 TF**，rf2o 必须关掉 |
| `scan_valid_ratio_threshold` | 0.25 | 有效点占比低于此值 → 判激光退化 |
| `laser_stale_sec` | 0.5 | 激光 odom 超过此时间未更新 → 退化 |
| `pos_jump_reject` | 0.30 m | 单帧位置跳变拒绝阈值 |
| `wheel_weight_floor` | **0.05** | 轮式最小权重（**不要回退到 0.3**） |
| `still_anchor_freeze` | **True** | 静止时冻结航向 |
| `mag_outlier_thresh_rad` | 0.35 | 磁野值判据（≈20°） |
| `bias_*` / `laser_correction_gain` | — | **上一代兜底逻辑**，磁绝对航向下 b≈0，保留仅为兼容 `AHRSMAG,OFF` |

## 起法

通常由主 launch 带起（`warehouse_inspector/inspection.launch.py`），不需要单独起。
单独调试见 `rf2o_laser.launch.py`。

```bash
ros2 launch odom_fusion rf2o_laser.launch.py
```

## 已知限制

- **位置完全依赖激光**。周围 5 m 内没有可匹配结构时 rf2o 会退化，位置冻结（不会乱跑，但也不前进）。
- **`/odom` 的 90° 位置错位没修**，只是融合不再使用它。将来若有人直接订阅 `/odom` 取位置，同样的问题会复现。
- 航向不含**地理朝向**：起步 yaw≈0 就是"启动时车头方向"。所以地图 X 轴跟着车头走，与真北无关。
