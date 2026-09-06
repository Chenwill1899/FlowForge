# GVF-Nav使用说明


## 目录结构

```
GVF-Nav/
└── src/
    ├── Controller/                 # 控制器模块
    │   └── drone_control/          # PX4外环控制
    ├── Interface/                  # 接口层
    │   ├── uav_simulator/          # 无人机仿真模块
    │   │   ├── dynamic_map_generator/  # 动态地图生成器
    │   │   ├── mockamap/               # 模拟地图
    │   │   ├── so3_control/            # SO3控制器
    │   │   ├── so3_quadrotor_simulator/  # 四旋翼仿真器
    │   │   └── Utils/                  # 工具包
    │   ├── b2_gvf_sim/             # B2 运动学机器人仿真
    │   ├── diff_drive_gvf_sim/     # 差速车仿真
    │   ├── gvf_cmd_bridge/         # PositionCommand -> Twist 桥
    │   ├── human_input_sim/        # 人机意图输入仿真
    │   └── joystick_drivers/       # 手柄驱动
    └── Planner/                    # 规划器模块
        ├── fluid/                  # 流体GVF核心模块（fluid 包）
        ├── common_msgs/            # 通用消息定义
        └── plan_env/               # 规划环境（SDF/ESDF 地图）
```

## 环境要求

- **ROS版本**: ROS Noetic
- **Ubuntu版本**: Ubuntu 20.04 (ROS Noetic)

## Git拉取和使用

### 1. 从Git仓库拉取代码

```bash
git clone https://github.com/RicardoLEE123/GVF-Nav.git
```

### 2. 编译工作空间

```bash
cd GVF-Nav/
catkin_make
```

### 3. 配置环境变量

```
source devel/setup.bash
```

## 运行说明

### 仿真环境运行（test_gvf_3d.launch）

用于仿真环境中测试和调试 GVF 算法（3D 势流引导）。

#### 启动步骤：

1. **启动仿真器**（启动无人机仿真器和RViz）
   ```bash
   roslaunch so3_quadrotor_simulator simulator.launch
   ```

2. **启动GVF规划器**
   ```bash
   roslaunch fluid test_gvf_3d.launch
   ```

#### test_gvf_3d.launch 配置说明

- **点云话题**: `/sim/local_map` (sensor_msgs::PointCloud2)
- **里程计话题**: `/sim/odom` (nav_msgs::Odometry)
- **控制命令话题**: `/position_cmd` (quadrotor_msgs::PositionCommand)
- **地图尺寸**: 100m × 100m × 3.5m（`virtual_ceil_height 3.4`；3D 模式必须加高地图，
  2.5m 地图下天花板体素索引会越界，且翻越走廊不足）
- **求解窗口**: 前向 12m / 横向 10m，z 窗 `0.2 ~ 2.6m`，细层分辨率 0.15m

该launch文件会自动启动：
- `formation_planning` 节点（GVF规划核心）
- `map_generator` 节点（局部感知地图生成）

### 离线 rosbag 回放

```bash
# 终端 1：启动两个 RViz 视角
roslaunch so3_quadrotor_simulator play_bag.launch

# 终端 2：自行播放 rosbag
rosbag play --clock /absolute/path/to/benchmark.bag
```

该启动文件不播放 bag、不启动仿真器或规划器；它会打开独立的 `b2_bag_fpv.rviz` FPV 视角和
`b2_bag_swarm.rviz` 全局视角。使用 `enable_fpv:=false` 或 `enable_swarm:=false` 可只打开一个窗口。
其中 B2 FPV 会将记录的世界系 `/human_intent` 反旋转为机体系摇杆显示，不改变规划器收到的原始话题。
由于该 B2 bag 的录制 occupancy 点云为空，启动文件会从 `/fastLIO/non_ground_points` 恢复仅用于显示的
`/b2/gvf/occupancy` 和 `/b2/gvf/occupancy_inflate` 点云；后者为相同障碍点云的保底显示，
并非离线重新计算的膨胀栅格。
这些 B2 专用点云显示还会按 `/Odometry` 当前高度裁剪，隐藏其下方的点。

### 统一 Benchmark（无人机 / 差速车）

从宿主机执行同一个入口；脚本自动进入本项目的 ROS Noetic Docker 容器，
完成 roscore、仿真、规划、意图回放、rosbag 录制和指标绘图：

```bash
./scripts/run_sim_paper.sh 3d    # 无人机；不传参数也默认 3d
./scripts/run_sim_paper.sh 2d    # 差速车（Scout 级运动学仿真）

# 无界面运行
BENCHMARK_ENABLE_RVIZ=false ./scripts/run_sim_paper.sh 2d

# 3D 世界系力脉冲 / 固定参考流线实验
SIM_PAPER_DISTURBANCE_MODE=noise ./scripts/run_sim_paper.sh 3d
SIM_PAPER_EXPERIMENT_MODE=fixed SIM_PAPER_DISTURBANCE_MODE=noise ./scripts/run_sim_paper.sh 3d
```

| 模式 | 仿真 / 规划 | 默认场景和输入 |
| --- | --- | --- |
| `3d` | `so3_quadrotor_simulator/simulator.launch` + `fluid/test_gvf_3d.launch` | `pillar_forest_mixed.yaml` 生成混合 3D 柱林，`pillar_forest_crossing_v1.yaml` 持续横穿 |
| `2d` | `diff_drive_gvf_sim/diff_drive_benchmark_sim.launch` + `fluid/test_gvf_diff_drive.launch` | 已有 `pillar.pcd` 柱林，`diff_drive_v1.yaml` 往返转向轨迹；从原点朝 +Y 出发 |

2D 通过 `gvf_cmd_bridge` 把 `PositionCommand` 转为差速车的 `/cmd_vel`，
使用差速车已有的速度、转向和障碍膨胀配置。3D 的力脉冲和固定参考实验参数
不适用于差速车，传给 `2d` 时会报错。同一工作空间一次只运行一个 benchmark。

**容器配置和编译**

`docker/` 与 `scripts/ros1_docker.sh` 参考旧 `ros1_docker` 的 Noetic + VNC 构造，
依赖配置已随本项目保存，无需旧 GVF-Nav 目录。宿主机需要 Docker 及当前用户的访问权限。
首次运行会自动构建镜像、启动容器并编译；修改 C++ 后需重新执行 `compile`：

```bash
./scripts/ros1_docker.sh build       # 构建镜像（首次自动执行）
./scripts/ros1_docker.sh compile     # 容器内 catkin_make，默认 4 个编译任务
./scripts/ros1_docker.sh shell       # 进入已配置 ROS 的交互终端
./scripts/ros1_docker.sh logs        # 查看桌面启动日志
./scripts/ros1_docker.sh stop        # 停止本项目容器
```

默认镜像 `flowforge:noetic-vnc`、容器 `flowforge-noetic`，使用独立 Docker 网络，
不占用宿主机的 ROS Master。RViz 在容器桌面显示，用 VNC 客户端连接
`127.0.0.1:5902`（仅绑定宿主机回环地址，无密码）；默认软件渲染，不依赖 GPU 或手柄设备。
项目按宿主机的相同绝对路径挂载，生成结果直接保存在项目目录。

可通过 `FLOWFORGE_IMAGE`、`FLOWFORGE_DOCKER_CONTAINER`、`FLOWFORGE_VNC_PORT`、
`FLOWFORGE_BUILD_JOBS` 调整镜像、容器名、VNC 端口和编译并行数。
更新 Dockerfile 后重新 `build`，再删除已停止的本项目容器并运行 `start`，以使用新镜像；
删除容器不会删除宿主机项目和实验结果。

**实验覆盖参数**（从宿主机自动传入容器）

- `SIM_PAPER_SCENARIO`：自定义场景 YAML；3D 默认混合柱林，2D 不设置时使用已有柱林 PCD。
- `BENCHMARK_MAP`：自定义 PCD；2D 同时指定场景 YAML 时优先生成场景地图。
- `BENCHMARK_TRACE_FILE`：自定义意图 YAML，最后一条事件必须释放摇杆。
- `BENCHMARK_SIM_EXTRA_ARGS`：仿真 launch 参数，如 `"init_x:=0.0 init_y:=1.5"`。
- `BENCHMARK_GVF_EXTRA_ARGS`：规划 launch 参数；3D 如 `"fluid_cruise_z:=1.0 max_speed:=3.0"`，2D 如 `"v_max:=0.8"`。
- `SIM_PAPER_DISTURBANCE_MODE`：仅 3D，`none`（默认）/`pulse`/`noise`。
- `SIM_PAPER_EXPERIMENT_MODE`：仅 3D，`online`（默认）/`fixed`；fixed 使用专用地图和 trace。
- `BENCHMARK_LOG_DIR`：自定义结果目录。

地图和 trace 路径应位于挂载的项目目录内；换场景时需同步设置匹配的起点和意图。
额外 launch 参数以空格分隔 `name:=value`，会替换该模式默认的额外参数串。

每次运行会在 `logs/sim_paper/<3d|2d>/<时间戳>/` 下生成：

- `metadata.txt`：模式、仿真/规划入口、运行时间、代码提交、退出和绘图状态。
- `benchmark.bag`：里程计、地图、意图、规划指令、`/cmd_vel`、TF 和诊断话题。
- `raw/`：roscore、仿真器、规划器、回放、录制与绘图进程日志。
- `data/`：实际轨迹、规划指令、意图等 CSV 时序。
- `config/`：本次地图、场景、trace、规划器参数及完整 ROS 参数快照。
- `benchmark_plot.png`、`metrics.json`、`summary.txt`：轨迹和速度图、数值指标与摘要。

所有结果和 catkin 构建产物均被 Git 忽略。旧的独立 baseline runner 已合并进此入口。
宿主机 Ctrl+C/TERM 会转发到容器运行进程，等待录包关闭和 ROS 清理后返回非零状态。

默认场景完整运行后，可在容器终端中检查产物、非零运动、松杆停车和实际平台类型：

```bash
python3 scripts/tests/check_benchmark_run.py 2d logs/sim_paper/2d/<时间戳>
python3 scripts/tests/check_benchmark_run.py 3d logs/sim_paper/3d/<时间戳>
```


### 3D 势流引导原理与调参

在共享 ESDF 的局部窗口上解速度势/压力投影形式（`src/Planner/fluid/src/fluid_solver_3d.cpp`，
障碍为 Neumann 零通量壁面，一个 3D 标量 Poisson，粗细双层 + matrix-free CG 求解）。
垂直躲避由场对障碍几何的响应自动涌现——**意图输入仍是平面的**，摇杆/trace 不变。
矮墙自动翻越、悬空梁自动钻底、墙上开洞自动穿行，越障后由高度归航项缓降回
`fluid_cruise_z`。

调参要点（先读 `test_gvf_3d.launch` 内注释）：

- **必须用加高地图**：`map_size_z 3.5` + `virtual_ceil_height 3.4`。注意此时 ESDF 里
  **没有**地面/天花板——垂直界限由求解器窗口的 `fluid_3d_floor_band` /
  `fluid_3d_ceil_band` 强制上下实心带保证；PCD 地面点阵
  （`generate_planar_wall_pcd.py` 的 `ground:` 键）是可选项，加上会让 ESDF 近地净空
  变小从而在低空自然减速，去掉则点云更轻、RViz 更干净。
- **求解窗口与耗时**：`fluid_3d_z_min` / `fluid_3d_z_max`（0.2/2.6）+ 细层
  `fluid_3d_grid_resolution_z` 0.15、粗层 0.30 分辨率，细层约 45k 胞。实测：空载
  ~11ms/解，benchmark 满载（与仿真栈争用）均值 ~30ms、p90 ~55-70ms @10Hz 重解，
  cmdCallback 的 try_lock 跳 tick 兜底。超预算旋钮按序：`fluid_cg_tolerance`
  （已 3e-5）→ `fluid_3d_grid_resolution_z 0.21` → 细窗 8→6m →
  `fluid_resolve_period 0.15`。
- **3D 局部参考流线跟踪**（`fluid_3d_streamline_*` / `fluid_3d_k_n`）：每次细层求解后
  从"机器人在上一条流线上的投影点"（而非机器人本身）沿流线方向前后各积分一条短流线
  （RK2，步长 `fluid_3d_streamline_ds`；进入实心胞/速度退化/出窗即停），控制律
  `v_nom = v_t * t_hat(p*) - k_n * e_perp`，其中 p\* 是参考流线上最邻近投影点、
  t_hat 为流线切向。水平方向另叠加有界的射线收敛修正（D > fluid_d_look 时把航向拉回
  摇杆射线，否则机器人会骑在流线的全场横向偏移上回不来）；垂直通道为
  "场 w + 高度归航 + 尾流阻尼"（`fluid_3d_k_n_z_ratio` 默认 0.0 即纯旧垂直行为；>0
  启用全三维投影修正）。新旧流线公共区域位移超 `fluid_3d_streamline_max_disp`（1.5m）
  时进入 DEGRADE（回退融合律）并冷却 `fluid_3d_streamline_degrade_cooldown`（0.5s）
  防抖。开启高度归航（默认，benchmark 需要回航高度）时跟踪误差最终有界；置
  `fluid_3d_homing_in_track=false` 后跟踪区内为纯投影控制律。
- **驻点横流闩锁**（`fluid_3d_stall_*` / `fluid_3d_crossflow_ratio`）：顶天立地宽墙
  前势流真驻停，检测到持续低速且前方低净空时向远场注入横流强制选边；有垂直缺口时
  场自己会产生 w，闩锁不触发。横流连续演化（一阶滤波 `fluid_3d_crossflow_tau` /
  速率上限 `fluid_3d_crossflow_rate_max`），`u*_lat = beta * v_cap`，
  不再 0↔0.5v_cap 跳变。
- **高度归航**（`fluid_3d_alt_*`）：门控在 `cruise_z` 处采样的净空上——障碍正上方
  时门关（保高度），越过后门开缓降。势流在墙后尾流下卷会把机体压低，
  `alt_gate_lo` / `alt_gate_hi` 收紧到 0.6/1.0 是实调结果。
- **3D 分轴速度钳**：水平速度 `sqrt(vx²+vy²)` 受当前人机输入速度限制（静态上限由
  `gvf/human_intent_max_speed`，本 launch 为 3.0 m/s），垂直速度 `|vz|` 单独受
  `gvf/fluid_3d_w_max`（本 launch 为 0.9 m/s）限制；二者不再用三维总范数互相缩放。
- **3D 指令平滑**：势流场每次重解后，发布前对名义速度施加独立的水平/垂直斜率限制
  `gvf/fluid_3d_cmd_accel_xy_max`、`gvf/fluid_3d_cmd_accel_z_max`（本 launch 默认
  5.0、2.5 m/s²）。释放摇杆也沿同一斜率平滑减速，避免瞬时速度跳变。
- **3D 横流闩锁平滑**：`fluid_3d_crossflow_tau` 和 `fluid_3d_crossflow_rate_max`
  默认设为 0.8 s、1.0，让驻点选边的横向速度渐变注入，减少绕障时的突然转向；它只
  改变一个标量状态更新，不增加势流求解耗时。

### 安全监督

发布前的最后一步是 `gvf_manager::checkSafetySupervisor()`：只消除朝障碍物内法向的
速度分量（按 `gvf/safety_min_clearance` 与 `gvf/safety_brake_decel` 的等效减速度），
临界距离处补充切向逃逸，然后再做全部速度/执行器钳制并发布——任何下游钳制都不会
破坏执行器上限。

### 实际飞行运行（gvf.launch）

用于真实无人机飞行场景（默认 3D 流体模式）。

#### 启动步骤：

1. **配置点云话题**

   在 `gvf.launch` 文件中，确保点云话题配置正确：
   ```xml
   <param name="gvf/cloud_topic" type="string" value="/drone_1_cloud_registered" />
   ```

   根据实际系统配置，可能需要修改为：
   - 点云话题：例如 `/drone_X_cloud_registered` 或 `/camera/depth/points`
   - 话题类型必须是 `sensor_msgs::PointCloud2`

2. **配置里程计话题**

   确保里程计话题与SLAM系统输出一致：
   ```xml
   <param name="gvf/odom_topic" type="string" value="/drone_1_visual_slam/odom" />
   ```

3. **配置控制命令话题**

   确保控制命令话题与飞控系统一致：
   ```xml
   <param name="gvf/cmd_topic" type="string" value="/drone_1_planning/pos_cmd" />
   ```

#### gvf.launch 配置说明

- **点云话题**: `/drone_1_cloud_registered` (实际飞行时需要配置)
- **里程计话题**: `/drone_1_visual_slam/odom` (实际飞行时需要配置)
- **控制命令话题**: `/drone_1_planning/pos_cmd`
- **地图尺寸**: 50m × 50m × 3.85m（可根据实际需求调整）

该launch文件会启动：
- `formation_planning` 节点（GVF规划核心）
- `rviz` 可视化
- `px4ctrl` 飞控节点（通过include启动）

## 代码逻辑与关键参数（按当前实现）

以下内容对应 `formation_planning` 节点当前实现（见 `src/Planner/fluid/src/gvf_manager.cpp`、
`gvf.cpp`、`fluid_guidance.cpp`）。

### 主控制链（人机意图流体引导）

`cmdCallback()` 50Hz 运行，人机意图经摇杆推杆得到平面速度意图（`/human_intent`，
上限 `gvf/human_intent_max_speed`），送入 3D 势流引导：

1. `gvf_manager::updateFluidAnchor()`：摇杆激活或航向变化超过 `gvf/fluid_heading_latch_deg`
   时锁存流场锚点（意图射线原点），其余 tick 不重锚。
2. `gvf::calcFluidGuidance3D()`：在共享 ESDF 上自节流（`gvf/fluid_resolve_period`）
   求解 3D 势流场，每 tick 在当前机体位置三线性重采样缓存场，得到三维速度指令。
3. 速度/执行器钳制：水平速度以当前摇杆幅度为上限（释放一 tick 内归零）；
   垂直分量单独受 `gvf/fluid_3d_w_max` 限制，并经
   `gvf/fluid_3d_cmd_accel_xy_max` / `gvf/fluid_3d_cmd_accel_z_max` 分轴 slew 限幅。
4. `gvf_manager::checkSafetySupervisor()`：消除朝障碍物内法向分量，临界距离处
   补充切线逃逸。
5. 发布 `quadrotor_msgs::PositionCommand`（位置为当前机体位置，速度为主指令）。

### 流体引导参数（`gvf/fluid_*`）

- 窗口/网格：`gvf/fluid_window_forward_size` / `fluid_window_lateral_size` /
  `fluid_window_rear_margin`、`gvf/fluid_grid_resolution`（细层）、
  `gvf/fluid_coarse_grid_resolution`（粗层）。
- 固体/速度：`gvf/fluid_d_s`（ESDF 固体阈值）、`gvf/fluid_d_drag` /
  `fluid_d_turn`（速度斜坡）、`gvf/fluid_speed_floor`。
- 求解器：`gvf/fluid_cg_tolerance` / `fluid_cg_max_iterations` /
  `fluid_cg_residual_acceptance`、`gvf/fluid_resolve_period`（求解节流周期）。
- 3D 势流：`gvf/fluid_3d_*`（细层分辨率、z 窗、横流闩锁、流线跟踪、高度归航、
  垂直预览）。详细含义见 `src/Planner/fluid/include/fluid/fluid_guidance.h`
  中的 `FluidGuidanceConfig` 字段注释。

### 安全参数（`gvf/safety_*`）

- `gvf/safety_min_clearance`：最小安全余量，必须 >= `gvf/fluid_d_s`（manager 会自动抬升）。
- `gvf/safety_brake_decel`：向内分量制动的等效减速度。

## 常见问题：障碍物前左右徘徊（抖动/犹豫）

现象：无人机在障碍物前方、通道两侧代价接近时，会出现左右来回“犹豫/徘徊”。

常见原因：
- 顶天立地宽墙前势流驻停，窗口微小移动使场在两侧间翻转。已由驻点横流闩锁
  （`gvf/fluid_3d_stall_*`）接管：检测持续低速 + 前方低净空后注入渐变横流强制选边。
- 障碍物反复进出求解窗口导致场重建跳变。
- ESDF/栅格离散带来梯度毛刺。

优先调参建议（不改代码）：
- 若绕障后回归延迟偏长：缩小 `gvf/fluid_3d_crossflow_tau`；若仍在两侧切换：
  加大 `gvf/fluid_3d_stall_min_time` / 降低 `gvf/fluid_3d_crossflow_ratio`。
- 若在窗口边界附近抖动：检查 `gvf/fluid_resolve_period`（求解节流）与
  `gvf/fluid_window_*` 是否让障碍物反复进出窗口。
- 流线跟踪频繁进 DEGRADE：加大 `gvf/fluid_3d_streamline_degrade_cooldown`。

## 点云配置详解

### 1. 点云话题配置

在launch文件中设置点云话题：

```xml
<param name="gvf/cloud_topic" type="string" value="/your_pointcloud_topic" />
```

### 2. 点云话题验证

启动节点前，检查点云话题是否存在：
```bash
rostopic list | grep cloud
rostopic hz /your_pointcloud_topic
rostopic echo /your_pointcloud_topic -n 1
```

### 配置检查清单

在实际飞行前，请确认以下配置：

- [ ] 点云话题已正确配置
- [ ] 点云话题正在发布且频率正常（建议>10Hz）
- [ ] 里程计话题已正确配置
- [ ] 里程计坐标系与点云坐标系一致或已正确转换
- [ ] 控制命令话题与飞控系统匹配
- [ ] 地图尺寸参数适合实际飞行环境
- [ ] TF变换树配置正确（如果使用多坐标系）

## 常用调试命令

### 查看话题列表
```bash
rostopic list
```

### 查看话题频率
```bash
rostopic hz /sim/local_map
rostopic hz /sim/odom
```

### 查看话题内容
```bash
rostopic echo /sim/local_map -n 1
rostopic echo /sim/odom -n 1
```

### 查看节点信息
```bash
rosnode list
rosnode info /formation_planning
```

### 可视化点云
```bash
rviz
# 添加 PointCloud2 显示，选择对应的点云话题
```

### 查看参数
```bash
rosparam list
rosparam get /formation_planning/gvf/cloud_topic
```

## 故障排查

### 1. 点云无数据

- 检查话题名称是否正确
- 检查话题是否在发布：`rostopic hz /your_topic`
- 检查坐标系设置是否正确
- 查看节点日志：`rosnode info /formation_planning`

### 2. 里程计数据异常

- 检查里程计话题配置
- 确认TF变换正确
- 检查坐标系一致性

### 3. 规划器无响应

- 检查点云和里程计数据是否正常
- 检查地图参数配置（地图尺寸、分辨率等）
- 查看节点日志输出


---

**注意**: 在实际飞行前，请务必在仿真环境中充分测试，确保系统稳定可靠。
