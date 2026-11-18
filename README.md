# auto_drive_remake

基于 **ROS 2** 的自动驾驶 **规划与控制（PNC）** 教学/重构项目。采用模块化分层架构，通过自定义地图、全局路径、参考线与局部轨迹等模块，逐步搭建完整的规划链路，并在 RViz2 中可视化验证。

**仓库**：https://github.com/rest271828/auto_drive_car

**近期重点**：规划链路已**闭环**——参考线 → Frenet 投影 → 路径决策（静止障碍物绕行）→ 五次多项式局部路径 → ST 速度决策（跟车 / 让行 / 抢行）→ 速度规划 → 轨迹合成 → `car_move_cmd` 沿轨迹驱动主车，在 RViz 中可以看到车自己开起来。另附不依赖 ROS 的离线闭环仿真（`tools/offline_sim`），三套场景配置均无碰撞跑完全程。

## 项目结构

```
auto_drive_remake/
├── src/
│   ├── base_msgs/          # 自定义消息与服务接口
│   ├── planning/           # 规划主包（地图、全局/局部规划、决策、车辆模型等）
│   └── data_plot/          # 轨迹/数据绘图（Python，待接入 launch）
├── build/                  # colcon 构建产物（已 gitignore）
├── install/
└── log/
```

### 包说明

| 包名 | 类型 | 说明 |
|------|------|------|
| `base_msgs` | 接口包 | PNC 地图、参考线、局部路径/速度/轨迹等消息，以及地图与全局路径服务 |
| `planning` | C++ 功能包 | 规划核心逻辑、URDF、配置、launch、RViz 配置 |
| `data_plot` | Python 包 | 基于 `base_msgs` 的数据绘图，launch 中尚未启用 |

## 系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                     planning_process (10Hz)                       │
│  初始化：车辆 TF 生成 → 请求地图 → 请求全局路径                      │
│  周期回调：定位 → 参考线 → 投影 → 决策 → 路径 → 速度 → 轨迹         │
└──────────────┬───────────────────────────────┬───────────────────┘
               │ PNCMapService                 │ GlobalPathService
               ▼                               ▼
     ┌─────────────────┐             ┌──────────────────┐
     │  pnc_map_server │             │ global_path_server│
     └────────┬────────┘             └────────┬─────────┘
              │                               │
     ┌────────▼────────┐            ┌─────────▼────────────┐
     │ PNCMapCreator   │            │ GlobalPlannerNormal   │
     │ (straight/sturn)│            │ (沿车道中心线规划)     │
     └─────────────────┘            └──────────────────────┘

闭环：
  car_move_cmd ──tf(map→base_footprint)──┐        obs_move_cmd ──tf(map→base_footprint_obsX) + Marker
        ▲                                ▼
        │                     planning_process (10Hz)
        │   参考线 → 主车/障碍物 Frenet 投影 → 路径决策 → 局部路径(五次多项式)
        │   → 障碍物向路径投影 → 速度决策(ST) → 速度规划 → 轨迹合成
        └──────────── /planning/local_trajectory ────────────┘
```

### planning 模块划分

| 目录 | 职责 |
|------|------|
| `pnc_map_creator` | 参数化生成直道 / S 弯 PNC 地图，提供 `pnc_map_server` 服务 |
| `global_planner` | 全局路径规划器基类 + `normal` 实现，`global_path_server` 服务 |
| `reference_line` | 匹配点截取 + **OSQP 二次规划平滑** + Frenet 投影参数 |
| `local_planner` | 局部路径、局部速度、轨迹拼接 |
| `decision_center` | 行为决策（跟车、换道等） |
| `vehicle_info` | 主车与障碍物车辆状态 |
| `move_cmd` | 主车 / 障碍物运动指令节点 |
| `common` | YAML 配置读取、曲线与多项式数学库 |
| `planning_process` | 规划总入口节点 |

## 当前进度

### 已完成

- [x] ROS 2 `ament_cmake` 工作空间与三包骨架（`base_msgs` / `planning` / `data_plot`）
- [x] 自定义消息与服务（`PNCMap`、`Referline`、`LocalPath`、`LocalTrajectory` 等）
- [x] PNC 地图服务：直道（`straight`）与 S 弯（`sturn`）两种地图生成
- [x] 全局路径服务：`normal` 规划器（沿地图中心线生成 `nav_msgs/Path`）
- [x] 统一 YAML 配置（车辆、地图、规划参数），支持多场景配置文件
- [x] 主车 URDF（xacro 分模块：底盘、轮子、相机、雷达）及障碍物车模型
- [x] `planning_launch.py`：启动 RViz2、主车 `robot_state_publisher`、`pnc_map_server`、`global_path_server`、`planning_process`
- [x] `planning_process` 初始化：车辆 TF 生成（主车 + 3 辆障碍物）、地图/全局路径服务请求
- [x] `planning_process` **10Hz 定时规划回调**：TF 监听主车定位、生成并发布参考线
- [x] `vehicle_info`：`MainCar` / `ObsCar` 从配置加载位姿与尺寸
- [x] `reference_line`：全局路径匹配点、前后截取、**OsqpEigen QP 平滑**、Frenet 投影参数计算、`reference_line` 话题 RViz 可视化
- [x] `common/math`：匹配点搜索、投影参数等曲线工具
- [x] `osqp_test`：OSQP 求解器独立测试节点（`src/planning/src/test/`）
- [x] `planning_callback` 全流程：主车/障碍物投影、障碍物 s 排序、路径决策、局部路径、障碍物向路径投影、速度决策、速度规划、轨迹合成、绘图信息、车辆状态更新
- [x] `Curve`：`frenet_to_cartesian`、参考线匹配点/投影点、按 s 插值、点向局部路径投影；修正曲率计算中航向角跨 ±π 的问题
- [x] `PolynomialCurve`：一次/三次/五次多项式边界值闭式解（对应 `local_path.curve_type`）
- [x] `decision_center`：静止障碍物左右绕行决策（考虑障碍物斜停时的实际占宽）；动态障碍物 ST 决策（FOLLOW / YIELD / OVERTAKE）
- [x] `local_path_planner` / `local_speeds_planner` / `local_trajectory_combiner`
- [x] `car_move_cmd`：订阅轨迹、按时间插值、发布主车动态 tf；`obs_move_cmd`：障碍物匀速直线运动 + 按配置尺寸发布 Marker
- [x] 场景切换：`ros2 launch planning planning_launch.py config:=<yaml>`
- [x] 离线闭环仿真 `tools/offline_sim`（g++ 即可编译，复用真实算法源码）

### 进行中 / 待完善

- [ ] 局部路径 / 速度的 QP 平滑（`LocalPathSmoother` / `LocalSpeedsSmoother` 目前为空实现，接口已留好）
- [ ] 速度规划目前是带约束的前向积分，加速度会在 +1.5 / 0 / -2 之间切换，后续用 QP 平滑 jerk
- [ ] 动态障碍物只做跟车/让行，不做超车换道
- [ ] `data_plot` 绘图节点：`/planning/plot_info` 已发布，节点待实现
- [ ] 全局规划器扩展：配置中预留 `astar` 类型，尚未实现

## 参考线平滑（二次规划）

当前阶段的核心实现位于 `reference_line_smoother.cpp`，使用 **OsqpEigen**（底层 OSQP）对参考线离散点做 QP 平滑。

### 流程

1. **匹配点**：`Curve::find_match_point` 在主车位置与全局路径间搜索最近点（支持上一帧索引加速）。
2. **截取**：以匹配点为中心，按配置 `reference_line.front_size` / `back_size` 从全局路径截取一段离散点。
3. **QP 平滑**：`ReferenceLineSmoother::smooth_reference_line` 优化各点 `(x, y)`。
4. **Frenet 参数**：`Curve::cal_projection_param` 为每个参考点计算 `s`、`l`、航向角、曲率等。
5. **可视化**：`ReferencelineCreator::referline_to_rviz` 转为 `nav_msgs/Path`，由 `planning_process` 发布到话题 `reference_line`（10Hz）。

### 优化问题形式

决策变量为 \(2n\) 维向量（\(n\) 个点的 \(x,y\) 坐标）。目标函数为关于相邻点差分的**分段二次型**（由权重 `w1`、`w2`、`w3` 构成 Hessian 矩阵 `P`），在保持路径形状的同时抑制一阶、二阶变化，使参考线更平滑。

约束为**盒约束**（box constraint）：

- 每个坐标满足 \(|x_i - x_i^{orig}| \le 0.2\)、\(|y_i - y_i^{orig}| \le 0.2\)（中间点）
- **首尾点固定**（偏差上界为 0），保证参考线端点与截取段一致

默认权重（`reference_line_smoother.h`）：`w1 = 100`，`w2 = 10`，`w3 = 1`。

### 相关代码

| 文件 | 说明 |
|------|------|
| `reference_line_creator.cpp` | 截取 + 调用平滑 + 投影参数 |
| `reference_line_smoother.cpp` | 构造 `P`、`Q`、上下界，调用 OSQP 求解 |
| `common/math/curve.cpp` | 匹配点、Frenet 投影 |
| `planning_process.cpp` | `planning_callback` 中周期生成并发布参考线 |

## 依赖

- **ROS 2**（Humble 或兼容发行版，需已 `source` 对应 `setup.bash`）
- **Eigen3**
- **OsqpEigen** + **OSQP**（参考线平滑；局部路径/速度平滑待接入）
- **yaml-cpp**
- 常用 ROS 包：`rclcpp`、`tf2`、`tf2_ros`、`geometry_msgs`、`nav_msgs`、`visualization_msgs`、`robot_state_publisher`、`joint_state_publisher`、`rviz2`、`xacro`

> CMake 中 Eigen 头文件路径写为 `/usr/local/include/eigen-3.4.0`，若本机安装位置不同，请在 `src/planning/CMakeLists.txt` 中调整 `include_directories`。

## 构建

在工作空间根目录执行：

```bash
cd /path/to/auto_drive_remake
colcon build --symlink-install
source install/setup.bash
```

仅编译 planning 相关包：

```bash
colcon build --packages-select base_msgs planning --symlink-install
source install/setup.bash
```

## 运行

```bash
source install/setup.bash
# 默认: 静态障碍物 + S 弯
ros2 launch planning planning_launch.py
# 切换场景
ros2 launch planning planning_launch.py config:=planning_onlane_obs_config.yaml   # 前方慢车, 跟车
ros2 launch planning planning_launch.py config:=planning_dynamic_obs_config.yaml  # 横穿车辆让行 + 宽障碍物绕行
```

launch 会启动：RViz2、主车 `robot_state_publisher`、`pnc_map_server`、`global_path_server`、`planning_process`、`car_move_cmd`、`obs_move_cmd`。

RViz 中可以看到：

| 显示 | 话题 | 说明 |
|------|------|------|
| ReferenceLine | `/planning/reference_line` | 绿色, QP 平滑后的参考线 |
| LocalPath | `/planning/local_path` | 蓝色, 局部路径 |
| LocalTrajectory | `/planning/local_trajectory_rviz` | 橙色, 轨迹(点间距反映速度) |
| ObsCar | `/planning/obs_markers` | 障碍物(尺寸取自配置文件) |

其他话题：`/planning/local_trajectory`（`base_msgs/LocalTrajectory`, 给 `car_move_cmd`）、`/planning/plot_info`（`base_msgs/PlotInfo`, 给绘图）。

> **tf 说明**：`map → base_footprint` 由 `car_move_cmd` 发布，`map → base_footprint_obsX` 由 `obs_move_cmd` 发布。主车 URDF 已去掉固定的 `map` 关节（否则 robot_state_publisher 会发一个原点处的静态 tf 与之冲突）；障碍物不再用 URDF 而用 Marker 显示。

独立运行各节点（调试用，需先 `export PLANNING_CONFIG=xxx.yaml`，不设置则用静态场景）：

```bash
ros2 run planning pnc_map_server
ros2 run planning global_path_server
ros2 run planning planning_process
ros2 run planning car_move_cmd --ros-args -r __ns:=/planning
ros2 run planning obs_move_cmd --ros-args -r __ns:=/planning
```

## 一键测试（ROS 2 Humble 机器上）

```bash
bash tools/ros_test/run_all.sh            # 环境检查 → colcon build → 离线仿真 → 三个场景各跑 40s 闭环
RVIZ=true bash tools/ros_test/run_all.sh  # 同时打开 RViz 看过程
```

结果汇总在 `test_logs/<时间>/summary.txt`：编译是否通过、规划频率、行驶距离、碰撞次数、最小车身间距、决策统计和节点报错。

## 闭环规划算法说明

**单位**：配置文件里的速度单位是 **m/帧**（规划周期 0.1s），代码内部统一换算成 m/s（`kPlanningCycle`），例如 `speed_limit: 1.0` 即 10 m/s。

1. **定位**：tf 监听主车与障碍物位姿（航向角从四元数取 yaw）；主车速度/加速度/曲率取自上一帧轨迹在当前时刻的插值。障碍物只保留 `obs_dis` 范围内的。
2. **Frenet 投影**：`find_projection_point` 找参考线投影点，`cartesian_to_frenet` 得到 `s, l, l', l''` 等。
3. **路径决策**（`DecisionCenter::make_path_decision`）：静止障碍物若挡住本车道走廊 `|l| < 车半宽 + safe_dis_l`，比较左右剩余空间，选可行且偏移更小的一侧，得到 `[s_begin, s_end]` 区间内的目标偏移 `target_l`；相邻区间合并。道路横向范围按地图生成方式取 `[-半宽/2, 1.5·半宽]`（右车道中心为 0）。
4. **局部路径**（`LocalPathPlanner`）：从主车当前 `(l, l', l'')` 出发，生成"保持 → 换道 → 保持 → 回中"的关键点，相邻关键点用多项式连接（`curve_type`: 0 一次 / 1 三次 / 2 五次），换道过渡长度 `max(safe_dis_s, 2s × 限速)`；再用 `frenet_to_cartesian` 转回 `x, y, θ, κ`。路径长度覆盖速度时域（`speed_size × 0.1s × 限速`）。
5. **速度决策**（`make_speed_decision`）：障碍物按匀速直线预测，每个时间步投影到局部路径上，横向距离小于 `(车宽+障碍宽)/2 + 0.3` 即视为占用路径，得到 `t_in / t_out`：
   - 静止或沿路径行驶 → **FOLLOW**：`s(t) ≤ s_obs(t) − (车长+障碍长)/2 − safe_dis_s`
   - 横穿，且主车以最大加速度能在它进入前通过 → **OVERTAKE**（不约束）
   - 横穿，来不及 → **YIELD**：离开前 `s(t)` 不超过穿越点前方安全距离
6. **速度规划**（`LocalSpeedsPlanner`）：0.1s 步长前向积分，每步允许速度取 道路限速、弯道限速 `√(a_lat/κ)`、路径终点、各 ST 约束 `√(v_obs² + 2b·gap)` 的最小值，加速度限制在 `[-6, 1.5] m/s²`。
7. **轨迹合成**：速度点的 `s` 在路径上插值得到带时间的轨迹，发布给 `car_move_cmd`。

## 离线闭环仿真（不需要 ROS）

`tools/offline_sim` 用几个极简头文件替代 rclcpp / 消息类型 / yaml-cpp，**直接编译 planning 包里的真实算法源码**（ConfigReader、参考线、决策、路径、速度、轨迹），按 `planning_callback` 同样的流程逐帧运行，并做碰撞检测。参考线 OSQP 平滑在离线版中替换为空实现。适合在没有 ROS 的机器上快速调算法。

```bash
cd tools/offline_sim
./build.sh
export SIM_SHARE_DIR=$(pwd)/../../src/planning   # 指向 config 所在目录
./offline_sim planning_static_obs_config.yaml static.csv     # SIM_LOG=1 可看决策日志
python3 plot_sim.py static.csv -o static.png
```

当前结果（2026-11）：

| 场景 | 结果 |
|------|------|
| static（S 弯, 3 个静止障碍物） | 25.3s 到终点，无碰撞，最小车身间距 0.50 m，最大横向加速度 2.6 m/s² |
| onlane（前方 6 m/s 慢车） | 全程跟车到终点，无碰撞 |
| dynamic（2 辆横穿车 + 宽障碍物） | 对 obs1 抢行、对 obs2 停车让行、绕开 obs3，无碰撞，最小间距 0.49 m |

## 配置

配置文件位于 `src/planning/config/`：

| 文件 | 场景 |
|------|------|
| `planning_static_obs_config.yaml` | 静态障碍物场景（**默认**；用 launch 参数 `config:=` 或环境变量 `PLANNING_CONFIG` 切换） |
| `planning_onlane_obs_config.yaml` | 车道内障碍物 |
| `planning_dynamic_obs_config.yaml` | 动态障碍物 |

主要配置项示例：

- `pnc_map.type`：`0` 直道，`1` S 弯
- `global_path.type`：`0` normal（沿中心线），`1` astar（预留）
- `reference_line.front_size` / `back_size`：参考线前后截取点数（影响 QP 规模）
- `local_path.curve_type`：多项式阶次（一次 / 三次 / 五次）
- `vehicle`：主车与障碍物初始位姿、尺寸、速度

修改配置后需重新 `colcon build`（或使用 `--symlink-install` 直接生效）。

## 自定义接口（base_msgs）

**服务**

- `PNCMapService`：按 `map_type` 请求 PNC 地图
- `GlobalPathService`：传入地图与规划器类型，返回全局路径

**消息（节选）**

- `PNCMap`：道路边界、中心线等可视化 Marker
- `Referline` / `ReferlinePoint`：Frenet 参考线
- `LocalPath`、`LocalSpeeds`、`LocalTrajectory`：局部规划结果

## 可执行节点

| 节点 | 命令 |
|------|------|
| PNC 地图服务 | `ros2 run planning pnc_map_server` |
| 全局路径服务 | `ros2 run planning global_path_server` |
| 规划总流程 | `ros2 run planning planning_process` |
| 主车运动指令 | `ros2 run planning car_move_cmd` |
| 障碍物运动指令 | `ros2 run planning obs_move_cmd` |
| 离线仿真 | `tools/offline_sim/build.sh`（见上文） |
| OSQP 测试 | `ros2 run planning osqp_test` |

## 开发说明

- 代码命名空间：`Planning`
- C++ 标准：C++17
- 构建时生成 `compile_commands.json`，便于 IDE 跳转与补全
- 新全局规划器可继承 `GlobalPlannerBase`，在 `GlobalPlannerType` 中扩展类型
- 新地图类型可继承 `PNCMapCreatorBase`，在 `pnc_map_server` 中注册

## 路线图（参考）

1. ~~`planning_process` 周期回调与参考线发布~~（已完成）
2. ~~参考线 OSQP 平滑~~（已完成）；局部路径 / 速度 OSQP（待做）
3. ~~补齐 `planning_callback`：障碍物投影、决策、局部路径与速度、轨迹合成~~（已完成）
4. ~~接入 `move_cmd` 实现闭环仿真~~（已完成）
5. 启用 `data_plot` 辅助调参
6. 扩展 A* 全局规划；~~多场景配置切换~~（已完成）

## 许可证

各包 `package.xml` 中许可证字段尚未填写，使用前请自行补充。

## 维护者

codspielen (`codspielen@git.com`)
