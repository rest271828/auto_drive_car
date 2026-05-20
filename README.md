# auto_drive_remake

基于 **ROS 2** 的自动驾驶 **规划与控制（PNC）** 教学/重构项目。采用模块化分层架构，通过自定义地图、全局路径、参考线与局部轨迹等模块，逐步搭建完整的规划链路，并在 RViz2 中可视化验证。

**仓库**：https://github.com/rest271828/auto_drive_car

**近期重点**：参考线模块已接入 **OSQP 二次规划平滑**（`ReferenceLineSmoother`），并在 `planning_process` 中以 10Hz 周期发布至 RViz。

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
│  周期回调：定位 → 参考线 → [决策/局部规划/轨迹...]（占位，开发中）    │
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

已实现链路：global_path → ReferencelineCreator → OSQP 平滑 → RViz 发布
待接入：decision_center → local_path / local_speeds → trajectory → move_cmd
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

### 进行中 / 待完善

- [ ] `planning_callback` 后半段（代码中已留占位）：主车/障碍物投影、路径决策、局部路径、速度决策、轨迹合成、绘图与车辆状态更新
- [ ] `decision_center`、`local_path_planner`、`local_speeds_planner`、各类 smoother：类结构已就绪，核心算法待填充
- [ ] 周期回调中障碍物定位筛选（`obses_`）及与参考线的交互逻辑
- [ ] `move_cmd` 节点：骨架已建，未接入 launch，未驱动车辆运动
- [ ] `data_plot` 绘图节点：未接入 launch
- [ ] 全局规划器扩展：配置中预留 `astar` 类型，尚未实现
- [ ] 障碍物动态场景：`planning_dynamic_obs_config.yaml`、`planning_onlane_obs_config.yaml` 待切换与联调

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
- **OsqpEigen** + **OSQP**（参考线平滑已实现；局部路径平滑待接入）
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
ros2 launch planning planning_launch.py
```

启动后预期行为：

1. RViz2 加载 `planning.rviz` 配置
2. 命名空间 `/car` 下发布主车模型
3. 命名空间 `/planning` 下运行地图服务、全局路径服务与规划总流程节点
4. `planning_process` 向 `pnc_map_server` 请求地图，再向 `global_path_server` 请求全局路径
5. 初始化后 **10Hz** 定时回调：监听主车 TF → QP 平滑参考线 → 发布 `/planning/reference_line`（RViz 显示）

独立运行各节点（调试用）：

```bash
ros2 run planning pnc_map_server
ros2 run planning global_path_server
ros2 run planning planning_process
```

## 配置

配置文件位于 `src/planning/config/`：

| 文件 | 场景 |
|------|------|
| `planning_static_obs_config.yaml` | 静态障碍物场景（**当前默认**，由 `ConfigReader` 加载） |
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
3. 补齐 `planning_callback`：障碍物投影、决策、局部路径与速度、轨迹合成
4. 接入 `move_cmd` 实现闭环仿真
5. 启用 `data_plot` 辅助调参
6. 扩展 A* 全局规划与多场景配置切换

## 许可证

各包 `package.xml` 中许可证字段尚未填写，使用前请自行补充。

## 维护者

codspielen (`codspielen@git.com`)
