# RTK Spline 全局规划器与 Pure Pursuit 落地清单

## 1. 目标与职责边界

- [x] 使用 RTK `x y theta` 路点构造平滑三次样条。
- [x] 将样条规划器实现为 Nav2 `nav2_core::GlobalPlanner` 插件。
- [x] 由 `planner_server` 加载插件并输出 `nav_msgs/Path`。
- [x] 由 `bt_navigator` 执行“周期规划 + 路径跟踪”行为树。
- [x] 由 Regulated Pure Pursuit 消费全局路径并输出 `/cmd_vel`。
- [x] 使用轻量差速机器人仿真发布 `/odom`、TF 和实际轨迹。
- [x] 在 RViz2 中显示 RTK 路点、全局路径、机器人和实际轨迹。
- [x] 明确能力边界：Pure Pursuit 负责路径跟踪和碰撞检测，不负责主动偏离路径绕障。

## 2. 数据链路

- [x] `waypoints.txt` 保存 `x y theta` 控制点。
- [x] `nav_demo` 读取文件并发布 `/rtk_waypoints`。
- [x] `RtkSplinePlanner` 订阅路点并建立弧长参数化三次样条。
- [x] `ComputePathToPose` 调用规划器并发布 `/plan`。
- [x] `FollowPath` 将 `/plan` 交给 Regulated Pure Pursuit。
- [x] Controller Server 发布 `/cmd_vel`。
- [x] `simple_robot_sim` 根据 `/cmd_vel` 更新机器人位姿。
- [x] 仿真节点发布 `/odom`、`odom -> base_link` TF 和 `/actual_trajectory`。

## 3. 全局规划器插件

### 3.1 插件接口与核心算法

- [x] 新建 `src/nav2_rtk_spline_planner/include/nav2_rtk_spline_planner/rtk_spline_planner.hpp`
  - 实现 `configure()`、`activate()`、`deactivate()`、`cleanup()`。
  - 实现 `createPlan(start, goal)`。
  - 保存 Costmap、TF、生命周期节点和参数回调。
  - 订阅可配置的 RTK waypoint topic。
- [x] 新建 `src/nav2_rtk_spline_planner/src/rtk_spline_planner.cpp`
  - 加载静态参数路点或 topic 路点。
  - 路点变化时才重建样条。
  - 将起点和终点投影到 RTK 样条。
  - 按 `path_resolution` 对样条进行采样。
  - 根据样条切线计算各 Path Pose 的朝向。
  - 可选检查 Costmap 碰撞。
  - 样条被障碍阻断时抛出 `nav2_core::PlannerException`，不返回截断路径。
  - 注册动态参数更新回调。
- [x] 新建 `src/nav2_rtk_spline_planner/include/nav2_rtk_spline_planner/cubic_spline.hpp`
  - 声明一维自然三次样条。
  - 声明弧长参数化二维样条。
  - 提供位置、航向、曲率、最近点和采样接口。
- [x] 新建 `src/nav2_rtk_spline_planner/src/cubic_spline.cpp`
  - 使用三对角方程求解样条系数。
  - 处理重复点、越界参数和短路径。
  - 实现二维样条弧长查询和最近点搜索。

### 3.2 Pluginlib 注册

- [x] 新建 `src/nav2_rtk_spline_planner/rtk_spline_planner_plugin.xml`
  - 插件名称：`nav2_rtk_spline_planner/RtkSplinePlanner`。
  - C++ 类型：`nav2_rtk_spline_planner::RtkSplinePlanner`。
  - 基类：`nav2_core::GlobalPlanner`。
- [x] 在 `rtk_spline_planner.cpp` 中使用 `PLUGINLIB_EXPORT_CLASS` 导出插件。

### 3.3 CMakeLists.txt

- [x] 修改 `src/nav2_rtk_spline_planner/CMakeLists.txt`。
- [x] 查找以下依赖：
  - `ament_cmake`
  - `rclcpp`
  - `rclcpp_lifecycle`
  - `nav2_util`
  - `nav2_core`
  - `nav_msgs`
  - `geometry_msgs`
  - `sensor_msgs`
  - `tf2_ros`
  - `nav2_costmap_2d`
  - `pluginlib`
- [x] 将以下源文件编译为共享库 `nav2_rtk_spline_planner`：
  - `src/rtk_spline_planner.cpp`
  - `src/cubic_spline.cpp`
- [x] 设置 C++17。
- [x] 添加：
  - `pluginlib_export_plugin_description_file(nav2_core rtk_spline_planner_plugin.xml)`
- [x] 安装共享库、头文件和插件 XML。
- [x] 安装 `config/` 和 `launch/` 目录。
- [x] 导出 include、library 和依赖。

### 3.4 package.xml

- [x] 修改 `src/nav2_rtk_spline_planner/package.xml`。
- [x] 声明与 CMake 一致的运行/编译依赖。
- [x] 在 `<export>` 中添加：
  - `<build_type>ament_cmake</build_type>`
  - `<nav2_core plugin="${prefix}/rtk_spline_planner_plugin.xml" />`
- [x] 添加 ament lint 测试依赖。

## 4. 全局规划器参数

- [x] 在 `src/nav2_rtk_spline_planner/config/nav2_pursuit_params.yaml` 配置 Planner Server。
- [x] 设置 `planner_plugins: ["RtkSplinePlanner"]`。
- [x] 设置插件类型：
  - `plugin: "nav2_rtk_spline_planner/RtkSplinePlanner"`
- [x] 设置样条采样：
  - `path_resolution: 0.5`
  - `search_radius: -1.0`
- [x] 设置路点输入：
  - `use_waypoint_topic: true`
  - `waypoint_topic: "rtk_waypoints"`
- [x] Pure Pursuit 演示启用：
  - `check_costmap_collision: true`
- [x] 保留独立参数示例：
  - `src/nav2_rtk_spline_planner/config/rtk_spline_planner_params.yaml`
- [x] 保留仅规划器演示参数：
  - `src/nav2_rtk_spline_planner/config/nav2_demo_params.yaml`

## 5. Regulated Pure Pursuit 配置

- [x] 在 `nav2_pursuit_params.yaml` 配置 Controller Server。
- [x] 设置：
  - `controller_plugins: ["FollowPath"]`
  - `controller_frequency: 20.0`
- [x] 配置控制器类型：
  - `plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"`
- [x] 配置速度：
  - `desired_linear_vel: 1.5`
  - `min_approach_linear_velocity: 0.3`
- [x] 配置前视距离：
  - `lookahead_dist: 4.0`
  - `min_lookahead_dist: 2.0`
  - `max_lookahead_dist: 8.0`
  - `lookahead_time: 1.5`
  - `use_velocity_scaled_lookahead_dist: true`
- [x] 配置转向：
  - `rotate_to_heading_angular_vel: 0.8`
  - `use_rotate_to_heading: true`
  - `rotate_to_heading_min_angle: 0.5`
  - `allow_reversing: false`
  - `max_angular_accel: 2.0`
- [x] 配置曲率限速：
  - `use_regulated_linear_velocity_scaling: true`
  - `regulated_linear_scaling_min_radius: 2.0`
  - `regulated_linear_scaling_min_speed: 0.3`
- [x] 配置碰撞检测：
  - `use_collision_detection: true`
  - `max_allowed_time_to_collision_up_to_carrot: 1.0`
- [x] 当前设置：
  - `use_cost_regulated_linear_velocity_scaling: false`
- [x] 配置 Progress Checker 和 Goal Checker。

## 6. Costmap 配置

- [x] 在 `nav2_pursuit_params.yaml` 配置 Global Costmap。
  - `global_frame: map`
  - `robot_base_frame: base_link`
  - `robot_radius: 0.5`
  - Rolling Window：`600 m x 300 m`
  - `resolution: 1.0`
- [x] 配置 Local Costmap。
  - `global_frame: odom`
  - `robot_base_frame: base_link`
  - `robot_radius: 0.5`
  - Rolling Window：`30 m x 30 m`
  - `resolution: 0.5`
- [x] 两个 Costmap 均加载：
  - `nav2_costmap_2d::ObstacleLayer`
  - `nav2_costmap_2d::InflationLayer`
- [x] Obstacle Layer 订阅 `/demo_obstacles`，数据类型为 `PointCloud2`。
- [x] Inflation Layer 设置：
  - `inflation_radius: 0.6`
  - `cost_scaling_factor: 3.0`

## 7. Behavior Tree

- [x] 新建 `src/nav2_rtk_spline_planner/config/navigate_spline.xml`。
- [x] 使用 `PipelineSequence` 组合规划与跟踪。
- [x] 使用 `RateController` 以 1 Hz 调用 `ComputePathToPose`。
- [x] 指定 `planner_id="RtkSplinePlanner"`。
- [x] 使用 `FollowPath` 并指定 `controller_id="FollowPath"`。
- [x] 新建 `navigate_through_poses_simple.xml`，避免 Humble 默认 ThroughPoses BT 依赖未启动的恢复服务器。
- [x] 在 YAML 中声明 BT Navigator 所需插件库。
- [x] Launch 运行时注入：
  - `default_nav_to_pose_bt_xml`
  - `default_nav_through_poses_bt_xml`

## 8. Pure Pursuit Launch

- [x] 新建 `src/nav2_rtk_spline_planner/launch/pure_pursuit_demo.launch.py`。
- [x] 声明 `waypoint_file` 启动参数。
- [x] 通过 `ament_index_python` 获取包安装目录。
- [x] 运行时生成临时 YAML，注入行为树绝对路径。
- [x] 启动静态 `map -> odom` TF。
- [x] 启动 `simple_robot_sim`。
- [x] 启动 `planner_server`。
- [x] 启动 `controller_server`。
- [x] 启动 `bt_navigator`。
- [x] 启动 `lifecycle_manager` 并自动激活 Nav2 节点。
- [x] 启动 `nav_demo`，默认等待 15 秒后发送导航目标。
- [x] 启动 RViz2 并加载 `pursuit_demo.rviz`。
- [x] 默认不启动障碍发布器，保持 Pure Pursuit 演示为纯路径跟踪。

## 9. Waypoint Demo Python 包

### 9.1 package.xml

- [x] 修改 `src/waypoint_demo/package.xml`。
- [x] 添加运行依赖：
  - `rclpy`
  - `nav_msgs`
  - `geometry_msgs`
  - `visualization_msgs`
  - `nav2_msgs`
  - `std_msgs`
  - `sensor_msgs`
  - `sensor_msgs_py`
  - `tf2_ros_py`
- [x] 设置构建类型为 `ament_python`。

### 9.2 setup.py

- [x] 修改 `src/waypoint_demo/setup.py`。
- [x] 将 `data/*` 安装到 `share/waypoint_demo/data/`。
- [x] 注册以下 console scripts：
  - `generate_waypoints`
  - `waypoint_visualizer`
  - `simple_robot_sim`
  - `nav_demo`
  - `obstacle_publisher`

### 9.3 Python 节点与数据

- [x] `waypoint_demo/generate_waypoints.py`
  - 生成长直线、转弯和折线路点。
- [x] `waypoint_demo/nav_demo.py`
  - 读取 `x y theta` 文件。
  - 发布 `/rtk_waypoints` 和 `/waypoint_markers`。
  - 调用 `NavigateToPose`。
  - 输出剩余距离和最终导航结果。
- [x] `waypoint_demo/simple_robot_sim.py`
  - 订阅 `/cmd_vel`。
  - 执行差速运动学积分。
  - 发布 `/odom` 和 TF。
  - 发布机器人 Marker 与 `/actual_trajectory`。
  - 支持线速度和角速度高斯噪声。
- [x] `waypoint_demo/waypoint_visualizer.py`
  - 用于仅规划器 Demo。
  - 调用 `ComputePathToPose` 并显示规划结果。
- [x] `waypoint_demo/obstacle_publisher.py`
  - 发布 `/demo_obstacles` PointCloud2。
  - 发布 `/obstacle_markers` MarkerArray。
  - Pure Pursuit 默认 Launch 不启动该节点。
- [x] `data/waypoints.txt`
  - 每个有效数据行格式为 `x y theta`。
  - 包含长直线、弯道和折线示例。

## 10. RViz 配置

- [x] 新建 `src/nav2_rtk_spline_planner/config/pursuit_demo.rviz`。
- [x] Fixed Frame 设置为 `map`。
- [x] 显示 `/waypoint_markers`。
- [x] 显示 `/rtk_waypoints`。
- [x] 显示 `/plan`。
- [x] 显示 `/local_plan`（控制器支持时）。
- [x] 显示 `/robot_marker`。
- [x] 显示 `/actual_trajectory`。
- [x] 显示 `/odom`。
- [x] 显示 Local Costmap。
- [x] 预留 `/obstacle_markers` 显示项。

## 11. 构建与运行

### 11.1 构建

```bash
cd /home/ma/planner_plugin_test
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_rtk_spline_planner waypoint_demo --symlink-install
source install/setup.bash
```

### 11.2 运行 Pure Pursuit 演示

```bash
ros2 launch nav2_rtk_spline_planner pure_pursuit_demo.launch.py
```

### 11.3 指定 waypoint 文件

```bash
ros2 launch nav2_rtk_spline_planner pure_pursuit_demo.launch.py \
  waypoint_file:=/home/ma/planner_plugin_test/src/waypoint_demo/data/waypoints.txt
```

### 11.4 仅运行全局规划器演示

```bash
ros2 launch nav2_rtk_spline_planner rtk_spline_demo.launch.py
```

## 12. 验证清单

- [ ] `planner_server` 状态为 `active`。
- [ ] `controller_server` 状态为 `active`。
- [ ] `bt_navigator` 状态为 `active`。
- [ ] `RtkSplinePlanner` 被 pluginlib 正确加载。
- [ ] `/rtk_waypoints` 有持续或 Transient Local 数据。
- [ ] `/plan` 是连续平滑样条。
- [ ] `/cmd_vel` 以预期频率发布。
- [ ] `/odom` 和 `odom -> base_link` TF 正常更新。
- [ ] RViz 中机器人沿 `/plan` 移动。
- [ ] `/actual_trajectory` 与 `/plan` 基本重合。
- [ ] 终端最终显示 `Navigation SUCCEEDED`。
- [ ] Ctrl+C 后所有节点正常退出。

验证命令：

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 topic echo /rtk_waypoints --once
ros2 topic echo /plan --once
ros2 topic hz /cmd_vel
ros2 topic echo /odom --once
ros2 topic echo /navigate_to_pose/_action/status --once
```

## 13. 能力边界与风险

- [x] Regulated Pure Pursuit 能进行路径跟踪、曲率限速、终点减速和前向碰撞检测。
- [x] `use_collision_detection: true` 表示检测到潜在碰撞后停止或报错，不表示生成绕障路径。
- [x] 当前 RTK Spline Planner 只在固定 RTK 样条上采样，不具备 A*、Hybrid-A* 或拓扑搜索能力。
- [x] `check_costmap_collision: true` 时，样条被阻挡会导致全局规划失败。
- [x] Pure Pursuit 不会主动偏离全局样条绕过障碍。
- [x] 若要求保持 RTK 全局参考并进行局部偏离，应使用 MPPI 或其他局部优化控制器。
- [x] 若要求保证找到障碍另一侧通路，应增加具备搜索能力的全局/局部规划层，不能仅依赖 Pure Pursuit。

## 14. 主要文件总览

- [x] `src/nav2_rtk_spline_planner/package.xml`
- [x] `src/nav2_rtk_spline_planner/CMakeLists.txt`
- [x] `src/nav2_rtk_spline_planner/rtk_spline_planner_plugin.xml`
- [x] `src/nav2_rtk_spline_planner/include/nav2_rtk_spline_planner/rtk_spline_planner.hpp`
- [x] `src/nav2_rtk_spline_planner/include/nav2_rtk_spline_planner/cubic_spline.hpp`
- [x] `src/nav2_rtk_spline_planner/src/rtk_spline_planner.cpp`
- [x] `src/nav2_rtk_spline_planner/src/cubic_spline.cpp`
- [x] `src/nav2_rtk_spline_planner/config/nav2_pursuit_params.yaml`
- [x] `src/nav2_rtk_spline_planner/config/navigate_spline.xml`
- [x] `src/nav2_rtk_spline_planner/config/navigate_through_poses_simple.xml`
- [x] `src/nav2_rtk_spline_planner/config/pursuit_demo.rviz`
- [x] `src/nav2_rtk_spline_planner/launch/pure_pursuit_demo.launch.py`
- [x] `src/waypoint_demo/package.xml`
- [x] `src/waypoint_demo/setup.py`
- [x] `src/waypoint_demo/data/waypoints.txt`
- [x] `src/waypoint_demo/waypoint_demo/generate_waypoints.py`
- [x] `src/waypoint_demo/waypoint_demo/nav_demo.py`
- [x] `src/waypoint_demo/waypoint_demo/simple_robot_sim.py`
- [x] `src/waypoint_demo/waypoint_demo/waypoint_visualizer.py`
- [x] `src/waypoint_demo/waypoint_demo/obstacle_publisher.py`
