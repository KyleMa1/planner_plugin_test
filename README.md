# planner_plugin_test — Nav2 RTK Spline 规划器工作空间

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

ROS 2 Humble 工作空间：RTK 路点三次样条全局规划器（Nav2 插件）、Pure Pursuit / MPPI 演示，以及轻量 waypoint 仿真工具。

## 目录结构

```
.
├── .github/workflows/ci.yml    # GitHub Actions CI
├── docs/                       # 设计与落地清单
├── docker/
│   └── entrypoint.sh           # 容器启动脚本
├── scripts/
│   └── git-tool.sh             # Git 交互式管理工具
├── src/
│   ├── nav2_rtk_spline_planner/  # Nav2 GlobalPlanner 插件
│   ├── waypoint_demo/            # 路点发布与简易仿真
│   └── navigation2/              # vcs import（不提交到 Git）
├── navigation2.repos           # Navigation2 humble 依赖
├── Dockerfile
├── docker-compose.yml
├── Makefile
├── CHANGELOG.md
└── LICENSE
```

## 快速开始（推荐：Docker）

```bash
git clone git@github.com:KyleMa1/planner_plugin_test.git
cd planner_plugin_test
make deps-import
make docker-build
make docker-dev
# 容器内:
make deps-rosdep
make build
source install/setup.bash
```

## 快速开始（本机）

```bash
cd planner_plugin_test
make deps-import          # 若尚未 clone navigation2
make deps-rosdep
make build
source install/setup.bash
```

### 演示 Launch

```bash
# RTK 样条 + Nav2 全流程
ros2 launch nav2_rtk_spline_planner rtk_spline_demo.launch.py

# Regulated Pure Pursuit 跟踪
ros2 launch nav2_rtk_spline_planner pure_pursuit_demo.launch.py

# MPPI 局部偏离演示
ros2 launch nav2_rtk_spline_planner mppi_local_deviation_demo.launch.py
```

路点文件默认：`src/waypoint_demo/data/waypoints.txt`。

## 常用命令

```bash
make help                              # 查看所有命令
make build                             # colcon 全量构建
make build-pkg PKG=nav2_rtk_spline_planner
make test
make clean
make git-tool                          # Git 交互式 push / pull
```

## 文档

| 文档 | 说明 |
|------|------|
| [RTK Spline 与 Pure Pursuit 清单](docs/RTK_SPLINE_PURE_PURSUIT_CHECKLIST.md) | 插件、行为树、仿真与验收项 |

## CI/CD

Push 到 `main` 或 PR 时，GitHub Actions 会 `vcs import` Navigation2 并 colcon 构建工作空间。

## License

[Apache License 2.0](LICENSE)
