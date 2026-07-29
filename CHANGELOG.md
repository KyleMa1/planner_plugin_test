# Changelog

本文件记录所有重要变更，格式遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)。

## [Unreleased]

### Added
- 工作空间规范化：`.gitignore`、`.editorconfig`、`Makefile`、`Dockerfile`、`docker-compose.yml`
- GitHub Actions CI（Navigation2 + 本仓库包构建）
- Git 交互式管理工具（`scripts/git-tool.sh`）
- `navigation2.repos` 用于 vcs 导入 Navigation2 humble
- `nav2_rtk_spline_planner` Nav2 全局规划插件与演示 launch
- `waypoint_demo` 路点发布与简易机器人仿真
- 文档：`docs/RTK_SPLINE_PURE_PURSUIT_CHECKLIST.md`

### Structure
- `src/nav2_rtk_spline_planner/` — RTK 三次样条 GlobalPlanner 插件
- `src/waypoint_demo/` — Python 演示与仿真节点
- `src/navigation2/` — 第三方 Nav2 源码（vcs import，不纳入 Git）
