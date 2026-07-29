# ══════════════════════════════════════════════════════════
#  planner_plugin_test — 统一构建入口
#  用法: make <target>
# ══════════════════════════════════════════════════════════

SHELL := /bin/bash
.DEFAULT_GOAL := help

ROS_DISTRO   ?= humble
DOCKER_IMAGE ?= planner_plugin_test:humble-dev

# ── 帮助 ─────────────────────────────────────────────────
.PHONY: help
help: ## 显示此帮助
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}'

# ── 构建 ─────────────────────────────────────────────────
.PHONY: build
build: ## colcon 全量构建
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	colcon build --symlink-install

.PHONY: build-pkg
build-pkg: ## 构建单个包: make build-pkg PKG=nav2_rtk_spline_planner
	@if [ -z "$(PKG)" ]; then echo "用法: make build-pkg PKG=<包名>"; exit 1; fi
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	colcon build --symlink-install --packages-select $(PKG)

.PHONY: build-up-to
build-up-to: ## 构建某个包及其依赖: make build-up-to PKG=nav2_rtk_spline_planner
	@if [ -z "$(PKG)" ]; then echo "用法: make build-up-to PKG=<包名>"; exit 1; fi
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	colcon build --symlink-install --packages-up-to $(PKG)

# ── 测试 ─────────────────────────────────────────────────
.PHONY: test
test: ## colcon 全量测试
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	colcon test && \
	colcon test-result --verbose

.PHONY: test-pkg
test-pkg: ## 测试单个包: make test-pkg PKG=nav2_rtk_spline_planner
	@if [ -z "$(PKG)" ]; then echo "用法: make test-pkg PKG=<包名>"; exit 1; fi
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	colcon test --packages-select $(PKG) && \
	colcon test-result --verbose

# ── 清理 ─────────────────────────────────────────────────
.PHONY: clean
clean: ## 清理 build/ install/ log/
	rm -rf build/ install/ log/
	@echo "已清理 build/ install/ log/"

# ── Docker ───────────────────────────────────────────────
.PHONY: docker-build
docker-build: ## 构建 Docker 镜像
	docker compose build

.PHONY: docker-dev
docker-dev: ## 启动 Docker 开发环境
	docker compose run --rm dev

.PHONY: docker-run-build
docker-run-build: ## 在 Docker 中执行完整构建
	docker compose run --rm build

.PHONY: docker-down
docker-down: ## 停止并清理 Docker 容器
	docker compose down

# ── 依赖 ─────────────────────────────────────────────────
.PHONY: deps-import
deps-import: ## 导入 Navigation2 源码 (vcs import)
	vcs import src < navigation2.repos

.PHONY: deps-rosdep
deps-rosdep: ## 安装 src 下包的系统依赖 (rosdep)
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	rosdep install -y --from-paths src --ignore-src --rosdistro $(ROS_DISTRO)

# ── Git ──────────────────────────────────────────────────
.PHONY: git-tool
git-tool: ## 启动 Git 交互式管理工具
	./scripts/git-tool.sh
