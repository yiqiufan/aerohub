# ForestScene 双机在团队 dev 上的开发验证

团队基线为 `13b7db0`，对照版本为 `6190575`。构建使用 `src/tool/navigation/planner/racer` 下的算法和适配器。运行参数沿用原完整探索用例。573 个原生源码与工作站接口编译输入按重定位后的相对路径逐文件对应。新版传感器桥的六个 Python 模块与 R1 运行版本的文本一致，构建直接取当前源码。旧版传感器补丁只用于追溯旧基线。

开发构建从本仓库的 racer_single/build.py 开始，使用新的 release 目录和经 dependencies.lock.json 核对的依赖缓存。隔离检查比较实际渲染配置、传感器测试、多机编排测试和两机的 ROS 接口。结果见本地 log/racer-ros2/dev-13b7db0-offline/，仿真结果以本工作区对应的新运行记录为准。旧版锁定清单和验收数字保留在原版本。

该入口沿用开发工作区挂载。正式交付还需按当前 AGENTS.md 核对修改过的上游源码来源、派生镜像和现行能力卡片规范。开发构建和本地仿真分别记录，不能代替对最终交付形态的验收。

## 同配置双机复验

地面运行 `forest-dev-13b7db0-ground-01` 通过 22 项就绪检查。两个飞行轮次使用相同的构建清单、机队配置、飞行配置和 AirSim 设置。规划区水平尺寸为 13 × 10 米，每机探索预算 300 秒。

| 运行标识 | 飞行与清理 | 诊断读回 |
|---|---|---|
| `forest-dev-13b7db0-complete-01` | 跟踪约 7 秒时采样间距为 0.947 米，触发 1 米保护。两机确认停止、降落和解除武装，Drone0 相机退出码为 1，需补做基础服务清理。飞行验收失败 | 通过 |
| `forest-dev-13b7db0-complete-02` | 跟踪约 151 秒，采样最小间距为 1.614 米。两机均原生结束、停止、降落并解除武装，飞行与清理通过 | Drone0 三个地图话题各有两条零时间戳空云，正时间戳的 1,128 条均匹配原生地图发布记录。Drone1 的 2,709 条全部匹配。辅助读回判定失败 |

本基线未复跑 R2–R5 或测量场景覆盖率。能力卡随后对齐到 Schema v1，独立静态校验通过，覆盖 26 张卡、34 种数据类型和 7 种硬件类型。

### 原始记录校验值

下表路径相对于各轮独立运行目录。校验算法为 SHA-256。

| 运行标识 | 记录 | SHA-256 |
|---|---|---|
| `forest-dev-13b7db0-ground-01` | `readiness/result.json` | `3c5b03fd73752aa3ba7f4dfda5e6c71e90c12ee135bd0b79729c67f749659849` |
| `forest-dev-13b7db0-ground-01` | `ground-flow-result.json` | `750352f6cb939e348d145d87832a97b3378ce45417fc3d4582e3038d3804f240` |
| `forest-dev-13b7db0-complete-01` | `build-manifest.json` | `e367a7fba3d311d52eec68090a37f7e2c4da85936f86f2976c4cee0ccb9fe21a` |
| `forest-dev-13b7db0-complete-01` | `mission/result.json` | `2e3af6bacab29d19ed6229127fc8265ee699ae3386e0a48b4b1c6555c93e60cf` |
| `forest-dev-13b7db0-complete-01` | `evaluation.json` | `f63a37c072a9cc13f586d28791ae0a4580fc37d4e34a241017c27bad84db98ce` |
| `forest-dev-13b7db0-complete-01` | `cleanup-result.json` | `658e40a4ecbea150b6c8422d71bacc2e88b49eabf0ff53479ad85e28da14eb1e` |
| `forest-dev-13b7db0-complete-01` | `recording-readback/recording-result.json` | `d1e293d12bb429b359a28c1c360a590880a0dbec392bc621e840d9e6515a1838` |
| `forest-dev-13b7db0-complete-02` | `build-manifest.json` | `e367a7fba3d311d52eec68090a37f7e2c4da85936f86f2976c4cee0ccb9fe21a` |
| `forest-dev-13b7db0-complete-02` | `mission/result.json` | `41bd7add4da881ef520a469d6efc25c73dfc2614ab52d213d74e44c3ec5a33f8` |
| `forest-dev-13b7db0-complete-02` | `evaluation.json` | `99de0c40023d0b06a828a4f7c3c471857b6c21fc288216c0e4dc573b8bad37f2` |
| `forest-dev-13b7db0-complete-02` | `cleanup-result.json` | `a8c3ed63226eb4286028bd9f28535995537522d27aa2629b781671f91ec7d13c` |
| `forest-dev-13b7db0-complete-02` | `recording-readback/recording-result.json` | `aaeb8afb504a1e00fb76d82a01003713598232d53f152cb29e755334defe4468` |
