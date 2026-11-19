# RACER 单机原生探索接入

将原生 RACER FSM、TSP/ACVRP、滚动 B-spline 与 AeroHub 的 FollowTimedPath 接通。工作站只调用已部署的机载能力，传感器与状态直接进入机载算法；不通过工作站转发运动参考。当前是固定映射的单活跃无人机示例，不能当作多机通用编排器。验收状态见 [validation.md](validation.md)，组件与能力契约见 [component.yaml](component.yaml) 和 [能力卡片](../../../../cards/planner/racer-single.yaml)。

## 源码、镜像与依赖

实际源码在 `src/planner/racer/ros2_adapter/{native,bridge}`；上游提交及导入基线在 `upstream.lock.json`。构建不读取旧实验目录、旧算法 install 或外部 checkout。原始四机 pillar 工具保留在 native 中，不代表本入口已完成四机 AirSim 验收。

`images.lock.json` 锁定四个已有 linux/amd64 镜像 ID：算法工具环境、机载 AS2/执行环境、flight-board 和工作站 ROS。它们是显式外部前置条件，不由本入口从零生成；缺少时先从团队渠道取得匹配镜像。原镜像不被修改，额外 NLOpt 只安装于短命构建/算法容器层。镜像来源与已安装软件完整表写入构建清单，不宣称可从空机完全独立构建。

额外依赖缓存含 `nlopt-debs/` 与 `nlopt-debs.sha256`，锁定 Jammy 的 libnlopt0/libnlopt-dev/libnlopt-cxx0/libnlopt-cxx-dev `2.7.1-3build1`。已有验证缓存可以复制复用，先检查 SHA256，禁止无记录升级依赖。配置中的 `.65 m/s` 是原生逐轴规划软代价尺度，执行范数检查 `1.5 m/s` 是此示例的独立范围，不是同一数学约束。

传感器平台代码复用 `simulator/airsim/sensors`，按 `patches/sensor-lock.json` 核对基线后在候选输出目录应用 `sensor-compat.patch`。此兼容分支保留已验证的图像采样时间、可靠 QoS、时序记录与 RPC 行为，不覆盖公共桥。后续公共化需要独立审查其他消费者。`workstation_interfaces` 是与固定机载镜像逐项对齐的消息/Action 子集，工作站自行编译类型支持，不复制不同环境的动态库。

## 构建与选择候选

以下在仓库根目录运行，路径均由使用者选择；Windows 可把大型 build/install 放到有空间的数据盘。已存在候选目录不会覆盖。

```text
python deployment/airsim/demos/racer_single/build.py --release <new-release-directory> --deps <reviewed-NLopt-cache>
```

入口在两个固定镜像内分别干净构建原生算法与工作站接口。`manifest.json` 记录源文件、镜像、实际命令、sensor 补丁及安装文件哈希；`native-build.log`、`interfaces-build.log` 和 `native/packages.tsv` 保留构建证据。失败目录保留，改正后使用新候选目录。首次编译按现有完整 RACER workspace；没有为单机运行删除原版能力。

创建不纳入版本控制的 local JSON，至少包含：

```json
{
  "run_root": "<new-run-directory>",
  "release": "<built-release-directory>",
  "project": "racer-single-example",
  "domain": 94,
  "airsim_host": "<address-reachable-from-Docker>",
  "rpc_port": 41461,
  "settings": "<actual-AirSim-settings-file>"
}
```

domain、地址、端口、project 与场景必须按本地已有部署检查，示例数字不是公共要求。当前入口明确固定 Drone0/sysid135/instance0 与停放 Drone1/sysid136/instance1，并占用 UDP 14583/14584；扩展映射须同步核对配置与节点，不能仅改 JSON 就声称支持多机。配置源在 `config/`；运行启动时冻结一份到 run_root。

## 会话、地面检查与任务

准备有屏 AirSim 场景及实际深度相机。当前示例用 640×480、水平 FOV90°、前视相机杆臂 0.46m；必须与平台实际配置/TF 一致。复用 `deployment/swarm/view.py` 为 settings 加入 external camera，`overhead.py` 调用共享动态更新函数，先把每架机体局部原点换算至统一世界系。此视图不读取碰撞事件，也不控制飞行器。

若旧会话留下 Offboard/预检状态，先在确认落地解除武装后停止其拥有的 PX4/AS2/桥，刷新 UE Stop/Play，再启动新会话。不要中断他人部署。下列命令前均加 `python deployment/airsim/demos/racer_single/run.py --local <local.json>`：

| 命令 | 行为 |
|---|---|
| `up` | 校验候选来源与安装哈希，生成本轮 Compose 并启动 PX4/时钟/AS2/导航 |
| `sensors` | 启动深度桥、相机位姿与速度适配、只读真值观测、动态俯视视图 |
| `start --case ground-01` | 记录两机实际 PX4 地面状态与 LOS；启动原生算法和不下发动作的影子 Adapter |
| `mission --case ground-01` | 等真实图像/地图/frontier/动作接口就绪，触发原生规划，确认曲线和逻辑停止 |
| `stop --case ground-01` | 再确认两机 landed/disarmed，再停止本轮 Adapter/原生进程并检查退出码 |
| `start --case flight-01 --fly` | 用新 case 启动允许真实执行的 Adapter，保存源码快照 |
| `logger-start --case flight-01` | 地面确认后替换本实例已有 idle logger，启动本轮文件日志；检查 logger-before.log 与 logger-start.log 的新路径 |
| `mission --case flight-01 --fly` | 原生 Takeoff → 有界自主探索 → stop/FINISH → 原生 Land → 联合等待 landed/disarmed |
| `logger-stop --case flight-01` | 地面确认后导出本轮 ULog；日志不是旧轮替代品 |
| `stop --case flight-01` | 地面确认后检查原生与 Adapter 的实际退出 |
| `stop-sensors` | 地面确认后回收本轮四个传感器/观察进程，并分别保存退出码 |
| `down` | 确认两机地面且本任务临时容器均停止，再停止基础 Compose；保留容器与日志以供审计 |

`start` 成功只证明进程已启动；任务自带真实数据/接口就绪检查，不能用容器健康灯代替。任务运行中需要人工中断时执行 `cancel --case flight-01`，向本任务进程发送 SIGINT 触发停止与落地恢复；继续等待结果，不直接停 PX4、时钟或 UE。取消仍是已有恢复路径，是否通过故障验收另见证据。

基础 navigation 使用锁定镜像中的实际节点二进制并通过 SIGINT 退出；down 检查包括它在内的全部五个基础服务退出码。此选择经过本镜像实际启停验证，不是对其他 ROS 启动器的通用要求。

只在 case 无源码变化时运行任务。切换源码后建立新 case，保留旧结果。当前任务按既有声明探索 20 秒、至少两个实际激活版本与 0.5m 水平位移；这些数字是本场景短程工程判据，不应复制为其他项目标准。任务退出码与原生子进程、容器清理结果分开评估。

## 停止、更新与回退

轨迹停止使用目标版本的 Action 终态、停止服务成功和请求后的 PID POSITION 保持反馈；不凭 POSITION 枚举单独确认。Land 成功后在原 10 秒预算内同时等待新鲜 landed/disarmed，顺序不固定。停止失败仍独立尝试原生 Land；必要时现有 PX4 NAV_LAND 恢复单独记录，不算原生 Land 成功，不强制 disarm。

更新时创建新 release，完成地面校验后选择它；旧候选和运行证据不删除。回退前先 cancel/落地/停止进程，恢复已审阅的 runtime-source 快照及匹配 native/interface 源码，选择原 release，使用新的 run_root/project 并刷新场景。`up` 校验实际 native/interface 源码和安装哈希，拒绝把不同版本混合。地图与 FSM 重新初始化，不保留探索进度，不宣称热切换或自动注册中心已实现。本次是首个正式候选；跨两个已验收正式版本的回退尚未实测，操作说明不作为回退成功证据。

## 证据与限制

每轮保存 Docker 命令、配置、运行源码快照、原生求解器日志、完整曲线、Action/停止反馈、AS2/PX4 参考、物理观测、ULog 和所有退出码。离线分析需要 Python 的 `pyulog` 与 `numpy`（本次解析版本另见 validation）；可通过 `--python-deps` 指定已锁定依赖目录：

```text
python deployment/airsim/demos/racer_single/analyze.py --root <run-directory> --case flight-01 --python-deps <locked-analysis-dependencies>
```

全部停止并完成分析后运行汇总检查：

```text
python deployment/airsim/demos/racer_single/verify_delivery.py --root <run-directory> --case flight-01 --ground-case ground-01
```

汇总检查保留本会话的历史非零退出，因此修复后需从新的 run_root/project 完整复验；不要删除失败容器或修改旧结果来获得通过。

活动飞行范围与停止/接触阶段的诊断窗口按验收声明区分。运动期间不轮询当前插件的清除型碰撞事件 RPC，场景碰撞仍开启；因此没有飞行碰撞事件统计。原生 no-frontier/FINISH 不作为整图覆盖证明。四机协同、RoboPi、故障注入及真机均未由此单机入口证明。
