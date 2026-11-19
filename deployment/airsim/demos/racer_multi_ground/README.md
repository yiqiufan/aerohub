# RACER 多机地面接入候选

当前默认 **双机、PC、仅地面影子验证**。后续候选 `20261117-stop-01` 的双机地面整轮（任务与清理）通过，见 [stop-diagnosis.json](stop-diagnosis.json)；不是多机飞行或完整复现通过。原双机/四机失败保留，旧双机摘要为 [acceptance.json](acceptance.json)，历轮对照见 [validation.md](validation.md)。本入口没有解锁、起飞或发送运动 Action 的命令。

## 架构与接口

- AirSim 提供场景和真实深度；flight-board 运行每机 PX4 SITL、数据桥与唯一 clock。每机 AS2、navigation 使用已锁定 onboard 镜像。工作站镜像运行 `mission.py`，不承载算法或高频控制。
- `src/planner/racer/ros2_adapter/multi/group.py` 复用 `deployment/swarm/orchestrator.py::ServiceThread`，在 inputs/native/shadow 各角色容器内，每机由独立线程监管独立进程；native 每机另有真实 FSM 与两类求解器子进程。当前全部在 PC；不满足最终恰好一个 RoboPi UAV 的混合部署要求，不能据此宣称该矩阵通过。
- `fleet.json` 明确 AirSim 名称、AS2 namespace、原生 RACER ID、PX4 instance/system ID 与 XRCE 端口。双机的 RACER ID 为 1、2，`native_slots=3`。这是此版本最后一个 ID 预留 ground-map slot 的约束，依据 `native/src/plan_env/src/multi_map_manager.cpp::stampMsgCallback`，不是 AeroHub 通用的机数规则。四机配置保存在 `fleets/four-ground.json`，双机副本在 `fleets/two-ground.json`。
- 原生 `/swarm_expl/drone_state`、`pair_opt`、`pair_opt_res`、`/planning/swarm_traj`、`/multi_map_manager/chunk_stamps`、`chunk_data` 的 send/recv 映射到各自共享话题；其他传感器与触发按机隔离。不能仅靠话题出现证明协同：本用例匹配请求/响应双方 ID 和 stamp，再核对发起方实际消费日志；地图还核对接收方后续广告的外来 chunk ID。
- `inputs.py` 使用各机实际 AS2 里程计、真实深度和相机外参。地面登记由 AS2 本地初始位置与配置中的已知出生点确定固定平移，前提是各机 ENU 轴向一致、场景出生点已核对、期间不重置定位。公共地图几何相同；voxel address 交换不能只改 frame 字符串。AirSim 位姿仅用于场景核对/观察相机，不馈给算法定位。
- `/racer_multi/<namespace>/odometry` 是 RACER 私有输入约定：姿态保留机体姿态，速度旋转到公共 ENU；不是新的权威 AS2 Odometry/TF。相机 pose 与 depth 保留同一采样时间。通用旋转配准、定位重置与板端坐标同步尚未验证。
- `shadow.py` 接收 `/racer_execution/drone_<id>/plan` 的真实 Bspline，将**全部**位置控制点平移回该机 odom，保留 knots、绝对时间、PVA/yaw，构造该机 `FollowTimedPath` 请求并检查实际 Action/modify/stop 就绪。它不调用发送 Goal/modify，明确回报 REJECTED；首个有效候选后请求 OPERATOR_STOPPED，防止持续重规划。不能把构造请求写成 AS2 执行接受或运动完成。
- 当前共享观察相机由单机入口的 `overhead.py` 与 swarm view 实现驱动。地面记录只证明周期更新/读回，不证明运动中动态质心跟踪。

## 构建与运行

使用 `../racer_single/images.lock.json` 的四个镜像 ID，`../racer_single/dependencies.lock.json` 的局部依赖和 `build.py` 生成的外部 release。`up` 校验 native/interface 源码、产物和依赖 hash；初版多机复用单机 release；停止修正涉及 native C++，后续使用独立完整重建候选 `20261117-stop-01`。Python 适配与运行源码由每轮 `source-snapshot/`、`runtime-source.json` 单独锁定。依赖安装只发生于本轮可丢弃容器可写层，共享镜像不修改。

先核对本 Demo 与各父目录约定。所有容器须已停止后才选择 fleet；本版各命令会读取当前 `fleet.json`，**运行中不要修改它**。将实际个人设置保存为本地 JSON（不提交），字段：`run_root`（新目录）、`release`（已构建候选）、`project`（唯一前缀）、`domain`（独立 DDS 域）、`airsim_host`、`rpc_port`、`settings`。场景实际车辆/相机/系统 ID/出生点须与 fleet 一致；修改 AirSim settings 后需要重开场景会话。

在 Windows PowerShell、仓库根目录执行，`$localConfig` 替换为个人配置路径：

```powershell
python deployment/airsim/demos/racer_multi_ground/run.py --local $localConfig up
```

确认各 PX4 实例已启动。实际场景核对脚本 `platform_probe.py` 应在 flight 镜像、相同挂载/网络中运行；本轮通过 `Deployment.run('platform', images['flight'], ROS+'exec python3 /code/platform_probe.py', False)` 执行，具体命令保存在每轮 `platform/command.json`。本版尚未提供独立 platform CLI 子命令。

```powershell
python deployment/airsim/demos/racer_multi_ground/run.py --local $localConfig sensors
python deployment/airsim/demos/racer_multi_ground/run.py --local $localConfig start
python deployment/airsim/demos/racer_multi_ground/run.py --local $localConfig mission
python deployment/airsim/demos/racer_multi_ground/run.py --local $localConfig stop
python deployment/airsim/demos/racer_multi_ground/run.py --local $localConfig down
```

逐步检查退出码。`mission` 或 `stop` 失败不代表清理已完成；独立检查 `cleanup-result.json`，只有所属 worker 都已停止、实际 PX4 landed/disarmed 可确认时再 `down`。本用例没有自动启动重试，PX4 启动失败先保存 inspect/log；容器不存在/已退出会返回明确的服务名错误，启动阶段同时保存失败快照。不要盲目重启整组或删除旧结果。`down` 停止而保留容器与证据；回退旧 release/config 时，还需在隔离源码树恢复与其 manifest 匹配的对应源码，再在新会话、新目录重新启动；不能仅切换 release 路径绕过源码 hash 校验。既有单机证据保留，本轮未执行旧版本回退或新候选单机飞行复验。

## 验收与适用范围

运行前 fleet 固定了地面就绪、观察及停止预算；相机 1 Hz 是本地诊断负载配置，不是 RACER 通用要求，未更改算法原生 peer freshness 或规划门限。`mission.py` 检查真实地图/frontier、双机原生状态/轨迹广播、配对优化消费、外来地图块消费、每机真实候选请求构造、停止确认、始终 disarmed。它不检查飞行 preflight 充分性，不得用于授权飞行。

测试源码 `multi/test_contract.py` 在锁定 onboard 镜像加载实际 ROS/AS2/native overlay 后执行：非零坐标变换的 PVA/yaw/时间保持、单机相机默认接口与多机参数、真实停止订阅及仿真时钟。3 项通过不代表多机整轮通过。最终同时核对任务、实际子进程退出和基础容器清理；任一失败保留失败。最终多机飞行、机间避碰、完整探索覆盖、板端混合及故障恢复均未覆盖。

## 分阶段启动固化

`up`先启动clock及fleet内各机PX4/Agent，沿用60秒就绪等待预算：检查这些容器仍运行、每个实际PX4实例landed/disarmed且DDS connected，再启动各机AS2/navigation。按fleet生成服务列表，没有逐机重启兜底；`startup-result.json`记录阶段、时间和错误。该结果只表示依赖启动，不等同AS2任务就绪、飞行或完整RACER复现通过。

容器退出或就绪超时会阻止第二阶段；第二阶段启动命令失败也返回失败。自动保存`startup-failure-containers.json`及各存在容器的`*-startup.log`，保留服务供核对实际状态后受控清理，不自动重启或强行停飞控。已退出/不存在的core服务返回明确错误，不再执行空容器ID。Docker探测调用有超时，正常启动后仍由任务/地面验收核对实际消息和能力。

临时方案两轮地面冷启动通过后，20261117-formal-startup-01直接调用正式飞行部署CLI的`up`（继承同一实现），无启动拦截。两机实际PX4状态、DDS及AS2/导航跨容器探针通过，12秒clock605个唯一时间戳，里程计600/599；最终landed/disarmed、7个容器退出0。镜像、飞控参数和12个settings/build/configuration工件与临时复验一致。报告见`../racer_multi_flight/startup-validation.json`。

6项启动回归覆盖就绪顺序、容器崩溃、地面超时、DDS未连接、第二阶段失败、按fleet生成服务；另有4项完整流程回归。合成故障回归不代表真实崩溃根因已修复。实际新入口仅验证PC双机地面，四机和RoboPi仍待验证；旧正常启动失败与降落失败记录保留。


## 可选双机诊断记录

原始深度、CameraInfo、输入位姿/里程计、占据/膨胀地图与原生查询值的记录使用
[diagnostics.md](diagnostics.md)。`diagnostic_flow.py --local <配置>` 调用本入口完成
地面 shadow、停止、清理及 rosbag2 载荷读回；不解锁、不起飞。配置中的
`diagnostics.enabled` 默认关闭，启用时也不改变原规划或碰撞判据。
飞行和板端验证范围与记录器地面验证分别判断。
