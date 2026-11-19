# ForestScene PC 双机完整探索交付

当前 PC 交付见 [双机运行与验收](PC_DUAL.md)。下文保留此前版本的记录。

版本 `0.2.0-forest-complete.20261119`。两轮同配置的原生完整探索、实际轨迹执行、停止降落和资源清理通过；整体状态保持 `partial`，第二轮辅助记录读回不完整，板端混合部署等原适用项尚未完成。版本身份见 [锁定清单](forest-complete.lock.json)，分项结果见 [验收摘要](forest-complete-validation.json)。本次固化没有修改飞行代码、控制参数、算法策略或共享镜像，也没有新起飞。

整理方式参考 [CAT-ORA 的运行入口与生命周期](../catora_open_field/README.md) 和 [SUPER 的四方职责、验收边界](../super_forest/README.md)。只复用组织方式，不继承它们的车辆数量、算法参数或历史执行链。原 [短程版本](FOREST_SHORT.md)、Warehouse 失败记录和此前的交付审查均保留。

## 部署与版本

宿主 UE/AirSim 提供 ForestScene 和真实 DepthPlanar；两个独立 PX4 SITL 提供飞控；PC Docker 中每机原生 RACER/Adapter/AS2 提供算力能力，工作站容器通过 ROS 调用。按角色使用 [四个固定镜像](../racer_single/images.lock.json)，镜像 ID 纳入本版本清单，外部 overlay 提供补编译内容。机数、RACER ID、ROS namespace、PX4 instance/system ID、AirSim vehicle 对应在 [fleet 配置](forest-complete.fleet.json) 中；PC 进程监管复用 `deployment/swarm`。当前没有完成一机 RoboPi、另一机 PC 的混合拓扑。

实际链路：DepthPlanar 与 AS2 里程计 → 原生地图/frontier/协同分配/B-spline → RACER Adapter → FollowTimedPath → AS2/PX4。完整模式的任务结束监督由 [COMPLETE_EXPLORATION.md](COMPLETE_EXPLORATION.md) 说明；任务脚本不是通用 Mission/Tool 运行时。

构建候选为 `20261119-frontier-diagnostics-01`，上游锁定 `f8265e091b11cc856c1334f8260850b9006b1953`。编译源码、2075 项安装/依赖产物与实际运行 Python 分开关联。构建 manifest 保留历史命令、父候选及当时 `actual_ground_flight: not executed` 的记录；飞行证据另由本次摘要关联，不能改写旧 manifest 冒充当时已测试。运行监督 Python 后于构建调整，其身份来自两轮相同的 `runtime-source.json`，而非旧构建字段。原始运行身份表保存在 [forest-complete.runtime-source.json](forest-complete.runtime-source.json)，本次变动的说明文件及新增校验属于交付元数据，不冒充飞行时运行代码。

## 固定用例与完成含义

本用例规划 ENU 范围 X `[-6,7]`、Y `[-6,4]`、Z `[0.25,1.5]`；执行边界及速度等见 [flight 配置](forest-complete.flight.json)。每机探索最长墙钟预算 300 秒，停止/降落使用独立预算。这些值只用于复现本例，不是其他项目的迁移门槛。

模式必须显式为 `explore-complete`。当前 fleet/flight 的 `scope` 已修正为 PC 双机完整探索；与实际测试输入相比只改说明字段，运行参数不变。锁定清单分别保留测试时输入哈希和本次说明修订后的交付哈希，原测试配置与证据不改写。`explore_wall_seconds: 20` 仅供 `explore` 短程分支使用；完整模式采用 local JSON 的 `completion_wall_budget_s: 300`，不会 20 秒自动结束。仓库默认 fleet/flight 仍属旧短程入口，本版本必须显式选择两份完整配置。

| 运行 | 飞行/停止降落/清理 | 跟踪阶段墙钟约数 | 激活轨迹数 D0/D1 | 执行拒绝 D0/D1 | 辅助记录读回 |
|---|---|---|---|---|---|
| forest-complete-01 | 通过 | 155.60 秒 | 10 / 33 | 0 / 0 | 通过 |
| forest-complete-02 | 通过 | 163.21 秒 | 19 / 38 | 0 / 0 | 未通过 |

两轮两机均观测到本轮触发后的 `return finished`、STOPPED 反馈应用及原生 FINISH，工作站锁存全队自然结束后收尾，结果 `termination_reason=native_finished`。不是预算耗尽，也不以休眠、No grid 或无人机暂时不动推断结束。真实运动及 AS2/PX4 参考有对应证据；两轮均确认原生 Land 和新鲜 landed/disarmed。**原生任务完成不等于整个 ForestScene 或所有体素已覆盖。**

第二轮 Drone1 原生 plan 读回 36 条，记录器写入/接收账为 38，query 对应的 plan ID 33、36 缺少 bag 消息；Adapter 有其激活事件。检查项 `all_cdr_messages_decode` 的失败还包括读回数与写入/接收账不一致，不能仅凭名字声称存在 CDR 解码异常。保留原失败，不删除该验收项，也不把它改写为飞行失败。

历史地面 inputs 退出 `-11` 原因仍未查明；隔离回放、后续地面及本版本两轮退出正常不能证明已根治。整套 UE 资产尚未按哈希锁定；磁参考差异未修复，平移式 ENU 配准的适用条件仍保留。四机飞行、RoboPi、完整故障注入、跨版本回退飞行、全阶段碰撞事件统计和长期可靠性均未由这两轮覆盖。

## 固定版本复验

在 AeroHub 仓库根目录的 PowerShell 中执行，主机需 Python 3.11+、可用 Docker 和上述已有镜像。UE 工程与 ForestScene 为外部前置资源。不要把下载源码、镜像存在或文件检查通过当作真实场景就绪。

将 [本地配置示例](forest-complete.local.example.json) 保存到不纳入 Git 的本地位置，填写 release、settings、新 run_root、project、未占用 DDS domain、Docker 可达的 AirSim 地址和不冲突的子网；两个配置路径填写本仓库内相应文件的绝对路径，或相对于 local JSON 的路径。示例的占位字符串必须换为实际值，domain/rpc_port 为数值。参数与地址不写回共享默认值。

以下 `check_complete.py` 专用于复验这个已命名的固定用例：只读检查源码、release 清单、安装产物、settings、所选配置、300 秒预算及四个镜像。它不是通用的新环境就绪检查；固定用例不匹配不能单独证明新环境不兼容。

```powershell
python -B deployment/airsim/demos/racer_multi_flight/check_complete.py --local <local.json> --images
```

退出码必须为 0。校验不启动任何容器，不证明 UE 实际加载的地图、settings 或网络资源就绪。确认 ForestScene `/Game/RacerDemo/Maps/ForestScene`、正确双机 settings、上一轮 landed/disarmed 及所属服务退出，再通过 UE Stop/Play 刷新会话。settings SHA 只锁配置文件，不锁整个 UE 工程。

以下命令会实际起飞，仅在本轮已授权且前置检查完成后执行：

```powershell
python -B deployment/airsim/demos/racer_multi_flight/full_flow.py --local <local.json> --mode explore-complete
```

复用原正式入口：分阶段启动 → 真实地面就绪/输入/起飞柱检查 → 原生算法与 Adapter → ULog → 双机起飞探索 → 全队原生结束 → 停止/原生 Land/landed-disarmed → 日志与容器清理。没有自动追加复验或扩大区域。两轮主机包装器额外做了场景确认和输入进程崩溃诊断环境设置；它们不提供规划或控制能力，不是正式运行依赖。直接 CLI 的编排方法和运行代码与两轮一致，本次未为整理文档另飞一轮。

## 更换机器或准备新配置

采用与 CAT-ORA/SUPER 相同的职责划分：安装位置、设备地址等由本地配置提供，任务/机群参数保存在所选用例配置，实际运行值随证据记录。不要求其他人采用本机磁盘布局，也不将本例预算、端口或车辆编号作为通用标准。

- 只改变仓库、release、settings 文件所在路径，或本地 run_root、project、DDS domain、AirSim 主机地址、Docker 子网时，填写新的 local JSON；源文件与用例内容未变可继续进行固定版本身份检查。检查器不判断网络是否可达、端口是否空闲或新机器资源是否充足。
- 若需修改 settings 内的连接端口、车辆/相机配置，或 fleet/flight 的车辆映射、规划范围、运行预算等，则建立单独的本地候选配置，明确记录相对本例的差异。旧哈希不匹配是预期的版本差异，不通过修改本基线清单或忽略错误来获得通过。
- 新环境先核对固定镜像与 overlay、Docker 挂载与网络、车辆/端口映射、传感器和坐标/时间约定；再按既有分阶段部署入口做真实地面就绪与接口验证。`full_flow.py` 会起飞，不是纯地面检查命令。当前未提供一个可以单独证明新环境已就绪的通用检查命令。
- 地面验证完成后，按变动的影响确定所需飞行及验收项，生成该候选自己的配置、源码/产物身份和分项结果。旧两轮只作为参考，不能自动算作新配置已通过；验证完成后再命名新的基线。

因此，固定版本校验的 0 退出码仅表示身份匹配；新环境是否可用依赖其实际地面与适用运行验证。本次整理未验证任何新的机器、端口组合或飞行参数。

## 检查结果、停止与回退

核对 `flow-result.json`、`mission/result.json`、`evaluation.json`、`cleanup-result.json`，记录验证单独看 `recording-result.json`。离线命令：

```powershell
python -B deployment/airsim/demos/racer_multi_flight/evaluate.py --run-root <run-root>
python -B deployment/airsim/demos/racer_multi_flight/readback.py --run-root <run-root> --release <release> --output <new-readback-output>
```

`evaluate.py` 会写 evaluation.json；审查历史证据时保留原件或使用已有归档结果，不覆盖已归档结论。读回使用新的输出目录，证据只读挂载、网络隔离；检查退出码与分项结果，不能只看文件生成。

正常入口自动收尾。需要中断时核对本轮 `<project>-mission` 的 `aerohub.racer.owner` 标签，再向该容器发送 `docker kill --signal=SIGINT <project>-mission`，等待任务的停止及降落恢复。不要先停时钟、PX4 或 UE；人工中断飞行不属于本版本两轮已验证项。无法确认地面状态时保留基础链路，不强制断电或解除武装。

`run.py stop/down` 是地面资源清理，不是空中停止命令。两机均确认 landed/disarmed 后，按 flow-stages.json 补做尚未完成的步骤：

```powershell
python -B deployment/airsim/demos/racer_multi_flight/run.py --local <local.json> logger-stop
python -B deployment/airsim/demos/racer_multi_flight/run.py --local <local.json> stop
python -B deployment/airsim/demos/racer_multi_flight/run.py --local <local.json> down
```

回退是停机选择旧版匹配源码、配置及 release，重建场景会话和地图后重新检查；仅更换 release 不成立。旧短程版使用它自己的清单与源码，当前完整探索代码不能直接继承旧短程通过结论。尚无自动热回退，也未测试跨版本回退飞行。

## 构建、证据与交付边界

现有候选身份未变，本次不重建共享镜像或编译产物。需重建时使用已有入口：

```powershell
python -B deployment/airsim/demos/racer_single/build.py --release <new-release> --deps <locked-dependency-cache>
```

依赖缓存须匹配 [dependencies.lock.json](../racer_single/dependencies.lock.json)。重建会产生新的 manifest，不能要求它与此增量候选的产物哈希自动相同，更不能直接沿用飞行通过结论。本次没有从空环境全量重建；镜像、依赖缓存和 UE 资产的可获得性仍是外部前置条件。

精简结果和关键证据 SHA256 在本目录的验收 JSON；大体积 bag/ULog 留在原数据盘，正式归档索引位于 `log/racer-ros2/20261119-complete-exploration/` 和 `log/racer-ros2/20261119-complete-repeat/`。获取原始证据时使用各自 index/raw-index 定位并校验；仅复制 Git 源码不会附带这些被忽略的大文件。实际本地位置在过程记录中，不作为共享入口的必需硬编码。

本次复用可复现构建、组件部署、端到端验收与能力卡片 Skill 已有判断，不新增 Skill，也不修改其他项目内容。此前 RACER 沉淀仍有效；本次完整结束、等待状态与记录缺项的处理已有覆盖，无需再写一套重复规则。
