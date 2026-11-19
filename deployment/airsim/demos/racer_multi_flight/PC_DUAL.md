# RACER 双机 ForestScene（PC）

版本 `0.2.1-pc-dual.20261119`。两轮正常探索均由两机原生 FSM 结束。两机均有真实轨迹执行、停止确认、原生降落、PX4 落地和解除武装记录。适用范围为指定场景、区域、传感器和 PC 容器拓扑。[能力卡片](../../../../cards/planner/racer-dual.yaml) · [组件清单](component.yaml) · [验收摘要与原始证据校验值](pc-dual-validation.json)

## 部署与准备

宿主 UE/AirSim 提供 ForestScene 和 DepthPlanar。两套 PX4 SITL 分别对应 Drone0、Drone1。PC 容器提供两机的原生 RACER、Adapter、AS2 和导航；工作站经 ROS 发起任务。原生前沿探索和协同分配产生 B-spline，Adapter 保留时序和中间点交给 FollowTimedPath，由 AS2/PX4 执行。[车辆与命名空间映射](forest-complete.fleet.json) · [镜像身份](../racer_single/images.lock.json)

使用可用的固定镜像、匹配的构建 release、双机 settings 与外部 UE 场景。源码、依赖及构建命令见[原完整探索基线](FOREST_COMPLETE.md#构建证据与交付边界)。本版本复用其原生安装产物，更新可选诊断进程。将[配置示例](forest-complete.local.example.json)另存为本地 JSON，填写主机可达地址、可用网络、release、settings 和新的 run_root。显式选用本目录的 fleet、flight 配置。每机 300 秒是这例正常探索的最长预算。

从仓库根目录执行下列命令。先运行只读身份检查，核对[本版本差异清单](pc-dual.lock.json)及其引用的原锁。运行环境还须确认 UE 实际加载 ForestScene 和匹配 settings，按原流程准备新会话。

```powershell
$demo = 'deployment/airsim/demos/racer_multi_flight'
$localConfig = Read-Host '本地配置 JSON 路径'
python -B "$demo/check_complete.py" --pc-dual --local "$localConfig" --images
```

## 运行与停止

以下仿真入口会起飞。它依次完成分阶段启动、地面就绪检查、双机探索和收尾。

```powershell
python -B "$demo/full_flow.py" --local "$localConfig" --mode explore-complete
```

正常完成要求本会话两机均收到原生结束反馈。工作站监督停止和 Land，随后核对每个 PX4 的 landed/disarmed，再清理本次拥有的服务。若流程中断，先查看 run_root 中的 flow-stages.json 和 PX4 状态，按[地面清理与回退说明](FOREST_COMPLETE.md)处理。旧版本复验须使用其匹配的源码和原锁。

固定验收用例启用可选记录器。共享运行入口从当前仓库只读挂载 multi 目录，记录器随 diagnostics 角色启动。关闭诊断可用于普通运行；本页的记录完整性结果只覆盖开启诊断的固定用例。与 R1 的区别是仓库挂载替代了测试目录挂载。R1 包装器还临时开启 inputs 的 Python 崩溃堆栈并确认 UE 地图。打包核对使用历史飞行数据和离线入口检查。

飞行结束后，从本轮目录复核运动与清理，再从匹配 release 做 ROS 类型读回。evaluate.py 会写入指定运行目录的 evaluation.json；历史原始目录应保留。

```powershell
$case = Get-Content -Raw -LiteralPath $localConfig | ConvertFrom-Json
$readback = Read-Host '新建读回输出目录的路径'
python -B "$demo/evaluate.py" --run-root "$($case.run_root)"
python -B "$demo/readback.py" --run-root "$($case.run_root)" --release "$($case.release)" --output "$readback"
```

## 当前 PC 验收

| 用例 | 任务与恢复 | 辅助记录 |
|---|---|---|
| R1 正常完整探索 | 同配置两轮两机均 native_finished。收尾轮 Drone0/1 激活 14/22 个轨迹版本，拒绝均为 0，均已停止、降落和解除武装 | 两轮通过。收尾轮分别读回 81,160 和 81,570 条 |
| R2 主动取消 | tracking 中取消后，双机均保留 cancelled，完成停止、降落和清理 | 通过 |
| R3 预算耗尽 | 首轮先触发间距保护。复验时 Drone1 实际耗尽 20 秒预算，Drone0 随同伴失败收尾。初判漏识别正在处理的同一停止请求，复核其确认和 Land 顺序后通过 | 两轮读回通过 |
| R4 单机输入中断 | Drone1 输入陈旧触发失败，Drone0 联动恢复，双机落地解除武装 | 通过 |
| R5 单机探索进程退出 | Drone1 原生进程退出 -9，无原生 FINISH。执行层确认停止，两机原生 Land、解除武装。测试专用核对允许预期的 native 容器退出 1 后继续受控清理 | Drone1 的辅助读回遇到 JSON 解析错误，记录完整性未通过 |
| R6 全阶段接触观测 | 缺少已验证的全程非消费式碰撞事件流 | 未覆盖 |

R5 的[专用收尾检查](reliability/r5_stop.py)只适用于已记录注入对象、归属和进程身份的双机故障测试。正常入口继续按原规则处理非零退出。持有原始 R5 运行目录时，可使用只读入口核对原始关键工件：

```powershell
$r5Run = Read-Host '已归档的 R5 运行目录'
python -B deployment/airsim/demos/racer_multi_flight/reliability/verify_r5.py --run-root "$r5Run"
```

精简索引保留每轮原始工件的 SHA256、用例判断和逐机结果。按 run 标识从测试归档取得 bag、ULog 和运行日志，路径由使用者提供。旧完整探索第二轮缺少的 plan 记录仍保留在[历史摘要](forest-complete-validation.json)。目前未量化整图覆盖率，历史 inputs 退出 -11 根因未确定；四机飞行、一机 RoboPi 的混合部署和跨版本回退飞行仍待验证。整体交付状态为 partial。
