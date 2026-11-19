# RACER 双机真实执行

当前 PC 交付见 [双机运行与验收](PC_DUAL.md)。下文保留此前版本的记录。

原完整探索基线：**ForestScene PC 双机完整探索 `0.2.0-forest-complete.20261119`**。两轮原生自然结束、飞行及清理通过；第二轮辅助记录未完整读回，总体仍为 `partial`。配置、校验、复现、停止与回退见 [FOREST_COMPLETE.md](FOREST_COMPLETE.md)，分项证据见 [forest-complete-validation.json](forest-complete-validation.json)。

## 历史短程版本与阶段记录

此前已验证短程基线：**ForestScene 双机 20 秒探索，调度修正后的两轮同配置验收通过**。固定版本、复跑、离线读回、停止及回退见 [FOREST_SHORT.md](FOREST_SHORT.md)，机器清单见 [forest-short.lock.json](forest-short.lock.json)。两轮第2轮 Drone1 仍有一次首点拒绝并重试恢复；整体状态保持 partial，完整覆盖、四机、板端与故障矩阵未完成。

本页下文保留先前阶段的记录；其中“重复验收未通过”指当时的 Warehouse/旧候选，不覆盖上述 ForestScene 新版本结论，也不表示旧场景问题已经修复。当前操作请优先使用新基线说明。

## 历史入口说明与阶段结果


入口 `run.py --local <本地配置.json> <command>`；复用 `../racer_multi_ground/fleet.json` 的机群、共享坐标与构建候选。当前只支持两机 PC，尚不满足一机 RoboPi 的最终部署要求。

顺序：`up` → `sensors` → `survey` → `start` → `logger-start` → `takeoff` 或 `explore` → 确认两机 landed/disarmed → `logger-stop` → `stop` → `down`。每轮使用独立 run_root、project、DDS domain；容器受控退出后刷新 UE 会话。失败恢复未确认时保留 PX4、AS2 和时钟，禁止强制运行清理。镜像及外部构建要求继承单机和地面入口。

`start` 保存源码快照与哈希；任务拒绝运行中源码变化。算法/Adapter 在算力角色，工作站只调用真实 Takeoff、FollowTimedPath、Land。公共轨迹完整平移回每机本地 odom，保留 PVA、偏航及时间。各机分别处理停止和恢复，单机失败通知另一机收尾。

飞行清单为 `flight.json`，实际规划范围来自 fleet.json。先验证两机最短起降，再验证各机 20 秒原生探索、至少两个实际激活 revision 和 0.5 m 水平位移、实际 AS2/PX4 参考、原生协同消费与正常停止降落。数值沿用本案例单机短测，不是通用迁移门限。飞行物理范围只在主动阶段检查，恢复阶段另行记录；不采集具有清除副作用的碰撞 RPC，因此不能声明完整避障通过。

本地配置字段：run_root、release、project、domain、airsim_host、rpc_port、settings；个人路径与地址不放入共享默认值。运行前需要现有 17 包 native 和 2 包 interfaces 候选，以及 manifest 锁定的依赖；本入口不重建或改写共享镜像。

状态和证据见 [validation.md](validation.md)。历史短程记录为acceptance.json，固化失败记录为solidification.json；历史排查及完整复跑结果为landing-recheck.json。最新同配置两轮结果为repeat-acceptance.json：一轮因Drone1降落超时失败，一轮完整通过，重复验收未通过；不能宣称间歇接地异常已根治。

## 从干净会话完整复跑

先确认上一轮两机落地解除武装且本轮拥有的容器已停止，然后在 UE Stop/Play。新的本地 JSON 采用新的 run_root、project 和未占用的 domain，保留上一轮目录。共享 fleet.json 当前为两机；本飞行协调器明确拒绝其他机数，不改变地面四机入口。

在仓库根目录执行：

```powershell
python deployment/airsim/demos/racer_multi_flight/full_flow.py --local <本地配置.json>
```

该命令会实际起飞。它依次执行地面检查、传感器、起飞柱检查、真实算法和 Adapter、两机 ULog、20秒原生探索、停止/降落确认、日志导出、工作容器和基础容器清理。`flow-stages.json`保留各步结果，`flow-result.json`仅在任务及清理均通过时通过。启动失败或落地未确认时保留服务，按错误阶段使用分步命令检查恢复，不自动删除运行目录或强杀飞控。任务失败但已安全落地时执行清理并保持失败结论。

构建输入未变化时复用通过manifest逐文件校验的外部native/interfaces install；Python运行入口、配置及其哈希另存source-snapshot，不声称旧构建manifest包含新运行脚本。C++或接口改变必须重新构建候选。回退需要匹配的旧runtime-source与release、停止全部任务并刷新场景，丢弃运行中地图；未验证热回退。

组件清单为`component.yaml`，能力卡片为`cards/planner/racer-dual.yaml`。单机和双机入口共享算法、平台和执行资源，不可对同一车辆同时启动。

离线复核证据：`python deployment/airsim/demos/racer_multi_flight/evaluate.py --run-root <运行目录>`。它读取实际任务、轨迹事件、AS2/PX4参考与清理记录，写出evaluation.json及关键工件SHA256；失败返回非零。仅用于当前两机案例，不代替完整地图或碰撞评估。

Drone1最短起降诊断：同一full_flow.py命令追加`--mode takeoff-drone1`，使用新的本地运行配置。仅Drone1发送运动请求，Drone0留在地面；不触发原生探索，也不作为多机探索验收。原生Takeoff/Land参数不变，模式明确记录在flow-stages.json和mission/result.json。

`up`现已复用多机地面入口的分阶段启动：PX4/Agent/clock实际就绪后启动AS2/navigation；说明及失败处理见[多机启动](../racer_multi_ground/README.md#分阶段启动固化)。正式入口PC双机冷启动地面验证通过，见[startup-validation.json](startup-validation.json)；不改变本页双机飞行重复验收未通过的结论。


20261117 Drone1最短起降接触对照已通过，正式分阶段入口完成起降和清理；见[contact-validation.json](contact-validation.json)及validation对应节。该对照没有重现历史持续物理运动异常，不替代双机完整探索重复验收，也不改变本页未通过结论。


正式分阶段入口下的一轮双机完整探索已通过，见[staged-explore-validation.json](staged-explore-validation.json)。保留Drone0的34次轨迹请求拒绝待查；单轮通过不覆盖历史重复验收失败，不能宣称间歇故障根治。


上述34次拒绝现已离线定位为停止后新Action的首点位置不连续；此前原因未知记录已在validation末节补正。见[rejection-analysis.json](rejection-analysis.json)。首点偏差来源仍待最小诊断，不改变原验收结果。


停止后首点诊断复现6次拒绝，本轮以运动状态快照滞后为主，求解边界误差仅毫米级。见[start-point-diagnosis.json](start-point-diagnosis.json)。尚未验证修复，不覆盖历史重复验收失败。


状态取样修正本轮使新Action状态年龄降至毫秒级、请求拒绝为零，但Drone1 Land Action反馈超时导致整轮失败。飞控反馈早于超时，但现场碰撞及高处停住的证据仍待定位，见[state-sampling-validation.json](state-sampling-validation.json)。不得把取样改善表述为完整验收通过。


2026-11-18 带诊断记录的双机 20 秒探索第二轮通过：Drone0/Drone1 激活 8/10 个轨迹版本，执行层拒绝均为 0，停止、原生降落及全部容器清理通过；两机各 16 项记录读回检查通过，保存 74/188 次真实轨迹查询。见 [诊断飞行验收](diagnostic-flight-validation.json)。首轮因空探针数组的 ROS YAML 参数错误在起飞前失败，记录保留；最小修正及四项真实原生节点隔离验证随验收说明。原生仍有 8/9 条限频无路径警告，本轮不等于规划零失败、历史窗墙碰撞根因已解决、重复可靠性或完整迁移通过。
