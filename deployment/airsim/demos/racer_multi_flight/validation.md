# 双机飞行验证记录

当前 PC 交付见 [双机运行与验收](PC_DUAL.md)。下文保留此前版本的记录。

2026-11-17，PC 两机、现有固定 Docker 镜像、20261117-stop-01 外部构建。

| 用例 | 结果 | 边界 |
| --- | --- | --- |
| 隔离 Adapter/坐标与身份回归 | 两机各两项通过 | 网络隔离构造及转换，不是实际 Action 消费 |
| 工作站双节点构造 | 通过，名称与 PX4 身份独立 | 不发送运动命令 |
| 20261117-dual-takeoff-01 | 两机 Takeoff/Land、landed/disarmed、日志及清理通过 | 未触发 RACER 探索 |
| 20261117-dual-explore-01 | 未通过 | Drone1 在 context 确认阶段触发输入新鲜度失败，尚未发送探索 trigger；Drone0 因同伴失败收尾 |

探索失败轮两机原生 Land 成功、落地解除武装，无 PX4 应急降落；所有测试资源退出。存在匹配的 PairOpt 请求/接受响应及原生发起方消费，但共享轨迹 ID 仍为静止轨迹 0，不能算运动协同或 FollowTimedPath 执行成功。

失败附近深度配对持续成功：Drone1 新帧在任务进入 tracking 前约 70 ms 已由输入桥接受，但该状态经过 0.2 s 状态发布和 DDS 转发，任务可能仍持有上一帧；上一帧相对任务时钟约 1.407 s。旧记录没有失败瞬间任务持有的 input_state，因此不能确证是图像老化、里程计时间超前还是时钟回调顺序。保留原结果，后续补记每次 input_state 与失败时分项年龄，尚未改变速率、新鲜度门限或算法。

证据根目录由本地索引记录：每轮 source-snapshot、runtime-source.json、configuration、mission/*/flight-events.jsonl、adapter/*、inputs/*、native/*、ulog/*.ulg、cleanup-result.json 和基础容器清理记录。最短起降旧 JSON 留有 pass_:false 初始化字段，但各机最终 pass:true、根 pass_:true；原文件保留，探索入口已统一最终 pass 字段。

第一轮结束时未覆盖（第二轮补充结果见下）：双机实际原生曲线激活与运动、20秒指标、运动时轨迹协同、完整覆盖/避障、故障注入、四机飞行、RoboPi 混合部署。不得称 RACER 多机复现完成。

下一轮运行前调整：25秒双机地面探针发现持续正常更新的1 Hz深度经状态转发后年龄最高1.548 s，里程计最高0.240 s。将深度最大年龄独立设为2.0 s，里程计仍为1.25 s，不修改算法/轨迹门限；这是与采样周期匹配的案例工程预算，不是通用要求。原explore-01仍为失败。地面探针记录于explore-02/probe，随后飞行结果如下。

## 第二轮短程结果：20261117-dual-explore-02

在下一轮飞行前按地面测量分离深度/里程计预算后，20秒原生探索任务通过。Drone0实际激活6段、最终水平位移1.699 m；Drone1激活16段、位移1.357 m。两机均确认停止、原生Land成功、最终landed/disarmed，无应急降落。7个工作容器、7个基础容器退出码全部为0。探针和任务容器也正常退出。`acceptance.json`记录本轮机器可读结果及追溯哈希。

存在拒绝后重规划：Drone0两次请求被拒，Drone1七次更新被拒；后者AS2日志明确记录加速度不连续（约1.62–1.74 m/s²差值）。还保留控制器拒绝无效/重复/乱序/陈旧样本的合并告警，不能据该合并信息认定唯一原因。随后原生重规划继续激活，不修改算法或连续性限制，不声明所有曲线均被无缝接受。

本轮只证明双机短程实际执行与正常收尾，未证明完整地图覆盖、无碰撞、长期稳定或板端混合部署。原第一轮失败保持失败；两轮为不同输入检查配置，不能合并成同一配置重复成功。配对消费计数涵盖任务整体（包括地面），不等于所有协同优化发生在运动阶段；共享轨迹的tracking阶段消息另记于analysis。

阶段统计：
```json
{
  "drone0": {
    "active_count": 6,
    "rejected_count": 2,
    "tracking_wall_s": 20.003473017,
    "minimum_observed_tracking_separation_m": 1.7552702823767834,
    "tracking_as2_reference_samples": 187,
    "tracking_px4_reference_samples": 856,
    "tracking_shared_trajectory_messages": 298
  },
  "drone1": {
    "active_count": 16,
    "rejected_count": 7,
    "tracking_wall_s": 20.000029685,
    "minimum_observed_tracking_separation_m": 1.7552702823767834,
    "tracking_as2_reference_samples": 897,
    "tracking_px4_reference_samples": 879,
    "tracking_shared_trajectory_messages": 298
  }
}
```

## 固化复验（运行前声明）

保持第二轮flight.json、两机fleet.json、20秒及运动/停止判据。新增完整流程编排，不改规划器和执行连续性阈值。上一轮Drone0两次拒绝来自首点与实际位置不连续（AS2日志约1794916195.456/6195.527），Drone1七次来自替换加速度不连续；原生REJECTED回调清理待提交状态并继续规划，若覆盖不足则先停止再规划。当前已有后续激活和受控停止证据，不据此声称每次替换无缝。完整入口的正常、失败落地后清理、落地未确认保留服务三个模拟编排回归通过；实际复跑待执行。

## 固化首轮失败与下一轮范围修正

20261117-dual-solid-01完整入口实际执行完全部11步，探索阶段失败，后续地面确认、ULog导出、7工作及7基础容器清理全部正常。Drone0实际水平距离5.628 m触发继承自单机的5.5 m圆形界限，两机均停止确认、原生Land及landed/disarmed。原始失败不改写。末帧物理NED位置约[0.845,-5.564,-0.427]，对应公共ENU水平[-5.564,0.845]仍在既定规划矩形内，说明圆形测试范围与双机矩形配置不匹配；不能据此认为原生规划越过规划边界。

下一轮飞行前，将物理水平半径改为10.0 m：由已有执行矩形相对两机地面原点的最远角距离9.924717 m向上取整得到，不按本轮飞行轨迹拟合。规划/执行矩形、算法参数、20秒时长、停止与降落预算均不变。这是显式的测试范围修正，新旧配置分别保留，不宣称同配置重复通过；矩形内并非已证明无障碍，碰撞验收仍未覆盖。

## 固化第二轮：完整复跑仍未通过

20261117-dual-solid-02从新UE会话由`full_flow.py`完成11个阶段。两机20秒原生探索、协同消息消费、AS2/PX4真实参考与停止确认通过；Drone0激活12段、位移3.820 m，Drone1激活15段、位移1.810 m。主动tracking采样最小间距2.268 m，非连续时间最小值证明。Drone0原生Land成功；Drone1在22秒原生Land等待超时后转PX4 NAV_LAND，最终落地解除武装，因此任务和完整流程均保持失败。

Drone1时序（相对Land请求，墙钟）：约22.05秒收到PX4 NAV_LAND接受，约55.09秒首次ground_contact，55.46秒maybe_landed，55.84秒landed。接地附近内部物理速度仍变化，约54秒观察到静止；没有据此认定地图残留、碰撞插件或飞控检测的唯一根因，也没有延长门限把本轮改判通过。实际记录为`mission/drone1/flight-events.jsonl`、两机ULog和AS2/PX4容器日志。

任务失败后完整入口仍独立核验实际PX4落地解除武装、导出新ULog并清理7个工作容器和7个基础容器，均退出0；两机各3个原生进程退出0且无升级终止。`evaluate.py`对首轮和第二轮均返回失败，同时分别记录cleanup通过。关键工件SHA256、阶段、指标及失败原因在`solidification.json`；旧`acceptance.json`保留explore-02历史通过，不能代表新配置完整通过。

当前结论：代码/入口固化、清单/能力卡片、构建输入锁定及真实失败恢复收尾已交付；干净会话完整成功复验尚未达成。后续应定位Drone1落地判定延迟，再按同一入口复验。未覆盖的完整地图、碰撞统计、板端混合部署及热回退不变。

## 落地排查后复验

双机原参数recheck-01：两机20秒探索/停止通过，Drone1原生Land通过，Drone0等待22秒超时后恢复，完整任务失败且清理通过。该轮Drone0与旧Drone1不同：18.90秒ground_contact、19.287秒maybe_landed、19.654秒landed，22.034秒PX4恢复命令接受。相同日志低推力/接触有延迟，但未见旧轮持续至55秒的同等异常，不能归成已证实单一根因。

下一轮运行前仅将Land Action任务等待独立配置为30秒，保持PX4检测参数、Land速度及原有联合landed/disarmed等待不变。依据是已接地后的AS2返回预算不足，并遵循用户允许适度放宽门限；这是案例预算，不是所有算法默认值。原两轮失败不改判，30秒也不能证明旧55秒仿真接触异常已修复。下一轮独立输出recheck-02。

## Drone1排查与最终完整复验

Drone1单独最短起降通过（20261117-drone1-takeoff-01），原生Land约7.94秒成功。原参数双机recheck-01中Drone1成功、Drone0失败，说明并非固定车辆的必现接口问题。与旧solid-02的持续仿真下降不同，recheck-01已于19.654秒landed而22秒Action等待仍未结束，保留不同失败机制，不合并成唯一根因。

20261117-dual-landing-recheck-02在飞行前声明Land任务预算30秒，其余flight/fleet参数不变。完整11步通过，离线evaluate.py通过。Drone0实际激活6段、位移1.635 m；Drone1激活17段、位移3.761 m。两机停止确认、原生Land、landed/disarmed通过，无PX4恢复。Land Action墙钟耗时分别10.146秒、8.924秒。7工作+7基础容器及6个原生算法/求解器子进程正常退出，无升级终止。

该轮成功不能证明30秒预算是通过的唯一原因，亦不能证明旧55秒仿真接触异常根治；旧失败均保留。排查已定位到渲染/物理/传感器不一致与结果等待预算两个层次，场景/插件的唯一根因仍未证实。没有修改飞控检测参数、关闭碰撞、改物理引擎或优化RACER。完整地图、避障事件统计及板端部署仍未覆盖。

最新机器报告landing-recheck.json，关键证据在独立run目录；旧acceptance.json和solidification.json为历史结果。四个编排回归通过（含Drone1模式传递）；源码与镜像仍按manifest校验。本轮使用已有平台/验收Skill，新增正式Skill说明仅覆盖真实固件版本与消息汇总语义的核查，不包含项目固定等待数值。

## 同一配置双机重复验收（两轮已完成，未通过）

运行前锁定两轮，允许变化仅为运行目录、Compose project、DDS domain和重新启动的UE会话；运行过程中不改算法、门限或等待预算。两轮runtime-source、build-manifest、flight/fleet文件哈希完全相同，运行中AirSim settings内容相同，容器实际镜像ID与锁定镜像一致。报告写入之前再次检查全部111个冻结文件一致；本节及repeat-acceptance.json是运行结束后追加的结果，不属于两轮运行源码快照。

| 用例 | 20秒探索、真实执行、协同与停止 | 原生Land与最终状态 | 完整流程 |
| --- | --- | --- | --- |
| 20261117-dual-repeat-01 | 通过；Drone0激活11段/位移4.312m，Drone1激活8段/0.885m | Drone0通过；Drone1的30秒原生Land及75秒PX4恢复等待均超时，事后才确认landed/disarmed | 失败；入口保留飞控和时钟，后续实际落地确认后显式导出ULog并受控清理 |
| 20261117-dual-repeat-02 | 通过；Drone0激活12段/位移2.739m，Drone1激活11段/1.181m | 两机原生Land与联合landed/disarmed通过，无PX4恢复 | 全部11步及离线评估通过 |

两轮最终均完成两机ULog导出，7个工作容器及7个基础容器退出0。第1轮的事后恢复/清理通过不改变原任务和flow-result失败；第2轮也不替换第1轮。主动tracking的采样最小间距分别约1.308m、3.005m，不是全程连续时间的无碰撞证明。

第1轮Drone1在Land后约110.831仿真秒才首次landed；不得把这个值当成墙钟耗时。墙钟Land后12至105秒的多个观测中，渲染高度基本固定但内部物理位置/向下速度继续变化，GPS和PX4估计也有相应运动数据。该现象支持继续排查AirSim/UE接触链，但不足以确定唯一根因，也不能据此认定是RACER规划错误或仅为Action等待过短。未更改飞控检测参数、物理引擎或碰撞设置。

机器可读汇总见repeat-acceptance.json；原始ULog、观测、任务事件、源码快照和运行结果保留在各case目录，精简副本及镜像身份在log/racer-ros2/20261117-dual-repeat。此前landing-recheck.json仍为历史通过，当前重复验收为一败一通过，不能宣称稳定复现。完整地图、避障事件统计、板端混合部署等仍未覆盖。

本次实际复用正式uav-sim-e2e-validation的预声明重复次数、冻结配置、失败保留、恢复未确认保留服务、实际落地后清理及日志分层诊断要求；既有指引已覆盖本次处理，无新增可复用机制被验证，因此回顾后不强行修改Skill或新增脚本模板。

## 分阶段启动固化

`up`先启动clock及fleet内各机PX4/Agent，沿用60秒就绪等待预算：检查这些容器仍运行、每个实际PX4实例landed/disarmed且DDS connected，再启动各机AS2/navigation。按fleet生成服务列表，没有逐机重启兜底；`startup-result.json`记录阶段、时间和错误。该结果只表示依赖启动，不等同AS2任务就绪、飞行或完整RACER复现通过。

容器退出或就绪超时会阻止第二阶段；第二阶段启动命令失败也返回失败。自动保存`startup-failure-containers.json`及各存在容器的`*-startup.log`，保留服务供核对实际状态后受控清理，不自动重启或强行停飞控。已退出/不存在的core服务返回明确错误，不再执行空容器ID。Docker探测调用有超时，正常启动后仍由任务/地面验收核对实际消息和能力。

临时方案两轮地面冷启动通过后，20261117-formal-startup-01直接调用正式飞行部署CLI的`up`（继承同一实现），无启动拦截。两机实际PX4状态、DDS及AS2/导航跨容器探针通过，12秒clock605个唯一时间戳，里程计600/599；最终landed/disarmed、7个容器退出0。镜像、飞控参数和12个settings/build/configuration工件与临时复验一致。报告见`../racer_multi_flight/startup-validation.json`。

6项启动回归覆盖就绪顺序、容器崩溃、地面超时、DDS未连接、第二阶段失败、按fleet生成服务；另有4项完整流程回归。合成故障回归不代表真实崩溃根因已修复。实际新入口仅验证PC双机地面，四机和RoboPi仍待验证；旧正常启动失败与降落失败记录保留。


## Drone1 最短起降接触对照：20261117-drone1-contact-03

用户刷新 UE 后，沿用正式分阶段启动及原 flight/fleet/settings/地图/插件/镜像/构建候选，只执行 `takeoff-drone1`。起飞高度0.75 m、速度0.35 m/s、原生Land速度0.35 m/s与30秒等待预算不变。任务和完整流程通过；Drone0的本轮ULog仅出现disarmed状态。7个工作容器及7个基础容器均退出0，另核对任务、survey容器正常退出。

以Land请求为墙钟零点：Action约9.144秒成功，PX4 ground_contact反馈11.040秒、maybe_landed 11.419秒、landed 11.786秒；PX4 disarmed反馈13.926秒、AS2未解锁反馈14.015秒。首次disarm请求被PX4以not landed拒绝，随后自动解除武装，原始告警保留。ULog自身时钟中，降落ground_contact到maybe_landed为0.354秒，到landed为0.693秒，再到自动disarm约2.010秒；COM_DISARM_LAND=2。不得把ROS sim_ns直接当作ULog时间戳相减；上述墙钟反馈与ULog内部间隔分别报告。

非消费式观测最后一个物理运动采样在Land后5.472秒，5.672秒开始后续115个采样物理速度为零；约6秒起渲染与物理位置一致。ULog显示ground_contact之前推力仍逐步降低，速度条件已改善，之后进入固件的接地/可能落地/落地迟滞，再自动disarm。按实际固件61907962核对检测源码，has_low_throttle是发布周期AND汇总，不能把某条false解释为同一瞬间所有条件不成立。本轮没有重现历史几十秒渲染静止而内部物理继续运动的问题；也没有通过调整检测门限获得成功。

本轮不安装此前失败的UE OnActorHit回调，运动期间不调用清除型碰撞接口。两机实际landed/disarmed后单次读取：Drone1保留的接触对象为Rubble_76，Drone0为Floor_1204；这是最后保留的接触记录，不是首次接触时间或全程碰撞统计，不能据此认定Rubble_76导致历史故障。对应scene样本、ULog及脚本归档到log/racer-ros2/20261117-contact/contact-03，原始大文件按evidence-index.json保留在独立run目录。

机器摘要见[contact-validation.json](contact-validation.json)。本轮只建立新入口下的单机最短起降对照；此前双机重复验收失败仍有效，完整探索后的间歇接地异常、完整碰撞统计及板端未解决/未覆盖。后续先恢复同配置双机完整流程并保留相同证据，不因这一轮通过修改场景物理或PX4门限。


## 正式分阶段入口双机完整探索：20261117-staged-dual-explore-01

用户刷新UE后，直接运行正式full_flow.py --mode explore，全部11阶段及正式evaluate.py通过。两机约20秒原生探索、原生协同消费、AS2/PX4真实参考、停止确认、原生Land及最终landed/disarmed通过。7个工作容器、7个基础容器及mission/survey共16个容器退出0。构建、镜像、运行源码冻结校验通过；settings/build-manifest及生成configuration与dual-repeat-02逐文件一致，未改变参数或等待预算。

Drone0实际激活4段、水平位移0.601m，Drone1激活10段、位移1.220m；tracking的AS2参考分别238/385条，PX4参考861/865条。采样最小间距1.026m，不是连续时间无碰撞证明。Drone0另有34次FollowTimedPath请求拒绝，只有笼统的rejected the request信息，具体原因待查；控制器还记录3次invalid/duplicate/out-of-order/stale复合告警，不能将两类记录直接归为同一根因。既有验收未规定零拒绝，因此保留本轮既定判据通过和该独立诊断缺口，不事后新增门限或抹去异常。

相对各机Land请求的墙钟：Drone0 Action成功9.835秒、landed反馈10.420秒、AS2未解锁12.638秒；Drone1分别9.078、9.439、9.134秒。Drone0首次disarm被拒后自动解除，Drone1disarm被接受且未解锁反馈先于landed，联合等待仍正常完成。ULog内接地到landed分别0.696/0.702秒；两机约6秒之后的场景样本物理速度为零，未重现历史几十秒的渲染/物理分离。ULog与ROS/墙钟分别报告，不直接相减。

机器摘要staged-explore-validation.json，原始ULog、配置、源码快照及消息证据按log/racer-ros2/20261117-staged-explore/01/evidence-index.json保留。本轮仅一次PC双机短程通过；此前重复验收失败、间歇接地根因和本轮Drone0请求拒绝仍保留，不宣称完整RACER复现或稳定性验收通过，完整碰撞统计与板端未覆盖。


## Drone0 34次拒绝离线定位（补正前轮原因未知记录）

仅解析20261117-staged-dual-explore-01原始证据，没有启动Docker/飞行或修改算法、执行代码、参数和门限。34条Adapter rejected与34条FollowPathBehavior告警按顺序及墙钟逐一匹配，均为initial point is not continuous with vehicle position；前轮只检索reject类关键词，遗漏了不含该词的具体告警，原“只有笼统反馈”结论在此补正。

被拒的是revision 3–31、33–37：全部initial=true、base=0，即停止后重新发起Action，并非滚动modify。执行端使用本轮日志中的0.15m首点连续性门限；邻近±100ms内导航里程计最近样本对应偏差约0.1695–1.9406m。该里程计不是执行端回调内精确位姿，定性依据为执行端明确告警，数值用于独立佐证。拒绝回应时仍剩约0.669–0.738秒至valid_from，且源码已先通过时间/元数据校验才进入首点检查，故这34条的触发条件不是迟到或滚动PVA连续性。

全部38条原生均匀三次B样条按(p0+4p1+p2)/6取首点并扣除已记录offset，与Adapter输出首点最大差3.14e-16m，排除本轮首点采样/平移错误。原生优化源码calcStartCost将起始状态写为加权代价，不能当成严格等式；但日志未记录每次solver输入起点及优化后边界误差，尚不能唯一归因为优化权重、运动期间采样延迟或其他原生规划因素。

两组拒绝之前，native均因insufficient active coverage请求停止，Action返回CANCELED（revision2、32），随后STOPPED反馈和控制器模式2位置保持确认。旧轨迹已取消，不是在拒绝后继续执行；后续revision32和38被接受且激活。第三次主动停止原因是solver failed。已有停止证据证明取消/切换保持链路，不等于严格零位移悬停。控制器3次invalid/duplicate/out-of-order/stale sample告警与这34次Action首点拒绝分属不同检查，不能混为一因。

报告见[rejection-analysis.json](rejection-analysis.json)，可重放离线脚本及源码索引在log/racer-ros2/20261117-staged-explore/rejections。原始任务/flow通过及此前重复验收失败都保留。下一步候选是针对停止后重新规划补充solver输入起点、输出首点和同期状态的最小诊断，再决定适配方法；不直接放宽0.15m，也不直接修改原生优化器。当前证据只确定拒绝谓词和首点转换正确，未证明上游偏差的唯一成因或任何修复有效。


## 停止后首点最小诊断（2026-11-18完成）

case为20261117-start-point-diag-01。仅增加FSM求解前后日志和已接纳odom时间戳，不改规划逻辑、参数、优化权重或执行门限。独立候选原生编译JobEnded rc=0，最终executable.sha256核对通过；界面中断造成宿主编排未收尾，随后补做接口构建和manifest并记录恢复，旧候选未覆盖。

正式双机完整流程及evaluate通过，16个所属容器退出0。Drone0复现6次新Action首点拒绝（revision16–21），有完整solver配对及执行端对应告警。求解边界误差0.00095–0.00309m，原生输出到Adapter首点差小于1e-9m；规划输入里程计年龄0.141–0.315s、速度0.796–1.046m/s，接收时邻近位置相对规划输入已变化0.163–0.321m。此前revision15已经CANCELED并STOPPED确认模式2。新的输入stamp在停止之后，却仍描述带速度的滞后状态；本轮支持运动状态快照滞后为主要偏差来源，不支持优化器大幅推离首点。

邻近导航odom不是AS2回调内精确位姿。滚动modify的未来首点不可直接与当前位置比较，诊断区分initial与modify。这6次证据不能倒推旧34次同因，未实施或验证任何修复。下一步应针对停止后的状态取样与调度做最小适配验证，保持既有预算和门限。机器报告start-point-diagnosis.json；原始证据按log/racer-ros2/20261117-start-point/evidence-index.json保留。无提交上传。


## 状态取样与调度修正：20261118-state-sampling-01

external_execution里程计改为独立接收组，仅写线程安全最新消息槽；FSM串行应用，不并发改fd_或地图。沿用0.5s年龄检查，拒绝重复/倒序状态；按机体状态初始化的规划不对同一采样反复求解，滚动轨迹边界逻辑保持。没有新增静止速度门槛，没有调整权重、飞行参数或首点门限。

候选state-sampling-01准备时遇Windows旧日志软链接失败；02因固定colcon不支持allow-overriding失败；03复用已核验依赖，仅重新编译exploration_manager成功。并发快照测试验证消费者忙时10000次接收只取最新值、旧快照不变、取空与后续接收；源码见racer_multi_ground/latest_sample_test.cpp，命令及日志归档，不等同ROS图测试。实际飞行提供调度效果证据。

Drone0新Action输入年龄从诊断轮中位0.186s/最大0.420s降到本轮中位0/最大0.018s；Drone1本轮中位0.012s/最大0.015s。两机Action请求拒绝均0；静态来源求解各10次/3次，采样stamp均不重复。20秒探索、原生协同、真实AS2/PX4参考及停止确认通过。本轮只能证明修正取得单轮改善，不证明所有历史拒绝已解决或长期零拒绝。

完整验收失败：Drone1的Land Action在原30秒预算超时，任务进入PX4恢复分支。实际landed反馈在Land后16.102秒、AS2未解锁反馈18.324秒，均早于超时，但用户报告碰撞且未正常降落。补查观测显示Drone1降落前约10秒渲染位置冻结而物理仍运动，最终位置高于初始约0.9353m。不能把landed/disarmed等同安全落地，必须同时保留场景接触受阻和Action超时；具体碰撞对象未记录，用户已重置UE。原始mission/flow/evaluation失败均保留。最终两机landed/disarmed，ULog保存，16个容器全部停止，mission退出1，其余15个退出0。

机器摘要[state-sampling-validation.json](state-sampling-validation.json)，原始证据按log/racer-ros2/20261118-state-sampling/evidence-index.json保留。旧候选与源码备份保留；回退须恢复匹配源码再选择旧release，不能绕过manifest。没有提交上传。


## Drone1 场景碰撞体核查：20261118-scene-contact

只读静态查询在历史受阻/最终位姿下，用实际Drone1根组件命中Window_Tall_56与Wall_Window_Tall_51；起点无同类重叠。受阻位置保持12.600秒，最终高于初始0.9353米；中心正下方为Floor_1204，不能当作已正常落地。窗户存在简单射线命中、复杂射线未命中的差异；尚不能据此判定碰撞资产错误。已确认当前settings与失败轮一致，本次地图与配置哈希未变。

这是用户重置UE后的静态几何核查，缺少原碰撞事件，不能宣称首次碰撞对象或Land超时完整根因已确定。没有飞行和场景/算法修改，旧轮失败保持。精简摘要见[scene-contact-validation.json](scene-contact-validation.json)，完整方法、原始查询和复查脚本见log/racer-ros2/20261118-scene-contact/report.md与evidence-index.json。下一步先对照窗墙位置的深度、占据地图及机体/轨迹范围。


## Drone1 窗墙离线对照：20261118-obstacle-offline

第13条原生轨迹在生效期穿过当前窗户简单面的覆盖范围；832条可配对AS2位置参考与原生曲线最大差2.640e-7m，未发现位置转换偏移。窗墙重叠附近4帧相机估计位姿与渲染位姿代理差1.323–2.016m，配对时间间隔27–54ms。原生此前曾检查到未来轨迹占据并停止；不能称完全没有碰撞检查。

原生检查基于中心曲线和膨胀占据，不等价于UE实际机体；0.7m为软优化距离。缺少历史深度像素、占据/膨胀/ESDF快照和逐点查询，未确定地图何时为何允许该路径。ULog armed区间无位置/速度/航向reset计数变化，不能因此认定定位准确。只读离线分析，无飞行、无参数修改，原验收失败保持。

摘要见[obstacle-offline-validation.json](obstacle-offline-validation.json)，详细限定、对照图、源码哈希及复验入口见log/racer-ros2/20261118-obstacle-offline/report.md。下一步先准备按机隔离并有采样时间的深度/地图/查询记录，不能把多个发布者的公共诊断话题混作单机地图。


## 窗墙资产局部对照：20261118-window-visibility

离线导出的 Window_Tall_56 对应渲染网格存在窗格空洞，但简单碰撞体为覆盖整个窗面的单凸体；当前实际材质为 Opaque，无实例模式覆盖或组件材质替换。7条已保存UE射线命中与导出几何一致。真实飞行代表帧按rendered proxy投影，3662条仅碰撞体命中的射线全部对应超量程深度，1673条渲染网格命中均对应近处深度。它支持当前资产视觉/碰撞差异，不能直接称为玻璃透明造成漏检。

这是资源与已录深度的局部对照，不是新飞行或旧接触/落地异常的唯一原因证明；没有修改资产、算法、参数或门限。没有完整FREE/UNKNOWN地图快照，缺少占据点不直接判为自由。后续若保留不可穿窗语义，应先审查深度可见面与碰撞边界一致的场景候选，不直接关闭碰撞或放宽算法门限。摘要见[window-visibility-validation.json](window-visibility-validation.json)，完整证据见log/racer-ros2/20261118-window-visibility/report.md。


## ForestScene 调度修正后的同配置重复验收（2026-11-18）

当前基线为0.1.0-forest-short.20261118，候选20261118-handoff-scheduling-01。两轮事前声明的20秒双机探索、正式离线验收、诊断读回、停止/原生Land/landed-disarmed及全部容器清理均通过；运行参数及工程门限不变。第2轮Drone1 revision5首点连续性拒绝，后续恢复；仍有执行余量不足的受控停止。两轮均无确认迟到或求解错过提交窗口，不保证长期零异常。详见[固化说明](FOREST_SHORT.md)与[机器摘要](forest-short-validation.json)。历史Warehouse窗墙、间歇接地失败保留，不以新场景通过宣称旧问题已解决。正式Skill未修改。


## 2026-11-19 PC 双机完整探索版本固化

版本 `0.2.0-forest-complete.20261119`，见 [复现入口](FOREST_COMPLETE.md)、[锁定清单](forest-complete.lock.json) 和 [分项验收](forest-complete-validation.json)。两轮配置/运行源码/构建清单相同；均由两机原生 return finished 与 FINISH 结束，随后停止、原生 Land、landed/disarmed 及清理通过。首轮记录读回通过，第二轮 Drone1 记录数不一致并缺 plan 33/36，保留未通过。历史 inputs -11 根因未明，不宣称修复；整体 partial，不包含 RoboPi、四机或全场景覆盖。固化仅整理交付文件及只读校验，未新增飞行。
