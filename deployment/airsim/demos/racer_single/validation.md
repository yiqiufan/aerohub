# 单机接入验收记录

状态：**partial**（RACER 项目整体）；**正式单机短程入口的本次完整流程通过**。最后一次新会话 `20261117-single-03` 的 `ground-01`、`flight-01` 及基础容器清理全部通过，`delivery-result.json` 的全部检查为 true。此前失败仍保留，不以此次通过改写历史。

## 范围与版本

运行前矩阵见 `acceptance.json`。本次没有改动既定飞行门限：20 秒探索窗口、至少两个实际激活版本和 0.5m 位移，原生起飞 0.75m/0.35m/s，Land 0.35m/s，停止 6 秒、Land 22 秒、Land 后联合确认 10 秒。原生规划速度尺度 0.65 是逐轴软代价，执行范数上界 1.5 是此场景独立检查。活动飞行范围和停止/接触阶段诊断分开记录。

RACER 上游为 `f8265e091b11cc856c1334f8260850b9006b1953`；完整基线、差异和文件校验分别见 `src/planner/racer/upstream.lock.json`、`integration-delta.json`。AeroHub 本地基线为 `4c06d210b08df47059dbe6e3073f0d5db35f3196`，本地新增文件与既有未提交修改均保留。未 commit、push 或上传。远端版本未成功核实，不能把缓存 origin/dev 当作当前远端。

四镜像 ID 见 `images.lock.json`，NLOpt 的四个 deb/版本/校验见 `dependencies.lock.json`。原生构建从正式源码重新编译 17 个包（20min17s）；工作站 as2_msgs、px4_msgs 两包独立构建。未重建或替换共享镜像。候选 `20261117-single-01` 的 `manifest.json`、两个构建日志、安装哈希和原生 executable.sha256 保留。

构建后的入口修正仅涉及运行/证据逻辑，包括实际导航二进制的 SIGINT 清理、地面联合确认和 logger 初始化；native/interface 源码未变化。每次 up 重新校验它们与构建清单及安装哈希，case 保存当前 runtime-source.json/source-snapshot，不能将早期构建脚本快照冒充这些后续修订也已重新构建。新增依赖锁检查在现有缓存上核对通过，完整更新后的 build.py 尚未再次执行全编译。

## 最后完整会话实测

| 项目 | 实测与结果 | 主要证据（相对运行目录） |
|---|---|---|
| 地面真实输入与原生 FSM | 真实深度/位姿/AS2 里程计、地图/frontier 就绪；生成曲线；停止 FINISH；两机 landed/disarmed；通过 | ground-01/ground-result.json、ground-events.jsonl |
| 原生探索与真实执行 | 10 个不同版本激活；10 次原生视点选择；任务结束水平位移 0.913m；通过预声明短程判据 | flight-01/flight-result.json、adapter-events.jsonl、exploration.log |
| 下游消费 | AS2 参考 372 条，PX4 参考 883 条；原生 Takeoff/FollowTimedPath/Land | flight-01/flight-events.jsonl、flight.ulg |
| 接受与拒绝 | 提交 10，接受/激活 10/10，拒绝 0；拒绝未算激活 | flight-01/analysis.json、adapter-events.jsonl |
| 停止与落地 | 停止协议和原生 FINISH 均确认；Land 请求后 Action 8.195s、landed 11.243s、disarmed 13.410s；均在原预算内 | flight-01/analysis.json、px4-ground.json |
| 飞控日志 | 当前飞行 ULog；0 次 dropout；固件 `61907962c9220437ec01a98fa26bfbf5cbb254b5` | flight-01/logger-start.log、logger-final.log、flight.ulg |
| 活动范围与停放机 | 活动飞行无声明范围违规；Drone1 最大观测位移 0.000m | flight-01/analysis.json |
| 动态俯视视图 | 889 条相机更新，实际相机位姿与世界系机群质心一致并随运动变化 | sensors/overhead.jsonl |
| 完整清理 | 算法/求解器子进程、Adapter、任务、四个传感器/观察进程及五个基础容器正常退出；无本轮运行残留 | flight-01/native-cleanup.json、各 cleanup-result.json、core-final.json、final-containers.json |

离线解析使用 numpy 2.5.5、pyulog 1.2.2；绘图使用锁定 onboard 镜像中 Matplotlib 3.5.1。`trajectory.png/pdf` 展示采样物理轨迹，已检查可读性；不是视频，也不证明整图覆盖或完整避障。

## 必须保留的失败与限制

- 会话 `20261117-single-01/ground-01`：PX4 启动子进程异常退出 -11，保存了 DDS/进程日志；地面恢复后可运行，根因未定，不能声称已解决启动可靠性问题。
- 同会话 flight-02 飞行通过，但 navigation 基础容器清理退出 143，因此该会话完整交付失败。直接运行实际节点并使用 SIGINT 的隔离启停测试退出 0；随后会话 02、03 的真实完整清理也均退出 0。此结论仅针对锁定镜像中的该节点。
- 会话 `20261117-single-02/ground-01`：原生规划和逻辑停止通过，最终 landed 判定失败。稍后独立 PX4 为 landed=true；缺少当时完整时序，无法断定是陈旧反馈还是状态变化。沿用 4 秒地面预算等待停止和新鲜 landed/disarmed 后，ground-02 及会话03 ground-01 通过。
- 会话02 flight-01 未起飞：PX4 已有 idle logger，重复 start 被拒；正式入口在两机地面确认后停旧 logger、启动文件 logger，并保存前后状态。随后会话02 flight-02 和会话03 flight-01 均得到当前 ULog。旧未飞行用例保留 not-flown.json。
- 会话02 flight-02 短程通过且所有清理正常，但含此前 ground-01 退出1，该会话聚合 `delivery-result.json` 仍为 false，未修改规则或删除旧容器来转成通过。会话03使用独立新目录完整复验。
- 会话02 flight-02 曾出现32次轨迹提交被拒、8次实际激活；拒绝记录中的 fault 为空，不能据此确定根因。该轮存在原生停止、恢复与重新提交，日志不足以证明全部拒绝由同一条件造成。最终会话的拒绝数量如上表；短程判据不等于持续无拒绝交接或长时间探索可靠性。
- 原生 no-frontier/FINISH 与有界窗口结束均不等于整图完成。场景碰撞保持开启，不轮询会清除状态的 simGetCollisionInfo；因此未覆盖飞行碰撞事件统计及完整避障验收。
- 四机 pillar 的 AirSim 接入、多机协同、RoboPi、故障注入、真机及跨两个已验收正式版本的回退未完成。本次不把它们标记通过；组件/能力卡片维持 partial。

## 证据定位与复跑

原始日志和大型产物保留于用户选定数据盘，未纳入 Git，也未上传。运行标识为 `20261117-single-01/02/03`，构建候选为 `20261117-single-01`。个人绝对路径、完整命令及 Skill 使用记录见本地 `skill/temp/racer-ros2/20261117-single-solidify.md`。每个运行目录的 evidence-index.json 给出文件 SHA256，最后会话 delivery-source.json 关联最终源码/构建/运行快照。

复跑顺序与本地配置格式见 README；依赖/镜像/场景须先满足前置条件。报告的通过限定于上表的正式单机短程完整流程，不是 RACER 全项目复现成功。
