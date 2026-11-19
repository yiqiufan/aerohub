# RACER 多机地面验证记录（2026-11-17）

当前：**后续双机地面整轮通过；完整多机复现仍未完成**。最新结果见本文“后续：双机停止修正与规划排查”及 stop-diagnosis.json。以下表格与首轮说明保留旧双机未通过的原始结论；四机未通过状态同样保留。全部测试未发送运动 Action、未解锁、未起飞；本轮最终两机实际 PX4 landed/disarmed，基础容器均退出 0，所有相关测试容器已停止。

| 证据层 | 双机 20261117-dual-ground-01 | 四机历史 |
| --- | --- | --- |
| 场景真实车辆及相机核对 | 两机实际列出，system ID/出生点核对通过 | 四机 probe 通过 |
| 真深度、AS2 里程计、frontier、各机执行端就绪 | 通过地面就绪 | 通过地面就绪 |
| 原生状态与 swarm trajectory 广播 | 两机均观测到；不是实际执行证据 | 四机均观测到 |
| PairOpt/Response 与原生发起方消费 | 1 组匹配成功，发起方 get response 1 | 未观测到，仍失败 |
| 外来地图块消费 | 两机互相广告对方已接收 chunk | 四机均广告其他机 chunk |
| 每机原生轨迹与 FollowTimedPath 请求构造 | Drone1 revision 1 通过；Drone0 无候选 | 多轮只有三机有候选 |
| 停止确认 | Drone1 FINISH；Drone0 PLAN_TRAJ，8 秒预算超时 | 未全部确认 |
| 清理 | Drone0 exploration 需升级终止、退出 -6；其余正常。整轮失败 | 部分轮 native 被强杀或 shadow 异常，均保留 |
| 飞行/多机执行/完整覆盖/RoboPi | 未执行 | 未执行 |

双机 Drone0 连续报 `No path to next viewpoint`，最终读到的 shadow 状态已 stopped，但 native 仍 PLAN_TRAJ；不能说停止修复已完整通过。清理日志明确显示 SIGINT/SIGTERM 后在 `ClientBase::wait_for_service_nanoseconds` 路径遇到无效 rcl context 并 abort。停止状态消费延迟、地面规划可达性与退出异常需要分别定位；目前没有证明单一根因，不能直接归因于 CPU、场景或时钟。地面当前位置不等同起飞后的规划初态，不能为凑齐候选伪造空中里程计。

四机曾观测 native 约 2.34 GiB、330–416% CPU；双机一次运行抽样 native 1.093 GiB、54.83% CPU。采样时刻/规划阶段不同，不能作为控制变量性能比较。双机实际发生配对优化是确定事实，四机无配对与高负载间因果仍未验证。

双机首次 Drone1 仍出现 `create entities failed: participant: 255` 后 PX4 子进程 -11（容器 245，非 OOM）。保存失败 inspect/log 后单独重启成功；这只是恢复，不是启动可靠性修复。双机重现该问题说明不能将全部失败归因于四机负载。

历史证据保持独立：

- `20261117-multi-ground-01`：初版 inputs 未启用 use_sim_time，原失败移至 inputs-before-simtime；修正后实际输入通过。Shadow 回调与父类 bool 属性同名，停止时 TypeError。其后修复回调名，真实 ROS 回归通过。
- `20261117-multi-ground-02/attempt-01`：回调已修复，仍存在普通 STOPPED 覆盖 OPERATOR_STOPPED 的顺序问题，已在影子适配器保持终止状态并回归验证。
- `20261117-multi-ground-02`：保留上一 attempt 后重新运行，影子停止修正、相机 3→1 Hz，仍缺全机候选/PairOpt，停止超时，native Drone0 退出 -9。不能将上述局部修复提升为四机通过。
- `20261117-dual-ground-01`：本次双机完整失败证据，任务结果、源码快照与清理报告的 hash 见 [acceptance.json](acceptance.json)。

原始产物存于本机 E 盘各独立运行目录以节省 D 盘；仓库忽略目录 `log/racer-ros2/20261117-multi-ground-index.json` 保存本机路径与双机 delivery-result 校验和。正式摘要记录相对证据文件/hash，不含本机地址。未上传或提交这些数据。

下一步仍是双机地面诊断：验证 native 对停止反馈的实际收取/调度，分离未找到可达视点与消息处理问题，补齐退出中求解器调用的收口。保留原算法参数与真实输入，先形成可复验的修正，再考虑飞行授权与四机恢复。当前没有调整原生 peer freshness、候选接受规则或验收预算。

## 2026-11-17 后续：双机停止修正与规划排查

`20261117-dual-ground-02` 使用新候选 `20261117-stop-01`，在原 fleet、相机频率、规划区域、35 秒观察与 8 秒停止预算下完成：**本轮双机 PC 地面影子检查全部通过，尚非多机飞行或完整复现通过**。详见 [stop-diagnosis.json](stop-diagnosis.json)。前文四机与双机旧轮失败保持原判定。

停止修正有旧版/新版隔离对照：使用真实 RACER 二进制与真实求解器、明确合成深度/里程计，在连续无路径重试中发送同样 OPERATOR_STOPPED。旧版 8 秒超时、清理需 -9；新版约 0.100 秒进入 FINISH，队列等待 9.414 ms，全部子进程退出 0。独立的忙时 SIGINT 回归从旧版 -9 改为正常退出；正常合成接受→原生激活→停止在旧新版都通过。合成确认不代表真实 AS2 接受或飞行。

原因及修改：停止反馈订阅原本与持续就绪的规划 timer 同属互斥回调组。新版独立接收反馈，只将消息入队；原串行状态机在周期开始和规划返回后应用，避免停止反馈持续等不到处理，也不并发访问地图/规划状态。退出时服务调用检查 context，并在原等待预算内响应 shutdown；正常 context 中的 RCLError 仍抛出。此修正不保证任意长时间阻塞求解都能被即时抢占。

真实双机复验中，两机均产生 revision 1 的原生曲线并通过 FollowTimedPath 请求构造；原生配对优化响应消费、双方地图块整合、停止 FINISH 与始终 landed/disarmed 均通过。6 个原生子进程、6 个算法/传感器角色容器、7 个基础容器与独立 ROS 记录器全部正常退出。没有发送运动 Goal、modify、解锁或起飞。两台 PX4 本轮直接正常启动，但一次成功不代表历史初始化故障已修复。

**规划失败的结论边界**：本轮 Drone0 没有出现 No path，不能把停止修正说成寻路问题的修复。旧轮起点约 `(0,0,-0.0066)`，目标 `(-3.0026,2.2503,0.5886)`；本轮起点约 `(0,-0.0015,-0.0338)`，目标 `(-0.0929,0.0287,0.8740)`。两轮目标与实际地图不是同一输入。虽然起点都低于配置 box_min_z=0.25，A* 对邻居而非起点本身检查盒边界，0.3 m 步长可能进入盒内；所以“地面高度低”不能单独证明旧轮无路径根因。本轮从更低起点生成曲线也否定了这种简单归因。

新增 A* 只读诊断记录 open-set 耗尽/时间/节点池等原因、首步边界/占据/射线拒绝计数、起终点与膨胀占据；没有改变搜索或膨胀。合成盒外回归已验证 `expanded=1, first_outside=26, first_admitted=0` 的可解释输出，但不能用这个人为条件解释旧轮真实地图。旧轮没有完整深度/map replay，唯一根因仍未确认。本轮已保存 26,091 条 ROS 消息（含两机真实深度、相机位姿、里程计与停止协议），便于以后故障重放。

构建：固定原四镜像，外部新 release 完整重建 17 个 native 包和 2 个工作站接口包，native/interface 源码 hash 与产物 manifest 对齐。旧单机 release 未覆盖；当前 C++ 修正尚未重跑单机飞行。原始数据索引在忽略目录 `log/racer-ros2/20261117-stop-diagnosis-index.json`，正式摘要保留关键文件 hash。

隔离回归可从仓库根执行（PowerShell，替换三个本地变量；三个 mode 各用新的 output 目录）：

```powershell
python deployment/airsim/demos/racer_multi_ground/diagnostics/run_regression.py --release $release --configuration $recordedConfiguration --output $newOutput --mode busy-stop
```

另外两个 mode 为 `busy-shutdown` 与 `accept-stop`。入口校验已构建产物、使用 `--network none`、不连接 PX4/AS2/UE，保存配置/源码和实际子进程退出记录。真实地面复跑仍用原 `run.py`；记录器命令保存在本轮 `record/command.json`，停止/退出证据为 `record/final.json`。
