# 双机诊断记录

用途：在既有 RACER 双机入口中保留原始深度、实际输入、地图载荷和原生查询值，供深度—位姿—地图—轨迹离线对照。默认关闭，不改变规划、碰撞或任务判据。PC 地面验证与飞行验证分别记录。

## 启用与运行

先使用固定镜像构建当前源码的独立 release；`plan_env` 类布局变更时，必须连同受影响消费者重编译。旧 release 的源码校验会拒绝混用。保留原候选及失败记录。

在不入库的本地配置中增加：

```json
"diagnostics": {
  "enabled": true,
  "queue_messages": 128,
  "bag_cache_bytes": 33554432,
  "probe_points": [0.0, 0.0, 0.75]
}
```

`probe_points` 是 `racer_world` 中的 XYZ 三元组列表，只用于本例的只读地面查询；根据待查位置配置，不是路线或避障参数。上面的坐标仅示范格式。飞行采集可省略/清空此项，真实轨迹查询记录仍启用。`queue_messages` 限制异步写盘队列；队列溢出或写盘失败明确记为记录不完整。订阅回调不等待队列腾空，记录器不发送运动命令；额外计算、传输和磁盘负载对执行链的影响仍需实测。

`bag_cache_bytes` 默认为 32 MiB，使用固定镜像中 rosbag2 的原生批量缓存，将 SQLite 写入合并。关闭 writer 时排空缓存并生成 metadata。缓存只改变记录方式，不改变发布频率、原始载荷或算法参数；最终必须以 bag 实际读回数量核对，不能用已提交到缓存的数量冒充已落盘数量。原无缓存写入在实际地面测试发生队列溢出，失败记录保留；离线对照和地面复验分别报告。

在 UE 已 Play 的干净会话中，由 Windows PowerShell 执行：

```powershell
Set-Location <AeroHub工作区>
python deployment/airsim/demos/racer_multi_ground/diagnostic_flow.py --local <本地配置.json>
```

使用新的 `run_root` 和项目名。入口依次执行正式分阶段启动、只读记录、原生地面 shadow、停止、地面确认、清理和离线读回。shadow 拒绝真实执行；入口无解锁、起飞或运动 API。任务失败与记录检查失败分别留证，任一失败不报告整轮通过。启动/地面状态不明时保留拥有的服务，先检查实际 PX4 状态再恢复或清理。

若保留多轮网络后 Docker 默认地址池耗尽，可在本地配置增加可选 `docker_subnet`（IPv4 CIDR）。先检查 Docker 网络及主机/VPN 路由，选择独立不冲突的私有子网；入口再次检查已有 Docker 子网重叠并保存 `network-before.json`。这不修改 Docker 全局地址池，也不删除旧网络；不要把某台机器选用的地址写成共享默认值。

双机飞行入口也识别同一可选配置，避免启动两个 observer。启用此选项不代表授权飞行，亦不证明新增记录在空中负载下通过。

## 输出与时间语义

每轮输出目录：

- `diagnostics/<namespace>/bag/`：标准 rosbag2 SQLite/CDR；包含原始 Image/CameraInfo、AS2 导航里程计、适配后深度/相机位姿/公共里程计、原始占据与局部膨胀点云、原生 plan/status、AS2 轨迹参考、clock/TF 和平台状态。无发布者的话题可能为空，按地面/飞行阶段分别判断。
- `receipts.jsonl`：topic、逐话题接收序号、接收墙钟、接收时 ROS 时间和原始 header stamp；不修改图像测量时间。固定镜像的 rclpy 不提供逐消息 MessageInfo，发布者身份来自 `publishers.jsonl` 中的周期性图快照，不能声称逐条 DDS 来源认证。
- `recorder-result.json`：接收/写入/队列丢弃数、错误、队列峰值、回调最大耗时、进程 CPU 与 RSS。BEST_EFFORT 订阅可能存在传输丢失；零队列丢弃不证明端到端零丢帧。通过匹配输入时间戳另行检查实际保留范围。
- `native/<namespace>/map-query.jsonl`：地图更新来源及进程内序号、最近本机输入 stamp、点云发布时间/点数、查询时刻与实际占据/膨胀/距离值。`ground_probe` 的 `trajectory_id` 为 null；`collision_check` 来自原有轨迹安全检查，记录轨迹 ID、实际查询点和原判定，不伪造 ACTIVE 来触发。
- `observer/observation.jsonl`：已有非消费式渲染/物理/GPS/气压观测；不轮询会清除标志的 `simGetCollisionInfo`，因此不提供 UE 碰撞事件完整统计。
- `diagnostic-verification/result.json`、`diagnostic-flow.json`：载荷读回、关联检查和整轮阶段结果；同时保留原生地面 `mission/result.json`。

地图点云的 stamp 是**发布时刻**，通过 `map_cloud` 记录关联地图序号，不冒充深度采样时刻。ESDF 更新、本机融合和 peer chunk 分开记录；一个序号不代表全部表示已经同步完成。点云是已有可视化范围内的占据快照，局部膨胀点云受局部 XY 范围、规划 Z 范围与可视化高度裁剪；它不是完整概率栅格或全量 ESDF 的序列化。查询日志保留实际访问位置的原始值，仍应结合算法源码判断语义。

## 离线使用与边界

`verify_diagnostics.py` 在机载镜像中加载同一 native overlay，用 rosbag2 顺序读取并反序列化全部消息，核对图像像素、配对 stamp、地图发布 stamp/点数与原生记录、各机发布者及地面状态。`diagnostic_flow.py` 在清理后自动执行该检查，命令保存在结果目录。

需要发布回放时，在独立 ROS domain、无控制/执行消费者的离线容器加载匹配消息 overlay，运行 `ros2 bag info <bag目录>` 或 `ros2 bag play <bag目录>`。bag 时间为接收墙钟，回放使用录制的 `/clock`，不要再用 `--clock` 生成墙钟仿真时间；需要 TF 静态耐久性时显式核对回放 QoS。读回成功不等于向真实控制链回放已验证。

地面通过只能证明本轮采集、实际地图查询与收尾；空中碰撞检查分支、动态状态精度、机体与地图几何差异、飞行负载、避障效果和 RoboPi 混合部署仍需各自验收。关闭 diagnostics 后复用正常入口；回退 native 候选时同时恢复匹配源码，不能跳过 manifest 检查。

## 2026-11-18 实测边界

独立候选 `20261118-diagnostics-01` 的 12 个包编译通过。首次真正进入地面采集的 `20261118-diagnostics-ground-03` 发生记录队列溢出；改用 rosbag2 批量缓存后，`20261118-diagnostics-ground-04` 两机各 16 项记录检查通过，实际读回 28074/28087 条消息，与各自接收数一致，队列无丢弃。每机保存 65 帧原始深度和 60 组完整匹配的适配后深度/位姿；地图 ground_probe 为 193/213 次。原生本机深度融合实际为 12/39 次，不把录制数冒充算法消费数。

原地面 shadow 的 `all_native_candidates` 仍失败：Drone0 无有效候选，日志显示地面起点在原规划高度范围外且邻居未被接纳；Drone1 有一个通过校验的候选。未改变规划范围或执行门限，整轮结果仍为失败。原生停止、landed/disarmed 与 15 个持续服务容器退出均通过，全程未解锁起飞。空中 collision_check 计数为零，相关分支和避障结果保持未验证。

原始网络启动失败、写盘失败、离线对照、最终记录结果及校验索引见本地证据包 `log/racer-ros2/20261118-diagnostics-ground/`；大体积 bag 按其中 `raw-artifact-index.json` 保留在外部运行目录。以上是该配置的地面记录验收，不是完整迁移或飞行验收通过。


## 2026-11-18 空中短程记录验证

双机飞行第二轮任务与记录读回均通过，详细范围及首轮失败见 [飞行验收](../racer_multi_flight/diagnostic-flight-validation.json)。原生轨迹查询已实际执行并可关联记录的计划；这不提供完整 UE 碰撞事件统计，也不证明历史窗墙接触已修复。探针点为空时，启动器省略该 ROS YAML 项，使用原生已声明的空 double 数组默认值；诊断文件与真实轨迹查询仍启用。首轮直接写入空 YAML 数组造成节点构造异常，失败与真实节点隔离对照均保留。飞行配置和算法门限未改变。

## 可选的休眠前沿决策记录

需要追查“没有可用前沿”时，在同一份本地 `diagnostics` 配置增加
`"frontier_decisions": true`，并保持 `"enabled": true`。
启动器将其转为原生参数 `frontier.decision_diagnostics`；缺省为 false。
直接启动原生节点时还须设置已有的 `RACER_DIAGNOSTIC_FILE`，仅开参数而无诊断文件不会记录。
此项只观察已有判断，不修改采样半径、可见点门限、规划范围或休眠簇重采样策略。

新增内容写入原有 `native/<namespace>/map-query.jsonl`：

- `frontier_candidates`：每个本轮新采样簇的一组候选。记录簇 UID、中心/包围盒、原始和过滤后点数、采样周期、规划盒、原可见点门限。每个候选保存位置及原判断顺序中的首个结果：`outside_box`、`inflated_occupancy`、`near_unknown`、`insufficient_visible` 或 `accepted`。
- `near_unknown` 保存首个触发的未知查询位置；未进入可见性检查时 `visible=-1`、`yaw=null`。进入后保存真实可见点数、视场外点数，以及射线首先遇到膨胀障碍/未知体素的点数。这些是每个过滤点的互斥结果，不是射线穿过的全部体素数；不额外查询地图，也不假称未执行分支已通过。
- `frontier_lifecycle`：`sampled_active/dormant`、`retained_active/dormant`、`removed_active/dormant`。保留旧簇不重新采样，保留原 UID 和 `sample_cycle`；不能将 retained 解读成“本轮视点筛选再次失败”。删除后重新发现的簇得到新 UID，未实现跨重建簇追踪。
- `frontier_snapshot`：分类完成后，以及 `stopExternalExecution` 请求停止时，保存 active/dormant 列表。停止原因结合对应原生停止日志读取；快照并不表示停止已确认。

UID 只在单个原生节点会话内有效，关联时必须同时使用运行目录和 namespace。
所有记录沿用原诊断文件的 `wall_ns`、`sim_ns`、`map_revision` 和 `last_input_stamp_ns`；
最近本机输入 stamp 不代表每次 peer 更新都来自该帧。`sample_cycle` 指最近实际采样周期，
当前快照的地图版本可能更新，不能把旧候选结论当作在最新地图上重新查询的结论。
记录用于解释筛选结果，不是完整地图转储，不能据此重放全部地图或恢复旧日志中未记录的历史决定。

`Frontier` 的诊断字段改变类布局，需连同 `active_perception` 的所有反向依赖重编译；
不得把候选库与旧消费者混用。继续使用固定共享镜像和独立 release，回退时选择匹配的旧源码与 release。
关闭开关只是停止新增记录，并不回退二进制。原短程候选仍独立保留。

离线测试入口见 [原生决策记录对照](../../../../src/tool/navigation/planner/racer/ros2_adapter/native/tests/frontier_diagnostics/test/README.md)。
它使用合成地图调用真实 C++ 库、对照基线/开关关闭/开关开启的视点输出，并检查实际 JSON 读回；
这不证明飞行负载、真实剩余休眠簇原因或长时间开销。实际地面和飞行记录另行验收。
