---
name: as2-extension-integration
description: 按原生 Aerostack2 扩展约定实现 Planner、Behavior、State Estimator 和 Perception 接入，检查插件注册、宿主加载、执行与状态接口。
---

# AS2 原生扩展接入

## 输入与依据

读取项目 AGENTS.md、开发者贡献指导、能力映射及接口契约，确认 AS2 锁定提交。官方架构见 https://aerostack2.github.io/_01_aerostack2_concepts/architecture/index.html ，Behavior 教程见 https://aerostack2.github.io/_09_development/_tutorials/_tutorials/behavior.html 。文档用于理解范式，编译接口、注册方式和参数以目标源码为准。

## 四类实现规则

- Planner：当前通过扩展 FollowPath/FollowTimedPath 适配。算法保留在 `src/tool/navigation/planner/<算法>/`，Adapter 生成执行 Goal 并处理 Feedback、Result、取消和机群协调；不把算法整个搬进执行 Behavior，也不直接绕过它发布运动参考。
- Behavior：先检查现有能力。不能满足功能需求时，按原生 `as2_behavior::BehaviorServer<ActionT>` 等约定创建，放在 `src/behavior/<行为>/`。既有 FollowPath 扩展保留原入口和时间化入口的互斥；不能通过取终点或忽略时间来伪造完整轨迹兼容。
- State Estimator：按原生插件机制复用或新增。读取实际基类、原生示例、loader、plugins.xml、CMake 导出与 launch。外部算法可独立运行，由插件消费其结果并提供 pose/twist/TF；不要求重复 EKF。代码放在 `src/basic_function/state_estimator/<算法>/`。
- Perception：优先复用原生能力，不满足时扩展原生功能，仍不适合再独立实现。检查 `as2_map_server` 和 `as2_behaviors_object_perception` 实际能力；允许不同环境表示，不强制经过二维地图服务器。适配与接口归入 `src/basic_function/perception/<能力>/`。

## 加载与运行边界

1. 优先独立 ROS 包，确认插件导出、包发现、实际类名与参数规则。插件不是仅写一个 XML 就能加载；独立节点也不等于 pluginlib 插件。
2. 在真实宿主中验证加载和接口，检查时间、坐标、TF 发布权和输出新鲜度。基类头文件是否安装与 include 路径是否导出是两个问题，避免硬编码个人路径来修复。
3. 明确外部算法进程由谁启动、健康检查及退出如何收口。State Estimator 插件不自动具有 Behavior 生命周期，也不自动将结果送入 PX4 EKF。
4. 参数默认值不得伪装成实机标定。保留原有部署可用性；添加显式选择入口，不擅自切换飞行中的权威状态源。

滚动 Planner 需要重置执行状态时，停止请求与停止完成必须分开处理。先等待当前
Behavior 返回停止确认，再从新的实测状态建立规划初值并刷新地图/前沿等依赖状态；
不能在执行仍变化时用旧预测状态立即发布下一条轨迹。若新轨迹无法同时满足动力学
上限和 P/V/A/偏航连续性，应停止后从悬停重规划，而不是放宽 Adapter 连续性门槛。

外部 SLAM 的 Adapter 应先形成完整、可验证的 `nav_msgs/Odometry` 契约，插件再负责 AS2 坐标、杆臂、陈旧值和发布权。若上游没有速度，先用两个有效且单调的采样位姿计算 twist，首帧保持未就绪；不要把有限差分、GPU 算法和 AS2 宿主职责揉进同一个插件。跟踪丢失时停止提交新权威状态，恢复或重启产生新地图 epoch 时禁止在运动中热切换。带 PGO 的全局位姿可能跳变，未另行提供平滑局部 chart 时必须向消费者声明。

定位精度通过不代表输出速率适合直接承担控制状态。低频 SLAM 与高频平台里程计并存时，优先让平台里程计提供连续的 `odom -> base_link` 和 body twist，让 SLAM 按时间配对后提供 `map -> odom` 全局校正。动态全局校正应以零阶保持方式随每个新的本地采样时间提交，使完整 TF 链满足控制 freshness；这不同于把陈旧 SLAM pose 重发成新的 `odom -> base_link`。插件必须分别验证两路 frame、时间、频率、失效语义和唯一 canonical 发布权。

同仓库存在不同 AS2 构建版本时，每个目标都需实际编译并在对应宿主中加载插件。FAST-LIVO2 在原基类中使用 `on_setup` 与插件发布接口，而新基类使用 `onSetup`、`getTransformationTypesAvailable` 和宿主拥有的 `StateEstimatorInterface`；仅保持类名和 plugin XML 不够。新宿主在 setup 之后才注册变换权限，主变换应在注册完成后的有效输入回调中提交。宿主拥有 canonical `earth/map/odom/base` 时，插件必须把状态转换到平台控制所使用的同一 `odom` 语义；算法原生点云世界可由插件独立发布静态 TF，不能为方便而声明恒等别名。这些差异必须通过非恒等变换、时间戳、拒绝、静默、实际控制和恢复测试核对。

## 规划与执行限制的语义核对

当 Adapter 需要验证速度、加速度等数值限制时，先查上游参数的实际消费代码，注明是逐轴限制还是向量范数、软代价还是硬约束、作用于控制点还是连续曲线。再与执行接口的单位、坐标和校验方式逐项对应；同名参数不能直接视为同一上界。保留源码位置、原曲线及采样方法；若两侧契约不同，在下一轮运行前说明转换或独立执行范围的依据，并保留原失败结果。不要通过截断曲线或改算法来掩盖语义差异。离散采样不能证明连续时间上界；数值契约差异的实际案例见[接入案例](references/integration-cases.md)。

## 停止接口的模式语义

停止确认应核对目标版本的停止服务、该任务 Action 终态和请求后的控制器反馈。不要假定悬停一定公布为 `HOVER`；某些插件的 `hoverMode()` 将当前状态冻结为位置参考，公布 `POSITION`。只有核对实际实现且停止服务返回成功，才能按该版本契约接受这种模式，不能仅看 POSITION 就认定已停止。

## 繁忙规划器的停止反馈

当执行端已报告停止而 Planner 持续重试时，分别记录反馈发送、订阅回调接收、状态应用和原生状态转换；核查 timer、订阅、地图与同步服务等待所属的 callback group。常就绪的长计算 timer 可能使同一互斥组的反馈订阅持续得不到处理。用真实节点的忙时停止、忙时关闭及正常接受/激活/停止对照，区分传输、调度和协议语义，不先延长停止门限。

需要解耦时，可让独立回调仅接收并排队，由原串行状态机在安全点应用；避免直接把所有回调改为并发而引入地图/规划数据竞争。记录队列等待和状态转换，检查规划完成后是否仍会提交停止前的结果。ROS context 关闭后应退出服务等待，关闭竞争中的异常与正常运行错误分开处理；这不等于任意求解器都能被即时抢占。

对接受/激活/停止反馈与观测状态分别记录接收和应用时刻。等待执行确认与应用新鲜状态是不同职责；先定位哪一环节阻塞，再选择匹配的回调和状态存储方式。若采用最新状态槽，验证旧快照在求解期间不变、取样只应用最新有效状态、重复/倒序/陈旧输入处理和真实负载下的状态年龄；是否限制同一初态重试由实际调度问题决定。取消后重新规划的首点配对及验收方法统一见[执行诊断](../uav-sim-e2e-validation/references/execution-diagnostics.md)，不在本Skill重复维护门限或故障矩阵。

记录或修改生命周期分支时，检查算法主动停止、任务取消、执行失败和反馈确认等实际入口；测试应经过目标节点回调和状态转换，而不只直接调用辅助函数。只覆盖的分支可报告通过，未覆盖分支单列。

历史停止契约和调度验证的输入、证据等级见[接入案例](references/integration-cases.md)。

## 交付

交付包、注册/加载配置、构建入口、最小接口测试、运行说明和限制。说明复用了什么、扩展了什么、是否改动上游。兼容问题优先在本项目通过 Adapter、Bridge、插件、配置或启动层解决；只有确实必须修改上游源码时才使用 Fork，并保留版权、上游基线和差异说明。所有 Fork 必须作为 Git 子模块接入，由 `.gitmodules` 记录可访问 URL、由父仓库 gitlink 锁定修改提交，构建直接使用该子模块。构建补丁或复制进主仓库的 Fork 源码可以用于临时诊断或迁移核对，但不能作为完成复现后的最终源码来源。记录位于 `skill/temp/<任务>/`；实际验收证据位于 `log/`。插件加载成功只是接口阶段，仍需独立端到端验收；从补丁迁移到 Fork 后须重跑受影响验证。

## 外部低阶控制的模式扩展

角速度与姿态都是执行参考时，优先复用已有执行宿主的会话、时效、撤销和模式反馈逻辑。显式声明模式并只校验激活字段，未使用字段不参与有效性判断；未知模式拒绝，模式切换要求重新接管。区分固定世界系姿态四元数、机体角速度与推力的单位和frame。扩消息保持源码默认值不等于DDS二进制兼容，全部消费者须重生成接口及补编译。

姿态/推力以独立话题经过AS2时，在控制器和最终平台两级检验共同源stamp、坐标、新鲜度及四元数归一化，保留源时间；模式协商通过不能替代实际PX4姿态环启用的证据。姿态转换至本地odom后再进行平台ENU/FLU→NED/FRD转换，避免只改frame名称。现有FollowReference姿态入口的验证方法见`deployment/airsim/demos/attitude_control/`，不将通用控制源测试当作DiffPhys策略闭环。

## Sim(3) 对齐与降级

单目 SLAM 依赖 Sim(3) 对齐时，冻结条件不能只有样本数和拟合 RMSE；还必须检查总基线、水平基线及多方向激励，避免在纯升降或近单一直线运动中冻结弱可观测变换。全局校正输入中断或算法退出后，插件保持最后一个已接受的 `map -> odom`，拒绝陈旧新值，同时由新鲜局部里程计继续提供 `odom -> base_link` 和 twist；该降级语义必须通过运动中故障注入验证。

## 原生起飞高度与阻塞诊断

锁定版 `takeoff_plugin_position` 将请求 `takeoff_height` 加到激活时的 earth 高度，
成功后保留该位置目标；其反馈高度是 earth z。不能把请求值当成绝对高度，
也不能把 position 插件的规则推广到 speed/trajectory 插件。诊断先记录实际插件、
激活起点、最终目标、控制输入/输出模式和送入 PX4 的参考。
Mountains 实测在车底碰撞时仍持续收到约0.485m/s向上速度，实际只爬升约0.35m，
估计高度却漂移至约2.5m；不能因此调 PID、放宽成功阈值或延长起飞预算。
本次通过 Demo 出生点配置避开车辆，原生3m Takeoff/hover/Land恢复通过。

## 高频本地状态与低频协同校正

协同 SLAM 的 peer 优化与 landmark 输出频率不应决定控制状态频率。插件将每个全局 pose 与时间邻近的 local odometry 配对，计算并保持 `map -> odom`；每个新 local sample 都以原采样 stamp 提交保持的校正、`odom -> base_link` 和 body twist。独立 smoke 使用非恒等校正检查组合结果，端到端 authority 则测量飞行窗口内 canonical source stamp 与本地 arrival gap。

不要把初始化等待、验证器阻塞或仿真时钟跳步混入控制 freshness。发布权检查还应确认运行图中选中的 `plugin_name`、canonical topic 与 TF 只有一个宿主；shadow 通过后才切换 authority，并要求相同源码、镜像和标定。
