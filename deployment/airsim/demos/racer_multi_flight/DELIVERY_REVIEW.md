# RACER 双机迁移交付核对（2026-11-19）

当前 PC 版本见 [运行与验收](PC_DUAL.md)。此前交付推进至 [PC 双机完整探索版本](FOREST_COMPLETE.md)：两轮飞行/自然结束/清理通过，第二轮辅助记录不完整。下文为此前短程阶段审查，相关版本与候选判断按当时范围保留，不代表当前状态。正式 Skill 后续范围恢复见 `skill/temp/racer-ros2/20261119-skill-scope-correction.md`；本次固化未改正式 Skill。

## 历史交付审查

本次交付是已有版本与证据的核对整理，没有新构建、飞行、参数修改或上传。整体保持 **partial／迁移未完成**。当前 PC 双机短程真实执行已经验证；不能据此声明四机原任务、RoboPi 混合部署或完整故障矩阵通过。正常休眠前沿不作为迁移阻塞项。

本说明与 [机器摘要](delivery-review.json) 配套。已有 [短程基线](FOREST_SHORT.md)、[组件](component.yaml) 和 [能力卡片](../../../../cards/planner/racer-dual.yaml) 的版本不自动升级为最新诊断候选；历史失败原样保留。

## 版本选择：不能混用源码、范围与产物

| 版本/配置 | 匹配构建候选 | 实际证据 | 交付结论 |
|---|---|---|---|
| `0.1.0-forest-short.20261118`，原范围 | `20261118-handoff-scheduling-01` | `20261118-handoff-repeat-01/02`，flight/读回/清理均通过 | 已命名短程基线，仍为partial |
| 扩大范围配置，原算法候选 | 同上 | `forest-expand-short-01/02`，flight/读回/清理均通过 | 独立配置验收，不等于全面探索新增区域 |
| 前沿诊断候选，扩大范围 | `20261119-frontier-diagnostics-01` | `forest-frontier-flight-01`，flight通过、读回未全通过 | 保留候选，不替换前两者的通过记录 |

原范围规划X[-6,4]，扩大范围X[-6,7]；Y[-6,4]、Z[0.25,1.5]保持。相应执行X上界4.5→7.5、物理观测半径10→11，20秒和原运动门限保持。这些都是本案例配置，不是公共迁移要求。

正式工作区当前保留原范围 fleet/flight 配置，但原生代码已加入诊断。与最新飞行冻结源相比，135项运行文件中只有这两份配置不同；与旧短程锁定的675项文件相比，有4个诊断相关源文件不同。**从当前工作区直接选择旧release不能视为旧基线复跑；选择当前release并使用当前默认配置，也不能冒充最近扩大范围飞行。** 应从对应已保存的完整源码快照运行。

旧基线只读校验入口 [check_baseline.py](check_baseline.py) 限定旧锁文件，不是任意候选通用校验器。不得修改锁文件以让不匹配的版本通过。本次对旧完整快照的675项源文件及完整身份检查通过；当前候选718项清单、2075项产物、135项运行文件及最近318项证据索引通过。

当前候选清单包含2项Python缓存文件；它们在冻结目录匹配，在正式工作区不同，不能误报为两个源码改动。后续构建清单应考虑排除生成缓存并明确schema，但本次不追改清单。当前manifest继承的构建历史、`validation.actual_ground_flight`文本属于构建时记录；后续飞行由独立运行证据关联，不能单看该字段，也不改写旧manifest。

## 已接通的职责与实际部署

依据 [AGENTS](../../../../AGENTS.md)、[四方职责](../../../../docs/项目简介.md#deployment-roles) 与 [Swarm基础设施](../../../swarm/README.md)：

- AirSim/UE在宿主机提供真实仿真深度与物理运动；工作站观察真值，不以AirSim运动API代替PX4执行。
- 两机各有独立PX4 SITL/Agent容器、AS2容器及导航输入容器。RACER native、Adapter等按角色分容器，各机在角色容器内由独立监管线程管理独立进程；不是每机一台物理计算板。
- `DepthPlanar/AS2 odom → RACER地图/前沿/协同分配/B-spline → Adapter → FollowTimedPath → AS2/PX4` 已有真实消费和飞行证据。[Adapter实现](../../../../src/tool/navigation/planner/racer/ros2_adapter/bridge/online_adapter.py)、[机群进程编排](../../../../src/tool/navigation/planner/racer/ros2_adapter/multi/group.py)、[任务](mission.py)。
- 任务在工作站ROS容器调用能力；高频运动参考不经工作站转发。原生Takeoff/Land与时间化FollowPath扩展分别承担起降和轨迹执行。
- 基础架构的一般Tool/Mission运行时、跨Behavior全局资源仲裁并未因本Demo而完成；卡片仍标为planner。

固定镜像ID见 [images.lock.json](../racer_single/images.lock.json)：flight `1eff3190ca78`、onboard执行 `ba618faf22ec`、algorithm `964048afd46a`、workstation `592f97da6b87`；本次实际inspect均为linux/amd64。algorithm使用机载工具环境v0.1.0，执行角色使用另一固定机载ID，不是两架无人机各选一个不同版本。实际程序通过外部overlay加载，共享镜像不重建、不commit。

仅复用Swarm的监管/观察部分，尚未完成通用配置生成与一机RoboPi的完整映射。当前 [飞行入口](run.py) 明确只允许两机；不能仅改机数便声明四机可飞。平移式共同ENU注册假设各轴对齐，已发现的磁参考航向差未修复；它不否定现有短程通过，但限制精确地图/障碍对齐的结论。

## 迁移验收矩阵

| 项目与依据 | 当前结果 | 尚缺什么 |
|---|---|---|
| 来源/许可证：[upstream.lock](../../../../src/tool/navigation/planner/racer/upstream.lock.json) | 锁定f8265e091b11cc856c1334f8260850b9006b1953，保留原生包及第三方声明 | 上游无根LICENSE，不把全部依赖统称一个许可 |
| 可复现构建：[build.py](../racer_single/build.py)、[依赖锁](../racer_single/dependencies.lock.json) | 固定镜像、NLOpt包哈希、独立overlay；当前ABI改动连同反向消费者重编译有证据 | 本轮只核对未重新构建；镜像、缓存、完整UE资产仍是外部前置条件 |
| 真实传感器/建图/协同 | 最新轮各73组深度/位姿配对，实际深度融合45/46次；协同消费通过 | 不代表地图完整、精确对齐或全部障碍可见 |
| 原生计划→AS2→PX4运动 | 最新轮激活11/17版本，拒绝0，水平位移3.20/4.78米 | 单轮零拒绝不证明长期无拒绝；中途交接停止不自动等于失败 |
| 正常停止/Land/清理 | 最新轮两机均确认；8个worker+7个基础容器退出0 | 人工中断、失流/退出等受控故障矩阵未完整验证 |
| 同配置重复性 | 旧短程两轮、扩大范围旧候选两轮通过 | 当前诊断候选只有一次飞行；不借旧轮声称新候选重复通过 |
| 诊断读回 | 当前Drone1通过；Drone0地图时间戳关联唯一性失败 | 同stamp的地图版本967/1046；最终外部停止确认快照仍缺失 |
| 场景障碍/探索效果 | 有原生占据检查、局部运动记录 | 未有完整飞行碰撞统计、覆盖率验收；Warehouse旧失败不被ForestScene通过消除 |
| 一机RoboPi+其余PC | 未执行 | 目标依赖、ABI、安装/进程映射、跨机DDS与时钟、地面和飞行验收 |
| 原四机pillar/AirSim四机 | 当前版本未完成对应接入验收 | 原内部仿真与当前AirSim双机不同；当前入口只允许两机 |
| 回退 | 旧源码/产物可核验；停机切换步骤存在 | 未执行跨版本飞行回退，不支持热回退 |
| 发布/团队获取 | 源码与证据在本地，尚未commit/push | 依用户要求暂不上传；本地索引不是远端可获取发布包 |

正式要求来自 [端到端验收Skill](../../../../skill/uav-sim-e2e-validation/SKILL.md)，具体20秒、机号、范围、位移和Land预算来自本Demo，不提升为AeroHub标准。完整任务指标应结合原项目目标和实际平台确定，不能以“消除休眠”替代探索效果指标。

最新飞行与诊断分层结果见 [本轮报告](../../../../log/racer-ros2/20261119-frontier-flight/report.md)。本地核对结果/所有运行哈希/源码差异见 [审计数据](../../../../log/racer-ros2/20261119-delivery-review/audit.json)。大体积原始记录仍在原数据盘，由本地索引定位，不复制UE工程或旧bag。

## 复现、停止、回退入口

1. 选择上述一行版本，绑定该行完整源码快照、匹配release、场景设置与范围；本机路径/地址保存在被忽略的交付目录，不能写进共享默认配置。版本间不整树覆盖。
2. 旧短程按 [FOREST_SHORT](FOREST_SHORT.md) 的check_baseline检查。当前候选按保存的manifest、runtime-source和对应两份配置逐项核对；[up](../racer_multi_ground/run.py) 校验native/interface、产物及依赖，不能代替场景与完整运行源身份检查。
3. 真实新会话、资源空闲和地面就绪检查通过后，才从所选源码根运行 `python deployment/airsim/demos/racer_multi_flight/full_flow.py --local <新local.json>`。这会起飞；本轮没有执行。每轮run_root必须新建，project/domain/subnet不能照抄旧轮。
4. 结束后分别运行 `evaluate.py --run-root <run>` 和 `readback.py --run-root <run> --release <matching-release> --output <new-output>`；保存全部结果，不以任务成功覆盖读回失败。
5. 正常流程自行停止、Land、等待两机新鲜landed/disarmed，再导出ULog和清理。`run.py stop/down`是落地后资源回收。人工SIGINT恢复路径见旧基线说明，未完成故障飞行验收；不能先停UE、时钟或PX4。
6. 回退先完成停止落地和所属资源清理，再在独立目录选旧源码与release、刷新场景/地图、重新检查就绪。不能只替换install，也不能把身份核对通过等同回退飞行通过。

## 参考已有项目整理方式与Skill回顾

参考 [SUPER](../super_forest/README.md) 的四方/历史模式分开、版本化JSON与Validation、成功与失败同时保留；参考 [CAT-ORA](../catora_open_field/README.md) 和 [混合部署](../catora_open_field/hybrid/README.md) 的进程归属、唯一运行目录、清理和负载口径。CAT-ORA旧单采样桥、集中式规划、固定队形参数及历史板端任务布置不作为RACER当前规范。

本轮实际使用 [构建Skill](../../../../skill/uav-reproducible-build/SKILL.md) 核对源码/产物、[组件Skill](../../../../skill/uav-component-deployment/SKILL.md) 核对运行依赖与回退、[验收Skill](../../../../skill/uav-sim-e2e-validation/SKILL.md) 分层判定，并用 [卡片Skill](../../../../skill/uav-capability-cards/SKILL.md) 核对planner分类与能力边界。

以下只是有依据的改进候选，本轮不改正式Skill，也不创建新Skill：

| 实际问题 | 现有覆盖与候选 | 适用边界/本次验证 |
|---|---|---|
| 多候选共用组件入口，旧基线与新源码混淆 | 构建/部署Skill已要求匹配源码产物；优先补项目版本目录和选择说明，必要时让已有清单校验区分构建源、运行源、配置、证据 | 适用于多候选overlay；本次真实发现4项诊断源差异及2份配置差异，未验证通用脚本/schema |
| 清单含缓存、继承历史构建与验证字段 | 现有Skill已要求实际来源；候选是明确派生候选的父版本与当前状态，排除生成缓存 | 适用于增量构建；本次只核对并记录，没有修订manifest或从空环境重建 |
| 停止记录方法合成测试通过但真实确认入口缺记录 | 验收Skill已有实际调用链要求；优先补项目事件路径用例，不新增普遍日志门限 | 适用于多个停止入口；本次已有真实缺口证据，修复尚未验证 |
| 正常休眠被延伸为持续排查 | 验收Skill已有任务范围/指标原则；按实际运动或任务阻断触发诊断，先分类算法原生行为 | 适用于自主算法；本轮能区分休眠、交接时间不足、20秒到时，不证明所有未来停顿同因 |
| 现有Skill尾部包含较多项目历史参数与诊断经过 | 后续可审查是否移到案例reference，主流程保留条件、输入、步骤、判断、输出和验证 | 仅整理候选；已有共享原要求仍有效，需逐条审查后再改，不把本机地址写成规则 |

无需新建一个“RACER迁移Skill”：这些问题均落在现有职责内。与算法无关的经验不能只凭本次一次飞行推广。按用户要求，当前只交付依据与候选，正式Skill待审查。

## 后续顺序

优先进入**双机混合部署的离线准备核对**：清点RACER目标架构依赖与现有RoboPi能力，确定一机上板的真实进程/接口边界，再安排设备地面验证。设备操作、登录或缺少材料时具体说明。故障恢复、障碍/任务效果与回退仍按原适用要求保留，不在本轮自行增加飞行次数或严苛阈值。诊断关联与最终快照单独处理，不因它们未完成否定已通过的运动层，也不把整体验收标为成功。
