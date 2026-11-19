# AirSim + PX4 SITL 核查要点

适用于当前项目的平台接入。以下是核查入口与已知边界，不是已经全部跑通的部署流程。

## 数据来源分开核实

| 通路 | 核查重点 |
| --- | --- |
| AirSim → PX4 SITL → AS2 平台适配 | 飞控收到的仿真测量与输出的估计状态要分开；PX4 内部有某 uORB 话题不代表已通过 DDS/MAVLink 暴露，须核对目标版本配置 |
| AirSim RPC → 传感器桥 → ROS | 图像/CameraInfo、激光点云等可独立提供给算法，不要求绕经 PX4；IMU 若直接使用 RPC，需明确其与 PX4 所用 IMU 的实例、噪声和时间关系 |
| 算法输出 → AS2 状态插件/感知接口 | 位姿与速度不等于地图；插件发布正常不代表输入正确或 PX4 已融合外部定位 |

AS2 的 IMU topic 名不能证明数据未经处理；检查角速度、加速度、orientation 的实际来源。算法需要 IMU 测量时，不以 PX4 已估计姿态替代。PX4 外部里程计回灌是另一个有方向的接口，需要单独配置和验证；它不是传感器桥自动附带的功能。共享 IMU 时还需考虑估计相关性，不能将两路结果视为独立测量随意融合。

当前入口：

- `simulator/airsim/config/`：场景、传感器与资源配置。
- `simulator/airsim/sensors/`：RPC 与 ROS 传感器桥。
- `deployment/flight_board/`：PX4 SITL、消息及传感器/时钟桥环境。
- `src/basic_function/perception/navigation_inputs/`：导航坐标注册；不是通用雷达逐点去畸变。
- `src/basic_function/state_estimator/fast_livo2/`：算法、原生插件和现有输出桥；检查是否另有输入桥，不推定输出桥能反向使用。
- `src/basic_function/state_estimator/pin_slam/`：复用现有标准 ROS 2 LiDAR 话题的算法 Adapter 与状态插件；不要再增加第二个 AirSim RPC 读取器。

先确认 scene 二进制、AirSim/PX4 源码版本与实际配置一致。源码中存在功能不证明下载的预编译场景包含它。检查仿真时间与 PX4 时间如何映射、暂停/reset 后是否清空旧数据，以及各车辆 TF/话题是否隔离。

## 观测数据与观测副作用

诊断渲染机体已接地而飞控仍报告运动时，分别采集渲染位姿、内部物理 kinematics、仿真传感器输出和飞控估计，不把其中一个默认当作其他数据的一致真值。按采样时钟对齐，并区分坐标原点和采样间隔。

RPC 名为 `get` 不保证无副作用。RACER 使用的锁定的 AirSim 插件中，`simGetCollisionInfo` 调用读取并清除碰撞标志的实现，同一标志也参与物理碰撞更新；两机静止时连续查询已验证该清除行为。它是否导致该案例的落地延迟尚未完成因果对照。其他版本先查实际源码与部署实现，在隔离地面条件下验证，不把这个案例当作所有 AirSim 版本的既定行为，也不通过关闭碰撞检测替代验证。

在后续RACER最短起降中，观测仅采集渲染位姿、内部物理状态、GPS和气压；运动期间不查询上述清除型碰撞接口，场景碰撞仍启用。AS2起飞、降落与PX4最终landed/disarmed通过，落地后才单次读取接地事件。该方式不提供飞行碰撞事件统计，不能替代完整避障验收；不同会话的一次通过也不能证明历史异常的唯一根因。需要飞行期碰撞事件时，应验证非消费式事件接口，而非把省略查询当作通用方案。

## FAST-LIVO2 案例的已验证边界

项目使用统一 AirSim RPC ROS 2 传感器桥提供 IMU、RGB 图像和 PointCloud2，再由 FAST-LIVO2 Adapter 转为锁定算法接口。AirSim 1.8.1 没有逐点采样时间，适配明确使用 instantaneous scan，不按扫描角速度伪造时间；仿真推导属性不能描述成真实硬件回传值。

坐标约定已用平台数据验证：AirSim IMU 和 LiDAR 的局部 FRD 转换为 FLU，相机保留光学坐标；LiDAR 到前视相机的矩阵为 `x_cam=-y_lidar, y_cam=-z_lidar, z_cam=x_lidar`。车辆沿 AirSim 世界 `+Y` 平移 1 m 的点云对照中，原始 FRD 补偿的中位配准误差为 `0.285 m`，错误横轴翻转为 `0.551 m`，因此不得为了修正算法轨迹而再次翻转横轴。

实际有屏 shadow 飞行证明三种测量进入真实算法，配准点云由 ROS 2 Perception 消费，状态误差通过预设 `0.5 m` 门限；随后 FAST-LIVO2 作为 AS2 状态/TF 权威完成闭环飞行，并在传感器中断和算法退出后受控降落。PX4 的航向 preflight 偶尔无法稳定，必须在启动上层链前用实际状态消息拒绝该轮运行，不能仅依赖启动日志或关闭检查。

AirSim 射线循环没有可供算法恢复的逐点时间，不能将其直接等同真实旋转雷达。后续诊断应优先保存可重放的原始三传感器记录，分开评估扫描模型、场景可观性、内外参与估计器参数；不要用合成字段、真值状态或放宽门限替代缺失语义。

PIN-SLAM 的当前 AirSim 接入直接消费标准 FLU XYZ PointCloud2。因为该帧是瞬时射线结果且没有逐点时间，配置必须保持 `deskew: false`，也不生成 ring、强度或时间字段。适配器在进入 GPU 前拒绝错误 frame、非单调时间、空帧和非单值 FLOAT32 XYZ；这些接口检查通过仍不代表神经地图或定位精度通过。

## 大图像跨容器传输与状态就绪

640×480 RGB8/32FC1 分别约0.92/1.23 MB。配对失败时，同时记录同一
`simGetImages` 批次的源stamp与消费者实际收到的stamp；源同步正常不能证明
DDS消费者收到完整配对。保持既定同步门限，先区分采样偏差与传输丢帧。

当前 Humble 固定镜像预置 UDP 配置；核查实际容器环境中的
`FASTRTPS_DEFAULT_PROFILES_FILE` 和 `FASTDDS_BUILTIN_TRANSPORTS`，显式使用
仓库 `deployment/airsim/config/dds/fastdds_udp.xml`。与 XRCE Agent 通信时，
现有 host 网络 Demo 使用 `ROS_LOCALHOST_ONLY=0`；逐项检查 PX4里程计发布端、
AS2里程计消息和历史TF，服务发现或周期发布的PlatformInfo不足以证明状态已就绪。

YOLO地面接入在 best-effort 下出现RGB/depth分别丢失；仅尝试共享IPC和builtin
传输没有解决。共享UDP配置、相机桥 `camera_reliable:=true` 与检测端同名参数
同时启用后，实际640×480、5 Hz输入通过10帧配对检查（最大偏差9 ms）。
这是当前版本的地面短时证据；其他负载与远端网络需重新测量。Reliable发布端
配best-effort订阅端不能当作已经验证的可靠链路。

新建消费者先等待实际深度stamp处的历史TF，再请求检测；保存每次返回的
reason及逐目标position_error。晚到TF可能超过算法现有等待预算，即使返回后
能够查询，也不能抹去该次三维测量失败。RGB与2D检测接通、三维测量有效、
检测精度和飞行闭环分别验收。可复验入口与当前边界见
`deployment/object_detection/airsim/README.md`。


## 原始颜色与静态RGB-D评分

AirSim Scene非压缩字节顺序须以实际二进制核对。可在传感器桥启动前同一次RPC
读取raw Scene与压缩PNG，按PNG解码后的BGR逐像素比较，再比较通道交换结果。
2026-11-15当前场景raw BGR与PNG解码MAE为0，通道交换约18.7，YOLO专属入口
因此显式配置 `source_color_order:=bgr`，共享桥仍输出rgb8。
源码描述、消息encoding与能检出目标三者均不能替代原始颜色核对；其他场景版本重测。

静态近距目标评分可用独立场景碰撞射线定位可见表面，保留原RGB、实例掩码、
深度、内参、实际相机pose及坐标换算。Actor原点不等于表面；碰撞几何也可能与
渲染表面不同，须报告该限制。采样时刻TF往返残差只能检查代数一致性，不能
充当独立的绝对定位精度。近目标有效且同帧远目标无有效深度时，应逐对象验收，
按公共契约保留PARTIAL，而非要求每个检出对象都有三维坐标。

输入就绪同时核查匹配stamp的深度、两路CameraInfo和历史TF。诊断间歇TF晚到时，
分别记录状态源/TF接收wall时刻、ROS clock、请求stamp与投影起止时本地最新TF；
在原等待预算内检验恢复和持续缺失拒绝，后续连续通过不抵消已有失败。


若独立best-effort观察者的TF/里程计新stamp持续推进，而算法Buffer停在旧stamp，
优先核对该消费者的动态TF QoS与调度。Humble TransformListener默认动态TF
reliable/depth100；YOLO地面对照中其Buffer落后537–735ms，独立观察者仅0–21ms。
该Tool改用sensor-data best-effort/depth5后，相同100次调用TF超时由4降为0，
仍有2次深度配对失败。仅调整动态TF，静态TF保持可靠transient-local，Buffer
继续按采样stamp查询和原预算等待；此对照不是长期网络可靠性或所有TF故障保证。

## ProjectAirSim 原生静态相机标定

ProjectAirSim/UE5.2 的原生 CameraInfo 在传感器初始化时由 capture-settings 生成，
原生 C++ ROS 桥使用 SensorDataQoS。归一化桥不得向它请求 reliable/transient-local，
否则图像正常而标定永远缺失。先核对实际 publisher QoS 与消息生命周期。

本项目 `simulator/projectairsim/sensor_node.py` 对已核实的零畸变 pinhole 模型，
从 resolved capture-settings 构造 timeless CameraInfo（fx=fy=width/(2 tan(FOV/2))），
持续提供给晚加入的消费者；同时校验收到的原生标定及实际图像尺寸。
该标定是仿真配置派生量，非额外传感器测量，不重写 RGB/depth 的采样 stamp。
仅适用于已核对源码的固定原生模型；动态标定、其他畸变或非方形像素须另行适配。
WestStreet 实际 RGB-D 投影验证通过了静态标定可用性，但跑步阶段的配对失败仍保留。

## ProjectAirSim导航LiDAR的初始化核查

场景模板的LiDAR不能仅凭16通道/100m等参数判为适用导航。WestStreet已观测到
模板vertical-fov为0..-90deg，FRD origin z+.2意味着位于机体下方；近地面实测
注册点云全在局部z[-.970,-.223]，ROGMap发布空地图，启动门阻止起飞。
先核对resolved_scene的原生FOV/安装位置及实际点云xyz范围。±15deg环扫、FRD
z-.2上置是此轮专属修正配置；真实复验已形成14826个膨胀占据点并通过地图
就绪门。这只证明该场景地面建图链，不是完整飞行、其他场景或真机通用推荐。

地图非空门失败时分别区分：虚拟地面膨胀导致丢弃、真实输入全在地面以下、
与大PointCloud2传输丢失。通过独立轻量订阅记录width/height/data长度，空width0
是实际空地图证据，不能继续以带宽猜测替代诊断。

## 原生双轴云台的实际几何

当前ProjectAirSim UnrealSensorFactory以零长度SpringArm实现lock-roll/pitch；
相机origin杆臂也挂在该稳定坐标系下，不能只抵消旋转而保持机体刚性杆臂TF。
支持该模式时，对原生actual_pose姿态历史按图像stamp插值，计算
q_body^-1*q_yaw对光学旋转和安装位移的共同作用；原生姿态仅充当模拟云台编码器，
不成为导航状态权威。缺姿态/过大插值间隔时拒绝该图像，不用最新姿态补齐。
稳定camera frame只发动态TF，保留刚性传感器原静态TF；其他云台模式另行验证。

采集裁减要核对resolved_scene内每个camera.enabled和capture-enabled，使用原生
capture-interval而非ROS节流。单路RGB/15Hz配置在WestStreet实测唯一采集13.77Hz，
其余capture关闭；不将窗口观察相机混入传感器数。验收分别检查配置、真实采样间隔、
动态TF、实际图像及算法消费；配置或TF水平不单独证明渲染姿态精度和持续追踪成功。

视觉指向动态云台时，目标来自Tracking有效状态（采样时间、valid_until、frame齐全），
使用自身状态TF计算指向，API留在平台桥，不能用Actor真值/LookAtObject替代感知。
SetPose确认不提供原子渲染采样姿态时，用原生仿真时间夹取切换区间，丢弃该区间
图像，区间后使用确认的新安装外参；未知结果与时钟回退要求重启，不能继续旧TF。
有意指向人体中心的俯角与机体roll/pitch稳定分开表达。记录实际相机确认、画幅居中、
观测连续性及飞行指标；框居中不证明高度先验测距或速度约束通过。

## 场景切换后的感知与碰撞语义

出现规划允许进入但机体受阻的现象时，先核对实际加载的地图、传感器设置及资产版本，再在相同坐标中对照原始深度/点云、算法占据或查询结果、轨迹与机体几何。渲染网格、传感器可见表面和物理碰撞体分别取证；缺少占据点不等于该处已知自由，传感器看不到也不等于可穿越。

局部静态射线或资产核对只能证明该处几何关系，不补造过去飞行的碰撞时序。环境无围墙时，分别核对传感器无返回值、算法未知空间处理及任务范围，不默认必须添加实体边界。范围选择仍由任务需求决定；不要为让地面规划通过而改变原探索目标。

RACER的窗墙案例发现可见网格空洞与整面简单碰撞体不同，真实深度局部对照支持这种差异，但未证明历史接触/降落失败的唯一原因。场景候选修正未完成，不能写成已验证修复；更换场景后的飞行通过也不修复旧场景。证据见[窗墙摘要](../../../deployment/airsim/demos/racer_multi_flight/window-visibility-validation.json)。
