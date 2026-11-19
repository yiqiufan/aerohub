# ForestScene 双机短程基线

版本 `0.1.0-forest-short.20261118`，状态仍为 `partial`。固定候选为 `20261118-handoff-scheduling-01`，源码、镜像、候选清单及 settings 的身份见 [forest-short.lock.json](forest-short.lock.json)。这是现有候选的命名与证据固化，没有重编译、改镜像或修改飞行逻辑。共享文件不保存个人磁盘路径、主机地址和 UE 工程安装位置；本地位置见交付给操作者的 local JSON。

## 已验证范围

PC Docker 中的两个独立 PX4 SITL、每机 RACER/Adapter/AS2 和工作站任务，宿主 AirSim 的 `/Game/RacerDemo/Maps/ForestScene`。复用四方职责和 `deployment/swarm`，尚未完成“一机 RoboPi，其余 PC”的最终混合部署。

正式链路为真实 DepthPlanar/AS2 里程计 → RACER 原生地图、frontier、协同分配和 B-spline → Adapter → FollowTimedPath → AS2/PX4。执行角色和工作站角色按现有组件清单区分。保持原规划范围、速度、20 秒时长、交接余量及 30 秒 Land 等待预算。每机至少两个激活版本、终点相对地面起点水平位移至少 0.5 m 是本 Demo 已有工程判据，不是 AeroHub 或 RACER 的通用要求；最大离起点距离只作补充观测。

| 同配置重复轮 | flow / evaluate / 诊断读回 / 清理 | Drone0 / Drone1 激活版本数 | 执行拒绝 | 确认迟到 / 求解错过提交窗口 |
|---|---|---|---|---|
| 20261118-handoff-repeat-01 | 全部通过 | 7 / 16 | 0 / 0 | 两机均 0 / 0 |
| 20261118-handoff-repeat-02 | 全部通过 | 4 / 18 | 0 / 1 | 两机均 0 / 0 |

两轮都确认停止、原生 Land、landed/disarmed 和所有所属容器正常退出。第 2 轮 Drone1 revision 5 的首点连续性检查触发拒绝，后续重试恢复；未进一步确定该次首点偏差根因。仍有剩余轨迹执行时间不足引发的受控停止。No grid/no frontier 不等于完整覆盖。两轮结果不证明长期零拒绝、连续无碰撞、原版四机 pillar 完整复现或旧 Warehouse 窗墙问题已解决。磁参考差异未修改，平移式 ENU 注册的限制仍保留。

精简机器证据见 [forest-short-validation.json](forest-short-validation.json)。原始失败和历史候选不删除；[validation.md](validation.md)按时间保留历史结果，旧场景与新场景分别判断。

## 准备与复跑

以下在 AeroHub 仓库根目录的 PowerShell 执行。主机需要 Python 3.11+ 和可访问的 Docker；四个镜像必须已在本地。复制 [forest-short.local.example.json](forest-short.local.example.json) 到不纳入 Git 的本地目录，填入候选、settings、新运行目录、project、未占用 DDS domain 和 Docker 可达的 AirSim 地址。网络地址由本机检查后选择，不能直接照抄其他机器。诊断参数保持模板值。

`check_baseline.py` 只读核对已验收源码、所有 manifest 产物、依赖与传感器文件、settings 和四个镜像，不启动服务：

```powershell
python deployment/airsim/demos/racer_multi_flight/check_baseline.py --local <local.json> --images
```

必须检查退出码为 0。检查通过不代表场景已加载或资源空闲：确认当前为 ForestScene、使用与清单一致的 settings、上一轮两机已落地解除武装且相关容器已停止；UE Stop/Play 刷新会话。确认本地配置的端口、DDS domain、project 和可选 docker_subnet 不与其他部署冲突。settings 文件校验不能替代实际 UE 加载配置的核对；地图依赖 UE 工程资源，本清单未锁定整套资产。

下面的命令会实际起飞，调用两轮使用的同一 `full_flow.execute` 和 `FlightDeployment`：

```powershell
python deployment/airsim/demos/racer_multi_flight/full_flow.py --local <local.json>
```

入口从分阶段启动开始，完成地面检查、传感器、起飞柱检查、算法/Adapter、ULog、双机探索、停止降落与清理。新 run_root 必须不存在；不要重用旧证据目录。没有自动追加第二轮、重试或扩大探索范围。上一轮独立的主机包装器用于场景确认，不是算法运行依赖；本次交付直接使用正式 CLI，飞行代码与两轮记录逐文件一致。

## 验收与诊断读回

```powershell
python deployment/airsim/demos/racer_multi_flight/evaluate.py --run-root <run-root>
python deployment/airsim/demos/racer_multi_flight/readback.py --run-root <run-root> --release <release> --output <new-readback-output>
```

分别核对退出码、flow-result.json、evaluation.json 和读回目录 recording-result.json。读回在所有记录器退出后进行，以固定 onboard 镜像、`--network none` 和只读证据挂载解析真实载荷，不连接飞控。它只证明记录完整性和原生 plan/query 关联，不替代飞行、避障或执行验收。新入口纳入原已验证的 readback_flight.py，保留地面专属检查不适用项；没有删除飞行检查来获得通过。每次读回写新目录，不覆盖原记录。

## 停止与回退

正常完整入口自行停止探索、降落、联合等待 landed/disarmed、导出 ULog 并清理。`run.py stop` 和 `down` 是落地后的资源清理，不是空中停止命令。

需要中断已运行任务时，先核对 `<project>-mission` 容器的 `aerohub.racer.owner` 标签与本轮 project 一致，再向它发送 `docker kill --signal=SIGINT <project>-mission`。代码将两机置为停止请求并进入各自恢复；继续等待任务结果。不要先停时钟、PX4 或 UE。此人工中断路径未纳入本次两轮故障注入验收。启动失败或落地未确认时保留服务，检查实际 PX4 状态与 flow-stages.json；不强制解除武装，不盲目重复完整入口。

只有两机地面确认后，才可用 `run.py --local <local.json> logger-stop`、`stop`、`down` 补做尚未完成的阶段；先核对 flow-stages.json，避免重复导出已完成日志。

回退采用停机切换：确认两机落地解除武装、回收本轮资源，选取旧版完整源码快照和与其匹配的 release，在独立目录恢复，保留当前工作区和旧证据，刷新 UE 与地图状态。先执行对应版本的校验，再走地面与飞行验证。仅换 release 路径而保留不同 native 源码会被拒绝。没有自动/热回退，也未以本次归档验证跨版本飞行回退。

## 构建与交付边界

固定候选通过已有完整构建后逐项核验依赖、增量编译 exploration_manager 得到；没有 ABI 布局变化。原 manifest 的 commands 是历史构建链，含旧目录；`exit_code: 143` 为继承字段，不能当作本轮构建退出码。当前成功依据是末次增量构建脚本的成功分支、日志、节点加载检查与该候选的实际飞行记录，原 manifest 不追改。

需要重新构建时使用仓库已有入口，必须选择新目录：

```powershell
python deployment/airsim/demos/racer_single/build.py --release <new-release> --deps <locked-dependency-cache>
```

这是构建入口，重新生成的候选不自动等同已飞行版本，也不能复用此清单的产物通过结论；按新的 manifest 校验并验收。本次没有从空环境重建，固定共享镜像、依赖缓存、UE 资产都是外部前置条件。源码快照保留在本地交付目录，既有大体积 install 和 bag 继续使用原数据盘目录；通过 SHA256 关联，不复制 UE 工程、不覆盖原候选。


## 本次固化检查

未启动新的飞行。已验收的运行代码与第2轮源码快照一致；本次仅更新交付元数据并新增只读校验/离线读回入口。673项初始源码身份校验与全部2075项manifest产物校验通过，另锁定上游来源文件及8项校验测试。四个本地镜像ID一致。正式读回入口对第2轮数据的两机各16项检查通过，完整JSON结果与原临时入口报告逐项一致。入口校验的8项正反检查通过，源码快照可独立执行同一身份检查。上述检查不替代新飞行、全量干净构建或回退飞行验收。
