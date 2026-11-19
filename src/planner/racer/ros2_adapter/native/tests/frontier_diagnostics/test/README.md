# 原生前沿决策记录离线对照

本测试针对 RACER 的 `FrontierFinder`，不是通用无人机验收门限。
使用合成地图调用实际安装库中的筛选、分类和保留/删除代码；不启动 PX4、AS2、AirSim 或运动客户端。

## 输入与运行

需要两个按相同工具镜像构建的 release：未增加该记录的基线，以及增加记录后的候选。
两个 release 均应包含 `manifest.json`、`native/install` 和已校验的 `deps/nlopt-debs`。
提供同一份实际运行产生的 `native-params.yaml`；本组边界夹具使用该 RACER 案例的
0.1 m 分辨率、30 个可见点门限及 1–1.5 m 候选半径，不适用于任意算法配置。
夹具临时设置地图、规划盒和一个视场距离用例，绝不写回运行配置。

在有 Docker 和 Python 的宿主执行（路径由使用者指定）：

```text
python run_probe.py --baseline --release <基线release> --params <native-params.yaml> --output <新基线结果目录>
python run_probe.py --release <候选release> --params <同一native-params.yaml> --output <新候选结果目录>
python test/verify.py <基线结果目录> <候选结果目录>
```

容器使用 manifest 中固定的 algorithm image，禁止拉取和外网访问，产物只写入指定结果目录。
`probe.cpp` 用测试编译单元的私有成员访问设置地图，不修改生产类的访问权限。
分别编译到匹配 ABI 的基线/候选库，比较完整视点位置、航向、可见点数和分类结果。

八个采样用例覆盖边界外、膨胀占据、邻近未知、严格 `>30` 的两侧、视场外、
射线未知遮挡和射线膨胀遮挡。另验证同一休眠 UID 保留时不重新采样、删除后新建 UID、
最终停止快照，以及关闭开关时没有新增前沿记录。
`check_entry.py` 在镜像内执行实际 `multi/native.py` 的参数生成部分，停在 `rclpy.init` 前；
检查缺省、总开关关闭和子开关开启等配置传递。它不证明完整 Supervisor 进程或 DDS 就绪。

## 结论边界

读回通过只能证明这些合成输入下的记录内容、生命周期关联和算法输出一致。
不重建旧飞行的完整地图，不解释旧休眠簇的确切失败条件，不证明动态负载、飞行避障、
长时间磁盘开销或混合板端部署通过。节点构造/退出冒烟与本测试、真实地面和飞行验收分别记录。
