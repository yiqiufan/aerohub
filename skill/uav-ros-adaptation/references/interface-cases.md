# 参数接口适配案例

## RACER 可选诊断参数

空探针列表进入ROS YAML后造成原生节点参数错误，飞行在起飞前失败。按可选参数语义省略空列表后，真实节点隔离检查覆盖该入口，随后飞行通过。未用伪造探针填充，也没有修改规划范围；具体接口证据见[诊断飞行摘要](../../../deployment/airsim/demos/racer_multi_flight/diagnostic-flight-validation.json)。该方法适用于有可选数组的目标参数接口；必填数组须按原契约报错，不能一概省略。
