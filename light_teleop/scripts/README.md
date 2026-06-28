# VR demo 启动命令说明

本文档记录 `light_teleop/scripts` 下两个脚本的用法，以及当前推荐的 VR 可视化 demo 启动命令。

## 1. 先检查手柄 UDP

运行 demo 前，先确认头显/手柄 UDP 数据能收到：

```bash
cd /home/ybd/my_robot

./light_teleop/scripts/check_vr_udp.sh 0.0.0.0 8080
```

正常时会看到类似：

```text
[packet #...] from 10.1.42.5:...
```

如果收不到，先不要跑 demo，优先检查头显发送端是否发到当前 WSL IP 和端口。

## 2. 不带关节运动惩罚启动

这个命令用于观察原始 IK 效果，方便和带惩罚的版本对比：

```bash
cd /home/ybd/my_robot

./light_teleop/scripts/run_vr_demo.sh 10.1.42.44 8080 \
  --left-ee-frame left_arm_link7 \
  --right-ee-frame right_arm_link7 \
  --tcp-control-offset 0.0 \
  --vr-debug-target-forward-offset 0.0 \
  --joint-motion-cost 0.0
```

## 3. 带关节运动惩罚启动

这是当前推荐的 `link7` 测试命令：

```bash
cd /home/ybd/my_robot

./light_teleop/scripts/run_vr_demo.sh 10.1.42.44 8080 \
  --left-ee-frame left_arm_link7 \
  --right-ee-frame right_arm_link7 \
  --tcp-control-offset 0.0 \
  --vr-debug-target-forward-offset 0.0 \
  --joint-motion-cost 0.10 \
  --joint-motion-cost-profile shoulder-heavy
```

## 4. 参数含义

`10.1.42.44 8080`

WSL 接收 VR UDP 的 IP 和端口。当前 WSL IP 是 `10.1.42.44`，端口用 `8080`。

`--left-ee-frame left_arm_link7`

左臂 IK 控制的目标 frame。`left_arm_link7` 在最后一个可动关节之后，当前用于测试腕部附近的跟随效果。

`--right-ee-frame right_arm_link7`

右臂 IK 控制的目标 frame。

`--tcp-control-offset 0.0`

控制点相对所选 frame 的局部偏移距离。`0.0` 表示目标点就在 `link7` frame 上，不再往末端外延。

`--vr-debug-target-forward-offset 0.0`

可视化里绿色/红色目标方块的显示偏移。设成 `0.0` 可以让显示目标和实际控制目标一致。

`--joint-motion-cost 0.0`

关闭关节运动惩罚。用于观察原始 IK 解。

`--joint-motion-cost 0.10`

打开关节运动惩罚。它会惩罚相邻两帧之间的关节变化，减少关节突然大幅运动。

`--joint-motion-cost-profile shoulder-heavy`

关节运动惩罚的权重分布。`shoulder-heavy` 对肩部关节惩罚更重，用来减少肩部乱甩。

## 5. 如何对比效果

两次运行尽量做同一段手柄动作，然后 `Ctrl+C` 停止，比较最后一行 `stopped:` 里的指标：

```text
final_left_error
final_right_error
joint_step_l2_mean
joint_step_l2_max
joint_step_abs_max
shoulder_step_abs_max
joint_vel_abs_max
shoulder_vel_abs_max
```

重点看：

`final_left_error / final_right_error`

末端跟随误差，越小越跟手。

`joint_step_l2_mean`

平均每帧整体关节运动量，越小越平滑。

`joint_step_l2_max`

最大单帧整体跳动，越小越少突然跳。

`shoulder_step_abs_max / shoulder_vel_abs_max`

肩部最大单帧动作和最大速度，越小越不容易出现肩部大幅乱甩。

如果 `joint_step_abs_max` 一直是 `0.10000`，说明某个关节已经打到当前单帧运动上限。

## 6. 测试完整姿态旋转

默认 `wrist-decoupled` 模式会锁住目标方块绕局部 z 轴的自转。如果要测试完整手柄姿态，包括 z 轴旋转，可以额外加：

```bash
--target-orientation-mode vr \
--orientation-cost 0.8
```

完整命令示例：

```bash
cd /home/ybd/my_robot

./light_teleop/scripts/run_vr_demo.sh 10.1.42.44 8080 \
  --left-ee-frame left_arm_link7 \
  --right-ee-frame right_arm_link7 \
  --tcp-control-offset 0.0 \
  --vr-debug-target-forward-offset 0.0 \
  --target-orientation-mode vr \
  --orientation-cost 0.8 \
  --joint-motion-cost 0.10 \
  --joint-motion-cost-profile shoulder-heavy
```
