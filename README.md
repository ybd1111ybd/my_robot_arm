# my_robot VR 遥操作仿真工程说明

本工程用于把 VR 手柄 UDP 数据映射成双臂末端目标，通过 Pink 逆运动学求解关节角，并用 Meshcat 做可视化仿真。当前 demo 只做可视化和链路验证，不向真实机器人下发控制命令。

## 工程结构

```text
/home/ybd/my_robot
├── light_teleop/                         # 当前主要开发模块
│   ├── light_teleop/
│   │   ├── vr_visual_demo.py             # VR 遥操作主程序
│   │   ├── pink_solver.py                # Pink IK 求解封装
│   │   ├── model_loader.py               # URDF/模型加载
│   │   └── visualizer.py                 # Meshcat 可视化
│   ├── config/vr_visual_demo.yml         # VR demo 默认参数
│   └── scripts/
│       ├── run_vr_demo.sh                # 启动 VR 可视化 demo
│       └── check_vr_udp.sh               # 检查 VR UDP 数据
├── jz_descripetion-main/robot_urdf/      # 机器人 URDF 和 mesh
├── test/teleop_vr_py/                    # VR UDP 解析、坐标映射、按钮处理
├── test/recv_vr_udp.py                   # 手柄 UDP 接收测试脚本
└── teleop_vr_recv-main/                  # 原始 VR 接收工程参考
```

## 当前环境

本机路径：

```bash
cd /home/ybd/my_robot
```

Python 环境：

```text
/home/ybd/my_robot/.venv-light-tp
```

运行脚本默认使用：

```bash
/home/ybd/my_robot/.venv-light-tp/bin/python
```

当前 WSL/主机网络测试中使用：

```text
WSL IP: 10.1.42.44
VR/headset IP: 10.1.42.5
UDP port: 8080
```

Meshcat 可视化地址：

```text
http://127.0.0.1:7000/static/
```

## 运行前检查手柄 UDP

先确认 VR 手柄数据能收到：

```bash
cd /home/ybd/my_robot

./light_teleop/scripts/check_vr_udp.sh 0.0.0.0 8080
```

或者直接运行：

```bash
python3 test/recv_vr_udp.py --host 0.0.0.0 --port 8080
```

正常时会持续看到：

```text
[packet #...] from 10.1.42.5:...
left_controller ...
right_controller ...
```

如果只看到 Windows 本机发来的 4 字节短包，说明 WSL UDP 接收是通的，但头显端 VR 程序没有正确发数据。

## 推荐启动命令

当前实测较好的参数：

```bash
cd /home/ybd/my_robot

./light_teleop/scripts/run_vr_demo.sh 10.1.42.44 8080 \
  --left-ee-frame left_arm_link7 \
  --right-ee-frame right_arm_link7 \
  --frequency 80 \
  --orientation-cost 0.15 \
  --joint-motion-cost 0.10 \
  --joint-motion-cost-profile shoulder-heavy
```

说明：

```text
--left-ee-frame/right-ee-frame link7
```

控制目标放在第七轴后的 link7 frame 附近，而不是更外侧的 link9。

```text
--frequency 80
```

VR 实测发送频率约 77Hz，IK 频率设为 80Hz 和输入频率更匹配。

```text
--orientation-cost 0.15
```

降低完整 VR 姿态跟随权重。这样保留 z 轴旋转，但不会让姿态目标过强地带动全臂乱动。

```text
--joint-motion-cost 0.10 --joint-motion-cost-profile shoulder-heavy
```

启用关节运动惩罚，尤其抑制肩部大幅动作。

## 不带关节惩罚对照

用于比较原始 IK 效果：

```bash
cd /home/ybd/my_robot

./light_teleop/scripts/run_vr_demo.sh 10.1.42.44 8080 \
  --left-ee-frame left_arm_link7 \
  --right-ee-frame right_arm_link7 \
  --frequency 80 \
  --orientation-cost 0.15 \
  --joint-motion-cost 0.0
```

## 当前默认配置

默认配置文件：

```text
light_teleop/config/vr_visual_demo.yml
```

当前关键默认值：

```yaml
host: 10.1.42.44
port: 8080
target_orientation_mode: vr
tcp_control_offset: 0.0
vr_debug_target_forward_offset: 0.0
frequency: 80
display_every: 2
arm_meshes_only: true
```

注意：配置文件默认 `left_ee_frame/right_ee_frame` 仍是 `link9`。当前推荐命令里显式覆盖为 `link7`。

## 手柄操作

```text
Grip 按住：开始跟随当前手柄相对运动
Grip 松开：保持最后目标
X/A primary button：复位到 home/reference
```

日志中会看到：

```text
Grip pressed, relocked VR reference
Grip released, holding last target
primary button pressed, reset reference/home target
```

复位逻辑已经做过修正：按 primary 复位时，会跳过当前帧后续 VR 目标重算，避免刚复位又被同一帧手柄目标拉走。

## 可视化标记含义

```text
青色/洋红小球：原始 VR 手柄点
绿色方块：映射后的目标，限速前
红色方块：真正发送给 Pink 的目标，限速后
蓝色/黄色小球：Grip 绑定参考点
```

绿色/红色方块显示偏移由配置控制：

```yaml
vr_debug_target_forward_offset: 0.0
vr_debug_target_offset_axis: z
```

`0.0` 表示显示方块不额外偏移。

## 目标点和 URDF 的关系

目标点不是在 XML 里直接写死的。

URDF 定义 link/joint/frame 的位置：

```text
jz_descripetion-main/robot_urdf/urdf/robot urdf.10.8.SLDASM.urdf
```

VR demo 根据运行时参数选择控制哪个 frame：

```text
--left-ee-frame left_arm_link7
--right-ee-frame right_arm_link7
```

当前目标是 `link7 frame`，不一定严格等于肉眼看到的 joint7 机械轴心。

## IK 解算思路

当前用 Pink 做差分逆运动学：

```text
VR 手柄数据
-> 坐标映射
-> 生成左右手目标 SE3
-> Pink FrameTask 追踪左右末端 frame
-> PostureTask 保持姿态偏好
-> 可选 Joint Motion Task 惩罚相邻帧关节变化
-> Meshcat 显示模型和目标
```

主要任务：

```text
FrameTask: 控制左右末端位置和姿态
PostureTask: 让关节姿态有偏好，避免完全无约束
MotionTask: 惩罚当前帧相对上一帧的关节变化
```

由于双臂关节较多，IK 解不唯一。同一个末端目标可能有很多组关节解，所以需要通过姿态权重、关节惩罚和目标位置选择来约束解的形态。

## 已做过的关键优化

### 1. 关节运动惩罚

加入 `--joint-motion-cost`，用于惩罚相邻两帧关节变化。

实测：

```text
joint_motion_cost=0.10
```

能降低平均关节运动量，但过大可能牺牲末端跟随。

### 2. 姿态权重降低

默认完整 VR 姿态模式：

```yaml
target_orientation_mode: vr
```

这样 z 轴可以旋转，但姿态变化较大时会拉动关节。把姿态权重从 `0.35` 降到 `0.15` 后，效果明显改善。

实测结果之一：

```text
orientation_cost=0.15
joint_motion_cost=0.10
joint_step_l2_mean=0.01394
joint_step_l2_max=0.09015
shoulder_step_abs_max=0.02129
final_left_error=0.000000
final_right_error=0.000000
```

### 3. 解算频率调整

VR UDP 实测约：

```text
773 packets / 10s = 77.3Hz
```

因此当前推荐：

```text
--frequency 80
```

如果 IK 频率高于 VR 输入频率，不会产生新的手柄信息，只是在同一个目标上多解几次。80Hz 和 VR 输入更匹配，Meshcat/CPU 压力也更低。

### 4. 目标偏移归零

之前目标方块和 TCP 控制点有 10cm 外偏。现在默认：

```yaml
tcp_control_offset: 0.0
vr_debug_target_forward_offset: 0.0
```

## 常用指标解释

程序每隔一段时间会打印：

```text
final_left_error / final_right_error
joint_step_l2_mean
joint_step_l2_max
joint_step_abs_max
shoulder_step_abs_max
joint_vel_abs_max
shoulder_vel_abs_max
dropped_old_packets
```

含义：

```text
final_left_error/final_right_error
```

末端目标误差，越小越跟手。

```text
joint_step_l2_mean
```

平均每帧整体关节运动量，越小越平滑。

```text
joint_step_l2_max
```

最大单帧整体跳动，越小越不容易突然抖。

```text
joint_step_abs_max
```

单个关节最大单帧变化量。

```text
shoulder_step_abs_max / shoulder_vel_abs_max
```

肩部最大单帧动作和最大速度，越小越不容易肩部乱甩。

```text
dropped_old_packets
```

程序主动丢弃旧 UDP 包，只使用最新包，避免跟随过期手柄位置。它不一定代表网络错误。

## 频率和单帧时间

```text
dt = 1 / frequency
```

例如：

```text
120Hz -> 8.33ms
80Hz  -> 12.5ms
```

频率越低，每帧时间越长。同样速度上限下，每帧允许的关节变化会更大。

例如最大关节速度 6 rad/s：

```text
120Hz: 6 * 0.00833 = 0.050 rad
80Hz:  6 * 0.0125 = 0.075 rad
```

所以 80Hz 下 `joint_step_abs_max` 可能比 120Hz 大，这是正常现象。

## 常见问题

### 收不到手柄

先跑：

```bash
python3 test/recv_vr_udp.py --host 0.0.0.0 --port 8080
```

如果 WSL 能 ping 到头显，但 tcpdump 看不到 UDP，通常是头显端 VR 发送程序没有发到当前 WSL IP/端口。

### 绿色方块偏移

检查：

```yaml
tcp_control_offset: 0.0
vr_debug_target_forward_offset: 0.0
```

如果命令里显式传了其它值，会覆盖 yml。

### z 轴不能旋转

检查：

```yaml
target_orientation_mode: vr
```

如果是 `wrist-decoupled`，代码会锁住局部 z 轴自转。

### 复位后马上又被拉走

已修复。复位时会跳过当前帧目标重算，避免同一帧把 home 目标覆盖掉。

## Git 说明

当前环境根目录存在一个只读空 `.git`，普通 git 无法使用。因此实际仓库元数据在：

```text
.git-real
```

常用命令：

```bash
git --git-dir=.git-real --work-tree=. status
git --git-dir=.git-real --work-tree=. add light_teleop
git --git-dir=.git-real --work-tree=. commit -m "message"
git --git-dir=.git-real --work-tree=. push
```

远端：

```text
https://github.com/ybd1111ybd/my_robot_arm.git
```

