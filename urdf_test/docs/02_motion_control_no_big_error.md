# 02 运控链路没有明显问题检查

## 目标

本阶段判断：

```text
从控制输入到机器人运动表现这条链路是否存在明显问题。
```

这里的运控链路包括：

```text
关节命令或 target 输入
  -> 控制器/映射层
  -> 机器人关节运动
  -> FK/IK/可视化反馈
```

对于当前 `light_teleop`，重点是：

```text
VR 手柄输入
  -> target 映射
  -> Pink IK
  -> 关节轨迹
  -> Meshcat 可视化结果
```

如果未来接真机，还要加入：

```text
关节命令发布
  -> 真机 controller
  -> 实际关节反馈
```

## 前置条件

开始本阶段前，应该先完成第一阶段：

```text
URDF 单关节 sweep 没有明显大错。
```

如果 URDF 的 joint axis、parent/child、link6/link7 语义还不确定，不建议先判断运控。

## 当前项目的基本结论

当前轨迹追踪已经基本没问题，这说明：

```text
Pink IK 可以稳定求解。
URDF 大体可用。
target 到 IK 的链路基本通。
Meshcat 可视化链路基本通。
```

但这还不能证明完整运控没有问题。还需要单独看：

```text
方向是否符合控制习惯
左右臂是否镜像一致
target 是否跟手
是否有明显延迟
是否有限速/滤波导致的滞后
腕部自由度是否按预期参与
是否存在突然翻折或跳变
```

## 检查一：单轴输入是否单调

每次只做一种输入，观察输出是否单调、可解释。

建议按下面顺序测：

```text
手柄向前/后平移
手柄向左/右平移
手柄向上/下平移
手柄向内/外转腕
手柄向上/下翻腕
```

每次观察：

```text
绿色 target 是否按预期移动
红色 target 是否跟随绿色 target
机器人末端是否跟随红色 target
关节是否连续运动，没有跳变
```

如果绿色 target 就不对，问题多半在 VR 映射层。

如果绿色对、红色不跟，问题多半在限速或滤波。

如果红色对、机器人不跟，问题多半在 IK 权重、姿态约束、关节限制或 posture cost。

## 检查二：方向是否符合控制习惯

方向检查不要只看“能不能动”，要看“是不是朝人想要的方向动”。

当前腕部约定：

```text
link6 / joint6：左右
link7 / joint7：上下
```

当前配置中：

```yaml
target_orientation_mode: wrist-decoupled
wrist_left_right_gain: -1.3
wrist_up_down_gain: 1.3
```

如果左右方向反了，优先改：

```yaml
wrist_left_right_gain: 1.3
```

如果上下方向反了，优先考虑给上下通道加符号能力，或者检查 `wrist_up_down_gain` 是否需要允许负号。

注意：方向反不一定是 URDF 错，也可能是控制映射 sign 和用户习惯不一致。

## 检查三：target 跟手性

当前 VR debug marker 语义：

```text
绿色方块：映射后的 target，限速前。
红色方块：实际发送给 Pink 的 target，限速后。
```

判断方法：

```text
绿色跟手，红色慢：
  target_max_speed 或滤波导致滞后。

绿色本身不跟手：
  VR 坐标映射、Grip reference、target_orientation_mode 或 TCP offset 有问题。

红色跟手，机器人慢：
  IK 权重、posture cost、关节限制或显示频率问题。
```

当前配置：

```yaml
target_max_speed: 0.8
position_alpha: 0.25
rotation_alpha: 0.75
display_every: 2
```

排查滞后时可以临时设：

```yaml
target_max_speed: 0
```

如果红绿重合后手感明显改善，说明之前主要是限速滞后。

## 检查四：Grip reference 是否稳定

Grip 按下时会锁定：

```text
当前 VR 手柄 pose
当前机器人 TCP pose
```

Grip 松开后保持最后 target。

需要检查：

```text
按下 Grip 的瞬间 target 不应该跳很远。
松开 Grip 后 target 不应该漂。
再次按下 Grip 后应该从当前末端重新绑定，不应该回旧 reference。
X/A 回 home 后 reference 应该重置。
```

如果 Grip 按下瞬间跳变，常见原因：

```text
当前 FK pose 和 transformer 内部 reference 不一致。
TCP offset 没有同步到绑定 pose。
VR packet 姿态有异常。
```

## 检查五：TCP 点是否合理

当前不是控制 `link9` 原点，而是控制前移 TCP 点：

```yaml
tcp_control_offset: 0.10
tcp_control_offset_axis: z
```

判断方法：

```text
转腕时，绿色/红色方块是否像真正末端点一样运动。
真实末端是否比以前更跟手。
是否还出现“腕部点先到，真正末端后到”的现象。
```

如果 TCP 方向不对，优先调：

```yaml
tcp_control_offset: -0.10
```

或者：

```yaml
tcp_control_offset_axis: x
```

TCP 问题不一定是运控问题。它可能只是末端控制点定义不对。

## 检查六：IK 权重是否合理

当前关键配置：

```yaml
posture_cost: 0.1
posture_cost_profile: joint6-priority
position_cost: 0.55
orientation_cost: 0.35
```

判断方法：

```text
末端位置跟不上：
  position_cost 可能太低，或 target_max_speed/滤波导致滞后。

腕部不愿意动：
  posture_cost 对 joint6/joint7 可能太硬，或 orientation_cost 太低。

整体开始乱飘：
  posture_cost 太低，或 wrist gain 太大，或 orientation_cost 太高。

姿态跟不上但位置还行：
  orientation_cost 可能太低，或姿态模式不合适。
```

当前 `joint6-priority` 意图：

```text
让 joint6/link6 更愿意承担左右动作。
```

如果上下不够，可以先调：

```yaml
wrist_up_down_gain: 1.5
```

不要一开始就改 posture profile。

## 检查七：左右臂镜像一致性

双臂任务里，左右臂不一定要完全同号，但应该符合人的控制直觉。

建议做三个动作：

```text
左手向内抓，右手向内抓。
左手上翻腕，右手上翻腕。
左手左右摆，右手左右摆。
```

观察：

```text
两边是否都朝“向内抓”的方向。
是否一边灵敏、一边迟钝。
是否一边方向反。
```

当前 `wrist_left_right_gain` 是全局参数。如果发现一边对、一边反，后续应拆成：

```yaml
left_wrist_left_right_gain: ...
right_wrist_left_right_gain: ...
```

这属于运控/映射层问题，不一定是 URDF 大错。

## 什么算运控大错

下面属于明显运控问题：

```text
target 输入连续，但关节输出突然跳变。
Grip 绑定时 target 大幅跳。
松开 Grip 后 target 继续漂。
红色 target 正确，机器人明显朝反方向走。
左臂输入导致右臂运动。
关节持续冲到 limit 附近但 target 并不极端。
机器人 motion 和 target 显示完全不一致。
```

出现这些问题时，不建议继续调 gain，应先定位链路：

```text
VR packet
  -> smoother
  -> transformer
  -> target
  -> target speed limit
  -> Pink IK
  -> q result
```

## 什么算可以接受

下面情况在当前阶段可以接受：

```text
轻微滞后，但 target 和机器人方向一致。
红色 target 因限速跟着绿色走。
腕部灵敏度还需调参。
TCP 点还需精调。
某个方向的 sign 需要改配置。
```

这些不是运控大错，而是调参项。

## 本阶段通过标准

可以认为运控链路基本没有大错，当满足：

```text
绿色 target 跟随手柄方向正确。
红色 target 与绿色 target 关系可解释。
机器人末端跟随红色 target。
Grip 绑定/松开/回 home 行为稳定。
单轴输入没有明显耦合到不可解释方向。
左右臂都能做向内抓取动作。
link6/link7 能按当前约定参与。
没有突然翻折、跳变或反向逃逸。
```

通过后，可以说：

```text
当前 URDF + 映射 + IK + 可视化链路足够支持 light_teleop 继续调参和上层功能开发。
```

但还不能说：

```text
真机下发已经安全。
物理仿真参数已经正确。
碰撞和动力学行为可信。
```
