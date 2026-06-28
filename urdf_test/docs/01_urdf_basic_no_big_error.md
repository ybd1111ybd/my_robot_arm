# 01 URDF 基本没有大错检查

## 目标

本阶段只判断一件事：

```text
URDF 是否存在明显大错。
```

这里的“没有大错”不是说 URDF 已经高精度标定，而是说它可以作为当前 IK、轨迹追踪、VR 可视化遥操作的基础模型继续使用。

通过本阶段后，可以认为：

```text
关节名基本对
关节链拓扑基本对
每个关节控制的是预期那一段 link
关节轴大方向基本对
左右臂镜像关系基本合理
link6/joint6 是腕部左右
link7/joint7 是腕部上下
```

不能证明：

```text
TCP 完全准确
零位完全准确
link 长度精确到毫米
惯量和质量正确
碰撞模型正确
真实机器人 FK 和 URDF FK 高精度一致
```

## 背景判断

当前轨迹追踪已经基本没有问题，这说明 URDF 没有明显的整体性错误，例如整条链断掉、关节顺序大错、模型无法 IK 等。

但轨迹能追不等于 URDF 完全正确。IK 和控制映射可能会补偿一些局部问题，例如：

```text
某个关节方向反了
TCP 点偏了
左右臂镜像符号有差异
某个腕部轴和控制习惯不一致
```

所以第一阶段最重要的是做单关节 sweep。

## 检查方式

每次只动一个关节，其它关节保持 home 或零位。

建议每个关节看两个方向：

```text
+10 deg
-10 deg
```

观察三件事：

```text
1. 动的是不是这个关节后面的那一段 link。
2. 旋转轴是不是预期方向。
3. 正方向是不是符合控制习惯。
```

## 重点关节语义

当前项目里最重要的腕部语义是：

```text
link6 / joint6：腕部左右动
link7 / joint7：腕部上下动
```

检查时尤其要确认：

```text
left_joint6  +10 deg / -10 deg 是否表现为左腕左右
left_joint7  +10 deg / -10 deg 是否表现为左腕上下
right_joint6 +10 deg / -10 deg 是否表现为右腕左右
right_joint7 +10 deg / -10 deg 是否表现为右腕上下
```

如果 joint6 和 joint7 的语义反了，或者某个轴明显不是预期方向，这属于需要优先处理的问题。

## 双臂检查表

建议记录成下面这种表。

```text
关节名        +10 deg 现象        -10 deg 现象        是否通过        备注
left_joint1
left_joint2
left_joint3
left_joint4
left_joint5
left_joint6
left_joint7
right_joint1
right_joint2
right_joint3
right_joint4
right_joint5
right_joint6
right_joint7
```

每行只需要写清楚：

```text
OK
方向反
轴不对
动错 link
mesh 明显偏
不确定，需复查
```

## 什么算大错

下面这些属于明显大错：

```text
动 left_joint，右臂也跟着动。
动某个 wrist joint，肩膀或肘部像主关节一样大幅运动。
动 joint6，结果表现成上下，而不是左右。
动 joint7，结果表现成左右，而不是上下。
某个 joint 的 child link 断开、飞走、绕奇怪点旋转。
parent/child 链明显接错。
左右臂不是镜像关系，而是明显一边接错。
mesh 和对应 link frame 分离很远，影响判断。
```

发现这类问题时，不建议继续调 VR 手感，应先回到 URDF 关节定义、origin、axis、parent/child 检查。

## 什么不一定算大错

下面这些不一定说明 URDF 大错，但必须记录：

```text
正方向和控制习惯相反。
左右臂某个关节符号相反。
零位姿态有小偏差。
TCP 点不是想象中的夹爪/工具中心。
link9/link10 哪个更适合作为控制末端还不确定。
某些 mesh 缺失或外观不完整。
```

方向反尤其常见。方向反可能来自：

```text
URDF axis 符号
控制器 joint sign
手柄映射 sign
左右臂镜像定义
```

不要一看到方向反就直接改 URDF。先记录现象，再决定应该在 URDF、driver，还是 teleop mapper 里修。

## 建议验收标准

第一阶段通过标准：

```text
14 个双臂关节单独 sweep 时：
  动作链正确
  旋转轴大体正确
  左右臂镜像关系合理
  link6/joint6 表现为腕部左右
  link7/joint7 表现为腕部上下
```

如果全部通过，可以写结论：

```text
URDF 没有明显大错。当前模型足够用于 light_teleop 的 IK、轨迹追踪和 VR 调参。
```

更严谨的表述是：

```text
URDF 的拓扑和关节轴大体可信，但尚未完成高精度 FK/TCP/物理属性标定。
```

## 和后续阶段的关系

第一阶段通过后，就可以继续做运控链路判断。

如果第一阶段不通过，不建议继续判断运控，因为运控表现可能被错误 URDF 干扰。

推荐顺序：

```text
第一阶段：URDF 单关节 sweep，排除大错。
第二阶段：运控/映射链路检查，确认控制没有明显问题。
第三阶段：物理属性检查，仅在需要物理仿真时做。
```
