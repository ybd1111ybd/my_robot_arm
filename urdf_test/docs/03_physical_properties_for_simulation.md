# 03 物理属性检查

## 目标

本阶段只在需要物理仿真时才做。

如果当前目标只是：

```text
轨迹追踪
IK
VR 可视化遥操作
Meshcat/RViz 观察运动学
```

那么前两个阶段通过就基本够用了：

```text
01 URDF 基本没有大错
02 运控链路没有明显问题
```

第三阶段关注的是：

```text
质量
质心
惯量
碰撞几何
摩擦
阻尼
关节动力学参数
物理仿真稳定性
```

这些参数对运动学 IK 不一定有影响，但会显著影响 Gazebo、MuJoCo、Isaac Sim 等物理仿真。

## 为什么物理属性要单独判断

URDF 里有三类信息：

```text
运动学：
  link、joint、origin、axis、limit。

视觉：
  visual mesh、颜色、显示 origin。

物理：
  inertial、mass、inertia、collision、dynamics、friction。
```

前两个阶段主要检查运动学和控制链路。

物理仿真需要额外检查：

```text
link 会不会因为质量/惯量错误而乱飞。
碰撞模型会不会穿透或卡住。
关节会不会抖动。
重力下机器人会不会不合理下坠。
接触时是否稳定。
```

所以不要用“轨迹追踪没问题”来证明物理属性正确。

## 什么时候需要做第三阶段

需要物理仿真时做，例如：

```text
在 Gazebo/MuJoCo/Isaac 中跑动力学。
需要重力、碰撞、接触、抓取仿真。
需要验证关节力矩、电机负载。
需要模拟夹爪接触物体。
需要做控制器动力学调参。
```

不需要物理仿真时，可以先不做，例如：

```text
只做 Pinocchio FK/IK。
只做 Meshcat 运动学可视化。
只做 VR target 映射。
只做关节轨迹可视化。
```

## 检查一：inertial 是否完整

每个参与仿真的 link 都应该有：

```xml
<inertial>
  <origin xyz="..." rpy="..." />
  <mass value="..." />
  <inertia ixx="..." ixy="..." ixz="..." iyy="..." iyz="..." izz="..." />
</inertial>
```

检查项：

```text
mass 是否为正数。
inertia 对角项是否为正数。
inertia 矩阵是否合理。
质心 origin 是否在 link 附近。
是否有 link 缺失 inertial。
是否有明显复制粘贴的惯量。
```

常见问题：

```text
质量为 0。
惯量为 0。
质量巨大或极小。
质心离 link 很远。
所有 link 惯量完全一样。
```

这些会导致仿真抖动、乱飞或控制器异常。

## 检查二：质量是否合理

先做量级检查，不需要一开始追求精确。

建议检查：

```text
整机质量是否接近真实机器人。
左右臂质量是否大体对称。
同类 link 质量是否合理。
末端小部件质量是否没有离谱。
```

判断例子：

```text
一个手腕小 link 质量几十公斤 -> 明显错误。
一个大臂 link 质量 0.001 kg -> 明显错误。
左右对称 link 质量差很多 -> 需要解释。
```

## 检查三：惯量是否物理可行

惯量矩阵应该是正定或至少合理的半正定。

基本检查：

```text
ixx > 0
iyy > 0
izz > 0
惯量量级和 mass/link 尺寸匹配
```

更严格的检查：

```text
惯量矩阵 eigenvalue 都大于 0。
满足三角不等式近似：
  ixx + iyy >= izz
  ixx + izz >= iyy
  iyy + izz >= ixx
```

如果不满足，物理仿真可能不稳定。

## 检查四：collision 几何是否可用

物理仿真主要依赖 collision，不是 visual。

检查项：

```text
每个需要接触的 link 是否有 collision。
collision mesh 是否存在。
collision origin 是否和 visual/link 大体对齐。
collision 是否过大导致自碰撞。
collision 是否过小导致穿透。
是否可以用简单几何替代复杂 STL。
```

建议：

```text
用于动力学仿真时，collision 尽量用 box/cylinder/sphere 或简化 mesh。
不要直接用过于复杂的 visual STL 做碰撞，容易慢且不稳定。
```

## 检查五：关节 limit 是否合理

物理仿真需要关节限制：

```xml
<limit lower="..." upper="..." effort="..." velocity="..." />
```

检查项：

```text
lower/upper 是否和真实机械限位一致。
effort 是否为正且量级合理。
velocity 是否为正且量级合理。
是否有上下限写反。
是否有过大范围导致仿真穿模。
```

运动学 IK 阶段只要不越界通常还能用，但物理仿真中错误 limit 会导致更明显问题。

## 检查六：joint dynamics

URDF 可包含：

```xml
<dynamics damping="..." friction="..." />
```

如果缺失，仿真不一定不能跑，但可能：

```text
关节太滑。
停止后抖动。
控制器需要非常强的阻尼才能稳定。
```

初期建议：

```text
先给合理的小 damping。
不要一开始把 friction/damping 写得极大。
```

## 检查七：重力下静态稳定

把机器人放进物理仿真，先不做复杂控制，只测：

```text
固定 base。
启用重力。
关节保持 home 或零位。
观察是否稳定。
```

异常现象：

```text
link 爆炸飞走。
关节疯狂抖动。
手臂在重力下不合理穿模。
某些 link 因碰撞模型互相挤压而弹开。
机器人整体质心明显离谱。
```

这些通常说明 inertial/collision/joint limit 有问题。

## 检查八：简单动作仿真

静态稳定后，再做简单动作：

```text
单关节低速运动。
双臂小幅运动。
腕部 joint6/joint7 小幅运动。
末端靠近桌面或物体但不接触。
```

观察：

```text
关节是否平滑。
控制器是否需要异常大的力矩。
是否出现抖动或穿透。
碰撞是否过早触发。
```

不要一开始就做抓取接触。抓取会同时暴露碰撞、摩擦、控制器、接触参数问题，定位困难。

## 检查九：接触和抓取

只有前面都稳定后，再测：

```text
夹爪接触物体。
轻微推物体。
抓取物体。
双臂靠近物体。
```

需要关注：

```text
摩擦系数。
接触刚度/阻尼。
夹爪 collision 形状。
物体质量。
控制器带宽。
```

接触仿真不稳定不一定是 URDF 运动学错，更多时候是物理参数和仿真器接触参数问题。

## 和前两个阶段的关系

三个阶段的关系：

```text
01 URDF 基本没有大错：
  主要判断运动学拓扑和 joint axis。

02 运控链路没有明显问题：
  主要判断 target、IK、映射、控制方向和跟手性。

03 物理属性检查：
  主要判断动力学仿真是否可信。
```

如果只做当前 light_teleop 的 VR 可视化遥操作：

```text
01 + 02 基本够用。
```

如果要做物理仿真：

```text
必须做 03。
```

## 本阶段通过标准

可以认为物理属性初步可用，当满足：

```text
主要 link 都有合理 mass/inertia。
inertia 没有明显非物理值。
collision mesh 存在且位置合理。
joint limit/effort/velocity 合理。
重力下模型稳定。
单关节低速动作稳定。
简单碰撞不爆炸、不明显穿透。
```

更严格的通过标准需要真实数据：

```text
整机质量和真实值接近。
各 link 质心和 CAD/测量值接近。
关节力矩和真实负载趋势接近。
抓取接触行为和真实机器人接近。
```

## 当前建议

当前阶段如果目标仍是：

```text
VR 遥操作
轨迹追踪
IK 调参
Meshcat 可视化
```

不要把时间先花在物理属性上。优先确保：

```text
单关节 sweep 没大错。
运控链路方向和跟手性没大错。
TCP 控制点定义合理。
```

等需要 Gazebo/MuJoCo/Isaac 物理仿真时，再启动本文档中的第三阶段。
