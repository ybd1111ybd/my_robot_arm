# light_teleop 模块任务与验收计划

## 1. 文档目的

本文档用于声明 `light_teleop` 各模块需要完成的任务、模块之间的交付边界、
阶段里程碑以及每个模块的验收标准。

本计划对应 [architecture.md](./architecture.md) 中定义的轻量遥操作架构：

```text
VR 手柄输入
  -> 输入解析
  -> 手柄相对运动映射
  -> Pink 逆运动学结算
  -> Meshcat 可视化
  -> 后续关节命令下发
```

阶段口径：

```text
第一阶段 = M1-M3：模型加载 + Pink 双臂 IK + Meshcat 可视化
第二阶段 = M4-M5：手柄输入 + Grip 相对遥操作
第一版   = M1-M6：可视化遥操作 + dry-run 下发接口，不接真机
真机版   = M7-M8：安全层 + 真实下发
```

当前控制范围只包含双臂 14 个关节：`left_joint1..7` 和 `right_joint1..7`。
腰、头、夹爪不参与 Pink 结算。夹爪开合由独立链路控制，本计划只保留夹爪
字段解析兼容。

## 2. 总体阶段划分

| 阶段 | 名称 | 目标 | 是否接真机 |
| --- | --- | --- | --- |
| M0 | 文档和结构确认 | 明确架构、模块、接口和验收标准 | 否 |
| M1 | 模型加载 | 加载 JZ URDF，确认双臂关节/frame，检查 mesh 缺失 | 否 |
| M2 | Pink 双臂 IK | 用模拟 target 驱动左右臂 IK | 否 |
| M3 | 可视化闭环 | Meshcat 显示机器人、末端和 target | 否 |
| M4 | 手柄输入接入 | 复用原版 VR UDP 协议，解析手柄状态 | 否 |
| M5 | 相对遥操作 | Grip 锁定后用手柄控制左右末端 | 否 |
| M6 | 下发接口预留 | 输出标准化关节命令，但默认不发真机 | 否 |
| M7 | 真机前安全层 | 加入限速、超时、急停、使能保护 | 否 |
| M8 | 真机下发 | 对接机器人控制器或 ROS 2 topic | 是 |

当前优先完成 M1 到 M3，然后再进入手柄接入。M1-M3 通过不要求夹爪 mesh
完整显示，但必须明确列出缺失资产并保证双臂运动学链路可运行。

## 3. 模块总览

| 模块 | 文件建议 | 核心职责 | 当前优先级 |
| --- | --- | --- | --- |
| 模型加载 | `model_loader.py` | 加载 URDF、解析关节和 frame | 高 |
| Pink 结算 | `pink_solver.py` | 创建任务、执行 IK、输出关节状态 | 高 |
| 可视化 | `visualizer.py` | Meshcat 显示机器人和目标 frame | 高 |
| 手柄接收 | `vr_receiver.py` | UDP 接收原始 bytes | 中 |
| 手柄解析 | `vr_parser.py` | 解析原版 VR 数据协议 | 中 |
| 相对映射 | `teleop_mapper.py` | 手柄相对运动到末端 target | 中 |
| 下发接口 | `command_publisher.py` | 将 Pink 输出转为控制命令 | 低 |
| 主循环 | `main.py` | 串联输入、映射、IK、可视化和下发 | 高 |
| 配置 | `config.py` 或简单 YAML | 管理路径、频率、frame、权重 | 中 |
| 测试工具 | `tools/` 或 `tests/` | 模拟 target、模拟手柄包、单元测试 | 中 |

## 4. 推荐目录结构

```text
light_teleop/
  docs/
    architecture.md
    plan.md
    执行.md
  light_teleop/
    __init__.py
    config.py
    model_loader.py
    pink_solver.py
    visualizer.py
    vr_receiver.py
    vr_parser.py
    teleop_mapper.py
    command_publisher.py
    main.py
  tests/
    test_vr_parser.py
    test_teleop_mapper.py
    test_model_loader.py
  tools/
    simulate_target.py
    send_fake_vr_packet.py
```

目录可以按实际开发逐步创建，不要求第一步全部实现。

## 5. 模块任务与验收标准

### 5.1 model_loader.py

目标：加载 `jz_descripetion-main` 的 URDF 和 meshes，生成 Pinocchio 可用的
机器人模型。

主要任务：

- 定义 URDF 默认路径。
- 定义 mesh/package 搜索路径。
- 检查 `package://jz_robot_description/...` 能否解析。
- 检查并报告当前 URDF 引用的缺失 mesh。
- 使用 Pinocchio 加载机器人模型。
- 构建或准备双臂 14 关节 reduced model。
- 输出 `robot.model`、`robot.data`、`visual_model`。
- 检查左右臂关节是否存在。
- 检查候选末端 frame 是否存在。
- 输出可用 frame 列表用于调试。

建议默认输入：

```text
jz_descripetion-main/robot_urdf/urdf/robot urdf.10.8.SLDASM.urdf
```

必须确认的关节：

```text
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

候选末端 frame：

```text
left_arm_link9   # 默认控制末端，末端法兰盘
right_arm_link9  # 默认控制末端，末端法兰盘
left_arm_link10  # 相机/附属挂载点，仅用于对比检查
right_arm_link10 # 相机/附属挂载点，仅用于对比检查
left_ee/right_ee 如果后续新增
```

验收标准：

- 能在 Python 中成功加载 URDF 的运动学模型。
- 能解析 `package://jz_robot_description/...` 的包路径。
- 能输出缺失 mesh 列表；当前夹爪相关 STL 缺失时允许几何降级。
- 能打印 robot model 的关节数量、frame 数量和关节名。
- 能找到全部左右臂 14 个关节。
- 能找到并可视化 `left_arm_link9/right_arm_link9` 等候选末端 frame。
- 能明确腰、头、夹爪不参与 IK。
- 能生成 Pink `Configuration` 所需的 `model`、`data` 和初始 `q`。

### 5.2 pink_solver.py

目标：用 Pink 作为唯一 IK 结算核心，把左右末端目标转换为关节状态。

主要任务：

- 创建 Pink `Configuration`。
- 创建左臂 `FrameTask`。
- 创建右臂 `FrameTask`。
- 创建只作用于双臂运动空间的 `PostureTask` 或等效姿态正则。
- 设置每个任务的初始 target。
- 每一帧调用 `solve_ik`。
- 对速度积分更新 `configuration.q`。
- 输出 `IkResult`，包含当前关节配置、左右臂关节值和左右当前末端位姿。
- 不输出 `JointCommand`，下发格式由 `command_publisher.py` 生成。

输入：

```text
left_target_to_world
right_target_to_world
dt
```

输出：

```text
q
left_joint_positions
right_joint_positions
left_ee_current_pose
right_ee_current_pose
```

验收标准：

- 给定静态 target 时，IK 能收敛到稳定姿态。
- 给定缓慢移动 target 时，左右臂能连续跟踪。
- 结算过程不出现 NaN 或 Inf。
- 输出关节值保持在 URDF 关节限制范围内。
- 腰、头、夹爪保持固定，不被 Pink 结算改变。
- `PostureTask` 能降低奇怪折叠姿态的出现概率。
- 结算频率在开发机上能稳定达到 50 Hz 以上。

### 5.3 visualizer.py

目标：用 Meshcat 显示机器人当前可用 meshes、当前末端 frame 和目标 frame。

主要任务：

- 启动 Meshcat visualizer。
- 显示 URDF 中当前可解析的 visual meshes。
- 对缺失 mesh 显示 warning，不阻塞双臂 IK 可视化。
- 每帧显示当前 `q`。
- 显示左末端当前 frame。
- 显示右末端当前 frame。
- 显示左目标 frame。
- 显示右目标 frame。
- 支持启动时打印 Meshcat URL。

显示对象：

```text
robot visual meshes
left_ee frame
right_ee frame
left_target frame
right_target frame
```

验收标准：

- 浏览器中能看到机器人当前可用模型几何。
- 缺失 mesh 会被明确报告；补齐或替换缺失 STL 前，不把“完整夹爪几何显示”
  作为 M3 必过项。
- `q` 更新时机器人姿态同步变化。
- 左右 target frame 可见。
- 当前末端 frame 和 target frame 能同时显示，便于调参。
- 能显示候选 TCP frame，用于确认最终 `left_ee/right_ee`。
- 可视化模块不依赖 VR 输入，可以配合模拟 target 单独运行。

### 5.4 vr_receiver.py

目标：复用原版手柄链路中的 UDP 接收方式，稳定接收 VR 数据包。

主要任务：

- 创建 UDP socket。
- 监听指定 host 和 port。
- 支持非阻塞或带 timeout 的接收。
- 返回原始 bytes。
- 支持干净退出。

默认参数：

```text
host = "0.0.0.0"
port = 8080
timeout_ms = 100
```

验收标准：

- 能监听 UDP 端口。
- 能接收原版 VR 发送端的数据包。
- 端口被占用时给出明确错误。
- 接收超时时主循环不会卡死。
- 关闭程序时 socket 能正确释放。

### 5.5 vr_parser.py

目标：解析原版 `teleop_vr_recv-main` 的 VR 数据协议，输出结构化手柄状态。

主要任务：

- 检查帧头 `0xAA 0xBB`。
- 检查数据包长度。
- 解析 torque。
- 解析 16 个 float 角度或夹爪值；这些字段第一版只作兼容和调试，不进入 Pink。
- 解析左右手柄 pose。
- 解析头显 pose。
- 解析左右手柄 input。
- 检查 NaN、Inf 和异常范围。
- 输出结构化数据对象。

输出对象建议：

```text
VrPacket
  torque
  angles[16]
  left_controller_pose
  right_controller_pose
  headset_pose
  left_input
  right_input
  has_vr_device_poses
  has_controller_inputs
```

验收标准：

- 能正确解析最小关节包。
- 能正确解析完整 VR pose + input 包。
- 错误帧头会被拒绝。
- 长度不足会被拒绝。
- NaN、Inf 和明显越界数据会被拒绝。
- 与 `teleop_vr_recv-main` 的数据字段语义保持一致。
- 明确 length 字段是 4 字节单位数，不是 packet byte length。

### 5.6 teleop_mapper.py

目标：把手柄 pose 转换为机器人左右末端 target pose。

主要任务：

- 维护左手和右手的遥操作状态机。
- Grip 按下时锁定 VR 参考位姿和机器人末端参考位姿。
- Grip 持续按住时计算相对位移和相对旋转。
- Grip 松开时保持上一次 target。
- A/X 按下时回到初始 target。
- 实现坐标轴映射。
- 实现比例缩放。
- 实现简单目标限速。
- 输出 Pink 使用的 SE3 target。
- `packet is None` 时返回上一帧 target 或 home target，不改变状态机。
- 对姿态映射使用明确的旋转矩阵/坐标基变换算法，不直接交换四元数分量。

状态：

```text
Idle
Locked
Tracking
Holding
Home
```

验收标准：

- Grip 未按下时，target 不随手柄乱动。
- Grip 第一次按下不会导致末端跳变。
- Grip 按住移动时，target 按相对增量移动。
- Grip 松开后，target 保持在最后位置。
- 再次按 Grip 时，从当前位置继续控制。
- A/X 能把 target 设置回预设初始位姿。
- 坐标映射方向符合预期：VR X/Y/Z 能映射到机器人 base_link 对应方向。
- 姿态映射有单轴旋转测试：绕 VR X/Y/Z 旋转时，机器人末端姿态方向符合预期。
- 没有 VR 数据时，主循环不阻塞，target 不跳变。

### 5.7 command_publisher.py

目标：为后续真机下发提供统一出口。第一阶段默认关闭。

主要任务：

- 定义标准关节命令数据结构。
- 接收 `IkResult`，由下发模块转换成 `JointCommand`。
- 支持 dry-run 模式。
- 后续支持 ROS 2 topic 或机器人 SDK 下发。
- 不修改 Pink 输出结果，只做格式转换和发布。

建议输出格式：

```text
JointCommand
  stamp
  left_joint_names
  left_positions
  right_joint_names
  right_positions
  mode
```

验收标准：

- dry-run 模式下能打印或记录关节命令。
- 输出关节名顺序固定。
- 输出关节名与 URDF 保持一致。
- 输出只包含双臂 14 个关节，不包含夹爪、腰、头。
- 默认不会对真机发命令。
- 后续替换具体下发接口时，不需要修改 `pink_solver.py`。

### 5.8 main.py

目标：串联所有模块，形成可运行主循环。

主要任务：

- 初始化配置。
- 加载模型。
- 初始化 Pink solver。
- 初始化可视化。
- 选择输入源：模拟 target 或 VR 手柄。
- 固定频率运行主循环。
- 每帧更新 target、IK、可视化。
- 可选调用 command publisher。
- 支持 Ctrl+C 干净退出。

验收标准：

- 能以模拟 target 模式启动。
- 能以 VR 输入模式启动。
- 主循环频率稳定。
- 任一模块异常时能给出明确日志。
- 退出时 UDP、Meshcat、下发接口能干净关闭。

### 5.9 config.py

目标：集中管理第一阶段必要配置，避免参数散落在代码中。

主要配置：

```text
urdf_path
package_dirs
left_ee_frame
right_ee_frame
left_joint_names
right_joint_names
control_frequency
ik_solver
ik_solver_default_policy
frame_task_cost
posture_task_cost
scale_factor
axis_mapping
udp_host
udp_port
publisher_enabled
```

验收标准：

- 所有路径和关键参数能从一个位置看到。
- 默认配置能在当前仓库结构下直接运行。
- 默认 package 路径能解析 `package://jz_robot_description/...`。
- 默认 solver 优先 `daqp`，不可用时使用 `qpsolvers.available_solvers[0]`。
- 配置错误时有明确提示。
- 不引入复杂配置系统，第一版可以先用 Python dataclass。

### 5.10 tests 和 tools

目标：为核心模块提供最小验证能力。

主要任务：

- `test_vr_parser.py`：验证数据包解析。
- `test_teleop_mapper.py`：验证 Grip 状态机和相对映射。
- `test_model_loader.py`：验证 URDF、关节名、frame 名。
- `simulate_target.py`：不用手柄，生成左右 target 轨迹。
- `send_fake_vr_packet.py`：发送模拟 UDP 包。

验收标准：

- 单元测试能独立运行。
- 模拟 target 可以驱动 Pink 和可视化。
- 假 VR 包可以验证 receiver 和 parser。
- 测试不依赖真机。

## 6. 阶段验收标准

### 6.1 M1 模型加载验收

- URDF 加载成功。
- 能解析 package 路径。
- 能输出缺失 mesh 列表。
- 可用 mesh 可以显示；缺失夹爪 mesh 不阻塞运动学验证。
- 左右臂 14 个关节存在。
- 腰、头、夹爪固定，不进入 IK 关节集合。
- 左右末端候选 frame 存在，并能显示用于确认 TCP。
- 初始 `q` 合法。

### 6.2 M2 Pink IK 验收

- 左右 `FrameTask` 可以创建。
- 静态 target 能收敛。
- 缓慢移动 target 能跟随。
- 关节不越界。
- 只有双臂 14 个关节变化。
- 结算无 NaN/Inf。
- `pink_solver.step()` 返回 `IkResult`，不返回 `JointCommand`。

### 6.3 M3 可视化闭环验收

- 模拟 target 驱动机器人运动。
- 当前末端 frame 和 target frame 同时显示。
- 候选 TCP frame 可视化。
- 机器人姿态连续变化。
- 运行 5 分钟不崩溃。

### 6.4 M4 手柄输入验收

- 能收到 VR UDP 数据。
- parser 能稳定输出 controller pose 和 buttons。
- 异常数据会被拒绝。
- 没有 VR 数据时主循环不阻塞。

### 6.5 M5 相对遥操作验收

- Grip 按下开始控制。
- Grip 松开保持。
- 再次按下从当前位置继续。
- A/X 回初始 target。
- 左右手可独立控制。
- 可视化中没有明显跳变。

### 6.6 M6 下发接口验收

- dry-run 输出关节命令。
- 关节顺序和名字固定。
- 默认不会连接真机。
- 可通过配置开启或关闭。

## 7. 总体验收标准

第一版整体完成的标准：

- 可以一条命令启动可视化 demo。
- JZ 机器人双臂运动学和当前可用 meshes 能正确显示。
- Pink 能驱动左右臂跟随 target。
- 手柄 Grip 相对控制逻辑可用。
- 所有核心模块边界清楚。
- 不接真机时也能完整验证主要链路。
- 下发模块有清晰接口，但默认禁用。
- 输出命令只覆盖双臂 14 关节，夹爪由独立链路控制。

## 8. 不在第一版做的事情

第一版暂不做：

- 真机执行。
- 自碰撞约束。
- 复杂 GUI。
- 多机器人 namespace。
- ROS 2 service 动态开关。
- 大型配置系统。
- 复杂日志系统。
- 自动标定 VR 坐标系。

这些内容等可视化遥操作闭环稳定后再评估。

## 9. 开发顺序建议

推荐执行顺序：

1. 实现 `model_loader.py`。
2. 实现 `visualizer.py`。
3. 实现 `pink_solver.py`。
4. 实现模拟 target demo。
5. 调整左右末端 frame 和 Pink 权重。
6. 实现 `vr_parser.py`。
7. 实现 `vr_receiver.py`。
8. 实现 `teleop_mapper.py`。
9. 将手柄接入 Pink demo。
10. 实现 dry-run `command_publisher.py`。

每一步都应该能独立运行或验证，不把多个未知问题压在一起调。
