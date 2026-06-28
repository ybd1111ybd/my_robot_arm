# teleop_vr_recv

VR遥操作UDP数据接收器 - C++实现

## 概述

`teleop_vr_recv` 是一个ROS2 C++包，用于接收VR手柄通过UDP发送的关节角度数据，并将其转换为ROS2消息发布到SmoothMotionEngine，实现双臂机器人的远程遥操作控制。

## 功能特点

- ✅ UDP数据接收 (默认端口8080)
- ✅ UDP接收开关控制 (默认关闭,通过服务动态控制)
- ✅ 数据包解析和验证 (帧头验证、数据长度检查)
- ✅ 角度转换 (度 → 弧度)
- ✅ 双臂独立控制 (左臂7关节 + 右臂7关节)
- ✅ 实时数据显示 (关节角度、FPS统计)
- ✅ ROS2标准接口
- ✅ Launch文件支持
- ✅ 配置文件支持

## 系统要求

- **ROS2**: Humble或更高版本
- **操作系统**: Ubuntu 22.04
- **编译器**: GCC 11+ (支持C++17)

## 依赖项

### ROS2包依赖
- `rclcpp` - ROS2 C++客户端库
- `std_msgs` - 标准消息类型
- `std_srvs` - 标准服务类型
- `sensor_msgs` - 传感器消息类型

### 系统依赖
- C++标准库 (C++17)
- POSIX socket库

## 安装与编译

### 1. 克隆代码到ROS2工作空间

```bash
cd ~/workspace/teleop_ws/src
# 代码已在 teleop_vr_recv 目录中
```

### 2. 安装依赖

```bash
cd ~/workspace/teleop_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 编译

```bash
cd ~/workspace/teleop_ws
colcon build --packages-select teleop_vr_recv --symlink-install
```

### 4. 加载环境

```bash
source ~/workspace/teleop_ws/install/setup.bash
```

## 使用方法

### 方式1: 直接运行节点

```bash
# 使用默认参数 (端口8080, 监听0.0.0.0)
ros2 run teleop_vr_recv teleop_vr_recv_node

# 指定端口
ros2 run teleop_vr_recv teleop_vr_recv_node --ros-args -p port:=9000

# 指定主机和端口
ros2 run teleop_vr_recv teleop_vr_recv_node --ros-args -p host:=127.0.0.1 -p port:=8080
```

### 方式2: 使用Launch文件

```bash
# 使用默认参数 (UDP接收默认关闭)
ros2 launch teleop_vr_recv teleop_vr_recv.launch.py

# 启动时开启UDP接收
ros2 launch teleop_vr_recv teleop_vr_recv.launch.py enable_udp_receive:=true

# 自定义参数
ros2 launch teleop_vr_recv teleop_vr_recv.launch.py port:=9000 enable_udp_receive:=true
```

### 方式3: 使用配置文件

配置文件位于 `config/teleop_vr_recv.toml`:

```toml
[udp]
host = "0.0.0.0"
port = 8080
enable_udp_receive = false  # 默认关闭
```

## UDP接收控制

UDP接收功能默认关闭,可以通过以下方式控制:

### 1. 启动时配置

```bash
# 方式1: 通过launch参数
ros2 launch teleop_vr_recv teleop_vr_recv.launch.py enable_udp_receive:=true

# 方式2: 通过ROS参数
ros2 run teleop_vr_recv teleop_vr_recv_node --ros-args -p enable_udp_receive:=true
```

### 2. 运行时动态控制

使用ROS服务 `~/enable_udp_receive` (类型: `std_srvs/srv/SetBool`):

```bash
# 启用UDP接收
ros2 service call /robot1/enable_udp_receive std_srvs/srv/SetBool "{data: true}"

# 禁用UDP接收
ros2 service call /enable_udp_receive std_srvs/srv/SetBool "{data: false}"
# 查看当前服务列表
ros2 service list | grep enable_udp_receive
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `host` | string | `"0.0.0.0"` | UDP监听地址 |
| `port` | int | `8080` | UDP监听端口 |
| `enable_udp_receive` | bool | `false` | UDP接收开关 (默认关闭) |

## 服务说明

### 提供的服务

| 服务名 | 服务类型 | 说明 |
|--------|----------|------|
| `~/enable_udp_receive` | `std_srvs/srv/SetBool` | 启用/禁用UDP接收 |

**服务参数**:
- `data`: `true` = 启用UDP接收, `false` = 禁用UDP接收
- `success`: 操作是否成功
- `message`: 结果消息

## 话题说明

### 发布话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/telecon/arm_left/joint_commands_input` | `std_msgs/msg/Float64MultiArray` | 左臂7个关节角度命令 (弧度) |
| `/telecon/arm_right/joint_commands_input` | `std_msgs/msg/Float64MultiArray` | 右臂7个关节角度命令 (弧度) |

## UDP数据包格式

### 数据包结构

```
+---------+---------+--------+---------+-------------------+
| 帧头1   | 帧头2   | 长度   | 扭矩    | 角度数据 (16个)   |
+---------+---------+--------+---------+-------------------+
| 0xAA    | 0xBB    | 0x10   | 2字节   | 16 x 4字节        |
| (1字节) | (1字节) | (1字节)| (int16) | (float, 小端序)   |
+---------+---------+--------+---------+-------------------+
```

**总长度**: 至少85字节

### 角度数据布局

- **索引 0-6**: 左臂关节J1-J7 (度)
- **索引 7**: 左臂夹爪值
- **索引 8-14**: 右臂关节J1-J7 (度)
- **索引 15**: 右臂夹爪值

### 数据格式
- **字节序**: 小端序 (Little-Endian)
- **角度单位**: 度 (°)
- **输出单位**: 弧度 (rad)

## 示例输出

```
=================================================
Publish Mode: SMOOTH (Float64MultiArray)
Topics:
  - telecon/arm_left/joint_commands_input
  - telecon/arm_right/joint_commands_input
=================================================
UDP receiver started on 0.0.0.0:8080
Waiting for VR data...
左手: J1:  45.0 J2: -30.0 J3:  60.0 J4:  15.0 J5: -45.0 J6:  30.0 J7:   0.0 夹爪:  50.0 | 右手: J1: -45.0 J2:  30.0 J3: -60.0 J4: -15.0 J5:  45.0 J6: -30.0 J7:   0.0 夹爪:  50.0 | FPS: 50.2
```

## 架构设计

```
┌─────────────┐
│  VR 设备    │
│             │
└──────┬──────┘
       │ UDP (端口8080)
       │ 数据包 (85字节)
       ▼
┌─────────────────────────┐
│  teleop_vr_recv_node    │
│                         │
│  ┌──────────────────┐   │
│  │  UDP接收器       │   │
│  │  - Socket接收    │   │
│  │  - 非阻塞模式    │   │
│  └────────┬─────────┘   │
│           │             │
│  ┌────────▼─────────┐   │
│  │  数据解析器      │   │
│  │  - 帧头验证      │   │
│  │  - 格式解析      │   │
│  │  - 度→弧度转换   │   │
│  └────────┬─────────┘   │
│           │             │
│  ┌────────▼─────────┐   │
│  │  ROS2发布器      │   │
│  │  - Float64Array  │   │
│  │  - QoS: depth=10 │   │
│  └──────────────────┘   │
└───────────┬─────────────┘
            │
            ▼
    ┌───────────────────┐
    │ SmoothMotionEngine│
    └───────────────────┘
```

## 代码结构

```
teleop_vr_recv/
├── CMakeLists.txt              # CMake配置文件
├── package.xml                 # ROS2包配置
├── README.md                   # 本文件
├── .gitignore                  # Git忽略文件
├── config/
│   └── teleop_vr_recv.toml    # 配置文件
├── include/teleop_vr_recv/
│   ├── udp_receiver.h        # UDP接收器类
│   ├── vr_data_parser.h      # VR数据解析器
│   └── teleop_node.h         # ROS2节点类
├── src/
│   ├── udp_receiver.cpp        # UDP接收实现
│   ├── vr_data_parser.cpp      # 数据解析实现
│   ├── teleop_node.cpp         # ROS2节点实现
│   └── teleop_node_main.cpp    # 主程序入口
└── launch/
    └── teleop_vr_recv.launch.py # 启动文件
```

## 故障排除

### 1. 编译错误

**问题**: 找不到头文件
```bash
fatal error: rclcpp/rclcpp.hpp: No such file or directory
```

**解决方案**:
```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 运行时错误

**问题**: 端口被占用
```bash
Failed to bind to 0.0.0.0:8080 - Address already in use
```

**解决方案**:
```bash
# 检查端口占用
sudo lsof -i :8080

# 使用其他端口
ros2 run teleop_vr_recv teleop_vr_recv_node --ros-args -p port:=9000
```

### 3. 无数据接收

**检查项**:
- 确认VR设备正在发送数据
- 确认网络连接正常
- 确认端口号匹配
- 使用 `tcpdump` 或 `wireshark` 抓包验证

```bash
# 监听UDP端口
sudo tcpdump -i any -n udp port 8080
```

## 性能指标

- **接收频率**: 最高可达100Hz (取决于VR设备发送频率)
- **延迟**: < 10ms (网络正常情况下)
- **CPU占用**: < 5% (单核)
- **内存占用**: < 50MB

## 开发与贡献

### 编译开发版本

```bash
colcon build --packages-select teleop_vr_recv --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### 代码风格

本项目遵循ROS2 C++代码风格指南。

## 许可证

MIT License

## 更新日志

### v1.1.0 (2026-01-12)
- ✅ 添加UDP接收开关控制功能
- ✅ 添加ROS服务 `~/enable_udp_receive`
- ✅ 添加配置文件支持 (TOML格式)
- ✅ UDP接收默认关闭,可通过服务动态控制

### v1.0.0 (2026-01-10)
- ✅ 初始版本发布
- ✅ UDP接收功能
- ✅ 数据解析和转换
- ✅ ROS2 Float64MultiArray发布
- ✅ Launch文件支持

## 相关项目

- [SmoothMotionEngine](../SmoothMotionEngine) - 平滑运动引擎
- [armcontrol](../armcontrol) - 机械臂控制器

## 参考资料

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [POSIX Socket Programming](https://man7.org/linux/man-pages/man7/socket.7.html)
