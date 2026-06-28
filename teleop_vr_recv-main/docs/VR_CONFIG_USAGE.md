# VR坐标转换配置使用说明

## 配置文件说明

配置文件位置：`config/teleop_vr_recv.toml`

### 配置参数详解

```toml
[vr_transform.left_arm]
# 1. Home位姿（机器人启动时的确定位置）
home_position = [0.5, 0.3, 0.4]           # [x, y, z] 单位：米
home_orientation = [0.0, 0.0, 0.0, 1.0]   # [qx, qy, qz, qw] 四元数

# 2. 比例缩放因子
scale_factor = 0.8                        # VR移动1米 → 机器人移动0.8米

# 3. 坐标轴映射
axis_mapping = [0, 1, 2]                  # VR坐标系 → 机器人坐标系的轴映射
```

---

## 在代码中使用配置

### 示例：在teleop_node中初始化转换器

```cpp
#include "teleop_vr_recv/vr_coordinate_transformer.h"
#include "teleop_vr_recv/teleop_vr_config.h"

class TeleopVrRecvNode : public rclcpp::Node {
private:
    VrCoordinateTransformer left_arm_transformer_;
    VrCoordinateTransformer right_arm_transformer_;
};

// 在初始化函数中
void TeleopVrRecvNode::initializeVrTransformers() {
    if (!isConfigInitialized()) {
        LOG_WARNING("配置未初始化，使用默认VR转换参数");
        return;
    }

    const auto& config = getConfigConst();

    // ============ 初始化左臂转换器 ============
    // 1. 设置Home位姿
    left_arm_transformer_.setRobotHomePose(
        config.vr_transform.left_arm.home_position.data(),
        config.vr_transform.left_arm.home_orientation.data()
    );

    // 2. 设置比例因子
    left_arm_transformer_.setScaleFactor(
        config.vr_transform.left_arm.scale_factor
    );

    // 3. 设置坐标轴映射
    left_arm_transformer_.setAxisMapping(
        config.vr_transform.left_arm.axis_mapping
    );

    LOG_INFO("左臂VR转换器初始化完成:");
    LOG_INFO("  Home位置: [" << config.vr_transform.left_arm.home_position[0] << ", "
             << config.vr_transform.left_arm.home_position[1] << ", "
             << config.vr_transform.left_arm.home_position[2] << "]");
    LOG_INFO("  比例因子: " << config.vr_transform.left_arm.scale_factor);
    LOG_INFO("  坐标轴映射: [" << config.vr_transform.left_arm.axis_mapping[0] << ", "
             << config.vr_transform.left_arm.axis_mapping[1] << ", "
             << config.vr_transform.left_arm.axis_mapping[2] << "]");

    // ============ 初始化右臂转换器（同理）============
    right_arm_transformer_.setRobotHomePose(
        config.vr_transform.right_arm.home_position.data(),
        config.vr_transform.right_arm.home_orientation.data()
    );
    right_arm_transformer_.setScaleFactor(
        config.vr_transform.right_arm.scale_factor
    );
    right_arm_transformer_.setAxisMapping(
        config.vr_transform.right_arm.axis_mapping
    );

    LOG_INFO("右臂VR转换器初始化完成");
}

// 在数据回调中使用
void TeleopVrRecvNode::onUdpDataReceived(const VrDataPacket& packet) {
    // 检测左手柄Grip按钮（锁定VR起点）
    if (packet.left_input.grip_button && !left_arm_transformer_.isReferenceLocked()) {
        left_arm_transformer_.lockVrReference(
            packet.left_controller.position,
            packet.left_controller.rotation
        );
        LOG_INFO("左臂VR参考起点已锁定");
    }

    // 转换VR位姿到机器人目标位姿
    if (left_arm_transformer_.isReferenceLocked()) {
        auto left_target = left_arm_transformer_.transform(
            packet.left_controller.position,
            packet.left_controller.rotation
        );

        // 使用转换后的位姿
        // left_target.position[0,1,2] = x, y, z
        // left_target.orientation[0,1,2,3] = qx, qy, qz, qw

        // 发布或发送给逆解算法
        publishTargetPose(left_target);
    }
}
```

---

## 配置调试流程

### 步骤1：设置Home位姿

找到你的机器人Home位置（启动时的确定位置），修改配置：

```toml
[vr_transform.left_arm]
home_position = [0.5, 0.3, 0.4]    # 修改为你的机器人Home位置
home_orientation = [0.0, 0.0, 0.0, 1.0]  # 修改为Home姿态
```

**如何获取Home位姿？**
- 方法1：让机器人移动到Home位置，读取末端位姿
- 方法2：从URDF或机器人文档中查找
- 方法3：使用你的机器人控制软件查看当前位姿

### 步骤2：调整比例因子

启动节点，测试VR手柄移动：

```toml
scale_factor = 0.8    # 先用默认值测试
```

- 如果机器人移动太快 → 减小 scale_factor（比如 0.5）
- 如果机器人移动太慢 → 增大 scale_factor（比如 1.0）

### 步骤3：修正坐标轴映射

先用默认值 `[0, 1, 2]` 测试：

1. 锁定VR起点（按Grip）
2. VR手柄向**右**移动 → 观察机器人移动方向
3. VR手柄向**上**移动 → 观察机器人移动方向
4. VR手柄向**前**移动 → 观察机器人移动方向

**常见坐标系映射：**

| VR动作 | 期望机器人动作 | 实际机器人动作 | 建议映射 |
|--------|--------------|--------------|---------|
| 向右 | 向右 | 向右 | `[0, 1, 2]` (默认) |
| 向右 | 向右 | 向左 | `[-0, 1, 2]` (X轴反向) |
| 向上 | 向上 | 向前 | `[0, 2, 1]` (Y和Z互换) |
| 向前 | 向前 | 向后 | `[0, 1, -2]` (Z轴反向) |

---

## 常见配置示例

### 示例1：标准双臂配置

```toml
[vr_transform.left_arm]
home_position = [0.5, 0.3, 0.4]
home_orientation = [0.0, 0.0, 0.0, 1.0]
scale_factor = 0.8
axis_mapping = [0, 1, 2]

[vr_transform.right_arm]
home_position = [0.5, -0.3, 0.4]    # 右臂在左臂旁边（Y值为负）
home_orientation = [0.0, 0.0, 0.0, 1.0]
scale_factor = 0.8
axis_mapping = [0, 1, 2]
```

### 示例2：VR的Y轴向上 → 机器人的Z轴向上

```toml
[vr_transform.left_arm]
home_position = [0.5, 0.3, 0.4]
home_orientation = [0.0, 0.0, 0.0, 1.0]
scale_factor = 0.8
axis_mapping = [0, 2, 1]    # VR的Y→机器人Z, VR的Z→机器人Y
```

### 示例3：更精细的控制（小比例）

```toml
[vr_transform.left_arm]
home_position = [0.5, 0.3, 0.4]
home_orientation = [0.0, 0.0, 0.0, 1.0]
scale_factor = 0.5    # VR移动1米，机器人只移动0.5米（更精细）
axis_mapping = [0, 1, 2]
```

---

## 配置验证

启动节点后，检查日志输出：

```
[INFO] 左臂VR转换器初始化完成:
[INFO]   Home位置: [0.5, 0.3, 0.4]
[INFO]   比例因子: 0.8
[INFO]   坐标轴映射: [0, 1, 2]
[INFO] 右臂VR转换器初始化完成
```

---

## 故障排除

### Q: 修改配置后不生效？
A: 重新编译并重启节点：
```bash
cd ~/workspace/teleop_ws
colcon build --packages-select teleop_vr_recv
source install/setup.bash
ros2 launch teleop_vr_recv teleop_vr_recv.launch.py
```

### Q: 配置文件在哪？
A: 两个位置（优先使用第二个）：
1. 源码：`src/teleop_vr_recv/config/teleop_vr_recv.toml`
2. 安装后：`install/teleop_vr_recv/share/teleop_vr_recv/config/teleop_vr_recv.toml`

修改后需要重新编译安装。

### Q: Home姿态的四元数怎么填？
A: 如果不知道，先用单位四元数 `[0.0, 0.0, 0.0, 1.0]`（表示无旋转），后续可以调整。

### Q: 左右臂的配置必须对称吗？
A: 不必须。每个臂可以有独立的Home位姿、比例因子和坐标轴映射。
