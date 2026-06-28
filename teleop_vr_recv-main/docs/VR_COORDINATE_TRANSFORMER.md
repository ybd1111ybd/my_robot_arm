# VR坐标转换器使用说明

## 功能说明

VrCoordinateTransformer 用于将Quest3手柄的位姿数据转换为机器人末端执行器的目标位姿。

## 核心特性

1. **相对运动映射**：无需标定，只需按下按钮锁定起点
2. **坐标轴映射**：自动处理VR坐标系和机器人坐标系的差异
3. **比例缩放**：将VR空间映射到机器人工作空间
4. **双臂支持**：可创建两个独立的转换器实例

---

## 快速开始

### 1. 基本使用流程

```cpp
#include "teleop_vr_recv/vr_coordinate_transformer.h"

// 创建左臂和右臂的转换器
teleop_vr::VrCoordinateTransformer left_arm_transformer(0.8);   // 比例因子0.8
teleop_vr::VrCoordinateTransformer right_arm_transformer(0.8);

// 步骤1: 设置机器人Home位姿（启动时的确定位置）
double left_home_pos[3] = {0.5, 0.3, 0.4};       // 左臂Home位置 [x,y,z]
double left_home_ori[4] = {0.0, 0.0, 0.0, 1.0};  // 左臂Home姿态 [qx,qy,qz,qw]
left_arm_transformer.setRobotHomePose(left_home_pos, left_home_ori);

double right_home_pos[3] = {0.5, -0.3, 0.4};
double right_home_ori[4] = {0.0, 0.0, 0.0, 1.0};
right_arm_transformer.setRobotHomePose(right_home_pos, right_home_ori);

// 步骤2: 当用户按下手柄Grip按钮时，锁定VR参考起点
float left_vr_pos[3] = {packet.left_controller.position[0],
                        packet.left_controller.position[1],
                        packet.left_controller.position[2]};
float left_vr_ori[4] = {packet.left_controller.rotation[0],
                        packet.left_controller.rotation[1],
                        packet.left_controller.rotation[2],
                        packet.left_controller.rotation[3]};
left_arm_transformer.lockVrReference(left_vr_pos, left_vr_ori);

// 步骤3: 实时转换手柄位姿到机器人目标位姿
auto left_target = left_arm_transformer.transform(left_vr_pos, left_vr_ori);

// 步骤4: 使用转换后的位姿（发送给逆解算法）
std::cout << "目标位置: [" << left_target.position[0] << ", "
          << left_target.position[1] << ", " << left_target.position[2] << "]" << std::endl;
std::cout << "目标姿态: [" << left_target.orientation[0] << ", "
          << left_target.orientation[1] << ", " << left_target.orientation[2] << ", "
          << left_target.orientation[3] << "]" << std::endl;
```

---

## 配置参数

### 比例因子 (Scale Factor)

控制VR空间到机器人空间的缩放比例：

```cpp
// VR手柄移动1米 → 机器人末端移动0.5米
transformer.setScaleFactor(0.5);
```

**建议值**：0.5 - 1.0

### 坐标轴映射 (Axis Mapping)

用于对齐VR坐标系和机器人坐标系：

```cpp
// 默认：不做映射（VR的XYZ → 机器人的XYZ）
transformer.setAxisMapping({1, 2, 3});

// VR的Y和Z互换（常见：VR的Y向上 → 机器人的Z向上）
transformer.setAxisMapping({1, 3, 2});

// VR的Z轴反向
transformer.setAxisMapping({1, 2, -3});

// VR的X轴反向（现在支持了！）
transformer.setAxisMapping({-1, 2, 3});
```

**映射规则说明（1-based索引）**：
- 使用1-based索引：`1`=X轴, `2`=Y轴, `3`=Z轴
- 负数表示反向：`-1`=-X轴, `-2`=-Y轴, `-3`=-Z轴
- `{1, 2, 3}` 表示 VR的X→机器人X, VR的Y→机器人Y, VR的Z→机器人Z
- `{1, 3, 2}` 表示 VR的X→机器人X, VR的Y→机器人Z, VR的Z→机器人Y
- `{-3, -1, 2}` 表示 VR的X→机器人-Z, VR的Y→机器人-X, VR的Z→机器人Y

---

## 常见坐标系问题

### Quest3 (OpenXR) 坐标系
```
Y↑ (向上)
|
+---→ X (向右)
/
Z (向后，朝向你)
```

### 常见机器人坐标系
```
Z↑ (向上)
|
+---→ X (向前)
/
Y (向左)
```

### 对应的映射设置

如果你的机器人是上述常见坐标系，建议使用：
```cpp
// Quest3 → 机器人坐标系
// VR的X(右) → 机器人的-Y(右)
// VR的Y(上) → 机器人的Z(上)
// VR的Z(后) → 机器人的-X(前)
transformer.setAxisMapping({1, 2, 0});  // 先测试这个
```

**调试方法**：
1. 先用默认的 `{1, 2, 3}` 启动
2. 测试VR手柄向右/上/前移动，看机器人往哪移动
3. 根据实际情况调整映射

---

## 完整示例：集成到 teleop_node

### 1. 在teleop_node.h中添加成员变量

```cpp
#include "teleop_vr_recv/vr_coordinate_transformer.h"

class TeleopVrRecvNode : public rclcpp::Node {
private:
    // VR坐标转换器
    VrCoordinateTransformer left_arm_transformer_;
    VrCoordinateTransformer right_arm_transformer_;

    // 发布机器人目标位姿
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_arm_target_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_arm_target_pose_pub_;
};
```

### 2. 在teleop_node.cpp中初始化

```cpp
void TeleopVrRecvNode::initializeParameters() {
    // ... 现有代码 ...

    // 从配置文件读取VR转换参数
    if (isConfigInitialized()) {
        const auto& config = getConfigConst();

        // 设置左臂Home位姿
        double left_home_pos[3] = {0.5, 0.3, 0.4};  // 从配置读取
        double left_home_ori[4] = {0.0, 0.0, 0.0, 1.0};
        left_arm_transformer_.setRobotHomePose(left_home_pos, left_home_ori);
        left_arm_transformer_.setScaleFactor(0.8);
        left_arm_transformer_.setAxisMapping({1, 2, 3});

        // 设置右臂Home位姿
        double right_home_pos[3] = {0.5, -0.3, 0.4};
        double right_home_ori[4] = {0.0, 0.0, 0.0, 1.0};
        right_arm_transformer_.setRobotHomePose(right_home_pos, right_home_ori);
        right_arm_transformer_.setScaleFactor(0.8);
        right_arm_transformer_.setAxisMapping({1, 2, 3});
    }
}

void TeleopVrRecvNode::initializePublishers() {
    // ... 现有代码 ...

    // 创建机器人目标位姿发布者
    left_arm_target_pose_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "left_arm/target_pose", qos);
    right_arm_target_pose_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "right_arm/target_pose", qos);
}

void TeleopVrRecvNode::publishControllerInputs(const VrDataPacket& packet) {
    // ... 现有代码 ...

    // 检测左手柄Grip按钮（用于锁定VR参考起点）
    if (packet.left_input.grip_button && !left_arm_transformer_.isReferenceLocked()) {
        left_arm_transformer_.lockVrReference(
            packet.left_controller.position,
            packet.left_controller.rotation
        );
        LOG_INFO("左臂VR参考起点已锁定");
    }

    // 检测右手柄Grip按钮
    if (packet.right_input.grip_button && !right_arm_transformer_.isReferenceLocked()) {
        right_arm_transformer_.lockVrReference(
            packet.right_controller.position,
            packet.right_controller.rotation
        );
        LOG_INFO("右臂VR参考起点已锁定");
    }

    // 转换并发布目标位姿
    if (left_arm_transformer_.isReferenceLocked()) {
        auto left_target = left_arm_transformer_.transform(
            packet.left_controller.position,
            packet.left_controller.rotation
        );

        // 发布为 [x, y, z, qx, qy, qz, qw]
        auto left_msg = std_msgs::msg::Float64MultiArray();
        left_msg.data.resize(7);
        left_msg.data[0] = left_target.position[0];
        left_msg.data[1] = left_target.position[1];
        left_msg.data[2] = left_target.position[2];
        left_msg.data[3] = left_target.orientation[0];
        left_msg.data[4] = left_target.orientation[1];
        left_msg.data[5] = left_target.orientation[2];
        left_msg.data[6] = left_target.orientation[3];
        left_arm_target_pose_pub_->publish(left_msg);
    }

    // 右臂同理...
}
```

---

## API 参考

### 构造函数
```cpp
VrCoordinateTransformer(double scale_factor = 1.0)
```

### 主要方法

| 方法 | 说明 |
|------|------|
| `setRobotHomePose(pos, ori)` | 设置机器人Home位姿 |
| `lockVrReference(vr_pos, vr_ori)` | 锁定VR参考起点 |
| `transform(vr_pos, vr_ori)` | 转换VR位姿到机器人目标位姿 |
| `setScaleFactor(scale)` | 设置比例因子 |
| `setAxisMapping(mapping)` | 设置坐标轴映射 |
| `resetVrReference()` | 重置VR参考起点 |
| `isReferenceLocked()` | 检查是否已锁定参考起点 |

---

## 调试建议

1. **先测试位置映射**：
   - 锁定起点后，VR手柄向右移动10cm
   - 检查机器人目标位置是否正确变化

2. **调整比例因子**：
   - 如果机器人移动太快/太慢，调整 `scale_factor`

3. **修正坐标轴**：
   - 如果方向不对，调整 `axis_mapping`

4. **日志输出**：
   ```cpp
   LOG_INFO("VR位置: [" << vr_pos[0] << ", " << vr_pos[1] << ", " << vr_pos[2] << "]");
   LOG_INFO("机器人目标位置: [" << target.position[0] << ", "
            << target.position[1] << ", " << target.position[2] << "]");
   ```

---

## 常见问题

### Q: 为什么必须先按Grip按钮？
A: 因为使用相对运动映射，需要先记录VR起点作为参考。按Grip按钮就是"这是我的起点"。

### Q: 每次启动都要按Grip吗？
A: 是的，这样你可以随意摆放Quest3，不需要固定位置。

### Q: 坐标轴映射太复杂了怎么办？
A: 先用默认值测试，记录VR手柄移动方向和机器人实际移动方向，再调整。

### Q: 能同时控制两个臂吗？
A: 可以，创建两个转换器实例，分别绑定左右手柄。
