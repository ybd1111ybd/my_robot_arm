# teleop_vr_py 纯 Python 复刻说明

这个目录下的 `teleop_vr_py/` 是从 `teleop_vr_recv-main/src` 中抽出来的
VR 手柄相关逻辑 Python 版，不依赖 ROS 2，不发布 topic，不使用 service。

## 文件对应关系

```text
teleop_vr_recv-main/src/udp_receiver.cpp
  -> teleop_vr_py/udp_receiver.py

teleop_vr_recv-main/src/vr_data_parser.cpp
  -> teleop_vr_py/vr_data_parser.py

teleop_vr_recv-main/include/teleop_vr_recv/data_smoother.h
teleop_vr_recv-main/src/vr_smoothing_pipeline.cpp
  -> teleop_vr_py/data_smoother.py
  -> teleop_vr_py/smoothing_pipeline.py

teleop_vr_recv-main/src/workspace_limiter.cpp
  -> teleop_vr_py/workspace_limiter.py

teleop_vr_recv-main/src/vr_coordinate_transformer.cpp
  -> teleop_vr_py/coordinate_transformer.py

teleop_vr_recv-main/src/vr_button_handler.cpp
  -> teleop_vr_py/button_handler.py
```

## 已复刻内容

- UDP 监听和超时接收。
- VR packet 帧头、长度、数值范围校验。
- torque、16 个角度/夹爪字段解析。
- 左右手柄 pose、头显 pose 解析。
- 左右手柄 input 解析。
- 允许真实设备发来的 `253 bytes / length=62` 扩展包；解析前面已知字段，忽略尾部扩展。
- 位置滤波：none、moving_avg、exp_moving_avg、kalman。
- 四元数 SLERP 平滑和符号翻转处理。
- `axis_mapping = [-2, 3, 1]` 坐标映射。
- 左右手 tool orientation offset：左手 Z +90 度，右手 Z -90 度。
- Grip 按下锁定 VR 参考，Grip 松开保持上一帧 target。
- X/A primary button 回 home target。
- 球形 workspace limit：clamp / saturate。

## 不包含内容

- ROS 2 node。
- ROS 2 topic 发布。
- ROS 2 service 开关。
- FK topic 订阅。
- 真机下发。

纯 Python 版里的当前末端 pose 暂时由内部 target 状态代替。后面接
`light_teleop` / Pink 时，可以把 Pink 当前末端 pose 传给 button handler 或
mapper，替代这里的 demo 内部状态。

## 自测

```bash
cd /home/luzhuang/cqy/my_robot/test
python3 test_teleop_vr_py.py
```

期望输出：

```text
test_teleop_vr_py: OK
```

## 监听真实手柄并打印 target

```bash
cd /home/luzhuang/cqy/my_robot/test
python3 vr_recv_demo.py \
    --host 10.1.42.3 \
    --port 8080 \
    --print-every 10
```

如果想启用工作空间限制：

```bash
python3 vr_recv_demo.py \
    --host 10.1.42.3 \
    --port 8080 \
    --print-every 10 \
    --workspace-limit \
    --workspace-radius 1.0
```

操作语义：

```text
Grip 按住：锁定当前手柄 pose，从当前 target 开始做相对控制
Grip 松开：停止跟随，保持上一帧 target
左 X / 右 A：回到配置里的 home target
```

打印的 `left_target/right_target` 才是机器人 base_link 语义下的末端目标；
`left_controller/right_controller` 只是 VR 手柄原始 pose，不能直接当机器人
target_pose 使用。
