# Ridgeback 底盘 + Franka 机械臂结合使用说明

## 📋 概述

在这个项目中，`ridgeback_franka.usd` 是一个**组合机器人模型**，包含：
- **Ridgeback 移动底盘**：提供移动能力
- **Franka 机械臂**：提供操作能力

本文档说明两者如何结合使用。

---

## 🔍 关键发现

### 1. 机器人模型结构

`ridgeback_franka.usd` 文件包含完整的组合机器人：
```
/World/Ridgebase/
├── base_link/              # 移动底盘
│   ├── Camera_01          # 底盘相机 1
│   └── Camera             # 底盘相机 2
└── panda_link0/           # Franka 机械臂基座
    ├── panda_joint1-7     # 7 个机械臂关节
    ├── panda_hand/        # 末端执行器
    │   ├── Camera         # 末端相机
    │   └── endeffector    # 末端点
    └── panda_finger_joint1-2  # 2 个手指关节
```

### 2. 在代码中的结合使用

**位置**：`demo_manual_pick.py` - 这是唯一同时使用两者功能的文件

---

## 🎯 详细代码分析

### 初始化部分（第 150-172 行）

```python
# 1. 导航控制器（控制移动底盘）
nav_controller = RidgebaseController(
    robot_articulation=robot,
    max_linear_speed=0.02,
    max_angular_speed=1.5,
    position_threshold=0.05,
    angle_threshold=0.02
)

# 2. Franka 机械臂运动规划控制器
cspace_controller = RMPFlowController(
    name="rmp_flow",
    robot_articulation=robot  # 同一个机器人对象
)

# 3. 拾取任务控制器（使用机械臂控制器）
pick_controller = PickController(
    name="pick_controller",
    cspace_controller=cspace_controller
)

# 4. 定义机械臂关节子集（只控制机械臂，不控制底盘）
franka_subset = ArticulationSubset(
    robot,
    ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 
     'panda_joint5', 'panda_joint6', 'panda_joint7', 
     'panda_finger_joint1', 'panda_finger_joint2']  # 9 个关节
)
```

**关键点**：
- ✅ 使用**同一个机器人对象** (`robot`) 创建两个控制器
- ✅ `RidgebaseController` 只控制移动底盘（3 个虚拟关节）
- ✅ `RMPFlowController` 控制 Franka 机械臂（用于运动规划）
- ✅ `ArticulationSubset` 用于单独控制机械臂关节

---

### 相机配置（第 75-96 行）

代码中配置了 3 个相机，分别位于不同位置：

```python
camera_configs = [
    {
        "prim_path": "/World/Ridgebase/base_link/Camera_01",  # 底盘相机 1
        "name": "cam_01",
        "resolution": (640, 640),
    },
    {
        "prim_path": "/World/Ridgebase/base_link/Camera",      # 底盘相机 2
        "name": "cam_02",
        "resolution": (640, 640),
    },
    {
        "prim_path": "/World/Ridgebase/panda_hand/Camera",     # 机械臂末端相机
        "name": "hand_cam",
        "resolution": (640, 640),
    }
]
```

---

### 工作流程：导航 → 拾取

代码实现了一个两阶段任务：

#### 阶段 1：导航（第 246-259 行）

```python
if not navigation_done:
    # 只使用移动底盘导航
    position, orientation = robot.get_world_pose()
    euler_angles = quat_to_euler_angles(orientation)
    current_pose = np.array([position[0], position[1], euler_angles[2]])
    
    action, done = nav_controller.get_action(current_pose)
    if action is not None:
        robot.apply_action(action)  # 控制移动底盘
    
    if done:
        print("Navigation completed, starting pick task!")
        navigation_done = True
        pick_started = True
```

**说明**：
- 只使用 `nav_controller` 控制移动
- 机械臂保持初始状态不动

#### 阶段 2：拾取（第 261-289 行）

```python
elif pick_started and not pick_success:
    # 关键步骤 1：获取机械臂基座位置（相对于移动底盘）
    pose = ObjectUtils.get_instance().get_object_xform_position(
        object_path="/World/Ridgebase/panda_link0"  # Franka 基座
    )
    quat = ObjectUtils.get_instance().get_transform_quat(
        object_path="/World/Ridgebase/panda_link0", 
        w_first=True
    )
    
    # 关键步骤 2：更新 RMPFlow 控制器的基座位置
    # 因为底盘移动了，机械臂基座位置也改变了
    cspace_controller.rmp_flow.set_robot_base_pose(pose, quat)
    pick_controller.set_robot_position(pose)
    
    # 关键步骤 3：获取机械臂关节状态
    joint_positions = franka_subset.get_joint_positions()
    
    # 关键步骤 4：获取末端执行器位置
    end_effector_position = ObjectUtils.get_instance().get_object_xform_position(
        object_path="/World/Ridgebase/endeffector"
    )
    
    # 关键步骤 5：计算拾取动作
    action = pick_controller.forward(
        picking_position=pick_position,
        current_joint_positions=joint_positions,
        ...
    )
    
    # 关键步骤 6：只对机械臂关节应用动作
    if action is not None:
        # 只控制机械臂，不移动底盘
        franka_subset.apply_action(
            action.joint_positions, 
            action.joint_velocities
        )
```

**关键结合点**：

1. **基座位置更新**（第 266 行）：
   ```python
   cspace_controller.rmp_flow.set_robot_base_pose(pose, quat)
   ```
   - 因为底盘移动了，机械臂基座的位置也改变了
   - 需要更新运动规划器的基座位置，确保运动规划正确

2. **关节子集控制**（第 289 行）：
   ```python
   franka_subset.apply_action(action.joint_positions, action.joint_velocities)
   ```
   - 只对机械臂的 9 个关节应用动作
   - **不会移动底盘**，保持导航后的位置

---

## 🔧 关节分离控制

### RidgebaseController 控制的关节

```python
# controllers/robot_controllers/ridgebase/ridgebase_controller.py:40-43
self._joints_subset = ArticulationSubset(
    robot_articulation,
    ["dummy_base_prismatic_x_joint",   # X 方向移动
     "dummy_base_prismatic_y_joint",   # Y 方向移动
     "dummy_base_revolute_z_joint"]    # Z 轴旋转
)
```

### Franka 机械臂的关节

```python
# demo_manual_pick.py:166-169
franka_subset = ArticulationSubset(
    robot,
    ['panda_joint1',    # 肩部旋转
     'panda_joint2',    # 肩部俯仰
     'panda_joint3',    # 肘部
     'panda_joint4',    # 前臂旋转
     'panda_joint5',    # 腕部俯仰
     'panda_joint6',    # 腕部旋转
     'panda_joint7',    # 腕部末端旋转
     'panda_finger_joint1',  # 手指 1
     'panda_finger_joint2']  # 手指 2
)
```

---

## 📊 完整工作流程

```
【阶段 1：导航】
1. 加载组合机器人 (ridgeback_franka.usd)
   ↓
2. 创建导航控制器 (RidgebaseController)
   ↓
3. 创建机械臂控制器 (RMPFlowController) - 但不使用
   ↓
4. 机器人移动到目标位置
   ↓
【阶段 2：拾取】
5. 获取当前机械臂基座位置（因为底盘移动了）
   ↓
6. 更新 RMPFlow 控制器的基座位置
   ↓
7. 使用机械臂控制器规划拾取动作
   ↓
8. 只对机械臂关节应用动作（底盘保持不动）
   ↓
9. 完成拾取任务
```

---

## 💡 关键要点

### 1. 为什么需要更新基座位置？

当移动底盘移动到新位置后：
- 机械臂的基座 (`panda_link0`) 也移动了
- RMPFlow 控制器需要知道基座的新位置
- 才能正确计算机械臂的运动规划

```python
# 获取当前基座位置
pose = ObjectUtils.get_instance().get_object_xform_position(
    object_path="/World/Ridgebase/panda_link0"
)

# 更新控制器
cspace_controller.rmp_flow.set_robot_base_pose(pose, quat)
```

### 2. 为什么使用 ArticulationSubset？

- **独立控制**：可以只控制机械臂，不影响底盘
- **灵活性**：两个控制器可以同时工作，但控制不同的关节
- **避免冲突**：导航时不动机械臂，操作时不动底盘

### 3. 相机位置

代码中使用了 3 个相机：
- 2 个在底盘上（观察前方）
- 1 个在机械臂末端（观察操作目标）

---

## 🎯 对比：demo_ridgebase_astar_nav.py

在 `demo_ridgebase_astar_nav.py` 中：

**只使用移动底盘**：
- ✅ 只有 `RidgebaseController`
- ❌ 没有机械臂控制器
- ❌ 没有 `franka_subset`
- 用途：纯导航演示

---

## 📝 总结

**是否有结合部分？**

✅ **是的！** 在 `demo_manual_pick.py` 中有完整的结合使用：

1. **同一个机器人对象**：`ridgeback_franka.usd` 包含两个部分
2. **两个控制器**：
   - `RidgebaseController` - 控制移动
   - `RMPFlowController` - 控制机械臂
3. **关节分离**：
   - 导航时只控制 3 个底盘关节
   - 操作时只控制 9 个机械臂关节
4. **基座位置同步**：移动后需要更新机械臂基座位置

这是典型的**移动操作机器人**（Mobile Manipulator）架构！

