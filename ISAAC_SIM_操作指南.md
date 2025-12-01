# Isaac Sim 导航演示操作指南

## 📋 目录
1. [快速开始](#快速开始)
2. [运行程序](#运行程序)
3. [设置起点和终点](#设置起点和终点)
4. [Isaac Sim GUI 操作](#isaac-sim-gui-操作)
5. [观察机器人移动](#观察机器人移动)
6. [调试和参数调整](#调试和参数调整)
7. [常见问题](#常见问题)

---

## 🚀 快速开始

### 1. 环境准备

确保你已经：
- ✅ 安装了 Isaac Sim 4.2
- ✅ 激活了 conda 环境：`conda activate labutopia`
- ✅ 安装了所有依赖包

### 2. 运行导航演示

```bash
cd /home/fengbohan/fengbohan/LabUtopia
python demo_ridgebase_astar_nav.py
```

---

## 🎮 运行程序

### 方式一：使用默认配置（随机起点终点）

1. **打开文件** `demo_ridgebase_astar_nav.py`
2. **确认配置**（第 105 行）：
   ```python
   USE_MANUAL_POINTS = False  # 使用随机生成的点
   ```
3. **运行程序**：
   ```bash
   python demo_ridgebase_astar_nav.py
   ```
4. **等待 Isaac Sim 启动**（首次启动可能需要几分钟）

### 方式二：手动指定起点和终点

1. **打开文件** `demo_ridgebase_astar_nav.py`
2. **修改配置**（第 105-110 行）：
   ```python
   USE_MANUAL_POINTS = True  # 启用手动指定
   MANUAL_START_POINT = [0.0, 0.0]  # 起点坐标 [x, y]
   MANUAL_END_POINT = [5.0, 2.0]    # 终点坐标 [x, y]
   ```
3. **查看场景边界**（在 `config/navigation/navigation_assets.yaml`）：
   ```yaml
   x_bounds: [-7.925, 9.175]  # X 轴范围
   y_bounds: [-1.525, 4.175]  # Y 轴范围
   ```
4. **运行程序**

---

## 🖱️ Isaac Sim GUI 操作

### 主窗口界面

启动后，你会看到 Isaac Sim 的主窗口，包含：

1. **3D 视图窗口**（中央）
   - 显示场景和机器人
   - 可以用鼠标旋转、平移、缩放视角

2. **时间轴控制栏**（底部）
   - ⏸️ **暂停/播放按钮**：控制仿真运行
   - ⏹️ **停止按钮**：停止仿真（会触发重置）
   - ⏪ **重置按钮**：重置到初始状态

3. **属性面板**（右侧）
   - 可以查看和编辑场景对象属性

### 基本操作

#### 视角控制
- **旋转视角**：按住鼠标左键拖动
- **平移视角**：按住鼠标中键（滚轮）拖动
- **缩放视角**：滚动鼠标滚轮
- **聚焦对象**：双击场景中的对象

#### 仿真控制
- **播放仿真**：点击底部播放按钮 ▶️ 或按 `Space` 键
- **暂停仿真**：点击暂停按钮 ⏸️ 或按 `Space` 键
- **停止仿真**：点击停止按钮 ⏹️（会触发重置）
- **单步执行**：暂停后点击单步按钮

### 观察机器人移动

1. **启动后**：
   - 机器人会出现在起点位置
   - 控制台会打印路径信息

2. **点击播放**：
   - 机器人开始沿路径移动
   - 可以看到机器人转向、前进

3. **观察路径**：
   - 机器人会依次经过所有航点
   - 到达终点后会自动重置并开始新任务

---

## ⚙️ 设置起点和终点

### 查看场景边界

场景边界定义在 `config/navigation/navigation_assets.yaml`：

```yaml
x_bounds: [-7.925000286102295, 9.174999809265136]
y_bounds: [-1.5250000715255738, 4.175000286102295]
```

### 设置有效坐标

**重要规则**：
1. ✅ 坐标必须在边界范围内
2. ✅ 坐标不能在障碍物上（黑色区域）
3. ✅ 坐标必须在自由空间（白色区域）

### 示例配置

```python
# 示例 1：场景左下角到右上角
USE_MANUAL_POINTS = True
MANUAL_START_POINT = [-5.0, 0.0]
MANUAL_END_POINT = [5.0, 3.0]

# 示例 2：短距离导航
USE_MANUAL_POINTS = True
MANUAL_START_POINT = [0.0, 0.0]
MANUAL_END_POINT = [2.0, 1.0]

# 示例 3：长距离导航
USE_MANUAL_POINTS = True
MANUAL_START_POINT = [-7.0, -1.0]
MANUAL_END_POINT = [8.0, 3.5]
```

### 验证坐标

如果坐标无效，程序会打印错误信息：

```
错误: 起点位于障碍物上
起点: [0.0, 0.0], 终点: [5.0, 2.0]
场景边界: x=[-7.925, 9.175], y=[-1.525, 4.175]
```

根据错误信息调整坐标。

---

## 🔧 调试和参数调整

### 调整机器人速度

在 `demo_ridgebase_astar_nav.py` 第 84-90 行：

```python
controller = RidgebaseController(
    robot_articulation=robot,
    max_linear_speed=0.02,      # 最大线速度 (m/s)，增大可加快移动
    max_angular_speed=1.5,      # 最大角速度 (rad/s)，增大可加快转向
    position_threshold=0.08,    # 位置到达阈值 (m)，增大更容易到达航点
    angle_threshold=0.1         # 角度到达阈值 (rad)，增大更容易完成转向
)
```

**建议值**：
- **慢速模式**：`max_linear_speed=0.01`（更精确）
- **快速模式**：`max_linear_speed=0.05`（更快但可能不够平滑）
- **默认模式**：`max_linear_speed=0.02`（平衡）

### 查看路径信息

程序会在控制台打印：

```
使用手动指定的点 - 起点: [0.0, 0.0], 终点: [5.0, 2.0]
Path found! Start: [0.0, 0.0], End: [5.0, 2.0]
Navigation completed! Resetting new navigation task...
```

### 调试技巧

1. **查看当前位姿**：
   在代码第 207-210 行添加打印：
   ```python
   print(f"当前位置: ({position[0]:.2f}, {position[1]:.2f}), 角度: {euler_angles[2]:.2f}")
   ```

2. **查看航点信息**：
   在代码第 166 行后添加：
   ```python
   print(f"航点数量: {len(waypoints)}")
   for i, wp in enumerate(waypoints):
       print(f"  航点 {i}: ({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f})")
   ```

3. **保存路径图像**：
   在 `utils/a_star.py` 的 `plan_navigation_path` 函数中，可以调用：
   ```python
   save_path_image(inflated_grid, path_grid, "path_visualization.png")
   ```

---

## 🐛 常见问题

### 问题 1：Isaac Sim 窗口没有显示

**解决方案**：
- 检查 `headless=False`（第 3 行）
- 确保有图形界面环境（不是 SSH 连接）
- 检查显卡驱动

### 问题 2：机器人不移动

**检查清单**：
1. ✅ 是否点击了播放按钮？
2. ✅ 控制台是否有错误信息？
3. ✅ 路径是否成功规划？（查看 "Path found!" 消息）
4. ✅ 机器人是否在起点位置？

### 问题 3：坐标验证失败

**错误信息**：
```
错误: 起点位于障碍物上
```

**解决方案**：
1. 查看场景边界范围
2. 选择自由空间（白色区域）的坐标
3. 可以先用随机模式测试，观察有效坐标范围

### 问题 4：路径规划失败

**可能原因**：
- 起点或终点在障碍物上
- 起点和终点之间没有可行路径

**解决方案**：
1. 验证坐标是否在自由空间
2. 尝试不同的起点和终点
3. 检查障碍物地图是否正确加载

### 问题 5：机器人移动太慢/太快

**调整速度参数**：
```python
max_linear_speed=0.02  # 调整这个值
```

- 增大 → 移动更快
- 减小 → 移动更慢但更精确

---

## 📊 操作流程图

```
启动程序
    ↓
Isaac Sim 窗口打开
    ↓
场景和机器人加载
    ↓
路径规划（A*算法）
    ↓
机器人重置到起点
    ↓
【等待用户操作】
    ↓
点击播放按钮 ▶️
    ↓
机器人开始移动
    ↓
依次经过航点
    ↓
到达终点
    ↓
自动重置新任务
    ↓
（循环）
```

---

## 💡 高级技巧

### 1. 实时修改参数

可以在代码中添加交互式输入：

```python
import sys
if len(sys.argv) > 1:
    USE_MANUAL_POINTS = True
    MANUAL_START_POINT = [float(sys.argv[1]), float(sys.argv[2])]
    MANUAL_END_POINT = [float(sys.argv[3]), float(sys.argv[4])]
```

然后运行：
```bash
python demo_ridgebase_astar_nav.py 0.0 0.0 5.0 2.0
```

### 2. 保存运行视频

可以在主循环中添加截图功能，保存为视频。

### 3. 多任务测试

程序会自动循环执行多个导航任务，可以观察不同路径的表现。

---

## 📝 总结

1. **运行程序**：`python demo_ridgebase_astar_nav.py`
2. **设置坐标**：修改 `USE_MANUAL_POINTS` 和坐标值
3. **控制仿真**：使用 Isaac Sim GUI 的播放/暂停按钮
4. **观察移动**：在 3D 视图中观察机器人沿路径移动
5. **调整参数**：根据需要修改速度和控制参数

祝你使用愉快！🎉

