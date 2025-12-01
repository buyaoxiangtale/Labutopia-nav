import os
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
import numpy as np
import yaml
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

from controllers.robot_controllers.ridgebase.ridgebase_controller import RidgebaseController
from utils.a_star import plan_navigation_path, real_to_grid, load_grid

def load_assets_config(config_path):
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config['assets']

def generate_random_points(x_bounds, y_bounds, grid, attempts=100):
    """Generate random start and end points in free space"""
    W = len(grid[0])
    H = len(grid)
    
    for _ in range(attempts):
        start_x = np.random.uniform(x_bounds[0], x_bounds[1])
        start_y = np.random.uniform(y_bounds[0], y_bounds[1])
        end_x = np.random.uniform(x_bounds[0], x_bounds[1])
        end_y = np.random.uniform(y_bounds[0], y_bounds[1])
        
        i_start, j_start = real_to_grid(
            start_x, start_y, x_bounds, y_bounds, (W, H)
        )
        i_end, j_end = real_to_grid(
            end_x, end_y, x_bounds, y_bounds, (W, H)
        )
        
        if grid[i_start][j_start] == 0 and grid[i_end][j_end] == 0:
            return [start_x, start_y], [end_x, end_y]
            
    raise RuntimeError("Failed to find valid start and end points")

def validate_points(start_point, end_point, x_bounds, y_bounds, grid):
    """Validate if start and end points are in free space"""
    W = len(grid[0])
    H = len(grid)
    
    i_start, j_start = real_to_grid(
        start_point[0], start_point[1], x_bounds, y_bounds, (W, H)
    )
    i_end, j_end = real_to_grid(
        end_point[0], end_point[1], x_bounds, y_bounds, (W, H)
    )
    
    if not (x_bounds[0] <= start_point[0] <= x_bounds[1] and 
            y_bounds[0] <= start_point[1] <= y_bounds[1]):
        return False, "起点超出场景边界"
    
    if not (x_bounds[0] <= end_point[0] <= x_bounds[1] and 
            y_bounds[0] <= end_point[1] <= y_bounds[1]):
        return False, "终点超出场景边界"
    
    if grid[i_start][j_start] != 0:
        return False, "起点位于障碍物上"
    
    if grid[i_end][j_end] != 0:
        return False, "终点位于障碍物上"
    
    return True, "验证通过"

def main():
    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene")
    
    robot_path = "/World/Ridgebase"
    add_reference_to_stage(
        usd_path="assets/robots/ridgeback_franka.usd",
        prim_path=robot_path
    )
    
    from omni.isaac.core.robots import Robot
    robot = Robot(
        prim_path=robot_path,
        name="ridgebase",
        position=np.array([0.0, 0.0, 0.0])
    )
    
    controller = RidgebaseController(
        robot_articulation=robot,
        max_linear_speed=0.02,
        max_angular_speed=1.5,
        position_threshold=0.08,
        angle_threshold=0.1
    )
    
    assets = load_assets_config("config/navigation/navigation_assets.yaml")
    nav_scene = assets[0]  

    add_reference_to_stage(
        usd_path=os.path.abspath(nav_scene['scene_asset_path']), 
        prim_path="/World"
    )
    
    grid, W, H = load_grid(nav_scene['barrier_image_path'])
    
    # ========== 配置选项 ==========
    # 设置为 True 使用手动指定的起点和终点
    # 设置为 False 使用随机生成的起点和终点
    USE_MANUAL_POINTS = False
    
    # 手动指定的起点和终点（仅在 USE_MANUAL_POINTS=True 时生效）
    # 坐标需要在场景边界内，且不在障碍物上
    # 场景边界: x=[-7.925, 9.175], y=[-1.525, 4.175]
    MANUAL_START_POINT = [0.0, 0.0]  # [x, y]
    MANUAL_END_POINT = [5.0, 2.0]    # [x, y]
    # ==============================
    
    print("=" * 60)
    print("Isaac Sim 导航演示程序")
    print("=" * 60)
    print(f"场景边界: x={nav_scene['x_bounds']}, y={nav_scene['y_bounds']}")
    print(f"使用模式: {'手动指定' if USE_MANUAL_POINTS else '随机生成'}")
    if USE_MANUAL_POINTS:
        print(f"起点: {MANUAL_START_POINT}")
        print(f"终点: {MANUAL_END_POINT}")
    print("=" * 60)
    print("\n操作提示:")
    print("1. 等待 Isaac Sim 窗口打开")
    print("2. 点击底部的播放按钮 ▶️ 开始仿真")
    print("3. 机器人会自动沿路径移动到终点")
    print("4. 到达终点后会自动重置并开始新任务")
    print("5. 使用鼠标可以旋转、平移、缩放视角")
    print("=" * 60)
    print()
    
    def reset_navigation():
        """Reset navigation task with new start/end points and path"""
        path_result = None
        while path_result is None:
            try:
                # 根据配置选择使用手动指定或随机生成起点和终点
                if USE_MANUAL_POINTS:
                    start_point = MANUAL_START_POINT
                    end_point = MANUAL_END_POINT
                    
                    # 验证手动指定的点是否有效
                    is_valid, message = validate_points(
                        start_point, end_point,
                        nav_scene['x_bounds'],
                        nav_scene['y_bounds'],
                        grid
                    )
                    if not is_valid:
                        print(f"错误: {message}")
                        print(f"起点: {start_point}, 终点: {end_point}")
                        print(f"场景边界: x={nav_scene['x_bounds']}, y={nav_scene['y_bounds']}")
                        raise RuntimeError(message)
                    print(f"使用手动指定的点 - 起点: {start_point}, 终点: {end_point}")
                else:
                    start_point, end_point = generate_random_points(
                        nav_scene['x_bounds'],
                        nav_scene['y_bounds'],
                        grid
                    )
                    print(f"使用随机生成的点 - 起点: {start_point}, 终点: {end_point}")
                
                task_info = {
                    "asset": nav_scene,
                    "start": start_point,
                    "end": end_point
                }
                
                path_result = plan_navigation_path(task_info)
                if path_result is not None:
                    merged_path_real, _ = path_result
                    print(f"\n✅ 路径规划成功!")
                    print(f"   起点: [{start_point[0]:.2f}, {start_point[1]:.2f}]")
                    print(f"   终点: [{end_point[0]:.2f}, {end_point[1]:.2f}]")
                    print(f"   路径点数: {len(merged_path_real)}")
                    print(f"   预计距离: {np.sqrt((end_point[0]-start_point[0])**2 + (end_point[1]-start_point[1])**2):.2f} 米")
                    
                    waypoints = []
                    for i in range(len(merged_path_real)):
                        x, y, r = merged_path_real[i]
                        
                        if i < len(merged_path_real) - 1:
                            next_x, next_y, _ = merged_path_real[i + 1]
                            theta = np.arctan2(next_y - y, next_x - x)
                        else:
                            theta = waypoints[-1][2] if waypoints else 0.0
                        waypoints.append([x, y, theta])
                    
                    controller.set_waypoints(waypoints)
                    
                    initial_position = np.array([start_point[0], start_point[1], 0.0])
                    initial_orientation = np.array([0.0, 0.0, waypoints[0][2]])  

                    from omni.isaac.core.utils.rotations import euler_angles_to_quat
                    initial_rotation = euler_angles_to_quat(initial_orientation)
                    
                    robot.set_world_pose(position=initial_position)
                    return True
                    
            except RuntimeError as e:
                print(f"Error: {e}")
                return False
        
        return False

    if not reset_navigation():
        simulation_app.close()
        return
        
    reset_need = False
    world.reset()
    robot.initialize()
    
    while simulation_app.is_running():
        world.step(render=True)
        
        if world.is_stopped():
            reset_need = True
            
        if world.is_playing():
            if reset_need:
                world.reset()
                robot.initialize()
                if not reset_navigation():  
                    simulation_app.close()
                    break

                reset_need = False
                
            position, orientation = robot.get_world_pose()
            from omni.isaac.core.utils.rotations import quat_to_euler_angles
            euler_angles = quat_to_euler_angles(orientation, extrinsic=False)
            current_pose = np.array([position[0], position[1], euler_angles[2]])
            
            action, done = controller.get_action(current_pose)
            
            if action is not None:
                robot.apply_action(action)
            
            if done or controller.is_path_complete():
                final_position = robot.get_world_pose()[0]
                print(f"\n🎉 导航任务完成!")
                print(f"   最终位置: [{final_position[0]:.2f}, {final_position[1]:.2f}]")
                print(f"   准备开始新的导航任务...\n")
                reset_need = True
    
    simulation_app.close()

if __name__ == "__main__":
    main()
