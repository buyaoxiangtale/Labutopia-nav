"""
从 USD 场景生成障碍物地图的完整示例

这个脚本展示了如何在 Isaac Sim 中从 USD 场景生成障碍物地图。
"""

import os
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})  # 设置为 False 可以看到场景

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.usd
from pxr import UsdGeom

# 导入我们的工具函数
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils.generate_barrier_map_simple import generate_barrier_map_from_usd, save_barrier_image


def main():
    # 初始化 Isaac Sim
    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene")
    
    # 配置参数
    usd_path = "assets/navigation_lab/navigation_lab_01/lab.usd"
    output_path = "assets/navigation/barrier/lab_1_generated.png"
    
    # 场景边界（从配置文件或手动指定）
    x_bounds = [-7.925000286102295, 9.174999809265136]
    y_bounds = [-1.5250000715255738, 4.175000286102295]
    
    # 图像分辨率
    resolution = (1024, 512)
    
    # 要忽略的 prim（例如地板、机器人等）
    ignore_prims = [
        "/World/Floor",
        "/World/Ground",
        "/World/Robot",
        "/World/Ridgebase"  # 如果场景中有机器人
    ]
    
    print("=" * 60)
    print("从 USD 场景生成障碍物地图")
    print("=" * 60)
    print(f"USD 文件: {usd_path}")
    print(f"输出路径: {output_path}")
    print(f"边界: x={x_bounds}, y={y_bounds}")
    print(f"分辨率: {resolution}")
    print("=" * 60)
    
    # 加载 USD 场景
    print("\n正在加载 USD 场景...")
    add_reference_to_stage(
        usd_path=os.path.abspath(usd_path),
        prim_path="/World"
    )
    
    # 等待场景加载
    world.reset()
    
    # 获取 stage
    stage = omni.usd.get_context().get_stage()
    
    if stage is None:
        print("错误: 无法获取 USD stage")
        simulation_app.close()
        return
    
    print("场景加载完成！")
    
    # 生成障碍物地图
    print("\n正在生成障碍物地图...")
    barrier_map = generate_barrier_map_from_usd(
        usd_path=usd_path,
        x_bounds=x_bounds,
        y_bounds=y_bounds,
        resolution=resolution,
        ignore_prims=ignore_prims,
        output_path=output_path
    )
    
    print("\n" + "=" * 60)
    print("生成完成！")
    print("=" * 60)
    print(f"障碍物地图形状: {barrier_map.shape}")
    print(f"障碍物像素数: {np.sum(barrier_map)}")
    print(f"自由空间像素数: {np.sum(barrier_map == 0)}")
    print(f"障碍物占比: {np.sum(barrier_map) / barrier_map.size * 100:.2f}%")
    print(f"图像已保存到: {output_path}")
    
    # 保持窗口打开（如果 headless=False）
    print("\n按 Ctrl+C 退出...")
    
    while simulation_app.is_running():
        world.step(render=True)
    
    simulation_app.close()


if __name__ == "__main__":
    main()

