"""
从 USD 场景生成障碍物地图的工具

使用方法：
    python utils/generate_barrier_map.py --usd_path <usd_file> --output <output_image> --x_bounds -10 10 --y_bounds -5 5
"""

import os
import argparse
import numpy as np
from PIL import Image
from typing import Tuple, List

# Isaac Sim imports
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni
import omni.usd
from pxr import UsdGeom, Gf, UsdPhysics
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera


def get_scene_bounds(stage) -> Tuple[List[float], List[float]]:
    """
    从 USD 场景中获取边界框
    
    Returns:
        x_bounds: [x_min, x_max]
        y_bounds: [y_min, y_max]
    """
    x_min, x_max = float('inf'), float('-inf')
    y_min, y_max = float('inf'), float('-inf')
    
    # 遍历所有 prim
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Xformable):
            xformable = UsdGeom.Xformable(prim)
            
            # 获取世界变换矩阵
            world_transform = xformable.ComputeLocalToWorldTransform(0)
            
            # 获取包围盒（如果存在）
            bbox_cache = UsdGeom.BBoxCache(0, [])
            bbox = bbox_cache.ComputeWorldBound(prim)
            
            if bbox.IsEmpty():
                continue
                
            # 获取边界
            bbox_min = bbox.GetRange().GetMin()
            bbox_max = bbox.GetRange().GetMax()
            
            x_min = min(x_min, bbox_min[0])
            x_max = max(x_max, bbox_max[0])
            y_min = min(y_min, bbox_min[1])
            y_max = max(y_max, bbox_max[1])
    
    # 如果未找到边界，使用默认值
    if x_min == float('inf'):
        x_min, x_max = -10.0, 10.0
        y_min, y_max = -5.0, 5.0
    
    return [x_min, x_max], [y_min, y_max]


def generate_barrier_map_raycast(
    stage,
    x_bounds: List[float],
    y_bounds: List[float],
    resolution: Tuple[int, int] = (1024, 512),
    height: float = 0.5,
    ray_length: float = 2.0
) -> np.ndarray:
    """
    使用射线投射方法生成障碍物地图
    
    Args:
        stage: USD stage
        x_bounds: [x_min, x_max]
        y_bounds: [y_min, y_max]
        resolution: (width, height) 图像分辨率
        height: 射线起始高度
        ray_length: 射线长度
        
    Returns:
        barrier_map: 二维数组，1=障碍物，0=自由空间
    """
    width, height_px = resolution
    barrier_map = np.zeros((height_px, width), dtype=np.uint8)
    
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds
    
    # 获取物理场景
    physics_scene = UsdPhysics.Scene.Get(stage, "/physicsScene")
    
    # 遍历每个像素点
    for i in range(height_px):
        for j in range(width):
            # 计算真实世界坐标
            x = x_min + (j / width) * (x_max - x_min)
            y = y_max - (i / height_px) * (y_max - y_min)
            
            # 从上方发射射线
            ray_start = Gf.Vec3f(x, y, height)
            ray_dir = Gf.Vec3f(0, 0, -ray_length)
            
            # 使用 USD 的射线查询（需要物理场景）
            # 这里简化处理：检查是否有 prim 在下方
            has_obstacle = False
            
            for prim in stage.Traverse():
                if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube):
                    xformable = UsdGeom.Xformable(prim)
                    bbox_cache = UsdGeom.BBoxCache(0, [])
                    bbox = bbox_cache.ComputeWorldBound(prim)
                    
                    if bbox.IsEmpty():
                        continue
                    
                    bbox_min = bbox.GetRange().GetMin()
                    bbox_max = bbox.GetRange().GetMax()
                    
                    # 检查是否在 XY 平面内
                    if (bbox_min[0] <= x <= bbox_max[0] and 
                        bbox_min[1] <= y <= bbox_max[1] and
                        bbox_min[2] <= height <= bbox_max[2] + ray_length):
                        has_obstacle = True
                        break
            
            if has_obstacle:
                barrier_map[i, j] = 1
    
    return barrier_map


def generate_barrier_map_camera(
    world: World,
    x_bounds: List[float],
    y_bounds: List[float],
    resolution: Tuple[int, int] = (1024, 512),
    camera_height: float = 5.0
) -> np.ndarray:
    """
    使用俯视相机生成障碍物地图（基于深度图）
    
    Args:
        world: Isaac Sim World
        x_bounds: [x_min, x_max]
        y_bounds: [y_min, y_max]
        resolution: (width, height) 图像分辨率
        camera_height: 相机高度
        
    Returns:
        barrier_map: 二维数组，1=障碍物，0=自由空间
    """
    width, height_px = resolution
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds
    
    # 计算场景中心
    center_x = (x_min + x_max) / 2
    center_y = (y_min + y_max) / 2
    
    # 创建俯视相机
    camera = Camera(
        prim_path="/World/TopDownCamera",
        name="top_down_camera",
        resolution=resolution,
        position=np.array([center_x, center_y, camera_height]),
        orientation=np.array([0.5, 0.5, 0.5, 0.5])  # 向下看
    )
    
    world.reset()
    camera.initialize()
    
    # 获取深度图
    world.step(render=False)
    depth_data = camera.get_depth()
    
    # 将深度图转换为障碍物地图
    # 如果深度小于某个阈值，认为是障碍物
    threshold = camera_height - 0.5  # 假设地面高度为 0
    barrier_map = (depth_data < threshold).astype(np.uint8)
    
    # 清理
    camera.destroy()
    
    return barrier_map


def generate_barrier_map_from_prims(
    stage,
    x_bounds: List[float],
    y_bounds: List[float],
    resolution: Tuple[int, int] = (1024, 512),
    ignore_prims: List[str] = None
) -> np.ndarray:
    """
    通过遍历 USD prim 生成障碍物地图（推荐方法）
    
    Args:
        stage: USD stage
        x_bounds: [x_min, x_max]
        y_bounds: [y_min, y_max]
        resolution: (width, height) 图像分辨率
        ignore_prims: 要忽略的 prim 路径列表
        
    Returns:
        barrier_map: 二维数组，1=障碍物，0=自由空间
    """
    if ignore_prims is None:
        ignore_prims = []
    
    width, height_px = resolution
    barrier_map = np.zeros((height_px, width), dtype=np.uint8)
    
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds
    
    # 创建边界框缓存
    bbox_cache = UsdGeom.BBoxCache(0, [])
    
    # 遍历所有几何体 prim
    for prim in stage.Traverse():
        prim_path = str(prim.GetPath())
        
        # 跳过要忽略的 prim
        if any(ignore in prim_path for ignore in ignore_prims):
            continue
        
        # 只处理几何体
        if not (prim.IsA(UsdGeom.Mesh) or 
                prim.IsA(UsdGeom.Cube) or 
                prim.IsA(UsdGeom.Cylinder) or
                prim.IsA(UsdGeom.Sphere)):
            continue
        
        # 获取世界空间的包围盒
        bbox = bbox_cache.ComputeWorldBound(prim)
        
        if bbox.IsEmpty():
            continue
        
        bbox_min = bbox.GetRange().GetMin()
        bbox_max = bbox.GetRange().GetMax()
        
        # 投影到 XY 平面
        x_min_prim = bbox_min[0]
        x_max_prim = bbox_max[0]
        y_min_prim = bbox_min[1]
        y_max_prim = bbox_max[1]
        
        # 转换为像素坐标
        j_min = int((x_min_prim - x_min) / (x_max - x_min) * width)
        j_max = int((x_max_prim - x_min) / (x_max - x_min) * width)
        i_min = int((y_max - y_max_prim) / (y_max - y_min) * height_px)
        i_max = int((y_max - y_min_prim) / (y_max - y_min) * height_px)
        
        # 限制在图像范围内
        j_min = max(0, min(j_min, width - 1))
        j_max = max(0, min(j_max, width - 1))
        i_min = max(0, min(i_min, height_px - 1))
        i_max = max(0, min(i_max, height_px - 1))
        
        # 标记障碍物区域
        barrier_map[i_min:i_max+1, j_min:j_max+1] = 1
    
    return barrier_map


def save_barrier_image(barrier_map: np.ndarray, output_path: str):
    """
    保存障碍物地图为 PNG 图像
    
    Args:
        barrier_map: 障碍物地图数组 (0=自由空间, 1=障碍物)
        output_path: 输出文件路径
    """
    # 转换为图像格式：0 -> 255 (白色，自由空间), 1 -> 0 (黑色，障碍物)
    image_array = (1 - barrier_map) * 255
    
    # 转换为 PIL Image
    image = Image.fromarray(image_array.astype(np.uint8), mode='L')
    
    # 保存
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    image.save(output_path)
    print(f"障碍物地图已保存到: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='从 USD 场景生成障碍物地图')
    parser.add_argument('--usd_path', type=str, required=True,
                       help='USD 场景文件路径')
    parser.add_argument('--output', type=str, required=True,
                       help='输出图像路径（PNG）')
    parser.add_argument('--x_bounds', type=float, nargs=2, default=None,
                       help='X 轴边界 [x_min, x_max]')
    parser.add_argument('--y_bounds', type=float, nargs=2, default=None,
                       help='Y 轴边界 [y_min, y_max]')
    parser.add_argument('--resolution', type=int, nargs=2, default=(1024, 512),
                       help='图像分辨率 [width, height]')
    parser.add_argument('--method', type=str, default='prims',
                       choices=['prims', 'raycast', 'camera'],
                       help='生成方法：prims (推荐), raycast, camera')
    parser.add_argument('--ignore_prims', type=str, nargs='*', default=[],
                       help='要忽略的 prim 路径（例如：/World/Robot）')
    
    args = parser.parse_args()
    
    # 初始化 Isaac Sim
    world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene")
    
    # 加载 USD 场景
    usd_path = os.path.abspath(args.usd_path)
    add_reference_to_stage(usd_path=usd_path, prim_path="/World")
    
    # 获取 stage
    stage = omni.usd.get_context().get_stage()
    
    # 获取边界（如果未指定）
    if args.x_bounds is None or args.y_bounds is None:
        print("自动检测场景边界...")
        x_bounds, y_bounds = get_scene_bounds(stage)
        print(f"检测到的边界: x={x_bounds}, y={y_bounds}")
    else:
        x_bounds = args.x_bounds
        y_bounds = args.y_bounds
    
    # 生成障碍物地图
    print(f"使用 {args.method} 方法生成障碍物地图...")
    if args.method == 'prims':
        barrier_map = generate_barrier_map_from_prims(
            stage, x_bounds, y_bounds, 
            resolution=tuple(args.resolution),
            ignore_prims=args.ignore_prims
        )
    elif args.method == 'raycast':
        barrier_map = generate_barrier_map_raycast(
            stage, x_bounds, y_bounds,
            resolution=tuple(args.resolution)
        )
    elif args.method == 'camera':
        world.reset()
        barrier_map = generate_barrier_map_camera(
            world, x_bounds, y_bounds,
            resolution=tuple(args.resolution)
        )
    else:
        raise ValueError(f"未知的方法: {args.method}")
    
    # 保存图像
    save_barrier_image(barrier_map, args.output)
    
    print(f"\n生成完成！")
    print(f"  边界: x={x_bounds}, y={y_bounds}")
    print(f"  分辨率: {barrier_map.shape[1]} x {barrier_map.shape[0]}")
    print(f"  障碍物像素数: {np.sum(barrier_map)}")
    print(f"  自由空间像素数: {np.sum(barrier_map == 0)}")
    
    simulation_app.close()


if __name__ == "__main__":
    main()

