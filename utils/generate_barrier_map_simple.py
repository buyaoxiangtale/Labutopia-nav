"""
简化版：从 USD 场景生成障碍物地图

这个版本更简单易用，直接在主程序中调用即可。
"""

import os
import numpy as np
from PIL import Image
from typing import Tuple, List, Optional
import omni.usd
from pxr import UsdGeom, Gf


def generate_barrier_map_from_usd(
    usd_path: str,
    x_bounds: List[float],
    y_bounds: List[float],
    resolution: Tuple[int, int] = (1024, 512),
    ignore_prims: Optional[List[str]] = None,
    output_path: Optional[str] = None
) -> np.ndarray:
    """
    从 USD 场景生成障碍物地图（最简单的方法）
    
    Args:
        usd_path: USD 场景文件路径
        x_bounds: [x_min, x_max] X 轴边界
        y_bounds: [y_min, y_max] Y 轴边界
        resolution: (width, height) 图像分辨率
        ignore_prims: 要忽略的 prim 路径列表，例如 ["/World/Robot", "/World/Floor"]
        output_path: 可选，输出图像路径
        
    Returns:
        barrier_map: 二维数组，1=障碍物，0=自由空间
        
    Example:
        barrier_map = generate_barrier_map_from_usd(
            usd_path="assets/navigation_lab/navigation_lab_01/lab.usd",
            x_bounds=[-7.925, 9.175],
            y_bounds=[-1.525, 4.175],
            resolution=(1024, 512),
            ignore_prims=["/World/Floor", "/World/Ground"],
            output_path="assets/navigation/barrier/lab_1.png"
        )
    """
    if ignore_prims is None:
        ignore_prims = []
    
    # 获取 USD context 和 stage
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    
    if stage is None:
        raise RuntimeError("USD stage 未初始化，请确保 Isaac Sim 已启动")
    
    width, height_px = resolution
    barrier_map = np.zeros((height_px, width), dtype=np.uint8)
    
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds
    
    # 创建边界框缓存
    bbox_cache = UsdGeom.BBoxCache(0, [])
    
    # 统计信息
    processed_count = 0
    skipped_count = 0
    
    # 遍历所有 prim
    for prim in stage.Traverse():
        prim_path = str(prim.GetPath())
        
        # 跳过要忽略的 prim
        if any(ignore_path in prim_path for ignore_path in ignore_prims):
            skipped_count += 1
            continue
        
        # 只处理有几何体的 prim
        is_geometric = (
            prim.IsA(UsdGeom.Mesh) or 
            prim.IsA(UsdGeom.Cube) or 
            prim.IsA(UsdGeom.Cylinder) or
            prim.IsA(UsdGeom.Sphere) or
            prim.IsA(UsdGeom.BasisCurves)
        )
        
        if not is_geometric:
            continue
        
        # 获取世界空间的包围盒
        try:
            bbox = bbox_cache.ComputeWorldBound(prim)
            
            if bbox.IsEmpty():
                continue
            
            bbox_min = bbox.GetRange().GetMin()
            bbox_max = bbox.GetRange().GetMax()
            
            # 投影到 XY 平面（忽略 Z 轴）
            x_min_prim = bbox_min[0]
            x_max_prim = bbox_max[0]
            y_min_prim = bbox_min[1]
            y_max_prim = bbox_max[1]
            
            # 检查是否在边界内
            if (x_max_prim < x_min or x_min_prim > x_max or
                y_max_prim < y_min or y_min_prim > y_max):
                continue
            
            # 转换为像素坐标
            # X 方向
            j_min = int((x_min_prim - x_min) / (x_max - x_min) * width)
            j_max = int((x_max_prim - x_min) / (x_max - x_min) * width)
            
            # Y 方向（需要反转，因为图像坐标原点在左上角）
            i_min = int((y_max - y_max_prim) / (y_max - y_min) * height_px)
            i_max = int((y_max - y_min_prim) / (y_max - y_min) * height_px)
            
            # 限制在图像范围内
            j_min = max(0, min(j_min, width - 1))
            j_max = max(0, min(j_max, width - 1))
            i_min = max(0, min(i_min, height_px - 1))
            i_max = max(0, min(i_max, height_px - 1))
            
            # 标记障碍物区域
            barrier_map[i_min:i_max+1, j_min:j_max+1] = 1
            processed_count += 1
            
        except Exception as e:
            print(f"警告: 处理 prim {prim_path} 时出错: {e}")
            skipped_count += 1
            continue
    
    print(f"障碍物地图生成完成:")
    print(f"  处理了 {processed_count} 个几何体")
    print(f"  跳过了 {skipped_count} 个 prim")
    print(f"  障碍物像素: {np.sum(barrier_map)}")
    print(f"  自由空间像素: {np.sum(barrier_map == 0)}")
    
    # 保存图像
    if output_path:
        save_barrier_image(barrier_map, output_path)
    
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
    
    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else '.', exist_ok=True)
    
    # 保存
    image.save(output_path)
    print(f"障碍物地图已保存到: {output_path}")


def load_and_generate_from_config(config_path: str):
    """
    从配置文件加载场景并生成障碍物地图
    
    Args:
        config_path: navigation_assets.yaml 配置文件路径
    """
    import yaml
    
    # 加载配置
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    assets = config['assets']
    
    for asset in assets:
        scene_path = asset['scene_asset_path']
        x_bounds = asset['x_bounds']
        y_bounds = asset['y_bounds']
        
        # 生成输出路径（与场景文件同名）
        scene_dir = os.path.dirname(scene_path)
        scene_name = os.path.splitext(os.path.basename(scene_path))[0]
        output_path = os.path.join(
            os.path.dirname(scene_dir),
            "barrier",
            f"{scene_name}_barrier.png"
        )
        
        print(f"\n处理场景: {scene_path}")
        print(f"输出路径: {output_path}")
        
        # 生成障碍物地图
        barrier_map = generate_barrier_map_from_usd(
            usd_path=scene_path,
            x_bounds=x_bounds,
            y_bounds=y_bounds,
            resolution=(1024, 512),
            ignore_prims=["/World/Floor", "/World/Ground", "/World/Robot"],
            output_path=output_path
        )


if __name__ == "__main__":
    # 示例用法
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "from_config":
        # 从配置文件生成
        config_path = sys.argv[2] if len(sys.argv) > 2 else "config/navigation/navigation_assets.yaml"
        load_and_generate_from_config(config_path)
    else:
        print("使用方法:")
        print("1. 在代码中调用:")
        print("   from utils.generate_barrier_map_simple import generate_barrier_map_from_usd")
        print("   barrier_map = generate_barrier_map_from_usd(...)")
        print("\n2. 从配置文件生成:")
        print("   python utils/generate_barrier_map_simple.py from_config [config_path]")

