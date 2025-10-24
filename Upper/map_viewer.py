#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地图查看器 - 查看已保存的SLAM地图
"""

import numpy as np
import matplotlib.pyplot as plt
import json
import sys
import os
from PIL import Image

class MapViewer:
    """地图查看器"""
    
    @staticmethod
    def view_map(filename):
        """查看单个地图"""
        print(f"\n📖 正在加载地图: {filename}")
        
        if filename.endswith('.json'):
            # 加载JSON元数据
            with open(filename, 'r', encoding='utf-8') as f:
                metadata = json.load(f)
            
            # 加载地图数据
            grid_map = np.load(metadata['npy_file'])
            
            # 显示元数据
            print("\n📊 地图信息:")
            print(f"  时间戳: {metadata['timestamp']}")
            print(f"  地图大小: {metadata['map_size']}")
            print(f"  分辨率: {metadata['resolution']}m/栅格")
            print(f"  最大范围: {metadata['max_range']}m")
            print(f"  扫描圈数: {metadata.get('scan_count', 'N/A')}")
            print(f"  总点数: {metadata.get('total_points', 'N/A')}")
            print(f"  轨迹长度: {metadata.get('trajectory_length', 'N/A')}")
            
            # 提取轨迹
            trajectory = metadata.get('robot_trajectory', [])
            
        elif filename.endswith('.npy'):
            # 直接加载NumPy数组
            grid_map = np.load(filename)
            metadata = None
            trajectory = []
            print("\n⚠️  只有地图数据，没有元数据")
        
        else:
            print("❌ 不支持的文件格式！")
            return
        
        # 可视化
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # 左图：原始数据
        ax1.set_title('地图数据（数值）', fontsize=14)
        ax1.set_aspect('equal')
        im1 = ax1.imshow(grid_map, cmap='gray_r', origin='lower', vmin=-1, vmax=100)
        plt.colorbar(im1, ax=ax1, label='占用概率')
        
        # 右图：PNG预览
        ax2.set_title('地图预览（PNG）', fontsize=14)
        ax2.set_aspect('equal')
        
        if metadata and 'png_file' in metadata and os.path.exists(metadata['png_file']):
            # 显示PNG图像
            png_img = Image.open(metadata['png_file'])
            ax2.imshow(png_img, origin='upper')
            ax2.axis('off')
        else:
            # 手动渲染
            img_array = np.zeros((grid_map.shape[0], grid_map.shape[1], 3), dtype=np.uint8)
            for i in range(grid_map.shape[0]):
                for j in range(grid_map.shape[1]):
                    value = grid_map[i, j]
                    if value == 0:
                        img_array[i, j] = [200, 200, 200]
                    elif value == -1:
                        img_array[i, j] = [255, 255, 255]
                    else:
                        intensity = max(0, 255 - int(value * 2.55))
                        img_array[i, j] = [intensity, intensity, intensity]
            
            ax2.imshow(np.flipud(img_array), origin='upper')
            ax2.axis('off')
        
        plt.tight_layout()
        plt.show()
        
        print("\n✅ 地图查看完成")
    
    @staticmethod
    def list_and_select():
        """列出所有地图并让用户选择"""
        save_dir = 'saved_maps'
        
        if not os.path.exists(save_dir):
            print("📂 还没有保存过地图")
            return
        
        json_files = sorted([f for f in os.listdir(save_dir) if f.endswith('_metadata.json')], reverse=True)
        
        if len(json_files) == 0:
            print("📂 还没有保存过地图")
            return
        
        print(f"\n📂 已保存的地图 ({len(json_files)}个):")
        for i, json_file in enumerate(json_files):
            json_path = os.path.join(save_dir, json_file)
            with open(json_path, 'r', encoding='utf-8') as f:
                metadata = json.load(f)
            print(f"  [{i+1}] {metadata['timestamp']} - 扫描{metadata.get('scan_count', 'N/A')}圈")
        
        # 用户选择
        try:
            choice = input("\n请输入要查看的地图编号（回车查看最新）: ").strip()
            if choice == '':
                index = 0
            else:
                index = int(choice) - 1
            
            if 0 <= index < len(json_files):
                selected_file = os.path.join(save_dir, json_files[index])
                MapViewer.view_map(selected_file)
            else:
                print("❌ 无效的编号")
        except ValueError:
            print("❌ 请输入有效的数字")
        except KeyboardInterrupt:
            print("\n\n👋 已取消")

def main():
    print("=" * 50)
    print("  SLAM地图查看器")
    print("=" * 50)
    
    if len(sys.argv) > 1:
        # 命令行参数指定文件
        filename = sys.argv[1]
        MapViewer.view_map(filename)
    else:
        # 交互式选择
        MapViewer.list_and_select()

if __name__ == "__main__":
    main()

