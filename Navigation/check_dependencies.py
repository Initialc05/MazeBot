#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
依赖检查脚本

功能：
1. 检查所有必需的Python库
2. 验证版本兼容性
3. 检查模块导入
4. 生成依赖报告

作者：MazeBot开发团队
日期：2025年1月14日
"""

import sys
import importlib
import subprocess
from typing import Dict, List, Tuple

def check_python_version():
    """检查Python版本"""
    print("=== Python版本检查 ===")
    version = sys.version_info
    print(f"Python版本: {version.major}.{version.minor}.{version.micro}")
    
    if version.major >= 3 and version.minor >= 7:
        print("✅ Python版本符合要求 (>=3.7)")
        return True
    else:
        print("❌ Python版本过低，需要Python 3.7或更高版本")
        return False

def check_package(package_name: str, min_version: str = None) -> Tuple[bool, str]:
    """检查单个包的安装状态和版本"""
    try:
        module = importlib.import_module(package_name)
        version = getattr(module, '__version__', 'Unknown')
        
        if min_version and version != 'Unknown':
            # 简单的版本比较
            try:
                current_parts = [int(x) for x in version.split('.')]
                min_parts = [int(x) for x in min_version.split('.')]
                
                if current_parts >= min_parts:
                    return True, version
                else:
                    return False, f"{version} (需要 >= {min_version})"
            except:
                return True, version  # 如果版本比较失败，假设版本足够
        
        return True, version
    except ImportError:
        return False, "未安装"

def check_required_packages():
    """检查必需的包"""
    print("\n=== 必需包检查 ===")
    
    required_packages = {
        'serial': ('pyserial', '3.5'),
        'numpy': ('numpy', '1.21.0'),
        'matplotlib': ('matplotlib', '3.5.0'),
        'PIL': ('Pillow', '9.0.0'),
    }
    
    all_ok = True
    
    for import_name, (package_name, min_version) in required_packages.items():
        is_ok, version = check_package(import_name, min_version)
        status = "✅" if is_ok else "❌"
        print(f"{status} {package_name}: {version}")
        
        if not is_ok:
            all_ok = False
    
    return all_ok

def check_standard_libraries():
    """检查标准库"""
    print("\n=== 标准库检查 ===")
    
    standard_libs = [
        'threading', 'queue', 'time', 'math', 'dataclasses', 
        'typing', 'os', 'json', 'datetime', 'collections'
    ]
    
    all_ok = True
    
    for lib in standard_libs:
        try:
            importlib.import_module(lib)
            print(f"✅ {lib}: 可用")
        except ImportError:
            print(f"❌ {lib}: 不可用")
            all_ok = False
    
    return all_ok

def check_maze_modules():
    """检查迷宫匹配模块"""
    print("\n=== 迷宫匹配模块检查 ===")
    
    maze_modules = [
        'maze_mapper',
        'maze_visualizer', 
        'maze_interface'
    ]
    
    all_ok = True
    
    for module in maze_modules:
        try:
            importlib.import_module(module)
            print(f"✅ {module}: 导入成功")
        except ImportError as e:
            print(f"❌ {module}: 导入失败 - {e}")
            all_ok = False
    
    return all_ok

def check_host_modules():
    """检查主系统模块"""
    print("\n=== 主系统模块检查 ===")
    
    # 添加主系统路径
    import os
    host_path = os.path.join(os.path.dirname(__file__), '..', 'HOST')
    if host_path not in sys.path:
        sys.path.append(host_path)
    
    host_modules = [
        'serial',
        'struct',
        'lidar_visualizer'
    ]
    
    all_ok = True
    
    for module in host_modules:
        try:
            importlib.import_module(module)
            print(f"✅ {module}: 导入成功")
        except ImportError as e:
            print(f"❌ {module}: 导入失败 - {e}")
            all_ok = False
    
    return all_ok

def check_functionality():
    """检查功能完整性"""
    print("\n=== 功能完整性检查 ===")
    
    try:
        # 测试迷宫匹配器创建
        from maze_mapper import MazeMapper
        mapper = MazeMapper((0, 0), (4, 4))
        print("✅ 迷宫匹配器创建成功")
        
        # 测试可视化器创建
        from maze_visualizer import MazeVisualizer
        visualizer = MazeVisualizer()
        print("✅ 可视化器创建成功")
        
        # 测试接口创建
        from maze_interface import MazeInterface
        interface = MazeInterface((0, 0), (4, 4))
        print("✅ 迷宫接口创建成功")
        
        # 清理
        interface.stop()
        
        return True
        
    except Exception as e:
        print(f"❌ 功能测试失败: {e}")
        return False

def generate_install_commands():
    """生成安装命令"""
    print("\n=== 安装命令 ===")
    print("如果缺少依赖，请运行以下命令：")
    print()
    print("pip install pyserial>=3.5")
    print("pip install numpy>=1.21.0")
    print("pip install matplotlib>=3.5.0")
    print("pip install Pillow>=9.0.0")
    print()
    print("或者一次性安装：")
    print("pip install -r ../HOST/requirements.txt")

def main():
    """主函数"""
    print("MazeBot迷宫匹配依赖检查")
    print("=" * 50)
    
    # 执行所有检查
    checks = [
        ("Python版本", check_python_version),
        ("必需包", check_required_packages),
        ("标准库", check_standard_libraries),
        ("迷宫匹配模块", check_maze_modules),
        ("主系统模块", check_host_modules),
        ("功能完整性", check_functionality)
    ]
    
    results = {}
    
    for check_name, check_func in checks:
        try:
            results[check_name] = check_func()
        except Exception as e:
            print(f"❌ {check_name}检查出错: {e}")
            results[check_name] = False
    
    # 总结报告
    print("\n" + "=" * 50)
    print("=== 检查结果总结 ===")
    
    all_passed = True
    for check_name, passed in results.items():
        status = "✅ 通过" if passed else "❌ 失败"
        print(f"{check_name}: {status}")
        if not passed:
            all_passed = False
    
    print("\n" + "=" * 50)
    if all_passed:
        print("🎉 所有依赖检查通过！可以开始实机测试。")
        print("\n下一步：")
        print("1. 运行快速测试: python quick_test.py")
        print("2. 运行实机测试: python maze_test_integration.py")
    else:
        print("⚠️ 部分依赖检查失败，请先解决依赖问题。")
        generate_install_commands()
    
    return all_passed

if __name__ == "__main__":
    main()
