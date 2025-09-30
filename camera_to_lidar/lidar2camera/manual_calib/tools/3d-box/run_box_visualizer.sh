#!/bin/bash

echo "========================================"
echo "3D框点云可视化工具 - GUI版本"
echo "========================================"
echo
echo "可用的box文件:"
echo "  1. box1.pcd"
echo "  2. box2.pcd"
echo "  3. box3.pcd"
echo "  4. box4.pcd"
echo

read -p "请输入要可视化的box编号 (1-4): " choice

case $choice in
    1)
        python visualize_box_gui.py box1.pcd ground.pcd
        ;;
    2)
        python visualize_box_gui.py box2.pcd ground.pcd
        ;;
    3)
        python visualize_box_gui.py box3.pcd ground.pcd
        ;;
    4)
        python visualize_box_gui.py box4.pcd ground.pcd
        ;;
    *)
        echo "无效的选择!"
        ;;
esac

echo "按任意键退出..."
read -n 1