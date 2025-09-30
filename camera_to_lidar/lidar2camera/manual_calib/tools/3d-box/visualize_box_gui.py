"""
点云3D框可视化工具 - GUI版本
功能：
- 可视化界面显示点云和3D框
- GUI控制面板调整位置、大小和方向
- 3D框底边与地面共面
- 实时更新显示
"""

import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np
import threading
import time


class Box3DVisualizerGUI:
    def __init__(self, box_pcd_path, ground_pcd_path):
        """
        初始化3D框可视化器GUI版本
        
        参数:
            box_pcd_path: box点云文件路径
            ground_pcd_path: ground点云文件路径
        """
        self.box_pcd_path = box_pcd_path
        self.ground_pcd_path = ground_pcd_path
        
        # 加载点云
        self.box_pcd = o3d.io.read_point_cloud(box_pcd_path)
        self.ground_pcd = o3d.io.read_point_cloud(ground_pcd_path)
        
        # 为点云设置颜色
        self.box_pcd.paint_uniform_color([0, 0, 0])  # 黑色
        self.ground_pcd.paint_uniform_color([0, 0, 0])  # 黑色
        
        # 计算地面平面
        self.ground_plane = self._fit_ground_plane()
        
        # 初始化3D框参数
        self.box_center = np.array([0.0, 0.0, 0.0])
        self.box_size = np.array([1.0, 1.0, 1.0])
        self.box_rotation = 0.0
        
        # 根据box点云初始化参数
        self._initialize_box_from_points()
        
        # 保存初始值用于滑动条范围
        self.initial_center = self.box_center.copy()
        self.initial_size = self.box_size.copy()
        
        # 防止循环更新的标志
        self._updating = False
        
        # 创建应用和窗口
        self.app = gui.Application.instance
        self.app.initialize()
        
        self.window = self.app.create_window("3D框调整工具", 1600, 900)
        self.widget3d = gui.SceneWidget()
        self.window.add_child(self.widget3d)
        
        # 创建控制面板
        self._create_control_panel()
        
        # 设置场景
        self.widget3d.scene = rendering.Open3DScene(self.window.renderer)
        self._setup_scene()
        
        # 添加几何体
        self._add_geometries()
        
        # 初始化3D框
        self.box_lineset = self._create_box_lineset()
        self._update_box()
        
        # 设置窗口布局
        self.window.set_on_layout(self._on_layout)
    
    def _fit_ground_plane(self):
        """拟合地面平面"""
        points = np.asarray(self.ground_pcd.points)
        
        if len(points) < 3:
            print("警告: 地面点云点数不足，使用默认水平面")
            return np.array([0, 0, 1, 0])
        
        plane_model, inliers = self.ground_pcd.segment_plane(
            distance_threshold=0.01,
            ransac_n=3,
            num_iterations=1000
        )
        
        [a, b, c, d] = plane_model
        print(f"地面平面方程: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0")
        
        return np.array(plane_model)
    
    def _initialize_box_from_points(self):
        """从box点云初始化3D框的参数"""
        points = np.asarray(self.box_pcd.points)
        
        if len(points) == 0:
            print("警告: box点云为空")
            return
        
        min_bound = points.min(axis=0)
        max_bound = points.max(axis=0)
        self.box_size = max_bound - min_bound
        
        center = (min_bound + max_bound) / 2
        a, b, c, d = self.ground_plane
        distance = (a * center[0] + b * center[1] + c * center[2] + d) / np.sqrt(a**2 + b**2 + c**2)
        
        normal = np.array([a, b, c]) / np.sqrt(a**2 + b**2 + c**2)
        ground_center = center - distance * normal
        self.box_center = ground_center + normal * (self.box_size[2] / 2)
    
    def _create_control_panel(self):
        """创建GUI控制面板"""
        em = self.window.theme.font_size
        margin = 0.5 * em
        
        # 创建控制面板
        self.panel = gui.Vert(0.5 * em, gui.Margins(margin))
        
        # 标题
        title = gui.Label("3D Box Adjustment Tool")
        self.panel.add_child(title)
        self.panel.add_fixed(0.5 * em)
        
        # === 位置控制 ===
        position_label = gui.Label("Position")
        self.panel.add_child(position_label)
        
        # X位置
        self.panel.add_child(gui.Label("X Position"))
        h_x = gui.Horiz(0.25 * em)
        self.pos_x_slider = gui.Slider(gui.Slider.DOUBLE)
        self.pos_x_slider.set_limits(-10, 10)
        self.pos_x_slider.double_value = self.box_center[0]
        self.pos_x_slider.set_on_value_changed(self._on_pos_x_slider_changed)
        h_x.add_child(self.pos_x_slider)
        self.pos_x_edit = gui.TextEdit()
        self.pos_x_edit.text_value = f"{self.box_center[0]:.2f}"
        self.pos_x_edit.set_on_text_changed(self._on_pos_x_edit_changed)
        h_x.add_fixed(0.25 * em)
        h_x.add_child(self.pos_x_edit)
        self.panel.add_child(h_x)
        
        # Y位置
        self.panel.add_child(gui.Label("Y Position"))
        h_y = gui.Horiz(0.25 * em)
        self.pos_y_slider = gui.Slider(gui.Slider.DOUBLE)
        self.pos_y_slider.set_limits(-10, 10)
        self.pos_y_slider.double_value = self.box_center[1]
        self.pos_y_slider.set_on_value_changed(self._on_pos_y_slider_changed)
        h_y.add_child(self.pos_y_slider)
        self.pos_y_edit = gui.TextEdit()
        self.pos_y_edit.text_value = f"{self.box_center[1]:.2f}"
        self.pos_y_edit.set_on_text_changed(self._on_pos_y_edit_changed)
        h_y.add_fixed(0.25 * em)
        h_y.add_child(self.pos_y_edit)
        self.panel.add_child(h_y)
        
        self.panel.add_fixed(em)
        
        # === 旋转控制 ===
        rotation_label = gui.Label("Rotation")
        self.panel.add_child(rotation_label)
        
        self.panel.add_child(gui.Label("Angle (degrees)"))
        h_rot = gui.Horiz(0.25 * em)
        self.rotation_slider = gui.Slider(gui.Slider.DOUBLE)
        self.rotation_slider.set_limits(-180, 180)
        self.rotation_slider.double_value = 0
        self.rotation_slider.set_on_value_changed(self._on_rotation_slider_changed)
        h_rot.add_child(self.rotation_slider)
        self.rotation_edit = gui.TextEdit()
        self.rotation_edit.text_value = "0.00"
        self.rotation_edit.set_on_text_changed(self._on_rotation_edit_changed)
        h_rot.add_fixed(0.25 * em)
        h_rot.add_child(self.rotation_edit)
        self.panel.add_child(h_rot)
        
        self.panel.add_fixed(em)
        
        # === 大小控制 ===
        size_label = gui.Label("Size")
        self.panel.add_child(size_label)
        
        # 长度 (X)
        self.panel.add_child(gui.Label("Length (X)"))
        h_sx = gui.Horiz(0.25 * em)
        self.size_x_slider = gui.Slider(gui.Slider.DOUBLE)
        self.size_x_slider.set_limits(0.1, 10)
        self.size_x_slider.double_value = self.box_size[0]
        self.size_x_slider.set_on_value_changed(self._on_size_x_slider_changed)
        h_sx.add_child(self.size_x_slider)
        self.size_x_edit = gui.TextEdit()
        self.size_x_edit.text_value = f"{self.box_size[0]:.2f}"
        self.size_x_edit.set_on_text_changed(self._on_size_x_edit_changed)
        h_sx.add_fixed(0.25 * em)
        h_sx.add_child(self.size_x_edit)
        self.panel.add_child(h_sx)
        
        # 宽度 (Y)
        self.panel.add_child(gui.Label("Width (Y)"))
        h_sy = gui.Horiz(0.25 * em)
        self.size_y_slider = gui.Slider(gui.Slider.DOUBLE)
        self.size_y_slider.set_limits(0.1, 10)
        self.size_y_slider.double_value = self.box_size[1]
        self.size_y_slider.set_on_value_changed(self._on_size_y_slider_changed)
        h_sy.add_child(self.size_y_slider)
        self.size_y_edit = gui.TextEdit()
        self.size_y_edit.text_value = f"{self.box_size[1]:.2f}"
        self.size_y_edit.set_on_text_changed(self._on_size_y_edit_changed)
        h_sy.add_fixed(0.25 * em)
        h_sy.add_child(self.size_y_edit)
        self.panel.add_child(h_sy)
        
        # 高度 (Z)
        self.panel.add_child(gui.Label("Height (Z)"))
        h_sz = gui.Horiz(0.25 * em)
        self.size_z_slider = gui.Slider(gui.Slider.DOUBLE)
        self.size_z_slider.set_limits(0.1, 10)
        self.size_z_slider.double_value = self.box_size[2]
        self.size_z_slider.set_on_value_changed(self._on_size_z_slider_changed)
        h_sz.add_child(self.size_z_slider)
        self.size_z_edit = gui.TextEdit()
        self.size_z_edit.text_value = f"{self.box_size[2]:.2f}"
        self.size_z_edit.set_on_text_changed(self._on_size_z_edit_changed)
        h_sz.add_fixed(0.25 * em)
        h_sz.add_child(self.size_z_edit)
        self.panel.add_child(h_sz)
        
        self.panel.add_fixed(em)
        
        # === 参数显示 ===
        params_label = gui.Label("Current Parameters")
        self.panel.add_child(params_label)
        
        self.params_text = gui.Label("")
        self.panel.add_child(self.params_text)
        self._update_params_text()
        
        self.panel.add_fixed(em)
        
        # === 操作按钮 ===
        self.reset_button = gui.Button("Reset")
        self.reset_button.set_on_clicked(self._on_reset)
        self.panel.add_child(self.reset_button)
        
        self.save_button = gui.Button("Save Parameters")
        self.save_button.set_on_clicked(self._on_save)
        self.panel.add_child(self.save_button)
        
        # 添加面板到窗口
        self.window.add_child(self.panel)
    
    def _setup_scene(self):
        """设置3D场景"""
        self.widget3d.scene.set_background([1, 1, 1, 1])  # 白色背景
        self.widget3d.scene.scene.set_sun_light(
            [-1, -1, -1],
            [1, 1, 1],
            75000
        )
        self.widget3d.scene.scene.enable_sun_light(True)
        
        # 设置相机
        bounds = self.box_pcd.get_axis_aligned_bounding_box()
        self.widget3d.setup_camera(60, bounds, bounds.get_center())
    
    def _add_geometries(self):
        """添加几何体到场景"""
        # 创建材质
        mat = rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        mat.point_size = 3.0
        
        # 添加点云
        self.widget3d.scene.add_geometry("box_pcd", self.box_pcd, mat)
        self.widget3d.scene.add_geometry("ground_pcd", self.ground_pcd, mat)
    
    def _create_box_lineset(self):
        """创建3D框的线集"""
        half_size = self.box_size / 2
        vertices = np.array([
            [-half_size[0], -half_size[1], -half_size[2]],
            [half_size[0], -half_size[1], -half_size[2]],
            [half_size[0], half_size[1], -half_size[2]],
            [-half_size[0], half_size[1], -half_size[2]],
            [-half_size[0], -half_size[1], half_size[2]],
            [half_size[0], -half_size[1], half_size[2]],
            [half_size[0], half_size[1], half_size[2]],
            [-half_size[0], half_size[1], half_size[2]],
        ])
        
        rotation_matrix = np.array([
            [np.cos(self.box_rotation), -np.sin(self.box_rotation), 0],
            [np.sin(self.box_rotation), np.cos(self.box_rotation), 0],
            [0, 0, 1]
        ])
        vertices = vertices @ rotation_matrix.T
        vertices += self.box_center
        
        lines = [
            [0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7],
        ]
        
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(vertices)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        colors = [[1, 0, 0] for _ in range(len(lines))]
        line_set.colors = o3d.utility.Vector3dVector(colors)
        
        return line_set
    
    def _adjust_box_to_ground(self):
        """确保3D框的底边与地面共面"""
        a, b, c, d = self.ground_plane
        normal = np.array([a, b, c]) / np.sqrt(a**2 + b**2 + c**2)
        center_bottom = self.box_center - normal * (self.box_size[2] / 2)
        distance = (a * center_bottom[0] + b * center_bottom[1] + c * center_bottom[2] + d) / np.sqrt(a**2 + b**2 + c**2)
        self.box_center = self.box_center - distance * normal
    
    def _update_box(self):
        """更新3D框显示"""
        self._adjust_box_to_ground()
        self.box_lineset = self._create_box_lineset()
        
        # 移除旧的线集并添加新的
        if self.widget3d.scene.has_geometry("box"):
            self.widget3d.scene.remove_geometry("box")
        
        mat = rendering.MaterialRecord()
        mat.shader = "unlitLine"
        mat.line_width = 3.0
        self.widget3d.scene.add_geometry("box", self.box_lineset, mat)
        
        self._update_params_text()
    
    def _update_params_text(self):
        """更新参数显示文本"""
        text = f"Center: ({self.box_center[0]:.2f}, {self.box_center[1]:.2f}, {self.box_center[2]:.2f})\n"
        text += f"Size: ({self.box_size[0]:.2f}, {self.box_size[1]:.2f}, {self.box_size[2]:.2f})\n"
        text += f"Rotation: {np.degrees(self.box_rotation):.1f} deg"
        self.params_text.text = text
    
    # 回调函数
    def _on_pos_x_slider_changed(self, value):
        if self._updating:
            return
        self._updating = True
        self.box_center[0] = value
        self.pos_x_edit.text_value = f"{value:.2f}"
        self._update_box()
        self._updating = False
    
    def _on_pos_x_edit_changed(self, text):
        if self._updating:
            return
        try:
            value = float(text)
            self._updating = True
            self.box_center[0] = value
            self.pos_x_slider.double_value = value
            self._update_box()
            self._updating = False
        except ValueError:
            pass
    
    def _on_pos_y_slider_changed(self, value):
        if self._updating:
            return
        self._updating = True
        self.box_center[1] = value
        self.pos_y_edit.text_value = f"{value:.2f}"
        self._update_box()
        self._updating = False
    
    def _on_pos_y_edit_changed(self, text):
        if self._updating:
            return
        try:
            value = float(text)
            self._updating = True
            self.box_center[1] = value
            self.pos_y_slider.double_value = value
            self._update_box()
            self._updating = False
        except ValueError:
            pass
    
    def _on_rotation_slider_changed(self, value):
        if self._updating:
            return
        self._updating = True
        self.box_rotation = np.radians(value)
        self.rotation_edit.text_value = f"{value:.2f}"
        self._update_box()
        self._updating = False
    
    def _on_rotation_edit_changed(self, text):
        if self._updating:
            return
        try:
            value = float(text)
            self._updating = True
            self.box_rotation = np.radians(value)
            self.rotation_slider.double_value = value
            self._update_box()
            self._updating = False
        except ValueError:
            pass
    
    def _on_size_x_slider_changed(self, value):
        if self._updating:
            return
        self._updating = True
        self.box_size[0] = max(0.1, value)
        self.size_x_edit.text_value = f"{value:.2f}"
        self._update_box()
        self._updating = False
    
    def _on_size_x_edit_changed(self, text):
        if self._updating:
            return
        try:
            value = float(text)
            value = max(0.1, value)
            self._updating = True
            self.box_size[0] = value
            self.size_x_slider.double_value = value
            self._update_box()
            self._updating = False
        except ValueError:
            pass
    
    def _on_size_y_slider_changed(self, value):
        if self._updating:
            return
        self._updating = True
        self.box_size[1] = max(0.1, value)
        self.size_y_edit.text_value = f"{value:.2f}"
        self._update_box()
        self._updating = False
    
    def _on_size_y_edit_changed(self, text):
        if self._updating:
            return
        try:
            value = float(text)
            value = max(0.1, value)
            self._updating = True
            self.box_size[1] = value
            self.size_y_slider.double_value = value
            self._update_box()
            self._updating = False
        except ValueError:
            pass
    
    def _on_size_z_slider_changed(self, value):
        if self._updating:
            return
        self._updating = True
        self.box_size[2] = max(0.1, value)
        self.size_z_edit.text_value = f"{value:.2f}"
        self._update_box()
        self._updating = False
    
    def _on_size_z_edit_changed(self, text):
        if self._updating:
            return
        try:
            value = float(text)
            value = max(0.1, value)
            self._updating = True
            self.box_size[2] = value
            self.size_z_slider.double_value = value
            self._update_box()
            self._updating = False
        except ValueError:
            pass
    
    def _on_reset(self):
        """重置到初始状态"""
        self._initialize_box_from_points()
        
        self._updating = True
        # 更新滑动条
        self.pos_x_slider.double_value = self.box_center[0]
        self.pos_y_slider.double_value = self.box_center[1]
        self.rotation_slider.double_value = 0
        self.size_x_slider.double_value = self.box_size[0]
        self.size_y_slider.double_value = self.box_size[1]
        self.size_z_slider.double_value = self.box_size[2]
        
        # 更新输入框
        self.pos_x_edit.text_value = f"{self.box_center[0]:.2f}"
        self.pos_y_edit.text_value = f"{self.box_center[1]:.2f}"
        self.rotation_edit.text_value = "0.00"
        self.size_x_edit.text_value = f"{self.box_size[0]:.2f}"
        self.size_y_edit.text_value = f"{self.box_size[1]:.2f}"
        self.size_z_edit.text_value = f"{self.box_size[2]:.2f}"
        self._updating = False
        
        self._update_box()
        print("已重置到初始状态")
    
    def _on_save(self):
        """保存3D框参数"""
        import json
        import os
        
        # 计算旋转矩阵
        cos_r = np.cos(self.box_rotation)
        sin_r = np.sin(self.box_rotation)
        
        # 计算三个方向向量（单位向量）
        x_direction = np.array([cos_r, sin_r, 0.0])  # 长度方向
        y_direction = np.array([-sin_r, cos_r, 0.0])  # 宽度方向
        z_direction = np.array([0.0, 0.0, 1.0])  # 高度方向（竖直向上）
        
        # 获取box文件名（不含路径和扩展名）
        box_name = os.path.splitext(os.path.basename(self.box_pcd_path))[0]
        
        # 构建当前box的参数
        box_params = {
            "center": self.box_center.tolist(),
            "x_direction": x_direction.tolist(),
            "y_direction": y_direction.tolist(),
            "z_direction": z_direction.tolist(),
            "size": self.box_size.tolist()
        }
        
        # 读取现有的参数文件（如果存在）
        filename = "box_parameters.json"
        all_params = {}
        if os.path.exists(filename):
            try:
                with open(filename, "r", encoding="utf-8") as f:
                    all_params = json.load(f)
            except:
                all_params = {}
        
        # 更新当前box的参数
        all_params[box_name] = box_params
        
        # 保存所有参数
        with open(filename, "w", encoding="utf-8") as f:
            json.dump(all_params, f, indent=4, ensure_ascii=False)
        
        print(f"\n3D框参数已保存到 {filename}")
        print(f"Box名称: {box_name}")
        print(json.dumps(box_params, indent=2, ensure_ascii=False))
        
        # 显示保存成功的对话框
        self.window.show_message_box("Save Success", f"Parameters for '{box_name}' saved to {filename}")
    
    def _on_layout(self, layout_context):
        """布局回调"""
        content_rect = self.window.content_rect
        panel_width = 300
        
        # 面板在右侧
        self.panel.frame = gui.Rect(
            content_rect.get_right() - panel_width,
            content_rect.y,
            panel_width,
            content_rect.height
        )
        
        # 3D视图在左侧
        self.widget3d.frame = gui.Rect(
            content_rect.x,
            content_rect.y,
            content_rect.width - panel_width,
            content_rect.height
        )
    
    def run(self):
        """运行GUI应用"""
        print("\n" + "="*60)
        print("3D框交互式调整工具 - GUI版本")
        print("="*60)
        print(f"\n正在加载:")
        print(f"  Box点云: {self.box_pcd_path}")
        print(f"  Ground点云: {self.ground_pcd_path}")
        print("\n使用右侧控制面板调整3D框的位置、旋转和大小")
        print("="*60 + "\n")
        
        self.app.run()


def main():
    """主函数"""
    import sys
    
    if len(sys.argv) < 2:
        print("使用方法: python visualize_box_gui.py <box_pcd_file> [ground_pcd_file]")
        print("\n可用的box文件:")
        print("  - box1.pcd")
        print("  - box2.pcd")
        print("  - box3.pcd")
        print("  - box4.pcd")
        print("\n示例: python visualize_box_gui.py box1.pcd ground.pcd")
        sys.exit(1)
    
    box_pcd_path = sys.argv[1]
    ground_pcd_path = sys.argv[2] if len(sys.argv) > 2 else "ground.pcd"
    
    try:
        visualizer = Box3DVisualizerGUI(box_pcd_path, ground_pcd_path)
        visualizer.run()
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
