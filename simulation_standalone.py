import pyvista as pv
from pyvistaqt import BackgroundPlotter
from PyQt5.QtCore import QTimer
import numpy as np
import config
import os
import kinematics

class RobotSimulation:
    def __init__(self, parent=None):
        self.plotter = BackgroundPlotter(show=False)
        self.plotter.set_background("#333333")
        
        self.show_tcp = True
        self.show_floor = True
        self.show_skeleton = False 
        self.side_view_right = False
        
        self.actors = []
        self.tcp_actors = []
        self.skeleton_actor = None
        self.floor_actor = None
        
        self._load_models()
        self.floor_actor = self._add_floor()
        self.plotter.add_axes()
        
        self.reset_camera()
        # 初始化
        self.update_simulation([0] * 6, np.eye(4))
        
        QTimer.singleShot(10, self.reset_camera)

    def _load_models(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        base_path = os.path.join(current_dir, "assets", "urdf", "meshes")
        
        pbr_props = { "pbr": True, "smooth_shading": True, "split_sharp_edges": True }
        colors = ["#7f8c8d", "#bdc3c7", "#95a5a6", "#bdc3c7", "#95a5a6", "#bdc3c7", "#e74c3c"]

        for i, stl_name in enumerate(config.STL_FILES):
            try:
                # 使用 os.path.join 組合完整路徑
                full_path = os.path.join(base_path, stl_name)
                
                mesh = pv.read(full_path)
                c = colors[i] if i < len(colors) else "white"
                actor = self.plotter.add_mesh(mesh, color=c, **pbr_props)
                self.actors.append(actor)
            except Exception as e:
                print(f"Error loading {stl_name}: {e}")
                # 為了避免 actors 數量不對導致 update_simulation 罷工，
                # 這裡建議即使讀取失敗，也要塞一個 None 或空 Actor 佔位，或者檢查路徑是否正確

                
        axes_config = [((1, 0, 0), "red"), ((0, 1, 0), "green"), ((0, 0, 1), "blue")]
        AXIS_LEN, AXIS_WIDTH, TIP_RADIUS = 0.1, 0.01, 0.03

        for direction, color in axes_config:
            arrow = pv.Arrow(start=(0,0,0), direction=direction, scale=AXIS_LEN, 
                             shaft_radius=AXIS_WIDTH, tip_radius=TIP_RADIUS)
            actor = self.plotter.add_mesh(arrow, color=color, show_scalar_bar=False)
            self.tcp_actors.append(actor)

    def _add_floor(self):
        floor = pv.Plane(center=(0, 0, 0), direction=(0, 0, 1), 
                         i_size=1.0, j_size=1.0, i_resolution=20, j_resolution=20)
        return self.plotter.add_mesh(floor, color='#333333', 
                                     show_edges=True, edge_color='#555555',
                                     line_width=1, opacity=0.5, 
                                     pickable=False, reset_camera=False)

    def get_widget(self):
        return self.plotter.interactor

    def toggle_tcp(self, state):
        self.show_tcp = state
        for actor in self.tcp_actors:
            actor.SetVisibility(state)
        self.plotter.render()

    def toggle_floor(self, state):
        self.show_floor = state
        if self.floor_actor:
            self.floor_actor.SetVisibility(state)
        self.plotter.render()

    def toggle_skeleton(self, state):
        self.show_skeleton = state
        pass

    def update_simulation(self, joint_angles, tcp_offset_mat=None):
        if len(self.actors) != 7: return

        # 若無 offset 則使用單位矩陣
        if tcp_offset_mat is None:
            tcp_offset_mat = np.eye(4)

        # 定義硬體修正 (Hardware Fix)
        T_hw_fix = np.eye(4)
        T_hw_fix[2, 3] = -0.0236  # Z = -23.6mm

        skeleton_points = []

        # 1. Base
        base_xyz = [x * config.SCALE_FACTOR for x in config.BASE_MESH_OFFSET['xyz']]
        T_base = kinematics.get_tf_matrix(base_xyz, config.BASE_MESH_OFFSET['rpy'])
        self.actors[0].user_matrix = T_base
        self.actors[0].SetVisibility(not self.show_skeleton)
        T_current_joint = T_base 
        skeleton_points.append(T_base[:3, 3])

        # 2. Links 1~6
        for i, params in enumerate(config.URDF_PARAMS):
            raw_angle = joint_angles[i]
            angle = -raw_angle if params.get('invert', False) else raw_angle
            xyz = [x * config.SCALE_FACTOR for x in params['xyz']]
            
            T_fixed = kinematics.get_tf_matrix(xyz, params['rpy'])
            T_rot = kinematics.get_rotation_matrix(params['axis'], angle)
            T_current_joint = T_current_joint @ T_fixed @ T_rot
            
            # --- Hardware Fix ---
            if i == 5: # Link 6
                T_flange_phys = T_current_joint @ T_hw_fix
                skeleton_points.append(T_flange_phys[:3, 3])
            else:
                skeleton_points.append(T_current_joint[:3, 3])

            # Mesh 保持原位
            vis_xyz = [x * config.SCALE_FACTOR for x in params.get('mesh_xyz', [0,0,0])]
            T_visual = kinematics.get_tf_matrix(vis_xyz, params.get('mesh_rpy', [0,0,0]))
            
            actor_idx = i + 1
            self.actors[actor_idx].user_matrix = T_current_joint @ T_visual
            self.actors[actor_idx].SetVisibility(not self.show_skeleton)

        # --- 3. 更新 TCP 箭頭 ---
        T_tcp_display = T_current_joint @ T_hw_fix @ tcp_offset_mat

        for actor in self.tcp_actors:
            actor.user_matrix = T_tcp_display
            actor.SetVisibility(self.show_tcp)

        # 4. 繪製骨架
        points_np = np.array(skeleton_points)
        if self.show_skeleton:
            if self.skeleton_actor is None:
                lines = pv.lines_from_points(points_np)
                self.skeleton_actor = self.plotter.add_mesh(
                    lines, color='yellow', line_width=5, 
                    render_lines_as_tubes=True, name="skeleton_lines", reset_camera=False
                )
            else:
                mesh = self.skeleton_actor.mapper.dataset
                mesh.points = points_np
            self.skeleton_actor.SetVisibility(True)
        else:
            if self.skeleton_actor:
                self.skeleton_actor.SetVisibility(False)

        self.plotter.render()

    def reset_camera(self):
        self.plotter.view_isometric()
        restore_floor = False
        if self.floor_actor and self.floor_actor.GetVisibility():
            self.floor_actor.SetVisibility(False)
            restore_floor = True
        self.plotter.reset_camera()
        if restore_floor and self.floor_actor:
            self.floor_actor.SetVisibility(True)
    
    def view_top(self): self.plotter.view_xy()
    def view_front(self): self.plotter.view_xz(negative=True)
    def view_side(self):
        if self.side_view_right: self.plotter.view_yz()
        else: self.plotter.view_yz(negative=True)
        self.side_view_right = not self.side_view_right

    def fit_view(self):
        restore_floor = False
        if self.floor_actor and self.floor_actor.GetVisibility():
            self.floor_actor.SetVisibility(False)
            restore_floor = True
        self.plotter.reset_camera()
        if restore_floor and self.floor_actor:
            self.floor_actor.SetVisibility(True)
    
    def draw_trajectory(self, points):
        """
        在 3D 空間中畫出路徑點連線
        :param points: 包含 [x, y, z] 的清單或 numpy array
        """
        # 1. 如果畫面上已經有舊的軌跡線，先把它移除
        if hasattr(self, 'path_actor') and self.path_actor is not None:
            self.plotter.remove_actor(self.path_actor)
            self.path_actor = None
            
        # 2. 如果點不到 2 個（無法連成線），就直接結束
        if not points or len(points) < 2:
            return
            
        # 3. 把座標轉換為 PyVista 吃的 numpy 格式
        points_array = np.array(points)
        
        # 4. 建立連續線段 (PolyData)
        path_line = pv.lines_from_points(points_array)
        
        # 5. 加入到 plotter 中，設定顏色、線寬
        # 使用 name 參數可以確保如果重複加入，PyVista 會自動覆蓋同名的物件
        self.path_actor = self.plotter.add_mesh(
            path_line, 
            color='white', 
            line_width=2, 
            name='robot_trajectory',
            reset_camera=False   #
        )