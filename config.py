import os
import numpy as np
import trimesh
from PySide6.QtWidgets import QWidget, QVBoxLayout
from vispy import scene

import kinematics

# ==========================================
# 全域機構與運算參數 (供 kinematics.py 讀取)
# ==========================================
SCALE_FACTOR = 1.0
IK_POS_TOLERANCE = 0.1
JOINT_LIMITS = [
    (-75, 255), (-50, 70), (-70, 70), (-95, 185), (-110, 110), (-120, 210)   
]

BASE_MESH_OFFSET = {'xyz': [0, 0, 0], 'rpy': [1.5708, 0, -1.5708]}
STL_FILES = [
    'base_link.STL', 'Link1.STL', 'Link2.STL', 'Link3.STL',
    'Link4.STL', 'Link5.STL', 'Link6.STL'
]
# 根據您重新定義原點後匯出的 URDF 參數
URDF_PARAMS = [
    {'xyz': [0,  0.1105,0],        'rpy': [3.1416, 0, 0],       'axis': 'y'}, 
    {'xyz': [-0.0234, 0, 0],       'rpy': [1.5708, -1.5708, 0], 'axis': 'x', 'invert': True}, 
    {'xyz': [0, 0, 0.18],          'rpy': [0, 0, 3.1416],       'axis': 'x'}, 
    {'xyz': [0, -0.0712, 0.04344], 'rpy': [3.1416, 0, -1.5708], 'axis': 'x', 'invert': True}, 
    {'xyz': [0.1053, 0, 0],        'rpy': [1.5708, 0, 1.5708],  'axis': 'x', 'invert': True}, 
    {'xyz': [0, 0, 0.0505],        'rpy': [3.1416, 0, 0],       'axis': 'z'}  
]

class Robot3DView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.canvas = scene.SceneCanvas(
            keys='interactive', show=False, bgcolor='#262626',
            config={'depth_size': 24, 'samples': 4}
        )
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = scene.cameras.TurntableCamera(
            fov=15, distance=2.5, elevation=30, azimuth=-225, center=(0, 0, 0.15)
        )

        scene.visuals.XYZAxis(parent=self.view.scene)

        self._make_floor()
        self._make_floor_grid(size=0.60, step=0.10)

        layout.addWidget(self.canvas.native)

        # 攝影棚 3 盞燈配置
        self.LIGHTS = [
            {'dir': np.array([-1.0,  1.0,  2.0]), 'strength': 0.30, 'two_sided': False}, # 主光源 (Key)
            {'dir': np.array([ 1.0, -1.0,  1.0]), 'strength': 0.20, 'two_sided': True},  # 側補光 (Fill)
            {'dir': np.array([ 0.0,  0.0, -1.0]), 'strength': 0.10, 'two_sided': True},  # 底部反光 (Bounce)
        ]
        self.AMBIENT = 0.55 # 基礎環境光

        self.actors = []
        self.ee_frame_visuals = []

        self._assemble_statically()

        self.drag_callback = None
        self.axis_drag_callback = None
        self.cancel_gizmo_callback = None 
        
        self._show_drag_sphere = False
        self._gizmo_mode = 'free'   
        self._active_drag_axis = None    
        self._is_dragging_free = False 
        self._drag_z = 0.0               
        self._last_mouse_pos = None
        self._bg_pressed = False
        self._bg_press_pos = None
        
        self.canvas.events.mouse_press.connect(self.on_mouse_press)
        self.canvas.events.mouse_move.connect(self.on_mouse_move)
        self.canvas.events.mouse_release.connect(self.on_mouse_release)
        
        self._update_gizmo_visuals()

    def _make_floor(self):
        self.floor = scene.visuals.Plane(
            width=0.6, height=0.6,
            color=(0.20, 0.20, 0.20, 1.0), 
            parent=self.view.scene
        )
        self.floor.set_gl_state('opaque', depth_test=True, cull_face=True)
        tf = scene.transforms.MatrixTransform()
        tf.translate((0, 0, -0.002))
        self.floor.transform = tf

    def _make_floor_grid(self, size: float = 0.60, step: float = 0.10):
        half = size / 2.0
        n = round(size / step) + 1
        ticks = np.linspace(-half, half, n)

        segments = []
        for t in ticks:
            segments.append([-half, t, 0.0])
            segments.append([ half, t, 0.0])
            segments.append([t, -half, 0.0])
            segments.append([t,  half, 0.0])

        pos = np.array(segments, dtype=np.float32)

        self.floor_grid = scene.visuals.Line(
            pos=pos,
            color=(0.45, 0.45, 0.45, 1.0), 
            connect='segments',
            width=1,
            antialias=True,
            parent=self.view.scene
        )
        
        grid_tf = scene.transforms.MatrixTransform()
        grid_tf.translate((0, 0, 0.0))
        self.floor_grid.transform = grid_tf

    EE_AXIS_LEN: float = 0.1
    
    def _make_ee_frame(self, parent_visual):
        
        if hasattr(self, 'ee_frame_visuals'):
            for v in self.ee_frame_visuals:
                v.parent = None
        self.ee_frame_visuals = []

        if hasattr(self, 'tcp_node') and self.tcp_node is not None:
            self.tcp_node.parent = None
        self.tcp_node = scene.Node(parent=parent_visual)

        # 基礎 TCP 座標線 -> Z 軸為 [0, 0, 1] 正向
        for end_pt, color in [([1, 0, 0], (0.95, 0.20, 0.20, 1.0)), 
                              ([0, 1, 0], (0.20, 0.85, 0.20, 1.0)), 
                              ([0, 0, 1], (0.20, 0.40, 0.95, 1.0))]:
            pts = np.array([[0.0, 0.0, 0.0], end_pt], dtype=np.float32)
            line = scene.visuals.Line(pos=pts, color=color, connect='strip', width=1.5, antialias=True, parent=self.tcp_node)
            tf = scene.transforms.MatrixTransform()
            tf.scale([self.EE_AXIS_LEN] * 3)
            line.transform = tf
            self.ee_frame_visuals.append(line)
            
        self.dragger_visual = scene.visuals.Sphere(radius=0.035, color=(0.5, 0.5, 0.5, 0.3), parent=self.tcp_node)
        self.dragger_visual_active = scene.visuals.Sphere(radius=0.04, color=(0.0, 0.9, 0.72, 0.6), parent=self.tcp_node)
        self.dragger_visual_active.visible = False
        
        cone_mesh_t = trimesh.creation.cone(radius=0.006, height=0.02)
        v_t = cone_mesh_t.vertices.astype(np.float32)
        f_t = cone_mesh_t.faces.astype(np.uint32)
        
        cone_mesh_r = trimesh.creation.cone(radius=0.006, height=0.02)
        v_r = cone_mesh_r.vertices.astype(np.float32)
        f_r = cone_mesh_r.faces.astype(np.uint32)

        # 直線箭頭
        self.gizmo_cones = []
        for ax, color, rot, pos in [
            ('x', (0.95, 0.20, 0.20, 1.0), [0, 90, 0], [0.1, 0, 0]),
            ('y', (0.20, 0.85, 0.20, 1.0), [-90, 0, 0], [0, 0.1, 0]),
            ('z', (0.20, 0.40, 0.95, 1.0), [0, 0, 0], [0, 0, 0.1])
        ]:
            cone = scene.visuals.Mesh(vertices=v_t, faces=f_t, color=color, parent=self.tcp_node)
            cone.set_gl_state('opaque', depth_test=True, cull_face=True) 
            
            tf = scene.transforms.MatrixTransform()
            if rot[0]: tf.rotate(rot[0], (1,0,0))
            if rot[1]: tf.rotate(rot[1], (0,1,0))
            tf.translate(pos)
            cone.transform = tf
            cone.visible = False
            self.gizmo_cones.append((ax, cone, pos))

        # 繞軸旋轉圓環
        self.gizmo_rings = []
        self.gizmo_ring_cones = []
        t = np.linspace(0, 2*np.pi, 60)
        c, s, zero = np.cos(t), np.sin(t), np.zeros_like(t)
        
        ring_offsets = {
            'x': [0.08, 0, 0],
            'y': [0, 0.08, 0],
            'z': [0, 0, 0.08] 
        }
        
        for ax, color, pts, rot, pos in [
            ('x', (0.95, 0.20, 0.20, 1.0), np.column_stack([zero, c, s])*0.06, [0,0,0], [0, 0.06, 0]), 
            ('y', (0.20, 0.85, 0.20, 1.0), np.column_stack([c, zero, s])*0.06, [0,90,0], [0, 0, 0.06]), 
            ('z', (0.20, 0.40, 0.95, 1.0), np.column_stack([c, s, zero])*0.06, [-90,0,0], [0.06, 0, 0]) 
        ]:
            offset = np.array(ring_offsets[ax], dtype=np.float32)
            ring = scene.visuals.Line(pos=pts + offset, color=color, width=1.5, antialias=True, connect='strip', parent=self.tcp_node)
            ring.visible = False
            self.gizmo_rings.append(ring)
            
            cone = scene.visuals.Mesh(vertices=v_r, faces=f_r, color=color, parent=self.tcp_node)
            cone.set_gl_state('opaque', depth_test=True, cull_face=True)
            
            tf = scene.transforms.MatrixTransform()
            if rot[0]: tf.rotate(rot[0], (1,0,0))
            if rot[1]: tf.rotate(rot[1], (0,1,0))
            tf.translate(np.array(pos) + offset) 
            cone.transform = tf
            cone.visible = False
            
            self.gizmo_ring_cones.append((ax, cone, np.array(pos) + offset))

    # 進階打光引擎：根據「世界座標」算光，把顏色貼回「局部座標」
    def _bake_lighting_post_transform(self, local_vertices, world_vertices, faces, base_color):
        v0 = world_vertices[faces[:, 0]]
        v1 = world_vertices[faces[:, 1]]
        v2 = world_vertices[faces[:, 2]]

        raw_normals = np.cross(v1 - v0, v2 - v0)
        norms = np.linalg.norm(raw_normals, axis=1, keepdims=True)
        norms = np.where(norms < 1e-12, 1.0, norms)
        normals = raw_normals / norms

        brightness = np.full(len(faces), self.AMBIENT, dtype=np.float64)
        for light in self.LIGHTS:
            d = light['dir'].astype(np.float64)
            d /= np.linalg.norm(d)
            dot = normals @ d
            contribution = np.abs(dot) if light['two_sided'] else np.clip(dot, 0, 1)
            brightness += light['strength'] * contribution
        brightness = np.clip(brightness, 0.0, 1.0)

        bc = np.array(base_color[:3], dtype=np.float64)
        
        new_vertices = local_vertices[faces.reshape(-1)].astype(np.float32) 
        new_faces = np.arange(len(faces) * 3, dtype=np.uint32).reshape(-1, 3)

        face_colors = np.outer(brightness, bc)
        vc = np.repeat(face_colors, 3, axis=0).astype(np.float32)
        vertex_colors = np.column_stack([vc, np.ones(len(vc), dtype=np.float32)])
        return new_vertices, new_faces, vertex_colors

    def _assemble_statically(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        base_path = os.path.join(current_dir, "assets", "urdf", "meshes")
        BASE_COLOR = (0.75, 0.75, 0.75, 1.0) 

        parent_node = self.view.scene
        
        # 取得組裝完成後的「初始零度世界矩陣」
        zero_matrices = kinematics.forward_kinematics_all([0.0]*6)

        for i, stl_name in enumerate(STL_FILES):
            full_path = os.path.join(base_path, stl_name)
            try:
                mesh_data = trimesh.load(full_path)
                vertices = np.array(mesh_data.vertices, dtype=np.float32) * 0.001
                faces    = np.array(mesh_data.faces,    dtype=np.uint32)
            except Exception:
                box      = trimesh.creation.box(extents=(0.05, 0.05, 0.05))
                vertices = np.array(box.vertices, dtype=np.float32)
                faces    = np.array(box.faces,    dtype=np.uint32)

            # --- 取出當前連桿的歸零轉換矩陣 ---
            M0 = zero_matrices[i] if i < len(zero_matrices) else np.eye(4)
            
            # --- 產生世界座標頂點 (僅供算光用) ---
            ones = np.ones((len(vertices), 1), dtype=np.float32)
            world_vertices = (M0 @ np.hstack([vertices, ones]).T).T[:, :3]

            # --- 神奇的打光魔法：用世界座標算光，用區域座標建模！ ---
            new_verts, new_faces, vc = self._bake_lighting_post_transform(vertices, world_vertices, faces, BASE_COLOR)

            # 退回 CPU Baking (shading=None)，完美保留硬邊銳利感
            mesh_visual = scene.visuals.Mesh(
                vertices=new_verts,
                faces=new_faces,
                vertex_colors=vc,
                shading=None, 
                parent=parent_node 
            )
            mesh_visual.set_gl_state('opaque', depth_test=True, cull_face=True)
            mesh_visual.transform = scene.transforms.MatrixTransform()
            self.actors.append(mesh_visual)

        self._make_ee_frame(self.actors[-1])
        self.update_joints([0, 0, 0, 0, 0, 0])

    def update_joints(self, joint_angles, tcp_mat=None):
        
        matrices = kinematics.forward_kinematics_all(joint_angles)
        
        for i, actor in enumerate(self.actors):
            if i < len(matrices):
                actor.transform.matrix = matrices[i].T.astype(np.float32)
                
        if tcp_mat is not None and hasattr(self, 'tcp_node'):
            from vispy import scene 
            if not isinstance(self.tcp_node.transform, scene.transforms.MatrixTransform):
                self.tcp_node.transform = scene.transforms.MatrixTransform()
            self.tcp_node.transform.matrix = tcp_mat.T.astype(np.float32)
                
        self.canvas.update()

    def set_gizmo_mode(self, mode):
        self._gizmo_mode = mode
        self._update_gizmo_visuals()

    def _update_gizmo_visuals(self):
        # 1. 軸向控制項依然由狀態機管理
        is_trans = (self._gizmo_mode == 'translate')
        is_rot = (self._gizmo_mode == 'rotate')

        # 2. 圓球控制項改由獨立開關 _show_drag_sphere 管理，不再受 _gizmo_mode 影響！
        show_sphere = getattr(self, '_show_drag_sphere', False)
        is_dragging_free = getattr(self, '_is_dragging_free', False)

        if show_sphere:
            self.dragger_visual.visible = not is_dragging_free
            self.dragger_visual_active.visible = is_dragging_free
        else:
            self.dragger_visual.visible = False
            self.dragger_visual_active.visible = False

        # 更新軸向元件顯示
        for _, cone, _ in self.gizmo_cones: cone.visible = is_trans
        for ring in self.gizmo_rings: ring.visible = is_rot
        for _, cone, _ in self.gizmo_ring_cones: cone.visible = is_rot
        self.canvas.update()

    def on_mouse_press(self, event):
        if event.button != 1: return
        
        tr = self.dragger_visual.get_transform('visual', 'document')
        origin_doc = tr.map([0,0,0])
        if origin_doc[3] == 0: return
        origin_2d = origin_doc[:2] / origin_doc[3]
        
        dist_center = np.linalg.norm(event.pos - origin_2d)
        hit = False

        if self._gizmo_mode == 'free':
            if dist_center < 30:
                self._active_drag_axis = 'free'
                self._is_dragging_free = True
                self._drag_z = origin_doc[2] / origin_doc[3]
                self.view.camera.interactive = False
                self._update_gizmo_visuals()
                event.handled = True
                hit = True

        elif self._gizmo_mode in ['translate', 'rotate']:
            min_dist_ax = 999
            best_ax = None
            target_list = self.gizmo_cones if self._gizmo_mode == 'translate' else self.gizmo_ring_cones
            for ax, _, local_pos in target_list:
                tip_doc = tr.map(local_pos)
                tip_2d = tip_doc[:2] / tip_doc[3]
                dist = np.linalg.norm(event.pos - tip_2d)
                if dist < min_dist_ax:
                    min_dist_ax, best_ax = dist, ax

            if min_dist_ax < 35 and best_ax:
                self._active_drag_axis = ('t' if self._gizmo_mode == 'translate' else 'r', best_ax)
                self._last_mouse_pos = event.pos
                self.view.camera.interactive = False
                event.handled = True
                hit = True

        if not hit:
            self._bg_pressed = True
            self._bg_press_pos = event.pos

    def on_mouse_move(self, event):
        if not self._active_drag_axis: return

        if self._active_drag_axis == 'free':
            if hasattr(self, 'drag_callback') and self.drag_callback:
                tr_inv = self.floor_grid.get_transform('document', 'visual')
                world_pos = tr_inv.map([event.pos[0], event.pos[1], self._drag_z])
                if world_pos[3] != 0:
                    world_xyz = world_pos[:3] / world_pos[3]
                    self.drag_callback(world_xyz)

        elif isinstance(self._active_drag_axis, tuple):
            if hasattr(self, 'axis_drag_callback') and self.axis_drag_callback:
                mode, ax = self._active_drag_axis
                dx = event.pos[0] - self._last_mouse_pos[0]
                dy = event.pos[1] - self._last_mouse_pos[1]
                
                tr = self.dragger_visual.get_transform('visual', 'document')
                origin_2d = tr.map([0,0,0])
                origin_2d = origin_2d[:2] / origin_2d[3]
                
                local_pos = {'x':[0.1,0,0], 'y':[0,0.1,0], 'z':[0,0,0.1]}[ax]
                tip_2d = tr.map(local_pos)
                tip_2d = tip_2d[:2] / tip_2d[3]
                
                vec_2d = tip_2d - origin_2d
                norm = np.linalg.norm(vec_2d)
                if norm > 0: vec_2d /= norm
                
                move_amount = (dx * vec_2d[0] + dy * vec_2d[1])

                origin_doc = tr.map([0,0,0])
                origin_2d = origin_doc[:2] / origin_doc[3]
                
                if mode == 't':
                    local_pos = {'x':[0.1,0,0], 'y':[0,0.1,0], 'z':[0,0,0.1]}[ax]
                    tip_2d = tr.map(local_pos)
                    tip_2d = tip_2d[:2] / tip_2d[3]
                    
                    vec_2d = tip_2d - origin_2d
                    norm = np.linalg.norm(vec_2d)
                    if norm > 0: vec_2d /= norm
                    
                    move_amount = (dx * vec_2d[0] + dy * vec_2d[1])
                    step = move_amount * 0.4 
                    self.axis_drag_callback(ax, step, 'Tool')
                    
                else:
                    v_prev = np.array([self._last_mouse_pos[0] - origin_2d[0], self._last_mouse_pos[1] - origin_2d[1]])
                    v_curr = np.array([event.pos[0] - origin_2d[0], event.pos[1] - origin_2d[1]])
                    
                    radius = np.linalg.norm(v_curr)
                    if radius > 15: 
                        angle_prev = np.arctan2(v_prev[1], v_prev[0])
                        angle_curr = np.arctan2(v_curr[1], v_curr[0])
                        d_angle = angle_curr - angle_prev
                        
                        if d_angle > np.pi: d_angle -= 2*np.pi
                        elif d_angle < -np.pi: d_angle += 2*np.pi
                        
                        local_pos = {'x':[0.1,0,0], 'y':[0,0.1,0], 'z':[0,0,0.1]}[ax]
                        tip_doc = tr.map(local_pos)
                        
                        is_facing_front = (tip_doc[2] / tip_doc[3]) < (origin_doc[2] / origin_doc[3])
                        
                        direction = -1 if is_facing_front else 1
                        
                        step = np.degrees(d_angle) * direction * 0.8 
                        self.axis_drag_callback('r'+ax, step, 'Tool')
                    
                self._last_mouse_pos = event.pos
                
        event.handled = True

    def on_mouse_release(self, event):
        if self._active_drag_axis:
            self._active_drag_axis = None
            self._is_dragging_free = False
            self.view.camera.interactive = True 
            self._update_gizmo_visuals()
            event.handled = True

        elif getattr(self, '_bg_pressed', False):
            dist = np.linalg.norm(event.pos - self._bg_press_pos)
            if dist < 5:
                if hasattr(self, 'cancel_gizmo_callback') and self.cancel_gizmo_callback:
                    self.cancel_gizmo_callback()
            self._bg_pressed = False
    
    def draw_trajectory_preview(self, points):
        # 1. 初始化軌跡線 (如果還沒建立)
        if not hasattr(self, 'path_actor'):
            self.path_actor = scene.visuals.Line(
                color='#00e6b8',
                width=1.5, # 稍微加粗一點點更好看
                antialias=True,
                method='gl',
                parent=self.view.scene
            )
            # 讓軌跡線無條件顯示在所有模型與拖曳球的最上層
            self.path_actor.set_gl_state(depth_test=False, blend=True)
            self.show_trajectory = True 

        # 2. 如果使用者選擇隱藏，或是沒有路徑點，就直接關閉顯示
        if not getattr(self, 'show_trajectory', True) or not points or len(points) < 2:
            if hasattr(self, 'path_actor'):
                self.path_actor.visible = False
            return
            
        # 3. 正常更新並顯示軌跡
        points_array = np.array(points, dtype=np.float32)
        self.path_actor.set_data(pos=points_array)
        self.path_actor.visible = True
            
        if not points or len(points) < 2:
            self.path_actor.set_data(pos=np.zeros((2, 3), dtype=np.float32))
            self.path_actor.visible = False
            return
            
        points_array = np.array(points, dtype=np.float32)
        self.path_actor.set_data(pos=points_array)
        self.path_actor.visible = True
        
    def get_tcp_matrix(self):
        return np.eye(4)