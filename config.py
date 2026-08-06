# config.py
import os
import numpy as np
import trimesh
from PySide6.QtWidgets import QWidget, QVBoxLayout
from vispy import scene

import kinematics

# =========================================================
# [1] 機器人運動學與硬體極限參數
# =========================================================
MAX_JOINT_SPEEDS = np.array([175.78, 56.25, 62.17, 281.25, 281.25, 112.50])     
MAX_JOINT_ACCELS = np.array([703.13, 225.00, 248.69, 1125.00, 1125.00, 450.00])   
MAX_JOINT_JERKS = MAX_JOINT_ACCELS * 10.0

MAX_LIN_SPEED = 200.0    
MAX_LIN_ACCEL = 500.0    
MAX_LIN_JERK = MAX_LIN_ACCEL * 10.0    

MAX_ROT_SPEED = 100.0     
MAX_ROT_ACCEL = 200.0     
MAX_ROT_JERK = MAX_ROT_ACCEL * 10.0     

GEAR_RATIOS = np.array([6.4, 20.0, 18.095, 4.0, 4.0, -10.0])
STEPS_PER_DEG = (6400.0 * GEAR_RATIOS) / 360.0

MCU_JOG_STEPS = np.array([4800.0, 6800.0, 6800.0, 6000.0, 6000.0, 8000.0])
JOG_SPEEDS_DEG = np.abs(MCU_JOG_STEPS / STEPS_PER_DEG)

MAX_PULSE_FREQ = 75000

JOINT_LIMITS = [
    (-75, 255), (-50, 70), (-70, 70), (-95, 185), (-110, 110), (-120, 210)   
]
SCALE_FACTOR = 1.0
IK_POS_TOLERANCE = 0.1
DEBUG_IK_PROFILER = False  


BASE_MESH_OFFSET = {'xyz': [0, 0, 0], 'rpy': [1.5708, 0, -1.5708]}
URDF_PARAMS = [
    {'xyz': [0, 0.1105, 0],        'rpy': [3.1416, 0, 0],       'axis': 'y'}, 
    {'xyz': [-0.0234, 0, 0],       'rpy': [1.5708, -1.5708, 0], 'axis': 'x', 'invert': True}, 
    {'xyz': [0, 0, 0.18],          'rpy': [0, 0, 3.1416],       'axis': 'x'}, 
    {'xyz': [0, -0.0712, 0.04344], 'rpy': [3.1416, 0, -1.5708], 'axis': 'x', 'invert': True}, 
    {'xyz': [0.1053, 0, 0],        'rpy': [1.5708, 0, 1.5708],  'axis': 'x', 'invert': True}, 
    {'xyz': [0, 0, 0.0505],        'rpy': [3.1416, 0, 0],       'axis': 'z',}  
]

# =========================================================
# [2] 數位孿生材質與模型索引庫
# =========================================================
MATERIAL_COLORS = {
    'default': (0.90, 0.90, 0.90), 
    'motor':   (0.18, 0.18, 0.18), 
    'screws':  (0.78, 0.80, 0.84), 
    'gear':    (0.85, 0.78, 0.58),  
    'cage':    (0.65, 0.65, 0.65)  
}

STL_FILES = [
    'base_link.STL', 
    'Link1_main.STL', 'Link1_motor.STL', 'Link1_screws.STL','Link1_gear.STL', 'Link1_cage.STL',
    'Link2_main.STL', 'Link2_motor.STL', 'Link2_screws.STL','Link2_gear.STL', 'Link2_cage.STL',
    'Link3_main.STL', 'Link3_motor.STL', 'Link3_screws.STL',
    'Link4_main.STL', 'Link4_motor.STL', 'Link4_screws.STL',
    'Link5_main.STL', 'Link5_motor.STL', 
    'Link6_main.STL', 'link6_tool.STL'
]

MESH_JOINT_INDICES = [
    0,             
    1, 1, 1, 1, 1, 
    2, 2, 2, 2, 2, 
    3, 3, 3,       
    4, 4, 4,       
    5, 5,          
    6, 6           
]


# =========================================================
# [3] 3D 視覺渲染主體 (Vispy Scene)
# =========================================================
class Robot3DView(QWidget):
    EE_AXIS_LEN: float = 0.1

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # --- 1. 初始化狀態變數 ---
        self.base_manager = None
        self.ee_frame_visuals = []
        self.tcp_node = None
        self.base_visual_groups = []
        self.path_actor = None
        self.show_trajectory = True
        self.BASE_AXIS_LEN = 0.12
        
        # Callback 綁定 (純視覺互動回呼)
        self.drag_callback = None
        self.axis_drag_callback = None
        self.cancel_gizmo_callback = None 
        self.raycast_callback = None
        self.raycast_hover_callback = None
        
        # 狀態控制變數
        self._picking_mode = False
        self._show_drag_sphere = False
        self._gizmo_mode = 'free'   
        self._active_drag_axis = None    
        self._is_dragging_free = False 
        self._drag_z = 0.0               
        
        self._drag_start_pos = None      # 紀錄點擊初始位置，用於死區判定
        self._drag_engaged = False       # 標記是否已突破死區
        self._last_mouse_pos = None
        self._bg_pressed = False
        self._bg_press_pos = None

        # --- 2. 建立 Canvas 與 Camera ---
        self.canvas = scene.SceneCanvas(
            keys='interactive', show=False, bgcolor='#262626',
            config={'depth_size': 24, 'samples': 4}
        )
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = scene.cameras.TurntableCamera(
            fov=15, distance=2.5, elevation=30, azimuth=-225, center=(0, 0, 0.15)
        )
        layout.addWidget(self.canvas.native)

        # --- 3. 場景燈光與環境建構 ---
        self.LIGHTS = [
            {'dir': np.array([-1.0,  1.0,  2.0]), 'strength': 0.30, 'two_sided': False},
            {'dir': np.array([ 1.0, -1.0,  1.0]), 'strength': 0.20, 'two_sided': True},
            {'dir': np.array([ 0.0,  0.0, -1.0]), 'strength': 0.10, 'two_sided': True},
        ]
        self.AMBIENT = 0.55 

        scene.visuals.XYZAxis(parent=self.view.scene)
        self._make_floor()
        self._make_floor_grid(size=0.60, step=0.10)

        # --- 4. 載入實體模型與事件綁定 ---
        self.actors = []
        self._assemble_statically()
        
        self.canvas.events.mouse_press.connect(self.on_mouse_press)
        self.canvas.events.mouse_move.connect(self.on_mouse_move)
        self.canvas.events.mouse_release.connect(self.on_mouse_release)
        
        self._update_gizmo_visuals()

    # =========================================================
    # 視覺元件建構與更新 (Meshes & Gizmos)
    # =========================================================
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
            pos=pos, color=(0.45, 0.45, 0.45, 1.0), 
            connect='segments', width=1, antialias=True, parent=self.view.scene
        )
        grid_tf = scene.transforms.MatrixTransform()
        grid_tf.translate((0, 0, 0.0))
        self.floor_grid.transform = grid_tf

    def set_base_manager(self, base_manager):
        if self.base_manager is not None:
            try:
                self.base_manager.data_changed.disconnect(self._on_base_changed)
            except (TypeError, RuntimeError): pass

        self.base_manager = base_manager

        if self.base_manager is not None:
            self.base_manager.data_changed.connect(self._on_base_changed)
            if self.actors: 
                self._make_base_frame(self.view.scene)

    def _on_base_changed(self):
        if self.actors:
            self._make_base_frame(self.view.scene)
    
    def _make_ee_frame(self, parent_visual):
        for v in self.ee_frame_visuals:
            v.parent = None
        self.ee_frame_visuals.clear()

        if self.tcp_node:
            self.tcp_node.parent = None
        self.tcp_node = scene.Node(parent=parent_visual)

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

        self.gizmo_rings = []
        self.gizmo_ring_cones = []
        t = np.linspace(0, 2*np.pi, 60)
        c, s, zero = np.cos(t), np.sin(t), np.zeros_like(t)
        
        ring_offsets = {
            'x': [0.08, 0, 0], 'y': [0, 0.08, 0], 'z': [0, 0, 0.08] 
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

    def _make_base_frame(self, parent_visual):
        if self.base_manager is None:
            return   

        if len(self.base_visual_groups) != len(self.base_manager.bases):
            for group in self.base_visual_groups:
                group['node'].parent = None
            self.base_visual_groups.clear()

            for idx in range(len(self.base_manager.bases)):
                node = scene.Node(parent=parent_visual)
                lines = []
                for _ in range(3):
                    lines.append(scene.visuals.Line(connect='strip', antialias=True, parent=node))
                marker = scene.visuals.Markers(parent=node)
                label = scene.visuals.Text("", font_size=9, parent=node, anchor_x='left', anchor_y='bottom')
                label.pos = (0.005, 0.005, 0.005)
                
                self.base_visual_groups.append({
                    'node': node, 'lines': lines, 'marker': marker, 'text': label
                })

        base_colors = [(0.95, 0.20, 0.20, 0.7), (0.20, 0.85, 0.20, 0.7), (0.20, 0.40, 0.95, 0.7)]

        for idx, base in enumerate(self.base_manager.bases):
            group = self.base_visual_groups[idx]
            node = group['node']

            if not base.get("in_box", True):
                node.visible = False
                continue
            
            node.visible = True
            T = self.base_manager.get_matrix(idx)
            tf = scene.transforms.MatrixTransform()
            tf.matrix = T.T
            node.transform = tf

            current_name = base.get("name", f"Base {idx}")
            if group['text'].text == current_name:
                continue

            base_end_pts = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            for i, line in enumerate(group['lines']):
                pts = np.array([[0.0, 0.0, 0.0], base_end_pts[i]], dtype=np.float32)
                line.set_data(pos=pts, color=base_colors[i], width=1.5)
                ltf = scene.transforms.MatrixTransform()
                ltf.scale([self.BASE_AXIS_LEN] * 3)
                line.transform = ltf

            group['marker'].set_data(
                pos=np.array([[0, 0, 0]], dtype=np.float32),
                face_color=(0.7, 0.7, 0.7, 0.6), size=6
            )
            group['text'].text = current_name
            group['text'].color = (1, 1, 1, 0.7)

    # =========================================================
    # STL 模型光影與載入
    # =========================================================
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

        parent_node = self.view.scene
        zero_matrices = kinematics.forward_kinematics_all([0.0]*6)

        for i, filename in enumerate(STL_FILES):
            full_path = os.path.join(base_path, filename)
            try:
                mesh_data = trimesh.load(full_path, force='mesh')
                vertices = np.array(mesh_data.vertices, dtype=np.float32) * 0.001
                faces    = np.array(mesh_data.faces,    dtype=np.uint32)
                
                color_key = 'default'
                for key in MATERIAL_COLORS.keys():
                    if key in filename.lower():
                        color_key = key
                        break
                        
                part_color = MATERIAL_COLORS[color_key]
                
            except Exception as e:
                print(f"[3D Render] 無法載入 {filename}: {e}")
                box      = trimesh.creation.box(extents=(0.05, 0.05, 0.05))
                vertices = np.array(box.vertices, dtype=np.float32)
                faces    = np.array(box.faces,    dtype=np.uint32)
                part_color = MATERIAL_COLORS['default']

            mat_idx = MESH_JOINT_INDICES[i]
            M0 = zero_matrices[mat_idx] if mat_idx < len(zero_matrices) else np.eye(4)
            
            ones = np.ones((len(vertices), 1), dtype=np.float32)
            world_vertices = (M0 @ np.hstack([vertices, ones]).T).T[:, :3]

            new_verts, new_faces, vc = self._bake_lighting_post_transform(
                vertices, world_vertices, faces, part_color
            )

            mesh_visual = scene.visuals.Mesh(
                vertices=new_verts, faces=new_faces, vertex_colors=vc,
                shading=None, parent=parent_node 
            )
            mesh_visual.set_gl_state('opaque', depth_test=True, cull_face=True)
            mesh_visual.transform = scene.transforms.MatrixTransform()
            self.actors.append(mesh_visual)

        self._make_ee_frame(self.actors[-1])
        self._make_base_frame(self.view.scene)
        self.update_joints([0, 0, 0, 0, 0, 0])

    def update_joints(self, joint_angles, tcp_mat=None):
        matrices = kinematics.forward_kinematics_all(joint_angles)
        
        for i, actor in enumerate(self.actors):
            mat_idx = MESH_JOINT_INDICES[i]
            if mat_idx < len(matrices):
                actor.transform.matrix = matrices[mat_idx].T.astype(np.float32)
                
        if tcp_mat is not None and self.tcp_node:
            if not isinstance(self.tcp_node.transform, scene.transforms.MatrixTransform):
                self.tcp_node.transform = scene.transforms.MatrixTransform()
            self.tcp_node.transform.matrix = tcp_mat.T.astype(np.float32)
                
        self.canvas.update()

    def set_gizmo_mode(self, mode):
        self._gizmo_mode = mode
        self._update_gizmo_visuals()

    def _update_gizmo_visuals(self):
        is_trans = (self._gizmo_mode == 'translate')
        is_rot = (self._gizmo_mode == 'rotate')

        if self._show_drag_sphere:
            self.dragger_visual.visible = not self._is_dragging_free
            self.dragger_visual_active.visible = self._is_dragging_free
        else:
            self.dragger_visual.visible = False
            self.dragger_visual_active.visible = False

        for _, cone, _ in self.gizmo_cones: cone.visible = is_trans
        for ring in self.gizmo_rings: ring.visible = is_rot
        for _, cone, _ in self.gizmo_ring_cones: cone.visible = is_rot
        self.canvas.update()

    def draw_trajectory_preview(self, points):
        if self.path_actor is None:
            self.path_actor = scene.visuals.Line(
                color='#00e6b8', width=1.5, antialias=True, method='gl', parent=self.view.scene
            )
            self.path_actor.set_gl_state(depth_test=False, blend=True)

        if not self.show_trajectory or not points or len(points) < 2:
            self.path_actor.visible = False
            return
            
        points_array = np.array(points, dtype=np.float32)
        self.path_actor.set_data(pos=points_array)
        self.path_actor.visible = True
        
    def get_tcp_matrix(self):
        return np.eye(4)

    # =========================================================
    # 滑鼠事件與 3D 空間互動 (Gizmo & Raycasting)
    # =========================================================
    def _get_ray_from_mouse(self, pos):
        """ 共用函式：將滑鼠 2D 座標轉換為 3D 世界射線 """
        trans = self.floor_grid.get_transform('canvas', 'visual')
        p_near = trans.map([pos[0], pos[1], -1.0])
        p_far  = trans.map([pos[0], pos[1],  1.0])
        p_near = p_near[:3] / p_near[3]
        p_far  = p_far[:3] / p_far[3]
        ray_dir = p_far - p_near
        ray_dir /= np.linalg.norm(ray_dir)
        return p_near, ray_dir

    def on_mouse_press(self, event):
        if event.button != 1: return
        
        # --- 處理 3D 射線擷取 ---
        if self._picking_mode and self.raycast_callback:
            p_near, ray_dir = self._get_ray_from_mouse(event.pos)
            self.raycast_callback(p_near, ray_dir)
            event.handled = True 
            return

        # --- 處理 Gizmo 拖曳判定 ---
        tr = self.dragger_visual.get_transform('visual', 'document')
        origin_doc = tr.map([0,0,0])
        if origin_doc[3] == 0: return
        origin_2d = origin_doc[:2] / origin_doc[3]
        
        dist_center = np.linalg.norm(event.pos - origin_2d)
        hit = False

        if self._gizmo_mode == 'free' and self._show_drag_sphere: 
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

        if hit:
            self._drag_start_pos = event.pos
            self._drag_engaged = False
        else:
            self._bg_pressed = True
            self._bg_press_pos = event.pos

    def on_mouse_move(self, event):
        # --- 處理 3D 射線 Hover ---
        if getattr(self, '_picking_mode', False) and getattr(self, 'raycast_hover_callback', None):
            p_near, ray_dir = self._get_ray_from_mouse(event.pos)
            self.raycast_hover_callback(p_near, ray_dir)

        if not self._active_drag_axis: return

        # 1. 拖曳死區 (解決點擊瞬移)
        if not self._drag_engaged:
            dist = np.linalg.norm(event.pos - self._drag_start_pos)
            if dist < 3.0: 
                return 
                
            self._drag_engaged = True
            self._last_mouse_pos = event.pos 
            return
            
        dx = event.pos[0] - self._last_mouse_pos[0]
        dy = event.pos[1] - self._last_mouse_pos[1]

        # --- 處理自由拖曳 (Free Drag) ---
        if self._active_drag_axis == 'free':
            if getattr(self, 'drag_callback', None):
                tr_inv = self.floor_grid.get_transform('document', 'visual')
                world_pos = tr_inv.map([event.pos[0], event.pos[1], self._drag_z])
                if world_pos[3] != 0:
                    world_xyz = world_pos[:3] / world_pos[3]
                    self.drag_callback(world_xyz)
            self._last_mouse_pos = event.pos

        # --- 處理單軸平移或旋轉 (Axis Drag) ---
        elif isinstance(self._active_drag_axis, tuple):
            if getattr(self, 'axis_drag_callback', None):
                mode, ax = self._active_drag_axis
                
                tr = self.dragger_visual.get_transform('visual', 'document')
                origin_doc = tr.map([0,0,0])
                origin_2d = origin_doc[:2] / origin_doc[3]
                
                if mode == 't':
                    local_pos = {'x':[0.1,0,0], 'y':[0,0.1,0], 'z':[0,0,0.1]}[ax]
                    tip_2d = tr.map(local_pos)
                    tip_2d = tip_2d[:2] / tip_2d[3]
                    
                    vec_2d = tip_2d - origin_2d
                    pixel_length = np.linalg.norm(vec_2d) 
                    
                    if pixel_length > 0:
                        vec_2d /= pixel_length
                        move_in_pixels = (dx * vec_2d[0] + dy * vec_2d[1])
                        
                        step = (move_in_pixels / pixel_length) * 100.0 
                        
                        if abs(step) >= 0.05:
                            self.axis_drag_callback(ax, step, 'Tool')
                            self._last_mouse_pos = event.pos 
                    
                else: 
                    # 1. 取得旋轉把手 (Cone) 的局部座標
                    local_pos = {'x': [0, 0.06, 0], 'y': [0, 0, 0.06], 'z': [0.06, 0, 0]}[ax]
                    # 2. 根據右手定則，算出把手在正向旋轉時的 3D 切線向量
                    local_tangent = {'x': [0, 0, 0.1], 'y': [0.1, 0, 0], 'z': [0, 0.1, 0]}[ax]
                    
                    cone_doc = tr.map(local_pos)
                    tangent_pt = [local_pos[0] + local_tangent[0], 
                                  local_pos[1] + local_tangent[1], 
                                  local_pos[2] + local_tangent[2]]
                    tangent_doc = tr.map(tangent_pt)
                    
                    cone_2d = cone_doc[:2] / cone_doc[3]
                    tangent_2d = tangent_doc[:2] / tangent_doc[3]
                    
                    # 3. 取得 2D 螢幕上的切線投影
                    vec_2d = tangent_2d - cone_2d
                    pixel_length = np.linalg.norm(vec_2d)
                    
                    if pixel_length > 1.0: 
                        vec_2d /= pixel_length
                        # 4. 將滑鼠位移 1:1 投影到這條切線上
                        move_in_pixels = (dx * vec_2d[0] + dy * vec_2d[1])
                        
                        # 5. 幾何轉換
                        step = (move_in_pixels / pixel_length) * 90.0
                        
                        if abs(step) >= 0.1:
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

        elif self._bg_pressed:
            dist = np.linalg.norm(event.pos - self._bg_press_pos)
            if dist < 5:
                if self.cancel_gizmo_callback:
                    self.cancel_gizmo_callback()
            self._bg_pressed = False