# picker.py
import numpy as np

class MeshPicker:
    def __init__(self, cam_engine, robot_view, on_hit_callback, on_hover_callback):
        self.cam_engine = cam_engine
        self.robot_view = robot_view
        self.on_hit_callback = on_hit_callback
        self.on_hover_callback = on_hover_callback 
        
        self.robot_view.raycast_callback = self.handle_raycast
        self.robot_view.raycast_hover_callback = self.handle_hover
        
    def toggle_picking_mode(self, enabled):
        self.robot_view._picking_mode = enabled
        if not enabled and self.on_hover_callback:
            self.on_hover_callback(None) 
            
    def _do_raycast(self, ray_origin, ray_direction):
        if self.cam_engine.workpiece_mesh is None: return None, []
        
        ray_origin_mm = np.array(ray_origin) * 1000.0 
        ray_dir_norm = np.array(ray_direction)
        ray_dir_norm /= np.linalg.norm(ray_dir_norm)
        
        locations, _, index_tri = self.cam_engine.workpiece_mesh.ray.intersects_location(
            ray_origins=[ray_origin_mm],
            ray_directions=[ray_dir_norm],
            multiple_hits=False
        )
        # 👑 這次我們連 locations (精確擊中點) 一起回傳
        return locations, index_tri

    def handle_raycast(self, ray_origin, ray_direction):
        locations, index_tri = self._do_raycast(ray_origin, ray_direction)
        if len(index_tri) > 0 and self.on_hit_callback:
            hit_pos = locations[0]
            print(f"[Picker] 命中目標！打中 Face ID: {index_tri[0]}，座標: {hit_pos}")
            # 👑 傳送 Hit Face ID 與 3D 世界座標給 GUI
            self.on_hit_callback(index_tri[0], hit_pos) 

    def handle_hover(self, ray_origin, ray_direction):
        locations, index_tri = self._do_raycast(ray_origin, ray_direction)
        if len(index_tri) > 0:
            hit_pos = locations[0]
            # 👑 傳入座標，讓引擎判斷哪一圈邊界離滑鼠最近
            boundary_pts = self.cam_engine.get_closest_boundary_points(index_tri[0], hit_pos)
            if boundary_pts is not None and self.on_hover_callback:
                self.on_hover_callback(boundary_pts)
        else:
            if self.on_hover_callback:
                self.on_hover_callback(None)