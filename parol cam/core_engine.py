# core_engine.py
import numpy as np
from scipy.spatial.transform import Rotation as R
from core_mesh import MeshManager
from core_math import rdp_closed, rdp_open
from scipy.spatial import cKDTree

class CAMEngine:
    def __init__(self):
        self.mesh_manager = MeshManager()
        self.cam_tcp_matrices = [] 
        self.segments = []
        self.extraction_mode = "planar" 
        self.tcp_align_mode = "minimum_twist" 

    @property
    def workpiece_mesh(self):
        return self.mesh_manager.workpiece_mesh

    def load_stl_mesh(self, filepath):
        return self.mesh_manager.load_stl_mesh(filepath)
        
    def apply_model_transform(self, x, y, z, rx, ry, rz):
        return self.mesh_manager.apply_model_transform(x, y, z, rx, ry, rz)
        
    def get_closest_boundary_points(self, hit_face_idx, hit_pos_world):
        if self.extraction_mode == "planar":
            return self.mesh_manager.get_closest_boundary_points(hit_face_idx, hit_pos_world)
        else:
            return self.mesh_manager.get_closest_3d_feature_loop(hit_pos_world)

    def clear_segments(self): self.segments.clear()
    def delete_segment(self, index):
        if 0 <= index < len(self.segments): self.segments.pop(index)
    def move_segment_up(self, index):
        if index > 0: self.segments[index], self.segments[index-1] = self.segments[index-1], self.segments[index]
    def move_segment_down(self, index):
        if index < len(self.segments) - 1: self.segments[index], self.segments[index+1] = self.segments[index+1], self.segments[index]

    def add_segment(self, hit_face_idx, hit_pos_world, **kwargs):
        if self.workpiece_mesh is None: raise ValueError("請先載入 STL 模型！")
        
        current_mode = self.extraction_mode
        if current_mode == "planar":
            facet_idx = self.mesh_manager.face_to_facet.get(hit_face_idx)
            if facet_idx is None: raise ValueError("目前為「平面」模式，請點擊平坦面。欲抓取倒角請切換為「3D輪廓」。")
            dense_locations = self.mesh_manager.get_closest_boundary_points(hit_face_idx, hit_pos_world)
            rep_normal = self.workpiece_mesh.facets_normal[facet_idx]
            dense_normals = [rep_normal] * len(dense_locations)
        else:
            dense_locations = self.mesh_manager.get_closest_3d_feature_loop(hit_pos_world)
            if dense_locations is None: raise ValueError("附近找不到符合條件的 3D 倒角/特徵邊緣。")
            
            _, vertex_indices = self.workpiece_mesh.kdtree.query(dense_locations)
            dense_normals = []
            current_ref_normal = self.workpiece_mesh.face_normals[hit_face_idx]
            for vi in vertex_indices:
                adj_faces = self.workpiece_mesh.vertex_faces[vi]
                adj_faces = adj_faces[adj_faces != -1]
                if len(adj_faces) == 0:
                    dense_normals.append(current_ref_normal)
                    continue
                face_norms = self.workpiece_mesh.face_normals[adj_faces]
                dots = np.dot(face_norms, current_ref_normal)
                best_face = adj_faces[np.argmax(dots)]
                best_normal = self.workpiece_mesh.face_normals[best_face] 
                dense_normals.append(best_normal)
                current_ref_normal = best_normal  

            avg_normal = np.mean(dense_normals, axis=0)
            norm_val = np.linalg.norm(avg_normal)
            rep_normal = avg_normal / norm_val if norm_val > 1e-6 else np.array([0.0, 0.0, 1.0])

        if dense_locations is None or len(dense_locations) == 0: raise ValueError("找不到對應的邊界。")
        
        # 👑 將所有參數存入該段落專屬的字典中 (若沒有傳入則使用預設值)
        params = {
            'offset_rx': 0.0, 'offset_ry': 0.0, 'offset_rz': 0.0,
            'lead_in_dist': 2.0, 'lead_in_angle': 45.0,
            'overcut_dist': 2.0, 'chordal_error': 0.05,
            'max_step': 5.0, 'loop_mode': 0,
            'align_mode': self.tcp_align_mode
        }
        params.update(kwargs)

        seg = {
            'face_idx': hit_face_idx,
            'hit_pos': hit_pos_world,
            'facet_normal': rep_normal,
            'mode': current_mode,
            'raw_locs': dense_locations,
            'raw_norms': dense_normals,
            'is_composite': False,
            'params': params # 👑 每個特徵擁有自己獨立的靈魂！
        }
        self.segments.append(seg)
        self._process_segment(seg)
        return seg['name']

    # 👑 新增：專門用來更新單一路徑參數的方法
    def update_segment_params(self, index, **new_params):
        if 0 <= index < len(self.segments):
            self.segments[index]['params'].update(new_params)
            self._process_segment(self.segments[index])

    def merge_segments(self, indices, **kwargs):
        indices = sorted(indices)
        if len(indices) < 2: return
        
        pool = []
        for idx in indices:
            pool.append({
                'locs': self.segments[idx]['raw_locs'].copy(),
                'norms': self.segments[idx]['raw_norms'].copy()
            })
            
        chain_locs = list(pool[0]['locs'])
        chain_norms = list(pool[0]['norms'])
        pool.pop(0)
        
        TOLERANCE = 1.0 
        
        while pool:
            head = np.array(chain_locs[0])
            tail = np.array(chain_locs[-1])
            
            matched_idx = -1
            match_type = "" 
            
            for i, cand in enumerate(pool):
                seg_s = cand['locs'][0]
                seg_e = cand['locs'][-1]
                
                if np.linalg.norm(seg_s - tail) < TOLERANCE:
                    matched_idx = i; match_type = "tail-start"; break
                if np.linalg.norm(seg_e - tail) < TOLERANCE:
                    matched_idx = i; match_type = "tail-end"; break
                if np.linalg.norm(seg_e - head) < TOLERANCE:
                    matched_idx = i; match_type = "head-end"; break
                if np.linalg.norm(seg_s - head) < TOLERANCE:
                    matched_idx = i; match_type = "head-start"; break
                    
            if matched_idx == -1:
                raise ValueError("選取的邊線並未首尾相連！請確認所有選取的特徵是連續不斷的。")
                
            cand = pool.pop(matched_idx)
            locs = cand['locs']
            norms = cand['norms']
            
            if match_type == "tail-start":
                chain_locs.extend(locs[1:])
                chain_norms.extend(norms[1:])
            elif match_type == "tail-end":
                chain_locs.extend(locs[::-1][1:])
                chain_norms.extend(norms[::-1][1:])
            elif match_type == "head-end":
                chain_locs = list(locs[:-1]) + chain_locs
                chain_norms = list(norms[:-1]) + chain_norms
            elif match_type == "head-start":
                chain_locs = list(locs[::-1][:-1]) + chain_locs
                chain_norms = list(norms[::-1][:-1]) + chain_norms

        # 繼承陣列首個元素的參數，並蓋上 UI 傳來的新數值
        params = self.segments[indices[0]]['params'].copy()
        params.update(kwargs)

        new_seg = {
            'face_idx': self.segments[indices[0]]['face_idx'],
            'hit_pos': self.segments[indices[0]]['hit_pos'],
            'facet_normal': self.segments[indices[0]]['facet_normal'],
            'mode': 'composite',
            'raw_locs': np.array(chain_locs),
            'raw_norms': np.array(chain_norms),
            'is_composite': True,
            'params': params # 👑 繼承獨立參數
        }
        
        for idx in reversed(indices):
            self.segments.pop(idx)
            
        self.segments.append(new_seg)
        self._process_segment(new_seg)

    def _process_segment(self, seg):
        # 👑 將運算模組改為「讀取自帶參數」，實現徹底解耦！
        p = seg['params']
        offset_rx = p.get('offset_rx', 0.0)
        offset_ry = p.get('offset_ry', 0.0)
        offset_rz = p.get('offset_rz', 0.0)
        lead_in_dist = p.get('lead_in_dist', 2.0)
        lead_in_angle = p.get('lead_in_angle', 45.0)
        overcut_dist = p.get('overcut_dist', 2.0)
        chordal_error = p.get('chordal_error', 0.05)
        max_step = p.get('max_step', 5.0)
        loop_mode = p.get('loop_mode', 0)
        align_mode = p.get('align_mode', "minimum_twist")
        
        def _rotate_about_axis(v, axis, angle_rad):
            axis = axis / np.linalg.norm(axis)
            return (v * np.cos(angle_rad) + np.cross(axis, v) * np.sin(angle_rad) + axis * np.dot(axis, v) * (1 - np.cos(angle_rad)))

        dense_locations = seg['raw_locs']
        dense_normals = seg['raw_norms']

        dist_head_tail = np.linalg.norm(dense_locations[0] - dense_locations[-1])
        auto_closed = bool(len(dense_locations) > 3 and dist_head_tail < 3.0)
        
        if loop_mode == 1: is_closed_loop = True
        elif loop_mode == 2: is_closed_loop = False
        else: is_closed_loop = auto_closed

        if is_closed_loop:
            n0 = dense_normals[0]
            n_end = dense_normals[-1]
            dot_n = np.clip(np.dot(n0, n_end), -1.0, 1.0)
            angle_z = np.arccos(dot_n)
            if angle_z > 1e-6:
                axis_z = np.cross(n_end, n0)
                if np.linalg.norm(axis_z) > 1e-8:
                    axis_z = axis_z / np.linalg.norm(axis_z)
                    for i in range(len(dense_normals)):
                        frac = i / (len(dense_normals) - 1)
                        dense_normals[i] = _rotate_about_axis(dense_normals[i], axis_z, angle_z * frac)
            dense_normals[-1] = dense_normals[0]

        if is_closed_loop: dp_locations = rdp_closed(dense_locations, epsilon=chordal_error)
        else: dp_locations = rdp_open(dense_locations, epsilon=chordal_error)

        tree = cKDTree(dense_locations)
        _, dp_idx = tree.query(dp_locations)
        dp_normals = np.array(dense_normals)[dp_idx]

        if is_closed_loop:
            if len(dp_locations) > 1 and np.linalg.norm(dp_locations[0] - dp_locations[-1]) < 1e-9:
                dp_locations = dp_locations[:-1]
                dp_normals = dp_normals[:-1]
            closed_chain = np.vstack([dp_locations, dp_locations[0:1]])
            closed_normals = np.vstack([dp_normals, dp_normals[0:1]])
        else:
            closed_chain = dp_locations
            closed_normals = dp_normals

        final_locations = []
        final_normals = []
        for i in range(len(closed_chain) - 1):
            p1 = closed_chain[i]; p2 = closed_chain[i + 1]
            n1 = closed_normals[i]; n2 = closed_normals[i + 1]
            dist = np.linalg.norm(p2 - p1)
            if dist > max_step:
                num_sub = int(np.ceil(dist / max_step))
                final_locations.extend(np.linspace(p1, p2, num_sub, endpoint=False))
                final_normals.extend(np.linspace(n1, n2, num_sub, endpoint=False))
            else:
                final_locations.append(p1)
                final_normals.append(n1)
                
        final_locations.append(closed_chain[-1])
        final_normals.append(closed_normals[-1])
        locations = np.array(final_locations)
        face_normals = np.array(final_normals)
        
        norms_mag = np.linalg.norm(face_normals, axis=1, keepdims=True)
        face_normals = face_normals / np.where(norms_mag == 0, 1e-8, norms_mag)

        num_points = len(locations)
        R_offset = R.from_euler('xyz', [offset_rx, offset_ry, offset_rz], degrees=True).as_matrix()
        raw_z_vecs = -face_normals
        
        smooth_z = []
        window = max(2, num_points // 8) 
        for i in range(num_points):
            avg_z = np.zeros(3)
            for j in range(i - window, i + window + 1):
                idx = j % (num_points - 1) if (is_closed_loop and num_points > 1) else np.clip(j, 0, num_points - 1)
                avg_z += raw_z_vecs[idx]
            norm_z = np.linalg.norm(avg_z)
            smooth_z.append(avg_z / norm_z if norm_z > 1e-6 else raw_z_vecs[i])
        z_vecs = np.array(smooth_z)

        if align_mode == "tangent":
            x_vecs, y_vecs = [], []
            for i in range(num_points):
                z_vec = z_vecs[i]
                
                if is_closed_loop:
                    if i == 0 or i == num_points - 1:
                        fwd_vec = locations[1] - locations[-2]
                    else:
                        fwd_vec = locations[i + 1] - locations[i - 1]
                else:
                    if i == 0:
                        fwd_vec = locations[1] - locations[0]
                    elif i == num_points - 1:
                        fwd_vec = locations[-1] - locations[-2]
                    else:
                        fwd_vec = locations[i + 1] - locations[i - 1]
                        
                norm_fwd = np.linalg.norm(fwd_vec)
                if norm_fwd < 1e-6: fwd_vec = np.array([1.0, 0.0, 0.0])
                else: fwd_vec = fwd_vec / norm_fwd
                
                y_vec = np.cross(z_vec, fwd_vec)
                if np.linalg.norm(y_vec) < 1e-6:
                    ref_vec = np.array([0.0, 1.0, 0.0]) if abs(z_vec[1]) < 0.99 else np.array([1.0, 0.0, 0.0])
                    y_vec = np.cross(z_vec, ref_vec)
                y_vec = y_vec / np.linalg.norm(y_vec)
                
                x_vec = np.cross(y_vec, z_vec)
                x_vecs.append(x_vec / np.linalg.norm(x_vec))
                y_vecs.append(y_vec)
        else:
            raw_x_vecs = []
            prev_x = None
            for i in range(num_points):
                z_vec = z_vecs[i]
                if i == 0:
                    ref_vec = np.array([1.0, 0.0, 0.0]) if np.abs(np.dot(z_vec, np.array([0.0, 1.0, 0.0]))) > 0.99 else np.array([0.0, 1.0, 0.0])
                    x_vec = np.cross(ref_vec, z_vec)
                else:
                    x_vec = prev_x - np.dot(prev_x, z_vec) * z_vec
                    if np.linalg.norm(x_vec) < 1e-6:
                        ref_vec = np.array([1.0, 0.0, 0.0]) if np.abs(z_vec[1]) < 0.99 else np.array([0.0, 1.0, 0.0])
                        x_vec = np.cross(ref_vec, z_vec)
                x_vec = x_vec / np.linalg.norm(x_vec)
                raw_x_vecs.append(x_vec)
                prev_x = x_vec

            if is_closed_loop and num_points > 2:
                twist_error = np.arctan2(np.dot(np.cross(z_vecs[0], raw_x_vecs[0]), raw_x_vecs[-1]), np.clip(np.dot(raw_x_vecs[0], raw_x_vecs[-1]), -1.0, 1.0))
                x_vecs, y_vecs = [], []
                for i in range(num_points):
                    x_corr = _rotate_about_axis(raw_x_vecs[i], z_vecs[i], -twist_error * (i / (num_points - 1)))
                    x_vecs.append(x_corr / np.linalg.norm(x_corr))
                    y_vecs.append(np.cross(z_vecs[i], x_corr) / np.linalg.norm(np.cross(z_vecs[i], x_corr)))
            else:
                x_vecs = raw_x_vecs
                y_vecs = [np.cross(z_vecs[i], x_vecs[i]) / np.linalg.norm(np.cross(z_vecs[i], x_vecs[i])) for i in range(num_points)]

        x_vecs, y_vecs = np.array(x_vecs), np.array(y_vecs)

        if is_closed_loop and overcut_dist > 0.0:
            accum = 0.0
            oc_locs, oc_zs, oc_xs, oc_ys = [], [], [], []
            for i in range(1, num_points):
                accum += np.linalg.norm(locations[i] - locations[i-1])
                oc_locs.append(locations[i]); oc_zs.append(z_vecs[i]); oc_xs.append(x_vecs[i]); oc_ys.append(y_vecs[i])
                if accum >= overcut_dist: break
            if oc_locs:
                locations = np.vstack([locations, np.array(oc_locs)])
                z_vecs = np.vstack([z_vecs, np.array(oc_zs)])
                x_vecs = np.vstack([x_vecs, np.array(oc_xs)])
                y_vecs = np.vstack([y_vecs, np.array(oc_ys)])
                num_points = len(locations)

        if lead_in_dist > 0.0:
            fwd = locations[1] - locations[0]
            if np.linalg.norm(fwd) > 1e-6:
                tangent = fwd / np.linalg.norm(fwd)
                dir_away = -tangent * np.cos(np.radians(lead_in_angle)) + np.cross(z_vecs[0], -tangent) * np.sin(np.radians(lead_in_angle))
                dir_away = dir_away / np.linalg.norm(dir_away)
                locations = np.insert(locations, 0, locations[0] + dir_away * lead_in_dist, axis=0)
                z_vecs = np.insert(z_vecs, 0, z_vecs[0], axis=0)
                x_vecs = np.insert(x_vecs, 0, x_vecs[0], axis=0)
                y_vecs = np.insert(y_vecs, 0, y_vecs[0], axis=0)
                num_points = len(locations)

        matrices = []
        for i in range(num_points):
            T_base = np.eye(4)
            T_base[:3, :3] = np.column_stack((x_vecs[i], y_vecs[i], z_vecs[i]))
            T_base[:3, 3] = locations[i]
            T_tcp = T_base.copy()
            T_tcp[:3, :3] = T_base[:3, :3] @ R_offset
            matrices.append(T_tcp)

        seg['locations'] = locations
        seg['matrices'] = matrices
        seg['is_closed_loop'] = is_closed_loop
        seg['outward_normals'] = -z_vecs 
        
        prefix = "群組" if seg.get('is_composite') else ('封閉' if is_closed_loop else '開放')
        seg['name'] = f"{prefix} {'平面' if seg['mode']=='planar' else '3D'} 特徵 {self.segments.index(seg) + 1} ({num_points}點)"

    def get_linked_path(self, safe_z=20.0):
        if not self.segments: return []
        tagged_path = []
        for i, seg in enumerate(self.segments):
            mats = seg['matrices']
            locs = seg['locations']
            
            # 擷取路徑起點與終點的「局部向外法向量」
            n_start = seg['outward_normals'][0]
            n_end = seg['outward_normals'][-1]

            if i == 0:
                approach = mats[0].copy()
                approach[:3, 3] += n_start * safe_z
                tagged_path.append({'type': 'PTP_APPROACH', 'matrix': approach, 'location': approach[:3, 3]})
            else:
                prev_mat = self.segments[i-1]['matrices'][-1]
                n_prev_end = self.segments[i-1]['outward_normals'][-1]
                
                retract = prev_mat.copy()
                retract[:3, 3] += n_prev_end * safe_z
                tagged_path.append({'type': 'RETRACT', 'matrix': retract, 'location': retract[:3, 3]})
                
                approach = mats[0].copy()
                approach[:3, 3] += n_start * safe_z
                tagged_path.append({'type': 'APPROACH', 'matrix': approach, 'location': approach[:3, 3]})
                
            plunge = mats[0].copy()
            tagged_path.append({'type': 'PLUNGE', 'matrix': plunge, 'location': plunge[:3, 3]})
            tagged_path.append({'type': 'CUT', 'matrices': mats, 'locations': locs})
            
            if i == len(self.segments) - 1:
                retract = mats[-1].copy()
                retract[:3, 3] += n_end * safe_z
                tagged_path.append({'type': 'RETRACT', 'matrix': retract, 'location': retract[:3, 3]})
                
        return tagged_path