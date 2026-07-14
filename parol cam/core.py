# core.py
import numpy as np
import trimesh
import networkx as nx
from scipy.spatial.transform import Rotation as R

from cam_settings import DEFAULT_CHORDAL_ERROR

class CAMEngine:
    def __init__(self):
        self.original_mesh = None   
        self.workpiece_mesh = None  
        self.toolpath_points = []
        self.cam_tcp_matrices = []
        
        self.face_to_facet = {}
        self.facet_boundary_indices = {}

    def load_stl_mesh(self, filepath):
        mesh = trimesh.load(filepath, force='mesh')
        if isinstance(mesh, trimesh.Scene):
            mesh = mesh.dump(concatenate=True)
            
        mesh.fix_normals()
        self.original_mesh = mesh
        self.workpiece_mesh = self.original_mesh.copy()
        
        self._build_topology_cache()
        return self.workpiece_mesh

    def _build_topology_cache(self):
        """預運算：找出所有平坦面並快取【所有】邊界迴路"""
        self.face_to_facet = {}
        self.facet_boundaries = {} # 👑 改為存放一個 list，裡面包含多條迴路
        
        print("[Core] 正在分析 STL 幾何拓樸與平面特徵...")
        facets = self.original_mesh.facets
        for facet_idx, facet_faces in enumerate(facets):
            for face_idx in facet_faces:
                self.face_to_facet[face_idx] = facet_idx
            
            target_faces = self.original_mesh.faces[facet_faces]
            edges = trimesh.geometry.faces_to_edges(target_faces)
            edge_counts = {}
            for edge in edges:
                e = tuple(sorted(edge))
                edge_counts[e] = edge_counts.get(e, 0) + 1
            boundary_edges = [e for e, count in edge_counts.items() if count == 1]
            
            if not boundary_edges: continue
            G = nx.Graph()
            G.add_edges_from(boundary_edges)
            components = list(nx.connected_components(G))
            
            # 👑 迴圈取出該平面的「所有」封閉輪廓 (外框與所有內孔洞)
            loops = []
            for comp in components:
                sub_G = G.subgraph(comp)
                try:
                    cycle = nx.find_cycle(sub_G)
                    ordered_vertices = [edge[0] for edge in cycle]
                    ordered_vertices.append(cycle[-1][1])
                except nx.NetworkXNoCycle:
                    ordered_vertices = list(nx.dfs_preorder_nodes(sub_G))
                loops.append(ordered_vertices)
                
            self.facet_boundaries[facet_idx] = loops
        print("[Core] 拓樸快取完成！")

    def _closest_dist_to_loop(self, loop_pts, target_pt):
        """👑 數學核心：計算空間中某一點，到一個 3D 多邊形線段的最短距離"""
        a = loop_pts
        b = np.roll(loop_pts, -1, axis=0) # 下一個點 (頭尾相連)
        ab = b - a
        ap = target_pt - a
        ab_norm2 = np.sum(ab**2, axis=1)
        ab_norm2[ab_norm2 == 0] = 1e-8 # 避免除以零
        t = np.sum(ap * ab, axis=1) / ab_norm2
        t = np.clip(t, 0, 1) # 限制投影點必須在線段內
        closest_pts = a + t[:, np.newaxis] * ab
        dists = np.linalg.norm(target_pt - closest_pts, axis=1)
        return np.min(dists)

    def get_closest_boundary_points(self, hit_face_idx, hit_pos_world):
        """👑 找出滑鼠點擊處最靠近的那一條邊界 (外框或內孔)"""
        facet_idx = self.face_to_facet.get(hit_face_idx)
        if facet_idx is None: return None
        loops = self.facet_boundaries.get(facet_idx)
        if not loops: return None

        min_dist = float('inf')
        best_loop_pts = None

        for loop in loops:
            pts = self.workpiece_mesh.vertices[loop]
            dist = self._closest_dist_to_loop(pts, hit_pos_world)
            if dist < min_dist:
                min_dist = dist
                best_loop_pts = pts

        return best_loop_pts

    def get_facet_boundary_points(self, hit_face_idx):
        facet_idx = self.face_to_facet.get(hit_face_idx)
        if facet_idx is None: return None
        v_indices = self.facet_boundary_indices.get(facet_idx)
        if v_indices is None: return None
        return self.workpiece_mesh.vertices[v_indices]

    def apply_model_transform(self, x, y, z, rx, ry, rz):
        if self.original_mesh is None: return None
        self.workpiece_mesh = self.original_mesh.copy()
        T_rot = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
        T_matrix = np.eye(4)
        T_matrix[:3, :3] = T_rot
        self.workpiece_mesh.apply_transform(T_matrix)
        T_trans = trimesh.transformations.translation_matrix([x, y, z])
        self.workpiece_mesh.apply_transform(T_trans)
        return self.workpiece_mesh

    # ==========================================
    # DP (Douglas-Peucker) 核心數學演算法
    # ==========================================
    def _rdp_closed(self, points, epsilon):
        """處理封閉迴路的 DP 演算法 (先找出最遠點切成兩半)"""
        dists = np.linalg.norm(points - points[0], axis=1)
        split_idx = np.argmax(dists)
        if split_idx == 0: 
            return np.array([points[0], points[-1]])
            
        half1 = self._rdp_open(points[:split_idx+1], epsilon)
        half2 = self._rdp_open(points[split_idx:], epsilon)
        return np.vstack((half1[:-1], half2))

    def _rdp_open(self, points, epsilon):
        """標準開放曲線的 DP 遞迴演算法"""
        if len(points) < 3:
            return points
            
        start = points[0]
        end = points[-1]
        line_vec = end - start
        line_len = np.linalg.norm(line_vec)
        
        if line_len == 0:
            dists = np.linalg.norm(points - start, axis=1)
            dmax = np.max(dists)
            index = np.argmax(dists)
        else:
            # 計算所有點到(起點-終點)這條直線的垂直距離
            line_dir = line_vec / line_len
            vecs = points - start
            cross_prods = np.cross(vecs, line_dir)
            dists = np.linalg.norm(cross_prods, axis=1)
            dmax = np.max(dists)
            index = np.argmax(dists)
            
        if dmax > epsilon:
            # 如果最大誤差超過容忍度，從該點切開並遞迴
            rec1 = self._rdp_open(points[:index+1], epsilon)
            rec2 = self._rdp_open(points[index:], epsilon)
            return np.vstack((rec1[:-1], rec2))
        else:
            # 否則中間的點全部捨棄，只留頭尾
            return np.array([start, end])

    def extract_contour_from_boundary(self, hit_face_idx, hit_pos_world, offset_rx=0, offset_ry=0, offset_rz=0):
        """👑 互動版：結合最近邊界判斷與 DP 壓縮"""
        if self.workpiece_mesh is None: raise ValueError("請先載入 STL 模型！")

        facet_idx = self.face_to_facet.get(hit_face_idx)
        if facet_idx is None:
            raise ValueError("您點擊的位置屬於曲面，目前魔術棒僅支援點選「平坦面」。")
            
        facet_normal = self.workpiece_mesh.facets_normal[facet_idx]
        
        # 👑 改為取得最靠近點擊處的邊界！
        dense_locations = self.get_closest_boundary_points(hit_face_idx, hit_pos_world)
        if dense_locations is None:
            raise ValueError("找不到對應的邊界。")

        # 1. DP 非均勻採樣：完美壓縮碎點
        dp_locations = self._rdp_closed(dense_locations, epsilon=DEFAULT_CHORDAL_ERROR)
        
        # 2. 直線安全細分：避免關節空間插值彎曲
        MAX_STEP = 5.0 # mm
        final_locations = []
        for i in range(len(dp_locations) - 1):
            p1 = dp_locations[i]
            p2 = dp_locations[i+1]
            dist = np.linalg.norm(p2 - p1)
            if dist > MAX_STEP:
                num_sub = int(np.ceil(dist / MAX_STEP))
                sub_pts = np.linspace(p1, p2, num_sub, endpoint=False)
                final_locations.extend(sub_pts)
            else:
                final_locations.append(p1)
                
        final_locations.append(dp_locations[-1])
        locations = np.array(final_locations)
        num_points = len(locations)
        
        face_normals = np.tile(facet_normal, (num_points, 1))

        # 姿態矩陣打包
        from scipy.spatial.transform import Rotation as R
        self.cam_tcp_matrices = [] 
        R_offset = R.from_euler('xyz', [offset_rx, offset_ry, offset_rz], degrees=True).as_matrix()

        for i in range(len(locations)):
            loc = locations[i]
            z_vec = -face_normals[i]
            
            ref_vec = np.array([0.0, 1.0, 0.0]) 
            if np.abs(np.dot(z_vec, ref_vec)) > 0.99: 
                ref_vec = np.array([1.0, 0.0, 0.0])
                
            x_vec = np.cross(ref_vec, z_vec)
            x_vec = x_vec / np.linalg.norm(x_vec)
            y_vec = np.cross(z_vec, x_vec)
            y_vec = y_vec / np.linalg.norm(y_vec)

            T_base_tcp = np.eye(4)
            T_base_tcp[:3, :3] = np.column_stack((x_vec, y_vec, z_vec))
            T_base_tcp[:3, 3] = loc
            
            T_tcp = T_base_tcp.copy()
            T_tcp[:3, :3] = T_base_tcp[:3, :3] @ R_offset
            self.cam_tcp_matrices.append(T_tcp)

        self.toolpath_points = locations
        print(f"[Core] 智慧邊界選取與 DP 壓縮成功！共生成 {num_points} 個點。")
        return self.toolpath_points, self.cam_tcp_matrices