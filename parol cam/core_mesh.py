# core_mesh.py
import numpy as np
import trimesh
import networkx as nx
from scipy.spatial.transform import Rotation as R
from core_math import closest_dist_to_loop

class MeshManager:
    def __init__(self):
        self.original_mesh = None   
        self.workpiece_mesh = None  
        self.face_to_facet = {}
        self.facet_boundaries = {}
        
        self.feature_loops = [] 
        self._last_angle_threshold = None

    def load_stl_mesh(self, filepath):
        mesh = trimesh.load(filepath, force='mesh')
        if isinstance(mesh, trimesh.Scene):
            mesh = mesh.dump(concatenate=True)
        mesh.fix_normals()
        self.original_mesh = mesh
        self.workpiece_mesh = self.original_mesh.copy()
        
        # 修復點：每次載入全新模型時，強制清空快取旗標，逼迫系統重新分析 3D 邊緣！
        self._last_angle_threshold = None 
        
        self._build_topology_cache()
        # 預設：面夾角 > 30度算邊緣；沿著邊緣走，轉角 > 55度就剪斷
        self._build_feature_cache(angle_threshold=30.0, path_turn_threshold=55.0) 
        return self.workpiece_mesh

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

    def _build_topology_cache(self):
        self.face_to_facet = {}
        self.facet_boundaries = {}
        print("[MeshManager] 正在分析 STL 平面特徵...")
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
            
            loops = []
            for comp in components:
                sub_G = G.subgraph(comp)
                endpoints = [n for n, d in sub_G.degree() if d == 1]
                
                if len(endpoints) == 2:
                    ordered_vertices = nx.shortest_path(sub_G, source=endpoints[0], target=endpoints[1])
                    loops.append(ordered_vertices)
                else:
                    try:
                        cycle = nx.find_cycle(sub_G)
                        ordered_vertices = [edge[0] for edge in cycle]
                        ordered_vertices.append(cycle[-1][1])
                        loops.append(ordered_vertices)
                    except nx.NetworkXNoCycle:
                        ordered_vertices = list(nx.dfs_preorder_nodes(sub_G))
                        loops.append(ordered_vertices)
                
            self.facet_boundaries[facet_idx] = loops

    def _build_feature_cache(self, angle_threshold=30.0, path_turn_threshold=55.0):
        if self._last_angle_threshold == angle_threshold: return
        #print(f"[MeshManager] 正在分析 3D 幾何邊緣 (面夾角 > {angle_threshold}°, 路徑轉角 > {path_turn_threshold}°)...")
        self._last_angle_threshold = angle_threshold

        # 1. 抓取 3D 實體的折角邊緣 (Dihedral Features)
        threshold_rad = np.radians(angle_threshold)
        adj_angles = self.original_mesh.face_adjacency_angles
        adj_edges = self.original_mesh.face_adjacency_edges
        mask = adj_angles > threshold_rad
        feature_edges_list = [tuple(sorted(e)) for e in adj_edges[mask]]

        # 2. 抓取 2D 薄板的絕對邊界 (Boundary Features)
        unique_edges, edge_counts = np.unique(np.sort(self.original_mesh.edges, axis=1), axis=0, return_counts=True)
        boundary_edges_list = [tuple(sorted(e)) for e in unique_edges[edge_counts == 1]]

        # 合併所有邊緣
        all_features = list(set(feature_edges_list + boundary_edges_list))

        if len(all_features) == 0:
            self.feature_loops = []
            return

        G = nx.Graph()
        G.add_edges_from(all_features)

        # 3. 尋找「轉角點」(Junctions)
        junctions = set()
        for node in G.nodes():
            deg = G.degree(node)
            if deg != 2:
                junctions.add(node) # 遇到三岔路口或端點，標記為轉角
            else:
                neighbors = list(G.neighbors(node))
                p_v = self.original_mesh.vertices[node]
                p_u = self.original_mesh.vertices[neighbors[0]]
                p_w = self.original_mesh.vertices[neighbors[1]]

                v1 = p_v - p_u
                n1 = np.linalg.norm(v1)
                v2 = p_w - p_v
                n2 = np.linalg.norm(v2)
                
                if n1 < 1e-6 or n2 < 1e-6:
                    continue
                    
                v1 /= n1
                v2 /= n2

                # 計算路徑轉角
                dot_val = np.clip(np.dot(v1, v2), -1.0, 1.0)
                turn_angle = np.degrees(np.arccos(dot_val))
                if turn_angle > path_turn_threshold:
                    junctions.add(node) # 撞到超過 55 度的牆壁，標記為轉角

        # 4. 無死角安全尋路
        loops = []
        visited_edges = set()

        for edge in G.edges():
            e_sort = tuple(sorted(edge))
            if e_sort in visited_edges: continue

            path = [edge[0], edge[1]]
            visited_edges.add(e_sort)

            # 往前探索
            curr = edge[1]
            prev = edge[0]
            while curr not in junctions:
                nbrs = [n for n in G.neighbors(curr) if n != prev]
                if not nbrs: break
                nxt = nbrs[0]
                nxt_e = tuple(sorted((curr, nxt)))
                if nxt_e in visited_edges: 
                    break
                visited_edges.add(nxt_e)
                path.append(nxt)
                prev = curr
                curr = nxt

            # 往後探索
            curr = edge[0]
            prev = edge[1]
            while curr not in junctions:
                nbrs = [n for n in G.neighbors(curr) if n != prev]
                if not nbrs: break
                nxt = nbrs[0]
                nxt_e = tuple(sorted((curr, nxt)))
                if nxt_e in visited_edges: 
                    break
                visited_edges.add(nxt_e)
                path.insert(0, nxt)
                prev = curr
                curr = nxt

            loops.append(path)

        self.feature_loops = loops
        print(f"[MeshManager] 3D 特徵邊緣分析完成！成功拆分為 {len(loops)} 條獨立特徵線。")

    def get_closest_boundary_points(self, hit_face_idx, hit_pos_world):
        facet_idx = self.face_to_facet.get(hit_face_idx)
        if facet_idx is None: return None
        loops = self.facet_boundaries.get(facet_idx)
        if not loops: return None

        min_dist = float('inf')
        best_loop_pts = None
        for loop in loops:
            pts = self.workpiece_mesh.vertices[loop]
            dist = closest_dist_to_loop(pts, hit_pos_world)
            if dist < min_dist:
                min_dist = dist
                best_loop_pts = pts
        return best_loop_pts

    def get_closest_3d_feature_loop(self, hit_pos_world):
        if not self.feature_loops: return None

        min_dist = float('inf')
        best_loop_pts = None
        for loop in self.feature_loops:
            pts = self.workpiece_mesh.vertices[loop]
            dist = closest_dist_to_loop(pts, hit_pos_world)
            if dist < min_dist:
                min_dist = dist
                best_loop_pts = pts
        return best_loop_pts