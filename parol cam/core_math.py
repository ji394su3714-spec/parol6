# core_math.py
import numpy as np

def closest_dist_to_loop(loop_pts, target_pt):
    """計算空間中某一點，到一個 3D 多邊形線段的最短距離"""
    a = loop_pts
    b = np.roll(loop_pts, -1, axis=0) 
    ab = b - a
    ap = target_pt - a
    ab_norm2 = np.sum(ab**2, axis=1)
    ab_norm2[ab_norm2 == 0] = 1e-8 
    t = np.sum(ap * ab, axis=1) / ab_norm2
    t = np.clip(t, 0, 1) 
    closest_pts = a + t[:, np.newaxis] * ab
    dists = np.linalg.norm(target_pt - closest_pts, axis=1)
    return np.min(dists)

def rdp_open(points, epsilon):
    """標準開放曲線的 DP 遞迴演算法"""
    if len(points) < 3: return points
    start, end = points[0], points[-1]
    line_vec = end - start
    line_len = np.linalg.norm(line_vec)
    
    if line_len == 0:
        dists = np.linalg.norm(points - start, axis=1)
        index = np.argmax(dists)
        dmax = dists[index]
    else:
        line_dir = line_vec / line_len
        vecs = points - start
        cross_prods = np.cross(vecs, line_dir)
        dists = np.linalg.norm(cross_prods, axis=1)
        index = np.argmax(dists)
        dmax = dists[index]
        
    if dmax > epsilon:
        rec1 = rdp_open(points[:index+1], epsilon)
        rec2 = rdp_open(points[index:], epsilon)
        return np.vstack((rec1[:-1], rec2))
    else:
        return np.array([start, end])

def rdp_closed(points, epsilon):
    """處理封閉迴路的 DP 演算法"""
    dists = np.linalg.norm(points - points[0], axis=1)
    split_idx = np.argmax(dists)
    if split_idx == 0: 
        return np.array([points[0], points[-1]])
        
    half1 = rdp_open(points[:split_idx+1], epsilon)
    half2 = rdp_open(points[split_idx:], epsilon)
    return np.vstack((half1[:-1], half2))