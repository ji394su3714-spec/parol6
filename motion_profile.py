import math

class SCurveProfile:
    def __init__(self, dist, v_max, a_max, j_max):
        """
        7 段 S 曲線規劃器 (純淨版：交由 PVT 引擎精準停駐)
        """
        self.dist = abs(dist)
        self.v_max = abs(v_max)
        self.a_max = abs(a_max)
        self.j_max = abs(j_max)

        if self.dist <= 1e-6 or self.v_max <= 1e-6:
            self.T_total = 0.0
            return

        # 1. 檢查加速度極限是否會被 Jerk 限制
        if self.v_max / self.a_max < self.a_max / self.j_max:
            self.a_max = math.sqrt(self.v_max * self.j_max)

        self.Tj = self.a_max / self.j_max  
        self.Ta = self.v_max / self.a_max  
        
        # 單側加速所需距離
        self.d_acc = 0.5 * self.v_max * (self.Ta + self.Tj)

        # 2. 邊界條件防呆：極速二分搜尋法
        if self.dist < 2 * self.d_acc:
            low, high = 0.0, self.v_max
            for _ in range(30): 
                mid_v = (low + high) / 2.0
                mid_a = self.a_max
                if mid_v / mid_a < mid_a / self.j_max:
                    mid_a = math.sqrt(mid_v * self.j_max)
                
                tj_temp = mid_a / self.j_max
                ta_temp = mid_v / mid_a
                d_acc_temp = 0.5 * mid_v * (ta_temp + tj_temp)
                
                if 2 * d_acc_temp > self.dist:
                    high = mid_v
                else:
                    low = mid_v
                    
            self.v_max = low
            if self.v_max / self.a_max < self.a_max / self.j_max:
                self.a_max = math.sqrt(self.v_max * self.j_max)
            self.Tj = self.a_max / self.j_max
            self.Ta = self.v_max / self.a_max
            self.d_acc = 0.5 * self.v_max * (self.Ta + self.Tj)

        # 3. 巡航時間 (等速段)
        self.Tv = (self.dist - 2 * self.d_acc) / self.v_max if self.v_max > 0 else 0
        if self.Tv < 0: self.Tv = 0

        # 4. 記錄 7 個段落的時間節點
        self.t1 = self.Tj
        self.t2 = self.Ta
        self.t3 = self.Ta + self.Tj
        self.t4 = self.t3 + self.Tv
        self.t5 = self.t4 + self.Tj
        self.t6 = self.t4 + self.Ta
        self.t7 = self.t6 + self.Tj
        self.T_total = self.t7

    def get_progress(self, t):
        """回傳當前時間的進度百分比 (0.0 ~ 1.0)"""
        if self.T_total <= 0: return 1.0
        if t <= 0: return 0.0
        if t >= self.T_total: return 1.0 

        # ==========================================
        # 原汁原味的完美 7 段位置微積分公式
        # ==========================================
        pos = 0.0
        if t <= self.t1:
            pos = (1/6) * self.j_max * t**3
        elif t <= self.t2:
            dt = t - self.t1
            v1 = 0.5 * self.j_max * self.t1**2
            p1 = (1/6) * self.j_max * self.t1**3
            pos = p1 + v1 * dt + 0.5 * self.a_max * dt**2
        elif t <= self.t3:
            dt = t - self.t2
            v1 = 0.5 * self.j_max * self.t1**2
            v2 = v1 + self.a_max * (self.t2 - self.t1)
            p1 = (1/6) * self.j_max * self.t1**3
            p2 = p1 + v1 * (self.t2 - self.t1) + 0.5 * self.a_max * (self.t2 - self.t1)**2
            pos = p2 + v2 * dt + 0.5 * self.a_max * dt**2 - (1/6) * self.j_max * dt**3
        elif t <= self.t4:
            dt = t - self.t3
            pos = self.d_acc + self.v_max * dt
        else:
            # 減速段 (對稱鏡像反推)
            dt_from_end = self.T_total - t
            pos_from_end = 0.0
            if dt_from_end <= self.Tj:
                pos_from_end = (1/6) * self.j_max * dt_from_end**3
            elif dt_from_end <= self.Ta:
                dt = dt_from_end - self.Tj
                v1 = 0.5 * self.j_max * self.Tj**2
                p1 = (1/6) * self.j_max * self.Tj**3
                pos_from_end = p1 + v1 * dt + 0.5 * self.a_max * dt**2
            else:
                dt = dt_from_end - self.Ta
                v1 = 0.5 * self.j_max * self.Tj**2
                v2 = v1 + self.a_max * (self.Ta - self.Tj)
                p1 = (1/6) * self.j_max * self.Tj**3
                p2 = p1 + v1 * (self.Ta - self.Tj) + 0.5 * self.a_max * (self.Ta - self.Tj)**2
                pos_from_end = p2 + v2 * dt + 0.5 * self.a_max * dt**2 - (1/6) * self.j_max * dt**3
            pos = self.dist - pos_from_end
            
        return pos / self.dist