import math

class SCurveProfile:
    def __init__(self, dist, v_max, a_max, j_max, v_min_ratio=0.03):
        """
        7 段 S 曲線規劃器 (含最低進給恆速接管 V_MIN 演算法)
        :param v_min_ratio: 最低進給速度比例 (預設為極速的 5%，確保撞線乾脆俐落)
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

        # ==========================================
        # 🌟 5. V_MIN 恆速接管設定 (取代斬尾)
        # ==========================================
        self.v_min = self.v_max * v_min_ratio 
        
        if self.v_min > 0 and self.T_total > 0:
            # 利用二分搜尋法，精準找出減速期速度掉到 V_MIN 的那一個瞬間 (t_cutoff)
            low, high = self.t4, self.T_total
            for _ in range(20):
                mid = (low + high) / 2.0
                if self._calc_v_at_t(mid) > self.v_min:
                    low = mid  # 速度還太快，時間往後推
                else:
                    high = mid # 速度已經太慢，時間往前推
            
            self.t_cutoff = mid
            self.pos_cutoff = self._calc_pos_at_t(self.t_cutoff)
        else:
            self.t_cutoff = self.T_total
            self.pos_cutoff = self.dist

    def _calc_v_at_t(self, t):
        """計算在時間 t 時的理論減速速度 (對稱微積分推導)"""
        dt_from_end = self.T_total - t
        if dt_from_end <= 0: return 0.0
        
        if dt_from_end <= self.Tj:
            return 0.5 * self.j_max * dt_from_end**2
        elif dt_from_end <= self.Ta:
            dt = dt_from_end - self.Tj
            v1 = 0.5 * self.j_max * self.Tj**2
            return v1 + self.a_max * dt
        else:
            dt = dt_from_end - self.Ta
            v1 = 0.5 * self.j_max * self.Tj**2
            v2 = v1 + self.a_max * (self.Ta - self.Tj)
            return v2 + self.a_max * dt - 0.5 * self.j_max * dt**2

    def _calc_pos_at_t(self, t):
        """原本的 S-Curve 位置核心公式 (封裝為內部呼叫)"""
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
        return pos

    def get_progress(self, t):
        """回傳當前時間的進度百分比 (0.0 ~ 1.0)"""
        if self.T_total <= 0: return 1.0
        if t <= 0: return 0.0

        # ==========================================
        # 攔截點：V_MIN 恆速滑行接管！
        # ==========================================
        if t >= self.t_cutoff:
            dt = t - self.t_cutoff
            
            # 放棄二次/三次減速曲線，直接使用 V_MIN 做等速直線推進
            pos = self.pos_cutoff + self.v_min * dt
            
            # 因為我們維持了 V_MIN (沒有讓速度掉到0)，所以會比理論時間更早抵達終點
            if pos >= self.dist:
                return 1.0
            return pos / self.dist

        # 如果還沒到接管點，就用完美的 S-Curve 算 pos
        pos = self._calc_pos_at_t(t)
        return pos / self.dist