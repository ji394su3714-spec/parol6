import math

class SCurveProfile:
    def __init__(self, dist, v_max, a_max, j_max, v_start=0.0, v_end=0.0):
        """
        正統非對稱 7 段式 S 曲線規劃器 (Asymmetric 7-Segment S-Curve)
        """
        self.dist = abs(dist)
        self.v_max = abs(v_max)
        self.a_max = abs(a_max)
        self.j_max = abs(j_max)
        self.v_start = min(abs(v_start), self.v_max)
        self.v_end = min(abs(v_end), self.v_max)
        
        if self.dist <= 1e-6:
            self.T_total = 0.0
            return

        # --- 邊界防呆：煞車距離不足 ---
        # 若初速太快，距離不夠減速到末速，必須強制調降初速 (實務上代表前一段必須提早減速)
        min_dec_dist = abs(self.v_start**2 - self.v_end**2) / (2 * self.a_max) if self.a_max > 0 else 0
        if min_dec_dist > self.dist:
            self.v_start = math.sqrt(max(0.0, self.v_end**2 + 2 * self.a_max * self.dist))

        # --- 核心解算器 ---
        self._calculate_profile()

        # --- 處理短距離 (無巡航段) ---
        if self.Tv < 0:
            # 使用二分逼近法尋找新的 v_peak，避開解析解的三次方程式
            low, high = max(self.v_start, self.v_end), self.v_max
            for _ in range(40):
                mid_v = (low + high) / 2.0
                self.v_max = mid_v
                self._calculate_profile()
                if self.Tv < 0:
                    high = mid_v
                else:
                    low = mid_v
            
            self.v_max = low
            self._calculate_profile()
            self.Tv = 0.0 # 強制收斂微小浮點數誤差

        # --- 7 個時間節點 ---
        self.t1 = self.Tj1
        self.t2 = self.t1 + self.Ta
        self.t3 = self.t2 + self.Tj1
        self.t4 = self.t3 + self.Tv
        self.t5 = self.t4 + self.Tj2
        self.t6 = self.t5 + self.Td
        self.t7 = self.t6 + self.Tj2
        self.T_total = self.t7

    def _calculate_profile(self):
        """內部計算方法：計算加速與減速的特徵時間與距離"""
        # --- 加速段 (v_start -> v_max) ---
        delta_v_acc = self.v_max - self.v_start
        if delta_v_acc <= 1e-6:
            self.Tj1 = self.Ta = self.d_acc = 0.0
        else:
            if delta_v_acc < (self.a_max**2) / self.j_max:
                # 達不到最大加速度
                self.Tj1 = math.sqrt(delta_v_acc / self.j_max)
                self.Ta = 0.0
            else:
                self.Tj1 = self.a_max / self.j_max
                self.Ta = (delta_v_acc / self.a_max) - self.Tj1
            self.d_acc = self.v_start * (2*self.Tj1 + self.Ta) + 0.5 * delta_v_acc * (2*self.Tj1 + self.Ta)

        # --- 減速段 (v_max -> v_end) ---
        delta_v_dec = self.v_max - self.v_end
        if delta_v_dec <= 1e-6:
            self.Tj2 = self.Td = self.d_dec = 0.0
        else:
            if delta_v_dec < (self.a_max**2) / self.j_max:
                # 達不到最大負加速度
                self.Tj2 = math.sqrt(delta_v_dec / self.j_max)
                self.Td = 0.0
            else:
                self.Tj2 = self.a_max / self.j_max
                self.Td = (delta_v_dec / self.a_max) - self.Tj2
            self.d_dec = self.v_end * (2*self.Tj2 + self.Td) + 0.5 * delta_v_dec * (2*self.Tj2 + self.Td)

        # --- 巡航段時間 ---
        self.Tv = (self.dist - self.d_acc - self.d_dec) / self.v_max if self.v_max > 0 else 0

    def get_progress(self, t):
        """將時間映射為距離進度 (0.0 ~ 1.0)，嚴格執行 7 段微積分"""
        if self.T_total <= 0 or self.dist <= 0: return 1.0
        if t >= self.T_total: return 1.0
        if t <= 0: return 0.0

        pos = 0.0
        v0 = self.v_start
        
        # 1. 加速爬升段 (Jerk > 0)
        if t <= self.t1:
            pos = v0*t + (1/6)*self.j_max*(t**3)
            return pos / self.dist
            
        pos += v0*self.t1 + (1/6)*self.j_max*(self.t1**3)
        v1 = v0 + 0.5*self.j_max*(self.t1**2)
        a1 = self.j_max*self.t1

        # 2. 等加速段 (Jerk = 0)
        if t <= self.t2:
            dt = t - self.t1
            pos += v1*dt + 0.5*a1*(dt**2)
            return pos / self.dist
            
        pos += v1*self.Ta + 0.5*a1*(self.Ta**2)
        v2 = v1 + a1*self.Ta

        # 3. 加速下降段 (Jerk < 0)
        if t <= self.t3:
            dt = t - self.t2
            pos += v2*dt + 0.5*a1*(dt**2) - (1/6)*self.j_max*(dt**3)
            return pos / self.dist
            
        pos += v2*self.Tj1 + 0.5*a1*(self.Tj1**2) - (1/6)*self.j_max*(self.Tj1**3)

        # 4. 巡航段 (Jerk = 0, Accel = 0)
        if t <= self.t4:
            dt = t - self.t3
            pos += self.v_max * dt
            return pos / self.dist
            
        pos += self.v_max * self.Tv

        # 5. 減速爬升段 (Jerk < 0)
        if t <= self.t5:
            dt = t - self.t4
            pos += self.v_max*dt - (1/6)*self.j_max*(dt**3)
            return pos / self.dist
            
        pos += self.v_max*self.Tj2 - (1/6)*self.j_max*(self.Tj2**3)
        v5 = self.v_max - 0.5*self.j_max*(self.Tj2**2)
        a5 = -self.j_max*self.Tj2

        # 6. 等減速段 (Jerk = 0)
        if t <= self.t6:
            dt = t - self.t5
            pos += v5*dt + 0.5*a5*(dt**2)
            return pos / self.dist
            
        pos += v5*self.Td + 0.5*a5*(self.Td**2)
        v6 = v5 + a5*self.Td

        # 7. 減速下降段 (Jerk > 0)
        dt = t - self.t6
        pos += v6*dt + 0.5*a5*(dt**2) + (1/6)*self.j_max*(dt**3)
        
        return pos / self.dist