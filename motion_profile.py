# motion_profile.py
import math

class TrapezoidalProfile:
    """
    工業級梯形/尖角速度輪廓產生器
    負責根據物理極限 (速度、加速度) 規劃出最完美的時間表
    """
    def __init__(self, distance, max_speed, max_accel):
        self.distance = abs(float(distance))
        self.T_total = 0.0
        
        # 防呆：距離太短就不動
        if self.distance < 1e-6:
            return

        # 1. 算出「理論上」加速到極速需要的時間與距離
        ta_ideal = max_speed / max_accel
        da_ideal = 0.5 * max_accel * (ta_ideal ** 2)

        # 2. 判斷是「梯形」還是「尖角三角形」
        if da_ideal * 2 > self.distance:
            # 【狀況 A：短距離】距離太短，來不及加速到極速就要煞車了 -> 變成尖角三角形
            self.da = self.distance / 2.0
            self.ta = math.sqrt(2.0 * self.da / max_accel)
            self.v_peak = max_accel * self.ta
            self.tc = 0.0
            self.dc = 0.0
        else:
            # 【狀況 B：長距離】距離夠長，有等速巡航期 -> 完美梯形
            self.ta = ta_ideal
            self.da = da_ideal
            self.v_peak = max_speed
            self.dc = self.distance - 2.0 * self.da
            self.tc = self.dc / max_speed

        # 3. 算出總耗時
        self.T_total = 2.0 * self.ta + self.tc

    def get_progress(self, t):
        """
        輸入當下時間 t，回傳當下的「進度百分比 (0.0 ~ 1.0)」
        這個進度可以直接無縫替換掉我們舊版 S-Curve 的 linear_t
        """
        if self.distance < 1e-6 or t <= 0: return 0.0
        if t >= self.T_total: return 1.0

        if t <= self.ta:
            # 階段 1：等加速度起步段
            s = 0.5 * (self.v_peak / self.ta) * (t ** 2)
            
        elif t <= self.ta + self.tc:
            # 階段 2：等速巡航段
            s = self.da + self.v_peak * (t - self.ta)
            
        else:
            # 階段 3：等減速度煞車段
            t_dec = t - self.ta - self.tc
            s = self.da + self.dc + (self.v_peak * t_dec) - 0.5 * (self.v_peak / self.ta) * (t_dec ** 2)

        return s / self.distance