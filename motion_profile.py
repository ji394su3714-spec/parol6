# motion_profile.py
import math

MAX_JOINT_SPEEDS = [130.0, 42.0, 46.0, 210.0, 210.0, 85.0]  # 各軸安全極速 (度/秒)
MAX_JOINT_ACCELS = [150.0, 60.0, 60.0, 250.0, 250.0, 100.0] # 各軸加速度 (度/秒^2)

class TrapezoidalProfile:
    """單軸梯形/尖角速度輪廓產生器"""
    def __init__(self, distance, max_speed, max_accel):
        self.distance = abs(float(distance))
        self.T_total = 0.0
        
        if self.distance < 1e-6:
            return

        ta_ideal = max_speed / max_accel
        da_ideal = 0.5 * max_accel * (ta_ideal ** 2)

        if da_ideal * 2 > self.distance:
            # 尖角三角形
            self.da = self.distance / 2.0
            self.ta = math.sqrt(2.0 * self.da / max_accel)
            self.v_peak = max_accel * self.ta
            self.tc = 0.0
            self.dc = 0.0
        else:
            # 完美梯形
            self.ta = ta_ideal
            self.da = da_ideal
            self.v_peak = max_speed
            self.dc = self.distance - 2.0 * self.da
            self.tc = self.dc / max_speed

        self.T_total = 2.0 * self.ta + self.tc

    def get_progress(self, t):
        if self.distance < 1e-6 or t <= 0: return 0.0
        if t >= self.T_total: return 1.0

        if t <= self.ta:
            s = 0.5 * (self.v_peak / self.ta) * (t ** 2)
        elif t <= self.ta + self.tc:
            s = self.da + self.v_peak * (t - self.ta)
        else:
            t_dec = t - self.ta - self.tc
            s = self.da + self.dc + (self.v_peak * t_dec) - 0.5 * (self.v_peak / self.ta) * (t_dec ** 2)

        return s / self.distance

class SynchronizedPTP:
    """6 軸同步 PTP 軌跡規劃器 (Time-Scaling 實作)"""
    def __init__(self, start_angles, target_angles, speed_percent=100.0):
        self.start_angles = start_angles
        self.target_angles = target_angles
        self.profiles = []
        self.T_sync = 0.0
        
        # 限制在 1% ~ 200% (解除了以前只能跑 100% 的封印，方便你測極限！)
        speed_factor = max(1.0, min(200.0, speed_percent)) / 100.0
        
        for i in range(6):
            dist = abs(target_angles[i] - start_angles[i])
            v_max = MAX_JOINT_SPEEDS[i] * speed_factor
            a_max = MAX_JOINT_ACCELS[i] * speed_factor
            
            prof = TrapezoidalProfile(dist, v_max, a_max)
            self.profiles.append(prof)
            
            # 找出最慢的員工，由他決定全部人的同步時間
            if prof.T_total > self.T_sync:
                self.T_sync = prof.T_total

    def get_positions(self, t):
        positions = []
        for i in range(6):
            if self.T_sync <= 0 or self.profiles[i].T_total <= 0:
                positions.append(self.target_angles[i])
                continue
                
            # 🌟 核心魔法：將統一時間依照該軸的物理能力進行比例壓縮
            scaled_t = t * (self.profiles[i].T_total / self.T_sync)
            progress = self.profiles[i].get_progress(scaled_t)
            
            delta = self.target_angles[i] - self.start_angles[i]
            current_pos = self.start_angles[i] + delta * progress
            positions.append(current_pos)
            
        return positions