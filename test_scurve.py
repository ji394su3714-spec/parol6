import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

class SCurveProfile:
    def __init__(self, dist, v_max, a_max, j_max, v_start=0.0, v_end=0.0):
        """正統非對稱 7 段式 S 曲線規劃器"""
        self.dist = abs(dist)
        self.v_max = abs(v_max)
        self.a_max = abs(a_max)
        self.j_max = abs(j_max)
        self.v_start = min(abs(v_start), self.v_max)
        self.v_end = min(abs(v_end), self.v_max)
        
        if self.dist <= 1e-6:
            self.T_total = 0.0
            return

        min_dec_dist = abs(self.v_start**2 - self.v_end**2) / (2 * self.a_max) if self.a_max > 0 else 0
        if min_dec_dist > self.dist:
            self.v_start = math.sqrt(max(0.0, self.v_end**2 + 2 * self.a_max * self.dist))

        self._calculate_profile()

        if self.Tv < 0:
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
            self.Tv = 0.0

        self.t1 = self.Tj1
        self.t2 = self.t1 + self.Ta
        self.t3 = self.t2 + self.Tj1
        self.t4 = self.t3 + self.Tv
        self.t5 = self.t4 + self.Tj2
        self.t6 = self.t5 + self.Td
        self.t7 = self.t6 + self.Tj2
        self.T_total = self.t7

    def _calculate_profile(self):
        delta_v_acc = self.v_max - self.v_start
        if delta_v_acc <= 1e-6:
            self.Tj1 = self.Ta = self.d_acc = 0.0
        else:
            if delta_v_acc < (self.a_max**2) / self.j_max:
                self.Tj1 = math.sqrt(delta_v_acc / self.j_max)
                self.Ta = 0.0
            else:
                self.Tj1 = self.a_max / self.j_max
                self.Ta = (delta_v_acc / self.a_max) - self.Tj1
            self.d_acc = self.v_start * (2*self.Tj1 + self.Ta) + 0.5 * delta_v_acc * (2*self.Tj1 + self.Ta)

        delta_v_dec = self.v_max - self.v_end
        if delta_v_dec <= 1e-6:
            self.Tj2 = self.Td = self.d_dec = 0.0
        else:
            if delta_v_dec < (self.a_max**2) / self.j_max:
                self.Tj2 = math.sqrt(delta_v_dec / self.j_max)
                self.Td = 0.0
            else:
                self.Tj2 = self.a_max / self.j_max
                self.Td = (delta_v_dec / self.a_max) - self.Tj2
            self.d_dec = self.v_end * (2*self.Tj2 + self.Td) + 0.5 * delta_v_dec * (2*self.Tj2 + self.Td)

        self.Tv = (self.dist - self.d_acc - self.d_dec) / self.v_max if self.v_max > 0 else 0

    def get_progress(self, t):
        if self.T_total <= 0 or self.dist <= 0: return 1.0
        if t >= self.T_total: return 1.0
        if t <= 0: return 0.0

        pos = 0.0
        v0 = self.v_start
        
        if t <= self.t1:
            return (v0*t + (1/6)*self.j_max*(t**3)) / self.dist
            
        pos += v0*self.t1 + (1/6)*self.j_max*(self.t1**3)
        v1 = v0 + 0.5*self.j_max*(self.t1**2)
        a1 = self.j_max*self.t1

        if t <= self.t2:
            dt = t - self.t1
            return (pos + v1*dt + 0.5*a1*(dt**2)) / self.dist
            
        pos += v1*self.Ta + 0.5*a1*(self.Ta**2)
        v2 = v1 + a1*self.Ta

        if t <= self.t3:
            dt = t - self.t2
            return (pos + v2*dt + 0.5*a1*(dt**2) - (1/6)*self.j_max*(dt**3)) / self.dist
            
        pos += v2*self.Tj1 + 0.5*a1*(self.Tj1**2) - (1/6)*self.j_max*(self.Tj1**3)

        if t <= self.t4:
            dt = t - self.t3
            return (pos + self.v_max * dt) / self.dist
            
        pos += self.v_max * self.Tv

        if t <= self.t5:
            dt = t - self.t4
            return (pos + self.v_max*dt - (1/6)*self.j_max*(dt**3)) / self.dist
            
        pos += self.v_max*self.Tj2 - (1/6)*self.j_max*(self.Tj2**3)
        v5 = self.v_max - 0.5*self.j_max*(self.Tj2**2)
        a5 = -self.j_max*self.Tj2

        if t <= self.t6:
            dt = t - self.t5
            return (pos + v5*dt + 0.5*a5*(dt**2)) / self.dist
            
        pos += v5*self.Td + 0.5*a5*(self.Td**2)
        v6 = v5 + a5*self.Td

        dt = t - self.t6
        return (pos + v6*dt + 0.5*a5*(dt**2) + (1/6)*self.j_max*(dt**3)) / self.dist

# ==========================================
# 互動式 UI 介面
# ==========================================
if __name__ == "__main__":
    # 初始參數
    init_dist = 100.0
    init_vmax = 50.0
    init_amax = 100.0
    init_jmax = 500.0
    init_vstart = 20.0
    init_vend = 10.0

    fig, axs = plt.subplots(4, 1, figsize=(10, 8))
    plt.subplots_adjust(left=0.1, bottom=0.35, right=0.95, top=0.95, hspace=0.4)

    # 初始化繪圖線條
    line_pos, = axs[0].plot([], [], 'b-', lw=2)
    line_vel, = axs[1].plot([], [], 'g-', lw=2)
    line_vstart = axs[1].axhline(init_vstart, color='g', linestyle='--', alpha=0.5)
    line_vend = axs[1].axhline(init_vend, color='g', linestyle='--', alpha=0.5)
    line_acc, = axs[2].plot([], [], 'r-', lw=2)
    line_jerk, = axs[3].plot([], [], 'm-', lw=2)

    axs[0].set_ylabel('Position')
    axs[1].set_ylabel('Velocity')
    axs[2].set_ylabel('Accel')
    axs[3].set_ylabel('Jerk')
    axs[3].set_xlabel('Time (s)')

    for ax in axs:
        ax.grid(True)

    # 建立滑桿 UI 區域
    axcolor = 'lightgoldenrodyellow'
    ax_dist = plt.axes([0.15, 0.25, 0.65, 0.03], facecolor=axcolor)
    ax_vmax = plt.axes([0.15, 0.21, 0.65, 0.03], facecolor=axcolor)
    ax_amax = plt.axes([0.15, 0.17, 0.65, 0.03], facecolor=axcolor)
    ax_jmax = plt.axes([0.15, 0.13, 0.65, 0.03], facecolor=axcolor)
    ax_vstart = plt.axes([0.15, 0.09, 0.65, 0.03], facecolor=axcolor)
    ax_vend = plt.axes([0.15, 0.05, 0.65, 0.03], facecolor=axcolor)

    s_dist = Slider(ax_dist, 'Distance', 1.0, 500.0, valinit=init_dist)
    s_vmax = Slider(ax_vmax, 'V_Max', 1.0, 200.0, valinit=init_vmax)
    s_amax = Slider(ax_amax, 'A_Max', 10.0, 500.0, valinit=init_amax)
    s_jmax = Slider(ax_jmax, 'J_Max', 50.0, 2000.0, valinit=init_jmax)
    s_vstart = Slider(ax_vstart, 'V_Start', 0.0, 150.0, valinit=init_vstart)
    s_vend = Slider(ax_vend, 'V_End', 0.0, 150.0, valinit=init_vend)

    def update(val):
        d = s_dist.val
        v = s_vmax.val
        a = s_amax.val
        j = s_jmax.val
        vs = s_vstart.val
        ve = s_vend.val

        profile = SCurveProfile(d, v, a, j, vs, ve)
        
        if profile.T_total > 0:
            t_arr = np.linspace(0, profile.T_total, 1000)
            dt = t_arr[1] - t_arr[0] if len(t_arr) > 1 else 1.0
            
            pos_arr = np.array([profile.get_progress(t) * d for t in t_arr])
            vel_arr = np.gradient(pos_arr, dt)
            acc_arr = np.gradient(vel_arr, dt)
            jerk_arr = np.gradient(acc_arr, dt)

            line_pos.set_data(t_arr, pos_arr)
            line_vel.set_data(t_arr, vel_arr)
            line_acc.set_data(t_arr, acc_arr)
            line_jerk.set_data(t_arr, jerk_arr)
            
            line_vstart.set_ydata([vs, vs])
            line_vend.set_ydata([ve, ve])

            # 動態調整座標軸範圍
            for ax, data_y in zip(axs, [pos_arr, vel_arr, acc_arr, jerk_arr]):
                ax.set_xlim(0, profile.T_total * 1.05)
                y_min, y_max = np.min(data_y), np.max(data_y)
                padding = max(abs(y_max - y_min) * 0.1, 1.0)
                ax.set_ylim(y_min - padding, y_max + padding)

        fig.canvas.draw_idle()

    # 綁定事件
    s_dist.on_changed(update)
    s_vmax.on_changed(update)
    s_amax.on_changed(update)
    s_jmax.on_changed(update)
    s_vstart.on_changed(update)
    s_vend.on_changed(update)

    # 初始化第一次繪圖
    update(None)
    plt.show()