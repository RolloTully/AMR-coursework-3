import numpy as np
import time


#========================
# Code by jiayee
#========================

# =======================
# 队列结构 / Sliding Queue
# =======================
class Queue:
    def __init__(self, length):
        self.length = length
        self.memory = []

    def push(self, var):
        self.memory.append(var)
        if len(self.memory) > self.length:
            self.memory.pop(0)

    def __call__(self):
        return np.array(self.memory)

# =======================
# 工具函数 / Utility Tools
# =======================

# 计算积分平方误差 / Integral of Square Error
def ISE(errors, times):
    return np.sum(np.square(errors) * np.array(times)[:, None], axis=0)

# 计算时间加权绝对误差 / Integral of Time-weighted Absolute Error
def ITAE(errors, times):
    return np.sum(np.abs(errors) * np.array(times)[:, None], axis=0)

# 将角度规范到 [-π, π] / Normalize angle to [-pi, pi]
def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# 将全局坐标转换到机体坐标系 / Transform from global to local frame
def DCM(vector, yaw):
    transform = np.array([
        [ np.cos(yaw),  np.sin(yaw), 0, 0],
        [-np.sin(yaw),  np.cos(yaw), 0, 0],
        [          0,           0, 1, 0],
        [          0,           0, 0, 1]
    ])
    return transform @ vector

# ==================================
# 控制器状态变量 / Controller States
# ==================================
error_queue = Queue(5)                     # 误差滑动窗口 / Error queue for derivative estimation
error_history = []                         # 错误历史 / For ISE/ITAE计算
time_history = []                          # 时间步历史 / For ISE/ITAE计算

control_gains = np.array([                 # 控制增益矩阵 / PD gains
    [0.6, 0.6, 0.6, 0.6],                  # Kp
    [0.1, 0.1, 0.1, 0.1]                   # Kd
])

prev_command = np.zeros(4)                 # 上一周期命令 / For smoothing
prev_target = np.zeros(4)                  # 上一目标点 / For detecting target switch
prev_ISE = np.ones(4)
prev_ITAE = np.ones(4)

review_interval = 30                       # 调参周期（帧） / Adaptive tuning interval
time_elapsed = 0.0

MAX_DELTA = 0.12                           # 最大变化量 / Max delta for smoothness
MAX_OUTPUT = 0.3                           # 最大速度输出 / Max control output

ISE_history = []
ITAE_history = []
P_history = []
D_history = []

# ==================================
# 主控制器函数 / Main controller
# ==================================
def controller(state, target_pos, dt):
    global error_queue, error_history, time_history
    global control_gains, prev_command, prev_target
    global prev_ISE, prev_ITAE, time_elapsed

    # === 状态解析 / Parse current and target states ===
    pos = np.array(state[:3])                     # 当前坐标 / current position
    yaw = wrap_angle(state[5])                    # 当前航向角 / current yaw
    target_xyz = np.array(target_pos[:3])
    target_yaw = wrap_angle(target_pos[3])
    target_all = np.array([*target_xyz, target_yaw])

    # === 检测目标切换 / Target switch detection ===
    if np.linalg.norm(target_all - prev_target) > 0.3:
        error_history = []
        time_history = []
        error_queue.memory.clear()
    prev_target = target_all.copy()

    # === 误差计算 / Compute errors ===
    error = np.array([
        target_xyz[0] - pos[0],
        target_xyz[1] - pos[1],
        target_xyz[2] - pos[2],
        wrap_angle(target_yaw - yaw)
    ])
    error[np.abs(error) < 0.002] = 0.0

    error_queue.push(error.tolist())
    error_history.append(error)
    time_history.append(dt)

    # === 增益调参逻辑 / Gain adaptation based on ISE & ITAE ===
    if len(error_history) >= review_interval:
        ise_now = ISE(error_history, time_history)
        itae_now = ITAE(error_history, time_history)

        delta_ise = ise_now - prev_ISE
        delta_itae = itae_now - prev_ITAE

        P_adj = np.where(np.abs(delta_ise) > 0.01, np.where(delta_ise > 0, -0.005, 0.003), 0.0)
        D_adj = np.where(np.abs(delta_itae) > 0.01, np.where(delta_itae > 0, -0.007, 0.004), 0.0)

        control_gains[0] = np.clip(control_gains[0] * (1 + P_adj), 0.1, 1.5)
        control_gains[1] = np.clip(control_gains[1] * (1 + D_adj), 0.05, 1.0)

        prev_ISE, prev_ITAE = ise_now, itae_now
        error_history, time_history = [], []

        P_history.append(control_gains[0])
        D_history.append(control_gains[1])
        ISE_history.append(ISE_loss)
        ITAE_history.append(ITAE_loss)

    # === 导数估计 / Estimate derivative ===
    error_array = error_queue()
    deriv = np.mean(np.diff(error_array, axis=0), axis=0) / dt if len(error_array) >= 2 else np.zeros(4)

    # === 控制器输出 / PD control law ===
    control_signal = control_gains[0] * error + control_gains[1] * deriv

    # === 平滑变化 / Smooth command transition ===
    local_command = DCM(control_signal, yaw)
    delta_cmd = np.clip(local_command - prev_command, -MAX_DELTA, MAX_DELTA)
    smoothed_cmd = prev_command + delta_cmd
    prev_command[:] = smoothed_cmd

    # === 限幅输出 / Clip to max speed ===
    vx, vy, vz, yaw_rate = np.clip(smoothed_cmd, [-MAX_OUTPUT]*3 + [-1.0], [MAX_OUTPUT]*3 + [1.0])

    time_elapsed += dt

    return float(vx), float(vy), float(vz), float(yaw_rate)
