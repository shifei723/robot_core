"""
Slash 双轮足机器人 — 腿高控制模块（提取版）
=============================================

从 slash_robot_control 工程提取的「腿高控制」完整逻辑，供 ROS2 / RViz
仿真复刻使用。所有函数原样对标固件 LeggedRobot.ino：

  腿高目标 → 斜坡插值 → （横滚差动 / 跳跃状态机接管）→ 左右腿高
      → 五连杆 IK (right_ik / left_ik) → 髋关节角度
      → 角度映射 (firmware_to_mujoco_*) → 位置执行器 ctrl

来源：
  - vmc.py    （结构常量、IK 反解、FK 正解、角度映射、RollController、JumpController）
  - main.py   （腿高插值 leg_height_ramp，原 main.py 主循环第 2 段）

依赖：仅标准库 math，无 MuJoCo / numpy 依赖，可直接移植到任意 ROS2 节点。

作者注释中的「固件」均指下位机源码 LeggedRobot.ino（参数权威来源：
/Users/shifei/Documents/code/2.下位机源码）。
"""

import math

# ==================== 结构常量（固件 ConfigurationParameter_t） ====================
THIGH = 0.15            # 大腿长度 m (L1 = L4)
SHANK = 0.24            # 小腿长度 m (L2 = L3)
MOTOR_SPACING = 0.085   # 电机间距 m (L5)
POSTURE_RESTRICTIONS_X = 0.08   # 前后偏移限幅 m
MIN_LOW = 0.13          # 最低腿高 m
MAX_HIGH = 0.37         # 最高腿高 m

# ==================== 腿高目标参数（main.py 参数段） ====================
DEFAULT_HEIGHT = 0.25   # 默认腿高 m
HEIGHT_RAMP_RATE = 0.5  # 腿高插值速率 m/s (固件: LegLength_f 低通 / RobotJumping 下蹲速度 0.3)


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _asin_safe(v):
    return math.asin(_clamp(v, -1.0, 1.0))


def _acos_safe(v):
    return math.acos(_clamp(v, -1.0, 1.0))


# =====================================================================
#  五连杆逆运动学（完全对标固件 RightInverseKinematics / LeftInverseKinematics）
# =====================================================================

def right_ik(x: float, y: float, pitch_deg: float):
    """
    右腿五连杆 IK

    :param x:          前后偏移 (m), 正=前
    :param y:          腿高 (m), 正=下
    :param pitch_deg:  俯仰目标 (度)
    :return:           (alpha_deg, beta_deg, error)
                        alpha = 前髋关节角, beta = 后髋关节角
                        error: 0=正常, 1=前髋超范围, 2=后髋超范围
    """
    x = _clamp(x, -POSTURE_RESTRICTIONS_X, POSTURE_RESTRICTIONS_X)
    y = _clamp(y, MIN_LOW, MAX_HIGH)

    AB = THIGH   # 大腿
    BC = SHANK   # 小腿
    OA = MOTOR_SPACING / 2.0

    pitch = math.radians(pitch_deg)

    # 坐标旋转 (加入俯仰角)
    cos_p = math.cos(pitch)
    sin_p = math.sin(pitch)
    x1 = x * cos_p - y * sin_p
    y1 = x * sin_p + y * cos_p

    OF = x1
    FC = y1

    # ---- 关节1 (前髋, alpha) ----
    OC = math.sqrt(OF * OF + FC * FC)
    if OC < 1e-8:
        return 0.0, 0.0, 1

    aOCF = _asin_safe(OF / OC)
    aAOC = math.pi / 2.0 + aOCF

    AC = math.sqrt(OA * OA + OC * OC - 2.0 * OA * OC * math.cos(aAOC))
    if AC >= (AB + BC):
        return 0.0, 0.0, 1

    aOCA = _acos_safe((OC * OC + AC * AC - OA * OA) / (2.0 * OC * AC))
    aOAC = math.pi - aOCA - aAOC
    aBAC = _acos_safe((AB * AB + AC * AC - BC * BC) / (2.0 * AB * AC))
    aBAG = math.pi - aBAC - aOAC

    # ---- 关节2 (后髋, beta) ----
    aOCF2 = -aOCF
    aEOC = math.pi / 2.0 + aOCF2
    OE = OA

    EC = math.sqrt(OE * OE + OC * OC - 2.0 * OE * OC * math.cos(aEOC))
    if EC >= (AB + BC):
        return 0.0, 0.0, 2

    aOCE = _acos_safe((OC * OC + EC * EC - OE * OE) / (2.0 * OC * EC))
    aOEC = math.pi - aOCE - aEOC
    aDEC = _acos_safe((AB * AB + EC * EC - BC * BC) / (2.0 * AB * EC))
    aDEH = math.pi - aDEC - aOEC

    alpha_deg = math.degrees(aBAG)
    beta_deg = math.degrees(aDEH)
    return alpha_deg, beta_deg, 0


def left_ik(x: float, y: float, pitch_deg: float):
    """
    左腿五连杆 IK（对标固件 LeftInverseKinematics）
    左腿镜像: x→-x, pitch→-pitch, 其余与右腿相同

    :return: (alpha_deg, beta_deg, error)
    """
    x = _clamp(x, -POSTURE_RESTRICTIONS_X, POSTURE_RESTRICTIONS_X)
    y = _clamp(y, MIN_LOW, MAX_HIGH)

    # 镜像
    x = -x
    pitch_deg = -pitch_deg

    AB = THIGH
    BC = SHANK
    OA = MOTOR_SPACING / 2.0

    pitch = math.radians(pitch_deg)

    cos_p = math.cos(pitch)
    sin_p = math.sin(pitch)
    x1 = x * cos_p - y * sin_p
    y1 = x * sin_p + y * cos_p

    OF = -x1   # 左腿取反
    FC = y1

    # ---- 关节1 (alpha) ----
    OC = math.sqrt(OF * OF + FC * FC)
    if OC < 1e-8:
        return 0.0, 0.0, 1

    aOCF = _asin_safe(OF / OC)
    aAOC = math.pi / 2.0 + aOCF

    AC = math.sqrt(OA * OA + OC * OC - 2.0 * OA * OC * math.cos(aAOC))
    if AC >= (AB + BC):
        return 0.0, 0.0, 1

    aOCA = _acos_safe((OC * OC + AC * AC - OA * OA) / (2.0 * OC * AC))
    aOAC = math.pi - aOCA - aAOC
    aBAC = _acos_safe((AB * AB + AC * AC - BC * BC) / (2.0 * AB * AC))
    aBAG = math.pi - aBAC - aOAC

    # ---- 关节2 (beta) ----
    aOCF2 = -aOCF
    aEOC = math.pi / 2.0 + aOCF2
    OE = OA

    EC = math.sqrt(OE * OE + OC * OC - 2.0 * OE * OC * math.cos(aEOC))
    if EC >= (AB + BC):
        return 0.0, 0.0, 2

    aOCE = _acos_safe((OC * OC + EC * EC - OE * OE) / (2.0 * OC * EC))
    aOEC = math.pi - aOCE - aEOC
    aDEC = _acos_safe((AB * AB + EC * EC - BC * BC) / (2.0 * AB * EC))
    aDEH = math.pi - aDEC - aOEC

    alpha_deg = math.degrees(aBAG)
    beta_deg = math.degrees(aDEH)
    return alpha_deg, beta_deg, 0


# =====================================================================
#  固件角度 → 执行器关节角映射（main.py 使用 MuJoCo position actuator）
# =====================================================================
#  固件 IK 输出 alpha/beta (度) → 执行器 ctrl (rad)
#  映射公式: q = π/2 - α_rad
#  验证: 站立 IK→32.24° → ctrl = π/2 - 0.5627 = 1.0081 ≈ keyframe 1.008

def firmware_to_mujoco_left(alpha_deg: float, beta_deg: float):
    """左腿: 固件IK角度(度) → (q_lf, q_lb) 执行器 ctrl (rad)"""
    q_lf = math.pi / 2.0 - math.radians(alpha_deg)
    q_lb = math.pi / 2.0 - math.radians(beta_deg)
    return q_lf, q_lb


def firmware_to_mujoco_right(alpha_deg: float, beta_deg: float):
    """右腿: 固件IK角度(度) → (q_rf, q_rb) 执行器 ctrl (rad)"""
    q_rf = math.pi / 2.0 - math.radians(alpha_deg)
    q_rb = math.pi / 2.0 - math.radians(beta_deg)
    return q_rf, q_rb


def mujoco_to_firmware(q_rad: float) -> float:
    """执行器关节角 (rad) → 固件角度 (度), firmware_to_mujoco 的逆映射"""
    return math.degrees(math.pi / 2.0 - q_rad)


# =====================================================================
#  五连杆正运动学（对标固件 forwardKinematics）
#  实际关节角 → 足端坐标, 用于实测腿高 h_meas 与平衡环力臂
# =====================================================================

def forward_kinematics(alpha_deg: float, beta_deg: float):
    """
    单腿五连杆正解 (固件 forwardKinematics 单腿支路)

    :param alpha_deg: 前髋关节角 (度, 固件约定)
    :param beta_deg:  后髋关节角 (度, 固件约定)
    :return:          (x, y) 足端坐标 (m), x 前正 / y 下正, 相对两电机中点
    """
    L1 = L4 = THIGH
    L2 = L3 = SHANK
    L5 = MOTOR_SPACING

    alpha = math.radians(180.0 - alpha_deg)
    beta = math.radians(beta_deg)

    ax = L1 * math.cos(alpha)
    ay = L1 * math.sin(alpha)
    cx = L5 + L4 * math.cos(beta)
    cy = L4 * math.sin(beta)

    a = 2.0 * (ax - cx) * L2
    b = 2.0 * (ay - cy) * L2
    l = math.hypot(ax - cx, ay - cy)
    c = L3 * L3 - L2 * L2 - l * l

    disc = math.sqrt(max(b * b + a * a - c * c, 0.0))
    denom = a + c
    if abs(denom) < 1e-9:
        return 0.0, 0.0
    theta1 = 2.0 * math.atan((b + disc) / denom)
    theta2 = 2.0 * math.atan((b - disc) / denom)

    theta1 = theta1 if theta1 >= 0 else theta1 + 2.0 * math.pi
    theta2 = theta2 if theta2 >= 0 else theta2 + 2.0 * math.pi
    theta = theta2 if theta1 >= math.pi / 2.0 else theta1

    x = ax + L2 * math.cos(theta) - MOTOR_SPACING / 2.0
    y = ay + L2 * math.sin(theta)
    return x, y


def measure_leg_height(alpha_l_deg, beta_l_deg, alpha_r_deg, beta_r_deg) -> float:
    """实测腿高 (m): FK 双腿足端 y 均值 (固件 (Left+Right)*0.5)"""
    _, y_l = forward_kinematics(alpha_l_deg, beta_l_deg)
    _, y_r = forward_kinematics(alpha_r_deg, beta_r_deg)
    return (y_l + y_r) * 0.5


# =====================================================================
#  URDF 五连杆膝关节精确求解
#
#  URDF 实测几何 (slash.urdf, 矢状面 = base_link 的 y-z 平面,
#  y 为前向 / z 为向上, x 为横向):
#
#      前电机(joint1)  后电机(joint1)
#      y=+0.043636      y=-0.041364     均在 z=+0.06
#
#  关节角 → 连杆方向角 (矢状面 atan2(z, y), 单位 rad) 的精确映射,
#  已通过逐关节齐次变换数值标定, 四条腿完全一致:
#
#      大腿(前) = -π/2 + q1_front
#      大腿(后) = -π/2 - q1_back
#      小腿(前) = -π/2 - KNEE_YAW + q1_front - q2_front
#      小腿(后) = -π/2 + KNEE_YAW - q1_back  + q2_back
#
#  KNEE_YAW 是 URDF 膝关节 origin 内建的 yaw 偏置 (之前靠手调
#  0.20 rad 补偿的就是它), 精确值 = π - 2.96357049946679。
# =====================================================================

URDF_MOTOR_Y_FRONT = 0.043636112488779   # 前电机矢状面 y (m)
URDF_MOTOR_Y_BACK = -0.041363887511213   # 后电机矢状面 y (m)
URDF_MOTOR_Z = 0.06                      # 电机矢状面 z (m)
URDF_KNEE_YAW = math.pi - 2.96357049946679   # 膝关节内建 yaw 偏置 (rad)


def five_bar_knee_angles_closed(q_hip_front: float, q_hip_back: float):
    """
    由髋关节角精确求解膝关节角, 使前后小腿末端严格重合 (五连杆闭环)。

    求解步骤:
      1. 由髋关节角算出两个膝点 Kf / Kb (矢状面坐标)
      2. 两圆 (Kf, SHANK) 与 (Kb, SHANK) 交点取下方解 = 足端 F
      3. 反解小腿方向角 → URDF 膝关节角

    :param q_hip_front: 前髋关节角 q1 (rad, URDF 约定)
    :param q_hip_back:  后髋关节角 q1 (rad, URDF 约定)
    :return:            (q_knee_front, q_knee_back) 膝关节角 (rad, URDF 约定)
    """
    # 1. 膝点位置
    th_f = -math.pi / 2.0 + q_hip_front
    th_b = -math.pi / 2.0 - q_hip_back
    kf_y = URDF_MOTOR_Y_FRONT + THIGH * math.cos(th_f)
    kf_z = URDF_MOTOR_Z + THIGH * math.sin(th_f)
    kb_y = URDF_MOTOR_Y_BACK + THIGH * math.cos(th_b)
    kb_z = URDF_MOTOR_Z + THIGH * math.sin(th_b)

    # 2. 两圆交点 (等半径, 交点在中垂线上)
    dy = kb_y - kf_y
    dz = kb_z - kf_z
    d = math.hypot(dy, dz)
    if d < 1e-9 or d > 2.0 * SHANK:
        return 0.0, 0.0
    half = d / 2.0
    off = math.sqrt(max(SHANK * SHANK - half * half, 0.0))
    my = (kf_y + kb_y) / 2.0
    mz = (kf_z + kb_z) / 2.0
    uy, uz = dy / d, dz / d
    # 两个候选解, 取 z 较小者 (足端向下)
    cand_y = (my + off * uz, my - off * uz)
    cand_z = (mz - off * uy, mz + off * uy)
    idx = 0 if cand_z[0] < cand_z[1] else 1
    foot_y, foot_z = cand_y[idx], cand_z[idx]

    # 3. 反解膝关节角
    shank_f = math.atan2(foot_z - kf_z, foot_y - kf_y)
    shank_b = math.atan2(foot_z - kb_z, foot_y - kb_y)
    q_knee_front = -math.pi / 2.0 - URDF_KNEE_YAW + q_hip_front - shank_f
    q_knee_back = shank_b + math.pi / 2.0 - URDF_KNEE_YAW + q_hip_back

    return q_knee_front, q_knee_back


# =====================================================================
#  腿高目标斜坡插值（提取自 main.py 主循环第 2 段）
# =====================================================================

def leg_height_ramp(current_h: float, target_h: float, dt: float,
                    rate: float = HEIGHT_RAMP_RATE) -> float:
    """
    腿高斜坡限速插值: 每步最多变化 rate*dt, 避免目标跳变导致腿瞬移。

    :param current_h: 当前腿高 (m)
    :param target_h:  目标腿高 (m)
    :param dt:        控制周期 (s)
    :param rate:      插值速率 (m/s), 默认 0.5 (固件 LegLength_f 低通)
    :return:          插值后的腿高 (m), 已限幅 [MIN_LOW, MAX_HIGH]
    """
    max_step = rate * dt
    next_h = current_h + _clamp(target_h - current_h, -max_step, max_step)
    return _clamp(next_h, MIN_LOW, MAX_HIGH)


# =====================================================================
#  横滚环 → 左右腿长差动（对标固件 RollPid, 论文 3.4.3）
# =====================================================================

class RollController:
    """
    横滚环: PID 差动调节两侧腿长维持横滚平衡。

    适用: 单边越障 (左轮平地/右轮高台)、横向斜坡。

    控制律 (几何前馈 + PID 反馈):
      e_r = r_d - r                                横滚误差 (rad)
      Δh = Kp·(W/2)·tan(e_r) + Ki·∫e_r dt + Kd·ṙ   单侧腿长差动量 (m)
      h_L = h + Δh,  h_R = h − Δh

    符号约定 (仿真已验证): roll < 0 = 右侧高 → e_r > 0 → Δh > 0
      → 左腿伸长/右腿缩短, 机身拉回水平。
    """

    def __init__(self, wheel_track: float = 0.328):
        self.wheel_track = wheel_track   # 轮距 m (左右轮中心距)
        self.kp = 0.9                    # 几何补偿系数 (1.0 = 瞬时完全补偿)
        self.ki = 0.6                    # 积分 (m/(rad·s)): 消除横滚稳态误差
        self.kd = 0.02                   # 横滚角速度阻尼 (s)
        self.i_limit = 0.05              # 积分限幅 m
        self.delta_limit = 0.06          # 单侧差动限幅 m (防 IK 超行程/奇异)
        self.enabled = True

        self._integral = 0.0
        self.dbg_delta = 0.0

    def compute(self, roll_rad: float, gyro_roll: float, dt: float,
                target_roll_rad: float = 0.0) -> float:
        """
        :param roll_rad:        实测横滚角 (rad), 负=右侧高
        :param gyro_roll:       横滚角速度 (rad/s)
        :param dt:              控制周期 (s)
        :param target_roll_rad: 目标横滚角 (rad), 默认 0 = 机身水平
        :return: Δh 单侧腿长差动量 (m), 左腿 +Δh / 右腿 −Δh
        """
        if not self.enabled:
            self._integral = 0.0
            self.dbg_delta = 0.0
            return 0.0

        e_r = target_roll_rad - roll_rad
        # 几何项: 使机身转回水平所需的单侧行程 (半轮距×tan)
        delta = self.kp * (self.wheel_track * 0.5) * math.tan(_clamp(e_r, -0.7, 0.7))
        # I 项: 单边地形是恒值扰动, 靠积分保持差动行程消除稳态误差
        self._integral = _clamp(self._integral + e_r * dt * self.ki,
                                -self.i_limit, self.i_limit)
        delta += self._integral
        # D 项: 横滚角速度阻尼 (与 e_r 同向 → gyro 取反)
        delta += self.kd * (-gyro_roll)

        delta = _clamp(delta, -self.delta_limit, self.delta_limit)
        self.dbg_delta = delta
        return delta

    def reset(self):
        self._integral = 0.0
        self.dbg_delta = 0.0


# =====================================================================
#  跳跃状态机 → 腿高设定值（对标固件 RobotJumping, 6 状态）
# =====================================================================

class JumpController:
    """
    跳跃状态机 (完全对标固件 RobotJumping, 6 状态), 输出腿高设定值 set_h。

    固件流程:
      0 IDLE      待机, set_h = DefaultHeight
      1 SQUAT     下蹲: 斜坡降到 crouch_h (固件 SquattingSpeed=0.3 m/s), 到位→LAUNCH
      2 LAUNCH    蹬伸: 斜坡升到 extend_h (固件 AscendingVelocity=25 m/s, 几乎瞬时),
                  到位或离地→RETRACT
      3 RETRACT   缩腿: set_h 直接置 crouch_h (空中收腿留落地行程),
                  到位或重新触地→LAND_WAIT
      4 LAND_WAIT 等待落地: 触地连续防抖计数 (固件 js>5) →RECOVER
      5 RECOVER   恢复: 斜坡回 DefaultHeight, 到位→IDLE

    配套控制 (固件 LeggedRobot.ino 行3616-3640):
      - 状态 2/3/4: 速度环冻结 (JumpingState=0, 积分停止, ik_x 保持)
      - 状态 1/4/5: 髋关节软增益 (固件 Motor_KP 50→25, 落地缓冲)
    """
    IDLE, SQUAT, LAUNCH, RETRACT, LAND_WAIT, RECOVER = range(6)

    def __init__(self, default_h: float = 0.25, crouch_h: float = 0.15,
                 extend_h: float = 0.36, squat_speed: float = 0.3,
                 ascend_speed: float = 25.0, land_debounce: int = 25):
        self.default_h = default_h
        self.crouch_h = crouch_h        # 固件 ParallelHigh
        self.extend_h = extend_h        # 固件 MaxHigh (留 0.01 远离 IK 奇异)
        self.squat_speed = squat_speed  # 固件 SquattingSpeed
        self.ascend_speed = ascend_speed  # 固件 AscendingVelocity
        self.land_debounce = land_debounce  # 落地防抖步数 (固件 js>5 @50Hz, 500Hz下等效加宽)

        self.step = self.IDLE           # 固件 body.JumpingStep
        self._set_h = default_h
        self._land_count = 0            # 固件 js

    def trigger(self, on_ground: bool) -> bool:
        """请求起跳 (固件: 仅双脚在地且空闲时响应)"""
        if self.step == self.IDLE and on_ground:
            self.step = self.SQUAT
            self._land_count = 0
            return True
        return False

    def update(self, h_meas: float, on_ground: bool, dt: float) -> float:
        """
        状态机单步 (固件 RobotJumping)

        :param h_meas:    实测腿高 m (FK 双腿 y 均值, 固件 (Left+Right)*0.5)
        :param on_ground: 触地标志 (固件 LegOffGround[2]==1 = 腿部受载)
        :param dt:        控制周期 s
        :return:          腿高设定值 set_h (m)
        """
        if self.step == self.IDLE:
            self._set_h = self.default_h
            self._land_count = 0

        elif self.step == self.SQUAT:
            max_step = abs(dt * self.squat_speed)
            self._set_h += _clamp(self.crouch_h - self._set_h, -max_step, max_step)
            if abs(h_meas - self.crouch_h) < 0.005:
                self.step = self.LAUNCH

        elif self.step == self.LAUNCH:
            max_step = abs(dt * self.ascend_speed)
            self._set_h += _clamp(self.extend_h - self._set_h, -max_step, max_step)
            if abs(h_meas - self.extend_h) < 0.005 or not on_ground:
                self.step = self.RETRACT

        elif self.step == self.RETRACT:
            self._set_h = self.crouch_h   # 固件: 直接置位 (斜坡版被固件注释掉)
            if abs(h_meas - self.crouch_h) < 0.005 or on_ground:
                self.step = self.LAND_WAIT

        elif self.step == self.LAND_WAIT:
            if on_ground:
                self._land_count += 1
                if self._land_count > self.land_debounce:
                    self.step = self.RECOVER
            else:
                self._land_count = 0

        elif self.step == self.RECOVER:
            max_step = abs(dt * self.squat_speed)
            self._set_h += _clamp(self.default_h - self._set_h, -max_step, max_step)
            if abs(h_meas - self.default_h) < 0.01:
                self.step = self.IDLE

        self._set_h = _clamp(self._set_h, MIN_LOW, MAX_HIGH)
        return self._set_h

    @property
    def active(self) -> bool:
        """跳跃进行中 (非待机)"""
        return self.step != self.IDLE

    @property
    def freeze_speed(self) -> bool:
        """速度环冻结 (固件 JumpingState=0: 蹬伸/缩腿/等待落地)"""
        return self.step in (self.LAUNCH, self.RETRACT, self.LAND_WAIT)

    @property
    def soft_legs(self) -> bool:
        """髋关节软增益 (固件 Motor_KP=25: 下蹲/等待落地/恢复)"""
        return self.step in (self.SQUAT, self.LAND_WAIT, self.RECOVER)

    @property
    def state_name(self) -> str:
        return ('待机', '下蹲', '蹬伸', '缩腿', '落地', '恢复')[self.step]

    def reset(self):
        self.step = self.IDLE
        self._set_h = self.default_h
        self._land_count = 0


# =====================================================================
#  组合入口: 单步腿高控制 → 髋关节 ctrl（复刻 main.py 主循环 2/4/5 段）
# =====================================================================

def leg_ctrl_step(current_h: float, target_h: float,
                  ik_x: float, pitch_sp_deg: float,
                  roll_delta: float, dt: float,
                  jump: JumpController | None = None,
                  h_meas: float | None = None,
                  on_ground: bool = True) -> tuple[float, tuple, tuple]:
    """
    一步「腿高控制」完整计算 (供 ROS2 节点直接调用)。

    :param current_h:    当前腿高 (m), 调用者保存跨步状态
    :param target_h:     腿高目标 (m, 面板/遥控)
    :param ik_x:         速度环输出的足端前后偏移 (m)
    :param pitch_sp_deg: 俯仰目标 (度, 已平滑; 喂负号进 IK, 见 main.py 注)
    :param roll_delta:   横滚差动量 (m, RollController.compute 输出; 0=关闭)
    :param dt:           控制周期 (s)
    :param jump:         跳跃状态机实例 (None=禁用跳跃接管)
    :param h_meas:       实测腿高 (m, measure_leg_height 输出; 供状态机)
    :param on_ground:    触地标志
    :return: (current_h_new, (q_lf, q_lb), (q_rf, q_rb))
    """
    # 1. 腿高目标: 跳跃中由状态机接管, 否则斜坡插值 (main.py 1045-1051)
    if jump is not None and jump.active:
        assert h_meas is not None, "跳跃接管时需要 h_meas"
        set_h = jump.update(h_meas, on_ground, dt)
    else:
        set_h = leg_height_ramp(current_h, target_h, dt)
    current_h = set_h

    # 2. 横滚差动 → 左右腿高 (main.py 1090-1091)
    h_l = _clamp(current_h + roll_delta, MIN_LOW, MAX_HIGH)
    h_r = _clamp(current_h - roll_delta, MIN_LOW, MAX_HIGH)

    # 3. IK 反解 (main.py 1093-1098: 仿真 pitch 轴约定与固件相反, 喂 -pitch_sp)
    alpha_r, beta_r, err_r = right_ik(ik_x, h_r, -pitch_sp_deg)
    alpha_l, beta_l, err_l = left_ik(ik_x, h_l, -pitch_sp_deg)

    # 4. 角度映射 + 髋关节限幅 (main.py 1100-1106)
    q_lf, q_lb = firmware_to_mujoco_left(alpha_l, beta_l)
    q_rf, q_rb = firmware_to_mujoco_right(alpha_r, beta_r)
    q_lf = _clamp(q_lf, 0.0, 1.54)
    q_lb = _clamp(q_lb, 0.0, 1.54)
    q_rf = _clamp(q_rf, 0.0, 1.54)
    q_rb = _clamp(q_rb, 0.0, 1.54)

    return current_h, (q_lf, q_lb), (q_rf, q_rb)


# =====================================================================
#  数值自检 (等价 main.py 429-445 的 IK 自检段)
# =====================================================================

def _self_test():
    print("====== IK 数值自检 ======")
    for h_test in [0.13, 0.20, 0.25, 0.30, 0.37]:
        a_r, b_r, e_r = right_ik(0.0, h_test, 0.0)
        a_l, b_l, e_l = left_ik(0.0, h_test, 0.0)
        q_lf, q_lb = firmware_to_mujoco_left(a_l, b_l)
        q_rf, q_rb = firmware_to_mujoco_right(a_r, b_r)
        # FK 往返验证
        fx_l, fy_l = forward_kinematics(a_l, b_l)
        fx_r, fy_r = forward_kinematics(a_r, b_r)
        print(f"  h={h_test:.2f}m: "
              f"R_ik=({a_r:.2f}°, {b_r:.2f}°) "
              f"L_ik=({a_l:.2f}°, {b_l:.2f}°) "
              f"ctrl=[{q_lf:.4f}, {q_lb:.4f}, {q_rf:.4f}, {q_rb:.4f}] "
              f"err=({e_r},{e_l}) "
              f"FK_y=({fy_l:.4f},{fy_r:.4f})")

    a0, b0, _ = right_ik(0.0, DEFAULT_HEIGHT, 0.0)
    q0 = math.pi / 2.0 - math.radians(a0)
    print(f"  站立验证: α_fw={a0:.2f}° → q_mj={q0:.4f} (keyframe=1.0080)")
    print(f"  误差: {abs(q0 - 1.008):.5f} rad")
    print("==========================")

    # 腿高斜坡插值验证
    h = 0.13
    for _ in range(20):
        h = leg_height_ramp(h, 0.25, 0.02)
    print(f"  斜坡插值: 0.13 → 0.25 目标, 20 步(0.02s)后 h={h:.4f} "
          f"(速率0.5m/s 理论 0.13+0.5*0.4=0.33, 限幅到目标 0.25)")

    # 组合入口验证 (站立, 无横滚/跳跃)
    h_new, (qlf, qlb), (qrf, qrb) = leg_ctrl_step(
        0.25, 0.25, ik_x=0.0, pitch_sp_deg=0.0, roll_delta=0.0, dt=0.02)
    print(f"  组合入口站立: h={h_new:.3f} q=({qlf:.4f},{qlb:.4f},{qrf:.4f},{qrb:.4f})")


if __name__ == "__main__":
    _self_test()
