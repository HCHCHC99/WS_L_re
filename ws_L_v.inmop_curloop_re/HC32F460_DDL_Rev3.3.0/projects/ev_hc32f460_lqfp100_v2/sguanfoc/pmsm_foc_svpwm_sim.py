"""
FOC + SVPWM + PMSM 电机模型学习仿真器

功能：
    1. PMSM 电流环 FOC
    2. Clarke / Park / IPark
    3. SVPWM
    4. 真实转子角度与估计转子角度
    5. Id / Iq 电流环
    6. RPM / 电磁转矩
    7. 功角
    8. SVPWM 扇区
    9. 实时示波器
   10. 示波器鼠标悬停读数
   11. 示波器滚轮缩放时间轴
   12. 总时间滑块历史回溯
   13. 历史状态恢复
   14. 0.01x ~ 100x 播放倍速
   15. 仿真异常不会导致窗口直接退出

说明：
    当前按照正弦 PMSM 模型处理实际 BLDC。
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import math
from dataclasses import dataclass


# ============================================================
# 常量
# ============================================================

PI = math.pi
TWO_PI = 2.0 * PI
SQRT3 = math.sqrt(3.0)
INV_SQRT3 = 1.0 / SQRT3


# ============================================================
# 电机参数
# ============================================================

POLE_PAIRS = 10

VBUS = 12.0

RS = 0.1

LS = 42.3e-6
LD = LS
LQ = LS

FLUX = 0.00084

KE_PHASE_PEAK = POLE_PAIRS * FLUX

KT = 1.5 * POLE_PAIRS * FLUX

J = 7.0e-6
B = 4.0e-5

T_LOAD = 0.0


# ============================================================
# 控制参数
# ============================================================

CONTROL_DT = 50e-6

GUI_INTERVAL_MS = 30

CURRENT_BW = 1500.0 * TWO_PI

KP_ID = LD * CURRENT_BW
KI_ID = RS * CURRENT_BW

KP_IQ = LQ * CURRENT_BW
KI_IQ = RS * CURRENT_BW

CURRENT_LIMIT = 5.0

MAX_VOLTAGE = VBUS / SQRT3


# ============================================================
# 示波器参数
# ============================================================

HISTORY_MAX_POINTS = 120000

SCOPE_MIN_WINDOW = 0.01
SCOPE_MAX_WINDOW = 10.0

SCOPE_DEFAULT_WINDOW = 0.50

SCOPE_ZOOM_FACTOR = 1.25

SCOPE_GRID_X_DIV = 10
SCOPE_GRID_Y_DIV = 8


# ============================================================
# 数学函数
# ============================================================

def normalize_angle(angle):

    while angle >= PI:
        angle -= TWO_PI

    while angle < -PI:
        angle += TWO_PI

    return angle


def clamp(value, minimum, maximum):

    if value > maximum:
        return maximum

    if value < minimum:
        return minimum

    return value


def wrap_360(angle):

    return angle % TWO_PI


def deg(angle):

    return math.degrees(angle)


# ============================================================
# Clarke
# ============================================================

def clarke(ia, ib, ic=None):

    alpha = ia

    beta = (
        ia +
        2.0 * ib
    ) * INV_SQRT3

    return alpha, beta


# ============================================================
# Park
# ============================================================

def park(alpha, beta, theta):

    s = math.sin(theta)
    c = math.cos(theta)

    d = alpha * c + beta * s

    q = beta * c - alpha * s

    return d, q


# ============================================================
# 逆 Park
# ============================================================

def inverse_park(d, q, theta):

    s = math.sin(theta)
    c = math.cos(theta)

    alpha = d * c - q * s

    beta = q * c + d * s

    return alpha, beta


# ============================================================
# Alpha Beta -> ABC
# ============================================================

def alpha_beta_to_abc(alpha, beta):

    ia = alpha

    ib = (
        -0.5 * alpha +
        0.5 * SQRT3 * beta
    )

    ic = (
        -0.5 * alpha -
        0.5 * SQRT3 * beta
    )

    return ia, ib, ic


# ============================================================
# PI
# ============================================================

class PIController:

    def __init__(
        self,
        kp,
        ki,
        output_limit
    ):

        self.kp = kp
        self.ki = ki
        self.output_limit = abs(output_limit)

        self.integral = 0.0

    def reset(self):

        self.integral = 0.0

    def update(
        self,
        reference,
        feedback,
        dt
    ):

        error = reference - feedback

        proportional = (
            self.kp *
            error
        )

        integral_new = (
            self.integral +
            self.ki *
            error *
            dt
        )

        output = (
            proportional +
            integral_new
        )

        if output > self.output_limit:

            output = self.output_limit

            if error < 0.0:
                self.integral = integral_new

        elif output < -self.output_limit:

            output = -self.output_limit

            if error > 0.0:
                self.integral = integral_new

        else:

            self.integral = integral_new

        return output


# ============================================================
# SVPWM
# ============================================================

@dataclass
class SVPWMResult:

    alpha: float
    beta: float

    magnitude: float
    angle: float

    sector: int

    duty_a: float
    duty_b: float
    duty_c: float

    va: float
    vb: float
    vc: float

    utilization: float


def get_sector(angle):

    angle = wrap_360(angle)

    sector = (
        int(angle / (PI / 3.0)) +
        1
    )

    if sector > 6:
        sector = 6

    return sector


def svpwm(alpha, beta, vbus):

    magnitude = math.sqrt(
        alpha * alpha +
        beta * beta
    )

    angle = math.atan2(
        beta,
        alpha
    )

    sector = get_sector(angle)

    max_voltage = (
        vbus /
        SQRT3
    )

    scale = 1.0

    if magnitude > max_voltage:

        scale = (
            max_voltage /
            magnitude
        )

        alpha *= scale
        beta *= scale

        magnitude = max_voltage

    va = alpha

    vb = (
        -0.5 * alpha +
        0.5 * SQRT3 * beta
    )

    vc = (
        -0.5 * alpha -
        0.5 * SQRT3 * beta
    )

    vmax = max(
        va,
        vb,
        vc
    )

    vmin = min(
        va,
        vb,
        vc
    )

    vzero = (
        -0.5 *
        (vmax + vmin)
    )

    va_mod = va + vzero
    vb_mod = vb + vzero
    vc_mod = vc + vzero

    duty_a = (
        0.5 +
        va_mod / vbus
    )

    duty_b = (
        0.5 +
        vb_mod / vbus
    )

    duty_c = (
        0.5 +
        vc_mod / vbus
    )

    duty_a = clamp(
        duty_a,
        0.0,
        1.0
    )

    duty_b = clamp(
        duty_b,
        0.0,
        1.0
    )

    duty_c = clamp(
        duty_c,
        0.0,
        1.0
    )

    utilization = (
        magnitude /
        max_voltage
        if max_voltage > 0.0
        else 0.0
    )

    return SVPWMResult(
        alpha=alpha,
        beta=beta,
        magnitude=magnitude,
        angle=angle,
        sector=sector,
        duty_a=duty_a,
        duty_b=duty_b,
        duty_c=duty_c,
        va=va_mod,
        vb=vb_mod,
        vc=vc_mod,
        utilization=utilization
    )


# ============================================================
# PMSM
# ============================================================

class PMSMMotor:

    def __init__(self):

        self.reset()

    def reset(
        self,
        theta_mech=0.0,
        omega_mech=0.0
    ):

        self.theta_mech = theta_mech

        self.omega_mech = omega_mech

        self.id = 0.0
        self.iq = 0.0

        self.ia = 0.0
        self.ib = 0.0
        self.ic = 0.0

        self.te = 0.0

        self.vd_real = 0.0
        self.vq_real = 0.0

    @property
    def theta_elec(self):

        return normalize_angle(
            POLE_PAIRS *
            self.theta_mech
        )

    @property
    def omega_elec(self):

        return (
            POLE_PAIRS *
            self.omega_mech
        )

    def step(
        self,
        v_alpha,
        v_beta,
        dt,
        load_torque
    ):

        theta = self.theta_elec

        omega_e = self.omega_elec

        vd_real, vq_real = park(
            v_alpha,
            v_beta,
            theta
        )

        self.vd_real = vd_real
        self.vq_real = vq_real

        id_old = self.id
        iq_old = self.iq

        did_dt = (
            vd_real
            - RS * id_old
            + omega_e * LQ * iq_old
        ) / LD

        diq_dt = (
            vq_real
            - RS * iq_old
            - omega_e *
            (
                LD * id_old +
                FLUX
            )
        ) / LQ

        self.id += (
            did_dt *
            dt
        )

        self.iq += (
            diq_dt *
            dt
        )

        current_mag = math.sqrt(
            self.id * self.id +
            self.iq * self.iq
        )

        if current_mag > 20.0:

            scale = (
                20.0 /
                current_mag
            )

            self.id *= scale
            self.iq *= scale

        self.te = (
            1.5 *
            POLE_PAIRS *
            (
                FLUX * self.iq
                +
                (LD - LQ) *
                self.id *
                self.iq
            )
        )

        domega_dt = (
            self.te
            - load_torque
            - B * self.omega_mech
        ) / J

        self.omega_mech += (
            domega_dt *
            dt
        )

        self.theta_mech += (
            self.omega_mech *
            dt
        )

        self.theta_mech = normalize_angle(
            self.theta_mech
        )

        i_alpha, i_beta = inverse_park(
            self.id,
            self.iq,
            theta
        )

        (
            self.ia,
            self.ib,
            self.ic
        ) = alpha_beta_to_abc(
            i_alpha,
            i_beta
        )

        return {
            "theta_real": self.theta_elec,
            "omega_mech": self.omega_mech,
            "omega_elec": self.omega_elec,
            "id_real": self.id,
            "iq_real": self.iq,
            "ia": self.ia,
            "ib": self.ib,
            "ic": self.ic,
            "te": self.te,
            "vd_real": vd_real,
            "vq_real": vq_real
        }


# ============================================================
# Observer
# ============================================================

class Observer:

    def __init__(self):

        self.offset_deg = 0.0
        self.delay_us = 0.0
        self.speed_error = 0.0

        self.filtered_angle = 0.0

    def reset(
        self,
        theta_real
    ):

        self.filtered_angle = theta_real

    def update(
        self,
        theta_real,
        omega_real,
        dt
    ):

        theta = normalize_angle(
            theta_real +
            math.radians(
                self.offset_deg
            )
        )

        delay_sec = (
            self.delay_us *
            1e-6
        )

        theta -= (
            omega_real *
            delay_sec
        )

        omega_est = (
            omega_real +
            self.speed_error
        )

        theta = normalize_angle(theta)

        self.filtered_angle = theta

        return theta, omega_est


# ============================================================
# 仿真显示数据
# ============================================================

@dataclass
class SimulationData:

    theta_real: float = 0.0
    theta_est: float = 0.0
    theta_error: float = 0.0

    omega_mech: float = 0.0
    omega_elec: float = 0.0

    ia: float = 0.0
    ib: float = 0.0
    ic: float = 0.0

    id_est: float = 0.0
    iq_est: float = 0.0

    id_real: float = 0.0
    iq_real: float = 0.0

    id_ref: float = 0.0
    iq_ref: float = 0.0

    vd: float = 0.0
    vq: float = 0.0

    v_alpha: float = 0.0
    v_beta: float = 0.0

    v_mag: float = 0.0
    v_angle: float = 0.0

    sector: int = 1

    duty_a: float = 0.5
    duty_b: float = 0.5
    duty_c: float = 0.5

    te: float = 0.0

    power_utilization: float = 0.0

    delta: float = 0.0


# ============================================================
# 历史记录
# ============================================================

@dataclass
class HistoryRecord:

    time: float

    data: SimulationData

    motor_theta_mech: float
    motor_omega_mech: float

    motor_id: float
    motor_iq: float

    motor_ia: float
    motor_ib: float
    motor_ic: float

    motor_te: float

    observer_angle: float

    pi_d_integral: float
    pi_q_integral: float


class HistoryBuffer:

    def __init__(self):

        self.records = []

    def clear(self):

        self.records.clear()

    def append(
        self,
        time,
        data,
        motor,
        observer,
        pi_d,
        pi_q
    ):

        record = HistoryRecord(
            time=time,
            data=SimulationData(
                **data.__dict__
            ),
            motor_theta_mech=motor.theta_mech,
            motor_omega_mech=motor.omega_mech,
            motor_id=motor.id,
            motor_iq=motor.iq,
            motor_ia=motor.ia,
            motor_ib=motor.ib,
            motor_ic=motor.ic,
            motor_te=motor.te,
            observer_angle=observer.filtered_angle,
            pi_d_integral=pi_d.integral,
            pi_q_integral=pi_q.integral
        )

        self.records.append(record)

        if len(self.records) > HISTORY_MAX_POINTS:

            del self.records[
                :len(self.records) -
                HISTORY_MAX_POINTS
            ]

    def get_times(self):

        return [
            r.time
            for r in self.records
        ]

    def nearest_index(
        self,
        target_time
    ):

        if not self.records:
            return -1

        low = 0
        high = len(self.records) - 1

        while low < high:

            mid = (
                low +
                high
            ) // 2

            if self.records[mid].time < target_time:

                low = mid + 1

            else:

                high = mid

        if low <= 0:
            return 0

        before = low - 1
        after = low

        if abs(
            self.records[before].time -
            target_time
        ) <= abs(
            self.records[after].time -
            target_time
        ):

            return before

        return after

    def get_record(
        self,
        target_time
    ):

        index = self.nearest_index(
            target_time
        )

        if index < 0:
            return None

        return self.records[index]


# ============================================================
# 示波器
# ============================================================

class Oscilloscope(tk.Canvas):

    def __init__(
        self,
        master,
        simulator,
        **kwargs
    ):

        super().__init__(
            master,
            bg="#101418",
            highlightthickness=1,
            highlightbackground="#444444",
            **kwargs
        )

        self.simulator = simulator

        self.time_window = SCOPE_DEFAULT_WINDOW

        self.mouse_x = None
        self.mouse_y = None

        self.last_width = 1
        self.last_height = 1

        self.bind(
            "<Motion>",
            self.on_mouse_move
        )

        self.bind(
            "<Leave>",
            self.on_mouse_leave
        )

        self.bind(
            "<MouseWheel>",
            self.on_mouse_wheel
        )

        self.bind(
            "<Button-4>",
            self.on_mouse_wheel
        )

        self.bind(
            "<Button-5>",
            self.on_mouse_wheel
        )

    # --------------------------------------------------------
    # 滚轮缩放
    # --------------------------------------------------------

    def on_mouse_wheel(
        self,
        event
    ):

        if hasattr(
            event,
            "delta"
        ):

            if event.delta > 0:

                direction = -1

            elif event.delta < 0:

                direction = 1

            else:

                return

        else:

            if event.num == 4:

                direction = -1

            else:

                direction = 1

        old_window = self.time_window

        if direction < 0:

            self.time_window /= SCOPE_ZOOM_FACTOR

        else:

            self.time_window *= SCOPE_ZOOM_FACTOR

        self.time_window = clamp(
            self.time_window,
            SCOPE_MIN_WINDOW,
            SCOPE_MAX_WINDOW
        )

        if abs(
            self.time_window -
            old_window
        ) > 1e-12:

            self.draw()

    # --------------------------------------------------------
    # 鼠标
    # --------------------------------------------------------

    def on_mouse_move(
        self,
        event
    ):

        self.mouse_x = event.x
        self.mouse_y = event.y

        self.draw()

    def on_mouse_leave(
        self,
        event
    ):

        self.mouse_x = None
        self.mouse_y = None

        self.draw()

    # --------------------------------------------------------
    # 当前查看时间
    # --------------------------------------------------------

    def get_view_end(self):

        return self.simulator.get_scope_end_time()

    def get_view_start(self):

        end = self.get_view_end()

        return max(
            0.0,
            end -
            self.time_window
        )

    # --------------------------------------------------------
    # 坐标
    # --------------------------------------------------------

    def plot_area(self):

        width = max(
            self.winfo_width(),
            500
        )

        height = max(
            self.winfo_height(),
            250
        )

        left = 65
        right = 20
        top = 25
        bottom = 35

        return (
            left,
            top,
            width - right,
            height - bottom
        )

    # --------------------------------------------------------
    # 数据映射
    # --------------------------------------------------------

    def time_to_x(
        self,
        t,
        start,
        end,
        x0,
        x1
    ):

        if end <= start:
            return x0

        return (
            x0 +
            (t - start) /
            (end - start) *
            (x1 - x0)
        )

    def value_to_y(
        self,
        value,
        ymin,
        ymax,
        y0,
        y1
    ):

        if ymax <= ymin:
            return (
                y0 +
                y1
            ) * 0.5

        return (
            y1 -
            (value - ymin) /
            (ymax - ymin) *
            (y1 - y0)
        )

    # --------------------------------------------------------
    # 网格
    # --------------------------------------------------------

    def draw_grid(
        self,
        x0,
        y0,
        x1,
        y1,
        start,
        end
    ):

        self.create_rectangle(
            x0,
            y0,
            x1,
            y1,
            outline="#394047"
        )

        for i in range(
            SCOPE_GRID_X_DIV + 1
        ):

            x = (
                x0 +
                (x1 - x0) *
                i /
                SCOPE_GRID_X_DIV
            )

            self.create_line(
                x,
                y0,
                x,
                y1,
                fill="#252B30"
            )

            t = (
                start +
                (end - start) *
                i /
                SCOPE_GRID_X_DIV
            )

            self.create_text(
                x,
                y1 + 15,
                text=f"{t:.3f}",
                fill="#AAB2B8",
                font=("Consolas", 8)
            )

        for i in range(
            SCOPE_GRID_Y_DIV + 1
        ):

            y = (
                y0 +
                (y1 - y0) *
                i /
                SCOPE_GRID_Y_DIV
            )

            self.create_line(
                x0,
                y,
                x1,
                y,
                fill="#252B30"
            )

    # --------------------------------------------------------
    # 自动 Y 范围
    # --------------------------------------------------------

    def get_series(
        self,
        record
    ):

        d = record.data

        return [
            d.omega_mech *
            60.0 /
            TWO_PI,

            d.iq_real,

            d.id_real,

            deg(d.theta_real),

            deg(d.theta_est),

            deg(d.delta),

            d.te
        ]

    # --------------------------------------------------------
    # 数据范围
    # --------------------------------------------------------

    def get_range(
        self,
        records,
        index
    ):

        values = []

        for record in records:

            series = self.get_series(
                record
            )

            values.append(
                series[index]
            )

        if not values:

            return -1.0, 1.0

        vmin = min(values)
        vmax = max(values)

        if abs(
            vmax - vmin
        ) < 1e-9:

            margin = max(
                1.0,
                abs(vmax) * 0.2
            )

        else:

            margin = (
                vmax - vmin
            ) * 0.12

        return (
            vmin - margin,
            vmax + margin
        )

    # --------------------------------------------------------
    # 绘制曲线
    # --------------------------------------------------------

    def draw_series(
        self,
        records,
        series_index,
        color,
        x0,
        y0,
        x1,
        y1,
        start,
        end,
        ymin,
        ymax
    ):

        if not records:
            return

        points = []

        width = max(
            x1 - x0,
            1
        )

        # ----------------------------------------------------
        # 根据屏幕像素进行降采样
        #
        # 不让几十万个历史点全部送入 Canvas。
        # ----------------------------------------------------

        target_points = int(
            width * 1.5
        )

        stride = max(
            1,
            len(records) //
            max(
                target_points,
                1
            )
        )

        for i in range(
            0,
            len(records),
            stride
        ):

            record = records[i]

            if (
                record.time <
                start
            ):
                continue

            if (
                record.time >
                end
            ):
                break

            value = self.get_series(
                record
            )[series_index]

            x = self.time_to_x(
                record.time,
                start,
                end,
                x0,
                x1
            )

            y = self.value_to_y(
                value,
                ymin,
                ymax,
                y0,
                y1
            )

            points.extend([
                x,
                y
            ])

        if len(points) >= 4:

            self.create_line(
                points,
                fill=color,
                width=1.5,
                smooth=False
            )

    # --------------------------------------------------------
    # 鼠标读数
    # --------------------------------------------------------

    def draw_cursor(
        self,
        records,
        x0,
        y0,
        x1,
        y1,
        start,
        end
    ):

        if (
            self.mouse_x is None or
            self.mouse_y is None
        ):
            return

        if not (
            x0 <= self.mouse_x <= x1 and
            y0 <= self.mouse_y <= y1
        ):
            return

        ratio = (
            self.mouse_x -
            x0
        ) / max(
            x1 - x0,
            1
        )

        target_time = (
            start +
            ratio *
            (end - start)
        )

        if not records:
            return

        nearest = None
        nearest_distance = float("inf")

        for record in records:

            if record.time < start:
                continue

            if record.time > end:
                break

            distance = abs(
                record.time -
                target_time
            )

            if distance < nearest_distance:

                nearest_distance = distance
                nearest = record

        if nearest is None:
            return

        cursor_x = self.time_to_x(
            nearest.time,
            start,
            end,
            x0,
            x1
        )

        self.create_line(
            cursor_x,
            y0,
            cursor_x,
            y1,
            fill="#E6E6E6",
            dash=(3, 3)
        )

        d = nearest.data

        values = [
            ("RPM", d.omega_mech * 60.0 / TWO_PI),
            ("Iq", d.iq_real),
            ("Id", d.id_real),
            ("θreal", deg(d.theta_real)),
            ("θest", deg(d.theta_est)),
            ("δ", deg(d.delta)),
            ("Te", d.te)
        ]

        text_lines = [
            f"t = {nearest.time:.6f} s"
        ]

        units = [
            "rpm",
            "A",
            "A",
            "deg",
            "deg",
            "deg",
            "N·m"
        ]

        for (
            item,
            unit
        ), unit_text in zip(
            values,
            units
        ):

            text_lines.append(
                f"{item:6s} = {unit:+.5f} {unit_text}"
            )

        text = "\n".join(
            text_lines
        )

        box_x = cursor_x + 10

        box_y = y0 + 10

        box_width = 170

        box_height = (
            18 *
            len(text_lines) +
            10
        )

        if (
            box_x +
            box_width >
            x1
        ):

            box_x = (
                cursor_x -
                box_width -
                10
            )

        self.create_rectangle(
            box_x,
            box_y,
            box_x + box_width,
            box_y + box_height,
            fill="#151A1E",
            outline="#777777"
        )

        self.create_text(
            box_x + 8,
            box_y + 7,
            text=text,
            anchor="nw",
            fill="#E5E5E5",
            font=("Consolas", 9)
        )

    # --------------------------------------------------------
    # 绘制
    # --------------------------------------------------------

    def draw(self):

        self.delete(
            "all"
        )

        (
            x0,
            y0,
            x1,
            y1
        ) = self.plot_area()

        self.last_width = self.winfo_width()
        self.last_height = self.winfo_height()

        end = self.get_view_end()

        start = max(
            0.0,
            end -
            self.time_window
        )

        records = (
            self.simulator.history.records
        )

        visible = []

        for record in records:

            if record.time >= start:

                if record.time <= end:

                    visible.append(
                        record
                    )

        self.draw_grid(
            x0,
            y0,
            x1,
            y1,
            start,
            end
        )

        if not visible:

            self.create_text(
                (
                    x0 + x1
                ) * 0.5,
                (
                    y0 + y1
                ) * 0.5,
                text="等待仿真数据...",
                fill="#889097",
                font=("Microsoft YaHei", 11)
            )

            return

        # ----------------------------------------------------
        # 每个通道单独 Y 轴
        #
        # 为了避免 RPM 把电流曲线压扁，
        # 每条曲线使用自己的 Y 缩放。
        # ----------------------------------------------------

        colors = [
            "#F5C542",
            "#35D07F",
            "#4EA5FF",
            "#FF6B6B",
            "#B88CFF",
            "#FF8C42",
            "#FFFFFF"
        ]

        names = [
            "RPM",
            "Iq",
            "Id",
            "θreal",
            "θest",
            "δ",
            "Te"
        ]

        units = [
            "rpm",
            "A",
            "A",
            "deg",
            "deg",
            "deg",
            "N·m"
        ]

        # ----------------------------------------------------
        # 第一条曲线作为主绘图区 Y 范围
        # ----------------------------------------------------

        ymin, ymax = self.get_range(
            visible,
            0
        )

        # ----------------------------------------------------
        # 绘制各曲线
        #
        # 不同物理量使用归一化映射，
        # 这样可以在一个示波器窗口观察。
        # ----------------------------------------------------

        for series_index in range(7):

            values = [
                self.get_series(
                    r
                )[series_index]
                for r in visible
            ]

            if not values:
                continue

            vmin = min(values)
            vmax = max(values)

            if abs(
                vmax - vmin
            ) < 1e-9:

                margin = max(
                    1.0,
                    abs(vmax) * 0.2
                )

            else:

                margin = (
                    vmax -
                    vmin
                ) * 0.12

            local_min = vmin - margin
            local_max = vmax + margin

            # ------------------------------------------------
            # 用独立的 Y 范围画曲线
            # ------------------------------------------------

            self.draw_series(
                visible,
                series_index,
                colors[series_index],
                x0,
                y0,
                x1,
                y1,
                start,
                end,
                local_min,
                local_max
            )

        # ----------------------------------------------------
        # 左上角通道说明
        # ----------------------------------------------------

        legend_x = x0 + 8
        legend_y = y0 + 8

        for i in range(7):

            self.create_text(
                legend_x,
                legend_y + i * 16,
                text=(
                    f"{names[i]:6s} "
                    f"({units[i]})"
                ),
                anchor="w",
                fill=colors[i],
                font=("Consolas", 8)
            )

        # ----------------------------------------------------
        # 当前时间窗口
        # ----------------------------------------------------

        self.create_text(
            x1,
            8,
            text=(
                f"窗口 = {self.time_window:.3f} s"
                f"    "
                f"{start:.3f} ~ {end:.3f} s"
            ),
            anchor="ne",
            fill="#D0D6DA",
            font=("Consolas", 9)
        )

        # ----------------------------------------------------
        # 鼠标十字光标
        # ----------------------------------------------------

        self.draw_cursor(
            visible,
            x0,
            y0,
            x1,
            y1,
            start,
            end
        )


# ============================================================
# Vector Canvas
# ============================================================

class VectorCanvas(tk.Canvas):

    def __init__(
        self,
        master,
        **kwargs
    ):

        super().__init__(
            master,
            bg="#FAFAFA",
            highlightthickness=1,
            highlightbackground="#CCCCCC",
            **kwargs
        )

        self.cx = 400
        self.cy = 330

        self.radius = 250

        self.data = SimulationData()

        self.bind(
            "<Configure>",
            self.on_resize
        )

    def on_resize(
        self,
        event
    ):

        self.cx = max(
            300,
            event.width * 0.5
        )

        self.cy = max(
            250,
            event.height * 0.45
        )

        self.radius = min(
            event.width * 0.32,
            event.height * 0.32,
            280
        )

        if hasattr(
            self,
            "_draw_pending"
        ):

            return

        self._draw_pending = True

        self.after_idle(
            self.redraw_after_resize
        )

    def redraw_after_resize(self):

        self._draw_pending = False

        self.update(
            self.data
        )

    def xy(
        self,
        angle,
        radius
    ):

        return (
            self.cx +
            radius *
            math.cos(angle),

            self.cy -
            radius *
            math.sin(angle)
        )

    def draw_arrow(
        self,
        angle,
        length,
        color,
        width=3,
        label=None
    ):

        x1 = self.cx
        y1 = self.cy

        x2, y2 = self.xy(
            angle,
            length
        )

        self.create_line(
            x1,
            y1,
            x2,
            y2,
            fill=color,
            width=width
        )

        arrow_size = 10

        a1 = (
            angle +
            PI * 0.85
        )

        a2 = (
            angle -
            PI * 0.85
        )

        p1 = (
            x2,
            y2
        )

        p2 = (
            x2 +
            arrow_size *
            math.cos(a1),

            y2 -
            arrow_size *
            math.sin(a1)
        )

        p3 = (
            x2 +
            arrow_size *
            math.cos(a2),

            y2 -
            arrow_size *
            math.sin(a2)
        )

        self.create_polygon(
            p1,
            p2,
            p3,
            fill=color,
            outline=color
        )

        if label:

            lx, ly = self.xy(
                angle,
                length + 20
            )

            self.create_text(
                lx,
                ly,
                text=label,
                fill=color,
                font=(
                    "Microsoft YaHei",
                    10,
                    "bold"
                )
            )

    def draw_axes(self):

        self.create_line(
            self.cx -
            self.radius -
            30,
            self.cy,
            self.cx +
            self.radius +
            30,
            self.cy,
            fill="#777777",
            dash=(4, 4)
        )

        self.create_line(
            self.cx,
            self.cy +
            self.radius +
            30,
            self.cx,
            self.cy -
            self.radius -
            30,
            fill="#777777",
            dash=(4, 4)
        )

        self.create_text(
            self.cx +
            self.radius +
            20,
            self.cy - 15,
            text="α",
            font=(
                "Arial",
                12,
                "bold"
            )
        )

        self.create_text(
            self.cx + 15,
            self.cy -
            self.radius -
            20,
            text="β",
            font=(
                "Arial",
                12,
                "bold"
            )
        )

    def draw_phase_axes(self):

        angles = [
            0.0,
            2.0 * PI / 3.0,
            4.0 * PI / 3.0
        ]

        names = [
            "A",
            "B",
            "C"
        ]

        colors = [
            "#DD2222",
            "#228822",
            "#2222CC"
        ]

        for (
            angle,
            name,
            color
        ) in zip(
            angles,
            names,
            colors
        ):

            x, y = self.xy(
                angle,
                self.radius
            )

            self.create_line(
                self.cx,
                self.cy,
                x,
                y,
                fill=color,
                dash=(5, 5),
                width=1
            )

            tx, ty = self.xy(
                angle,
                self.radius + 25
            )

            self.create_text(
                tx,
                ty,
                text=name,
                fill=color,
                font=(
                    "Arial",
                    10,
                    "bold"
                )
            )

    def draw_hexagon(self):

        points = []

        for i in range(6):

            angle = (
                i *
                PI /
                3.0
            )

            x, y = self.xy(
                angle,
                self.radius
            )

            points.extend([
                x,
                y
            ])

        self.create_polygon(
            points,
            outline="#BBBBBB",
            fill="",
            width=2
        )

        for i in range(6):

            angle = (
                i *
                PI /
                3.0 +
                PI /
                6.0
            )

            x, y = self.xy(
                angle,
                self.radius * 0.78
            )

            self.create_text(
                x,
                y,
                text=f"S{i + 1}",
                fill="#999999",
                font=(
                    "Arial",
                    10,
                    "bold"
                )
            )

    def draw_rotor(
        self,
        theta
    ):

        rotor_radius = (
            self.radius *
            0.55
        )

        self.create_oval(
            self.cx -
            rotor_radius,
            self.cy -
            rotor_radius,
            self.cx +
            rotor_radius,
            self.cy +
            rotor_radius,
            outline="#555555",
            width=2
        )

        self.draw_arrow(
            theta,
            rotor_radius,
            "#9B27B0",
            width=3,
            label="d(real)"
        )

        q_angle = (
            theta +
            PI / 2.0
        )

        self.create_line(
            self.cx,
            self.cy,
            *self.xy(
                q_angle,
                rotor_radius * 0.8
            ),
            fill="#9B27B0",
            dash=(5, 3),
            width=2
        )

        self.create_text(
            *self.xy(
                q_angle,
                rotor_radius * 0.9
            ),
            text="q(real)",
            fill="#9B27B0",
            font=(
                "Arial",
                9
            )
        )

        nx, ny = self.xy(
            theta,
            rotor_radius * 0.35
        )

        self.create_oval(
            nx - 12,
            ny - 12,
            nx + 12,
            ny + 12,
            fill="#FF4444",
            outline="#AA0000"
        )

        self.create_text(
            nx,
            ny,
            text="N",
            fill="white",
            font=(
                "Arial",
                9,
                "bold"
            )
        )

        sx, sy = self.xy(
            theta + PI,
            rotor_radius * 0.35
        )

        self.create_oval(
            sx - 12,
            sy - 12,
            sx + 12,
            sy + 12,
            fill="#4488FF",
            outline="#000088"
        )

        self.create_text(
            sx,
            sy,
            text="S",
            fill="white",
            font=(
                "Arial",
                9,
                "bold"
            )
        )

    def draw_estimated_axis(
        self,
        theta
    ):

        length = (
            self.radius *
            0.78
        )

        self.create_line(
            self.cx,
            self.cy,
            *self.xy(
                theta,
                length
            ),
            fill="#0066FF",
            dash=(7, 4),
            width=2
        )

        x, y = self.xy(
            theta,
            length
        )

        self.create_text(
            x,
            y,
            text="d(est)",
            fill="#0066FF",
            font=(
                "Arial",
                10,
                "bold"
            )
        )

        q_angle = (
            theta +
            PI / 2.0
        )

        self.create_line(
            self.cx,
            self.cy,
            *self.xy(
                q_angle,
                length * 0.65
            ),
            fill="#0066FF",
            dash=(4, 3),
            width=1
        )

        xq, yq = self.xy(
            q_angle,
            length * 0.7
        )

        self.create_text(
            xq,
            yq,
            text="q(est)",
            fill="#0066FF",
            font=(
                "Arial",
                9
            )
        )

    def draw_current_vector(
        self,
        data
    ):

        magnitude = math.sqrt(
            data.id_real ** 2 +
            data.iq_real ** 2
        )

        if magnitude <= 0.01:
            return

        i_alpha, i_beta = inverse_park(
            data.id_real,
            data.iq_real,
            data.theta_real
        )

        angle = math.atan2(
            i_beta,
            i_alpha
        )

        length = min(
            magnitude / 5.0,
            1.0
        ) * self.radius * 0.8

        self.draw_arrow(
            angle,
            length,
            "#00AA55",
            width=4,
            label="I"
        )

    def draw_voltage_vector(
        self,
        data
    ):

        if data.v_mag < 0.001:
            return

        length = (
            data.v_mag /
            MAX_VOLTAGE *
            self.radius *
            0.85
        )

        length = min(
            length,
            self.radius * 0.85
        )

        self.draw_arrow(
            data.v_angle,
            length,
            "#FF8800",
            width=4,
            label="Vref"
        )

    def draw_delta(
        self,
        data
    ):

        delta = data.delta

        if abs(delta) < 0.02:
            return

        radius = 65

        steps = 30

        points = []

        for i in range(
            steps + 1
        ):

            a = (
                data.theta_real +
                delta *
                i /
                steps
            )

            x, y = self.xy(
                a,
                radius
            )

            points.extend([
                x,
                y
            ])

        if abs(delta) > math.radians(60):

            color = "#FF2222"

        elif abs(delta) > math.radians(30):

            color = "#FF8800"

        else:

            color = "#00AA44"

        self.create_line(
            points,
            fill=color,
            width=3
        )

        middle = (
            data.theta_real +
            delta * 0.5
        )

        x, y = self.xy(
            middle,
            radius + 20
        )

        self.create_text(
            x,
            y,
            text=f"δ={deg(delta):+.1f}°",
            fill=color,
            font=(
                "Arial",
                10,
                "bold"
            )
        )

    def update(
        self,
        data
    ):

        self.data = data

        self.delete(
            "all"
        )

        self.draw_axes()

        self.draw_phase_axes()

        self.draw_hexagon()

        self.draw_rotor(
            data.theta_real
        )

        self.draw_estimated_axis(
            data.theta_est
        )

        self.draw_current_vector(
            data
        )

        self.draw_voltage_vector(
            data
        )

        self.draw_delta(
            data
        )

        lines = [

            f"Sector = {data.sector}",

            f"θreal = {deg(data.theta_real):+.2f}°",

            f"θest  = {deg(data.theta_est):+.2f}°",

            f"θerr  = {deg(data.theta_error):+.2f}°",

            "",

            f"Id_ref = {data.id_ref:+.3f} A",

            f"Iq_ref = {data.iq_ref:+.3f} A",

            f"Id_est = {data.id_est:+.3f} A",

            f"Iq_est = {data.iq_est:+.3f} A",

            "",

            f"Id_real = {data.id_real:+.3f} A",

            f"Iq_real = {data.iq_real:+.3f} A",

            "",

            f"Vd = {data.vd:+.3f} V",

            f"Vq = {data.vq:+.3f} V",

            f"|V| = {data.v_mag:.3f} V",

            "",

            f"Te = {data.te:+.4f} N·m",

            f"ω = {data.omega_mech:.1f} rad/s",

            f"RPM = {data.omega_mech * 60.0 / TWO_PI:.1f}"
        ]

        for i, text in enumerate(lines):

            self.create_text(
                20,
                20 + i * 17,
                text=text,
                anchor="w",
                fill="#444444",
                font=(
                    "Consolas",
                    9
                )
            )

        bottom = max(
            500,
            self.winfo_height() - 45
        )

        self.create_text(
            20,
            bottom,
            text=(
                f"Duty A={data.duty_a:.3f}   "
                f"B={data.duty_b:.3f}   "
                f"C={data.duty_c:.3f}"
            ),
            anchor="w",
            font=(
                "Consolas",
                10,
                "bold"
            ),
            fill="#444444"
        )

        self.create_text(
            20,
            bottom + 23,
            text=(
                f"SVPWM利用率="
                f"{data.power_utilization * 100.0:.1f}%"
            ),
            anchor="w",
            font=(
                "Consolas",
                10
            ),
            fill="#444444"
        )


# ============================================================
# 主程序
# ============================================================

class FOCSimulator:

    def __init__(
        self,
        root
    ):

        self.root = root

        self.root.title(
            "FOC 电流环 + SVPWM + PMSM 仿真"
        )

        self.root.geometry(
            "1500x980"
        )

        self.root.minsize(
            1200,
            800
        )

        self.running = False

        self.after_id = None

        self.sim_time = 0.0

        self.view_time = 0.0

        self.id_ref = 0.0
        self.iq_ref = 0.7

        self.load_torque = 0.0

        self.playback_speed = 1.0

        self.is_restoring_history = False

        self.motor = PMSMMotor()

        self.observer = Observer()

        self.pi_d = PIController(
            KP_ID,
            KI_ID,
            MAX_VOLTAGE
        )

        self.pi_q = PIController(
            KP_IQ,
            KI_IQ,
            MAX_VOLTAGE
        )

        self.history = HistoryBuffer()

        self.data = SimulationData()

        self.build_ui()

        self.reset()

    # ========================================================
    # UI
    # ========================================================

    def build_ui(self):

        root_frame = tk.Frame(
            self.root,
            bg="#EEF1F4"
        )

        root_frame.pack(
            fill=tk.BOTH,
            expand=True
        )

        # ====================================================
        # 左侧
        # ====================================================

        left = tk.Frame(
            root_frame,
            bg="white",
            bd=1,
            relief=tk.GROOVE
        )

        left.pack(
            side=tk.LEFT,
            fill=tk.Y,
            padx=8,
            pady=8
        )

        tk.Label(
            left,
            text="FOC 仿真控制",
            font=(
                "Microsoft YaHei",
                14,
                "bold"
            ),
            bg="white",
            fg="#223344"
        ).pack(
            pady=(12, 8)
        )

        # ----------------------------------------------------
        # 电流指令
        # ----------------------------------------------------

        frame = tk.LabelFrame(
            left,
            text="电流指令",
            bg="white",
            padx=8,
            pady=6
        )

        frame.pack(
            fill=tk.X,
            padx=8,
            pady=5
        )

        self.id_entry = self.make_entry(
            frame,
            "Id_ref (A)",
            "0.0"
        )

        self.iq_entry = self.make_entry(
            frame,
            "Iq_ref (A)",
            "0.7"
        )

        # ----------------------------------------------------
        # 观测器
        # ----------------------------------------------------

        frame = tk.LabelFrame(
            left,
            text="角度观测器",
            bg="white",
            padx=8,
            pady=6
        )

        frame.pack(
            fill=tk.X,
            padx=8,
            pady=5
        )

        self.offset_entry = self.make_entry(
            frame,
            "角度误差 (deg)",
            "0.0"
        )

        self.delay_entry = self.make_entry(
            frame,
            "延迟 (us)",
            "0.0"
        )

        self.speed_error_entry = self.make_entry(
            frame,
            "速度误差",
            "0.0"
        )

        # ----------------------------------------------------
        # 机械
        # ----------------------------------------------------

        frame = tk.LabelFrame(
            left,
            text="机械参数",
            bg="white",
            padx=8,
            pady=6
        )

        frame.pack(
            fill=tk.X,
            padx=8,
            pady=5
        )

        self.load_entry = self.make_entry(
            frame,
            "负载转矩",
            "0.0"
        )

        # ----------------------------------------------------
        # 初始状态
        # ----------------------------------------------------

        frame = tk.LabelFrame(
            left,
            text="初始状态",
            bg="white",
            padx=8,
            pady=6
        )

        frame.pack(
            fill=tk.X,
            padx=8,
            pady=5
        )

        self.initial_angle_entry = self.make_entry(
            frame,
            "机械角 (deg)",
            "0.0"
        )

        self.initial_speed_entry = self.make_entry(
            frame,
            "初始 RPM",
            "0.0"
        )

        # ----------------------------------------------------
        # 播放控制
        # ----------------------------------------------------

        frame = tk.LabelFrame(
            left,
            text="播放控制",
            bg="white",
            padx=8,
            pady=6
        )

        frame.pack(
            fill=tk.X,
            padx=8,
            pady=5
        )

        speed_row = tk.Frame(
            frame,
            bg="white"
        )

        speed_row.pack(
            fill=tk.X,
            pady=3
        )

        tk.Label(
            speed_row,
            text="播放倍速",
            bg="white",
            width=16,
            anchor="w"
        ).pack(
            side=tk.LEFT
        )

        self.speed_entry = tk.Entry(
            speed_row,
            width=10,
            font=("Consolas", 10)
        )

        self.speed_entry.pack(
            side=tk.LEFT
        )

        self.speed_entry.insert(
            0,
            "1.0"
        )

        tk.Label(
            speed_row,
            text="x",
            bg="white"
        ).pack(
            side=tk.LEFT,
            padx=3
        )

        self.speed_button = tk.Button(
            frame,
            text="应用倍速",
            command=self.apply_speed
        )

        self.speed_button.pack(
            fill=tk.X,
            pady=2
        )

        tk.Label(
            frame,
            text="范围：0.01x ~ 100x",
            bg="white",
            fg="#777777",
            font=("Consolas", 8)
        ).pack(
            anchor="w"
        )

        # ----------------------------------------------------
        # 开始 / 重置
        # ----------------------------------------------------

        button_frame = tk.Frame(
            left,
            bg="white"
        )

        button_frame.pack(
            fill=tk.X,
            padx=8,
            pady=8
        )

        self.start_button = tk.Button(
            button_frame,
            text="▶ 开始",
            command=self.toggle,
            bg="#27AE60",
            fg="white",
            font=(
                "Microsoft YaHei",
                10,
                "bold"
            ),
            width=9
        )

        self.start_button.pack(
            side=tk.LEFT,
            padx=3
        )

        tk.Button(
            button_frame,
            text="↺ 重置",
            command=self.reset,
            bg="#95A5A6",
            fg="white",
            font=(
                "Microsoft YaHei",
                10
            ),
            width=9
        ).pack(
            side=tk.LEFT,
            padx=3
        )

        # ----------------------------------------------------
        # 快捷测试
        # ----------------------------------------------------

        frame = tk.LabelFrame(
            left,
            text="快速测试",
            bg="white",
            padx=8,
            pady=6
        )

        frame.pack(
            fill=tk.X,
            padx=8,
            pady=5
        )

        buttons = [
            ("Iq = +0.7A", lambda: self.set_iq(0.7)),
            ("Iq = 0A", lambda: self.set_iq(0.0)),
            ("Iq = -0.7A", lambda: self.set_iq(-0.7)),
            ("角度误差 +10°", lambda: self.set_offset(10.0)),
            ("角度误差 -10°", lambda: self.set_offset(-10.0)),
            ("角度误差 0°", lambda: self.set_offset(0.0))
        ]

        for text, command in buttons:

            tk.Button(
                frame,
                text=text,
                command=command
            ).pack(
                fill=tk.X,
                pady=1
            )

        # ----------------------------------------------------
        # 电机参数
        # ----------------------------------------------------

        frame = tk.LabelFrame(
            left,
            text="电机参数",
            bg="white",
            padx=8,
            pady=6
        )

        frame.pack(
            fill=tk.X,
            padx=8,
            pady=5
        )

        motor_text = (
            f"Pole pairs = {POLE_PAIRS}\n"
            f"KV         = 650 rpm/V\n"
            f"Vbus       = {VBUS:.1f} V\n"
            f"Rs         = {RS:.4f} Ω\n"
            f"Ld/Lq      = {LD * 1e6:.2f} uH\n"
            f"Flux       = {FLUX:.6f} Wb\n"
            f"Kt         = {KT:.6f} N·m/A\n"
            f"J          = {J:.2e}\n"
            f"B          = {B:.2e}\n"
            f"Current BW = {CURRENT_BW / TWO_PI:.0f} Hz\n"
            f"Ts         = {CONTROL_DT * 1e6:.0f} us"
        )

        tk.Label(
            frame,
            text=motor_text,
            justify=tk.LEFT,
            anchor="w",
            bg="white",
            fg="#555555",
            font=("Consolas", 8)
        ).pack(
            anchor="w"
        )

        # ====================================================
        # 右侧
        # ====================================================

        right = tk.Frame(
            root_frame,
            bg="#EEF1F4"
        )

        right.pack(
            side=tk.RIGHT,
            fill=tk.BOTH,
            expand=True,
            padx=(0, 8),
            pady=8
        )

        # ----------------------------------------------------
        # 上半部分：空间矢量
        # ----------------------------------------------------

        self.canvas = VectorCanvas(
            right,
            width=850,
            height=500
        )

        self.canvas.pack(
            fill=tk.BOTH,
            expand=True
        )

        # ----------------------------------------------------
        # 示波器
        # ----------------------------------------------------

        scope_frame = tk.LabelFrame(
            right,
            text="示波器  |  鼠标移动查看数值  |  滚轮缩放时间轴",
            bg="white"
        )

        scope_frame.pack(
            fill=tk.BOTH,
            expand=True,
            pady=(6, 0)
        )

        self.scope = Oscilloscope(
            scope_frame,
            self,
            height=300
        )

        self.scope.pack(
            fill=tk.BOTH,
            expand=True,
            padx=3,
            pady=3
        )

        # ----------------------------------------------------
        # 总时间滑块
        # ----------------------------------------------------

        timeline_frame = tk.Frame(
            right,
            bg="white"
        )

        timeline_frame.pack(
            fill=tk.X,
            pady=(5, 0)
        )

        tk.Label(
            timeline_frame,
            text="历史时间",
            bg="white",
            font=(
                "Microsoft YaHei",
                9,
                "bold"
            )
        ).pack(
            side=tk.LEFT,
            padx=5
        )

        self.timeline = tk.Scale(
            timeline_frame,
            from_=0.0,
            to=1.0,
            resolution=0.001,
            orient=tk.HORIZONTAL,
            showvalue=False,
            command=self.on_timeline_move,
            bg="white",
            highlightthickness=0
        )

        self.timeline.pack(
            side=tk.LEFT,
            fill=tk.X,
            expand=True
        )

        self.timeline_label = tk.Label(
            timeline_frame,
            text="0.000 s",
            bg="white",
            width=20,
            anchor="e",
            font=("Consolas", 9)
        )

        self.timeline_label.pack(
            side=tk.RIGHT,
            padx=5
        )

        # ----------------------------------------------------
        # 状态文本
        # ----------------------------------------------------

        result_frame = tk.Frame(
            right,
            bg="white"
        )

        result_frame.pack(
            fill=tk.X,
            pady=(5, 0)
        )

        self.result_text = scrolledtext.ScrolledText(
            result_frame,
            height=8,
            font=("Consolas", 8),
            bg="#FAFAFA"
        )

        self.result_text.pack(
            fill=tk.X,
            padx=3,
            pady=3
        )

        self.result_text.config(
            state=tk.DISABLED
        )

    # ========================================================
    # Entry
    # ========================================================

    def make_entry(
        self,
        parent,
        label,
        default
    ):

        frame = tk.Frame(
            parent,
            bg="white"
        )

        frame.pack(
            fill=tk.X,
            pady=2
        )

        tk.Label(
            frame,
            text=label,
            bg="white",
            fg="#555555",
            width=16,
            anchor="w"
        ).pack(
            side=tk.LEFT
        )

        entry = tk.Entry(
            frame,
            width=11,
            font=("Consolas", 9)
        )

        entry.pack(
            side=tk.RIGHT
        )

        entry.insert(
            0,
            default
        )

        return entry

    # ========================================================
    # 快捷
    # ========================================================

    def set_iq(
        self,
        value
    ):

        self.iq_entry.delete(
            0,
            tk.END
        )

        self.iq_entry.insert(
            0,
            str(value)
        )

    def set_offset(
        self,
        value
    ):

        self.offset_entry.delete(
            0,
            tk.END
        )

        self.offset_entry.insert(
            0,
            str(value)
        )

    # ========================================================
    # 播放倍速
    # ========================================================

    def apply_speed(self):

        try:

            speed = float(
                self.speed_entry.get()
            )

            if not math.isfinite(speed):

                raise ValueError

            speed = clamp(
                speed,
                0.01,
                100.0
            )

            self.playback_speed = speed

            self.speed_entry.delete(
                0,
                tk.END
            )

            self.speed_entry.insert(
                0,
                f"{speed:g}"
            )

        except Exception:

            self.write_result(
                "播放倍速输入错误。\n"
                "请输入 0.01 ~ 100 之间的数字。"
            )

    # ========================================================
    # 读取参数
    # ========================================================

    def read_parameters(self):

        self.id_ref = float(
            self.id_entry.get()
        )

        self.iq_ref = float(
            self.iq_entry.get()
        )

        self.load_torque = float(
            self.load_entry.get()
        )

        self.observer.offset_deg = float(
            self.offset_entry.get()
        )

        self.observer.delay_us = float(
            self.delay_entry.get()
        )

        self.observer.speed_error = float(
            self.speed_error_entry.get()
        )

        self.id_ref = clamp(
            self.id_ref,
            -CURRENT_LIMIT,
            CURRENT_LIMIT
        )

        self.iq_ref = clamp(
            self.iq_ref,
            -CURRENT_LIMIT,
            CURRENT_LIMIT
        )

    # ========================================================
    # 重置
    # ========================================================

    def reset(self):

        self.running = False

        if self.after_id is not None:

            try:

                self.root.after_cancel(
                    self.after_id
                )

            except Exception:

                pass

            self.after_id = None

        try:

            initial_angle = math.radians(
                float(
                    self.initial_angle_entry.get()
                )
            )

            initial_rpm = float(
                self.initial_speed_entry.get()
            )

        except Exception:

            initial_angle = 0.0
            initial_rpm = 0.0

        initial_omega = (
            initial_rpm *
            TWO_PI /
            60.0
        )

        self.motor.reset(
            initial_angle,
            initial_omega
        )

        self.observer.reset(
            self.motor.theta_elec
        )

        self.pi_d.reset()
        self.pi_q.reset()

        self.sim_time = 0.0
        self.view_time = 0.0

        self.history.clear()

        self.data = SimulationData()

        self.update_once()

        self.timeline.configure(
            from_=0.0,
            to=1.0
        )

        self.timeline.set(
            0.0
        )

        self.timeline_label.config(
            text="0.000 s"
        )

        self.start_button.config(
            text="▶ 开始",
            bg="#27AE60"
        )

    # ========================================================
    # 开始 / 停止
    # ========================================================

    def toggle(self):

        if self.running:

            self.stop()

        else:

            self.start()

    def start(self):

        try:

            self.read_parameters()

            self.apply_speed()

        except Exception:

            self.write_result(
                "参数输入错误，请检查输入。"
            )

            return

        self.running = True

        self.view_time = self.sim_time

        self.start_button.config(
            text="⏹ 停止",
            bg="#E74C3C"
        )

        self.run_gui_cycle()

    def stop(self):

        self.running = False

        if self.after_id is not None:

            try:

                self.root.after_cancel(
                    self.after_id
                )

            except Exception:

                pass

            self.after_id = None

        self.view_time = self.sim_time

        self.start_button.config(
            text="▶ 开始",
            bg="#27AE60"
        )

        self.scope.draw()

    # ========================================================
    # 核心 FOC
    # ========================================================

    def control_step(self):

        theta_real = self.motor.theta_elec

        omega_real = self.motor.omega_elec

        theta_est, omega_est = (
            self.observer.update(
                theta_real,
                omega_real,
                CONTROL_DT
            )
        )

        ia = self.motor.ia
        ib = self.motor.ib
        ic = self.motor.ic

        i_alpha, i_beta = clarke(
            ia,
            ib,
            ic
        )

        id_est, iq_est = park(
            i_alpha,
            i_beta,
            theta_est
        )

        id_ref = self.id_ref
        iq_ref = self.iq_ref

        vd_pi = self.pi_d.update(
            id_ref,
            id_est,
            CONTROL_DT
        )

        vq_pi = self.pi_q.update(
            iq_ref,
            iq_est,
            CONTROL_DT
        )

        vd_ff = (
            -omega_est *
            LQ *
            iq_est
        )

        vq_ff = (
            omega_est *
            (
                LD * id_est +
                FLUX
            )
        )

        vd = (
            vd_pi +
            vd_ff
        )

        vq = (
            vq_pi +
            vq_ff
        )

        voltage_mag = math.sqrt(
            vd * vd +
            vq * vq
        )

        if voltage_mag > MAX_VOLTAGE:

            scale = (
                MAX_VOLTAGE /
                voltage_mag
            )

            vd *= scale
            vq *= scale

        v_alpha, v_beta = inverse_park(
            vd,
            vq,
            theta_est
        )

        pwm = svpwm(
            v_alpha,
            v_beta,
            VBUS
        )

        motor_state = self.motor.step(
            pwm.alpha,
            pwm.beta,
            CONTROL_DT,
            self.load_torque
        )

        theta_real_new = (
            self.motor.theta_elec
        )

        theta_error = normalize_angle(
            theta_est -
            theta_real_new
        )

        delta = normalize_angle(
            pwm.angle -
            theta_real_new
        )

        self.data = SimulationData(

            theta_real=theta_real_new,

            theta_est=theta_est,

            theta_error=theta_error,

            omega_mech=self.motor.omega_mech,

            omega_elec=self.motor.omega_elec,

            ia=self.motor.ia,

            ib=self.motor.ib,

            ic=self.motor.ic,

            id_est=id_est,

            iq_est=iq_est,

            id_real=self.motor.id,

            iq_real=self.motor.iq,

            id_ref=id_ref,

            iq_ref=iq_ref,

            vd=vd,

            vq=vq,

            v_alpha=pwm.alpha,

            v_beta=pwm.beta,

            v_mag=pwm.magnitude,

            v_angle=pwm.angle,

            sector=pwm.sector,

            duty_a=pwm.duty_a,

            duty_b=pwm.duty_b,

            duty_c=pwm.duty_c,

            te=self.motor.te,

            power_utilization=pwm.utilization,

            delta=delta
        )

        self.sim_time += CONTROL_DT

        # ====================================================
        # 保存历史
        # ====================================================

        self.history.append(
            self.sim_time,
            self.data,
            self.motor,
            self.observer,
            self.pi_d,
            self.pi_q
        )

    # ========================================================
    # GUI 周期
    # ========================================================

    def run_gui_cycle(self):

        if not self.running:

            return

        try:

            gui_dt = (
                GUI_INTERVAL_MS /
                1000.0
            )

            # ------------------------------------------------
            # 播放倍速
            #
            # 1x = 正常 30ms
            # 0.01x = 每次推进 0.3ms
            # 100x = 每次推进 3s
            #
            # 同时限制单次最大计算量。
            # ------------------------------------------------

            simulation_dt = (
                gui_dt *
                self.playback_speed
            )

            steps = int(
                simulation_dt /
                CONTROL_DT
            )

            if steps < 1:

                steps = 1

            steps = min(
                steps,
                60000
            )

            for _ in range(steps):

                self.control_step()

            self.view_time = self.sim_time

            self.update_timeline()

            self.canvas.update(
                self.data
            )

            self.scope.draw()

            self.update_result_panel()

        except Exception as exc:

            self.write_result(
                "仿真发生异常，但程序窗口已保留。\n\n"
                "错误信息：\n" +
                str(exc)
            )

            self.running = False

            self.start_button.config(
                text="▶ 开始",
                bg="#27AE60"
            )

            self.after_id = None

            return

        self.after_id = self.root.after(
            GUI_INTERVAL_MS,
            self.run_gui_cycle
        )

    # ========================================================
    # 当前示波器结束时间
    # ========================================================

    def get_scope_end_time(self):

        if self.running:

            return self.sim_time

        return self.view_time

    # ========================================================
    # 时间轴
    # ========================================================

    def update_timeline(self):

        maximum = max(
            self.sim_time,
            0.001
        )

        self.timeline.configure(
            from_=0.0,
            to=maximum
        )

        self.timeline.set(
            self.view_time
        )

        self.timeline_label.config(
            text=(
                f"{self.view_time:.6f} s"
            )
        )

    def on_timeline_move(
        self,
        value
    ):

        if self.running:

            return

        try:

            target = float(
                value
            )

        except Exception:

            return

        if not self.history.records:

            return

        self.restore_history(
            target
        )

    # ========================================================
    # 恢复历史状态
    # ========================================================

    def restore_history(
        self,
        target_time
    ):

        if self.is_restoring_history:

            return

        record = self.history.get_record(
            target_time
        )

        if record is None:

            return

        self.is_restoring_history = True

        try:

            self.view_time = (
                record.time
            )

            # ------------------------------------------------
            # 电机状态
            # ------------------------------------------------

            self.motor.theta_mech = (
                record.motor_theta_mech
            )

            self.motor.omega_mech = (
                record.motor_omega_mech
            )

            self.motor.id = (
                record.motor_id
            )

            self.motor.iq = (
                record.motor_iq
            )

            self.motor.ia = (
                record.motor_ia
            )

            self.motor.ib = (
                record.motor_ib
            )

            self.motor.ic = (
                record.motor_ic
            )

            self.motor.te = (
                record.motor_te
            )

            # ------------------------------------------------
            # Observer
            # ------------------------------------------------

            self.observer.filtered_angle = (
                record.observer_angle
            )

            # ------------------------------------------------
            # PI 状态
            # ------------------------------------------------

            self.pi_d.integral = (
                record.pi_d_integral
            )

            self.pi_q.integral = (
                record.pi_q_integral
            )

            # ------------------------------------------------
            # 当前显示数据
            # ------------------------------------------------

            self.data = SimulationData(
                **record.data.__dict__
            )

            self.canvas.update(
                self.data
            )

            self.scope.draw()

            self.update_timeline()

            self.update_result_panel()

        finally:

            self.is_restoring_history = False

    # ========================================================
    # 静态更新
    # ========================================================

    def update_once(self):

        theta_real = (
            self.motor.theta_elec
        )

        theta_est = theta_real

        self.data.theta_real = (
            theta_real
        )

        self.data.theta_est = (
            theta_est
        )

        self.data.theta_error = 0.0

        self.data.omega_mech = (
            self.motor.omega_mech
        )

        self.data.omega_elec = (
            self.motor.omega_elec
        )

        self.data.id_real = (
            self.motor.id
        )

        self.data.iq_real = (
            self.motor.iq
        )

        self.data.ia = (
            self.motor.ia
        )

        self.data.ib = (
            self.motor.ib
        )

        self.data.ic = (
            self.motor.ic
        )

        self.data.id_ref = (
            self.id_ref
        )

        self.data.iq_ref = (
            self.iq_ref
        )

        self.data.te = (
            self.motor.te
        )

        self.canvas.update(
            self.data
        )

        self.scope.draw()

        self.update_result_panel()

    # ========================================================
    # 结果面板
    # ========================================================

    def update_result_panel(self):

        d = self.data

        rpm = (
            d.omega_mech *
            60.0 /
            TWO_PI
        )

        electrical_rpm = (
            d.omega_elec *
            60.0 /
            TWO_PI
        )

        current_mag = math.sqrt(
            d.ia ** 2 +
            d.ib ** 2 +
            d.ic ** 2
        )

        text = f"""
============================================================
FOC + SVPWM 实时状态
============================================================

仿真时间
    simulation time   = {self.sim_time:.6f} s
    view time         = {self.view_time:.6f} s
    playback speed    = {self.playback_speed:.3f} x

------------------------------------------------------------
转子位置
------------------------------------------------------------

    theta_real        = {deg(d.theta_real):+9.3f} deg
    theta_est         = {deg(d.theta_est):+9.3f} deg
    theta_error       = {deg(d.theta_error):+9.3f} deg

------------------------------------------------------------
三相电流
------------------------------------------------------------

    Ia                = {d.ia:+9.4f} A
    Ib                = {d.ib:+9.4f} A
    Ic                = {d.ic:+9.4f} A

    Iabc magnitude    = {current_mag:9.4f} A

------------------------------------------------------------
电流环
------------------------------------------------------------

    Id_ref            = {d.id_ref:+9.4f} A
    Iq_ref            = {d.iq_ref:+9.4f} A

    Id_est            = {d.id_est:+9.4f} A
    Iq_est            = {d.iq_est:+9.4f} A

    Id_real           = {d.id_real:+9.4f} A
    Iq_real           = {d.iq_real:+9.4f} A

    Id error          = {d.id_ref - d.id_est:+9.4f} A
    Iq error          = {d.iq_ref - d.iq_est:+9.4f} A

------------------------------------------------------------
电压
------------------------------------------------------------

    Vd                = {d.vd:+9.4f} V
    Vq                = {d.vq:+9.4f} V

    Valpha            = {d.v_alpha:+9.4f} V
    Vbeta             = {d.v_beta:+9.4f} V

    |Vref|            = {d.v_mag:9.4f} V
    Vref angle        = {deg(d.v_angle):+9.3f} deg

------------------------------------------------------------
SVPWM
------------------------------------------------------------

    Sector            = {d.sector}

    Duty A            = {d.duty_a:9.4f}
    Duty B            = {d.duty_b:9.4f}
    Duty C            = {d.duty_c:9.4f}

    Voltage usage     = {d.power_utilization * 100.0:8.2f} %

------------------------------------------------------------
电机
------------------------------------------------------------

    Te                = {d.te:+9.5f} N.m

    omega_mech        = {d.omega_mech:+9.3f} rad/s
    omega_elec        = {d.omega_elec:+9.3f} rad/s

    RPM               = {rpm:+9.2f}
    Electrical RPM    = {electrical_rpm:+9.2f}

------------------------------------------------------------
功角
------------------------------------------------------------

    delta             = {deg(d.delta):+.3f} deg

------------------------------------------------------------
示波器
------------------------------------------------------------

    time window       = {self.scope.time_window:.4f} s

    操作：
        鼠标移动       -> 查看该时间点数据
        滚轮向上       -> 放大时间轴
        滚轮向下       -> 缩小时间轴
        暂停后拖动滑块 -> 回看历史

------------------------------------------------------------
电机参数
------------------------------------------------------------

    Pole pairs        = {POLE_PAIRS}
    KV                = 650 rpm/V
    Vbus              = {VBUS:.3f} V
    Rs                = {RS:.6f} ohm
    Ld                = {LD * 1e6:.3f} uH
    Lq                = {LQ * 1e6:.3f} uH
    Flux              = {FLUX:.7f} Wb
    Kt                = {KT:.7f} N.m/A

------------------------------------------------------------
控制器
------------------------------------------------------------

    Current BW        = {CURRENT_BW / TWO_PI:.1f} Hz

    Kp_d              = {KP_ID:.6f}
    Ki_d              = {KI_ID:.6f}

    Kp_q              = {KP_IQ:.6f}
    Ki_q              = {KI_IQ:.6f}

============================================================
"""

        self.write_result(
            text
        )

    # ========================================================
    # 安全写文本
    # ========================================================

    def write_result(
        self,
        text
    ):

        try:

            self.result_text.config(
                state=tk.NORMAL
            )

            self.result_text.delete(
                "1.0",
                tk.END
            )

            # 注意：
            #
            # Tkinter Text.insert：
            #
            # insert(index, chars)
            #
            # 这里严格只传两个位置参数，
            # 避免之前出现：
            #
            # wrong # args:
            # should be ".!scrolledtext insert ..."
            #

            self.result_text.insert(
                "1.0",
                str(text)
            )

            self.result_text.config(
                state=tk.DISABLED
            )

        except Exception:

            # 即使结果窗口自身出现问题，
            # 也不要让主程序直接退出。
            try:

                self.result_text.config(
                    state=tk.NORMAL
                )

                self.result_text.delete(
                    "1.0",
                    tk.END
                )

                self.result_text.insert(
                    "1.0",
                    "界面输出错误：\n" +
                    str(text)
                )

                self.result_text.config(
                    state=tk.DISABLED
                )

            except Exception:

                pass


# ============================================================
# main
# ============================================================

def main():

    root = tk.Tk()

    app = FOCSimulator(
        root
    )

    root.mainloop()


if __name__ == "__main__":

    main()