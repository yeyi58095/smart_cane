#!/usr/bin/env python3
import sys
import math
import threading
import signal

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatusArray

from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtCore import Qt, QTimer, QPointF
from PyQt5.QtGui import QPainter, QPen, QColor, QPolygonF, QFont
from std_msgs.msg import String

# ===================== ROS NODE =====================
class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('qt_cmd_vel_ui')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_sub = self.create_subscription(Twist, '/nav_cmd_vel', self.nav_cb, 10)
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_cb,
            10
        )

        self.nav_vx = 0.0
        self.nav_wz = 0.0

        self._pulse_success = False
        self._pulse_failed = False
        self._pulse_canceled = False

        self.collision_state = ""
        self.state_sub = self.create_subscription(String, '/collision_state', self.state_cb, 10)


    def nav_cb(self, msg: Twist):
        self.nav_vx = float(msg.linear.x)
        self.nav_wz = float(msg.angular.z)

    def state_cb(self, msg: String):
        # collision_guidance 發空字串代表解除
        self.collision_state = (msg.data or "").strip()


    def status_cb(self, msg: GoalStatusArray):
        # 4 SUCCEEDED, 5 CANCELED, 6 ABORTED
        for s in msg.status_list:
            if s.status == 4:
                self._pulse_success = True
                break
            if s.status == 6:
                self._pulse_failed = True
                break
            if s.status == 5:
                self._pulse_canceled = True
                break

    def take_pulses(self):
        suc = self._pulse_success
        fail = self._pulse_failed
        can = self._pulse_canceled
        self._pulse_success = False
        self._pulse_failed = False
        self._pulse_canceled = False
        return suc, fail, can

    def publish_cmd(self, vx: float, wz: float):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)


# ===================== QT WIDGET =====================
class Joystick(QWidget):
    def __init__(self, rosnode: CmdVelBridge):
        super().__init__()
        self.node = rosnode
        self.setWindowTitle("ROS2 CmdVel UI (Q to quit)")

        # allow resize
        self.resize(640, 640)
        self.setMinimumSize(520, 520)
        self.setMouseTracking(True)

        # Control limits
        self.v_max = 0.22
        self.w_max = 2.84

        # Scale params
        self.deadzone = 0.06
        self.rings = 10
        self.spokes = 8
        self.publish_hz = 20
        self.exp_s_max = 5.0

        # UI state
        self.hover = False
        self.pressed = False
        self.mx = 0
        self.my = 0

        # status badge
        self.status_text = ""
        self.status_color = QColor(60, 60, 60)
        self.status_timer = QTimer()
        self.status_timer.setSingleShot(True)
        self.status_timer.timeout.connect(self.clear_status)

        # geometry (computed)
        self.top_h = 46
        self.bottom_h = 92
        self.margin = 24
        self.cx = 0
        self.cy = 0
        self.R = 0
        self.scale = 1.0
        self.recompute_geometry()

        # timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(int(1000 / self.publish_hz))

    # ---------- resize ----------
    def recompute_geometry(self):
        w = self.width()
        h = self.height()

        # circle region height excludes bottom panel
        usable_h = h - self.top_h - self.bottom_h - 2 * self.margin
        usable_w = w - 2 * self.margin

        self.R = int(0.5 * min(usable_w, usable_h))
        self.cx = w // 2
        self.cy = self.top_h + self.margin + usable_h // 2

        # scale factor relative to a "base" radius 200
        self.scale = max(0.6, self.R / 200.0)

    def resizeEvent(self, e):
        self.recompute_geometry()
        super().resizeEvent(e)
        self.update()

    # ---------- quit/stop ----------
    def stop_robot(self):
        self.node.publish_cmd(0.0, 0.0)

    def closeEvent(self, event):
        self.pressed = False
        self.stop_robot()
        event.accept()

    def keyPressEvent(self, e):
        if e.key() in (Qt.Key_Q, Qt.Key_Escape):
            self.pressed = False
            self.stop_robot()
            self.close()
        else:
            super().keyPressEvent(e)

    # ---------- status badge ----------
    def show_status(self, text: str, color: QColor, ms: int = 3000):
        self.status_text = text
        self.status_color = color
        self.status_timer.start(ms)

    def clear_status(self):
        self.status_text = ""
        self.update()

    # ---------- mouse ----------
    def mouseMoveEvent(self, e):
        self.mx = e.x()
        self.my = e.y()
        self.hover = self.in_circle(self.mx, self.my)
        self.update()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            # 只允許在上半圓開始控制
            if self.in_circle(e.x(), e.y()):
                self.pressed = True
            else:
                self.pressed = False
            self.update()


    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.pressed = False
            self.stop_robot()
            self.update()

    def in_upper_circle(self, x, y):
        # 只允許上半圓（含直徑線）
        if y > self.cy:
            return False
        return (x - self.cx) ** 2 + (y - self.cy) ** 2 <= self.R ** 2

    def in_circle(self, x, y):
        # 只允許上半圓（含直徑線）
        # if y > self.cy:
        #    return False
        # return (x - self.cx) ** 2 + (y - self.cy) ** 2 <= self.R ** 2
        return self.in_upper_circle(x, y)

    # ---------- mapping ----------
    def clamp01(self, x: float) -> float:
        return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)

    # r = 1 - 2^{-s}
    def exp_radius_from_s(self, s: float) -> float:
        s = max(0.0, float(s))
        return 1.0 - (2.0 ** (-s))

    # s = -log2(1-r)
    def s_from_exp_radius(self, r01: float) -> float:
        r = self.clamp01(r01)
        if r >= 0.999999:
            return 60.0
        return -math.log2(1.0 - r)

    def mouse_dir_and_rawmag(self):
        dx = self.mx - self.cx

        # ★ 上半圓限制：若滑鼠跑到下半部，直接投影到直徑線 y=cy
        my_clamped = self.my if self.my <= self.cy else self.cy
        dy = self.cy - my_clamped  # y up, always >= 0

        r = math.hypot(dx, dy)
        if r < 1e-6:
            return 0.0, 0.0, 0.0

        mag_raw = self.clamp01(r / self.R)
        ux = dx / r
        uy = dy / r
        return ux, uy, mag_raw


    def mag_cmd_from_raw(self, mag_raw: float) -> float:
        if mag_raw < self.deadzone:
            return 0.0
        x = (mag_raw - self.deadzone) / (1.0 - self.deadzone)
        x = self.clamp01(x)
        s = self.s_from_exp_radius(x)
        return min(1.0, s / self.exp_s_max)

    def disp_mag_from_cmdmag(self, mag_cmd: float) -> float:
        mag_cmd = self.clamp01(mag_cmd)
        s = mag_cmd * self.exp_s_max
        return self.exp_radius_from_s(s)

    def compute_target_twist(self):
        ux, uy, mag_raw = self.mouse_dir_and_rawmag()
        mag_cmd = self.mag_cmd_from_raw(mag_raw)
        vx = self.v_max * mag_cmd * uy
        wz = - self.w_max * mag_cmd * ux
        return vx, wz, mag_raw, mag_cmd, ux, uy

    # ---------- main loop ----------
    def on_timer(self):
        if self.pressed:
            vx, wz, *_ = self.compute_target_twist()
            self.node.publish_cmd(vx, wz)

        suc, fail, can = self.node.take_pulses()
        if suc:
            self.show_status("✅ Navigation Succeeded", QColor(0, 150, 0), ms=2800)
        elif fail:
            self.show_status("❌ Navigation Failed", QColor(200, 0, 0), ms=3500)
        elif can:
            self.show_status("⚠️ Navigation Canceled", QColor(180, 120, 0), ms=2500)

        self.update()

    # ---------- drawing ----------
    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        self.draw_top_status_bar(p)

        self.draw_grid(p)

        # pink arrow
        if self.hover or self.pressed:
            vx, wz, mag_raw, mag_cmd, ux, uy = self.compute_target_twist()
            color = QColor(255, 120, 170) if not self.pressed else QColor(200, 20, 90)
            self.draw_arrow(p, ux, uy, mag_raw, color)

        # green arrow (on top)
        nav_x = 0.0 if self.w_max <= 1e-9 else -(self.node.nav_wz / self.w_max)
        nav_y = 0.0 if self.v_max <= 1e-9 else (self.node.nav_vx / self.v_max)
        nav_mag = math.hypot(nav_x, nav_y)
        if nav_mag > 1e-6:
            ux = nav_x / nav_mag
            uy = nav_y / nav_mag
            mag_cmd = min(1.0, nav_mag)
            mag_disp = self.disp_mag_from_cmdmag(mag_cmd)
            self.draw_arrow(p, ux, uy, mag_disp, QColor(0, 220, 0), is_green=True)

        # center dot
        p.setPen(QPen(Qt.white, 1))
        p.setBrush(QColor(240, 240, 240))
        p.drawEllipse(int(self.cx - 4*self.scale), int(self.cy - 4*self.scale),
                      int(8*self.scale), int(8*self.scale))

        # bottom panel
        self.draw_bottom_panel(p)

        # status badge (top-right)
        # self.draw_status_badge(p)

    def draw_grid(self, p: QPainter):
        # ----- styles -----
        thin = max(1, int(1*self.scale))
        mid  = max(2, int(2*self.scale))

        # ----- outer semicircle (just the boundary) -----
        p.setPen(QPen(QColor(150, 150, 150), mid))
        # QRect: (cx-R, cy-R, 2R, 2R), startAngle=0, span=180deg (counterclockwise)
        # In Qt: angles are 1/16 degree, 0 at 3 o'clock, CCW positive.
        rect_x = self.cx - self.R
        rect_y = self.cy - self.R
        rect_w = 2 * self.R
        rect_h = 2 * self.R
        p.drawArc(rect_x, rect_y, rect_w, rect_h, 0 * 16, 180 * 16)

        # ----- spokes (only upper half) -----
        p.setPen(QPen(QColor(210, 210, 210), thin))
        for i in range(self.spokes + 1):
            # i=0..spokes : from left(-pi) to right(0) across top
            ang = math.pi * (i / self.spokes)  # 0..pi
            # convert to math coords: ang measured from +x toward +y (CCW)
            x2 = self.cx + math.cos(ang) * self.R
            y2 = self.cy - math.sin(ang) * self.R
            p.drawLine(int(self.cx), int(self.cy), int(x2), int(y2))

        # ----- rings (only upper half, exponential spacing) -----
        p.setPen(QPen(QColor(170, 170, 170), mid))
        for i in range(1, self.rings + 1):
            s = (i / self.rings) * self.exp_s_max
            x = self.exp_radius_from_s(s)
            raw = self.deadzone + (1.0 - self.deadzone) * x
            rr = int(self.R * raw)

            # draw semicircle arc of radius rr
            rx = self.cx - rr
            ry = self.cy - rr
            rw = 2 * rr
            rh = 2 * rr
            p.drawArc(rx, ry, rw, rh, 0 * 16, 180 * 16)

        # ----- deadzone semicircle -----
        p.setPen(QPen(QColor(160, 160, 160), thin, Qt.DashLine))
        rr = int(self.R * self.deadzone)
        rx = self.cx - rr
        ry = self.cy - rr
        rw = 2 * rr
        rh = 2 * rr
        p.drawArc(rx, ry, rw, rh, 0 * 16, 180 * 16)

        # ----- axes (horizontal full diameter, vertical only upward) -----
        p.setPen(QPen(QColor(150, 150, 150), mid))
        # horizontal line across diameter
        p.drawLine(self.cx - self.R, self.cy, self.cx + self.R, self.cy)
        # vertical line only upwards
        p.drawLine(self.cx, self.cy, self.cx, self.cy - self.R)

        # ----- axis labels -----
        p.setPen(QPen(QColor(110, 110, 110), 1))
        font = QFont()
        font.setPointSize(max(9, int(10 * self.scale)))
        p.setFont(font)

        p.drawText(self.cx + self.R - int(92*self.scale), self.cy - int(8*self.scale), f"+w {self.w_max:.2f}")
        p.drawText(self.cx - self.R + int(8*self.scale),  self.cy - int(8*self.scale), f"-w {self.w_max:.2f}")
        p.drawText(self.cx + int(8*self.scale), self.cy - self.R + int(22*self.scale), f"+v {self.v_max:.2f}")

        # ----- ticks (0.25/0.5/0.75/1.0) on the axes -----
        p.setPen(QPen(QColor(140, 140, 140), thin))
        for frac in [0.25, 0.5, 0.75, 1.0]:
            x = self.disp_mag_from_cmdmag(frac)  # 0..1 in display space
            rr = int(self.R * (self.deadzone + (1.0 - self.deadzone) * x))

            # ticks on +w and -w
            p.drawLine(self.cx + rr, self.cy - int(6*self.scale), self.cx + rr, self.cy + int(6*self.scale))
            p.drawLine(self.cx - rr, self.cy - int(6*self.scale), self.cx - rr, self.cy + int(6*self.scale))

            # ticks on +v (up only)
            p.drawLine(self.cx - int(6*self.scale), self.cy - rr, self.cx + int(6*self.scale), self.cy - rr)

            # spokes
            p.setPen(QPen(QColor(210, 210, 210), max(1, int(1*self.scale))))
            for i in range(self.spokes):
                ang = (2 * math.pi) * (i / self.spokes)
                x2 = self.cx + math.cos(ang) * self.R
                y2 = self.cy + math.sin(ang) * self.R
                p.drawLine(int(self.cx), int(self.cy), int(x2), int(y2))

            # rings
            p.setPen(QPen(Qt.gray, max(2, int(2*self.scale))))
            for i in range(1, self.rings + 1):
                s = (i / self.rings) * self.exp_s_max
                x = self.exp_radius_from_s(s)
                raw = self.deadzone + (1.0 - self.deadzone) * x
                rr = int(self.R * raw)
                p.drawEllipse(self.cx - rr, self.cy - rr, 2 * rr, 2 * rr)

            # deadzone
            p.setPen(QPen(QColor(160, 160, 160), max(1, int(1*self.scale)), Qt.DashLine))
            rr = int(self.R * self.deadzone)
            p.drawEllipse(self.cx - rr, self.cy - rr, 2 * rr, 2 * rr)

            # axes
            p.setPen(QPen(QColor(160, 160, 160), max(2, int(2*self.scale))))
            p.drawLine(self.cx - self.R, self.cy, self.cx + self.R, self.cy)
            p.drawLine(self.cx, self.cy - self.R, self.cx, self.cy + self.R)

            # labels
            p.setPen(QPen(QColor(110, 110, 110), 1))
            font = QFont()
            font.setPointSize(max(9, int(10 * self.scale)))
            p.setFont(font)
            p.drawText(self.cx + self.R - int(92*self.scale), self.cy - int(8*self.scale), f"+w {self.w_max:.2f}")
            p.drawText(self.cx - self.R + int(8*self.scale),  self.cy - int(8*self.scale), f"-w {self.w_max:.2f}")
            p.drawText(self.cx + int(8*self.scale), self.cy - self.R + int(22*self.scale), f"+v {self.v_max:.2f}")
            p.drawText(self.cx + int(8*self.scale), self.cy + self.R - int(8*self.scale),  f"-v {self.v_max:.2f}")

    def draw_top_status_bar(self, p: QPainter):
        # bar area
        bx = int(18 * self.scale)
        by = int(12 * self.scale)
        bw = self.width() - int(2 * 18 * self.scale)
        bh = int(self.top_h * self.scale)

        # background
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(245, 245, 245))
        p.drawRoundedRect(bx, by, bw, bh, 12, 12)

        # text
        # Collision Avioding
        if self.node.collision_state:
            p.setPen(QPen(QColor(200, 0, 0), max(2, int(2 * self.scale))))
            font = QFont()
            font.setPointSize(max(11, int(12 * self.scale)))
            font.setBold(True)
            p.setFont(font)
            p.drawText(bx + int(14 * self.scale), by + int(28 * self.scale),
                    self.node.collision_state)
        elif self.status_text:
            # Nav Status
            p.setPen(QPen(self.status_color, max(2, int(2 * self.scale))))
            font = QFont()
            font.setPointSize(max(11, int(12 * self.scale)))
            font.setBold(True)
            p.setFont(font)
            p.drawText(bx + int(14 * self.scale), by + int(28 * self.scale), self.status_text)
        else:
            p.setPen(QPen(QColor(110, 110, 110), 1))
            font = QFont()
            font.setPointSize(max(10, int(11 * self.scale)))
            p.setFont(font)
            p.drawText(bx + int(14 * self.scale), by + int(28 * self.scale),
                    "Status: Ready   (Nav success/fail will show here)")

    def draw_bottom_panel(self, p: QPainter):
        bx = int(28*self.scale)
        bw = self.width() - int(2*28*self.scale)
        by = self.height() - self.bottom_h
        bh = self.bottom_h - int(18*self.scale)

        p.setPen(Qt.NoPen)
        p.setBrush(QColor(245, 245, 245))
        p.drawRoundedRect(bx, by, bw, bh, 12, 12)

        # values
        if self.hover or self.pressed:
            tv, tw, *_ = self.compute_target_twist()
        else:
            tv, tw = 0.0, 0.0
        nv = self.node.nav_vx
        nw = self.node.nav_wz

        p.setPen(QPen(QColor(70, 70, 70), 1))
        font = QFont()
        font.setPointSize(max(10, int(11 * self.scale)))
        p.setFont(font)

        line1_y = by + int(26*self.scale)
        line2_y = by + int(50*self.scale)

        p.drawText(bx + int(14*self.scale), line1_y,
                   f"Target (pink):  v={tv:+.3f} m/s   w={tw:+.3f} rad/s")
        p.drawText(bx + int(14*self.scale), line2_y,
                   f"Nav (green):     v={nv:+.3f} m/s   w={nw:+.3f} rad/s")

        # hint (right side)
        p.setPen(QPen(QColor(120, 120, 120), 1))
        font2 = QFont()
        font2.setPointSize(max(9, int(10 * self.scale)))
        p.setFont(font2)
        p.drawText(self.width() - int(250*self.scale), line2_y,
                   "")

    def draw_status_badge(self, p: QPainter):
        if not self.status_text:
            return
        bw = int(240*self.scale)
        bh = int(36*self.scale)
        bx = self.width() - bw - int(18*self.scale)
        by = int(16*self.scale)

        p.setPen(Qt.NoPen)
        p.setBrush(QColor(250, 250, 250))
        p.drawRoundedRect(bx, by, bw, bh, 10, 10)

        p.setPen(QPen(self.status_color, max(2, int(2*self.scale))))
        font = QFont()
        font.setPointSize(max(10, int(11*self.scale)))
        font.setBold(True)
        p.setFont(font)
        p.drawText(bx + int(12*self.scale), by + int(24*self.scale), self.status_text)

    def draw_arrow(self, p: QPainter, ux: float, uy: float, mag01: float,
                   color: QColor = None, is_green: bool = False):
        mag = max(0.0, min(1.0, float(mag01)))
        if mag <= 1e-4:
            return

        if color is None:
            color = QColor(255, 120, 170)

        shaft_w = max(4, int((6 if is_green else 5) * self.scale))
        head_w = max(16, int((22 if is_green else 18) * self.scale))
        head_l = max(20, int((30 if is_green else 26) * self.scale))

        # widget coords
        dirx = ux
        diry = -uy
        length = mag * self.R
        x2 = self.cx + dirx * length
        y2 = self.cy + diry * length

        p.setPen(QPen(color, shaft_w, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        p.drawLine(int(self.cx), int(self.cy), int(x2), int(y2))

        dlen = math.hypot(dirx, diry)
        if dlen < 1e-6:
            return
        dx = dirx / dlen
        dy = diry / dlen
        px = -dy
        py = dx

        tip = QPointF(x2, y2)
        base = QPointF(x2 - dx * head_l, y2 - dy * head_l)
        left = QPointF(base.x() + px * (head_w / 2), base.y() + py * (head_w / 2))
        right = QPointF(base.x() - px * (head_w / 2), base.y() - py * (head_w / 2))

        poly = QPolygonF([tip, left, right])
        p.setPen(QPen(color, 1))
        p.setBrush(color)
        p.drawPolygon(poly)


# ===================== MAIN =====================
def main():
    rclpy.init()

    node = CmdVelBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    th = threading.Thread(target=executor.spin, daemon=True)
    th.start()

    app = QApplication(sys.argv)
    ui = Joystick(node)
    ui.show()

    def handle_sigint(_sig, _frame):
        try:
            ui.pressed = False
            ui.stop_robot()
        except Exception:
            pass
        ui.close()

    signal.signal(signal.SIGINT, handle_sigint)

    rc = app.exec_()

    try:
        node.publish_cmd(0.0, 0.0)
    except Exception:
        pass

    rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
