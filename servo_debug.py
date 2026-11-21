# -*- coding: utf-8 -*-
"""
舵机调试助手 (Servo Debug Tool)
================================
通过串口向 STM32 下位机发送 PWM 脉宽指令, 实现舵机角度控制。

串口协议 (ASCII 文本, 换行 \\n 结尾):
    PWM:<脉宽us>\\n    设置脉宽, 例: PWM:1500\\n   (对应 STM32 定时器 CCR 值, 单位 us)
    RESET\\n           复位(回到中位)

角度与 PWM 的换算由上位机完成(线性映射):
    pwm = pmin + (pmax - pmin) * angle / max_angle
其中 pmin/pmax 可在界面中自行标定(默认 500 ~ 2500us)。
"""

import json
import os
import sys
import time

import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QMainWindow, QMessageBox,
    QPlainTextEdit, QPushButton, QRadioButton, QSlider, QSpinBox,
    QVBoxLayout, QWidget,
)

# ----------------------------- 常量配置 -----------------------------

SERVO_TYPES = [          # 舵机类型: (显示名, 最大角度)
    ("180° 舵机", 180),
    ("270° 舵机", 270),
    ("90° 舵机", 90),
    ("360° 舵机(脉宽控速型)", 360),
]
BAUD_RATES = ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]

DEFAULT_PWM_MIN = 500    # 默认最小脉宽 us (对应 0°)
DEFAULT_PWM_MAX = 2500   # 默认最大脉宽 us (对应最大角度)
SEND_INTERVAL_MS = 30    # 拖动滑块时的发送节流间隔

# PyInstaller 打包后 __file__ 指向临时目录, 改用 exe 所在路径, 保证配置文件跟随 exe
if getattr(sys, 'frozen', False):
    BASE_DIR = os.path.dirname(os.path.abspath(sys.executable))
else:
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(BASE_DIR, "servo_config.json")


# ----------------------------- 主窗口 -----------------------------

class ServoDebugWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.ser = None                       # 串口对象
        self.max_angle = 180                  # 当前舵机最大角度
        self.pwm_min = DEFAULT_PWM_MIN
        self.pwm_max = DEFAULT_PWM_MAX
        self._updating_ui = False             # 防止控件互相联动时死循环

        # 节流发送: 滑块连续拖动时只按固定频率发送
        self._send_timer = QTimer(self)
        self._send_timer.setSingleShot(True)
        self._send_timer.timeout.connect(self._flush_pending_pwm)
        self._pending_pwm = None

        # 串口接收轮询
        self._read_timer = QTimer(self)
        self._read_timer.timeout.connect(self._poll_serial)

        self._build_ui()
        self._load_config()
        self._refresh_ports()

    # ------------------------- 界面搭建 -------------------------

    def _build_ui(self):
        self.setWindowTitle("舵机调试助手 - STM32 串口控制")
        self.resize(860, 640)

        root = QVBoxLayout()
        root.addWidget(self._build_serial_group())
        root.addWidget(self._build_servo_group())
        root.addWidget(self._build_angle_group())
        root.addWidget(self._build_pwm_group())
        root.addWidget(self._build_log_group())

        container = QWidget()
        container.setLayout(root)
        self.setCentralWidget(container)
        self.statusBar().showMessage("串口未连接")

    def _build_serial_group(self):
        """串口设置区"""
        grp = QGroupBox("串口设置")
        lay = QHBoxLayout(grp)

        lay.addWidget(QLabel("端口:"))
        self.combo_port = QComboBox()
        self.combo_port.setMinimumWidth(160)
        lay.addWidget(self.combo_port)

        btn_refresh = QPushButton("刷新")
        btn_refresh.clicked.connect(self._refresh_ports)
        lay.addWidget(btn_refresh)

        lay.addWidget(QLabel("波特率:"))
        self.combo_baud = QComboBox()
        self.combo_baud.setEditable(True)
        self.combo_baud.addItems(BAUD_RATES)
        self.combo_baud.setCurrentText("115200")
        lay.addWidget(self.combo_baud)

        lay.addWidget(QLabel("(8N1)"))

        self.btn_open = QPushButton("打开串口")
        self.btn_open.clicked.connect(self._toggle_serial)
        lay.addWidget(self.btn_open)

        lay.addStretch()
        return grp

    def _build_servo_group(self):
        """舵机参数区: 类型选择 + 脉宽标定"""
        grp = QGroupBox("舵机参数 (角度 → PWM 线性映射)")
        lay = QGridLayout(grp)

        # 舵机类型
        lay.addWidget(QLabel("舵机类型:"), 0, 0)
        self.combo_type = QComboBox()
        for name, ang in SERVO_TYPES:
            self.combo_type.addItem(name, ang)
        self.combo_type.currentIndexChanged.connect(self._on_type_changed)
        lay.addWidget(self.combo_type, 0, 1)

        # 脉宽范围
        lay.addWidget(QLabel("最小脉宽 (0° 对应):"), 1, 0)
        self.spin_pwm_min = QSpinBox()
        self.spin_pwm_min.setRange(0, 3000)
        self.spin_pwm_min.setValue(DEFAULT_PWM_MIN)
        self.spin_pwm_min.setSuffix(" us")
        self.spin_pwm_min.valueChanged.connect(self._on_pwm_range_changed)
        lay.addWidget(self.spin_pwm_min, 1, 1)

        lay.addWidget(QLabel("最大脉宽 (最大角对应):"), 1, 2)
        self.spin_pwm_max = QSpinBox()
        self.spin_pwm_max.setRange(1, 5000)
        self.spin_pwm_max.setValue(DEFAULT_PWM_MAX)
        self.spin_pwm_max.setSuffix(" us")
        self.spin_pwm_max.valueChanged.connect(self._on_pwm_range_changed)
        lay.addWidget(self.spin_pwm_max, 1, 3)

        self.lbl_mapping = QLabel("")
        self.lbl_mapping.setStyleSheet("color: gray;")
        lay.addWidget(self.lbl_mapping, 2, 0, 1, 4)

        # 常用脉宽预设
        lay.addWidget(QLabel("快速预设:"), 3, 0)
        preset_box = QHBoxLayout()
        for text, lo, hi in [("500~2500us", 500, 2500),
                             ("1000~2000us", 1000, 2000),
                             ("500~2750us", 500, 2750)]:
            btn = QPushButton(text)
            btn.clicked.connect(lambda _, lo=lo, hi=hi: self._apply_preset(lo, hi))
            preset_box.addWidget(btn)
        preset_box.addStretch()
        wrap = QWidget()
        wrap.setLayout(preset_box)
        lay.addWidget(wrap, 3, 1, 1, 3)
        return grp

    def _build_angle_group(self):
        """角度控制区: 滑块 + 数值 + 复位"""
        grp = QGroupBox("角度控制")
        lay = QGridLayout(grp)

        self.slider_angle = QSlider(Qt.Horizontal)
        self.slider_angle.setRange(0, self.max_angle)
        self.slider_angle.setValue(self.max_angle // 2)
        self.slider_angle.valueChanged.connect(self._on_slider_changed)
        lay.addWidget(self.slider_angle, 0, 0, 1, 4)

        lay.addWidget(QLabel("角度:"), 1, 0)
        self.spin_angle = QDoubleSpinBox()
        self.spin_angle.setDecimals(1)
        self.spin_angle.setRange(0, self.max_angle)
        self.spin_angle.setValue(self.max_angle // 2)
        self.spin_angle.setSuffix(" °")
        self.spin_angle.valueChanged.connect(self._on_spin_changed)
        lay.addWidget(self.spin_angle, 1, 1)

        self.chk_autosend = QCheckBox("拖动滑块时自动发送")
        self.chk_autosend.setChecked(True)
        lay.addWidget(self.chk_autosend, 1, 2)

        btn_send_angle = QPushButton("发送角度")
        btn_send_angle.clicked.connect(self._send_current_angle)
        lay.addWidget(btn_send_angle, 1, 3)

        # 快捷位置按钮
        quick_box = QHBoxLayout()
        for text, ratio in [("0°", 0.0), ("中位", 0.5), ("最大", 1.0)]:
            btn = QPushButton(text)
            btn.clicked.connect(lambda _, r=ratio: self._goto_ratio(r))
            quick_box.addWidget(btn)
        quick_box.addStretch()

        # 复位按钮
        self.btn_reset = QPushButton("复位 (回到中位)")
        self.btn_reset.setMinimumHeight(32)
        self.btn_reset.setStyleSheet(
            "QPushButton { background-color: #d9534f; color: white; font-weight: bold; }"
            "QPushButton:hover { background-color: #c9302c; }")
        self.btn_reset.clicked.connect(self._on_reset)
        quick_box.addWidget(self.btn_reset)

        wrap = QWidget()
        wrap.setLayout(quick_box)
        lay.addWidget(wrap, 2, 0, 1, 4)

        self.lbl_current = QLabel("")
        self.lbl_current.setStyleSheet("font-size: 16px; font-weight: bold;")
        lay.addWidget(self.lbl_current, 3, 0, 1, 4)
        return grp

    def _build_pwm_group(self):
        """手动 PWM 控制: 用于标定/微调"""
        grp = QGroupBox("手动 PWM 控制 (微调标定)")
        lay = QHBoxLayout(grp)

        self.spin_pwm_manual = QSpinBox()
        self.spin_pwm_manual.setRange(DEFAULT_PWM_MIN, DEFAULT_PWM_MAX)
        self.spin_pwm_manual.setValue((DEFAULT_PWM_MIN + DEFAULT_PWM_MAX) // 2)
        self.spin_pwm_manual.setSuffix(" us")
        lay.addWidget(self.spin_pwm_manual)

        for delta in (-100, -10, -1):
            btn = QPushButton(f"{delta:+d}")
            btn.setFixedWidth(48)
            btn.clicked.connect(lambda _, d=delta: self._nudge_pwm(d))
            lay.addWidget(btn)

        btn_send_pwm = QPushButton("发送 PWM")
        btn_send_pwm.clicked.connect(self._send_manual_pwm)
        lay.addWidget(btn_send_pwm)

        for delta in (+1, +10, +100):
            btn = QPushButton(f"{delta:+d}")
            btn.setFixedWidth(48)
            btn.clicked.connect(lambda _, d=delta: self._nudge_pwm(d))
            lay.addWidget(btn)

        lay.addStretch()

        self.chk_sync = QCheckBox("发送后同步角度显示")
        self.chk_sync.setChecked(True)
        lay.addWidget(self.chk_sync)
        return grp

    def _build_log_group(self):
        """通信日志区"""
        grp = QGroupBox("通信日志")
        lay = QVBoxLayout(grp)

        tool_box = QHBoxLayout()
        self.chk_hex = QCheckBox("接收按 HEX 显示")
        tool_box.addWidget(self.chk_hex)
        btn_clear = QPushButton("清空日志")
        btn_clear.clicked.connect(lambda: self.txt_log.clear())
        tool_box.addWidget(btn_clear)
        tool_box.addStretch()
        wrap = QWidget()
        wrap.setLayout(tool_box)
        lay.addWidget(wrap)

        self.txt_log = QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumBlockCount(2000)
        lay.addWidget(self.txt_log)
        return grp

    # ------------------------- 角度 / PWM 换算 -------------------------

    def angle_to_pwm(self, angle):
        """角度 → 脉宽 (线性映射)"""
        span = self.pwm_max - self.pwm_min
        pwm = self.pwm_min + span * angle / float(self.max_angle)
        return int(round(pwm))

    def pwm_to_angle(self, pwm):
        """脉宽 → 角度"""
        span = self.pwm_max - self.pwm_min
        angle = (pwm - self.pwm_min) * self.max_angle / float(span)
        return max(0.0, min(float(self.max_angle), angle))

    def center_pwm(self):
        return (self.pwm_min + self.pwm_max) // 2

    # ------------------------- 控件事件 -------------------------

    def _on_type_changed(self, index):
        """切换舵机类型: 调整角度范围并保持当前比例位置"""
        old_max = self.max_angle
        new_max = self.combo_type.itemData(index)
        ratio = self.spin_angle.value() / float(old_max) if old_max else 0.5

        self.max_angle = new_max
        self._updating_ui = True
        self.slider_angle.setRange(0, new_max)
        self.spin_angle.setRange(0, new_max)
        self.spin_angle.setValue(round(new_max * ratio, 1))
        self.slider_angle.setValue(int(round(new_max * ratio)))
        self._updating_ui = False

        self._update_mapping_label()
        self._update_current_label()
        self._log(f"舵机类型切换为 {new_max}°")

    def _on_pwm_range_changed(self):
        """脉宽标定变化: 校验 + 同步手动 PWM 输入范围"""
        lo, hi = self.spin_pwm_min.value(), self.spin_pwm_max.value()
        if lo >= hi:                       # 保证 min < max
            self._updating_ui = True
            self.spin_pwm_max.setValue(lo + 1)
            self._updating_ui = False
            hi = lo + 1
        if self._updating_ui:
            return
        self.pwm_min, self.pwm_max = lo, hi

        mid = (lo + hi) // 2
        self._updating_ui = True
        self.spin_pwm_manual.setRange(lo, hi)
        self.spin_pwm_manual.setValue(mid)
        self._updating_ui = False

        self._update_mapping_label()
        self._update_current_label()

    def _apply_preset(self, lo, hi):
        self.spin_pwm_min.setValue(lo)
        self.spin_pwm_max.setValue(hi)
        self._log(f"应用脉宽预设: {lo}~{hi} us")

    def _on_slider_changed(self, value):
        if self._updating_ui:
            return
        self._updating_ui = True
        self.spin_angle.setValue(float(value))
        self._updating_ui = False
        self._update_current_label()
        if self.chk_autosend.isChecked():
            self._throttled_send()

    def _on_spin_changed(self, value):
        if self._updating_ui:
            return
        self._updating_ui = True
        self.slider_angle.setValue(int(round(value)))
        self._updating_ui = False
        self._update_current_label()

    def _goto_ratio(self, ratio):
        """跳转到比例位置: 0=0°, 0.5=中位, 1=最大"""
        self.spin_angle.setValue(round(self.max_angle * ratio, 1))
        self._send_current_angle()

    def _on_reset(self):
        """复位按钮: 回到中位并发送指令"""
        mid_angle = self.max_angle / 2.0
        self.spin_angle.setValue(mid_angle)
        self.send_line("RESET")                       # 通知下位机复位(可忽略)
        self._send_pwm_value(self.center_pwm())       # 同时发送中位脉宽, 保证必复位
        self._log(f"复位: 角度 {mid_angle:.1f}°, 脉宽 {self.center_pwm()} us")

    def _send_current_angle(self):
        """按当前角度计算 PWM 并发送"""
        angle = self.spin_angle.value()
        self._send_pwm_value(self.angle_to_pwm(angle))

    def _nudge_pwm(self, delta):
        self.spin_pwm_manual.setValue(self.spin_pwm_manual.value() + delta)
        self._send_manual_pwm()

    def _send_manual_pwm(self):
        pwm = self.spin_pwm_manual.value()
        self._send_pwm_value(pwm)
        if self.chk_sync.isChecked():
            # 反算角度并同步显示(不回发)
            angle = self.pwm_to_angle(pwm)
            self._updating_ui = True
            self.spin_angle.setValue(round(angle, 1))
            self.slider_angle.setValue(int(round(angle)))
            self._updating_ui = False
            self._update_current_label()

    # ------------------------- 发送逻辑 -------------------------

    def _throttled_send(self):
        """滑块拖动时按 SEND_INTERVAL_MS 节流发送最新 PWM"""
        self._pending_pwm = self.angle_to_pwm(self.spin_angle.value())
        if not self._send_timer.isActive():
            self._send_timer.start(SEND_INTERVAL_MS)

    def _flush_pending_pwm(self):
        if self._pending_pwm is not None:
            self._send_pwm_value(self._pending_pwm)
            self._pending_pwm = None

    def _send_pwm_value(self, pwm):
        pwm = int(max(self.pwm_min, min(self.pwm_max, pwm)))
        self.send_line(f"PWM:{pwm}")

    def send_line(self, line):
        """发送一行 ASCII 指令"""
        if self.ser is None or not self.ser.is_open:
            self._log("发送失败: 串口未打开", color="red")
            return
        data = (line + "\n").encode("ascii")
        try:
            self.ser.write(data)
            self._log(f"TX: {line}", color="blue")
        except serial.SerialException as exc:
            self._log(f"发送失败: {exc}", color="red")

    # ------------------------- 串口开关 -------------------------

    def _refresh_ports(self):
        self.combo_port.clear()
        for info in serial.tools.list_ports.comports():
            self.combo_port.addItem(f"{info.device} - {info.description}", info.device)
        if self.combo_port.count() == 0:
            self._log("未检测到可用串口, 请确认驱动与连接后点击刷新")

    def _toggle_serial(self):
        if self.ser is None:
            port = self.combo_port.currentData()
            if not port:
                QMessageBox.warning(self, "提示", "请先选择串口")
                return
            try:
                baud = int(self.combo_baud.currentText())
            except ValueError:
                QMessageBox.warning(self, "提示", "波特率必须是数字")
                return
            try:
                self.ser = serial.Serial(
                    port=port, baudrate=baud,
                    bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE, timeout=0)
            except serial.SerialException as exc:
                QMessageBox.critical(self, "错误", f"打开串口失败:\n{exc}")
                self.ser = None
                return
            self.ser.reset_input_buffer()
            self._read_timer.start(10)
            self.btn_open.setText("关闭串口")
            self.statusBar().showMessage(f"已连接 {port} @ {baud}")
            self._log(f"串口已打开: {port} @ {baud} (8N1)", color="green")
        else:
            self._read_timer.stop()
            self._send_timer.stop()
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.btn_open.setText("打开串口")
            self.statusBar().showMessage("串口未连接")
            self._log("串口已关闭", color="green")

    def _poll_serial(self):
        """定时轮询串口接收"""
        if self.ser is None or not self.ser.is_open:
            return
        try:
            n = self.ser.in_waiting
            if n:
                data = self.ser.read(n)
            else:
                return
        except serial.SerialException as exc:
            self._log(f"串口读取异常, 自动断开: {exc}", color="red")
            self._toggle_serial()
            return
        if self.chk_hex.isChecked():
            text = " ".join(f"{b:02X}" for b in data)
        else:
            text = data.decode("utf-8", errors="replace")
        self._log(f"RX: {text}", color="darkgreen")

    # ------------------------- 界面刷新 -------------------------

    def _update_mapping_label(self):
        self.lbl_mapping.setText(
            f"映射: 0° → {self.pwm_min} us,  {self.max_angle}° → {self.pwm_max} us,  "
            f"中位 {self.max_angle / 2:.0f}° → {self.center_pwm()} us")

    def _update_current_label(self):
        angle = self.spin_angle.value()
        pwm = self.angle_to_pwm(angle)
        self.lbl_current.setText(f"当前角度: {angle:.1f}°    当前脉宽: {pwm} us")

    def _log(self, text, color=None):
        stamp = time.strftime("%H:%M:%S")
        if color:
            self.txt_log.appendHtml(
                f'<span style="color:gray">[{stamp}]</span> '
                f'<span style="color:{color}">{text}</span>')
        else:
            self.txt_log.appendPlainText(f"[{stamp}] {text}")

    # ------------------------- 配置持久化 -------------------------

    def _load_config(self):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as fp:
                cfg = json.load(fp)
        except (OSError, ValueError):
            return
        self.combo_baud.setCurrentText(str(cfg.get("baud", "115200")))
        self.combo_type.setCurrentIndex(
            max(0, self.combo_type.findData(cfg.get("max_angle", 180))))
        self.spin_pwm_min.setValue(cfg.get("pwm_min", DEFAULT_PWM_MIN))
        self.spin_pwm_max.setValue(cfg.get("pwm_max", DEFAULT_PWM_MAX))
        self.chk_autosend.setChecked(cfg.get("autosend", True))
        self.chk_hex.setChecked(cfg.get("hex_show", False))
        self._last_port = cfg.get("last_port", "")

    def _save_config(self):
        cfg = {
            "baud": self.combo_baud.currentText(),
            "max_angle": self.max_angle,
            "pwm_min": self.pwm_min,
            "pwm_max": self.pwm_max,
            "autosend": self.chk_autosend.isChecked(),
            "hex_show": self.chk_hex.isChecked(),
            "last_port": self.combo_port.currentData() or "",
        }
        try:
            with open(CONFIG_FILE, "w", encoding="utf-8") as fp:
                json.dump(cfg, fp, ensure_ascii=False, indent=2)
        except OSError:
            pass

    def closeEvent(self, event):
        if self.ser is not None:
            self._toggle_serial()
        self._save_config()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = ServoDebugWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
