import sys

# === 强制指定 Qt 插件路径 ===
import PyQt5
import os

dirname = os.path.dirname(PyQt5.__file__)
plugin_path = os.path.join(dirname, "Qt5", "plugins", "platforms")
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = plugin_path

import math
import time
import threading
from collections import deque

import minimalmodbus
import serial
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QTabWidget,
    QGroupBox,
    QDoubleSpinBox,
    QSpinBox,
    QComboBox,
    QPlainTextEdit,
    QSplitter,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QTextCursor
import pyqtgraph as pg


# ================= 全局设置 =================
pg.setConfigOptions(antialias=True)
FONT = QFont("Microsoft YaHei", 9)
QApplication.setFont(FONT)


# ================= 驱动器通信封装 =================
class MotorDriver:
    def __init__(self, port, slave_addr):
        self.port = port
        self.slave_addr = slave_addr
        self.lock = threading.Lock()
        self.instrument = None
        self.connect()

    def connect(self):
        try:
            self.instrument = minimalmodbus.Instrument(self.port, self.slave_addr)
            self.instrument.serial.baudrate = 115200
            self.instrument.serial.parity = serial.PARITY_EVEN
            self.instrument.serial.stopbits = 1
            self.instrument.serial.timeout = 0.05
            self.instrument.mode = minimalmodbus.MODE_RTU
            return True
        except Exception as e:
            print(f"串口连接失败: {e}")
            return False

    def write_register(self, reg, value, signed=False):
        with self.lock:
            self.instrument.write_register(reg, value, functioncode=6, signed=signed)

    def write_long(self, reg, value, signed=True):
        with self.lock:
            self.instrument.write_long(reg, value, signed=signed)

    def read_register(self, reg, signed=False):
        with self.lock:
            return self.instrument.read_register(reg, functioncode=3, signed=signed)

    def read_long(self, reg, signed=True):
        with self.lock:
            return self.instrument.read_long(reg, functioncode=3, signed=signed)


# ================= 工作线程基类 =================
class BaseWorker(QThread):
    data_updated = pyqtSignal(float, float, float, float, float)
    log_signal = pyqtSignal(str)
    finished_signal = pyqtSignal()

    def __init__(self, driver, params):
        super().__init__()
        self.driver = driver
        self.params = params
        self.running = False
        self.start_time = 0

        # ⚠️修复横线Bug：保存最后一次有效数据，防止读取失败时归零
        self.last_pos = 0
        self.last_cur = 0
        self.last_spd = 0

    def stop(self):
        self.running = False
        self.wait()

    def log(self, msg):
        self.log_signal.emit(msg)

    def read_telemetry(self):
        """通用遥测数据读取（带独立容错），失败时返回上一次有效值"""
        try:
            self.last_pos = self.driver.read_long(0x002C, signed=True)
        except Exception:
            pass

        try:
            cur_raw = self.driver.read_register(0x0011, signed=False)
            self.last_cur = cur_raw * 0.01
        except Exception:
            pass

        try:
            self.last_spd = float(self.driver.read_long(0x001E, signed=True))
        except Exception:
            pass

        return self.last_pos, self.last_cur, self.last_spd

    def run(self):
        self.running = True
        # ⚠️改用高精度时钟，解决正弦波的卡顿感
        self.start_time = time.perf_counter()
        self.log("运动线程启动")
        try:
            self._setup_mode()
        except Exception as e:
            self.log(f"模式初始化失败: {e}")
            self.running = False
            self.finished_signal.emit()
            return

        self._run_loop()
        self._stop_motor()
        self.finished_signal.emit()


# ================= 力矩模式工作线程 =================
class TorqueWorker(BaseWorker):
    def _setup_mode(self):
        self.driver.write_register(0x0086, int(self.params["rated_current"] * 100))
        self.driver.write_register(0x0087, int(self.params["work_current"] * 100))
        self.driver.write_register(0x0081, 0)
        cur = self.driver.read_register(0x0080)
        new_mode = (cur & 0xFF00) | 0x01
        self.driver.write_register(0x0080, new_mode)
        self.log("力矩模式初始化完成")

    def _run_loop(self):
        while self.running:
            # ⚠️实时读取 params，支持外部回车动态修改
            target_torque = int(self.params["target_current"] * 100)
            target_torque = max(-500, min(500, target_torque))

            try:
                self.driver.write_register(0x0040, target_torque, signed=True)
            except Exception:
                pass

            actual_pos, actual_current, actual_speed = self.read_telemetry()
            t = time.perf_counter() - self.start_time
            self.data_updated.emit(
                t, actual_pos, actual_current, actual_speed, target_torque / 100.0
            )
            time.sleep(0.02)

    def _stop_motor(self):
        try:
            self.driver.write_register(0x0040, 0, signed=True)
        except:
            pass
        self.log("力矩归零，电机停止")


# ================= 速度模式工作线程 =================
class SpeedWorker(BaseWorker):
    def _setup_mode(self):
        self.driver.write_register(0x0086, int(self.params["rated_current"] * 100))
        self.driver.write_register(0x0087, int(self.params["work_current"] * 100))
        self.driver.write_register(0x0081, 0)
        cur = self.driver.read_register(0x0080)
        new_mode = (cur & 0xFF00) | 0x00
        self.driver.write_register(0x0080, new_mode)
        self.log("速度模式(开环)初始化完成")

    def _run_loop(self):
        while self.running:
            # ⚠️实时读取 params
            target_duty = self.params["target_duty"]
            duty_value = max(-1000, min(1000, int(target_duty * 10)))

            try:
                self.driver.write_register(0x0040, duty_value, signed=True)
            except Exception:
                pass

            actual_pos, actual_current, actual_speed = self.read_telemetry()
            t = time.perf_counter() - self.start_time
            self.data_updated.emit(
                t, actual_pos, actual_current, actual_speed, target_duty
            )
            time.sleep(0.02)

    def _stop_motor(self):
        try:
            self.driver.write_register(0x0040, 0, signed=True)
        except:
            pass
        self.log("速度归零，电机停止")


# ================= 位置模式工作线程 =================
class PositionWorker(BaseWorker):
    def _setup_mode(self):
        self.driver.write_register(0x0068, 2)
        self.driver.write_register(0x0069, self.params["encoder_lines"])
        self.driver.write_long(0x006C, self.params["rated_rpm"], signed=False)
        self.driver.write_register(0x00C1, 2)
        self.driver.write_register(0x0046, self.params["max_speed"])
        cur = self.driver.read_register(0x0080)
        new_mode = (cur & 0xFF00) | 0x03
        self.driver.write_register(0x0080, new_mode)
        self.driver.write_register(0x700A, 1)
        time.sleep(0.1)
        self.driver.write_register(0x0081, 0)
        self.log("位置模式初始化完成")

    def _run_loop(self):
        while self.running:
            # ⚠️每次循环都读取最新参数，支持动态调幅调频
            amp = self.params["amplitude"]
            freq = self.params["frequency"]
            wave_type = self.params["wave_type"]
            steps = self.params.get("scurve_steps", 3)

            t = time.perf_counter() - self.start_time
            target = 0

            try:
                if wave_type == "sine":
                    target = int(amp * math.sin(2 * math.pi * freq * t))
                    self.driver.write_long(0x0048, target, signed=True)
                    time.sleep(0.01)
                elif wave_type == "square":
                    if int(t * freq * 2) % 2 == 0:
                        target = amp
                    else:
                        target = -amp
                    self.driver.write_long(0x0048, target, signed=True)
                    time.sleep(0.01)
                elif wave_type == "scurve":
                    period = 1.0 / freq
                    phase = (t % period) / period
                    if phase < 0.5:
                        p = phase * 2
                        smooth = 3 * p**2 - 2 * p**3
                        target = int(-amp + 2 * amp * smooth)
                    else:
                        p = (phase - 0.5) * 2
                        smooth = 3 * p**2 - 2 * p**3
                        target = int(amp - 2 * amp * smooth)
                    self.driver.write_long(0x0048, target, signed=True)
                    time.sleep(0.01)
            except Exception:
                pass

            actual_pos, actual_current, actual_speed = self.read_telemetry()
            self.data_updated.emit(t, actual_pos, actual_current, actual_speed, target)

    def _stop_motor(self):
        try:
            self.driver.write_register(0x0046, 200)
            self.driver.write_long(0x0048, 0, signed=True)
        except:
            pass
        self.log("位置归零命令已发送")


# ================= 主窗口 =================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.driver = MotorDriver("COM12", 1)
        if not self.driver.instrument:
            self.setWindowTitle("连接失败 - 请检查串口")
            # 即使没连上也不崩溃退出，方便调试界面

        self.current_worker = None

        # 优化UI渲染：分离数据接收和图表刷新
        self.data_buffer = {
            "time": deque(maxlen=500),
            "pos": deque(maxlen=500),
            "current": deque(maxlen=500),
            "speed": deque(maxlen=500),
            "cmd": deque(maxlen=500),
        }
        self.latest_vals = (0, 0, 0, 0)

        self.init_ui()
        self.setup_plot()

        # 使用定时器以 30FPS 刷新UI，防止图表卡死主线程
        self.render_timer = QTimer()
        self.render_timer.timeout.connect(self.update_gui_plots)
        self.render_timer.start(33)

    def init_ui(self):
        self.setWindowTitle("🦋 蝴蝶扑翼机 - 极客控制终端")
        self.setGeometry(100, 100, 1400, 900)
        self.setStyleSheet("""
            QMainWindow { background-color: #1e1e1e; }
            QGroupBox { color: #ffffff; border: 1px solid #555; border-radius: 5px; margin-top: 10px; padding-top: 10px; font-weight: bold;}
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #00FFFF;}
            QLabel { color: #ffffff; }
            QPushButton { background-color: #007ACC; color: white; border: none; padding: 8px 10px; border-radius: 3px; font-weight: bold;}
            QPushButton:hover { background-color: #0098FF; }
            QPushButton:pressed { background-color: #005C99; }
            QTabWidget::pane { background-color: #252526; border: 1px solid #555; }
            QTabBar::tab { background-color: #2D2D30; color: #aaa; padding: 8px 15px; }
            QTabBar::tab:selected { background-color: #007ACC; color: white; }
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox { background-color: #3E3E42; color: #00FFFF; border: 1px solid #555; padding: 4px; }
            QPlainTextEdit { background-color: #111111; color: #00FF41; font-family: Consolas; border: 1px solid #555;}
        """)

        main_splitter = QSplitter(Qt.Horizontal)

        self.tab_widget = QTabWidget()
        self.torque_tab = self.create_torque_tab()
        self.speed_tab = self.create_speed_tab()
        self.position_tab = self.create_position_tab()
        self.tab_widget.addTab(self.position_tab, "位置模式 (主控)")
        self.tab_widget.addTab(self.speed_tab, "速度模式")
        self.tab_widget.addTab(self.torque_tab, "力矩模式")
        main_splitter.addWidget(self.tab_widget)

        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)

        realtime_group = QGroupBox("实时遥测 HUD")
        realtime_layout = QGridLayout(realtime_group)
        self.lbl_current = QLabel("0.00 A")
        self.lbl_current.setStyleSheet(
            "color: #FF3131; font-size: 16px; font-weight: bold;"
        )
        self.lbl_speed = QLabel("0 RPM")
        self.lbl_speed.setStyleSheet(
            "color: #39FF14; font-size: 16px; font-weight: bold;"
        )
        self.lbl_pos = QLabel("0")
        self.lbl_pos.setStyleSheet(
            "color: #00FFFF; font-size: 16px; font-weight: bold;"
        )
        self.lbl_cmd = QLabel("0")
        self.lbl_cmd.setStyleSheet(
            "color: #FF00FF; font-size: 16px; font-weight: bold;"
        )
        realtime_layout.addWidget(QLabel("🔥 电流 (AMP):"), 0, 0)
        realtime_layout.addWidget(self.lbl_current, 0, 1)
        realtime_layout.addWidget(QLabel("⚙️ 速度 (RPM):"), 0, 2)
        realtime_layout.addWidget(self.lbl_speed, 0, 3)
        realtime_layout.addWidget(QLabel("🎯 实际位置:"), 1, 0)
        realtime_layout.addWidget(self.lbl_pos, 1, 1)
        realtime_layout.addWidget(QLabel("📌 指令位置:"), 1, 2)
        realtime_layout.addWidget(self.lbl_cmd, 1, 3)
        right_layout.addWidget(realtime_group)

        plot_group = QGroupBox("波形示波器")
        plot_layout = QVBoxLayout(plot_group)
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground("#111116")
        plot_layout.addWidget(self.plot_widget)
        right_layout.addWidget(plot_group)

        terminal_group = QGroupBox("系统日志终端")
        terminal_layout = QVBoxLayout(terminal_group)
        self.terminal = QPlainTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setMaximumBlockCount(200)
        terminal_layout.addWidget(self.terminal)
        right_layout.addWidget(terminal_group)

        main_splitter.addWidget(right_widget)
        main_splitter.setSizes([350, 1050])
        self.setCentralWidget(main_splitter)

    # ⚠️ 新增核心：回车/修改完成时，立刻下发新参数给线程
    def update_live_params(self):
        if self.current_worker and self.current_worker.isRunning():
            if isinstance(self.current_worker, PositionWorker):
                self.current_worker.params["amplitude"] = self.pos_amplitude.value()
                self.current_worker.params["frequency"] = self.pos_frequency.value()
                self.current_worker.params["wave_type"] = (
                    self.pos_wave_type.currentText()
                )
                self.current_worker.params["scurve_steps"] = (
                    self.pos_scurve_steps.value()
                )
                self.on_log(">> 动态指令已下发：位置参数实时更新")
            elif isinstance(self.current_worker, SpeedWorker):
                self.current_worker.params["target_duty"] = (
                    self.speed_target_duty.value()
                )
                self.on_log(">> 动态指令已下发：占空比实时更新")
            elif isinstance(self.current_worker, TorqueWorker):
                self.current_worker.params["target_current"] = (
                    self.torque_target_current.value()
                )
                self.on_log(">> 动态指令已下发：力矩实时更新")

    def create_torque_tab(self):
        widget = QWidget()
        layout = QGridLayout(widget)
        self.torque_rated_current = QDoubleSpinBox()
        self.torque_rated_current.setRange(0.1, 5.0)
        self.torque_rated_current.setValue(0.5)
        self.torque_work_current = QDoubleSpinBox()
        self.torque_work_current.setRange(0.1, 5.0)
        self.torque_work_current.setValue(0.5)
        self.torque_target_current = QDoubleSpinBox()
        self.torque_target_current.setRange(-5.0, 5.0)
        self.torque_target_current.setValue(0.1)
        self.torque_start_btn = QPushButton("▶ 启动力矩模式")
        self.torque_stop_btn = QPushButton("⏹ 紧急停止")
        self.torque_stop_btn.setStyleSheet("background-color: #CC0000;")

        layout.addWidget(QLabel("额定电流 (A):"), 0, 0)
        layout.addWidget(self.torque_rated_current, 0, 1)
        layout.addWidget(QLabel("工作电流 (A):"), 1, 0)
        layout.addWidget(self.torque_work_current, 1, 1)
        layout.addWidget(QLabel("目标力矩 (A):"), 2, 0)
        layout.addWidget(self.torque_target_current, 2, 1)
        layout.addWidget(self.torque_start_btn, 3, 0, 1, 2)
        layout.addWidget(self.torque_stop_btn, 4, 0, 1, 2)
        layout.setRowStretch(5, 1)

        self.torque_start_btn.clicked.connect(lambda: self.start_motion("torque"))
        self.torque_stop_btn.clicked.connect(self.stop_motion)
        # 绑定回车/失焦实时生效
        self.torque_target_current.editingFinished.connect(self.update_live_params)
        return widget

    def create_speed_tab(self):
        widget = QWidget()
        layout = QGridLayout(widget)
        self.speed_rated_current = QDoubleSpinBox()
        self.speed_rated_current.setRange(0.1, 5.0)
        self.speed_rated_current.setValue(0.5)
        self.speed_work_current = QDoubleSpinBox()
        self.speed_work_current.setRange(0.1, 5.0)
        self.speed_work_current.setValue(0.5)
        self.speed_target_duty = QDoubleSpinBox()
        self.speed_target_duty.setRange(-100, 100)
        self.speed_target_duty.setValue(20)
        self.speed_start_btn = QPushButton("▶ 启动速度模式")
        self.speed_stop_btn = QPushButton("⏹ 紧急停止")
        self.speed_stop_btn.setStyleSheet("background-color: #CC0000;")

        layout.addWidget(QLabel("额定电流 (A):"), 0, 0)
        layout.addWidget(self.speed_rated_current, 0, 1)
        layout.addWidget(QLabel("工作电流 (A):"), 1, 0)
        layout.addWidget(self.speed_work_current, 1, 1)
        layout.addWidget(QLabel("目标占空比 (%):"), 2, 0)
        layout.addWidget(self.speed_target_duty, 2, 1)
        layout.addWidget(self.speed_start_btn, 3, 0, 1, 2)
        layout.addWidget(self.speed_stop_btn, 4, 0, 1, 2)
        layout.setRowStretch(5, 1)

        self.speed_start_btn.clicked.connect(lambda: self.start_motion("speed"))
        self.speed_stop_btn.clicked.connect(self.stop_motion)
        # 绑定回车/失焦实时生效
        self.speed_target_duty.editingFinished.connect(self.update_live_params)
        return widget

    def create_position_tab(self):
        widget = QWidget()
        layout = QGridLayout(widget)
        self.pos_encoder_lines = QSpinBox()
        self.pos_encoder_lines.setRange(100, 10000)
        self.pos_encoder_lines.setValue(700)
        self.pos_rated_rpm = QSpinBox()
        self.pos_rated_rpm.setRange(1000, 20000)
        self.pos_rated_rpm.setValue(6000)
        self.pos_max_speed = QSpinBox()
        self.pos_max_speed.setRange(100, 20000)
        self.pos_max_speed.setValue(18000)

        # ⚠️ 运动核心参数，允许实时修改
        self.pos_amplitude = QSpinBox()
        self.pos_amplitude.setRange(1, 10000)
        self.pos_amplitude.setValue(128)
        self.pos_frequency = QDoubleSpinBox()
        self.pos_frequency.setRange(0.1, 50)
        self.pos_frequency.setValue(4.0)
        self.pos_wave_type = QComboBox()
        self.pos_wave_type.addItems(["scurve", "sine", "square"])
        self.pos_scurve_steps = QSpinBox()
        self.pos_scurve_steps.setRange(2, 20)
        self.pos_scurve_steps.setValue(4)

        self.pos_start_btn = QPushButton("▶ 启动扑翼模式")
        self.pos_stop_btn = QPushButton("⏹ 紧急停止")
        self.pos_stop_btn.setStyleSheet("background-color: #CC0000;")

        layout.addWidget(QLabel("编码器线数:"), 0, 0)
        layout.addWidget(self.pos_encoder_lines, 0, 1)
        layout.addWidget(QLabel("最大追踪速度 (RPM):"), 1, 0)
        layout.addWidget(self.pos_max_speed, 1, 1)

        lbl = QLabel("--- 动态运行参数 ---")
        lbl.setStyleSheet("color:#00FFFF; margin-top:10px;")
        layout.addWidget(lbl, 2, 0, 1, 2)
        layout.addWidget(QLabel("摆动幅值 (脉冲) [回车生效]:"), 3, 0)
        layout.addWidget(self.pos_amplitude, 3, 1)
        layout.addWidget(QLabel("摆动频率 (Hz) [回车生效]:"), 4, 0)
        layout.addWidget(self.pos_frequency, 4, 1)
        layout.addWidget(QLabel("波形轨迹类型:"), 5, 0)
        layout.addWidget(self.pos_wave_type, 5, 1)
        layout.addWidget(QLabel("S曲线分段数:"), 6, 0)
        layout.addWidget(self.pos_scurve_steps, 6, 1)

        layout.addWidget(self.pos_start_btn, 7, 0, 1, 2)
        layout.addWidget(self.pos_stop_btn, 8, 0, 1, 2)
        layout.setRowStretch(9, 1)

        self.pos_start_btn.clicked.connect(lambda: self.start_motion("position"))
        self.pos_stop_btn.clicked.connect(self.stop_motion)

        # 绑定回车/失焦/切换下拉框 时实时生效
        self.pos_amplitude.editingFinished.connect(self.update_live_params)
        self.pos_frequency.editingFinished.connect(self.update_live_params)
        self.pos_wave_type.currentIndexChanged.connect(self.update_live_params)
        self.pos_scurve_steps.editingFinished.connect(self.update_live_params)
        return widget

    def setup_plot(self):
        self.plot_widget.clear()

        # 定义极客配色画笔
        pen_cmd = pg.mkPen(color="#00FFFF", width=1.5, style=Qt.DashLine)  # 指令虚线
        pen_pos = pg.mkPen(color="#FF00FF", width=2.0)  # 实际位置粉色实线
        pen_cur = pg.mkPen(color="#FF3131", width=1.5)
        pen_spd = pg.mkPen(color="#39FF14", width=1.5)

        self.p1 = self.plot_widget.addPlot(
            row=0, col=0, title="<span style='color: #00FFFF;'>POSITION TRACKING</span>"
        )
        self.p1.showGrid(True, True, alpha=0.2)
        self.p1.addLegend(offset=(10, 10))
        self.cmd_curve = self.p1.plot(pen=pen_cmd, name="指令 (CMD)")
        self.pos_curve = self.p1.plot(pen=pen_pos, name="实际 (ACT)")

        self.p2 = self.plot_widget.addPlot(
            row=1, col=0, title="<span style='color: #FF3131;'>MOTOR CURRENT (A)</span>"
        )
        self.p2.showGrid(True, True, alpha=0.2)
        self.cur_curve = self.p2.plot(pen=pen_cur)

        self.p3 = self.plot_widget.addPlot(
            row=2, col=0, title="<span style='color: #39FF14;'>MOTOR SPEED (RPM)</span>"
        )
        self.p3.showGrid(True, True, alpha=0.2)
        self.spd_curve = self.p3.plot(pen=pen_spd)

        self.plot_widget.setContentsMargins(10, 10, 10, 10)

    def start_motion(self, mode):
        if self.current_worker and self.current_worker.isRunning():
            self.stop_motion()
            time.sleep(0.2)

        # 清空图表缓存，干干净净重新画
        for key in self.data_buffer:
            self.data_buffer[key].clear()

        if mode == "torque":
            params = {
                "rated_current": self.torque_rated_current.value(),
                "work_current": self.torque_work_current.value(),
                "target_current": self.torque_target_current.value(),
            }
            self.worker = TorqueWorker(self.driver, params)
        elif mode == "speed":
            params = {
                "rated_current": self.speed_rated_current.value(),
                "work_current": self.speed_work_current.value(),
                "target_duty": self.speed_target_duty.value(),
            }
            self.worker = SpeedWorker(self.driver, params)
        elif mode == "position":
            params = {
                "encoder_lines": self.pos_encoder_lines.value(),
                "rated_rpm": self.pos_rated_rpm.value(),
                "max_speed": self.pos_max_speed.value(),
                "amplitude": self.pos_amplitude.value(),
                "frequency": self.pos_frequency.value(),
                "wave_type": self.pos_wave_type.currentText(),
                "scurve_steps": self.pos_scurve_steps.value(),
            }
            self.worker = PositionWorker(self.driver, params)

        self.worker.data_updated.connect(self.on_data_updated)
        self.worker.log_signal.connect(self.on_log)
        self.worker.finished_signal.connect(self.on_worker_finished)
        self.current_worker = self.worker
        self.worker.start()

    def stop_motion(self):
        if self.current_worker and self.current_worker.isRunning():
            self.current_worker.stop()
            self.current_worker = None
            self.on_log("⚡ 收到用户停止指令")

    def on_data_updated(self, t, pos, current, speed, cmd):
        """仅仅用于接收数据并存入内存，彻底解决UI卡顿"""
        self.data_buffer["time"].append(t)
        self.data_buffer["pos"].append(pos)
        self.data_buffer["current"].append(current)
        self.data_buffer["speed"].append(speed)
        self.data_buffer["cmd"].append(cmd)
        # 暂存最新数值供UI刷新
        self.latest_vals = (pos, current, speed, cmd)

    def update_gui_plots(self):
        """由定时器触发，批量渲染图表"""
        if len(self.data_buffer["time"]) < 2:
            return

        pos, current, speed, cmd = self.latest_vals
        self.lbl_current.setText(f"{current:.2f} A")
        self.lbl_speed.setText(f"{speed:.0f} RPM")
        self.lbl_pos.setText(f"{pos:.0f}")
        self.lbl_cmd.setText(f"{cmd:.2f}" if isinstance(cmd, float) else f"{cmd:.0f}")

        times = list(self.data_buffer["time"])
        self.pos_curve.setData(times, list(self.data_buffer["pos"]))
        self.cmd_curve.setData(times, list(self.data_buffer["cmd"]))
        self.cur_curve.setData(times, list(self.data_buffer["current"]))
        self.spd_curve.setData(times, list(self.data_buffer["speed"]))

        # 平滑滚动 X 轴
        x_max = times[-1]
        x_min = max(0, x_max - 5)  # 始终显示最近 5 秒的数据
        self.p1.setXRange(x_min, x_max, padding=0)
        self.p2.setXRange(x_min, x_max, padding=0)
        self.p3.setXRange(x_min, x_max, padding=0)

    def on_log(self, msg):
        # ⚠️修复终端不显示：必须使用 appendPlainText
        self.terminal.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {msg}")
        self.terminal.moveCursor(QTextCursor.End)

    def on_worker_finished(self):
        self.current_worker = None
        self.on_log("✅ 运动线程已安全结束")

    def closeEvent(self, event):
        self.stop_motion()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
