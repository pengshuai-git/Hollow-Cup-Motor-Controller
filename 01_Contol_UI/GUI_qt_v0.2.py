import sys
import os

# === 强制指定 Qt 插件路径 ===
import PyQt5

plugin_path = os.path.join(
    os.path.dirname(PyQt5.__file__), "Qt5", "plugins", "platforms"
)
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = plugin_path

import math
import time
import threading
from collections import deque

import minimalmodbus
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QTextCursor
import pyqtgraph as pg

# ================= 全局设置 =================
pg.setConfigOptions(antialias=True)
FONT = QFont("Microsoft YaHei", 9)


# ================= 驱动器通信封装 =================
class MotorDriver:
    def __init__(self):
        self.lock = threading.Lock()
        self.instrument = None

    def connect(self, port, baud, slave):
        with self.lock:
            try:
                if self.instrument and self.instrument.serial:
                    self.instrument.serial.close()
                self.instrument = minimalmodbus.Instrument(port, slave)
                self.instrument.serial.baudrate = baud
                self.instrument.serial.parity = serial.PARITY_EVEN
                self.instrument.serial.stopbits = 1
                self.instrument.serial.timeout = 0.05
                self.instrument.mode = minimalmodbus.MODE_RTU
                return True
            except:
                return False

    def disconnect(self):
        with self.lock:
            if self.instrument and self.instrument.serial:
                self.instrument.serial.close()
                self.instrument = None

    def is_connected(self):
        return self.instrument is not None

    def emergency_stop(self):
        """硬件级急停（强制切回开环，清空所有指令）"""
        try:
            with self.lock:
                if self.instrument:
                    cur = self.instrument.read_register(0x0080, functioncode=3)
                    self.instrument.write_register(0x0080, cur & 0xFF00, functioncode=6)
                    self.instrument.write_register(
                        0x0040, 0, functioncode=6, signed=True
                    )
                    self.instrument.write_register(
                        0x0042, 0, functioncode=6, signed=True
                    )
                    self.instrument.write_long(0x0048, 0, signed=True)
        except:
            pass

    def write_reg(self, reg, val, signed=True):
        try:
            with self.lock:
                self.instrument.write_register(reg, val, functioncode=6, signed=signed)
        except:
            pass

    def write_long(self, reg, val, signed=True):
        try:
            with self.lock:
                self.instrument.write_long(reg, val, signed=signed)
        except:
            pass

    def read_reg(self, reg, signed=False):
        try:
            with self.lock:
                return self.instrument.read_register(reg, functioncode=3, signed=signed)
        except:
            return 0

    def read_long(self, reg, signed=True):
        try:
            with self.lock:
                return self.instrument.read_long(reg, functioncode=3, signed=signed)
        except:
            return 0


# ================= 工作线程基类 =================
class BaseWorker(QThread):
    data_signal = pyqtSignal(tuple)  # (时间, 位置, 电流, 速度, 指令)
    log_signal = pyqtSignal(str)
    finished_signal = pyqtSignal()

    def __init__(self, driver, params):
        super().__init__()
        self.driver = driver
        self.params = params
        self.running = False
        self.start_time = 0
        self.last_p = 0
        self.last_c = 0.0
        self.last_s = 0.0

    def stop(self):
        self.running = False

    def log(self, msg):
        self.log_signal.emit(msg)

    def read_telemetry(self):
        if not self.driver.is_connected():
            return self.last_p, self.last_c, self.last_s
        try:
            self.last_p = self.driver.read_long(0x002C)
        except:
            pass
        try:
            self.last_c = self.driver.read_reg(0x0011, signed=False) * 0.01
        except:
            pass
        try:
            self.last_s = float(self.driver.read_long(0x001E))
        except:
            pass
        return self.last_p, self.last_c, self.last_s

    def run(self):
        self.running = True
        self.start_time = time.perf_counter()
        self.data_signal.emit(("CLEAR",))
        self.log(f"[{self.__class__.__name__}] 启动")
        try:
            self._setup_mode()
        except Exception as e:
            self.log(f"初始化失败: {e}")
            self.running = False
            self.finished_signal.emit()
            return
        self._run_loop()
        self._stop_motor()
        self.finished_signal.emit()

    def _setup_mode(self):
        raise NotImplementedError

    def _run_loop(self):
        raise NotImplementedError

    def _stop_motor(self):
        pass


# ================= 各模式实现 =================
class PurePositionWorker(BaseWorker):
    def _setup_mode(self):
        self.driver.write_reg(0x0068, 2)  # 编码器反馈
        self.driver.write_reg(0x0069, self.params["encoder_lines"])
        self.driver.write_reg(0x00C1, 2)  # 非固定区间位置控制
        cur = self.driver.read_reg(0x0080, signed=False)
        self.driver.write_reg(0x0080, (cur & 0xFF00) | 0x03)  # 外接测速闭环
        self.driver.write_reg(0x0046, self.params["move_speed"])

    def _run_loop(self):
        while self.running:
            p, c, s = self.read_telemetry()
            self.data_signal.emit((time.perf_counter(), p, c, s, p))
            time.sleep(0.02)


class FlappingWorker(BaseWorker):
    def _setup_mode(self):
        self.driver.write_reg(0x0068, 2)
        self.driver.write_reg(0x0069, self.params["encoder_lines"])
        self.driver.write_long(0x006C, self.params["rated_rpm"], signed=False)
        self.driver.write_reg(0x00C1, 2)
        self.driver.write_reg(0x0046, self.params["max_speed"])
        self.driver.write_reg(0x0081, 0)  # 禁用堵转保护
        cur = self.driver.read_reg(0x0080, signed=False)
        self.driver.write_reg(0x0080, (cur & 0xFF00) | 0x03)

    def _run_loop(self):
        amp = self.params["amplitude"]
        freq = self.params["frequency"]
        wave = self.params["wave_type"]
        while self.running:
            t = time.perf_counter() - self.start_time
            target = 0
            try:
                if wave == "sine":
                    target = int(amp * math.sin(2 * math.pi * freq * t))
                elif wave == "square":
                    target = amp if (int(t * freq * 2) % 2 == 0) else -amp
                elif wave == "scurve":
                    period = 1.0 / freq
                    phase = (t % period) / period
                    p = phase * 2 if phase < 0.5 else (phase - 0.5) * 2
                    smooth = 3 * p**2 - 2 * p**3
                    target = (
                        int(-amp + 2 * amp * smooth)
                        if phase < 0.5
                        else int(amp - 2 * amp * smooth)
                    )
                self.driver.write_long(0x0048, target)
            except:
                pass
            p, c, s = self.read_telemetry()
            self.data_signal.emit((t, p, c, s, target))
            time.sleep(0.015)  # 约 66 Hz 更新


class ClosedSpeedWorker(BaseWorker):
    def _setup_mode(self):
        self.driver.write_reg(0x0068, 2)
        self.driver.write_reg(0x0069, self.params["encoder_lines"])
        self.driver.write_reg(0x0081, 0)
        self.driver.write_reg(0x00C1, 0)  # 速度闭环模式
        cur = self.driver.read_reg(0x0080, signed=False)
        self.driver.write_reg(0x0080, (cur & 0xFF00) | 0x03)  # 外接测速闭环
        self.driver.write_reg(0x0046, 20000)  # 放开速度上限

    def _run_loop(self):
        target_rpm = int(self.params["target_rpm"])
        while self.running:
            self.driver.write_reg(0x0040, target_rpm, signed=True)
            p, c, s = self.read_telemetry()
            self.data_signal.emit(
                (time.perf_counter() - self.start_time, p, c, s, target_rpm)
            )
            time.sleep(0.02)


class SpeedWorker(BaseWorker):
    def _setup_mode(self):
        self.driver.write_reg(0x0086, int(self.params["rated_current"] * 100))
        self.driver.write_reg(0x0087, int(self.params["work_current"] * 100))
        self.driver.write_reg(0x0081, 0)
        cur = self.driver.read_reg(0x0080, signed=False)
        self.driver.write_reg(0x0080, (cur & 0xFF00) | 0x00)  # 开环占空比模式

    def _run_loop(self):
        duty = max(-1000, min(1000, int(self.params["target_duty"] * 10)))
        while self.running:
            self.driver.write_reg(0x0040, duty, signed=True)
            p, c, s = self.read_telemetry()
            self.data_signal.emit(
                (
                    time.perf_counter() - self.start_time,
                    p,
                    c,
                    s,
                    self.params["target_duty"],
                )
            )
            time.sleep(0.02)


class TorqueWorker(BaseWorker):
    def _setup_mode(self):
        self.driver.write_reg(0x0086, int(self.params["rated_current"] * 100))
        self.driver.write_reg(0x0087, int(self.params["work_current"] * 100))
        self.driver.write_reg(0x0081, 0)
        cur = self.driver.read_reg(0x0080, signed=False)
        self.driver.write_reg(0x0080, (cur & 0xFF00) | 0x01)  # 力矩模式

    def _run_loop(self):
        target = max(-500, min(500, int(self.params["target_current"] * 100)))
        while self.running:
            self.driver.write_reg(0x0040, target, signed=True)
            p, c, s = self.read_telemetry()
            self.data_signal.emit(
                (time.perf_counter() - self.start_time, p, c, s, target / 100.0)
            )
            time.sleep(0.02)


# ================= 主窗口 =================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.driver = MotorDriver()
        self.current_worker = None
        self.frame_counter = 0
        self.data_buffer = {
            "time": deque(maxlen=400),
            "pos": deque(maxlen=400),
            "current": deque(maxlen=400),
            "speed": deque(maxlen=400),
            "cmd": deque(maxlen=400),
        }
        self.init_ui()
        self.setup_plot()
        self.render_timer = QTimer()
        self.render_timer.timeout.connect(self.update_plots)
        self.render_timer.start(33)  # 约 30 fps
        self.scan_ports()

    def init_ui(self):
        self.setWindowTitle("AQMD2405 电机控制系统")
        self.setGeometry(100, 100, 1400, 850)
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; }
            QGroupBox { color: white; border: 1px solid #555; border-radius: 4px; margin-top: 1.5em; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }
            QLabel { color: #ddd; }
            QPushButton { background-color: #4CAF50; color: white; border: none; padding: 6px 12px; border-radius: 3px; }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:pressed { background-color: #3d8b40; }
            QPushButton#btnStop { background-color: #f44336; }
            QPushButton#btnStop:hover { background-color: #da190b; }
            QTabWidget::pane { background: #333; border: 1px solid #444; }
            QTabBar::tab { background: #444; color: #ddd; padding: 8px 15px; }
            QTabBar::tab:selected { background: #555; color: white; border-top: 2px solid #4CAF50; }
            QSpinBox, QDoubleSpinBox, QComboBox, QLineEdit { background: #444; color: white; border: 1px solid #666; }
            QPlainTextEdit { background: #1e1e1e; color: #0f0; font-family: Consolas; }
        """)

        main_splitter = QSplitter(Qt.Horizontal)

        # ---------- 左侧控制区 ----------
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)

        # 串口设置
        comm_group = QGroupBox("串口通讯")
        comm_layout = QGridLayout(comm_group)
        self.cb_port = QComboBox()
        self.btn_refresh = QPushButton("刷新")
        self.btn_refresh.clicked.connect(self.scan_ports)
        self.cb_baud = QComboBox()
        self.cb_baud.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.cb_baud.setCurrentText("115200")
        self.sb_id = QSpinBox()
        self.sb_id.setRange(1, 247)
        self.sb_id.setValue(1)
        self.btn_connect = QPushButton("连接")
        self.btn_connect.clicked.connect(self.toggle_connection)
        comm_layout.addWidget(QLabel("COM口:"), 0, 0)
        comm_layout.addWidget(self.cb_port, 0, 1)
        comm_layout.addWidget(self.btn_refresh, 0, 2)
        comm_layout.addWidget(QLabel("波特率:"), 1, 0)
        comm_layout.addWidget(self.cb_baud, 1, 1)
        comm_layout.addWidget(QLabel("从站ID:"), 2, 0)
        comm_layout.addWidget(self.sb_id, 2, 1)
        comm_layout.addWidget(self.btn_connect, 2, 2)
        left_layout.addWidget(comm_group)

        # ---------- 新增：清除数据 / 重置视图 按钮栏 ----------
        ctrl_layout = QHBoxLayout()
        self.btn_clear_data = QPushButton("🧹 清除数据")
        self.btn_clear_data.setStyleSheet("background-color: #ff9800;")
        self.btn_clear_data.clicked.connect(self.clear_data)
        self.btn_reset_view = QPushButton("🔄 重置视图")
        self.btn_reset_view.setStyleSheet("background-color: #2196F3;")
        self.btn_reset_view.clicked.connect(self.reset_view)
        ctrl_layout.addWidget(self.btn_clear_data)
        ctrl_layout.addWidget(self.btn_reset_view)
        left_layout.addLayout(ctrl_layout)

        # 模式选项卡
        self.tab_widget = QTabWidget()
        self.tab_widget.addTab(self.create_pure_pos_tab(), "纯位置模式")
        self.tab_widget.addTab(self.create_flapping_tab(), "扑翼轨迹")
        self.tab_widget.addTab(self.create_closed_speed_tab(), "闭环速度")
        self.tab_widget.addTab(self.create_speed_tab(), "开环速度")
        self.tab_widget.addTab(self.create_torque_tab(), "力矩模式")
        left_layout.addWidget(self.tab_widget)

        main_splitter.addWidget(left_widget)

        # ---------- 右侧区域 ----------
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)

        # 垂直分割器，用于调节三个区域的高度
        v_splitter = QSplitter(Qt.Vertical)

        # 1. 实时遥测区域
        realtime_group = QGroupBox("实时遥测监控")
        realtime_layout = QGridLayout(realtime_group)
        self.lbl_current = QLabel("0.00 A")
        self.lbl_current.setStyleSheet(
            "color:#ffeb3b; font-size:16px; font-weight:bold;"
        )
        self.lbl_speed = QLabel("0 RPM")
        self.lbl_speed.setStyleSheet("color:#4caf50; font-size:16px; font-weight:bold;")
        self.lbl_pos = QLabel("0")
        self.lbl_pos.setStyleSheet("color:#2196F3; font-size:16px; font-weight:bold;")
        self.lbl_cmd = QLabel("0")
        self.lbl_cmd.setStyleSheet("color:#e91e63; font-size:16px; font-weight:bold;")

        realtime_layout.addWidget(QLabel("电流:"), 0, 0)
        realtime_layout.addWidget(self.lbl_current, 0, 1)
        realtime_layout.addWidget(QLabel("速度:"), 0, 2)
        realtime_layout.addWidget(self.lbl_speed, 0, 3)
        realtime_layout.addWidget(QLabel("位置:"), 1, 0)

        # 位置数值 + 清零小按钮
        pos_layout = QHBoxLayout()
        pos_layout.setContentsMargins(0, 0, 0, 0)
        pos_layout.addWidget(self.lbl_pos)
        self.btn_clear_pos = QPushButton("0")
        self.btn_clear_pos.setFixedSize(20, 20)
        self.btn_clear_pos.setStyleSheet(
            "background: transparent; color: #ff9800; border: none; font-size: 12px; font-weight: bold;"
        )
        self.btn_clear_pos.setToolTip("清零编码器位置")
        self.btn_clear_pos.clicked.connect(self.clear_position)
        pos_layout.addWidget(self.btn_clear_pos)
        realtime_layout.addLayout(pos_layout, 1, 1)

        realtime_layout.addWidget(QLabel("指令:"), 1, 2)
        realtime_layout.addWidget(self.lbl_cmd, 1, 3)
        v_splitter.addWidget(realtime_group)

        # 2. 曲线区域
        plot_group = QGroupBox("波形曲线")
        plot_layout = QVBoxLayout(plot_group)
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground("#2b2b2b")
        plot_layout.addWidget(self.plot_widget)
        v_splitter.addWidget(plot_group)

        # 3. 日志区域
        log_group = QGroupBox("系统日志")
        log_layout = QVBoxLayout(log_group)
        self.terminal = QPlainTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setMaximumBlockCount(150)
        log_layout.addWidget(self.terminal)
        v_splitter.addWidget(log_group)

        # 调整初始比例：遥测区域固定高度约 80，曲线区域占大部分，日志区域可调
        v_splitter.setSizes([80, 500, 200])
        v_splitter.setChildrenCollapsible(True)

        right_layout.addWidget(v_splitter)
        main_splitter.addWidget(right_widget)
        main_splitter.setSizes([350, 1050])
        self.setCentralWidget(main_splitter)

    # ------------------ 辅助功能 ------------------
    def scan_ports(self):
        self.cb_port.clear()
        for p in serial.tools.list_ports.comports():
            self.cb_port.addItem(p.device)

    def toggle_connection(self):
        if self.driver.is_connected():
            self.stop_motion()
            self.driver.disconnect()
            self.btn_connect.setText("连接")
            self.on_log("串口已断开")
        else:
            if self.driver.connect(
                self.cb_port.currentText(),
                int(self.cb_baud.currentText()),
                self.sb_id.value(),
            ):
                self.btn_connect.setText("已连接")
                self.on_log("串口连接成功")
            else:
                self.on_log("连接失败")

    def clear_position(self):
        if not self.driver.is_connected():
            self.on_log("未连接串口，无法清零")
            return
        try:
            self.driver.write_reg(0x700A, 1)
            self.on_log("✅ 编码器位置已清零")
            pos = self.driver.read_long(0x002C)
            self.on_log(f"当前实际位置: {pos}")
        except Exception as e:
            self.on_log(f"位置清零失败: {e}")

    def on_log(self, msg):
        self.terminal.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {msg}")
        self.terminal.moveCursor(QTextCursor.End)

    def update_live_params(self):
        if not self.current_worker or not self.current_worker.isRunning():
            return
        if isinstance(self.current_worker, FlappingWorker):
            self.current_worker.params["amplitude"] = self.flap_amplitude.value()
            self.current_worker.params["frequency"] = self.flap_frequency.value()
            self.current_worker.params["wave_type"] = self.flap_wave_type.currentText()
        elif isinstance(self.current_worker, ClosedSpeedWorker):
            self.current_worker.params["target_rpm"] = self.cspeed_target.value()
        elif isinstance(self.current_worker, SpeedWorker):
            self.current_worker.params["target_duty"] = self.speed_duty.value()
        elif isinstance(self.current_worker, TorqueWorker):
            self.current_worker.params["target_current"] = self.torque_target.value()

    # ------------------ Tab 构建 ------------------
    def create_pure_pos_tab(self):
        w = QWidget()
        lay = QGridLayout(w)
        self.pp_encoder = QSpinBox()
        self.pp_encoder.setRange(10, 10000)
        self.pp_encoder.setValue(700)
        self.pp_speed = QSpinBox()
        self.pp_speed.setRange(10, 10000)
        self.pp_speed.setValue(1000)
        self.pp_target = QSpinBox()
        self.pp_target.setRange(-2000000, 2000000)
        self.pp_target.setValue(5000)
        self.btn_pp_start = QPushButton("启动位置监视器")
        self.btn_abs = QPushButton("绝对移动")
        self.btn_rel = QPushButton("相对移动")
        self.btn_home = QPushButton("归零")
        self.btn_pp_stop = QPushButton("紧急停止")
        self.btn_pp_stop.setObjectName("btnStop")
        lay.addWidget(QLabel("编码器线数:"), 0, 0)
        lay.addWidget(self.pp_encoder, 0, 1)
        lay.addWidget(QLabel("移动速度:"), 1, 0)
        lay.addWidget(self.pp_speed, 1, 1)
        lay.addWidget(self.btn_pp_start, 2, 0, 1, 2)
        lay.addWidget(QLabel("目标值:"), 3, 0)
        lay.addWidget(self.pp_target, 3, 1)
        lay.addWidget(self.btn_abs, 4, 0)
        lay.addWidget(self.btn_rel, 4, 1)
        lay.addWidget(self.btn_home, 5, 0)
        lay.addWidget(self.btn_pp_stop, 5, 1)
        lay.setRowStretch(6, 1)
        self.btn_pp_start.clicked.connect(lambda: self.start_motion("pure_pos"))
        self.btn_pp_stop.clicked.connect(self.stop_motion)
        self.btn_abs.clicked.connect(lambda: self.send_pos_cmd(0))
        self.btn_rel.clicked.connect(lambda: self.send_pos_cmd(1))
        self.btn_home.clicked.connect(self.send_home_cmd)
        return w

    def send_pos_cmd(self, typ):
        if not isinstance(self.current_worker, PurePositionWorker):
            self.on_log("请先启动位置监视器")
            return
        self.driver.write_reg(0x0046, self.pp_speed.value())
        self.driver.write_reg(0x0047, typ)
        self.driver.write_long(0x0048, self.pp_target.value())
        self.on_log(
            f"发送{'绝对' if typ == 0 else '相对'}位置指令: {self.pp_target.value()}"
        )

    def send_home_cmd(self):
        if not isinstance(self.current_worker, PurePositionWorker):
            return
        self.driver.write_reg(0x0046, self.pp_speed.value())
        self.driver.write_reg(0x0047, 0)
        self.driver.write_long(0x0048, 0)
        self.on_log("归零指令已发送")

    def create_flapping_tab(self):
        w = QWidget()
        lay = QGridLayout(w)
        self.flap_encoder = QSpinBox()
        self.flap_encoder.setRange(10, 10000)
        self.flap_encoder.setValue(700)
        self.flap_rpm = QSpinBox()
        self.flap_rpm.setRange(100, 20000)
        self.flap_rpm.setValue(6000)
        self.flap_max = QSpinBox()
        self.flap_max.setRange(100, 20000)
        self.flap_max.setValue(18000)
        self.flap_amplitude = QSpinBox()
        self.flap_amplitude.setRange(1, 10000)
        self.flap_amplitude.setValue(128)
        self.flap_frequency = QDoubleSpinBox()
        self.flap_frequency.setRange(0.1, 50)
        self.flap_frequency.setValue(4.0)
        self.flap_wave_type = QComboBox()
        self.flap_wave_type.addItems(["scurve", "sine", "square"])
        self.flap_start = QPushButton("启动轨迹")
        self.flap_stop = QPushButton("紧急停止")
        self.flap_stop.setObjectName("btnStop")
        lay.addWidget(QLabel("编码器线数:"), 0, 0)
        lay.addWidget(self.flap_encoder, 0, 1)
        lay.addWidget(QLabel("最大速度:"), 1, 0)
        lay.addWidget(self.flap_max, 1, 1)
        lay.addWidget(QLabel("振幅:"), 2, 0)
        lay.addWidget(self.flap_amplitude, 2, 1)
        lay.addWidget(QLabel("频率:"), 3, 0)
        lay.addWidget(self.flap_frequency, 3, 1)
        lay.addWidget(QLabel("波形:"), 4, 0)
        lay.addWidget(self.flap_wave_type, 4, 1)
        lay.addWidget(self.flap_start, 5, 0, 1, 2)
        lay.addWidget(self.flap_stop, 6, 0, 1, 2)
        lay.setRowStretch(7, 1)
        self.flap_start.clicked.connect(lambda: self.start_motion("flapping"))
        self.flap_stop.clicked.connect(self.stop_motion)
        self.flap_amplitude.editingFinished.connect(self.update_live_params)
        self.flap_frequency.editingFinished.connect(self.update_live_params)
        self.flap_wave_type.currentIndexChanged.connect(self.update_live_params)
        return w

    def create_closed_speed_tab(self):
        w = QWidget()
        lay = QGridLayout(w)
        self.cspeed_encoder = QSpinBox()
        self.cspeed_encoder.setRange(10, 10000)
        self.cspeed_encoder.setValue(700)
        self.cspeed_target = QDoubleSpinBox()
        self.cspeed_target.setRange(-20000, 20000)
        self.cspeed_target.setValue(100)
        self.cspeed_start = QPushButton("启动速度闭环")
        self.cspeed_stop = QPushButton("紧急停止")
        self.cspeed_stop.setObjectName("btnStop")
        lay.addWidget(QLabel("编码器线数:"), 0, 0)
        lay.addWidget(self.cspeed_encoder, 0, 1)
        lay.addWidget(QLabel("目标转速(RPM):"), 1, 0)
        lay.addWidget(self.cspeed_target, 1, 1)
        lay.addWidget(self.cspeed_start, 2, 0, 1, 2)
        lay.addWidget(self.cspeed_stop, 3, 0, 1, 2)
        lay.setRowStretch(4, 1)
        self.cspeed_start.clicked.connect(lambda: self.start_motion("closed_speed"))
        self.cspeed_stop.clicked.connect(self.stop_motion)
        self.cspeed_target.editingFinished.connect(self.update_live_params)
        return w

    def create_speed_tab(self):
        w = QWidget()
        lay = QGridLayout(w)
        self.speed_rated = QDoubleSpinBox()
        self.speed_rated.setRange(0.1, 5)
        self.speed_rated.setValue(0.5)
        self.speed_work = QDoubleSpinBox()
        self.speed_work.setRange(0.1, 5)
        self.speed_work.setValue(0.5)
        self.speed_duty = QDoubleSpinBox()
        self.speed_duty.setRange(-100, 100)
        self.speed_duty.setValue(20)
        self.speed_start = QPushButton("启动开环速度")
        self.speed_stop = QPushButton("紧急停止")
        self.speed_stop.setObjectName("btnStop")
        lay.addWidget(QLabel("额定电流(A):"), 0, 0)
        lay.addWidget(self.speed_rated, 0, 1)
        lay.addWidget(QLabel("工作电流(A):"), 1, 0)
        lay.addWidget(self.speed_work, 1, 1)
        lay.addWidget(QLabel("占空比(%):"), 2, 0)
        lay.addWidget(self.speed_duty, 2, 1)
        lay.addWidget(self.speed_start, 3, 0, 1, 2)
        lay.addWidget(self.speed_stop, 4, 0, 1, 2)
        lay.setRowStretch(5, 1)
        self.speed_start.clicked.connect(lambda: self.start_motion("speed"))
        self.speed_stop.clicked.connect(self.stop_motion)
        self.speed_duty.editingFinished.connect(self.update_live_params)
        return w

    def create_torque_tab(self):
        w = QWidget()
        lay = QGridLayout(w)
        self.torque_rated = QDoubleSpinBox()
        self.torque_rated.setRange(0.1, 5)
        self.torque_rated.setValue(0.5)
        self.torque_work = QDoubleSpinBox()
        self.torque_work.setRange(0.1, 5)
        self.torque_work.setValue(0.5)
        self.torque_target = QDoubleSpinBox()
        self.torque_target.setRange(-5, 5)
        self.torque_target.setValue(0.1)
        self.torque_start = QPushButton("启动力矩")
        self.torque_stop = QPushButton("紧急停止")
        self.torque_stop.setObjectName("btnStop")
        lay.addWidget(QLabel("额定电流(A):"), 0, 0)
        lay.addWidget(self.torque_rated, 0, 1)
        lay.addWidget(QLabel("工作电流(A):"), 1, 0)
        lay.addWidget(self.torque_work, 1, 1)
        lay.addWidget(QLabel("目标力矩(A):"), 2, 0)
        lay.addWidget(self.torque_target, 2, 1)
        lay.addWidget(self.torque_start, 3, 0, 1, 2)
        lay.addWidget(self.torque_stop, 4, 0, 1, 2)
        lay.setRowStretch(5, 1)
        self.torque_start.clicked.connect(lambda: self.start_motion("torque"))
        self.torque_stop.clicked.connect(self.stop_motion)
        self.torque_target.editingFinished.connect(self.update_live_params)
        return w

    # ------------------ 曲线及绘图 ------------------
    def setup_plot(self):
        self.plot_widget.clear()
        self.p1 = self.plot_widget.addPlot(row=0, col=0, title="位置反馈 (脉冲)")
        self.p1.showGrid(True, True, alpha=0.3)
        self.p1.addLegend()
        self.cmd_curve = self.p1.plot(
            pen=pg.mkPen(color="#e91e63", width=2, style=Qt.DashLine), name="指令"
        )
        self.pos_curve = self.p1.plot(
            pen=pg.mkPen(color="#2196F3", width=2), name="实际位置"
        )
        self.p1.scene().sigMouseClicked.connect(
            lambda event: event.double() and self.p1.autoRange()
        )

        self.p2 = self.plot_widget.addPlot(row=1, col=0, title="实时电流 (A)")
        self.p2.showGrid(True, True, alpha=0.3)
        self.cur_curve = self.p2.plot(pen=pg.mkPen(color="#ffeb3b", width=2))
        self.p2.scene().sigMouseClicked.connect(
            lambda event: event.double() and self.p2.autoRange()
        )

        self.p3 = self.plot_widget.addPlot(row=2, col=0, title="实时转速 (RPM)")
        self.p3.showGrid(True, True, alpha=0.3)
        self.spd_curve = self.p3.plot(pen=pg.mkPen(color="#4caf50", width=2))
        self.p3.scene().sigMouseClicked.connect(
            lambda event: event.double() and self.p3.autoRange()
        )

    # ------------------ 运动控制核心 ------------------
    def start_motion(self, mode):
        if not self.driver.is_connected():
            self.on_log("请先连接串口")
            return
        if self.current_worker and self.current_worker.isRunning():
            self.stop_motion()
            time.sleep(0.1)
        params = {}
        if mode == "pure_pos":
            params = {
                "encoder_lines": self.pp_encoder.value(),
                "move_speed": self.pp_speed.value(),
            }
            worker = PurePositionWorker(self.driver, params)
        elif mode == "flapping":
            params = {
                "encoder_lines": self.flap_encoder.value(),
                "rated_rpm": self.flap_rpm.value(),
                "max_speed": self.flap_max.value(),
                "amplitude": self.flap_amplitude.value(),
                "frequency": self.flap_frequency.value(),
                "wave_type": self.flap_wave_type.currentText(),
            }
            worker = FlappingWorker(self.driver, params)
        elif mode == "closed_speed":
            params = {
                "encoder_lines": self.cspeed_encoder.value(),
                "target_rpm": self.cspeed_target.value(),
            }
            worker = ClosedSpeedWorker(self.driver, params)
        elif mode == "speed":
            params = {
                "rated_current": self.speed_rated.value(),
                "work_current": self.speed_work.value(),
                "target_duty": self.speed_duty.value(),
            }
            worker = SpeedWorker(self.driver, params)
        elif mode == "torque":
            params = {
                "rated_current": self.torque_rated.value(),
                "work_current": self.torque_work.value(),
                "target_current": self.torque_target.value(),
            }
            worker = TorqueWorker(self.driver, params)
        else:
            return
        worker.data_signal.connect(self.on_data)
        worker.log_signal.connect(self.on_log)
        worker.finished_signal.connect(self.on_worker_finished)
        self.current_worker = worker
        worker.start()

    def stop_motion(self):
        if self.current_worker:
            self.current_worker.stop()
            self.current_worker.wait(500)
        self.driver.emergency_stop()
        self.on_log("已执行紧急停止")
        self.clear_data()

    def on_data(self, data):
        if data == ("CLEAR",):
            for k in self.data_buffer:
                self.data_buffer[k].clear()
            return
        self.frame_counter += 1
        if self.frame_counter % 2 != 0:
            _, p, c, s, cmd = data
            self.lbl_current.setText(f"{c:.2f} A")
            self.lbl_speed.setText(f"{s:.0f} RPM")
            self.lbl_pos.setText(f"{p:.0f}")
            self.lbl_cmd.setText(
                f"{cmd:.2f}" if isinstance(cmd, float) else f"{cmd:.0f}"
            )
            return
        t, p, c, s, cmd = data
        self.data_buffer["time"].append(t)
        self.data_buffer["pos"].append(p)
        self.data_buffer["current"].append(c)
        self.data_buffer["speed"].append(s)
        self.data_buffer["cmd"].append(cmd)
        self.lbl_current.setText(f"{c:.2f} A")
        self.lbl_speed.setText(f"{s:.0f} RPM")
        self.lbl_pos.setText(f"{p:.0f}")
        self.lbl_cmd.setText(f"{cmd:.2f}" if isinstance(cmd, float) else f"{cmd:.0f}")

    def update_plots(self):
        if len(self.data_buffer["time"]) < 2:
            return
        times = list(self.data_buffer["time"])
        self.pos_curve.setData(times, list(self.data_buffer["pos"]))
        self.cmd_curve.setData(times, list(self.data_buffer["cmd"]))
        self.cur_curve.setData(times, list(self.data_buffer["current"]))
        self.spd_curve.setData(times, list(self.data_buffer["speed"]))
        x_max = times[-1]
        x_min = max(0, x_max - 5)
        self.p1.setXRange(x_min, x_max)
        self.p2.setXRange(x_min, x_max)
        self.p3.setXRange(x_min, x_max)

    def on_worker_finished(self):
        self.current_worker = None
        self.on_log("工作线程已结束")

    def clear_data(self):
        for k in self.data_buffer:
            self.data_buffer[k].clear()
        self.pos_curve.clear()
        self.cmd_curve.clear()
        self.cur_curve.clear()
        self.spd_curve.clear()
        self.p1.setXRange(0, 5)
        self.p2.setXRange(0, 5)
        self.p3.setXRange(0, 5)
        self.on_log("数据已清除，曲线重置")

    def reset_view(self):
        if len(self.data_buffer["time"]) == 0:
            self.on_log("无数据，无法重置视图")
            return
        times = list(self.data_buffer["time"])
        x_max = times[-1]
        x_min = max(0, x_max - 5)
        self.p1.setXRange(x_min, x_max)
        self.p2.setXRange(x_min, x_max)
        self.p3.setXRange(x_min, x_max)
        self.p1.autoRange()
        self.p2.autoRange()
        self.p3.autoRange()
        self.on_log("视图已重置")

    def closeEvent(self, e):
        self.stop_motion()
        self.driver.disconnect()
        e.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(FONT)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
