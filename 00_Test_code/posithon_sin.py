import math
import minimalmodbus
import serial
import time

PORT = "COM12"
SLAVE_ADDRESS = 1
instrument = minimalmodbus.Instrument(PORT, SLAVE_ADDRESS)
instrument.serial.baudrate = 115200
instrument.serial.parity = serial.PARITY_EVEN
instrument.serial.stopbits = 1
instrument.serial.timeout = 0.1
instrument.mode = minimalmodbus.MODE_RTU

# --- 位置控制寄存器 ---
REG_POS_SPEED = 0x0046
REG_TARGET_POS = 0x0048

# --- 模式配置寄存器 ---
REG_CTRL_MODE = 0x0080
REG_FEEDBACK_TYPE = 0x0068
REG_PULSES_PER_REV = 0x0069
REG_MAX_RPM = 0x006C
REG_SERVO_MODE = 0x00C1
REG_STALL_TIME = 0x0081

# --- 参数 ---
ENCODER_LINES = 700  # 请根据实际编码器修改
RATED_RPM = 6000
MAX_SPEED = 4800  # 额定转速的80%
AMPLITUDE = 128
FREQUENCY = 5
WAVE_TYPE = "sine"  # "square" / "sine" / "scurve"


try:
    # ========== 1. 配置驱动器为位置闭环模式 ==========
    # 0x700A 寄存器：写 1 清零位置计数（参见手册 6.3.16 节）
    instrument.write_register(0x700A, 1, functioncode=6)
    time.sleep(0.1)
    print("✅ 编码器位置计数值已清零，当前机械位置设为 0 点")

    # 设置编码器反馈
    instrument.write_register(REG_FEEDBACK_TYPE, 2, functioncode=6)
    instrument.write_register(REG_PULSES_PER_REV, ENCODER_LINES, functioncode=6)
    # 额定转速（32位）
    instrument.write_long(REG_MAX_RPM, RATED_RPM, signed=False)
    # 伺服控制方式：非固定区间（0x02）
    instrument.write_register(REG_SERVO_MODE, 2, functioncode=6)
    # 通讯控制模式：外接测速闭环（低字节=3）
    cur = instrument.read_register(REG_CTRL_MODE, functioncode=3)
    new_mode = (cur & 0xFF00) | 0x03
    instrument.write_register(REG_CTRL_MODE, new_mode, functioncode=6)
    # 禁用堵转保护
    instrument.write_register(REG_STALL_TIME, 0, functioncode=6)
    print("✅ 驱动器已切换至编码器位置闭环模式")

    # 设置位置闭环速度上限
    instrument.write_register(REG_POS_SPEED, MAX_SPEED, functioncode=6)
    print(f"位置闭环速度上限: {MAX_SPEED} RPM")

    # ========== 2. 扑翼运动循环 ==========
    print(f"幅值={AMPLITUDE}, 频率={FREQUENCY}Hz, 波形={WAVE_TYPE}")
    start_time = time.time()

    while True:
        if WAVE_TYPE == "square":
            half_period = 1.0 / (2.0 * FREQUENCY)
            instrument.write_long(REG_TARGET_POS, AMPLITUDE, signed=True)
            time.sleep(half_period)
            instrument.write_long(REG_TARGET_POS, -AMPLITUDE, signed=True)
            time.sleep(half_period)

        elif WAVE_TYPE == "sine":
            current_time = time.time() - start_time
            target_pos = int(
                AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * current_time)
            )
            instrument.write_long(REG_TARGET_POS, target_pos, signed=True)
            time.sleep(0.01)  # 100Hz 更新率

        elif WAVE_TYPE == "scurve":
            # ... 您的 S 曲线代码 ...
            pass
        else:
            break

except KeyboardInterrupt:
    print("\n归位中...")
    instrument.write_register(REG_POS_SPEED, 200, functioncode=6)
    instrument.write_long(REG_TARGET_POS, 0, signed=True)
    time.sleep(1)
    print("退出")
except Exception as e:
    print(f"错误: {e}")
