import minimalmodbus
import serial
import time

# --- 通讯配置 ---
PORT = "COM12"
SLAVE_ADDRESS = 1
instrument = minimalmodbus.Instrument(PORT, SLAVE_ADDRESS)
instrument.serial.baudrate = 115200
instrument.serial.parity = serial.PARITY_EVEN
instrument.serial.stopbits = 1
instrument.serial.timeout = 0.1
instrument.mode = minimalmodbus.MODE_RTU

# --- 力矩控制参数 ---
TARGET_CURRENT_A = 0.01  # 目标电流 (A)
RATED_CURRENT_A = 0.5  # 电机额定电流 (A)
MAX_WORK_CURRENT_A = 0.5  # 最大工作电流 (A)

# 寄存器地址
REG_CTRL_MODE = 0x0080  # 控制模式（低字节=1力矩）
REG_TORQUE = 0x0040  # 力矩设定
REG_RATED_CUR = 0x0086  # 额定电流
REG_WORK_CUR = 0x0087  # 工作电流
REG_STALL_TIME = 0x0081  # 堵转停止时间（0=禁用）
REG_REAL_CURRENT = 0x0011  # 实时电流（×0.01A）
REG_FAULT_CODE = 0x0017  # 故障码


# --- 初始化驱动器为力矩模式 ---
def init_torque_mode():
    # 1. 配置电流限制
    instrument.write_register(REG_RATED_CUR, int(RATED_CURRENT_A * 100), functioncode=6)
    instrument.write_register(
        REG_WORK_CUR, int(MAX_WORK_CURRENT_A * 100), functioncode=6
    )
    # 2. 禁用堵转保护（避免因堵转自动制动导致报错）
    instrument.write_register(REG_STALL_TIME, 0, functioncode=6)
    # 3. 设置通讯控制模式为力矩控制（低字节=1）
    current_mode = instrument.read_register(REG_CTRL_MODE, functioncode=3)
    new_mode = (current_mode & 0xFF00) | 0x01
    instrument.write_register(REG_CTRL_MODE, new_mode, functioncode=6)
    print("✅ 驱动器已初始化：力矩模式 + 堵转保护已禁用")


# --- 设定目标力矩 ---
def set_torque(current_A):
    value = int(current_A * 100)
    value = max(-500, min(500, value))
    instrument.write_register(REG_TORQUE, value, functioncode=6)
    print(f"⚙️ 设定力矩: {current_A:.2f}A -> 寄存器 {value}")


# --- 读取实时电流 ---
def read_actual_current():
    raw = instrument.read_register(REG_REAL_CURRENT, functioncode=3, signed=False)
    actual_A = raw * 0.01
    return actual_A


# --- 读取故障码 ---
def read_fault():
    return instrument.read_register(REG_FAULT_CODE, functioncode=3)


# --- 清除故障（通过重新初始化力矩模式并停止力矩）---
def clear_fault():
    print("⚠️ 尝试清除故障...")
    instrument.write_register(REG_TORQUE, 0, functioncode=6)
    time.sleep(0.2)
    init_torque_mode()
    time.sleep(0.1)


# --- 停止力矩输出 ---
def stop_torque():
    instrument.write_register(REG_TORQUE, 0, functioncode=6)
    print("🛑 力矩归零")


# ========== 主程序 ==========
try:
    init_torque_mode()
    time.sleep(0.1)

    # 检查初始故障
    fault = read_fault()
    if fault != 0:
        print(f"⚠️ 初始故障码: {fault}，尝试清除...")
        clear_fault()

    # 设定目标力矩（只需写一次，驱动器会持续维持）
    set_torque(TARGET_CURRENT_A)
    time.sleep(0.2)

    print("\n🦋 力矩对比模式运行中（每0.5秒读取一次实际电流）")
    print("按 Ctrl+C 停止\n")

    while True:
        # 读取实际电流
        actual = read_actual_current()
        print(
            f"📊 设定: {TARGET_CURRENT_A:5.2f}A  |  实时: {actual:5.2f}A  |  偏差: {actual - TARGET_CURRENT_A:+.2f}A"
        )

        # 检查故障
        fault = read_fault()
        if fault != 0:
            print(f"❌ 检测到故障码 {fault} (2=堵转,6=过流,7=过热,8=过压,9=欠压)")
            clear_fault()
            # 重新设定力矩
            set_torque(TARGET_CURRENT_A)

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\n⚠️ 用户中断，停止力矩并退出...")
    stop_torque()
    time.sleep(0.5)
    print("程序结束。")
