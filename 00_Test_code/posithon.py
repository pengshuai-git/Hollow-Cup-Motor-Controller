import minimalmodbus
import serial
import time

# --- 1. 最基础通讯配置 ---
PORT = "COM12"
SLAVE_ADDRESS = 1

REG_POS_SPEED = 0x0046  # 设定位置闭环速度 (单位: RPM)
REG_TARGET_POS = 0x0048  # 目标转动位置 (32位)

# --- 2. 初始化串口 ---
instrument = minimalmodbus.Instrument(PORT, SLAVE_ADDRESS)
instrument.serial.baudrate = 9600
instrument.serial.parity = serial.PARITY_EVEN
instrument.serial.stopbits = 1
instrument.serial.timeout = 0.2
instrument.mode = minimalmodbus.MODE_RTU

print("🦋 蝴蝶扑翼机 - 丝滑双速循环测试启动...\n")
print("按 【Ctrl + C】 随时停止")

# ================= 扑翼动作参数配置 =================
# 你可以自由修改这里的数值，寻找最佳的空气动力学姿态

# 【动作 A：翅膀下压 (高爆发产生升力)】
POS_DOWN = 128  # 下压的目标脉冲位置
SPEED_DOWN = 18000  # 下压速度：快 (RPM)
TIME_DOWN = 0.05  # 下压行程耗时 (秒) ⚠️ 调参关键点

# 【动作 B：翅膀上扬 (慢速复位减少阻力)】
POS_UP = -128  # 上扬的目标脉冲位置
SPEED_UP = 18000  # 上扬速度：慢 (RPM)
TIME_UP = 0.05  # 上扬行程耗时 (秒) ⚠️ 调参关键点
# ====================================================

try:
    # 先给一个初始速度，确保安全
    instrument.write_register(REG_POS_SPEED, 60, functioncode=6)

    while True:
        # ----------------- 翅膀下拍 -----------------
        # 1. 瞬间改写为“下拍速度”
        instrument.write_register(REG_POS_SPEED, SPEED_DOWN, functioncode=6)
        # 2. 下发下拍极限位置
        instrument.write_long(REG_TARGET_POS, POS_DOWN, signed=True)
        # 3. 等待行程结束。为了“无停顿”，这个时间要卡得刚好！
        time.sleep(TIME_DOWN)

        # ----------------- 翅膀上扬 -----------------
        # 1. 瞬间改写为“上扬速度”
        instrument.write_register(REG_POS_SPEED, SPEED_UP, functioncode=6)
        # 2. 下发上扬极限位置
        instrument.write_long(REG_TARGET_POS, POS_UP, signed=True)
        # 3. 等待行程结束
        time.sleep(TIME_UP)

except KeyboardInterrupt:
    print("\n\n🚨 停止扑翼，翅膀缓慢归位到 0 点...")
    instrument.write_register(REG_POS_SPEED, 30, functioncode=6)  # 用极慢的安全速度归零
    instrument.write_long(REG_TARGET_POS, 0, signed=True)
    time.sleep(2)
    print("归零完成，程序退出。")
