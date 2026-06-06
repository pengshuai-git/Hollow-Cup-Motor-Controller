import math
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
instrument.serial.baudrate = 115200
instrument.serial.parity = serial.PARITY_EVEN
instrument.serial.stopbits = 1
instrument.serial.timeout = 0.1
instrument.mode = minimalmodbus.MODE_RTU

print("🦋 蝴蝶扑翼机 - 频率与幅值控制模式启动...\n")
print("按 【Ctrl + C】 随时停止\n")

# ================= 扑翼动作核心参数 =================
AMPLITUDE = 128  # 幅值 (脉冲数)：代表从中心0点到极值点的距离
FREQUENCY = 1.0  # 频率 (Hz)：每秒拍打的完整次数

# 切换波形类型 (三选一)
# WAVE_TYPE = "square"   # 方波：瞬间爆发，到点折返 (粗暴但响应最快)
# WAVE_TYPE = "sine"     # 正弦波：理论最平滑，但在 9600 波特率下容易卡顿
WAVE_TYPE = "sine"  # 多段S曲线：兼顾方波的低延迟和正弦波的柔和末端 🌟推荐🌟

# --- S曲线专属调参 ---
SCURVE_STEPS = 3  # S曲线的切分段数。
# 值越大越平滑，但受限于波特率；值越小越接近方波。建议设在 4~6 之间。

MAX_SPEED = 18000  # 驱动器的底盘追踪速度
# ====================================================

try:
    # 提前放开速度限制，让位置指令完全接管运动
    instrument.write_register(REG_POS_SPEED, MAX_SPEED, functioncode=6)
    print(f"当前参数: 幅值={AMPLITUDE}, 频率={FREQUENCY}Hz, 波形={WAVE_TYPE}")

    start_time = time.time()

    while True:
        if WAVE_TYPE == "square":
            # --- 【方波控制逻辑】 ---
            half_period = 1.0 / (2.0 * FREQUENCY)

            instrument.write_long(REG_TARGET_POS, AMPLITUDE, signed=True)
            time.sleep(half_period)

            instrument.write_long(REG_TARGET_POS, -AMPLITUDE, signed=True)
            time.sleep(half_period)

        elif WAVE_TYPE == "scurve":
            # --- 【多段 S 轨迹优化逻辑】 ---
            half_period = 1.0 / (2.0 * FREQUENCY)
            step_time = half_period / SCURVE_STEPS

            # 1. 翅膀下拍 (从 -AMPLITUDE 运动到 AMPLITUDE)
            for i in range(1, SCURVE_STEPS + 1):
                # 计算当前进度 t (0 到 1)
                t = i / SCURVE_STEPS
                # Smoothstep 公式：产生两端平缓、中间陡峭的曲线
                smooth_factor = 3 * (t**2) - 2 * (t**3)

                target_pos = int(-AMPLITUDE + (2 * AMPLITUDE) * smooth_factor)
                try:
                    instrument.write_long(REG_TARGET_POS, target_pos, signed=True)
                except Exception:
                    pass
                time.sleep(step_time)

            # 2. 翅膀上扬 (从 AMPLITUDE 运动到 -AMPLITUDE)
            for i in range(1, SCURVE_STEPS + 1):
                t = i / SCURVE_STEPS
                smooth_factor = 3 * (t**2) - 2 * (t**3)

                target_pos = int(AMPLITUDE - (2 * AMPLITUDE) * smooth_factor)
                try:
                    instrument.write_long(REG_TARGET_POS, target_pos, signed=True)
                except Exception:
                    pass
                time.sleep(step_time)

        elif WAVE_TYPE == "sine":
            # --- 【正弦波控制逻辑】 ---
            current_time = time.time() - start_time
            target_pos = int(
                AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * current_time)
            )
            try:
                instrument.write_long(REG_TARGET_POS, target_pos, signed=True)
            except Exception:
                pass
            time.sleep(0.01)

        else:
            print("未知的波形配置！请检查 WAVE_TYPE 拼写。")
            break

except KeyboardInterrupt:
    print("\n\n🚨 停止扑翼，翅膀缓慢归位到 0 点...")
    try:
        instrument.write_register(REG_POS_SPEED, 60, functioncode=6)
        instrument.write_long(REG_TARGET_POS, 0, signed=True)
    except:
        pass
    time.sleep(1)
    print("归零完成，程序退出。")
