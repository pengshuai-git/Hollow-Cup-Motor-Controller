import minimalmodbus
import serial
import time
import sys

# ================= 配置区 =================
PORT_NAME = 'COM12'        # 你刚才测试成功的端口
SLAVE_ADDRESS = 1          # 从站地址

# --- 循环参数配置 ---
LOOP_COUNT = 5             # 循环次数 (例如：5次)
DUTY_CYCLE = 20.0          # 目标占空比百分比 (例如：20%)
RUN_TIME = 0.5             # 每次正转/反转的持续时间 (秒)，调小它就能转得更快
STOP_TIME = 0.2            # 换向时的刹车缓冲时间 (秒)，⚠️ 建议不要低于 0.3 秒！

REG_TARGET_DUTY = 0x0040   # 目标占空比寄存器

# ================= 初始化 =================
try:
    instrument = minimalmodbus.Instrument(PORT_NAME, SLAVE_ADDRESS)
    instrument.serial.baudrate = 9600
    instrument.serial.bytesize = 8
    instrument.serial.parity   = serial.PARITY_EVEN
    instrument.serial.stopbits = 1
    instrument.serial.timeout  = 1.0
    instrument.mode = minimalmodbus.MODE_RTU   
    print(f"成功打开串口 {PORT_NAME}，准备执行快速循环测试...")
except Exception as e:
    print(f"串口打开失败: {e}")
    sys.exit()

def set_motor_duty(duty_cycle_percent):
    val = int(duty_cycle_percent * 10)
    try:
        instrument.write_register(REG_TARGET_DUTY, val, functioncode=6, signed=True)
    except Exception as e:
        print(f"通讯异常: {e}")

# ================= 快速循环逻辑 =================
if __name__ == '__main__':
    print(f"\n=== 开始执行快速循环正反转 (共 {LOOP_COUNT} 次) ===")
    print("⚠️ 提示：在运行过程中，随时按 【Ctrl + C】 可以紧急停止电机！\n")
    
    try:
        for i in range(1, LOOP_COUNT + 1):
            print(f"--- 第 {i}/{LOOP_COUNT} 次循环 ---")
            
            # 1. 正转
            print(f"  -> 正转 {DUTY_CYCLE}%")
            set_motor_duty(DUTY_CYCLE)
            time.sleep(RUN_TIME)
            
            # 2. 停止缓冲 (极其重要)
            print("  -> 刹车缓冲")
            set_motor_duty(0)
            time.sleep(STOP_TIME)
            
            # 3. 反转
            print(f"  -> 反转 -{DUTY_CYCLE}%")
            set_motor_duty(-DUTY_CYCLE)
            time.sleep(RUN_TIME)
            
            # 4. 停止缓冲 (极其重要)
            print("  -> 刹车缓冲")
            set_motor_duty(0)
            time.sleep(STOP_TIME)
            
        print("\n=== 所有循环执行完毕，电机已停止 ===")
        set_motor_duty(0) # 确保最终停止
        
    except KeyboardInterrupt:
        # 捕获 Ctrl+C 紧急停止信号
        print("\n\n🚨 收到紧急停止信号 (Ctrl+C)！正在强制刹车...")
        set_motor_duty(0)
        print("电机已安全停止，程序退出。")
        sys.exit()