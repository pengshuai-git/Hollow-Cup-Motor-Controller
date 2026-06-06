# 空心杯闭环控制

这是一个基于 PyQt5 的 GUI 控制程序，用于测试和运行运动控制逻辑。项目包含两部分：测试代码和界面控制代码。

## 目录结构

- `00_Test_code/` - 运动与控制算法测试脚本
- `01_Contol_UI/` - GUI 程序文件，当前主界面文件为 `GUI_qt_v0.2.py`
- `.asset/UI.jpg` - 项目 UI 截图或界面说明

## 依赖库

- PyQt5
- pyqtgraph
- minimalmodbus
- pyserial

如果需要，可使用项目虚拟环境中的 `pip` 安装：

```cmd
.\.venv\Scripts\pip.exe install PyQt5 pyqtgraph minimalmodbus pyserial
```

## 运行方式

1. 打开 VS Code 并选择解释器：
   - 请选择项目内的虚拟环境 `./.venv/Scripts/python.exe`
2. 运行主程序：
   - 直接使用 VS Code 运行 `01_Contol_UI/GUI_qt_v0.2.py`
   - 或者在终端中运行：
     ```cmd
     .\.venv\Scripts\python.exe 01_Contol_UI\GUI_qt_v0.2.py
     ```



## 说明

- `GUI_qt_v0.2.py` 是当前可运行的 GUI 界面文件。
- 如果 VS Code 运行时出现环境问题，请先确认右下角 Python 解释器已切换到 `.venv`。
- `.asset/UI.jpg` 可用于参考界面布局。
