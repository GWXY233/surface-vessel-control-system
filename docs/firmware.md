# 固件与诊断

## 正常程序

打开 `firmware/Boat_Nano_595_Display600/Boat_Nano_595_Display600.ino`，选择经典 Arduino Nano、ATmega328P、16 MHz。新旧 bootloader 按实物上传成功的选项选择。依赖 Servo 库，原验证使用 Servo 1.3.0、Arduino AVR core 1.8.8。

```sh
arduino-cli core install arduino:avr@1.8.8
arduino-cli lib install Servo@1.3.0
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old firmware/Boat_Nano_595_Display600
```

正常程序约使用 11500 字节 Flash 和 556 字节静态 SRAM。库版本或编译器变化可能改变占用。

| 参数 | 默认值 |
| --- | --- |
| 目标转速 | 595 r/min |
| 编码器换算 | 1019 个 A 相上升沿 / 10 圈输出轴 |
| 接收机稳定等待 | 500 ms |
| 目标爬升 | 300 r/min/s，约 2 秒到目标 |
| 舵机中位与端点 | 1500 μs，600～2400 μs |
| 数字显示上限 | 600 r/min |
| 超速停机阈值 | 原始反馈 ≥610 r/min |
| 稳态带宽检查 | 570～630 r/min，连续约 3 秒超出会停机 |
| 电池低压截止 | 默认关闭，启用前校准 A0 与参考电压 |

595 是闭环目标。电机能力、传动阻力与电源压降会影响是否能达到该值，骤然卸载也可能使转速短时上冲。

## 屏幕报码

| 显示 | 含义 |
| --- | --- |
| E001 | 编码器参数缺失、启动时未确认运动，或运行时反馈丢失 |
| E002 | 原始反馈达到 610 r/min 超速保护阈值 |
| E003 | 到达目标参考值后，持续无法保持 570～630 r/min 区间 |
| E004 | 已启动后转向 CH1 信号丢失 |
| 0000 | 当前转速为零或仍在等待启动 |
| ---- | 暂时无法计算可靠转速、校准缺失或相应显示条件不满足 |

电机未动、接线断开或机械卡死也可能导致 E001。驱动故障和可选的电池截止没有单独 E005 等报码。初始缺 CH1 时等待，运行中的失联会锁停。

关闭总电源，等屏幕熄灭和电机停稳，处理原因后再重启。

**普通数字显示采用 `min(measuredRPM, 600)`。PI、串口与保护仍读取未截断的 `measuredRPM`，E 类故障显示优先。校准模式显示原始计数，不按 600 封顶。**

## 临时 PWM 诊断程序

`firmware/Boat_Nano_Drive_Diagnostic/Boat_Nano_Drive_Diagnostic.ino` 基于 V5.3，目标仍为 600 r/min。运行时每 2 秒交替显示实际转速和 `Pxxx` PWM 指令，用于检查输出余量。`P255` 表示软件指令已到 100%，实际电机电压还取决于功率路径。

该诊断程序与 V5.4 正常程序分目录保存。使用时先卸桨，测试结束后按需要恢复正常程序。
