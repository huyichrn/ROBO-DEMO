# STM32F407 CAN 环境搭建与联调记录

记录日期为 2026 年 11 月 20 日。已在 Windows 上打通电脑、USB-CAN 分析仪和正点原子探索者 V3 开发板之间的 CAN 链路，并通过 ST-LINK 烧录了带 LCD 收发监视页的固件。Keil 用于编译，OpenOCD 用于连接、烧录、校验和复位。

## 硬件连接

| 项目 | 本次联调配置 |
| --- | --- |
| 开发板 | 正点原子探索者 V3，实物确认是 STM32F407。工程目标为 `STM32F407ZGTx`，OpenOCD 读到 1024 KiB Flash。 |
| 板端 CAN | CAN1，PA11 为 RX，PA12 为 TX。CAN/USB 选择排针用两个跳线帽连接 PA11→CRX、PA12→CTX。 |
| 分析仪 | USB-CAN 分析仪通道 1（CH1）。脚本中的索引是 `-Channel 0`。 |
| 总线接线和参数 | CAN_H 对 CAN_H，CAN_L 对 CAN_L，GND 共地。两端按需接入 120 Ω 终端。两侧使用 500 kbit/s、正常模式。 |
| 烧录和调试 | ST-LINK/V2 经 SWD 连接。OpenOCD 识别到 `V2J35S7`、SWD DPIDR `0x2BA01477`、芯片识别寄存器读值 `0x100F6413`，目标电压约 3.23 V。 |

## Windows 驱动和工具

- **USB-CAN 分析仪。** Windows 已识别设备 `VID_04D8&PID_0053`，使用已有的 WinUSB 驱动。复用了厂商 USB CAN TOOL 及其 32 位 `ControlCAN.dll`。新增 [Test-CanReceive.ps1](../Test-CanReceive.ps1) 调用厂商库，负责打开通道、单次发送、检查应答和读取分析仪错误计数。USB CAN TOOL 已打开并占用设备时，先在工具中断开设备再运行脚本。
- **ST-LINK。** 起初 Windows 识别到 `VID_0483&PID_3748`，设备状态为代码 28。以管理员权限通过 `pnputil /add-driver ... /install` 安装了 Keil 随附的 `ARM\STLink\USBDriver\stlink_dbg_winusb.inf`。设备状态随后变为 `Started`，驱动为 WinUSB。
- **SWD 工具。** 工程的调试器选项已改为 ST-LINK，但 Keil 烧录仍报 `No ST-LINK detected`。随后使用 xPack OpenOCD，通过 `interface/stlink.cfg` 和 `target/stm32f4x.cfg` 连接目标。连接成功后执行 `program {固件路径} verify reset exit`，日志确认 `Verified OK` 并复位板子。

## Keil 工程与构建

1. 打开 [atk_f407.uvprojx](<../实验27 CAN通信实验/Projects/MDK-ARM/atk_f407.uvprojx>)，目标为 `CAN`。原配置选用 ARM Compiler 5，本机 Keil MDK 5.40 Lite 装有 ARM Compiler 6.22，因此把工程切换到 ARM Compiler 6.22。
2. 下载工程指定的 Keil 设备包 `Keil.STM32F4xx_DFP.2.15.0`。PackUnzip 未注册该设备，随后将官方包解压至 Keil 配置的设备包目录 `%LOCALAPPDATA%\Arm\Packs\Keil\STM32F4xx_DFP\2.15.0`。
3. 初次链接超过了 Keil Lite 的容量限制。启用 MicroLIB 和 `-Oz`，先构建无 LCD 的版本联调总线。加入 LCD 监视页后，测试构建通过 `LCD_FONT_16_ONLY` 只链接 16 像素字库，通过 `CAN_TEST_NO_HEARTBEAT` 关闭周期心跳。最终 LCD 构建链接成功，产出约 29 KiB 的 Flash 内容。
4. 构建产物位于工程的 `Output` 目录。[atk_f407.hex](<../实验27 CAN通信实验/Output/atk_f407.hex>) 用于烧录，`atk_f407.axf` 和 `atk_f407.map` 用于 SWD 调试与符号定位。

## 联调顺序与验证结果

1. **探测总线。** 分析仪两个通道最初都能提交测试帧，但未收到板端应答。烧录初版固件后，通过 SWD 读到 CAN1 接收计数为 0，控制器已进入 bus-off。随后停止测试版的周期心跳，让板子上电后等待电脑请求。
2. **验证板内路径。** 临时将 CAN1 切到静默内部回环，通过 SWD 注入 SDO 请求。接收中断和 SDO 处理通过，测试后复位回普通模式。
3. **验证外部总线。** 用户确认 CAN 跳线位置后，在 CH1 用 `0x601` 读取对象 `0x2001:00`，收到 `0x581` 应答。该版连续三次复测通过，分析仪收发错误计数均为 0。
4. **验证 LCD 固件。** 加入监视页后重新编译、烧录、校验，并再次通过同一读对象测试。SWD 确认 LCD 控制器已初始化，且固件记录的最近 RX 和 TX 与分析仪捕获的请求、应答一致。LCD 实际显示由现场在屏幕上查看。

## 电脑端复测

从工作区根目录用 32 位 Windows PowerShell 运行下面的命令。脚本默认使用 CH1 和 500 kbit/s，收到匹配的 `0x581` SDO 应答时输出 `PASS`。

```powershell
& "$env:WINDIR\SysWOW64\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -ExecutionPolicy Bypass -File .\Test-CanReceive.ps1
```

手动发送参数、应答条件及 LCD 计数含义见 [CAN 功能与测试说明](02-CAN功能与测试说明.md)。
