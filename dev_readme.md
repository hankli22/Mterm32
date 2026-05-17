# 开发经验教训

> **规则：**
> - 仅记录影响设备运行、编译通过性、硬件兼容性等"大事"。
> - 除非对应的临时代码被移除，否则不要删除已有条目。
> - 每条记录附带时间戳。

---

## 2026-05-17 — ESP32-C6 Flash 芯片误判（4MB→8MB）导致无法启动

**现象：** 迁移至 PlatformIO 后首次烧录，设备完全无反应（屏幕不亮、GNSS 模块未上电）。串口输出：
```
I (33) boot.esp32c6: SPI Flash Size : 8MB
E (371) spi_flash: Detected size(4096k) smaller than the size in the binary image header(8192k). Probe failed.
```

**根因：** ROM bootloader 通过 JEDEC ID 查表将实际 4MB 的 flash 芯片误判为 8MB，`CONFIG_ESPTOOLPY_FLASHSIZE` 与 `HEADER_FLASHSIZE_UPDATE` 设置均被 ROM 级检测覆盖。ESP-IDF 启动流程中 `spi_flash_init()` 将 ROM 写入的 8MB 与实际检测的 4MB 比对，不匹配即 `assert` 崩溃。

**修复：** linker `--wrap` 劫持 `bootloader_common_get_chip_size()`，强制返回 4MB。涉及文件：
- `src/main.cpp` → `__wrap_bootloader_common_get_chip_size()`（临时）
- `platformio.ini` → `-Wl,--wrap=bootloader_common_get_chip_size`（临时）

**注意：** 若更换使用标准 8MB flash 的 ESP32-C6 开发板，需移除此 workaround。

---

## 2026-05-17 — 主任务栈过小导致疑似栈溢出

**现象：** `app_main()` 启动即崩溃，`Power::init()` 未被调用。

**根因：** `CONFIG_ESP_MAIN_TASK_STACK_SIZE = 3584`，app_main 调用链涉及 NVS、USB Serial/JTAG、SPI、U8g2、UART 等初始化，栈用量大，疑似溢出。

**修复：** `sdkconfig.esp32-c6-devkitc-1` 中增大至 8192。另在 `app_main()` 开头添加 GPIO 测试代码（LED 亮起 = 已进入 app_main），测试通过后可移除。
