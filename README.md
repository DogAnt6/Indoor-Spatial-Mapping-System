# Indoor-Spatial-Mapping-System
The Indoor Spatial Mapping System is an data acquisition device designed to produce a three-dimensional point cloud map of an enclosed space, such as a hallway or room. The system is built around the MCP432E401Y microcontroller operating at an individualized bus speed of 14 MHz.
<img width="447" height="417" alt="image" src="https://github.com/user-attachments/assets/54f916cf-53b0-4e88-9c2f-523943fe6e58" />
<img width="386" height="464" alt="image" src="https://github.com/user-attachments/assets/7b785a3b-bc45-4fb7-83c0-9a933b7589e3" />


<img width="1618" height="167" alt="image" src="https://github.com/user-attachments/assets/c357b75f-7033-4ecd-9c09-fd91b14a7f71" />

## Device Characteristics Table

| Parameter | Value | Notes |
| --- | --- | --- |
| Microcontroller | MSP432401Y | ARM Cortex-M4F, 120 MHz max clock speed |
| Assigned Bus Speed | 14 MHz | LSD of Student number: 1 |
| Operating Voltage | 3.3 V | MCU I/O and ToF sensor |
| Flash / SRAM | 1 MB / 256 KB | On-chip MSP432E401Y |
| I2C Bus (ToF -> MCU) | 100kH | PB2 (SCL), PB3 (SDA); I2C0 module |
| ToF Sensor | VL53L1X | Long mode, up to 4000 mm |
| ToF I2C Address | 0x29 | Default device address |
| Timing Budget | 100 ms | Per measurement |
| Inter-measurement Period | 150 ms | Minimum between measurements |
| Angular Resolution | 11.25° (32 steps / 360°) | Stepper motor full-step sequence |
| UART Baud Rate | 115,200 bps | 8N1 framing, USB-Serial to PC |
| UART Port | COM9 (configurable) | Assigned by PC OS; set in Python script |
| Programming Language (MCU) | C (Keil MDK) | MSP432E401Y + VL53L1X Driver |
| Programming Language (PC) | Python | PySerial, Open3d |
| Displacement per Scan | 30 cm (300 mm) | Manual linear movement along the x-axis |
| Stepper Motor Port | PH0–PH3 | full-step sequence via 4-bit output |
| Input Buttons | PM0, PM1 | Falling-edge interrupts (NVIC IRQ 72) |
| LED – Measurement Status | PN0 | Flashes each ToF measurement |
| LED – UART Tx Status | PN1 | Flashes at the start of each UART message |
| LED – Additional Status | PF4 | Flashes during returnHome() routine |
| Output File Formats | .xyz, .ply | Point cloud saved by Python script |
