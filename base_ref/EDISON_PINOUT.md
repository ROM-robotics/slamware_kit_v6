# Intel Edison Pinout for Slamware System

**Comprehensive Pin Reference for Intel Edison Integration with STM32 Base Controller**

---

## 📋 Table of Contents

- [Edison Module Overview](#edison-module-overview)
- [Breakout Board Pinout](#breakout-board-pinout)
- [Slamware Connection Mapping](#slamware-connection-mapping)
- [UART Configuration](#uart-configuration)
- [Additional Peripherals](#additional-peripherals)
- [Power Requirements](#power-requirements)
- [Pin Usage Guidelines](#pin-usage-guidelines)

---

## 🎯 Edison Module Overview

Intel Edison is a compute module with 70-pin connector (Hirose DF40 series):

### Key Features

| Feature | Specification |
|---------|---------------|
| **Form Factor** | 35.5mm × 25.0mm × 3.9mm |
| **Connector** | 70-pin Hirose DF40C-70DP-0.4V(51) |
| **CPU** | Intel Atom (Silvermont) dual-core @ 500MHz |
| **MCU** | Intel Quark @ 100MHz |
| **RAM** | 1GB LPDDR3 |
| **Storage** | 4GB eMMC |
| **I/O Voltage** | 1.8V (VSYS), 3.3V (VBAT) |
| **Power** | 3.3V - 4.5V input |

---

## 🔌 Breakout Board Pinout

### Standard Edison Breakout (SparkFun/Arduino Compatible)

```
                    ┌─────────────────────────┐
                    │   Intel Edison Module    │
                    │    (Top View)            │
                    └───────┬─────────┬────────┘
                            │         │
    J17 (Left Side)         │         │        J18 (Right Side)
    ════════════════════════════════════════════════════════
    Pin | Signal      | Dir |         | Dir | Signal      | Pin
    ════════════════════════════════════════════════════════
     1  | VSYS (NC)   |     |         |     | GP182 (PWM) | 1
     2  | VSYS        | PWR |         |     | GP135       | 2
     3  | NC          |     |         |     | GP27        | 3
     4  | VIN (4V-14V)| PWR |         |     | GP20        | 4
     5  | GP13        | I/O |         | PWR | GP28        | 5
     6  | GP165       | I/O |         | I/O | GP111       | 6
     7  | GP36        | I/O |         | I/O | GP109       | 7
     8  | GP27        | I/O |         | I/O | GP115       | 8
     9  | GP20        | I/O |         | I/O | GP128       | 9
    10  | GP28        | I/O |         | I/O | GP12        | 10
    11  | GP111       | I/O |         | I/O | GP183       | 11
    12  | GP109       | I/O |         | I/O | GP110       | 12
    13  | GP115       | I/O |         | I/O | GP114       | 13
    14  | GP128       | I/O |         |     | GP129       | 14
    15  | GP12        | I/O |         |     | GP130       | 15
    16  | GP183       | I/O |         |     | NC          | 16
    17  | GP110       | I/O |         |     | FW_RCVR     | 17
    18  | GP114       | I/O |         | PWR | VSYS        | 18
    19  | GP129       | I/O |         | PWR | GND         | 19
    20  | GND         | PWR |         | PWR | GND         | 20
```

### Mini Breakout (Compact Version)

```
    J1 (Expansion Header)
    ═══════════════════════════════════════
    Pin | Signal           | Function
    ═══════════════════════════════════════
     1  | VSYS (3.3V)      | Power Output
     2  | GND              | Ground
     3  | GP44 (UART2_TX)  | UART TX
     4  | GP45 (UART2_RX)  | UART RX
     5  | GP77 (I2C6_SDA)  | I2C Data
     6  | GP78 (I2C6_SCL)  | I2C Clock
     7  | GP182 (PWM2)     | PWM Output
     8  | GP135 (SPI5_CS)  | SPI Chip Select
```

---

## 🔗 Slamware Connection Mapping

### Critical Connections: Edison ↔ STM32F10x

```
┌────────────────────────────────────────────────────────────────┐
│                  UART Control Bus Connection                   │
├────────────────────────────────────────────────────────────────┤
│  Intel Edison          Signal           STM32F10x              │
│  ════════════          ══════           ═════════              │
│                                                                 │
│  GP131 (UART1_TX) ──→ PCIE_CTX ──→ PA10 (USART1_RX)          │
│                       [115200 baud]                            │
│                                                                 │
│  GP130 (UART1_RX) ←── PCIE_CRX ←── PA9  (USART1_TX)          │
│                       [115200 baud]                            │
│                                                                 │
│  GP14  (GPIO)     ←── PCIE_nCCMD ←── PC9  (GPIO_Out)         │
│                       [Command Signal]                         │
│                                                                 │
│  GP165 (GPIO)     ──→ PCIE_CBUSY ──→ PA12 (GPIO_In)          │
│                       [Busy Signal]                            │
│                                                                 │
│  GND              ──────────────────── GND                     │
│                                                                 │
│  VSYS (3.3V)      ─────[Optional]───── 3.3V                   │
│                       [If powering STM32 from Edison]          │
└────────────────────────────────────────────────────────────────┘
```

### Physical Pin Mapping

| Edison Pin | Edison GPIO | Function | Direction | STM32 Pin | STM32 Function |
|------------|-------------|----------|-----------|-----------|----------------|
| J18-14 | GP131 | UART1_TX | Output | PA10 | USART1_RX |
| J18-15 | GP130 | UART1_RX | Input | PA9 | USART1_TX |
| J17-2 | GP14 | GPIO | Input | PC9 | GPIO_Out (nCCMD) |
| J17-6 | GP165 | GPIO | Output | PA12 | GPIO_In (CBUSY) |
| J18-19 | GND | Ground | - | GND | Ground |

---

## 📡 UART Configuration

### UART1 (Primary Control Bus)

**Edison Side Configuration:**
```bash
# Linux device: /dev/ttyMFD2 (UART1)
stty -F /dev/ttyMFD2 115200 raw -echo -icrnl -ixon

# Settings:
# - Baud: 115200
# - Data: 8 bits
# - Parity: None
# - Stop: 1 bit
# - Flow control: None
```

**GPIO Pin Details:**
```
GP131 (UART1_TXD):
  - Edison Pin: J18-14
  - Mux Mode: Mode 1 (UART)
  - Pull: None
  - Voltage: 1.8V (internal), 3.3V compatible via level shifter

GP130 (UART1_RXD):
  - Edison Pin: J18-15
  - Mux Mode: Mode 1 (UART)
  - Pull: Pull-up enabled
  - Voltage: 1.8V (internal), 3.3V compatible via level shifter
```

### Alternative UART Options

| UART | Device | TX Pin | RX Pin | Use Case |
|------|--------|--------|--------|----------|
| **UART0** | /dev/ttyMFD0 | GP46 | GP47 | **Console (debug)** |
| **UART1** | /dev/ttyMFD1 | GP131 | GP130 | **STM32 Control (primary)** |
| **UART2** | /dev/ttyMFD2 | GP44 | GP45 | Expansion/Sensors |

---

## 🔧 Additional Peripherals

### I2C Buses (For Sensors/Expansion)

**I2C Bus 6 (Recommended for External Sensors):**
```
GP77 (I2C6_SDA) - J17-? 
GP78 (I2C6_SCL) - J17-?

Linux device: /dev/i2c-6

# Example: Scan I2C bus
i2cdetect -y 6
```

### SPI Interface (For High-Speed Peripherals)

**SPI5 (Available for Expansion):**
```
GP109 (SPI5_MOSI) - J18-7
GP115 (SPI5_MISO) - J18-8
GP114 (SPI5_CLK)  - J18-13
GP111 (SPI5_CS0)  - J18-6

Linux device: /dev/spidev5.1
Max Speed: 25MHz
```

### PWM Outputs (For Motor Control/LED)

```
GP182 (PWM0) - J18-1
GP12  (PWM1) - J18-10
GP13  (PWM2) - J17-5
GP183 (PWM3) - J18-11

Frequency range: 1Hz - 40MHz
Duty cycle resolution: 8-bit (256 steps)
```

### GPIO General Purpose

**Available GPIOs for Custom Functions:**
```
High-speed GPIOs (recommended):
  - GP20, GP27, GP28 (J17-9, J17-8, J17-10)
  - GP135, GP182 (J18-2, J18-1)
  - GP165, GP36 (J17-6, J17-7)

Medium-speed GPIOs:
  - GP12, GP13, GP14 (J18-10, J17-5, J17-2)
  - GP109, GP110, GP111, GP114, GP115 (J18-7,12,6,13,8)
```

---

## ⚡ Power Requirements

### Power Input Options

**Option 1: Via Breakout Board**
```
VIN (J17-4): 7V - 15V DC
  └─ Onboard regulator → 3.3V for Edison
  └─ Max current: 3A continuous
```

**Option 2: Direct VSYS**
```
VSYS (J17-2, J18-18): 3.3V - 4.5V DC
  └─ Direct to Edison power
  └─ Max current: 1.5A continuous
  └─ Bypass protection (use with caution)
```

### Power Consumption

| Mode | Current | Power @ 3.3V |
|------|---------|--------------|
| **Idle (WiFi off)** | 150mA | ~0.5W |
| **Active (CPU 100%)** | 400mA | ~1.3W |
| **WiFi transmit** | 600mA | ~2.0W |
| **Peak (WiFi + CPU)** | 800mA | ~2.6W |
| **Sleep mode** | 10mA | ~0.03W |

### Power Sequencing

```
1. Apply VSYS (3.3V - 4.5V)
2. Wait 10ms for stabilization
3. Release reset (if controlled)
4. Edison boots in ~10 seconds
5. Linux kernel loads
6. User applications start
```

---

## 📌 Slamware-Specific Pin Usage

### Standard Slamware Configuration

```
┌─────────────────────────────────────────────────────────┐
│               Edison Pin Allocation                      │
├─────────────────────────────────────────────────────────┤
│ UART1 (GP130/131)  → STM32 Base Controller             │
│ UART0 (GP46/47)    → Debug Console (USB-Serial)        │
│ UART2 (GP44/45)    → RPLidar Module (Optional)         │
│ I2C6  (GP77/78)    → IMU Sensor (MPU6050/BMI160)       │
│ SPI5  (GP109-115)  → Reserved (Future expansion)       │
│ GPIO  (GP14)       → nCMD Signal (STM32 attention)     │
│ GPIO  (GP165)      → BUSY Signal (Edison status)       │
│ GPIO  (GP182)      → LED Status (PWM for brightness)   │
│ GPIO  (GP20-28)    → User buttons / indicators         │
└─────────────────────────────────────────────────────────┘
```

### Typical Hardware Stack

```
┌────────────────────────────────────────────┐
│         RPLidar A1/A2 (UART2)              │
│         360° Laser Scanner                 │
└──────────────┬─────────────────────────────┘
               │ UART @ 115200
┌──────────────▼─────────────────────────────┐
│         Intel Edison Module                │
│  ┌──────────────────────────────────────┐  │
│  │ Slamware Core (Navigation Stack)    │  │
│  │  - SLAM mapping                      │  │
│  │  - Path planning                     │  │
│  │  - Localization                      │  │
│  └──────────────────────────────────────┘  │
└──────────────┬─────────────────────────────┘
               │ UART1 @ 115200
┌──────────────▼─────────────────────────────┐
│         STM32F10x Base Controller          │
│  - Motor control (60Hz)                    │
│  - Sensor reading                          │
│  - Safety systems                          │
└────────────────────────────────────────────┘
```

---

## 🛠️ Pin Configuration Commands

### Linux GPIO Control (Edison Side)

**Export GPIO for use:**
```bash
# Enable GPIO (e.g., GP14)
echo 14 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio14/direction
echo 1 > /sys/class/gpio/gpio14/value

# Read GPIO state
cat /sys/class/gpio/gpio14/value

# Cleanup
echo 14 > /sys/class/gpio/unexport
```

### UART Setup Script

```bash
#!/bin/bash
# setup_uart.sh - Configure UART1 for STM32 communication

UART_DEV="/dev/ttyMFD1"

# Configure serial port
stty -F $UART_DEV 115200 cs8 -cstopb -parenb -echo raw

# Disable flow control
stty -F $UART_DEV -crtscts -ixon -ixoff

# Flush buffers
cat $UART_DEV > /dev/null &
CATPID=$!
sleep 0.1
kill $CATPID 2>/dev/null

echo "UART1 configured for STM32 communication"
```

### Python GPIO Example

```python
#!/usr/bin/env python
import mraa
import time

# Initialize GPIO14 (nCMD signal to STM32)
ncmd_pin = mraa.Gpio(14)
ncmd_pin.dir(mraa.DIR_OUT)

# Initialize GPIO165 (BUSY signal from Edison)
busy_pin = mraa.Gpio(165)
busy_pin.dir(mraa.DIR_OUT)

# Signal STM32 attention
def signal_stm32():
    ncmd_pin.write(0)  # Pull low
    time.sleep(0.001)  # 1ms pulse
    ncmd_pin.write(1)  # Release

# Indicate Edison busy
def set_busy(state):
    busy_pin.write(1 if state else 0)

# Example usage
set_busy(True)
signal_stm32()
time.sleep(0.1)
set_busy(False)
```

### Python UART Example

```python
#!/usr/bin/env python
import serial
import struct

# Open UART1
ser = serial.Serial(
    port='/dev/ttyMFD1',
    baudrate=115200,
    bytesize=8,
    parity='N',
    stopbits=1,
    timeout=1
)

# Send command to STM32 (Get Battery Status)
def get_battery_status():
    # Command: 0xF8 0x30 (SLAMWARECORE_CTRL_BUS + GET_BASE_STATUS)
    cmd = bytearray([0xF8, 0x30])
    ser.write(cmd)
    
    # Wait for response
    response = ser.read(16)  # Read expected response size
    
    if len(response) >= 2:
        battery_percent = response[0]
        power_status = response[1]
        print(f"Battery: {battery_percent}%, Status: {power_status}")
        return battery_percent, power_status
    return None, None

# Main loop
try:
    while True:
        get_battery_status()
        time.sleep(1)
except KeyboardInterrupt:
    ser.close()
```

---

## ⚠️ Pin Usage Guidelines

### Critical Warnings

1. **Voltage Levels:**
   - Edison GPIOs are **1.8V logic** internally
   - Breakout boards include level shifters for 3.3V compatibility
   - **Never apply 5V directly to Edison pins**

2. **Current Limits:**
   - Max current per GPIO: **6mA** (absolute max: 10mA)
   - Do not drive LEDs, relays, or motors directly
   - Use transistor/MOSFET drivers for loads >5mA

3. **Reserved Pins:**
   - GP46/GP47 (UART0) - Console, do not reassign
   - GP134-GP137 - WiFi/Bluetooth, do not use
   - GP19, GP84, GP42, GP43 - USB OTG

### Best Practices

```
✅ DO:
  - Use level shifters for 5V devices
  - Add pull-ups on I2C lines (4.7kΩ)
  - Include ESD protection on exposed pins
  - Use series resistors (100-330Ω) for LED indicators
  - Add decoupling capacitors near Edison (100nF + 10µF)

❌ DON'T:
  - Connect 5V signals directly
  - Source/sink >6mA per pin
  - Use pins without checking pinmux configuration
  - Forget to configure pin direction before use
  - Hot-plug Edison with power applied
```

### Pinmux Configuration (Advanced)

Edison pins can be multiplexed for different functions:

```bash
# Check current pinmux (on Edison via SSH)
cat /sys/kernel/debug/gpio_debug/current_pinmux

# Example output:
# pin 130 (GP130): mode1 func1 (UART1_RXD) pullup
# pin 131 (GP131): mode1 func1 (UART1_TXD) pulldown

# Change pinmux (requires root, be careful!)
# echo <pin> <mode> > /sys/kernel/debug/gpio_debug/set_pinmux
```

---

## 📖 Reference Documents

### Official Intel Resources

- [Intel Edison Datasheet](https://www.intel.com/content/dam/support/us/en/documents/edison/sb/edison_datasheet_331179002.pdf)
- [Edison Breakout Hardware Guide](https://www.intel.com/content/dam/support/us/en/documents/edison/sb/edison_breakout_hg_331190006.pdf)
- [Edison GPIO Pin Multiplexing](https://software.intel.com/en-us/iot/hardware/edison/pins)

### Community Resources

- [SparkFun Edison Hookup Guide](https://learn.sparkfun.com/tutorials/edison-getting-started-guide)
- [MRAA Library Documentation](https://iotdk.intel.com/docs/master/mraa/)
- [UPM Sensor Library](https://iotdk.intel.com/docs/master/upm/)

### SlamTec Integration

- SlamTec Slamware SDK Documentation
- RPLidar Communication Protocol
- Base Controller Interface Specification (see main README.md)

---

## 🔍 Troubleshooting

### UART Not Working

**Check:**
```bash
# 1. Verify UART device exists
ls -l /dev/ttyMFD*

# 2. Check permissions
sudo chmod 666 /dev/ttyMFD1

# 3. Test loopback (short TX to RX on Edison)
echo "test" > /dev/ttyMFD1 &
cat /dev/ttyMFD1

# 4. Check kernel messages
dmesg | grep tty
```

### GPIO Not Responding

**Debug:**
```bash
# 1. Check if GPIO exported
ls /sys/class/gpio/gpio14/

# 2. Verify direction
cat /sys/class/gpio/gpio14/direction

# 3. Read current value
cat /sys/class/gpio/gpio14/value

# 4. Check pinmux
cat /sys/kernel/debug/gpio_debug/current_pinmux | grep 14
```

### Power Issues

**Symptoms & Solutions:**
```
Problem: Edison not booting
  → Check VIN voltage (7-15V) or VSYS (3.3-4.5V)
  → Measure current (should be 150-400mA during boot)
  → Inspect power LED on breakout

Problem: Random resets
  → Add bulk capacitor (100µF) near VIN
  → Check voltage droop during WiFi transmission
  → Reduce CPU load or disable WiFi temporarily

Problem: USB not detected
  → Check USB OTG cable (must support data)
  → Install Edison drivers (Windows)
  → Try different USB port (avoid USB hubs)
```

---

## 📊 Quick Reference Card

```
╔════════════════════════════════════════════════════════╗
║         EDISON PINOUT QUICK REFERENCE                  ║
╠════════════════════════════════════════════════════════╣
║  UART1 (STM32):  GP130 (RX), GP131 (TX) @ 115200     ║
║  UART0 (Debug):  GP46 (RX), GP47 (TX) @ 115200       ║
║  I2C6 (Sensor):  GP77 (SDA), GP78 (SCL)              ║
║  SPI5 (Expand):  GP109/115/114/111 (MOSI/MISO/CLK/CS)║
║  GPIO (Control): GP14 (nCMD), GP165 (BUSY)           ║
║  PWM (Output):   GP182, GP12, GP13, GP183            ║
║  Power Input:    VIN (7-15V) or VSYS (3.3-4.5V)      ║
║  Logic Level:    1.8V internal, 3.3V on breakout     ║
║  Max GPIO Current: 6mA per pin                        ║
╚════════════════════════════════════════════════════════╝
```

---

*Document Version: 1.0*  
*Last Updated: January 1, 2026*  
*Compatible with: Intel Edison (all revisions), Slamware Base Reference Firmware v0.90*
