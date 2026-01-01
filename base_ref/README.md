# SlamTec Base Reference Firmware

**STM32F10x-based Robot Base Controller for Slamware Navigation System**

---

## 📋 Table of Contents

- [Overview](#overview)
- [Hardware Architecture](#hardware-architecture)
- [Software Architecture](#software-architecture)
- [Communication Protocol](#communication-protocol)
- [Intel Edison Integration](#intel-edison-integration)
- [Building the Project](#building-the-project)
- [Hardware Pin Mapping](#hardware-pin-mapping)
- [Driver Components](#driver-components)
- [Development Guide](#development-guide)
- [Troubleshooting](#troubleshooting)

---

## 🎯 Overview

This firmware implements a real-time base controller for SlamTec's robot vacuum platform. It manages:

- **Motor Control**: Differential drive with closed-loop speed control (60Hz)
- **Sensor Reading**: IR distance, sonar, bumpers, encoders
- **Battery Management**: Charging detection, power monitoring
- **Safety Systems**: Watchdog, collision detection, ground detection
- **Communication**: High-level commands from Intel Edison navigation core

### Key Specifications

| Parameter | Value |
|-----------|-------|
| **MCU** | STM32F10x (ARM Cortex-M3 @ 72MHz) |
| **IDE** | IAR Embedded Workbench / Keil MDK |
| **RTOS** | Bare-metal (super-loop architecture) |
| **Flash** | ~128KB program memory |
| **RAM** | ~20KB SRAM |
| **Control Loop** | 60Hz motor control, 1kHz system tick |

---

## 🔧 Hardware Architecture

### System Block Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    Intel Edison Module                       │
│            (Yocto Linux + Slamware Core)                    │
│                 Navigation & Mapping                         │
└────────────────────┬────────────────────────────────────────┘
                     │ UART @ 115200 baud
                     │ (Labeled as "PCIe" in schematic)
                     ↓
┌─────────────────────────────────────────────────────────────┐
│                  STM32F10x Base Controller                   │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Core Functions:                                      │  │
│  │  • Real-time motor control (60Hz PWM)               │  │
│  │  • Encoder odometry (6390 pulses/meter)             │  │
│  │  • Sensor fusion & filtering                        │  │
│  │  • Safety interlocks (1s heartbeat timeout)         │  │
│  └──────────────────────────────────────────────────────┘  │
└───┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬───────┘
    │     │     │     │     │     │     │     │     │
   M1    M2    IR   Sonar Bump Batt LED  Beep Encoder
  (L)   (R)  Sensors  x4    x2   Monitor WS2812 PWM  x2
```

### Board Revisions

| Revision | Features | UART Mapping |
|----------|----------|--------------|
| **Rev 3** | Basic sensors, 3 home IR | USART1 → Edison |
| **Rev 4** | Added sonar, improved IR | USART3 → Edison |
| **Rev 6** | Enhanced sonar array (4ch) | USART3 → Edison |

---

## 💻 Software Architecture

### Directory Structure

```
base_ref/
├── src/
│   ├── main.c                    # Main loop & initialization
│   ├── request_handler.c         # Command processor
│   ├── hwconf.c                  # Hardware configuration
│   ├── bump_monitor.c            # Collision event handler
│   │
│   ├── drv/                      # Driver layer
│   │   ├── motor.c/h             # Walking motors + PWM
│   │   ├── battery.c/h           # Power monitoring
│   │   ├── bump.c/h              # Collision sensors
│   │   ├── distir.c/h            # Bottom IR distance
│   │   ├── homeir.c/h            # Home beacon receivers
│   │   ├── sonar.c/h             # Ultrasonic sensors
│   │   ├── led.c/h               # WS2812 RGB LED
│   │   ├── beep.c/h              # Audio feedback
│   │   ├── serial_channel.c/h    # UART abstraction
│   │   ├── drv_ctrlbus.c/h       # Control bus signals
│   │   └── watchdog.c/h          # Safety watchdog
│   │
│   ├── boardfunc/                # Board-specific configs
│   │   ├── revision3/hwconf.h
│   │   ├── revision4/hwconf.h
│   │   └── revision6/hwconf.h
│   │
│   └── common/
│       └── common.h              # Global definitions
│
├── iar_ldscript/                 # Linker scripts
├── keil/                         # Keil project files
├── base_ref.ewp                  # IAR project
└── base_ref.eww                  # IAR workspace
```

### Program Flow

```c
main()
  ↓
init_board()              // MCU low-level: clocks, GPIO, interrupts
  ↓
init_dev()                // Peripherals: UART, motors, sensors
  ├─ drv_serialchannel_init(USART1, 115200)
  ├─ net_bind()           // Attach interchip protocol
  ├─ init_battery()
  ├─ init_walkingmotor()
  ├─ init_walkingmotor_odometer()
  ├─ init_bump_detect()
  ├─ init_sonar()
  └─ health_monitor_init()
  ↓
enable_watchdog()
  ↓
╔════════════════════════════════════════════╗
║            MAIN LOOP (infinite)            ║
╠════════════════════════════════════════════╣
║  1. net_poll_request()                     ║
║     └─ Check for incoming commands         ║
║                                            ║
║  2. on_host_request()                      ║
║     └─ Parse & execute Edison commands     ║
║                                            ║
║  3. dev_heartbeat()                        ║
║     ├─ heartbeat_battery()                 ║
║     ├─ heartbeat_bumpermonitor()           ║
║     ├─ heartbeat_beep()                    ║
║     ├─ heartbeat_distir()                  ║
║     ├─ heartbeat_sonar()                   ║
║     ├─ speedctl_heartbeat()   (60Hz!)     ║
║     └─ health_monitor_heartbeat()          ║
║                                            ║
║  4. shutdown_heartbeat()                   ║
║     └─ Stop motors if no Edison command    ║
║        received for >1 second              ║
║                                            ║
║  5. mark_watchdog()                        ║
║     └─ Reset hardware watchdog timer       ║
╚════════════════════════════════════════════╝
```

---

## 🔌 Communication Protocol

### Physical Connection

**UART-based Serial Protocol (Mislabeled as "PCIe")**

```
Pin Mapping (STM32 ↔ Intel Edison):

STM32F1              Signal              Intel Edison
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PA9  (USART1_TX) ─────→ PCIE_CRX  ────→ UART RX
PA10 (USART1_RX) ←───── PCIE_CTX  ←──── UART TX
PC9  (GPIO)      ─────→ PCIE_nCCMD ───→ Command signal
PA12 (GPIO)      ←───── PCIE_CBUSY ←─── Busy flag
GND              ─────────────────────── GND

Configuration:
• Baud Rate: 115200 bps
• Data: 8 bits, No parity, 1 stop bit
• Flow Control: None (hardware)
• Protocol: Custom binary packet format
```

### Protocol Stack

```
┌─────────────────────────────────────────────┐
│  Application Layer (request_handler.c)     │
│  Commands: Motor, Sensor, Status, Events    │
├─────────────────────────────────────────────┤
│  Interchip Protocol (net_* functions)       │
│  • Packet framing with header/CRC           │
│  • Command/Response model                   │
│  • Error handling                           │
├─────────────────────────────────────────────┤
│  Serial Channel Driver (serial_channel.c)   │
│  • RX ring buffer (512 bytes)               │
│  • TX ring buffer (512 bytes)               │
│  • Interrupt-driven I/O                     │
├─────────────────────────────────────────────┤
│  USART Hardware (STM32 peripheral)          │
│  • DMA transfers                            │
│  • Hardware FIFO                            │
└─────────────────────────────────────────────┘
```

### Command Set

#### **Base Command Code: 0xF8** (SLAMWARECORE_CTRL_BUS)

| SubCmd | Name | Request | Response | Description |
|--------|------|---------|----------|-------------|
| `0x10` | CONNECT_BASE | Protocol version | Model, FW/HW ver, S/N | Handshake |
| `0x20` | GET_BASE_CONF | - | Robot geometry, sensors | Configuration |
| `0x30` | GET_BASE_STATUS | - | Battery %, charging status | Power info |
| `0x31` | GET_BASE_MOTOR_DATA | - | Left/right encoder counts | Odometry |
| `0x32` | GET_BASE_SENSOR_DATA | - | Sonar/IR distances (mm) | Environment |
| `0x33` | GET_BASE_BUMPER_DATA | - | Collision bitmap | Safety |
| `0x34` | GET_AUTO_HOME_DATA | Data type | Beacon IR signals | Docking |
| `0x40` | SET_BASE_MOTOR | Speed (mm/s) L/R | ACK | Direct control |
| `0x41` | SET_V_AND_GET_DEADRECKON | Vx, Vy, ω | Δx, Δy, Δθ | Velocity + pose |
| `0x50` | POLL_BASE_CMD | - | Pending commands | STM32→Edison |
| `0x60` | SEND_EVENT | Event code | ACK | Start/stop actions |
| `0x90` | HEALTH_MGMT | Health query | Status/errors | Diagnostics |

#### Example Transaction

```
Edison Request (Get Motor Data):
┌────┬────┬────┬─────┬─────┐
│ F8 │ 31 │ ... checksum ...│
└────┴────┴────┴─────┴─────┘
 CMD  Sub  Payload  CRC

STM32 Response:
┌────────────┬────────────┬────────┬─────┐
│ Left_mm    │ Right_mm   │ Status │ CRC │
│ (4 bytes)  │ (4 bytes)  │ (1 B)  │     │
└────────────┴────────────┴────────┴─────┘
   12345         12340        0x00
```

### Safety Features

1. **Heartbeat Timeout**
   ```c
   if ((current_time - last_command_time) > 1000ms) {
       set_walkingmotor_speed(0, 0);  // Emergency stop
   }
   ```

2. **Watchdog Timer**
   - Hardware watchdog enabled
   - Must be reset every loop iteration
   - Auto-reboot if firmware hangs

3. **Ground Detection**
   - Wheels lifted → Stop motors immediately
   - Prevents runaway if picked up

---

## 🖥️ Intel Edison Integration

### Edison Overview

Intel Edison is a dual-core compute module running Yocto Linux:

| Component | Specification |
|-----------|---------------|
| **CPU** | Intel Atom (Silvermont) @ 500MHz dual-core |
| **MCU** | Intel Quark @ 100MHz (I/O coprocessor) |
| **RAM** | 1GB LPDDR3 |
| **Storage** | 4GB eMMC flash |
| **WiFi** | 802.11 a/b/g/n dual-band |
| **Bluetooth** | 4.0 LE |
| **OS** | Yocto Linux 1.6+ (custom builds) |

### Edison Role in System

```
┌─────────────────────────────────────────┐
│         Intel Edison (Master)           │
├─────────────────────────────────────────┤
│ • SLAM mapping (laser scan processing)  │
│ • Path planning & navigation            │
│ • WiFi connectivity & cloud sync        │
│ • User interface (app/web)              │
│ • High-level decision making            │
│ • Sensor fusion algorithms              │
└──────────────┬──────────────────────────┘
               │ Commands (10-50 Hz)
               ↓
┌─────────────────────────────────────────┐
│         STM32F10x (Slave)               │
├─────────────────────────────────────────┤
│ • Motor control loop (60Hz)             │
│ • Encoder reading (interrupt-driven)    │
│ • Sensor acquisition (ADC, timers)      │
│ • Safety interlocks (real-time)         │
│ • Low-level hardware abstraction        │
└─────────────────────────────────────────┘
```

---

## 🔨 Building the Project

### Prerequisites

**Option 1: IAR Embedded Workbench (Recommended)**

```bash
# Required:
- IAR EWARM 7.40+ (or latest version)
- IAR License (commercial or time-limited eval)

# Optional:
- J-Link debugger (for JTAG/SWD programming)
- ST-Link V2 (alternative programmer)
```

**Option 2: Keil MDK**

```bash
# Required:
- Keil MDK-ARM 5.x
- ARM Compiler 5/6
- Keil license

# Project files provided in keil/ directory
```

### Build Steps (IAR)

```bash
# 1. Open workspace
$ cd base_ref/
$ iar base_ref.eww

# 2. Select build configuration:
#    - Debug (with debug symbols)
#    - Release_Rev6 (optimized for Rev 6 board)

# 3. Set board revision (if needed):
#    Project → Options → C/C++ Compiler → Preprocessor
#    Add: CONFIG_BREAKOUT_REV=6

# 4. Build:
#    Project → Make (F7)
#    or
#    Project → Rebuild All

# 5. Flash to target:
#    Project → Download and Debug (Ctrl+D)
```

### Build Output

```
Release_Rev6/Exe/
├── base_ref.bin        # Raw binary (for DFU)
├── base_ref.hex        # Intel HEX (for ST-Link)
└── base_ref.out        # ELF with debug symbols
```

### Flashing Firmware

**Using ST-Link Utility:**
```bash
# Windows
ST-LINK_CLI.exe -P base_ref.hex -V -Rst

# Linux (openocd)
openocd -f interface/stlink-v2.cfg \
        -f target/stm32f1x.cfg \
        -c "program base_ref.hex verify reset exit"
```

**Using J-Link:**
```bash
JLinkExe -device STM32F103CB -if SWD -speed 4000
> loadbin base_ref.bin 0x08000000
> r
> g
> exit
```

---

## 📌 Hardware Pin Mapping

### MCU: STM32F103 (LQFP64 package)

#### Motor Control

| Pin | Function | Type | Description |
|-----|----------|------|-------------|
| PD4 | MOTO_LF_EN | GPIO Out | Left motor forward enable |
| PD9 | MOTO_LB_EN | GPIO Out | Left motor backward enable |
| PE14 | MOTO_L_PWM | TIM1_CH4 | Left motor PWM (0-100%) |
| PD6 | MOTO_RF_EN | GPIO Out | Right motor forward enable |
| PD7 | MOTO_RB_EN | GPIO Out | Right motor backward enable |
| PE13 | MOTO_R_PWM | TIM1_CH3 | Right motor PWM (0-100%) |

#### Encoders

| Pin | Function | Type | Frequency |
|-----|----------|------|-----------|
| PD3 | ENCODER_L | EXTI3 | ~6.39kHz @ 1m/s |
| PD2 | ENCODER_R | EXTI2 | ~6.39kHz @ 1m/s |

Resolution: **6390 pulses/meter** (wheel circumference ~300mm)

#### Sensors - IR Distance

| Pin | Function | ADC Channel | Range |
|-----|----------|-------------|-------|
| PC1 | BOTTOM_IR_R2 | ADC12_IN11 | 0-100mm |
| PC2 | BOTTOM_IR_R1 | ADC12_IN12 | 0-100mm |
| PA4 | BOTTOM_IR_R4 | ADC12_IN4 | 0-100mm |
| PC4 | BOTTOM_IR_R3 | ADC12_IN14 | 0-100mm |
| PC7 | BOTTOM_IR_E | TIM3 (LED) | IR emitter PWM |

#### Sensors - Sonar (Rev 3+)

| Pin | Function | Direction | Range |
|-----|----------|-----------|-------|
| PE10 | SONAR_TRIG1 | Output | Trigger pulse |
| PE5 | SONAR_ECHO1 | Input | Echo timing |
| PE11 | SONAR_TRIG2 | Output | 20-4000mm |
| PE7 | SONAR_ECHO2 | Input | typical |
| PE12 | SONAR_TRIG3 | Output | |
| PE8 | SONAR_ECHO3 | Input | |
| PE15 | SONAR_TRIG4 | Output | |
| PE9 | SONAR_ECHO4 | Input | |

#### Sensors - Home IR Beacon

| Pin | Function | Timer | Purpose |
|-----|----------|-------|---------|
| PD12 | HOME_IR_R1 | TIM4_CH1 | Left beacon receiver |
| PD13 | HOME_IR_R2 | TIM4_CH2 | Main beacon receiver |
| PD14 | HOME_IR_R3 | TIM4_CH3 | Right beacon receiver |

#### Collision Detection

| Pin | Function | Type | Trigger |
|-----|----------|------|---------|
| PB5 | BUMP_DETECT_L | GPIO In | Active LOW |
| PB13 | BUMP_DETECT_R | GPIO In | Active LOW |
| PD10 | GROUND_DETECT_R | GPIO In | Wheel lift |
| PD1 | GROUND_DETECT_L | GPIO In | Wheel lift |

#### Power Management

| Pin | Function | Type | Description |
|-----|----------|------|-------------|
| PA6 | BATT_DETECT | ADC12_IN6 | Battery voltage (divider) |
| PC3 | BATT_MONITOR | ADC12_IN13 | Current sense |
| PE2 | BATT_READY | GPIO In | Charger ready |
| PE3 | BATT_CHRG | GPIO In | Charging status |
| PE4 | BATT_FAULT | GPIO In | Charger fault |
| PB8 | HOCHARGE_DETECT | GPIO In | Home dock detect |
| PB9 | DCCHARGE_DETECT | GPIO In | DC adapter detect |
| PA0 | CHARGE_PWM | PWM Out | Charge current control |

#### User Feedback

| Pin | Function | Type | Description |
|-----|----------|------|-------------|
| PB12 | LED_WS2812 | GPIO Out | RGB LED data (800kHz) |
| PA15 | BEEP_PWM | TIM2_CH1 | Buzzer PWM (1-5kHz) |

#### Communication

| Pin | Function | USART | Baud | Connected To |
|-----|----------|-------|------|--------------|
| PA9 | PCIE_CRX | USART1_TX | 115200 | Edison RX |
| PA10 | PCIE_CTX | USART1_RX | 115200 | Edison TX |
| PC9 | PCIE_nCCMD | GPIO Out | - | Command signal |
| PA12 | PCIE_CBUSY | GPIO In | - | Busy flag |
| PA2 | DISPLAY_TX | USART2_TX | 115200 | Optional display |
| PA3 | DISPLAY_RX | USART2_RX | 115200 | Optional display |
| PC10 | PC10_TX | USART3_TX | 115200 | Debug/expansion |
| PC11 | PC11_RX | USART3_RX | 115200 | Debug/expansion |

#### Programming/Debug

| Pin | Function | Type |
|-----|----------|------|
| PA13 | SWDIO | SWD |
| PA14 | SWCLK | SWD |

---

## 🔧 Driver Components

### Motor Control (`motor.c/h`)

**Features:**
- Differential drive control
- Closed-loop speed control (60Hz PID)
- PWM frequency: 20kHz
- Speed range: -1000 to +1000 (mm/s)

**Key Functions:**
```c
void init_walkingmotor(void);
void set_walkingmotor_speed(_s32 lSpeed, _s32 rSpeed);
_u32 walkingmotor_cumulate_ldist_mm(void);  // Odometry
float walkingmotor_delta_ldist_mm_f(void);   // Incremental
```

**Control States:**
```c
enum motorCtrlState_t {
    MOTOR_CTRL_STATE_RELEASE = 0,   // Coast (high-Z)
    MOTOR_CTRL_STATE_FORWARD = 1,   // Forward
    MOTOR_CTRL_STATE_BACKWARD = 2,  // Reverse
    MOTOR_CTRL_STATE_BRAKE = 3,     // Active brake
};
```

### Battery Management (`battery.c/h`)

**Monitoring:**
- Battery voltage (via ADC with divider)
- Charge current measurement
- Percentage estimation (lookup table)
- Low battery warning (< 30%)
- Critical shutdown (< 10%)

**Charge Detection:**
- Home dock contact detection
- DC adapter presence
- Fault monitoring

**Functions:**
```c
_u8 get_electricitypercentage(void);     // 0-100%
_u8 get_home_charge_status(void);        // Boolean
_u8 get_dc_charge_status(void);          // Boolean
_u16 get_battery_voltage_adc(void);      // Raw ADC
```

### Collision Detection (`bump.c/h`)

**Hardware:**
- Mechanical switches (normally closed)
- Debouncing in software (50ms)
- Bitmap output (left/right)

**Integration:**
```c
void init_bump_detect(void);
_u8 get_bump_bitmap(void);  // Bit 0: Right, Bit 1: Left
```

### Distance IR Sensors (`distir.c/h`)

**Configuration:**
- 4 bottom-facing IR sensors
- GP2Y0A21YK (Sharp) compatible
- Range: 10-80cm (exponential curve)
- ADC sampling: 1kHz → filtered

**Output:**
```c
typedef struct {
    _u8 bottomSensorBitmap;  // Digital (threshold)
    _u16 bottomSensorAdc[4]; // Raw ADC values
} distir_value_t;

distir_value_t* get_distir_value(void);
```

### Ultrasonic Sensors (`sonar.c/h`)

**Specifications:**
- HC-SR04 compatible
- 4 channels (Rev 3+)
- Range: 2cm - 4m
- Trigger: 10µs pulse
- Echo: Pulse width → distance

**Timing:**
```c
void init_sonar(void);
_u32 sonar_get(_u8 channel);  // Returns distance in mm
```

### Home Beacon IR (`homeir.c/h`)

**Purpose:**
- Docking station localization
- 3 receivers (left, main, right)
- IR modulation detection (38kHz carrier)
- Signal strength output

**Functions:**
```c
_u8 homeir_getleftdata(void);   // 0-255 signal strength
_u8 homeir_getmaindata(void);
_u8 homeir_getrightdata(void);
```

### LED Control (`led.c/h`)

**WS2812B RGB LED:**
- Addressable LED (NeoPixel protocol)
- 800kHz bitstream generation
- Color format: GRB (24-bit)
- Timing-critical (bit-banging)

**API:**
```c
void drv_led_init(void);
void drv_led_set_color(_u8 r, _u8 g, _u8 b);
void drv_led_shutdown(void);
```

### Audio Feedback (`beep.c/h`)

**Buzzer Control:**
- PWM-based tone generation
- Frequency: 1000-5000 Hz
- Patterns: power-on, error, confirm

**Sequences:**
```c
void play_poweron(void);      // Startup jingle
void play_error(void);        // Warning beep
void heartbeat_beep(void);    // Pattern player
```

---

## 📚 Development Guide

### Adding New Commands

**1. Define command in `ctrl_bus_cmd.h`:**
```c
#define SLAMWARECORECB_CMD_MY_NEW_CMD  (0x70)

typedef struct _my_new_request {
    _u32 parameter1;
    _u16 parameter2;
} __attribute__((packed)) my_new_request_t;

typedef struct _my_new_response {
    _u8 result;
    _u32 data;
} __attribute__((packed)) my_new_response_t;
```

**2. Implement handler in `request_handler.c`:**
```c
case SLAMWARECORECB_CMD_MY_NEW_CMD:
    {
        my_new_request_t *req = (my_new_request_t *)request->payload;
        my_new_response_t ans_pkt;
        
        // Process request
        ans_pkt.result = do_something(req->parameter1);
        ans_pkt.data = get_data();
        
        net_send_ans(channel, &ans_pkt, sizeof(my_new_response_t));
    }
    break;
```

### Adding New Sensors

**1. Create driver files: `drv/newsensor.c` and `drv/newsensor.h`**

**2. Initialize in `main.c::init_dev()`:**
```c
init_newsensor();
```

**3. Add heartbeat if needed:**
```c
void dev_heartbeat(void) {
    // ... existing code ...
    heartbeat_newsensor();
}
```

**4. Expose data via command handler**

### Debugging Tips

**Serial Debug Output:**
```c
// Enable in common.h:
#define _DEBUG

// Use in code:
DBG_OUT("Motor speed: L=%d R=%d\r\n", left_speed, right_speed);
```

**Real-time Monitoring:**
```bash
# Connect USB-Serial adapter to USART2/3
screen /dev/ttyUSB0 115200

# Watch debug output
```

**LED Status Codes:**
- **Blue:** Normal operation
- **Red:** Error state
- **Green:** Charging
- **Blinking:** Waiting for Edison

---

## 🐛 Troubleshooting

### Problem: Motors don't respond

**Check:**
1. Motor enable pins configured? (`pinMode()` calls)
2. PWM timer initialized? (`TIM1` running)
3. Speed command received from Edison? (add `DBG_OUT`)
4. H-bridge power supply present?
5. Ground detection triggered? (wheels lifted)

**Debug:**
```c
// In motor.c, add:
DBG_OUT("Set speed L=%d R=%d\r\n", lSpeed, rSpeed);
```

### Problem: No communication with Edison

**Check:**
1. UART pins correct? (PA9=TX, PA10=RX for USART1)
2. Baud rate matches? (115200)
3. Edison booted? (check via Edison serial console)
4. TX/RX crossed? (STM32 TX → Edison RX)
5. Voltage levels compatible? (3.3V logic both sides)

**Test:**
```c
// Echo test in main loop:
if (USART1->SR & USART_SR_RXNE) {
    uint8_t data = USART1->DR;
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = data;  // Echo back
}
```

### Problem: Encoder counts wrong

**Check:**
1. Interrupt enabled? (`NVIC_EnableIRQ`)
2. Pull-up resistors on encoder pins?
3. Debouncing adequate?
4. Direction detection correct?

**Calibration:**
```c
// Measure real distance vs. reported:
// Move robot exactly 1 meter
// Check: walkingmotor_cumulate_ldist_mm()
// Adjust: ODOMETER_EST_PULSE_PER_METER
```

### Problem: Sonar readings unstable

**Check:**
1. Echo timeout implemented? (prevent infinite wait)
2. Multiple triggers interfering? (sequence channels)
3. Acoustic reflections? (angled surfaces)
4. Power supply noise? (capacitors near sensors)

**Filter:**
```c
// Median filter for 5 samples
_u32 median_filter(_u32 samples[5]) {
    // Sort and return middle value
}
```

### Problem: Battery percentage incorrect

**Check:**
1. ADC reference voltage calibrated? (`ADC_REF_VOLT`)
2. Voltage divider ratio correct?
3. Battery chemistry curve matches? (Li-ion vs NiMH)

**Calibrate:**
```c
// Measure battery voltage with multimeter
// Compare with: get_battery_voltage_adc()
// Adjust divider ratio in hwconf.h
```

---

## 📖 References

### Hardware Datasheets

- [STM32F103 Reference Manual](https://www.st.com/resource/en/reference_manual/cd00171190.pdf)
- [STM32F103 Datasheet](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)
- [WS2812B LED Datasheet](https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf)
- [HC-SR04 Ultrasonic Sensor](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)

### Software Documentation

- [IAR C/C++ Compiler Guide](https://wwwfiles.iar.com/arm/webic/doc/EWARM_DevelopmentGuide.ENU.pdf)
- [CMSIS Documentation](https://www.keil.com/pack/doc/CMSIS/General/html/index.html)

### SlamTec Resources

- [Slamware SDK](https://www.slamtec.com/en/Support)
- [RPLidar SDK](https://github.com/Slamtec/rplidar_sdk)

---

## 📄 License

```
SlamTec Base Ref Design
Copyright 2009 - 2017 RoboPeak
Copyright 2013 - 2017 Shanghai SlamTec Co., Ltd.
http://www.slamtec.com
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED.
```

---

## 🤝 Contributing

For commercial licensing, custom development, or technical support:

**Contact:** SlamTec Co., Ltd.  
**Email:** dev@slamtec.com  
**Web:** https://www.slamtec.com

---

## 📊 Version History

| Version | Date | Changes |
|---------|------|---------|
| 0.90 | 2017-Q2 | Rev 6 support, sonar array, health monitor |
| 0.80 | 2016-Q4 | Rev 4 support, improved IR sensors |
| 0.70 | 2016-Q2 | Rev 3 initial release |

**Current Firmware Version:** `0.90` (defined in `common.h`)

---

*Last Updated: January 1, 2026*
