# SARE Cart Control — Embedded Firmware

> STM32F401RCTx · CubeIDE · HAL · 3-Layer Architecture

---

## Table of Contents

- [Project Overview](#project-overview)
- [Target Hardware](#target-hardware)
- [Software Architecture](#software-architecture)
- [Dependency Audit](#dependency-audit)
- [Include Path Rules](#include-path-rules)
- [Project Structure](#project-structure)
- [Sensor Drivers](#sensor-drivers)

---

## Project Overview

Embedded firmware for the SARE autonomous cart control system. Built on the **STM32F401RCTx** microcontroller using a strict 3-layer architecture to ensure clean separation of concerns between application logic, hardware abstraction, and peripheral drivers.

---

## Target Hardware

| Property | Value |
|---|---|
| **MCU** | STM32F401RCTx (LQFP64) |
| **Flash** | 256 KB |
| **RAM** | 64 KB |
| **Clock** | Up to 84 MHz (HSI default) |
| **Toolchain** | STM32CubeIDE + GCC ARM |
| **HAL** | STM32F4xx HAL v1.x |

---

## Software Architecture

The firmware follows a strict **3-layer architecture**. Each layer may only communicate with the layer directly below it — never skip a layer or reference upward.

```
┌──────────────────────────────────────────┐
│             APP LAYER                    │
│         main.c / main.h                  │
│   (Application logic & entry point)      │
└───────────────────┬──────────────────────┘
                    │  ↓ calls ECUAL only
┌───────────────────▼──────────────────────┐
│            ECUAL LAYER                   │
│  ecual_init · SIM7600EH · HX711 · NFC    │
│  (External Component / Sensor Drivers)   │
└───────────────────┬──────────────────────┘
                    │  ↓ calls MCAL only
┌───────────────────▼──────────────────────┐
│            MCAL LAYER                    │
│          mcal_std_types.h                │
│  (MCU Abstraction + Type Definitions)    │
└───────────────────┬──────────────────────┘
                    │  ↓
┌───────────────────▼──────────────────────┐
│          HAL / CMSIS (Drivers/)          │
│     STM32F4xx_HAL_Driver + CMSIS         │
│         ⚠️  Do NOT modify                │
└──────────────────────────────────────────┘
```

---

## Dependency Audit

### Full Include Chain

```
main.c
  └── main.h
        └── ECUAL/ecual_init.h
              ├── ../MCAL/mcal_std_types.h
              │     ├── stm32f4xx_hal.h        (HAL)
              │     ├── <stdint.h>
              │     ├── <stdbool.h>
              │     └── <stddef.h>
              │
              ├── SIM7600EH/sim7600_port.h
              │     ├── ../../MCAL/mcal_std_types.h
              │     └── sim7600_port_cfg.h
              │
              ├── Hx711/hx711.h
              │     └── ../../MCAL/mcal_std_types.h
              │
              └── NFC/nfc.h
                    └── ../../MCAL/mcal_std_types.h
```

### Per-File Dependency Table

| File | Allowed Includes | Layer |
|---|---|---|
| `main.c` | `main.h` only | APP |
| `main.h` | `ECUAL/ecual_init.h` only | APP |
| `ecual_init.h` | `mcal_std_types.h` + sensor driver headers | ECUAL |
| `ecual_init.c` | `ecual_init.h` only | ECUAL |
| `sim7600_port.h` | `mcal_std_types.h` + `sim7600_port_cfg.h` | ECUAL |
| `sim7600_port.c` | `sim7600_port.h` only | ECUAL |
| `sim7600_port_cfg.h` | *(none — pure config macros)* | ECUAL |
| `hx711.h` | `mcal_std_types.h` only | ECUAL |
| `hx711.c` | `hx711.h` only | ECUAL |
| `nfc.h` | `mcal_std_types.h` only | ECUAL |
| `nfc.c` | `nfc.h` only | ECUAL |
| `mcal_std_types.h` | `stm32f4xx_hal.h` + std C headers | MCAL |

---

## Include Path Rules

### ✅ Allowed

```c
// APP layer (main.h)
#include "ECUAL/ecual_init.h"

// ECUAL layer — gateway header (ecual_init.h)
#include "../MCAL/mcal_std_types.h"
#include "SIM7600EH/sim7600_port.h"
#include "Hx711/hx711.h"
#include "NFC/nfc.h"

// ECUAL layer — driver headers (e.g. sim7600_port.h)
#include "../../MCAL/mcal_std_types.h"

// ECUAL layer — driver sources (e.g. sim7600_port.c)
#include "sim7600_port.h"   // own header only

// MCAL layer (mcal_std_types.h)
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
```

### ❌ Forbidden

```c
// ECUAL source/header directly including HAL — VIOLATION
#include "stm32f4xx_hal.h"   // in any ECUAL file

// APP layer bypassing ECUAL — VIOLATION
#include "MCAL/mcal_std_types.h"   // in main.c / main.h

// Upward / circular include — VIOLATION
#include "ECUAL/ecual_init.h"   // in any driver .c/.h file
```

### Relative Path Reference

| File Location | Path to `mcal_std_types.h` |
|---|---|
| `Core/Inc/ECUAL/ecual_init.h` | `"../MCAL/mcal_std_types.h"` |
| `Core/Inc/ECUAL/SIM7600EH/sim7600_port.h` | `"../../MCAL/mcal_std_types.h"` |
| `Core/Inc/ECUAL/Hx711/hx711.h` | `"../../MCAL/mcal_std_types.h"` |
| `Core/Inc/ECUAL/NFC/nfc.h` | `"../../MCAL/mcal_std_types.h"` |
| `Core/Src/ECUAL/ecual_init.c` | `"../../Inc/ECUAL/ecual_init.h"` |
| `Core/Src/ECUAL/SIM7600EH/sim7600_port.c` | *(via `sim7600_port.h`)* |
| `Core/Src/ECUAL/Hx711/hx711.c` | *(via `hx711.h`)* |
| `Core/Src/ECUAL/NFC/nfc.c` | *(via `nfc.h`)* |

---

## Project Structure

```
SARE-CART-CONTROL/
│
├── README.md
│
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32f4xx_hal_conf.h
│   │   ├── stm32f4xx_it.h
│   │   ├── ECUAL/
│   │   │   ├── ecual_init.h
│   │   │   ├── SIM7600EH/
│   │   │   │   ├── sim7600_port.h
│   │   │   │   └── sim7600_port_cfg.h
|   |   |   |   └── sim7600_gps.h
│   │   │   ├── Hx711/
│   │   │   │   └── hx711.h
│   │   │   └── NFC/
│   │   │       └── nfc.h
│   │   └── MCAL/
│   │       └── mcal_std_types.h
│   │
│   └── Src/
│       ├── main.c
│       ├── stm32f4xx_hal_msp.c
│       ├── stm32f4xx_it.c
│       ├── system_stm32f4xx.c
│       ├── syscalls.c
│       ├── sysmem.c
│       └── ECUAL/
│           ├── ecual_init.c
│           ├── SIM7600EH/
│           │   └── sim7600_port.c
│           ├── Hx711/
│           │   └── hx711.c
│           └── NFC/
│               └── nfc.c
│
└── Drivers/
    ├── CMSIS/
    └── STM32F4xx_HAL_Driver/
```

---

## Sensor Drivers

| Driver | Component | Interface | HAL Modules |
|---|---|---|---|
| **SIM7600EH** | GSM / GPS Module | UART — USART2 (PA2/PA3) | `HAL_UART_*` |
| **HX711** | Load Cell ADC (Weight) | GPIO 2-wire | `HAL_GPIO_*` |
| **NFC** | NFC Reader | SPI or I2C | `HAL_SPI_*` / `HAL_I2C_*` |

> **Note:** Driver source files are pending implementation. Header and folder structure is in place.

---

*SARE Embedded Systems Team — 2026*
