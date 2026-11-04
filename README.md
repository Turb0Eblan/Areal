# Areal by BI

Firmware for the control system that drives the lifting mechanisms and lighting of the **Areal by BI** architectural building model. Built on ESP32 (uPesy WROOM) with PlatformIO and the Arduino framework.

## Architecture

```
              USB Serial / Bluetooth ("Areal_Master")
                              │
                       ┌──────▼──────┐
                       │   Master    │
                       └──────┬──────┘
                              │  ESP-NOW (WiFi channel 1, 3 attempts + ACK)
      ┌──────────────┬────────┼─────────┬────────────────┐
      ▼              ▼        ▼         ▼                ▼
   m2-1           m2-2       m4      m1-1 / m1-2     LED slaves
 (left,         (right,    (center,  (spare,         (blue/red/
  2 motors)      2 motors)  5 motors) 1 motor)        yellow/orange)
```

A single **Master** receives commands and forwards them to the right **Slave** devices over ESP-NOW. Each Slave keeps its settings (steps, trim, speed, acceleration) in NVS, so they survive a power cycle.

## Repository layout

| Folder | Purpose |
|---|---|
| `Areal Msater/` | Master controller — receives commands and routes them over ESP-NOW |
| `Areal motor 2 Slave/` | `m2-1` — left mechanism, 2 synchronized stepper motors (TMC2209) |
| `Areal motor 2 Slave copy/` | `m2-2` — right mechanism, same as `m2-1` |
| `Areal motor 4 Slave/` | `m4` — center vertical stack of 5 motors (TB6600 + TMC2209), SIM/SEQ modes |
| `Areal motor 1 Slave/` | `m1-1`, `m1-2` — single motor with homing (spare) |
| `Areal slave/` | LED controller: NeoPixel GRBW strips, effects and window simulation (⚠️ not integrated yet) |
| `3d models/` | STL/DXF/DWG models: staircases, escalator, fasteners, end caps, light boxes, logo |
| `Cuts/` | Laser-cutting layouts (CorelDRAW) — power supply enclosures |
| `documentation.txt` | Full technical documentation: pins, states, all commands |
| `commands.txt` | Quick command cheat sheet |

## Building and flashing

You need [PlatformIO](https://platformio.org/) (CLI or the VS Code extension). The easiest way is to open `Areal by BI.code-workspace`, which includes all projects.

```bash
cd "Areal motor 4 Slave"
pio run -t upload        # build and flash
pio device monitor       # serial monitor, 115200 baud
```

Dependencies are pulled automatically from `platformio.ini`:
- motors — `TMCStepper`, `AccelStepper`
- LED — `Adafruit NeoPixel`, `FastLED`, `NeoPixelBus`

## Master commands

Sent over USB Serial (115200) or Bluetooth `Areal_Master`:

| Command | Action |
|---|---|
| `m2-1 up` / `m2-1 down` | left mechanism: raise / lower |
| `m2-2 up` / `m2-2 down` | right mechanism: raise / lower |
| `m4 up` / `m4 down` | center mechanism: raise / lower |
| `motors up` / `motors down` | all mechanisms at once |
| `<target> stop` | stop |
| `<target> mode:sim` / `mode:seq` | simultaneous / sequential movement |
| `lube <target> [cycles] [waitMs]` | lube mode: repeated down/up cycles |

Each Slave also accepts its own commands directly over Serial/Bluetooth — `HOME`, `POS`, `SPD:<N>`, `ACC:<N>`, `TRIM1:<N>`, `RESET` and more. Type `i` or `help` on any device for the full list, or see [`documentation.txt`](documentation.txt).

## Status

| Device | Motors | Status |
|---|---|---|
| Master | — | ✅ working |
| m2-1, m2-2 | 2 + 2 | ✅ working |
| m4 | 5 | ✅ working |
| m1-1, m1-2 | 1 + 1 | 💤 spare |
| LED (×4) | — | ⚠️ firmware exists, control from the Master is not implemented |
