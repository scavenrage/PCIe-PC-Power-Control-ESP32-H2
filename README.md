# PCIe PC Power Control — ESP32-H2

A PCIe x1 card that allows remote power control of a desktop PC via WiFi/Zigbee, using an ESP32-H2 module. It draws power from the PCIe slot's 3.3Vaux rail (active even when the PC is off) and controls the motherboard's PWR_SW and RESET pins via two N-channel MOSFETs, simulating physical button presses.

**Interactive schematic, PCB view, and BOM:** https://oshwlab.com/scavenrage/project_nexpziyw
**Firmware (shared with the ESP32-H2 Zigbee relay family):** https://github.com/scavenrage/ESP32-H2-Custom-Zigbee

---

## Why

Full remote power management of a desktop PC, integrated into a broader voice-controlled smart home setup — no external relay module, no extra power cable, no visible hack. Just a card that lives inside the case.

## Features

- Power on/off the PC remotely via Home Assistant
- Trigger a hardware reset remotely
- Passthrough connectors (H1/H3) keep the original case buttons fully functional in parallel
- Powered entirely from the PCIe 3.3Vaux rail (pin B10) — no additional power cables required
- Compatible with any motherboard with a free PCIe slot and standard front panel connector (JFP1 or equivalent)

## Specs

| | |
|---|---|
| MCU | ESP32-H2-MINI-1-N4 (WiFi + Bluetooth + Zigbee) |
| Power supply | 3.3Vaux from PCIe slot (pin B10), active in S5 state |
| Outputs | 2× independent, N-channel MOSFET (SOT-23), PWR_SW and RESET |
| Pulse duration | 300 ms (configurable in firmware) |
| PCB | 2-layer, 1.6 mm — ENIG (gold fingers) recommended, HASL works fine |
| Firmware | ESPHome-compatible custom Zigbee firmware, `smart_switch` config, impulse relay on CH1/CH2 |
| Form factor | PCIe x1 card |

## How it works

- **Power:** 3.3Vaux is taken from PCIe pin B10, staying powered in ATX standby (S5). This feeds the ESP32-H2 directly, no regulator needed.
- **Control outputs:** GPIO4 drives PWR_SW, GPIO5 drives RESET, each through an N-channel MOSFET with a series gate resistor and pull-down to GND (keeps the MOSFETs off during ESP32 boot). A HIGH pulse closes the contact to GND for 300 ms, simulating a button press.
- **Passthrough:** H1 and H3 are identical 4-pin headers (PWR/GND/RES/GND) wired in parallel — one to the motherboard front panel header, one to the case's original buttons. Both work simultaneously.
- **Programming:** U9 is a 4-pin UART header (+3.3V/GND/TX/RX), bench-only, used for the first flash.

## Before you build

- **BIOS settings required:** `ErP Ready` → Disabled, `Power on by PCI-E` → Enabled
- Order the PCB at **1.6 mm** thickness. ENIG finish recommended for the gold-finger edge connector (durability), but HASL works equally well electrically.
- If using ENIG, remove the solder mask on both Top and Bottom layers over the gold-finger area.
- The H2 boot header is not populated — GPIO9 is handled directly during first programming via U9.
- Windows Fast Startup does **not** interfere — the card simulates a physical button press, not a network wake signal.

## ⚠️ Critical: firmware configuration

The firmware, as distributed, defaults to **a single channel configured as a stable relay** if you skip `configure.py` and flash the pre-built binaries as-is — this default is **not compatible** with this board and can hold the outputs active indefinitely. Before flashing, you **must** run the `configure.py` wizard from the [firmware repo](https://github.com/scavenrage/ESP32-H2-Custom-Zigbee) and select **2 channels**, both set to **"impulse relay"** (CH1 = PWR_SW, CH2 = RESET). If this step is skipped or a different channel type is chosen, the GPIO output stays permanently HIGH instead of pulsing for 300 ms — holding PWR_SW or RESET continuously active, preventing the machine from powering on or keeping it stuck in reset. Follow the firmware repo's instructions exactly and generate your own `nvs_config.bin` before first flash.

## Assembly

1. Solder SMD components (resistors, capacitors, MOSFETs, LED) — reflow or hot air
2. Solder the ESP32-H2-MINI module
3. Solder the U9 UART header (bench use only, can be removed after first flash)
4. Flash the firmware via U9 (see the [firmware repo](https://github.com/scavenrage/ESP32-H2-Custom-Zigbee) for `configure.py` + flashing instructions — configure as 2× impulse relay, CH1/CH2)
5. Solder H1 and H3 right-angle headers
6. Insert the card into a free PCIe x1 slot
7. Connect H1 → motherboard front panel header (PWR/GND/RES/GND)
8. Connect H3 → the case's original power and reset buttons
9. Set BIOS: `ErP Ready` OFF, `Power on by PCI-E` ON
10. Add the device in Home Assistant via ZHA

## Bill of Materials

| Designator | Part | Qty | Supplier Part # |
|---|---|---|---|
| U3 | ESP32-H2-MINI-1-N4 | 1 | LCSC C18203134 |
| R1, R2, R5, R19, R6 | 10 kΩ 0603 | 5 | LCSC C25804 |
| C1 | 100 nF 0603 | 1 | LCSC C30926 |
| U9 | 4-pin header (UART, bench only) | 1 | LCSC C2935898 |
| H2 | 2-pin header, right-angle | 1 | — |

*Full BOM with all passive/MOSFET values available in the [OSHWLab project](https://oshwlab.com/scavenrage/project_nexpziyw) and in `BOM.csv` in this repo.*

## License

Hardware released under [CERN-OHL-P-2.0](https://ohwr.org/cern_ohl_p_v2.txt), matching the OSHWLab project license.
