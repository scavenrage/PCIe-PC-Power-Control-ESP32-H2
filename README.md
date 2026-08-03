# PCIe PC Power Control — ESP32-H2

A PCIe x1 card that allows remote power control of a desktop PC via Zigbee (Home Assistant/ZHA), using an ESP32-H2 module. It draws power from the PCIe slot's 3.3Vaux rail (active even when the PC is off) and controls the motherboard's PWR_SW and RESET pins via two N-channel MOSFETs, simulating physical button presses. The card also senses the PC's real power state from the slot's standard (switched) 3.3V rail and reports it over Zigbee.

**Interactive schematic, PCB view, and BOM:** https://oshwlab.com/scavenrage/project_nexpziyw
**Firmware:** included in this repository (`main/`).

---

## Why

Full remote power management of a desktop PC, integrated into a broader voice-controlled smart home setup — no external relay module, no extra power cable, no visible hack. Just a card that lives inside the case.

## Features

- Power on/off the PC remotely via Home Assistant, with the switch state reflecting the PC's **real** power state (not just the command sent)
- Trigger a hardware reset remotely
- Passthrough connectors (H1/H3) keep the original case buttons fully functional in parallel
- Powered entirely from the PCIe 3.3Vaux rail (pin B10) — no additional power cables required
- Compatible with any motherboard with a free PCIe slot and standard front panel connector (JFP1 or equivalent)

## Specs

| | |
|---|---|
| MCU | ESP32-H2-MINI-1-N4 (Bluetooth LE 5 + IEEE 802.15.4/Zigbee — no WiFi radio) |
| Power supply | 3.3Vaux from PCIe slot (pin B10), active in S5 state |
| Outputs | 2× independent, N-channel MOSFET (SOT-23), PWR_SW (GPIO4) and RESET (GPIO5) |
| Sense input | GPIO1, PC real power state from the PCIe slot's standard (switched) 3.3V rail |
| Pulse duration | 300 ms (fixed in firmware, `main/config.h`) |
| PCB | 2-layer, 1.6 mm — ENIG (gold fingers) recommended, HASL works fine |
| Firmware | Dedicated Zigbee firmware (this repo), fixed 2-channel impulse relay + real-state sense, no configuration wizard |
| Form factor | PCIe x1 card |

## How it works — hardware

- **Power:** 3.3Vaux is taken from PCIe pin B10, staying powered in ATX standby (S5). This feeds the ESP32-H2 directly, no regulator needed.
- **Control outputs:** GPIO4 drives PWR_SW, GPIO5 drives RESET, each through an N-channel MOSFET with a series gate resistor and pull-down to GND (keeps the MOSFETs off during ESP32 boot). A HIGH pulse closes the contact to GND for 300 ms, simulating a button press.
- **Sense input:** GPIO1 reads the PCIe slot's **standard** 3.3V rail (main rail, present only when the motherboard is in S0/powered on — different from the 3.3Vaux on pin B10 that powers the ESP32 even in S5). This is how the firmware knows the PC's true state.
- **Passthrough:** H1 and H3 are identical 4-pin headers (PWR/GND/RES/GND) wired in parallel — one to the motherboard front panel header, one to the case's original buttons. Both work simultaneously.
- **Programming:** U9 is a 4-pin UART header (+3.3V/GND/TX/RX), bench-only, used for the first flash.

## Before you build

- **BIOS settings required:** `ErP Ready` → Disabled, `Power on by PCI-E` → Enabled
- Order the PCB at **1.6 mm** thickness. ENIG finish recommended for the gold-finger edge connector (durability), but HASL works equally well electrically.
- If using ENIG, remove the solder mask on both Top and Bottom layers over the gold-finger area.
- The H2 boot header is not populated — GPIO9 is handled directly during first programming via U9.
- Windows Fast Startup does **not** interfere — the card simulates a physical button press, not a network wake signal.

## Firmware

This repository ships its **own dedicated firmware** (`main/`), forked from `smart_switch` but simplified: no NVS configuration wizard, no `configure.py` step. All behavior is fixed in `main/config.h`:

- **EP1 "Power"** — a Zigbee ON or OFF command (either direction) triggers a single 300 ms pulse on GPIO4 (PWR_SW), exactly like a real ATX power button. This endpoint's On/Off attribute **never reflects the command**: it always reflects the **real PC state** read from GPIO1. If you turn the HA switch off but the PC doesn't actually power down, the switch goes back to showing "on" — it only changes when GPIO1 confirms a real state change.
- **EP2 "Reset"** — a Zigbee ON command triggers a 300 ms pulse on GPIO5 (RESET). OFF is ignored. The attribute goes on during the pulse and returns to off by itself at the end — never stuck on.
- **Real PC sense (GPIO1)** — sampled every 100 ms, considered stable after 2 s of a constant level (to ignore ATX power supply transients), and pushed to Zigbee immediately: at boot/join (no need to press anything first) and on every confirmed change.

### Assumptions to verify before the first flash

1. **Sense polarity**: assumes HIGH = PC on, LOW = PC off (`main/pc_sense.c`, function `pc_sense_read_raw()`). Verify with a multimeter against the schematic; if inverted, flip the comparison in that function.
2. **GPIO1 protection**: assumes adequate protection/current limiting is already present between the PCIe 3.3V rail and GPIO1, per this board's design. The firmware also enables the internal pull-down as a safety net for a deterministic LOW level on an undriven signal.
3. **Status LED (GPIO22)**: inherited from the `smart_switch` family. Check the schematic if you need visual boot/status feedback — the firmware still drives the pin harmlessly even if unconnected.

### Reporting Power/Reset over Zigbee — how it works (confirmed working on hardware)

EP1 "Power"'s On/Off attribute must reflect **only** the real state of the PC (`pc_sense`/GPIO1), never the ON/OFF command received over Zigbee. This required 3 fixes in `main/main.c`:

1. **Explicit Bind + Configure Reporting** (`setup_reporting()`, called on first confirmed coordinator reachability). `esp_zb_zcl_set_attribute_val(..., true)` alone does **not** send an unsolicited report — that boolean is `check_value_range` (min/max validation), not "send now". A binding + a reporting configuration for the attribute is required, otherwise locally-originated changes stay only in the local attribute table. Requires an On/Off cluster in **CLIENT** role in addition to SERVER on both endpoints (otherwise an `esp-idf#15962` assert on `direction=TO_CLI`).
2. **Explicit push after boot** (`reporting_ready_cb()`, ~500 ms after join). Without this, the initial state stayed invisible in HA until the PC genuinely changed state or the 5-minute keepalive fired.
3. **Deferred, non-synchronous re-assertion after a command** (`handle_on_off()` → `on_pc_state_changed(pc_sense_get_state())`). When a command arrives, ZBOSS writes the attribute to the command's value automatically before calling our handler. Writing the attribute again *synchronously, inside the same callback* gets silently overwritten once ZBOSS finishes. The fix is to queue the correction with `esp_zb_scheduler_alarm`, so it runs after ZBOSS has finished processing the current command.

`zb_report_on_off()` logs every write (value, `set_attribute_val` status, `report_attr_cmd_req` outcome, reporting-ready state) — useful for future diagnosis if this ever breaks again (e.g. after an OTA update or an esp-zigbee-lib version change).

### Reset as a "button" instead of a 2-state switch

Not achievable with standard firmware/ZCL alone: an On/Off cluster in server role (required so ZHA can command it) is always shown by ZHA as a 2-state **switch** — there's no native ZCL device type that ZHA recognizes as a stateless button. The only way is a custom **ZHA/zigpy quirk** (Python, Home Assistant side, not firmware) remapping the endpoint to the `button` platform — not implemented here. The fallback in place: the Reset attribute always returns to off by itself after the 300 ms pulse, never stuck on.

### Building and flashing

**Option A — building from source (requires ESP-IDF):**

```
idf.py set-target esp32h2
idf.py build
idf.py -p COMx flash
```

No NVS/configure.py step needed: the `nvs` partition stays empty/unused by this firmware.

**Option B — flashing the pre-built binaries (no ESP-IDF needed):**

For the very first flash (via the U9 UART header) you don't need the full ESP-IDF toolchain — just [esptool.py](https://github.com/espressif/esptool), a lightweight standalone tool:

```
pip install esptool
```

Then flash the 4 files in [`binaries/`](binaries/) at their fixed offsets in a single command:

```
esptool.py --chip esp32h2 --port COMx --baud 460800 write_flash --flash_mode dio --flash_freq 48m --flash_size 4MB ^
  0x0     binaries/bootloader.bin ^
  0x8000  binaries/partition-table.bin ^
  0x10000 binaries/ota_data_initial.bin ^
  0x20000 binaries/pcie_power_control.bin
```

(On Linux/macOS replace the trailing `^` line continuations with `\`.) Replace `COMx` with the actual serial port (e.g. `COM12` on Windows, `/dev/ttyUSB0` on Linux). No `nvs_config.bin` step is needed — unlike `smart_switch`, this firmware doesn't use the NVS partition at all.

> **Keeping `binaries/` in sync:** these 4 files are a snapshot of a specific build. After any firmware change, regenerate them from a fresh `idf.py build`:
> ```
> cp build/bootloader/bootloader.bin      binaries/bootloader.bin
> cp build/partition_table/partition-table.bin  binaries/partition-table.bin
> cp build/ota_data_initial.bin           binaries/ota_data_initial.bin
> cp build/pcie_power_control.bin         binaries/pcie_power_control.bin
> ```

### OTA

`OTA_IMAGE_TYPE = 0x0105` in `main/ota.h', so an OTA update meant for one firmware can't be offered/accepted by the other. Initial version: v1.0.0. Note the `.ota` file (built with `make_ota.py`, see the repository root's tooling) is only for **over-the-air updates of a device already joined to the Zigbee network** — it is not usable for the first flash; use Option A or B above for that.

## Bill of Materials

Full BOM available in [`hardware/BOM.xlsx`](hardware/BOM.xlsx) in this repo, and in the [OSHWLab project](https://oshwlab.com/scavenrage/project_nexpziyw). Gerbers for PCB fabrication: [`hardware/Gerber_PCB.zip`](hardware/Gerber_PCB.zip). Schematic: [`hardware/Schematic.png`](hardware/Schematic.png).

## Assembly

1. Solder the ESP32-H2-MINI module
2. Solder SMD components (resistors, capacitors, MOSFETs, LED) — reflow or hot air
3. Solder the U9 UART header (bench use only, can be removed after first flash)
4. Flash the firmware via U9 — either build from source or flash the ready-made files in [`binaries/`](binaries/) with `esptool.py` (see [Building and flashing](#building-and-flashing) above)
5. Solder H1 and H3 right-angle headers
6. Insert the card into a free PCIe x1 slot
7. Connect H1 → motherboard front panel header (PWR/GND/RES/GND)
8. Connect H3 → the case's original power and reset buttons
9. Set BIOS: `ErP Ready` OFF, `Power on by PCI-E` ON
10. Add the device in Home Assistant via ZHA

## P.S.
photos are from rev.1 without status reading from A10 (not-always-present 3,3V)

## License

Hardware released under [CERN-OHL-P-2.0](https://ohwr.org/cern_ohl_p_v2.txt), matching the OSHWLab project license.
