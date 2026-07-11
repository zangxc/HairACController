# AC Controller — ESP32-S3 LVGL UI + RS485 Sniffer

## Hardware

**Board**: Waveshare ESP32-S3-Touch-LCD-4.3B (ESP32-S3-N16R8, 800×480 RGB LCD, capacitive touch)  
**Flash**: 16MB, **PSRAM**: 8MB OPI  
**USB**: built-in JTAG/CDC (VID 0x303A PID 0x1001), no separate UART chip on USB

### RS485 (onboard transceiver, auto DE/RE — no manual control pin)
| GPIO | RS485 pin | Direction |
|------|-----------|-----------|
| 43   | RXD       | Input  ← data in |
| 44   | TXD       | Output → data out |

**Bus**: 3 devices + this board. ~40m total cable. 1× master controller (far end), 1× slave controller, 1× indoor AC unit (middle). All communicate at **9600 baud** (confirmed by logic analyser: 104µs bit period). Traffic pattern: burst <100ms, idle ~900ms, repeat ~1/s.

### Other occupied pins (do NOT use)
| GPIO | Function |
|------|----------|
| 4    | Touch INT |
| 5    | LCD RGB DE |
| 0,1,2,3,7,10,14,17,18,21,38–42,45–48 | LCD RGB data/sync |
| 8,9  | I2C (touch + RTC) |
| 11,12,13 | SPI (TF card) |
| 15,16 | CAN TX/RX |
| 19,20 | USB D-/D+ |

---

## Build & Flash

**Arduino-CLI path**: `/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli`

### Compile
```bash
ARDUINO_CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
FQBN="esp32:esp32:waveshare_esp32_s3_touch_lcd_5B:PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,UploadSpeed=921600,USBMode=hwcdc,UploadMode=default,CDCOnBoot=cdc"
"$ARDUINO_CLI" compile --fqbn "$FQBN" --build-path /private/tmp/arduino-build-09-lvgl \
  /Users/zangxc/Documents/Arduino/09_lvgl_Porting_copy
```

### Flash
```bash
# UploadMode must be "cdc" (not "default") for this board's JTAG/CDC USB
PORT=$(ls /dev/cu.usbmodem* | head -1)
lsof "$PORT" | awk 'NR>1{print $2}' | xargs kill 2>/dev/null; sleep 1
FQBN_U="esp32:esp32:waveshare_esp32_s3_touch_lcd_5B:PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,UploadSpeed=921600,USBMode=hwcdc,UploadMode=cdc,CDCOnBoot=cdc"
"$ARDUINO_CLI" upload -p "$PORT" --fqbn "$FQBN_U" \
  --input-dir /private/tmp/arduino-build-09-lvgl \
  /Users/zangxc/Documents/Arduino/09_lvgl_Porting_copy
```

**Key flags**:
- `CDCOnBoot=cdc` — required for `Serial.print` to appear on USB port
- `UploadMode=cdc` — required to flash; `UploadMode=default` fails with "No serial data received"
- `PSRAM=enabled` — OPI PSRAM, required for frame buffers
- `PartitionScheme=app3M_fat9M_16MB` — 16MB flash layout

### Read serial output
```bash
PORT=$(ls /dev/cu.usbmodem* | head -1)
lsof "$PORT" | awk 'NR>1{print $2}' | xargs kill 2>/dev/null
stty -f "$PORT" 115200 cs8 -cstopb -parenb raw -echo
timeout 30 dd if="$PORT" bs=1 count=10000 | strings
```
The port number (e.g. `14201` vs `14301`) changes between flash sessions. Always use `ls /dev/cu.usbmodem*` to find it. If a process holds the port, kill it first (Arduino IDE serial monitor is the usual culprit).

---

## Codebase Structure

Single sketch file: `09_lvgl_Porting_copy.ino`

| Section | Description |
|---------|-------------|
| §1 | RS485 pin/config defines (`RS485_RX_PIN`, `RS485_INVERTED`) |
| §2 | Config table: 6 × 9600-baud parity/stop combos |
| §3 | Shared volatile state struct (`g_sniffer`) written by tasks, read by LVGL timer |
| §4–5 | AC state variables + LVGL widget pointers |
| §6–8 | UI event callbacks + zone/panel refresh |
| §9 | LVGL 500ms timer → updates sniffer status bar label |
| §10 | `create_air_con_dashboard()` — builds full LVGL UI |
| §11 | Packet validation heuristics (XOR, arith sum, length+uniqueness) |
| §12 | `TaskRawSampler` — GPIO 100kHz edge-triggered sampler (Core 1, priority 5) |
| §13 | `TaskRS485Sniffer` — UART scanner cycling 6 configs (Core 1, priority 3) |
| §14–15 | `setup()` + `loop()` |

**LVGL port**: `lvgl_v8_port.cpp/.h` — handles LCD + touch init, mutex, timer task on Core 0.

---

## RS485 Sniffer Design

### Two parallel tasks on Core 1
1. **Raw GPIO sampler** (priority 5): edge-triggered, 100kHz, 60ms capture window. Decodes bit period and soft-decodes UART frames, printing which format (8N1/8E1/etc.) matches each byte.
2. **UART scanner** (priority 3): `Serial1` cycling through 6 configs at 9600 baud (8N1/8N2/8E1/8E2/8O1/8O2), 8s per slot. Pauses while sampler holds the pin.

### On-screen status bar
Small label directly below the 5 zone buttons in the right panel. Updated every 500ms by LVGL timer. Format: `UART:8E1 V:3 I:1 | RAW:104us(9615baud) #12`

### Trigger logic (important)
Sampler waits for **first falling edge after ≥100ms of silence** — ensures capture starts at true packet start, not mid-packet. Bus is idle ~900ms between bursts so this reliably catches clean packet starts.

### Polarity
`RS485_INVERTED 0` = normal (idle HIGH). If output shows `L60000` (line stuck LOW), either A/B wires swapped or wrong pin. **Do not set INVERTED=1 as a workaround** — fix the wiring.

### Packet validation heuristics (in order)
1. XOR of all bytes == 0x00
2. XOR of bytes[0..n-2] == bytes[n-1]  
3. Arithmetic sum (low byte) of bytes[0..n-2] == bytes[n-1]
4. Same + 1 (Mitsubishi variant)
5. Length 4–48 bytes AND 2–20 unique byte values

---

## Known Issues / History

- **Pins were originally reversed** (RX=44, TX=43). Fixed 2026-07-11 after checking Waveshare schematic. Always verify against official docs.
- **`UploadMode=default` fails** with "No serial data received" at baud change. Use `UploadMode=cdc`.
- **`CDCOnBoot=default`** (disabled) means Serial goes to UART0 pins, not USB. Always use `CDCOnBoot=cdc`.
- **Gap timer was 5ms** initially — too short, framed every byte as its own packet. Now 30ms.
- **40m cable** causes signal reflections that make raw sampler mis-estimate bit period on short bursts. The "first edge after 100ms silence" trigger mitigates this.

---

## Libraries Required
- `ESP32_Display_Panel` (v0.1.4+) — online or offline install
- `lvgl` v8.4.0 — **offline install only** (custom `lv_conf.h` in sketch)
- `ESP32_IO_Expander` (v0.0.4+)

Arduino-ESP32 package: `esp32 by Espressif Systems` v3.0.7 (offline package available from Waveshare).
