# Haier Ducted AC RS485 Protocol — Reverse Engineered
**Date**: 2026-07-11  
**Unit**: Haier ducted split system with zone controller  
**Protocol**: Modbus RTU framing, 9600 baud, **8N2** (8 data bits, no parity, 2 stop bits)  
**Bus wiring**: GPIO43=RX, GPIO44=TX on Waveshare ESP32-S3-Touch-LCD-4.3B  
**Cable**: ~40m RS485, 3 devices (master controller, slave controller, indoor unit)

---

## Does this match a known open-source protocol?

**No exact match found.** This is Haier's proprietary ducted/zoned AC wired controller protocol.  
It is **NOT** the same as:
- Haier WiFi hOn/SmartAir2 protocol (ESPHome `climate: haier` — that's a UART WiFi dongle protocol)
- Haier IR remote protocol (IRremoteESP8266)
- Daikin Altherma Modbus (uses register base 0xAFA5, function 0x03/0x40)
- Samsung HVAC buscontrol (2400 baud 8E1, 14-byte frames)

The closest structural match is **Daikin proprietary Modbus** — both use standard Modbus framing (RTU CRC, FC03/FC06) but with vendor-specific register maps and non-standard response function codes (our FC 0x20 response). The `0x9C40` register base is unique to this Haier system.

---

## Bus Topology

```
[Master Controller] ──┐
[Slave Controller]  ──┤── RS485 bus (A/B) ── [Indoor AC Unit addr=0x02]
[This ESP32 board]  ──┘                      [broadcasts status as addr=0x61]
```

- **Master (addr 0x02)**: sends FC03 read requests and FC06 write commands to the indoor unit
- **Indoor unit**: responds to 0x02 queries AND broadcasts status frames with source addr 0x61  
- **Slave controller**: also on bus, echoes some frames (26-byte 0x61 variant)
- **Traffic pattern**: master polls every ~1s, burst <100ms then ~900ms idle

---

## Frame Structure

### Each "long frame" in captures is actually TWO back-to-back frames merged by our 30ms gap timer.
The request and response arrive within ~10ms of each other.

### FC03 Read Request (8 bytes)
```
02 03 9C 40 00 13 CRC_L CRC_H
│  │  ├──────┘ └──────┘
│  │  reg=0x9C40  count=19 regs
│  FC03
addr=0x02
```

### FC20 Read Response (27 bytes) — Haier proprietary
```
02 20 [24 data bytes] CRC_L CRC_H
│  │
│  FC=0x20 (Haier proprietary — not standard Modbus)
addr=0x02
```

### FC06 Write Request (8 bytes) — standard Modbus
```
02 06 REG_H REG_L VAL_H VAL_L CRC_L CRC_H
```

### FC06 Write ACK (7 bytes) — Haier proprietary
```
02 10 [5 data bytes]
│  │
│  FC=0x10 (Haier ACK — looks like FC16 WriteMultiple but only 7 bytes)
addr=0x02
```

---

## Register Map (FC06 Writes to Control AC)

| Register | Name | Values | Notes |
|----------|------|--------|-------|
| **0x9C49** | **Zone bitmask** | see below | CONFIRMED — primary control register |
| **0x9C4A** | **Power on/off** | `0x0001`=ON, `0x0000`=OFF | CONFIRMED |
| **0x9C4B** | Zone damper state | `0x48`, `0x50`, `0x58`, `0x6A`, `0x82`, `0xA6` | companion to 0x9C4D, exact meaning TBD |
| **0x9C4C** | Fan / mode config | `0x0048` always seen | upper nibble=mode? lower=fan? needs capture |
| **0x9C4D** | Zone enable flags | `0x0040`, `0x0060`, `0x0080` | companion to 0x9C4B |

### Zone Bitmask — Register 0x9C49 — FULLY CONFIRMED

```
Bit 0 (0x01) = Zone 1: Up Beds
Bit 1 (0x02) = Zone 2: Rumpus  
Bit 2 (0x04) = Zone 3: Dining
Bit 3 (0x08) = Zone 4: Office
Bit 4 (0x10) = Zone 5: Living
```

Verified from sequential zone-on capture:
| Written value | Active zones |
|--------------|--------------|
| `0x00` | All zones off |
| `0x01` | Up Beds only |
| `0x03` | Up Beds + Rumpus |
| `0x07` | Up Beds + Rumpus + Dining |
| `0x0F` | Up Beds + Rumpus + Dining + Office |
| `0x1F` | All 5 zones |
| `0xFF` | Power-off state (written when AC turned off) |

### To send a complete zone/temp command, master writes these in sequence:
```
FC06 write 0x9C4A = 0x0001   (ensure power on)
FC06 write 0x9C4B = <damper> (zone damper state)
FC06 write 0x9C4C = 0x0048   (fan/mode — keep constant until decoded)
FC06 write 0x9C49 = <zones>  (zone bitmask)
FC06 write 0x9C4D = <enable> (zone enable flags)
```

---

## Status Broadcast — Address 0x61 (Indoor Unit)

The indoor unit broadcasts status every ~1s regardless of master polling.  
Two variants: 27-byte (indoor unit) and 26-byte (slave controller echo).

### 27-byte variant (primary indoor unit status)

```
61 00 [power] 00 [zones] [fan] [mode] 00 [temp] [year_H] [year_L] [month] [day] [hour] [min] [sec] [setpt] [setpt2] ... [flag] ... CRC_L CRC_H
```

| Byte | Field | Values | Notes |
|------|-------|--------|-------|
| [0] | Device ID | `0x61` | always |
| [2] | **Power state** | `0xB0`=ON, `0xAF`=OFF/standby | CONFIRMED |
| [4] | Zone/status flags | varies | changes with zone activity |
| [5] | **Fan speed** | `0x01`=low, `0x03`=med, `0x04`=auto, `0x05`=high | tentative |
| [6] | **Mode** | `0x02`=Heat, `0x03`=Cool, `0x04`=Fan/Dry | tentative — needs dedicated capture |
| [8] | **Room temp** | raw ÷ 10 = °C | `0xD2`=21.0°C, `0xDC`=22.0°C CONFIRMED |
| [9–10] | Year | `0x07 0xB4` = ? | may be model/firmware |
| [11] | Month | `0x1A`=26? | possibly in different format |
| [12] | Day | `0x07` | |
| [13] | Hour | `0x0B`=11 | |
| [14] | Hour (again?) | `0x16`=22 | likely actual hour |
| [15] | Minute | `0x0F`=15, `0x10`=16, `0x11`=17 | CONFIRMED matches time |
| [17] | **Setpoint** | direct °C value | `0x15`=21°C CONFIRMED — echoes what was written |
| [21] | Status flag | `0x40`=running, `0x00`=idle | |
| [25–26] | CRC | Modbus CRC16 | |

### Temperature encoding
- Room temp: `byte[8]` ÷ 10 = °C (e.g. `0xD2`=210÷10=21.0°C)
- Setpoint: `byte[17]` = direct °C integer (e.g. `0x15`=21°C)

---

## FC20 Response Data Block (24 bytes from master's read response)

The master's FC03 read of registers 0x9C40–0x9C52 returns 24 data bytes via FC 0x20.  
This contains the full AC state as seen by the master. Key variable bytes:

| Data byte | Content | Notes |
|-----------|---------|-------|
| [6] | Zone bitmask (current) | matches 0x9C49 writes |
| [13] | Zone/mode config | includes 0x45 baseline |
| [15] | Zone damper value | matches 0x9C4B writes |
| [17–18] | Zone enable | matches 0x9C4D writes |

---

## Known Unknowns

| What | Status | How to decode |
|------|--------|---------------|
| Mode values (cool/heat/fan/dry) | tentative | Dedicated capture: turn on, cycle modes, watch 0x61 byte[6] |
| Fan speed values | tentative | Dedicated capture: cycle fan speed only |
| Setpoint write register | partial | 0x9C49 used for zones AND possibly temp in different context |
| 0x9C4B/4D exact bitmask | unclear | Write each zone individually and record both values |
| FC20 response full decode | partial | Need to correlate each data byte with known state |

---

## Captures

| File | Contents |
|------|----------|
| `capture_20260711_221612.log` | First session: power on/off, temp change, zones |
| `capture_zones_modes_20260711_222921.log` | Sequential zone-on (all 5) + mode cycling |

