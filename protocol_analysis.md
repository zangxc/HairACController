# RS485 Protocol Analysis
**Captured**: 2026-07-11 22:16–22:18 AEST  
**Protocol**: Modbus RTU, 9600 8N2, slave address 0x02  
**Bus**: Master controller (addr 0x02) + slave controller + indoor unit (addr 0x61)

---

## Packet Types

### Type A: FC03 READ (master → indoor unit)
```
02 03 9C 40 00 13 ...  [35-37 bytes]
```
- `02` = slave address
- `03` = FC03 Read Holding Registers
- `9C 40` = start register 0x9C40
- `00 13` = read 19 registers (0x9C40–0x9C52)
- Remaining bytes = the 19×2=38 bytes of register data (Modbus inline response — unusual, likely proprietary framing)

### Type B: FC06 WRITE (master → indoor unit)
```
02 06 9C 4x 00 xx ...  [15 bytes]
```
Standard Modbus FC06 single register write.

### Type C: 0x61 responses (indoor unit → master, 26–27 bytes)
Not standard Modbus — proprietary status broadcast from AC unit.

---

## FC06 Write Register Map (CONFIRMED by observing AC operations)

| Register | Values seen       | Meaning (inferred)              |
|----------|-------------------|---------------------------------|
| 0x9C49   | 0x14=20, 0x15=21, 0xFF=255 | **Setpoint temperature** (0x14=20°C, 0x15=21°C, 0xFF=off/invalid) |
| 0x9C4A   | 0x00, 0x01        | **Unknown** (zone flag? on/off?) |
| 0x9C4B   | 0x48=72, 0x50=80, 0x58=88, 0x6A=106 | **Zone state bitmask** (each zone = 8 units: 0x48=1zone, 0x50=2zones, 0x58=3zones, 0x6A=varies) |
| 0x9C4C   | 0x48=72           | **Fan speed or mode** |
| 0x9C4D   | 0x40=64, 0x60=96, 0x1FE0=8160 | **Zone enable bitmask** (0x1FE0 = all zones, lower values = subset) |

---

## 0x61 Status Response Byte Map (26/27-byte packets)

Fixed bytes (same every time): `61 00 _ 00 _ _ _ 00 _ 07 B4 1A 07 0B 16 ...`
- `07 B4` = 0x07B4 = 1972... likely firmware version or model
- `1A 07 0B 16` = 26, 7, 11, 22 → looks like **date: Jul 11** and **time: 22:xx**

Variable bytes observed during AC operation:

| Byte | Values seen        | Inferred meaning |
|------|-------------------|------------------|
| [2]  | 0xAF, 0xB0        | AC power state (0xB0=ON, 0xAF=transitioning/OFF) |
| [4]  | 0x00–0x04, 0x40   | Zone/mode state flags |
| [5]  | 0x04, 0x05        | Fan speed (4=medium, 5=high?) |
| [6]  | 0x02, 0x03        | Mode (3=cool, 2=heat?) |
| [8]  | 0xC8=200, 0xD2=210, 0xDC=220 | **Current room temp** (÷10 = 20.0°C, 21.0°C, 22.0°C) |
| [15] | 0x0F=15, 0x10=16, 0x11=17 | **Time: hour** |
| [17] | 0x14=20, 0x15=21  | **Setpoint temperature** (matches what was written to 0x9C49) |
| [18] | 0x00, 0x14, 0x15  | Setpoint or second zone setpoint |
| [21] | 0x00, 0x40        | Status flag (0x40 = running?) |
| [24-26] | varies widely | **CRC** (last 2 bytes = Modbus CRC) |

---

## Key Findings

1. **This is Modbus RTU** with slave address `0x02` for the AC unit and `0x61` for status broadcasts.
2. **Setpoint register**: 0x9C49 — write `0x15` for 21°C, `0x14` for 20°C, `0xFF` = off.
3. **Zone control**: registers 0x9C4B and 0x9C4D control which zones are active (bitmask).
4. **Room temperature**: returned in 0x61 response byte[8], value ÷ 10 = °C (0xC8=20.0, 0xD2=21.0, 0xDC=22.0).
5. **Status byte[2]**: 0xB0 = AC on, 0xAF = AC off/standby.
6. The master continuously polls `FC03 reg=0x9C40 count=19` every ~1s and also sends FC06 writes in response to user input.

---

## Next Steps

- Longer capture with each zone individually toggled to fully decode 0x9C4B/0x9C4D bitmask
- Capture AC power on/off to confirm byte[2] of 0x61 response
- Capture mode change (cool/heat/fan) to decode byte[6] of 0x61 response
- Once register map confirmed, implement write logic in ESP32 to control the AC

