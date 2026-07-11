# RS485 Protocol Analysis — CONFIRMED
**Protocol**: Modbus RTU, 9600 baud, **8N2** (no parity, 2 stop bits)  
**GPIO**: RX=pin 43, TX=pin 44 (per Waveshare schematic)  
**Slave address**: `0x02` = AC indoor unit, `0x61` = AC status broadcast

---

## Register Map (FC06 Write to control AC)

| Register | Meaning | Values |
|----------|---------|--------|
| **0x9C49** | **Zone bitmask** | bit0=UpBeds, bit1=Rumpus, bit2=Dining, bit3=Office, bit4=Living. `0x00`=all off, `0x1F`=all on, `0xFF`=power off |
| **0x9C4A** | Power on/off | `0x01`=on, `0x00`=off |
| **0x9C4B** | Zone damper state | Companion to 0x9C4D — written together (exact bit map TBD) |
| **0x9C4C** | Fan/mode config | `0x48` seen always — may encode mode+fan in nibbles |
| **0x9C4D** | Zone enable flags | `0x40`, `0x60`, `0x80` — companion to 0x9C4B |

### Zone bitmask (0x9C49) — FULLY CONFIRMED
```
bit 0 = 0x01 = Zone 1: Up Beds
bit 1 = 0x02 = Zone 2: Rumpus
bit 2 = 0x04 = Zone 3: Dining
bit 3 = 0x08 = Zone 4: Office
bit 4 = 0x10 = Zone 5: Living
```
Verified sequence from capture:
- `0x00` → all zones off
- `0x01` → Up Beds only
- `0x03` → Up Beds + Rumpus
- `0x07` → Up Beds + Rumpus + Dining
- `0x0F` → Up Beds + Rumpus + Dining + Office
- `0x1F` → all 5 zones
- `0xFF` → power off state

### To turn on/control AC:
1. Write `0x9C4A = 0x0001` (power on)
2. Write `0x9C49 = <zone_bitmask>` (set active zones)
3. Write `0x9C4C = 0x0048` (keep fan/mode — always written with config)
4. Write `0x9C4B` + `0x9C4D` (damper states — mirror what master sends, TBD)

---

## Status Response — 0x61 Broadcast (27 bytes)

The AC unit broadcasts status every ~1s regardless of polling.

| Byte | Meaning | Values |
|------|---------|--------|
| [0]  | Device ID | `0x61` always |
| [2]  | **Power state** | `0xB0`=ON, `0xAF`=OFF/standby |
| [5]  | **Fan speed** | `0x01`=low?, `0x03`=med?, `0x04`=auto?, `0x05`=high? |
| [6]  | **Mode** | `0x02`=Heat, `0x03`=Cool, `0x04`=Fan/Dry (TBC) |
| [8]  | **Room temp** | value ÷ 10 = °C (e.g. `0xD2`=21.0°C, `0xDC`=22.0°C) |
| [9–14] | Date/time | `D2 07 0B 1A 07 10` = year(07D2=2002?), month, day, hour, min |
| [17] | **Setpoint echo** | Mirrors last written setpoint temp in °C |
| [21] | Status flag | `0x40`=running, `0x00`=idle |
| [25–26] | CRC | Modbus CRC16 |

**Note**: The 26-byte variant is a second device (slave controller) responding. Same format, byte[4] varies with zone state.

---

## Temperature Encoding (0x9C49 / setpoint)

The setpoint register value equals °C directly:
- `0x14` = 20°C
- `0x15` = 21°C  
- `0x16` = 22°C etc.

**Important**: During zone-only changes, the same register is also used as a zone bitmask. Context determines meaning — when 0x9C4A is written first (power), it switches to zone bitmask mode. The master writes these registers in sequence to form a complete command.

---

## What still needs a capture

- [ ] Mode change: need 0x9C4C values written when switching heat→cool→fan→dry
- [ ] Fan speed: need dedicated fan-only capture to decode 0x9C4C nibbles
- [ ] Temp setpoint: confirm temp is written to a separate register (may be 0x9C48 or combined in 0x9C4C)
- [ ] Full 0x9C4B/4D bitmask per individual zone state

---

## Captures

| File | Contents |
|------|----------|
| `capture_20260711_221612.log` | First 2-min session: power on, temp change, some zones |
| `capture_zones_modes_20260711_222921.log` | Zone-by-zone sequence + mode cycling |
