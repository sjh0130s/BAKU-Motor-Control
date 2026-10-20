# Final PCB schematic and firmware review (2026-10-20)

Based on user-supplied `SCH_Schematic1_2026-10-20_바쿠모터회로도최종.pdf` (page 1). This is a schematic review and simulated software test, **not** physical PCB validation.

## Confirmed ATmega328P-AU signals
| Signal | MCU port (physical pin) | Firmware |
|---|---|---|
| ETH_CS | PD7 (11) | `spi_eth()` |
| ETH_INT | PB1 (13) | Input, W5500 polled |
| CAN_CS | PB2 (14, hardware SS) | `spi_can()` |
| CAN_INT | PD2 (32) | Input, MCP2515 polled |
| MOSI | PB3 (15) | SPI master output |
| MISO | PB4 (16) | SPI input |
| SCK | PB5 (17) | SPI clock |
| SW_MANUAL | PD3 (1) | Active LOW external pull-up |
| SW_AUTO | PD4 (2) | Active LOW external pull-up |
| UART TX/RX | PD1 (31) / PD0 (30) | 115200 8N1 |

PB0 is **not** `ETH_INT1` in the final schematic: the previous comment was incorrect and was amended.

## Mode decoding (subject to verification on the real car)
- Manual: PD3 LOW / PD4 HIGH, only ASMS mode 2 packets accepted
- Auto: PD3 HIGH / PD4 LOW, requires ASMS mode 1 and PC commands
- Throttle center: both HIGH, periodic MCU CAN motor commands inhibited; on transition emit HALT once if handshake already completed
- Both LOW: invalid; stop commands and reject remote control
- 30ms software debounce; every transition clears prior drive instructions
- ASMS-origin mode 3 emergency command in 3B or 5B packets latches software ESTOP until reset. **Does not replace independent physical E-STOP.**

## Outstanding hardware checks: do not claim verified
- J2 12V input is followed by NCP1117ST50T3G 5V regulator and AP2112K 3.3V regulator. Verify input polarity, heatsinking, voltage and current capacity **on assembled board** before powering; do not power from USBasp and 12V concurrently without validating power-domain interactions.
- MCP2515 crystal X2 and C3/C7 connection appears unclear in schematic; verify in netlist/PCB and continuity.
- CANH/CANL resistor R3 is annotated **82.5 kOhm**; if intended as a CAN termination it is not typical 120 Ohm. Confirm bus end termination and actual installed part.
- W5500 module/74AHCT125 MISO level and CS path requires hardware inspection; SPI pin numbers correct in firmware alone do **not** prove Ethernet works.
- The code parses bytes 6 and 7 of non-handshake CAN extended ID 0x1801D0EF as speed; confirm which feedback page the controller actually sends.
- `PHASE_CURRENT_REF_A=500.0f` is a configured current reference, **not an established safe tested limit**.
- No real AVR build, new HEX flash, Ethernet ARP confirmation, CAN or motor bench test of this revision has been performed.

## Network and build settings
PCB address `10.177.21.2`, control UDP 5000. RPM + timestamp little-endian payload: 8 bytes, UDP to destination `10.177.21.1:5001` by default, target 20ms (50Hz). CAN motor command period remains 50ms (20Hz).
To send feedback to diagnostic laptop `10.177.21.50`, define `BAKU_PC_IP_LAST_OCTET=50` for AVR build (Microchip Studio AVR/GNU C Compiler Symbols), rebuild and flash again.
