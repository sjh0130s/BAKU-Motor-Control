# 2026-10-21 PCB code update

- Removed physical 3-position selector dependency (PD3/PD4).
- ASMS mode 1 (AUTO) and mode 2 (MANUAL) are latched until another ASMS mode or reset.
- A MANUAL joystick requires fresh ASMS packets (1000ms watchdog); AUTO requires fresh PC speed commands (1000ms watchdog) but not a repeated ASMS AUTO.
- Optional compile-time `DEBUG_ALLOW_PC_AUTO_WITHOUT_ASMS=1` grants PC speed control in *unselected* state only, never overriding MANUAL or ESTOP. **Off by default.**
- Retained CAN handshake, 50ms CAN RUN/HALT heartbeat, 8-byte RPM/timestamp speed telemetry and E-STOP latch.
- New mock C tests for debug OFF/ON, mode latch, stale HALT, CAN and E-STOP; **not executed during upload; not hardware validated**.
- Existing schematic pinout: CAN_CS PB2, ETH_CS PD7, CAN_INT PD2, ETH_INT PB1.
- Old 2026-10-20 folder untouched.
