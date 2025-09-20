# Low-Power DMA UART Logger (Sanitized)

This project demonstrates a **low-power data logging framework** for 32-bit microcontrollers.  
The firmware captures fixed-length sensor packets using **DMA + interrupt-driven UART**, timestamps each packet with **microsecond precision**,  
and stores the results in a ring buffer while remaining in a **deep-sleep mode** between events.

---

### Design Highlights
- **DMA-Driven UART Reception** – CPU stays asleep while DMA transfers incoming packets directly to RAM.
- **Microsecond Timestamping** – Low-power real-time counter provides long-duration timing with minimal drift.
- **Energy Mode 2 Operation** – Demonstrates >90% power savings compared to active mode during idle periods.
- **Bootloader-Friendly** – Code structure supports safe firmware updates and recovery hooks.

### File Structure
- `low_power_logger.c` – Single, sanitized C file containing:
  - Initialization of clocks and peripherals (platform-agnostic placeholders).
  - DMA setup and ring buffer management.
  - Interrupt Service Routines for packet reception and buffer rollover.
  - Low-power sleep loop with wake-on-UART.

### Security Notice
This code is **not production firmware**.  
Vendor-specific register definitions and hardware constants have been replaced with placeholders  
(`// [REDACTED MCU-specific code]`) to comply with NDA while preserving logic and educational value.

### Intended Use
A learning reference for:
- Embedded engineers exploring **DMA, low-power modes, and real-time logging**.
- IoT or environmental sensing applications where **long deployments and tight energy budgets** are critical.
