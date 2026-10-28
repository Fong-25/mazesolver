
---
name: avr-firmware-optimization
description: Optimize existing PlatformIO Arduino AVR firmware for runtime speed, SRAM, flash, and deterministic timing while preserving behavior. Use for Uno, classic Nano, and Mega 2560 projects.
---

# AVR Firmware Optimization

## 1. Identify the real target
- Read platformio.ini and inspect every relevant environment.
- Confirm the exact MCU, board ID, clock, framework, and library versions.
- Distinguish ATmega328P Uno/Nano from ATmega2560 Mega and newer Nano variants.
- Read the existing code and identify timing, memory, and correctness constraints.
- Do not assume that every board called Nano uses the same MCU.

## 2. Establish a baseline
- Record the current Git diff and working-tree state.
- Build the original code before optimization.
- Record PlatformIO flash and RAM usage for the target environment.
- Inspect the ELF, map file, and disassembly when needed.
- Measure runtime behavior with a reproducible workload.
- Record timing, stack/heap concerns, and functional tests relevant to the change.
- Never fabricate measurements or claim hardware verification without evidence.

## 3. Optimize SRAM first when constrained
- Check global/static variables, buffers, stack use, and dynamic allocations.
- Use F("literal") for suitable constant strings passed to Arduino Print APIs.
- Consider PROGMEM for larger constant tables or strings accessed with the correct AVR APIs.
- Avoid unnecessary String concatenation and heap churn in long-running firmware.
- Bound buffers and check indexes before reading or writing.
- Avoid copying large structs or arrays unnecessarily.
- Do not move mutable data into flash.
- Do not assume that moving data to PROGMEM automatically makes ordinary pointer access valid.

## 4. Optimize runtime only with evidence
- Identify hot paths before changing algorithms.
- Prefer bounded work and predictable execution time in time-critical paths.
- Replace blocking delays only when requirements need concurrent or responsive behavior.
- Keep interrupt handlers short; defer expensive work.
- Use direct port access only when measurements justify it and pin mappings are verified.
- Check atomicity when sharing multi-byte values between ISRs and main code.
- Preserve timer, PWM, UART, I2C, and SPI behavior.
- Compare generated code or measured timing for claimed speed improvements.

## 5. Protect correctness
- Make small, reviewable changes.
- Preserve public APIs, protocol formats, timing guarantees, and electrical behavior.
- Check integer promotions, signedness, overflow, and narrowing conversions.
- Avoid undefined behavior and out-of-bounds access.
- Never change clock settings, fuses, bootloaders, or hardware pin mappings
  as an incidental optimization.

## 6. Verify every change
- Build the affected environment with pio run -e <env>.
- Compare flash and RAM reports against the baseline.
- Run available host tests and PlatformIO tests.
- Re-run relevant timing and hardware tests when the change affects behavior.
- Report what improved, what regressed, and what remains unverified.
- Keep a change only when its benefit is demonstrated and correctness is preserved.
