## Current Status

This repository currently contains the original single-file AbleTone firmware.

Current implemented features:
- AVR register-level GPIO setup
- Bit-banged SPI output to MCP4821 12-bit DAC
- Note-selection inputs using PORTD
- Key up/down shifting with interrupts
- RGB LED PWM feedback using TCA0 split mode and TCB3

Planned refactor:
- Split DAC logic into dac.c/dac.h
- Split software SPI logic into soft_spi.c/soft_spi.h
- Split RGB PWM logic into rgb.c/rgb.h
- Split note/key logic into synth.c/synth.h
- Add hardware wiring notes and test documentation