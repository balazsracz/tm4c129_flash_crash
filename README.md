# Repro case for TM4C129x device crashing upon flash write.

This application demonstrates that valid application code can crash the TM4C129x processor core in an unrecoverable state. The demo runs on the EK-TM4C1294XL LaunchPad.

## Installation

1. Download TivaWare
2. The contents of the tm4c129 folder need to be copied into a new folder under `sw-tm4c-2.2.0.295/examples/boards/ek-tm4c1294xl`, such as `flash_crash`
3. make sure arm-none-eabi-gcc is in the $PATH
4. `cd sw-tm4c-2.2.0.295/examples/boards/ek-tm4c1294xl/flash_crash`
5. `make`
6. flash the gcc/blinky.axf program into the launchpad

## Use
1. make sure that JP4 and JP5 are aligned towards "UART"
2. connect the DEBUG USB port to the computer. start a terminal emulator on the virtual COM port seen (115200/N/8/1).
3. reset the launchpad. You should see Hello, World on the terminal.
4. press SW1 button to launch the tests.

## Reproduce crash

1. Uncomment the numbers in the blacklist in `blinky.c`
2. build the binary, flash it, run it.
3. observe that the tests halt at the blacklisted test cases
4. attempt to connect with an emulator (builtin or external). You will find that the processor core is inaccessible to the emulator (e.g. cannot read registers, or it appears as if all registers were having the same value).
