# AVR version

This version should free the project of all Arduino dependancies. Another goal is to make the project smaller on the memory footprint.

## Current size

```
avr-size --format=avr --mcu=attiny85 main.elf
AVR Memory Usage
----------------
Device: attiny85

Program:    8174 bytes (99.8% Full)
(.text + .data + .bootloader)

Data:        197 bytes (38.5% Full)
(.data + .bss + .noinit)
```

## Compilation

To compile and flash the program : run the command `make flash`
If you want to compile only : `make all`

## TODO

- [ ] Include again button support
- [ ] Include again level selection
- [ ] Reduce code size
