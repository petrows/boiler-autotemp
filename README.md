# Boiler autotemp project

AVR controller project for auto-ajust bolier temperature and remote control

Project description page: https://habr.com/ru/post/262437/

## Compile firmware for AVR

```bash
# Install deps
sudo apt install gcc-avr avr-libc
# Go to project
cd avr/controller-host
cmake . -DAVR_TYPE=atmega8
# Yes, call cmake second time!
cmake . -DAVR_TYPE=atmega8
# Build it
make
```

This will build binary and hex file for your. Replace `atmega8` with your exact controller model (if differs).

