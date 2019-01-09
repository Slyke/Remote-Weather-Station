#!/bin/bash

python ../esp-open-sdk/esptool/esptool.py --baud 57600 write_flash 0x00000 src/weather_station-0x00000.bin 0x10000 src/weather_station-0x10000.bin
