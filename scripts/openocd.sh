#!/usr/bin/env bash


openocd -s /usr/share/openocd/scripts -f interface/stlink-v2.cfg -f target/stm32l4x.cfg
