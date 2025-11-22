#!/usr/bin/env bash


openocd -s /usr/share/openocd/scripts -f interface/stlink-dap.cfg -f target/stm32f4x.cfg
