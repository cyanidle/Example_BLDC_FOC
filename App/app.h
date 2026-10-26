#pragma once

#include "main.h"

#include <cyphal/cyphal.h>
#include <voltbro/utils.hpp>

// communications.cpp
inline constexpr size_t CYPHAL_QUEUE_SIZE = 200;
inline constexpr millis_t DELAY_ON_ERROR_MS = 500;
std::shared_ptr<CyphalInterface> get_interface();
void in_loop_reporting(millis_t);
void setup_subscriptions();
void cyphal_loop();
void start_cyphal();
void set_cyphal_mode(uint8_t mode);
void set_cyphal_health(uint8_t health);
CanardNodeID get_node_id();

// common.cpp
micros_t micros_64();
millis_t millis_32();
void start_timers();

// app.cpp: applies one runtime-config value with the same vocabulary as the CAN
// config topic (see ConfigId there and mods/config_defs.lua in the repo root).
void app_apply_config(int32_t id, float value);
