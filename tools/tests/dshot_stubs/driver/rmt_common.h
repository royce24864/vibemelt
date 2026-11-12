#pragma once
#include <cstddef>
#include <cstdint>
#include "gpio.h"
constexpr int ESP_OK = 0;
constexpr int ESP_ERR_TIMEOUT = 1;
constexpr int ESP_FAIL = 2;
constexpr int RMT_CLK_SRC_DEFAULT = 0;
using rmt_channel_handle_t = void *;
using rmt_encoder_handle_t = void *;
struct rmt_symbol_word_t { uint32_t duration0; uint32_t duration1; };
int rmt_enable(rmt_channel_handle_t channel);
int rmt_disable(rmt_channel_handle_t channel);
int rmt_del_channel(rmt_channel_handle_t channel);
int rmt_del_encoder(rmt_encoder_handle_t encoder);
