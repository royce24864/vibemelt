#pragma once
#include "rmt_common.h"
struct rmt_transmit_config_t {
  int loop_count;
  struct { unsigned queue_nonblocking : 1; } flags;
};
int rmt_tx_wait_all_done(rmt_channel_handle_t channel, int timeout_ms);
int rmt_transmit(rmt_channel_handle_t channel, rmt_encoder_handle_t encoder,
                 const void *payload, size_t size,
                 const rmt_transmit_config_t *config);
