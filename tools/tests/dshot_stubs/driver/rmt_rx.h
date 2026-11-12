#pragma once
#include "rmt_common.h"
struct rmt_rx_done_event_data_t {
  rmt_symbol_word_t *received_symbols;
  size_t num_symbols;
};
struct rmt_rx_event_callbacks_t {
  bool (*on_recv_done)(rmt_channel_handle_t,
                       const rmt_rx_done_event_data_t *, void *);
};
struct rmt_receive_config_t { uint32_t signal_range_min_ns; uint32_t signal_range_max_ns; };
int rmt_receive(rmt_channel_handle_t channel, void *payload, size_t size,
                const rmt_receive_config_t *config);
