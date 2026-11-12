#include "DShotRMT.h"

#include <cassert>
#include <cstdio>

static int64_t now_us = 100000;
static int wait_result = ESP_OK;
static int queue_result = ESP_OK;
static int receive_enable_result = ESP_OK;
static int wait_calls = 0;
static int transmit_calls = 0;
static const uint16_t *owned_payload = nullptr;
static uint16_t queued_payload = 0;
int64_t esp_timer_get_time() { return now_us; }
void delayMicroseconds(uint32_t delay) { now_us += delay; }

int rmt_tx_wait_all_done(rmt_channel_handle_t, int timeout_ms) {
  assert(timeout_ms == 0); // No API wait may block the motor task.
  ++wait_calls;
  return wait_result;
}
int rmt_transmit(rmt_channel_handle_t, rmt_encoder_handle_t, const void *payload,
                 size_t size, const rmt_transmit_config_t *config) {
  assert(config->flags.queue_nonblocking == 1);
  assert(size == sizeof(uint16_t));
  ++transmit_calls;
  if(queue_result == ESP_OK) {
    owned_payload = static_cast<const uint16_t *>(payload);
    queued_payload = *owned_payload;
  }
  return queue_result;
}
int rmt_receive(rmt_channel_handle_t, void *, size_t, const rmt_receive_config_t *) { return ESP_OK; }
int rmt_enable(rmt_channel_handle_t) { return receive_enable_result; }
int rmt_disable(rmt_channel_handle_t) { return ESP_OK; }
int rmt_del_channel(rmt_channel_handle_t) { return ESP_OK; }
int rmt_del_encoder(rmt_encoder_handle_t) { return ESP_OK; }
dshot_result_t init_rmt_tx_channel(gpio_num_t, rmt_channel_handle_t *channel, bool) {
  *channel = reinterpret_cast<void *>(1);
  return {true, DSHOT_TX_INIT_SUCCESS};
}
dshot_result_t init_rmt_rx_channel(gpio_num_t, rmt_channel_handle_t *channel,
                                   rmt_rx_event_callbacks_t *, void *) {
  *channel = reinterpret_cast<void *>(2);
  return {true, DSHOT_RX_INIT_SUCCESS};
}
dshot_result_t init_dshot_encoder(rmt_encoder_handle_t *encoder,
                                  const rmt_ticks_t &, uint16_t, uint16_t) {
  *encoder = reinterpret_cast<void *>(3);
  return {true, DSHOT_ENCODER_INIT_SUCCESS};
}

int main() {
  DShotRMT esc(GPIO_NUM_1, DSHOT300, false);
  assert(esc.sendThrottle(0).tx_outcome == DSHOT_TX_FAILED);
  assert(esc.begin().success);
  auto result = esc.sendThrottle(1000);
  assert(result.success && result.tx_outcome == DSHOT_TX_QUEUED);
  assert(esc.getThrottleValue() == 1000 && transmit_calls == 1);
  assert(*owned_payload == queued_payload); // Persistent after send returns.
  const uint16_t old_payload = queued_payload;

  result = esc.sendThrottle(0); // Same-clock call is a skip, not a sent stop.
  assert(!result.success && result.tx_outcome == DSHOT_TX_SKIPPED);
  assert(transmit_calls == 1 && esc.getThrottleValue() == 1000);
  assert(*owned_payload == old_payload);

  now_us += 500;
  wait_result = ESP_ERR_TIMEOUT;
  result = esc.sendThrottle(0);
  assert(!result.success && result.tx_outcome == DSHOT_TX_FAILED);
  assert(transmit_calls == 1 && *owned_payload == old_payload);
  assert(esc.getThrottleValue() == 1000); // Busy RMT still owns original payload.

  wait_result = ESP_OK;
  result = esc.sendThrottle(0);
  assert(result.success && result.tx_outcome == DSHOT_TX_QUEUED);
  assert(transmit_calls == 2 && queued_payload == 0);
  assert(esc.getThrottleValue() == 0); // One frame, no repeated-command loop.

  now_us += 500;
  queue_result = ESP_ERR_TIMEOUT;
  result = esc.sendThrottle(1200);
  assert(!result.success && result.tx_outcome == DSHOT_TX_FAILED);
  assert(esc.getThrottleValue() == 0);
  queue_result = ESP_OK;
  result = esc.sendThrottle(900);
  assert(result.success && result.tx_outcome == DSHOT_TX_QUEUED);
  assert(esc.getThrottleValue() == 900);
  assert(wait_calls == 5);

  DShotRMT bidirectional(GPIO_NUM_1, DSHOT300, true);
  assert(bidirectional.begin().success);
  receive_enable_result = ESP_FAIL;
  result = bidirectional.sendThrottle(0);
  assert(!result.success && result.tx_outcome == DSHOT_TX_QUEUED);
  assert(result.result_code == DSHOT_RECEIVER_FAILED);
  std::puts("DShot transport tests passed (payload lifetime, skip, busy, stop, queue failure)");
}
