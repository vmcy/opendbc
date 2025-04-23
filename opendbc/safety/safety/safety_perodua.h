#pragma once

#include "safety_declarations.h"

// === RX HOOK ===
// This version just always allows controls
static void perodua_rx_hook(const CANPacket_t *to_push) {
  UNUSED(to_push);

  //int addr = GET_ADDR(to_push);

  controls_allowed = true;
}

// === TX HOOK ===
// Only allow LKAS steering command through
static bool perodua_tx_hook(const CANPacket_t *to_send) {
  int addr = GET_ADDR(to_send);

  // STEERING_LKAS 
  if (addr == 0x1D0) {
    return true;
  }

  return false;
}

static bool perodua_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == 2) {
    // 0x1D0 is STEERING_LKAS
    block_msg = ((addr == 0x1D0));
  }

  return block_msg;
}

// === INIT ===
// No checks yet, just register hooks to satisfy Panda
static safety_config perodua_init(uint16_t param) {
  UNUSED(param);

  static RxCheck perodua_rx_checks[] = {
    {.msg = {{0x1A0, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 56U}, { 0 }, { 0 }}},  // WHEEL_SPEED
    {.msg = {{0x208, 0, 6, .ignore_checksum = true, .ignore_counter = true, .frequency = 31U}, { 0 }, { 0 }}},  // PCM_BUTTONS
    {.msg = {{0x273, 2, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 20U}, { 0 }, { 0 }}},  // ACC_CMD_HUD
  };

  static const CanMsg PERODUA_TX_MSGS[] = {
    {0x1D0, 0, 5, false},  // STEERING_LKAS
  };

  safety_config ret = BUILD_SAFETY_CFG(perodua_rx_checks, PERODUA_TX_MSGS);
  ret.disable_forwarding = false;
  return ret;
}

// === REGISTER ===
const safety_hooks perodua_hooks = {
  .init = perodua_init,
  .rx = perodua_rx_hook,
  .tx = perodua_tx_hook,
  .fwd = perodua_fwd_hook,
};
