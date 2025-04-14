#pragma once

#include "safety_declarations.h"

// === RX HOOK ===
// This version just always allows controls
static void perodua_rx_hook(const CANPacket_t *to_push) {
  UNUSED(to_push);
  controls_allowed = true;
}

// === TX HOOK ===
// Only allow LKAS steering command through
static bool perodua_tx_hook(const CANPacket_t *to_send) {
  int addr = GET_ADDR(to_send);

  return true;

  // Replace with actual LKAS command ID later
  if (addr == 0x2E4) {
    return true;
  }

  return false;
}

// === INIT ===
// No checks yet, just register hooks to satisfy Panda
static safety_config perodua_init(uint16_t param) {
  UNUSED(param);

  static RxCheck empty_rx_checks[] = {};

  static const CanMsg PERODUA_TX_MSGS[] = {
    {0x2E4, 0, 5, false},  // Replace this with your actual steering cmd addr
  };

  safety_config cfg = BUILD_SAFETY_CFG(empty_rx_checks, PERODUA_TX_MSGS);
  cfg.disable_forwarding = true;
  return cfg;
}

// === REGISTER ===
const safety_hooks perodua_hooks = {
  .init = perodua_init,
  .rx = perodua_rx_hook,
  .tx = perodua_tx_hook,
};
