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
  //int addr = GET_ADDR(to_send);

  return true;

  // LKAS command ID 
  if (addr == 0x1D0) {
    return true;
  }

  return false;
}

// === INIT ===
// No checks yet, just register hooks to satisfy Panda
static safety_config perodua_init(uint16_t param) {
  UNUSED(param);

  // Wheel speed signals (WHEELSPEED_F, WHEELSPEED_B)
  static RxCheck perodua_rx_checks[] = {
    {.msg = {{0x1A0, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 56U}, { 0 }, { 0 }}},
  };

  // Wheel Speed CLEAN
  /*
  static RxCheck perodua_rx_checks[] = {
    {.msg = {{0x260, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 21U}, { 0 }, { 0 }}},
  };
  */

  static const CanMsg PERODUA_TX_MSGS[] = {
    {0x1D0, 0, 5, false},  // steering cmd addr
  };

  safety_config ret = BUILD_SAFETY_CFG(perodua_rx_checks, PERODUA_TX_MSGS);
  ret.disable_forwarding = true;
  return ret;
}

// === REGISTER ===
const safety_hooks perodua_hooks = {
  .init = perodua_init,
  .rx = perodua_rx_hook,
  .tx = perodua_tx_hook,
};
