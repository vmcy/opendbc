#pragma once

#include "safety_declarations.h"

const CanMsg DNGA_TX_MSGS[] = {{464, 0, 8, false}, {628, 0, 8, false}, {625, 0, 8, false}, {627, 0, 8, false}};

RxCheck dnga_rx_checks[] = {
  //{.msg = {{0x35F, 0, 8, .frequency = 20U}, { 0 }, { 0 }}},
};

// === RX HOOK ===
// This version just always allows controls
static void perodua_rx_hook(const CANPacket_t *to_push) {
  // UNUSED(to_push);
  // controls_allowed = true;
  vehicle_moving = true;
  controls_allowed = true;
  UNUSED(to_push);
}

// === TX HOOK ===
// Only allow LKAS steering command through
static bool perodua_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);
  int len = GET_LEN(to_send);
  UNUSED(addr);
  UNUSED(len);

  return tx;
}

static bool perodua_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == 2) {
    bool is_lkas_msg = ((addr == 464) || (addr == 628));
    bool is_acc_msg = ((addr == 625) || (addr == 627));
    block_msg = is_lkas_msg || is_acc_msg;
  }

  return block_msg;
}

// === INIT ===
// No checks yet, just register hooks to satisfy Panda
static safety_config perodua_init(uint16_t param) {
  UNUSED(param);
  return BUILD_SAFETY_CFG(dnga_rx_checks, DNGA_TX_MSGS);
}

// === REGISTER ===
const safety_hooks perodua_hooks = {
  .init = perodua_init,
  .rx = perodua_rx_hook,
  .tx = perodua_tx_hook,
  .fwd = perodua_fwd_hook,
};