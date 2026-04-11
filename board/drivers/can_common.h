#include "can_common_declarations.h"
#include "board/safety/safety.h"

uint32_t safety_tx_blocked = 0;
uint32_t safety_rx_invalid = 0;
uint32_t tx_buffer_overflow = 0;
uint32_t rx_buffer_overflow = 0;

can_health_t can_health[CAN_HEALTH_ARRAY_SIZE] = {{0}, {0}, {0}};

// Ignition detected from CAN meessages
bool ignition_can = false;
uint32_t ignition_can_cnt = 0U;

int can_live = 0;
int pending_can_live = 0;
int can_silent = ALL_CAN_SILENT;
bool can_loopback = false;

// ********************* instantiate queues *********************
#define can_buffer(x, size) \
  static CANPacket_t elems_##x[size]; \
  extern can_ring can_##x; \
  can_ring can_##x = { .w_ptr = 0, .r_ptr = 0, .fifo_size = (size), .elems = (CANPacket_t *)&(elems_##x) };

#define CAN_RX_BUFFER_SIZE 4096U
#define CAN_TX_BUFFER_SIZE 416U

#ifdef STM32H7
// ITCM RAM and DTCM RAM are the fastest for Cortex-M7 core access
__attribute__((section(".axisram"))) can_buffer(rx_q, CAN_RX_BUFFER_SIZE)
__attribute__((section(".itcmram"))) can_buffer(tx1_q, CAN_TX_BUFFER_SIZE)
__attribute__((section(".itcmram"))) can_buffer(tx2_q, CAN_TX_BUFFER_SIZE)
#else
can_buffer(rx_q, CAN_RX_BUFFER_SIZE)
can_buffer(tx1_q, CAN_TX_BUFFER_SIZE)
can_buffer(tx2_q, CAN_TX_BUFFER_SIZE)
#endif
can_buffer(tx3_q, CAN_TX_BUFFER_SIZE)

// FIXME:
// cppcheck-suppress misra-c2012-9.3
can_ring *can_queues[CAN_QUEUES_ARRAY_SIZE] = {&can_tx1_q, &can_tx2_q, &can_tx3_q};

// ********************* interrupt safe queue *********************
bool can_pop(can_ring *q, CANPacket_t *elem) {
  bool ret = 0;

  ENTER_CRITICAL();
  if (q->w_ptr != q->r_ptr) {
    *elem = q->elems[q->r_ptr];
    if ((q->r_ptr + 1U) == q->fifo_size) {
      q->r_ptr = 0;
    } else {
      q->r_ptr += 1U;
    }
    ret = 1;
  }
  EXIT_CRITICAL();

  return ret;
}

bool can_push(can_ring *q, const CANPacket_t *elem) {
  bool ret = false;
  uint32_t next_w_ptr;

  ENTER_CRITICAL();
  if ((q->w_ptr + 1U) == q->fifo_size) {
    next_w_ptr = 0;
  } else {
    next_w_ptr = q->w_ptr + 1U;
  }
  if (next_w_ptr != q->r_ptr) {
    q->elems[q->w_ptr] = *elem;
    q->w_ptr = next_w_ptr;
    ret = true;
  }
  EXIT_CRITICAL();
  if (!ret) {
    #ifdef DEBUG
      print("can_push to ");
      if (q == &can_rx_q) {
        print("can_rx_q");
      } else if (q == &can_tx1_q) {
        print("can_tx1_q");
      } else if (q == &can_tx2_q) {
        print("can_tx2_q");
      } else if (q == &can_tx3_q) {
        print("can_tx3_q");
      } else {
        print("unknown");
      }
      print(" failed!\n");
    #endif
  }
  return ret;
}

uint32_t can_slots_empty(const can_ring *q) {
  uint32_t ret = 0;

  ENTER_CRITICAL();
  if (q->w_ptr >= q->r_ptr) {
    ret = q->fifo_size - 1U - q->w_ptr + q->r_ptr;
  } else {
    ret = q->r_ptr - q->w_ptr - 1U;
  }
  EXIT_CRITICAL();

  return ret;
}

void can_clear(can_ring *q) {
  ENTER_CRITICAL();
  q->w_ptr = 0;
  q->r_ptr = 0;
  EXIT_CRITICAL();
  // handle TX buffer full with zero ECUs awake on the bus
  refresh_can_tx_slots_available();
}

// assign CAN numbering
// bus num: CAN Bus numbers in panda, sent to/from USB
//    Min: 0; Max: 127; Bit 7 marks message as receipt (bus 129 is receipt for but 1)
// cans: Look up MCU can interface from bus number
// can number: numeric lookup for MCU CAN interfaces (0 = CAN1, 1 = CAN2, etc);
// bus_lookup: Translates from 'can number' to 'bus number'.
// can_num_lookup: Translates from 'bus number' to 'can number'.
// forwarding bus: If >= 0, forward all messages from this bus to the specified bus.

// Helpers
// Panda:       Bus 0=CAN1   Bus 1=CAN2   Bus 2=CAN3
// TODO(ejones): modify bus config for MCAN (both sides); disable 3rd bus
// Mapping with current harness (M-CAN_dongle_003_stamped.pdf):
// Bus 0 = CAN1 = M-CAN_1 = to head unit (i think)
// Bus 2 = CAN3 = M-CAN_2 = to car (i think)
// also see added declarations in board/safety/declarations.h

bus_config_t bus_config[BUS_CONFIG_ARRAY_SIZE] = {
  { .bus_lookup = 0U, .can_num_lookup = 0U, .forwarding_bus = -1, .can_speed = 5000U, .can_data_speed = 20000U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
  { .bus_lookup = 1U, .can_num_lookup = 1U, .forwarding_bus = -1, .can_speed = 5000U, .can_data_speed = 20000U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
  { .bus_lookup = 2U, .can_num_lookup = 2U, .forwarding_bus = -1, .can_speed = 5000U, .can_data_speed = 20000U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
  { .bus_lookup = 0xFFU, .can_num_lookup = 0xFFU, .forwarding_bus = -1, .can_speed = 333U, .can_data_speed = 333U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
};

void can_init_all(void) {
  for (uint8_t i=0U; i < PANDA_CAN_CNT; i++) {
    #ifndef CANFD
      bus_config[i].can_data_speed = 0U;
    #endif
    can_clear(can_queues[i]);
    (void)can_init(i);
  }
}

void can_set_orientation(bool flipped) {
  bus_config[0].bus_lookup = flipped ? 2U : 0U;
  bus_config[0].can_num_lookup = flipped ? 2U : 0U;
  bus_config[2].bus_lookup = flipped ? 0U : 2U;
  bus_config[2].can_num_lookup = flipped ? 0U : 2U;
}

#ifdef PANDA_JUNGLE
void can_set_forwarding(uint8_t from, uint8_t to) {
  bus_config[from].forwarding_bus = to;
}
#endif

void ignition_can_hook(CANPacket_t *to_push) {
  (void)to_push;
}

bool can_tx_check_min_slots_free(uint32_t min) {
  return
    (can_slots_empty(&can_tx1_q) >= min) &&
    (can_slots_empty(&can_tx2_q) >= min) &&
    (can_slots_empty(&can_tx3_q) >= min);
}

uint8_t calculate_checksum(const uint8_t *dat, uint32_t len) {
  uint8_t checksum = 0U;
  for (uint32_t i = 0U; i < len; i++) {
    checksum ^= dat[i];
  }
  return checksum;
}

void can_set_checksum(CANPacket_t *packet) {
  packet->checksum = 0U;
  packet->checksum = calculate_checksum((uint8_t *) packet, CANPACKET_HEAD_SIZE + GET_LEN(packet));
}

bool can_check_checksum(CANPacket_t *packet) {
  return (calculate_checksum((uint8_t *) packet, CANPACKET_HEAD_SIZE + GET_LEN(packet)) == 0U);
}

void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook) {
  if (skip_tx_hook || safety_tx_hook(to_push) != 0) {
    // disable bus 1/CAN2 since it's disconnected on our harness
#ifdef NO_MITM
    if (bus_number < PANDA_BUS_CNT && bus_number != UNUSED_BUS_1 && bus_number != UNUSED_BUS_2) {
#else
    if (bus_number < PANDA_BUS_CNT && bus_number != UNUSED_BUS) {
#endif
      // add CAN packet to send queue
      tx_buffer_overflow += can_push(can_queues[bus_number], to_push) ? 0U : 1U;
      process_can(CAN_NUM_FROM_BUS_NUM(bus_number));
    }
  } else {
    safety_tx_blocked += 1U;
    to_push->returned = 0U;
    to_push->rejected = 1U;

    // data changed
    can_set_checksum(to_push);
    rx_buffer_overflow += can_push(&can_rx_q, to_push) ? 0U : 1U;
  }
}

bool is_speed_valid(uint32_t speed, const uint32_t *all_speeds, uint8_t len) {
  bool ret = false;
  for (uint8_t i = 0U; i < len; i++) {
    if (all_speeds[i] == speed) {
      ret = true;
    }
  }
  return ret;
}

// ********************* precondition logic *********************

// is the user currently requesting preconditioning to be active?
bool precondition_requested = false;
// timestamp of when the user requested preconditioning
uint32_t precondition_requested_ts = 0U;
// timestamp of last attempt to send precondition message, used for retry logic
uint32_t precondition_last_attempt_ts = 0U;
// number of ticks remaining to send precondition start/stop messages, used for initial burst logic
uint8_t precondition_start_ticks_remaining = 0U;
uint8_t precondition_stop_ticks_remaining = 0U;
// number of times we've retried sending precondition messages, used for retry logic
uint8_t precondition_retries = 0U;
// has preconditioning been confirmed to be starting by the 2AD status frame?
bool precondition_starting_confirmed = false;
// has preconditioning been confirmed to be active by the 2AD status frame?
bool precondition_started_confirmed = false;
// has precondition stop been confirmed by the 2AD status frame?
bool precondition_stop_confirmed = true;
// track previous state of star button for edge detection
bool star_button_prev = false;

#define PRECONDITION_DEBOUNCE_US 5000000U  // 5 seconds
#define PRECONDITION_START_PHASE1_TICKS 3U // 4003 message
#define PRECONDITION_START_PHASE2_TICKS 3U // E007 message
#define PRECONDITION_START_TICKS (PRECONDITION_START_PHASE1_TICKS + PRECONDITION_START_PHASE2_TICKS)
#define PRECONDITION_STOP_PHASE1_TICKS 3U  // 0000 message
#define PRECONDITION_STOP_PHASE2_TICKS 3U  // E007 message
#define PRECONDITION_STOP_TICKS (PRECONDITION_STOP_PHASE1_TICKS + PRECONDITION_STOP_PHASE2_TICKS)
#define PRECONDITION_RETRY_US 10000000U  // 10 seconds
#define PRECONDITION_MAX_RETRIES 10U
#define PRECONDITION_STARTED_TIMEOUT_US 80000000U  // 80 seconds

void send_precondition_start_msg(uint8_t ticks_remaining) {
  CANPacket_t packet = {0};
  packet.addr = 0x0C7U;
  packet.data_len_code = 8U;
  if (ticks_remaining > PRECONDITION_START_PHASE2_TICKS) {
    // send 0000004003000000 to 0x0C7
    packet.data[3] = 0x40U;
    packet.data[4] = 0x03U;
  } else {
    // send 000000E007000000 to 0x0C7
    packet.data[3] = 0xE0U;
    packet.data[4] = 0x07U;
  }
  can_set_checksum(&packet);
  can_send(&packet, CAR_BUS, true);
}

void send_precondition_stop_msg(uint8_t ticks_remaining) {
  CANPacket_t packet = {0};
  packet.addr = 0x0C7U;
  packet.data_len_code = 8U;
  if (ticks_remaining <= PRECONDITION_STOP_PHASE2_TICKS) {
    // send 000000E007000000 to 0x0C7
    packet.data[3] = 0xE0U;
    packet.data[4] = 0x07U;
  }
  can_set_checksum(&packet);
  can_send(&packet, CAR_BUS, true);
}

#ifndef NO_MITM
// intercept NAV messages on 0x4ED while precondition is requested,
// respond with the same message but replace the last 3 bytes with 10 A0 00
// Returns whether to actually forward the message.
bool precondition_fwd_hook(CANPacket_t *to_send, uint8_t bus_fwd_num) {
  if (precondition_requested && to_send->addr == 0x0C7U && bus_fwd_num == CAR_BUS) {
    // block 0x0C7 messages from the head unit while preconditioning is requested
    return false;
  }

  // only modify messages when precondition is requested, the message is going to the car, with the correct address
  if (!precondition_requested || to_send->addr != 0x4EDU || bus_fwd_num != CAR_BUS) {
    // don't modify, but still forward the message
    return true;
  }

  to_send->data[5] = 0x10U;
  to_send->data[6] = 0xA0U;
  to_send->data[7] = 0x00U;
  can_set_checksum(to_send);
  // normal forwarding logic will do the send
  return true;
}
#endif

void start_preconditioning(uint32_t now) {
  precondition_requested = true;
  precondition_requested_ts = now;
  precondition_last_attempt_ts = now;
  precondition_start_ticks_remaining = PRECONDITION_START_TICKS;
  precondition_starting_confirmed = false;
  precondition_started_confirmed = false;
  precondition_retries = 0U;
}

void stop_preconditioning(uint32_t now) {
  precondition_requested = false;
  precondition_last_attempt_ts = now;
  precondition_stop_ticks_remaining = PRECONDITION_STOP_TICKS;
  precondition_stop_confirmed = false;
  precondition_retries = 0U;
}

void precondition_can_rx_hook(CANPacket_t *to_push) {
#ifdef NO_MITM
  // when precondition is active and we see a 0x4ED from the head unit,
  // immediately send our modified version back on the same bus
  if (precondition_requested && to_push->addr == 0x4EDU && to_push->bus == CAR_BUS) {
    CANPacket_t modified = *to_push;
    modified.data[5] = 0x10U;
    modified.data[6] = 0xA0U;
    modified.data[7] = 0x00U;
    can_set_checksum(&modified);
    can_send(&modified, CAR_BUS, true);
  }
#endif

  // 0x2AD status frame: second byte indicates precondition state
  //   0x01 = off/idle, 0x05 = starting, 0x15 = fully running
  if (to_push->addr == 0x2ADU) {
    uint8_t status = to_push->data[1];
    if (precondition_requested && !precondition_starting_confirmed) {
      if (status == 0x05U || status == 0x15U) {
        precondition_starting_confirmed = true;
      }
    }
    if (precondition_requested && !precondition_started_confirmed) {
      if (status == 0x15U) {
        precondition_started_confirmed = true;
      }
    }
    if (precondition_requested && precondition_started_confirmed) {
      if (status == 0x05U) {
        // preconditioning was previously fully active, but now it's only showing as starting.
        // this is a weird situation to be in; let's just reset the current attempt time,
        // and let the retry logic continue as normal if it doesn't resolve itself after a while
        precondition_last_attempt_ts = microsecond_timer_get();
        precondition_started_confirmed = false;
      }
      if (status == 0x01U) {
        // preconditioning was previously fully active, but now it's showing as off.
        // it's possible that the car has reached the (rare) "Precondition complete" state.
        // until we have a better way to distinguish that state from a real failure mode (TODO(ejones)),
        // let's just assume everything is fine and reset our state
        uint32_t now = microsecond_timer_get();
        stop_preconditioning(now);
        precondition_stop_ticks_remaining = 0U;
        precondition_stop_confirmed = true;
      }
    }
    if (!precondition_requested && !precondition_stop_confirmed && precondition_stop_ticks_remaining == 0U) {
      if (status == 0x01U) {
        precondition_stop_confirmed = true;
      }
    }
  }

  // 0x448 has button presses:
  // 7F 00 00 00 00 00 00 00 -> Idle
  // 15 00 00 01 00 00 00 00 -> Mute
  // 67 00 00 00 00 10 00 00 -> Star
  // 31 00 00 00 04 00 00 00 -> Menu
  // 7A 00 04 00 00 00 00 00 -> speak
  // listen for star button press (rising edge only), and toggle preconditioning
  if (to_push->addr == 0x448U) {
    bool star_button = (to_push->data[5] == 0x10U);
    if (star_button && !star_button_prev) {
      uint32_t now = microsecond_timer_get();
      if (!precondition_requested) {
        start_preconditioning(now);
      } else if (get_ts_elapsed(now, precondition_requested_ts) > PRECONDITION_DEBOUNCE_US) {
        stop_preconditioning(now);
      }
    }
    star_button_prev = star_button;
  }
}

// called every 40ms by fast_tick_handler in main.c
void precondition_tick(void) {
  uint32_t now = microsecond_timer_get();
  uint32_t time_since_last_attempt = get_ts_elapsed(now, precondition_last_attempt_ts);

  // give up and send one stop attempt if start retries exhausted
  if (precondition_requested
      && precondition_start_ticks_remaining == 0U
      && precondition_retries >= PRECONDITION_MAX_RETRIES
      && ((!precondition_starting_confirmed && time_since_last_attempt > PRECONDITION_RETRY_US)
          || (!precondition_started_confirmed && time_since_last_attempt > PRECONDITION_STARTED_TIMEOUT_US))) {
    stop_preconditioning(now);
    precondition_retries = PRECONDITION_MAX_RETRIES;  // don't retry the stop; this is a failure case already
  }

  // retry start if not confirmed to be starting yet and it's been long enough
  if (precondition_requested && !precondition_starting_confirmed
      && precondition_start_ticks_remaining == 0U
      && precondition_retries < PRECONDITION_MAX_RETRIES
      && time_since_last_attempt > PRECONDITION_RETRY_US) {
    precondition_last_attempt_ts = now;
    precondition_start_ticks_remaining = PRECONDITION_START_TICKS;
    precondition_retries++;
  }

  // retry start if not confirmed to be started yet and it's been long enough (i.e. we got 2AD 05 but not 15 after a long time)
  if (precondition_requested && !precondition_started_confirmed
      && precondition_start_ticks_remaining == 0U
      && precondition_retries < PRECONDITION_MAX_RETRIES
      && time_since_last_attempt > PRECONDITION_STARTED_TIMEOUT_US) {
    precondition_last_attempt_ts = now;
    precondition_start_ticks_remaining = PRECONDITION_START_TICKS;
    precondition_retries++;
  }

  // retry stop if not confirmed
  if (!precondition_requested && !precondition_stop_confirmed
      && precondition_stop_ticks_remaining == 0U
      && precondition_retries < PRECONDITION_MAX_RETRIES
      && time_since_last_attempt > PRECONDITION_RETRY_US) {
    precondition_last_attempt_ts = now;
    precondition_stop_ticks_remaining = PRECONDITION_STOP_TICKS;
    precondition_retries++;
  }

  // send initial burst of start messages
  if (precondition_requested && precondition_start_ticks_remaining > 0U) {
    send_precondition_start_msg(precondition_start_ticks_remaining);
    precondition_start_ticks_remaining--;
  }

  // send initial burst of stop messages
  if (!precondition_requested && precondition_stop_ticks_remaining > 0U) {
    send_precondition_stop_msg(precondition_stop_ticks_remaining);
    precondition_stop_ticks_remaining--;
  }
}
