#include "bxcan_declarations.h"

// IRQs: CAN1_TX, CAN1_RX0, CAN1_SCE
//       CAN2_TX, CAN2_RX0, CAN2_SCE
//       CAN3_TX, CAN3_RX0, CAN3_SCE

CAN_TypeDef *cans[CAN_ARRAY_SIZE] = {CAN1, CAN2, CAN3};
uint8_t can_irq_number[CAN_IRQS_ARRAY_SIZE][CAN_IRQS_ARRAY_SIZE] = {
  { CAN1_TX_IRQn, CAN1_RX0_IRQn, CAN1_SCE_IRQn },
  { CAN2_TX_IRQn, CAN2_RX0_IRQn, CAN2_SCE_IRQn },
  { CAN3_TX_IRQn, CAN3_RX0_IRQn, CAN3_SCE_IRQn },
};

bool can_set_speed(uint8_t can_number) {
  bool ret = true;
  CAN_TypeDef *CANx = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

  ret &= llcan_set_speed(
    CANx,
    bus_config[bus_number].can_speed,
    can_loopback,
    (unsigned int)(can_silent) & (1U << can_number)
  );
  return ret;
}

void can_clear_send(CAN_TypeDef *CANx, uint8_t can_number) {
  can_health[can_number].can_core_reset_cnt += 1U;
  llcan_clear_send(CANx);
}

void update_can_health_pkt(uint8_t can_number, uint32_t ir_reg) {
  CAN_TypeDef *CANx = CANIF_FROM_CAN_NUM(can_number);
  uint32_t esr_reg = CANx->ESR;

  can_health[can_number].bus_off = ((esr_reg & CAN_ESR_BOFF) >> CAN_ESR_BOFF_Pos);
  can_health[can_number].bus_off_cnt += can_health[can_number].bus_off;
  can_health[can_number].error_warning = ((esr_reg & CAN_ESR_EWGF) >> CAN_ESR_EWGF_Pos);
  can_health[can_number].error_passive = ((esr_reg & CAN_ESR_EPVF) >> CAN_ESR_EPVF_Pos);

  can_health[can_number].last_error = ((esr_reg & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);
  if ((can_health[can_number].last_error != 0U) && (can_health[can_number].last_error != 7U)) {
    can_health[can_number].last_stored_error = can_health[can_number].last_error;
  }

  can_health[can_number].receive_error_cnt = ((esr_reg & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
  can_health[can_number].transmit_error_cnt = ((esr_reg & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);

  can_health[can_number].irq0_call_rate = interrupts[can_irq_number[can_number][0]].call_rate;
  can_health[can_number].irq1_call_rate = interrupts[can_irq_number[can_number][1]].call_rate;
  can_health[can_number].irq2_call_rate = interrupts[can_irq_number[can_number][2]].call_rate;

  if (ir_reg != 0U) {
    can_health[can_number].total_error_cnt += 1U;

    // RX message lost due to FIFO overrun
    if ((CANx->RF0R & (CAN_RF0R_FOVR0)) != 0U) {
      can_health[can_number].total_rx_lost_cnt += 1U;
      CANx->RF0R &= ~(CAN_RF0R_FOVR0);
    }
    can_clear_send(CANx, can_number);
  }
}

// ***************************** CAN *****************************
// CANx_SCE IRQ Handler
static void can_sce(uint8_t can_number) {
  update_can_health_pkt(can_number, 1U);
}

// CANx_TX IRQ Handler
void process_can(uint8_t can_number) {
  if (can_number != 0xffU) {

    ENTER_CRITICAL();

    CAN_TypeDef *CANx = CANIF_FROM_CAN_NUM(can_number);
    uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

    // check for empty mailbox
    CANPacket_t to_send;
    if ((CANx->TSR & (CAN_TSR_TERR0 | CAN_TSR_ALST0)) != 0U) { // last TX failed due to error arbitration lost
      can_health[can_number].total_tx_lost_cnt += 1U;
      CANx->TSR |= (CAN_TSR_TERR0 | CAN_TSR_ALST0);
    }
    if ((CANx->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
      // add successfully transmitted message to my fifo
      if ((CANx->TSR & CAN_TSR_RQCP0) == CAN_TSR_RQCP0) {
        if ((CANx->TSR & CAN_TSR_TXOK0) == CAN_TSR_TXOK0) {
          CANPacket_t to_push;
          to_push.fd = 0U;
          to_push.returned = 1U;
          to_push.rejected = 0U;
          to_push.extended = (CANx->sTxMailBox[0].TIR >> 2) & 0x1U;
          to_push.addr = (to_push.extended != 0U) ? (CANx->sTxMailBox[0].TIR >> 3) : (CANx->sTxMailBox[0].TIR >> 21);
          to_push.data_len_code = CANx->sTxMailBox[0].TDTR & 0xFU;
          to_push.bus = bus_number;
          WORD_TO_BYTE_ARRAY(&to_push.data[0], CANx->sTxMailBox[0].TDLR);
          WORD_TO_BYTE_ARRAY(&to_push.data[4], CANx->sTxMailBox[0].TDHR);
          can_set_checksum(&to_push);

          rx_buffer_overflow += can_push(&can_rx_q, &to_push) ? 0U : 1U;
        }

        // clear interrupt
        // careful, this can also be cleared by requesting a transmission
        CANx->TSR |= CAN_TSR_RQCP0;
      }

      if (can_pop(can_queues[bus_number], &to_send)) {
        if (can_check_checksum(&to_send)) {
          can_health[can_number].total_tx_cnt += 1U;
          // only send if we have received a packet
          CANx->sTxMailBox[0].TIR = ((to_send.extended != 0U) ? (to_send.addr << 3) : (to_send.addr << 21)) | (to_send.extended << 2);
          CANx->sTxMailBox[0].TDTR = to_send.data_len_code;
          BYTE_ARRAY_TO_WORD(CANx->sTxMailBox[0].TDLR, &to_send.data[0]);
          BYTE_ARRAY_TO_WORD(CANx->sTxMailBox[0].TDHR, &to_send.data[4]);
          // Send request TXRQ
          CANx->sTxMailBox[0].TIR |= 0x1U;
        } else {
          can_health[can_number].total_tx_checksum_error_cnt += 1U;
        }

        refresh_can_tx_slots_available();
      }
    }

    EXIT_CRITICAL();
  }
}

// is the user currently requesting preconditioning to be active?
bool precondition_enabled = false;
// timestamp of when the user requested preconditioning
uint32_t precondition_requested_ts = 0U;
// timestamp of last attempt to send precondition message, used for retry logic
uint32_t precondition_last_attempt_ts = 0U;
// number of ticks remaining to send precondition start/stop messages, used for initial burst logic
uint8_t precondition_start_ticks_remaining = 0U;
uint8_t precondition_stop_ticks_remaining = 0U;
// number of times we've retried sending precondition messages, used for retry logic
uint8_t precondition_retries = 0U;
// has preconditioning been confirmed by the 2AD status frame?
bool precondition_confirmed = false;
// has precondition stop been confirmed by the 2AD status frame?
bool precondition_stop_confirmed = true;
// track previous state of star button for edge detection
bool star_button_prev = false;

#define PRECONDITION_DEBOUNCE_US 5000000U  // 5 seconds
#define PRECONDITION_START_PHASE1_TICKS 3U // 4003 message
#define PRECONDITION_START_PHASE2_TICKS 3U // E007 message
#define PRECONDITION_STOP_PHASE1_TICKS 3U  // 0000 message
#define PRECONDITION_STOP_PHASE2_TICKS 3U  // E007 message
#define PRECONDITION_RETRY_US 10000000U  // 10 seconds
#define PRECONDITION_MAX_RETRIES 10U

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

// intercept NAV messages on 0x4ED while precondition is requested,
// respond with the same message but replace the last 3 bytes with 10 A0 00
// Returns whether to actually forward the message.
bool precondition_fwd_hook(CANPacket_t *to_send, uint8_t bus_fwd_num) {
  if (precondition_enabled && to_send->addr == 0x0C7U && bus_fwd_num == CAR_BUS) {
    // block 0x0C7 messages from the head unit while preconditioning is requested
    return false;
  }
  
  // only modify messages when precondition is requested, the message is going to the car, with the correct address
  if (!precondition_enabled || to_send->addr != 0x4EDU || bus_fwd_num != CAR_BUS) {
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

void precondition_can_rx_hook(CANPacket_t *to_push) {
  // 0x2AD status frame: second byte indicates precondition state
  //   0x01 = off/idle, 0x05 = starting, 0x15 = fully running
  if (to_push->addr == 0x2ADU) {
    uint8_t status = to_push->data[1];
    if (precondition_enabled && !precondition_confirmed) {
      if (status == 0x05U || status == 0x15U) {
        precondition_confirmed = true;
      }
    }
    if (!precondition_enabled && !precondition_stop_confirmed && precondition_stop_ticks_remaining == 0U) {
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
      if (!precondition_enabled) {
        precondition_enabled = true;
        precondition_requested_ts = now;
        precondition_last_attempt_ts = now;
        precondition_start_ticks_remaining = PRECONDITION_START_PHASE1_TICKS + PRECONDITION_START_PHASE2_TICKS;
        precondition_confirmed = false;
        precondition_retries = 0U;
      } else if (get_ts_elapsed(now, precondition_requested_ts) > PRECONDITION_DEBOUNCE_US) {
        precondition_enabled = false;
        precondition_last_attempt_ts = now;
        precondition_stop_ticks_remaining = PRECONDITION_STOP_PHASE1_TICKS + PRECONDITION_STOP_PHASE2_TICKS;
        precondition_stop_confirmed = false;
        precondition_retries = 0U;
      }
    }
    star_button_prev = star_button;
  }
}

// called every 40ms by fast_tick_handler in main.c
void precondition_tick(void) {
  uint32_t now = microsecond_timer_get();

  // retry start if not confirmed
  if (precondition_enabled && !precondition_confirmed
      && precondition_start_ticks_remaining == 0U
      // TODO(ejones): if we go through all our retries, maybe we should reset precondition_enabled to false and require the user to press the button again...
      && precondition_retries < PRECONDITION_MAX_RETRIES
      && get_ts_elapsed(now, precondition_last_attempt_ts) > PRECONDITION_RETRY_US) {
    precondition_last_attempt_ts = now;
    precondition_start_ticks_remaining = PRECONDITION_START_PHASE1_TICKS + PRECONDITION_START_PHASE2_TICKS;
    precondition_retries++;
  }

  // retry stop if not confirmed
  if (!precondition_enabled && !precondition_stop_confirmed
      && precondition_stop_ticks_remaining == 0U
      && precondition_retries < PRECONDITION_MAX_RETRIES
      && get_ts_elapsed(now, precondition_last_attempt_ts) > PRECONDITION_RETRY_US) {
    precondition_last_attempt_ts = now;
    precondition_stop_ticks_remaining = PRECONDITION_STOP_PHASE1_TICKS + PRECONDITION_STOP_PHASE2_TICKS;
    precondition_retries++;
  }

  // send initial burst of start messages
  if (precondition_enabled && precondition_start_ticks_remaining > 0U) {
    send_precondition_start_msg(precondition_start_ticks_remaining);
    precondition_start_ticks_remaining--;
  }

  // send initial burst of stop messages
  if (precondition_stop_ticks_remaining > 0U) {
    send_precondition_stop_msg(precondition_stop_ticks_remaining);
    precondition_stop_ticks_remaining--;
  }
}


// CANx_RX0 IRQ Handler
// blink blue when we are receiving CAN messages
void can_rx(uint8_t can_number) {
  CAN_TypeDef *CANx = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

  while ((CANx->RF0R & CAN_RF0R_FMP0) != 0U) {
    can_health[can_number].total_rx_cnt += 1U;

    // can is live
    pending_can_live = 1;

    // add to my fifo
    CANPacket_t to_push;

    to_push.fd = 0U;
    to_push.returned = 0U;
    to_push.rejected = 0U;
    to_push.extended = (CANx->sFIFOMailBox[0].RIR >> 2) & 0x1U;
    to_push.addr = (to_push.extended != 0U) ? (CANx->sFIFOMailBox[0].RIR >> 3) : (CANx->sFIFOMailBox[0].RIR >> 21);
    to_push.data_len_code = CANx->sFIFOMailBox[0].RDTR & 0xFU;
    to_push.bus = bus_number;
    WORD_TO_BYTE_ARRAY(&to_push.data[0], CANx->sFIFOMailBox[0].RDLR);
    WORD_TO_BYTE_ARRAY(&to_push.data[4], CANx->sFIFOMailBox[0].RDHR);
    can_set_checksum(&to_push);

    precondition_can_rx_hook(&to_push);

    // forwarding (panda only)
    int bus_fwd_num = safety_fwd_hook(bus_number, to_push.addr);
    if (bus_fwd_num != -1) {
      CANPacket_t to_send;

      to_send.fd = 0U;
      to_send.returned = 0U;
      to_send.rejected = 0U;
      to_send.extended = to_push.extended; // TXRQ
      to_send.addr = to_push.addr;
      to_send.bus = to_push.bus;
      to_send.data_len_code = to_push.data_len_code;
      (void)memcpy(to_send.data, to_push.data, dlc_to_len[to_push.data_len_code]);
      can_set_checksum(&to_send);

      if (precondition_fwd_hook(&to_send, bus_fwd_num)) {
        can_send(&to_send, bus_fwd_num, true);
        can_health[can_number].total_fwd_cnt += 1U;
      }
    }

    safety_rx_invalid += safety_rx_hook(&to_push) ? 0U : 1U;
    ignition_can_hook(&to_push);

    led_set(LED_BLUE, true);
    rx_buffer_overflow += can_push(&can_rx_q, &to_push) ? 0U : 1U;

    // next
    CANx->RF0R |= CAN_RF0R_RFOM0;
  }
}

static void CAN1_TX_IRQ_Handler(void) { process_can(0); }
static void CAN1_RX0_IRQ_Handler(void) { can_rx(0); }
static void CAN1_SCE_IRQ_Handler(void) { can_sce(0); }

static void CAN2_TX_IRQ_Handler(void) { process_can(1); }
static void CAN2_RX0_IRQ_Handler(void) { can_rx(1); }
static void CAN2_SCE_IRQ_Handler(void) { can_sce(1); }

static void CAN3_TX_IRQ_Handler(void) { process_can(2); }
static void CAN3_RX0_IRQ_Handler(void) { can_rx(2); }
static void CAN3_SCE_IRQ_Handler(void) { can_sce(2); }

bool can_init(uint8_t can_number) {
  bool ret = false;

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN2_TX_IRQn, CAN2_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN2_RX0_IRQn, CAN2_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN2_SCE_IRQn, CAN2_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN3_TX_IRQn, CAN3_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_RX0_IRQn, CAN3_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_SCE_IRQn, CAN3_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)

  if (can_number != 0xffU) {
    CAN_TypeDef *CANx = CANIF_FROM_CAN_NUM(can_number);
    ret &= can_set_speed(can_number);
    ret &= llcan_init(CANx);
    // in case there are queued up messages
    process_can(can_number);
  }
  return ret;
}
