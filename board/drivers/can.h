// IRQs: CAN1_TX, CAN1_RX0, CAN1_SCE
//       CAN2_TX, CAN2_RX0, CAN2_SCE
//       CAN3_TX, CAN3_RX0, CAN3_SCE

typedef struct {
  volatile uint32_t w_ptr;
  volatile uint32_t r_ptr;
  uint32_t fifo_size;
  CAN_FIFOMailBox_TypeDef *elems;
} can_ring;

#define CAN_BUS_RET_FLAG 0x80U
#define CAN_BUS_NUM_MASK 0x7FU

#define BUS_MAX 4U

uint32_t can_rx_errs = 0;
uint32_t can_send_errs = 0;
uint32_t can_fwd_errs = 0;
uint32_t gmlan_send_errs = 0;
extern int can_live, pending_can_live;

// must reinit after changing these
extern int can_loopback, can_silent;
extern uint32_t can_speed[4];

void can_set_forwarding(int from, int to);

bool can_init(uint8_t can_number);
void can_init_all(void);
bool can_tx_check_min_slots_free(uint32_t min);
void can_send(CAN_FIFOMailBox_TypeDef *to_push, uint8_t bus_number, bool skip_tx_hook);
bool can_pop(can_ring *q, CAN_FIFOMailBox_TypeDef *elem);

// Ignition detected from CAN meessages
bool ignition_can = false;
bool ignition_cadillac = false;
uint32_t ignition_can_cnt = 0U;

// end API

#define ALL_CAN_SILENT 0xFF
#define ALL_CAN_LIVE 0

int can_live = 0, pending_can_live = 0, can_loopback = 0, can_silent = ALL_CAN_SILENT;

// ********************* instantiate queues *********************

#define can_buffer(x, size) \
  CAN_FIFOMailBox_TypeDef elems_##x[size]; \
  can_ring can_##x = { .w_ptr = 0, .r_ptr = 0, .fifo_size = size, .elems = (CAN_FIFOMailBox_TypeDef *)&elems_##x };

can_buffer(rx_q, 0x1000)
can_buffer(tx1_q, 0x100)
can_buffer(tx2_q, 0x100)
can_buffer(tx3_q, 0x100)
can_buffer(txgmlan_q, 0x100)
can_ring *can_queues[] = {&can_tx1_q, &can_tx2_q, &can_tx3_q, &can_txgmlan_q};

// global CAN stats
int can_rx_cnt = 0;
int can_tx_cnt = 0;
int can_txd_cnt = 0;
int can_err_cnt = 0;
int can_overflow_cnt = 0;

// ********************* interrupt safe queue *********************

bool can_pop(can_ring *q, CAN_FIFOMailBox_TypeDef *elem) {
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

bool can_push(can_ring *q, CAN_FIFOMailBox_TypeDef *elem) {
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
    can_overflow_cnt++;
    #ifdef DEBUG
      puts("can_push failed!\n");
    #endif
  }
  return ret;
}

uint32_t can_slots_empty(can_ring *q) {
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
}

// assign CAN numbering
// bus num: Can bus number on ODB connector. Sent to/from USB
//    Min: 0; Max: 127; Bit 7 marks message as receipt (bus 129 is receipt for but 1)
// cans: Look up MCU can interface from bus number
// can number: numeric lookup for MCU CAN interfaces (0 = CAN1, 1 = CAN2, etc);
// bus_lookup: Translates from 'can number' to 'bus number'.
// can_num_lookup: Translates from 'bus number' to 'can number'.
// can_forwarding: Given a bus num, lookup bus num to forward to. -1 means no forward.

// Panda:       Bus 0=CAN1   Bus 1=CAN2   Bus 2=CAN3
CAN_TypeDef *cans[] = {CAN1, CAN2, CAN3};
uint8_t bus_lookup[] = {0,1,2};
uint8_t can_num_lookup[] = {0,1,2,-1};
int8_t can_forwarding[] = {-1,-1,-1,-1};
uint32_t can_speed[] = {5000, 5000, 5000, 333};
#define CAN_MAX 3U

#define CANIF_FROM_CAN_NUM(num) (cans[num])
#define CAN_NUM_FROM_CANIF(CAN) ((CAN)==CAN1 ? 0 : ((CAN) == CAN2 ? 1 : 2))
#define BUS_NUM_FROM_CAN_NUM(num) (bus_lookup[num])
#define CAN_NUM_FROM_BUS_NUM(num) (can_num_lookup[num])

void process_can(uint8_t can_number);

bool can_set_speed(uint8_t can_number) {
  bool ret = true;
  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

  ret &= llcan_set_speed(CAN, can_speed[bus_number], can_loopback, (unsigned int)(can_silent) & (1U << can_number));
  return ret;
}

void can_init_all(void) {
  bool ret = true;
  for (uint8_t i=0U; i < CAN_MAX; i++) {
    can_clear(can_queues[i]);
    ret &= can_init(i);
  }
  UNUSED(ret);
}

void can_flip_buses(uint8_t bus1, uint8_t bus2){
  bus_lookup[bus1] = bus2;
  bus_lookup[bus2] = bus1;
  can_num_lookup[bus1] = bus2;
  can_num_lookup[bus2] = bus1;
}

// TODO: Cleanup with new abstraction
void can_set_gmlan(uint8_t bus) {
  if(board_has_gmlan()){
    // first, disable GMLAN on prev bus
    uint8_t prev_bus = can_num_lookup[3];
    if (bus != prev_bus) {
      switch (prev_bus) {
        case 1:
        case 2:
          puts("Disable GMLAN on CAN");
          puth(prev_bus + 1U);
          puts("\n");
          current_board->set_can_mode(CAN_MODE_NORMAL);
          bus_lookup[prev_bus] = prev_bus;
          can_num_lookup[prev_bus] = prev_bus;
          can_num_lookup[3] = -1;
          bool ret = can_init(prev_bus);
          UNUSED(ret);
          break;
        default:
          // GMLAN was not set on either BUS 1 or 2
          break;
      }
    }

    // now enable GMLAN on the new bus
    switch (bus) {
      case 1:
      case 2:
        puts("Enable GMLAN on CAN");
        puth(bus + 1U);
        puts("\n");
        current_board->set_can_mode((bus == 1U) ? CAN_MODE_GMLAN_CAN2 : CAN_MODE_GMLAN_CAN3);
        bus_lookup[bus] = 3;
        can_num_lookup[bus] = -1;
        can_num_lookup[3] = bus;
        bool ret = can_init(bus);
        UNUSED(ret);
        break;
      case 0xFF:  //-1 unsigned
        break;
      default:
        puts("GMLAN can only be set on CAN2 or CAN3\n");
        break;
    }
  } else {
    puts("GMLAN not available on black panda\n");
  }
}

// TODO: remove
void can_set_obd(uint8_t harness_orientation, bool obd){
  if(obd){
    puts("setting CAN2 to be OBD\n");
  } else {
    puts("setting CAN2 to be normal\n");
  }
  if(board_has_obd()){
    if(obd != (bool)(harness_orientation == HARNESS_STATUS_NORMAL)){
        // B5,B6: disable normal mode
        set_gpio_mode(GPIOB, 5, MODE_INPUT);
        set_gpio_mode(GPIOB, 6, MODE_INPUT);
        // B12,B13: CAN2 mode
        set_gpio_alternate(GPIOB, 12, GPIO_AF9_CAN2);
        set_gpio_alternate(GPIOB, 13, GPIO_AF9_CAN2);
    } else {
        // B5,B6: CAN2 mode
        set_gpio_alternate(GPIOB, 5, GPIO_AF9_CAN2);
        set_gpio_alternate(GPIOB, 6, GPIO_AF9_CAN2);
        // B12,B13: disable normal mode
        set_gpio_mode(GPIOB, 12, MODE_INPUT);
        set_gpio_mode(GPIOB, 13, MODE_INPUT);
    }
  } else {
    puts("OBD CAN not available on this board\n");
  }
}

// CAN error
void can_sce(CAN_TypeDef *CAN) {
  ENTER_CRITICAL();

  #ifdef DEBUG
    if (CAN==CAN1) puts("CAN1:  ");
    if (CAN==CAN2) puts("CAN2:  ");
    #ifdef CAN3
      if (CAN==CAN3) puts("CAN3:  ");
    #endif
    puts("MSR:");
    puth(CAN->MSR);
    puts(" TSR:");
    puth(CAN->TSR);
    puts(" RF0R:");
    puth(CAN->RF0R);
    puts(" RF1R:");
    puth(CAN->RF1R);
    puts(" ESR:");
    puth(CAN->ESR);
    puts("\n");
  #endif

  can_err_cnt += 1;
  llcan_clear_send(CAN);
  EXIT_CRITICAL();
}

// ***************************** CAN *****************************

void process_can(uint8_t can_number) {
  if (can_number != 0xffU) {

    ENTER_CRITICAL();

    CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
    uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

    // check for empty mailbox
    CAN_FIFOMailBox_TypeDef to_send;
    if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
      // add successfully transmitted message to my fifo
      if ((CAN->TSR & CAN_TSR_RQCP0) == CAN_TSR_RQCP0) {
        can_txd_cnt += 1;

        if ((CAN->TSR & CAN_TSR_TXOK0) == CAN_TSR_TXOK0) {
          CAN_FIFOMailBox_TypeDef to_push;
          to_push.RIR = CAN->sTxMailBox[0].TIR;
          to_push.RDTR = (CAN->sTxMailBox[0].TDTR & 0xFFFF000FU) | ((CAN_BUS_RET_FLAG | bus_number) << 4);
          to_push.RDLR = CAN->sTxMailBox[0].TDLR;
          to_push.RDHR = CAN->sTxMailBox[0].TDHR;
          can_send_errs += can_push(&can_rx_q, &to_push) ? 0U : 1U;
        }

        if ((CAN->TSR & CAN_TSR_TERR0) == CAN_TSR_TERR0) {
          #ifdef DEBUG
            puts("CAN TX ERROR!\n");
          #endif
        }

        if ((CAN->TSR & CAN_TSR_ALST0) == CAN_TSR_ALST0) {
          #ifdef DEBUG
            puts("CAN TX ARBITRATION LOST!\n");
          #endif
        }

        // clear interrupt
        // careful, this can also be cleared by requesting a transmission
        CAN->TSR |= CAN_TSR_RQCP0;
      }

      if (can_pop(can_queues[bus_number], &to_send)) {
        can_tx_cnt += 1;
        // only send if we have received a packet
        CAN->sTxMailBox[0].TDLR = to_send.RDLR;
        CAN->sTxMailBox[0].TDHR = to_send.RDHR;
        CAN->sTxMailBox[0].TDTR = to_send.RDTR;
        CAN->sTxMailBox[0].TIR = to_send.RIR;

        if (can_tx_check_min_slots_free(MAX_CAN_MSGS_PER_BULK_TRANSFER)) {
          usb_outep3_resume_if_paused();
        }
      }
    }

    EXIT_CRITICAL();
  }
}

void ignition_can_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);

  ignition_can_cnt = 0U;  // reset counter

  if (bus == 0) {
    // TODO: verify on all supported GM models that we can reliably detect ignition using only this signal,
    // since the 0x1F1 signal can briefly go low immediately after ignition
    if ((addr == 0x160) && (len == 5)) {
      // this message isn't all zeros when ignition is on
      ignition_cadillac = GET_BYTES_04(to_push) != 0;
    }
    // GM exception
    if ((addr == 0x1F1) && (len == 8)) {
      // Bit 5 is ignition "on"
      bool ignition_gm = ((GET_BYTE(to_push, 0) & 0x20) != 0);
      ignition_can = ignition_gm || ignition_cadillac;
    }
    // Tesla exception
    if ((addr == 0x348) && (len == 8)) {
      // GTW_status
      ignition_can = (GET_BYTE(to_push, 0) & 0x1) != 0;
    }
  }
}

// ********************* precondition logic *********************

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
#define PRECONDITION_NAV_OVERRIDE_US 90000000U  // 90 seconds
#define PRECONDITION_START_PHASE1_TICKS 3U // 4003 message
#define PRECONDITION_START_PHASE2_TICKS 3U // E007 message
#define PRECONDITION_STOP_PHASE1_TICKS 3U  // 0000 message
#define PRECONDITION_STOP_PHASE2_TICKS 3U  // E007 message
#define PRECONDITION_RETRY_US 10000000U  // 10 seconds
#define PRECONDITION_MAX_RETRIES 10U

void send_precondition_start_msg(uint8_t ticks_remaining) {
  CAN_FIFOMailBox_TypeDef packet;
  packet.RIR = ADDR_TO_RIR(0x0C7U);
  packet.RDTR = 8U;  // DLC = 8
  if (ticks_remaining > PRECONDITION_START_PHASE2_TICKS) {
    // send 0000004003000000 to 0x0C7
    // bytes 0-3: 00 00 00 40
    packet.RDLR = 0x40000000U;
    // bytes 4-7: 03 00 00 00
    packet.RDHR = 0x00000003U;
  } else {
    // send 000000E007000000 to 0x0C7
    // bytes 0-3: 00 00 00 E0
    packet.RDLR = 0xE0000000U;
    // bytes 4-7: 07 00 00 00
    packet.RDHR = 0x00000007U;
  }
  can_send(&packet, CAR_BUS, true);
}

void send_precondition_stop_msg(uint8_t ticks_remaining) {
  CAN_FIFOMailBox_TypeDef packet;
  packet.RIR = ADDR_TO_RIR(0x0C7U);
  packet.RDTR = 8U;
  if (ticks_remaining <= PRECONDITION_STOP_PHASE2_TICKS) {
    // send 000000E007000000 to 0x0C7
    // bytes 0-3: 00 00 00 E0
    packet.RDLR = 0xE0000000U;
    // bytes 4-7: 07 00 00 00
    packet.RDHR = 0x00000007U;
  } else {
    packet.RDLR = 0U;
    packet.RDHR = 0U;
  }
  can_send(&packet, CAR_BUS, true);
}

// intercept NAV messages on 0x4ED during the first 90s since preconditioning was requested
// respond with the same message but replace the last 3 bytes with 10 A0 00
void precondition_fwd_hook(CAN_FIFOMailBox_TypeDef *to_send, int bus_fwd_num) {
  if (!precondition_enabled || GET_ADDR(to_send) != 0x4EDU || bus_fwd_num != CAR_BUS) {
    return;
  }
  uint32_t elapsed = get_ts_elapsed(TIM2->CNT, precondition_requested_ts);
  if (elapsed > PRECONDITION_NAV_OVERRIDE_US) {
    return;
  }
  // modify bytes 5-7: byte5=0x10, byte6=0xA0, byte7=0x00
  // RDHR holds bytes 4-7: keep byte 4, replace bytes 5-7
  to_send->RDHR = (to_send->RDHR & 0xFFU) | (0x10U << 8) | (0xA0U << 16) | (0x00U << 24);
}

void precondition_can_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  // 0x2AD status frame: second byte indicates precondition state
  //   0x01 = off/idle, 0x05 = starting, 0x15 = fully running
  if (GET_ADDR(to_push) == 0x2ADU) {
    uint8_t status = GET_BYTE(to_push, 1);
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
  if (GET_ADDR(to_push) == 0x448U) {
    bool star_button = (GET_BYTE(to_push, 5) == 0x10U);
    if (star_button && !star_button_prev) {
      uint32_t now = TIM2->CNT;
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
  uint32_t now = TIM2->CNT;

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

// CAN receive handlers
// blink blue when we are receiving CAN messages
void can_rx(uint8_t can_number) {
  CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);
  while ((CAN->RF0R & CAN_RF0R_FMP0) != 0) {
    can_rx_cnt += 1;

    // can is live
    pending_can_live = 1;

    // add to my fifo
    CAN_FIFOMailBox_TypeDef to_push;
    to_push.RIR = CAN->sFIFOMailBox[0].RIR;
    to_push.RDTR = CAN->sFIFOMailBox[0].RDTR;
    to_push.RDLR = CAN->sFIFOMailBox[0].RDLR;
    to_push.RDHR = CAN->sFIFOMailBox[0].RDHR;

    // modify RDTR for our API
    to_push.RDTR = (to_push.RDTR & 0xFFFF000F) | (bus_number << 4);

    precondition_can_rx_hook(&to_push);

    // forwarding (panda only)
    int bus_fwd_num = (can_forwarding[bus_number] != -1) ? can_forwarding[bus_number] : safety_fwd_hook(bus_number, &to_push);
    if (bus_fwd_num != -1) {
      CAN_FIFOMailBox_TypeDef to_send;
      to_send.RIR = to_push.RIR | 1; // TXRQ
      to_send.RDTR = to_push.RDTR;
      to_send.RDLR = to_push.RDLR;
      to_send.RDHR = to_push.RDHR;

      precondition_fwd_hook(&to_send, bus_fwd_num);

      can_send(&to_send, bus_fwd_num, true);
    }

    can_rx_errs += safety_rx_hook(&to_push) ? 0U : 1U;
    ignition_can_hook(&to_push);

    current_board->set_led(LED_BLUE, true);
    can_send_errs += can_push(&can_rx_q, &to_push) ? 0U : 1U;

    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_TX_IRQ_Handler(void) { process_can(0); }
void CAN1_RX0_IRQ_Handler(void) { can_rx(0); }
void CAN1_SCE_IRQ_Handler(void) { can_sce(CAN1); }

void CAN2_TX_IRQ_Handler(void) { process_can(1); }
void CAN2_RX0_IRQ_Handler(void) { can_rx(1); }
void CAN2_SCE_IRQ_Handler(void) { can_sce(CAN2); }

void CAN3_TX_IRQ_Handler(void) { process_can(2); }
void CAN3_RX0_IRQ_Handler(void) { can_rx(2); }
void CAN3_SCE_IRQ_Handler(void) { can_sce(CAN3); }

bool can_tx_check_min_slots_free(uint32_t min) {
  return
    (can_slots_empty(&can_tx1_q) >= min) &&
    (can_slots_empty(&can_tx2_q) >= min) &&
    (can_slots_empty(&can_tx3_q) >= min) &&
    (can_slots_empty(&can_txgmlan_q) >= min);
}

void can_send(CAN_FIFOMailBox_TypeDef *to_push, uint8_t bus_number, bool skip_tx_hook) {
  if (skip_tx_hook || safety_tx_hook(to_push) != 0) {
    // disable bus 1/CAN2 since it's disconnected on our harness
    if (bus_number < BUS_MAX && bus_number != UNUSED_BUS) {
      // add CAN packet to send queue
      // bus number isn't passed through
      to_push->RDTR &= 0xF;
      if ((bus_number == 3U) && (can_num_lookup[3] == 0xFFU)) {
        gmlan_send_errs += bitbang_gmlan(to_push) ? 0U : 1U;
      } else {
        can_fwd_errs += can_push(can_queues[bus_number], to_push) ? 0U : 1U;
        process_can(CAN_NUM_FROM_BUS_NUM(bus_number));
      }
    }
  }
}

void can_set_forwarding(int from, int to) {
  can_forwarding[from] = to;
}

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
    CAN_TypeDef *CAN = CANIF_FROM_CAN_NUM(can_number);
    ret &= can_set_speed(can_number);
    ret &= llcan_init(CAN);
    // in case there are queued up messages
    process_can(can_number);
  }
  return ret;
}

