/*
Pure CAN Passthrough Module for Testing
This firmware simply forwards ALL messages bidirectionally between:
- CAN1 (car side) <-> CAN3 (EPS side)
No modifications, filtering, or processing. Used for testing basic Ocelot connectivity.
*/

// ********************* Includes *********************
#include "../config.h"
#include "libc.h"

#include "main_declarations.h"
#include "critical.h"
#include "faults.h"

#include "drivers/registers.h"
#include "drivers/interrupts.h"
#include "drivers/llcan.h"
#include "drivers/llgpio.h"
#include "drivers/adc.h"

#include "board.h"

#include "drivers/clock.h"
#include "drivers/timer.h"

#include "gpio.h"

#include "drivers/uart.h"
#include "drivers/usb.h"

// Guarded MIN (needed before first use)
#ifndef MIN
#define MIN(a,b) (( (a) < (b) ) ? (a) : (b))
#endif

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// Define globals declared in main_declarations.h
uint8_t hw_type = 0;
const struct board *current_board = NULL;
volatile bool is_enumerated = false;
volatile uint32_t heartbeat_counter = 0;
volatile uint32_t uptime_cnt = 0;
volatile bool siren_enabled = false;
volatile bool green_led_enabled = false;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

#include "vw_pla/can.h"

// ---------- helpers to ensure PHYs are awake/normal ----------
static void force_can_online(void) {
  // make sure library silent mode is off
  can_silent = ALL_CAN_LIVE;

  if (current_board != NULL) {
    if (current_board->set_can_mode) {
      #ifndef CAN_MODE_NORMAL
      #define CAN_MODE_NORMAL 0
      #endif
      current_board->set_can_mode(CAN_MODE_NORMAL);
    }
    if (current_board->enable_can_transceivers) {
      current_board->enable_can_transceivers(true);
    }
    if (current_board->enable_can_transceiver) {
      // enable the bridge sides explicitly
      current_board->enable_can_transceiver(0, true); // CAN1
      current_board->enable_can_transceiver(2, true); // CAN3
      // keep CAN2 off
      current_board->enable_can_transceiver(1, false);
    }
  }
}

// ********************* usb debugging *********************
void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

int usb_cb_ep1_in(void *usbdata, int len, bool hardwired) {
  UNUSED(hardwired);
  CAN_FIFOMailBox_TypeDef *reply = (CAN_FIFOMailBox_TypeDef *)usbdata;
  int ilen = 0;
  while (ilen < MIN(len/0x10, 4) && can_pop(&can_rx_q, &reply[ilen])) {
    ilen++;
  }
  return ilen*0x10;
}

void usb_cb_ep2_out(void *usbdata, int len, bool hardwired) {
  UNUSED(hardwired);
  uint8_t *usbdata8 = (uint8_t *)usbdata;
  uart_ring *ur = get_ring_by_number(usbdata8[0]);
  if ((len != 0) && (ur != NULL)) {
    if ((usbdata8[0] < 2U)) {
      for (int i = 1; i < len; i++) {
        while (!putc(ur, usbdata8[i])) {
          // wait
        }
      }
    }
  }
}

void usb_cb_ep3_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}

void usb_cb_ep3_out_complete() {
  if (can_tx_check_min_slots_free(MAX_CAN_MSGS_PER_BULK_TRANSFER)) {
    usb_outep3_resume_if_paused();
  }
}

void usb_cb_enumeration_complete() {
  puts("USB enumeration complete\n");
  is_enumerated = true;
}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired) {
  UNUSED(hardwired);
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest) {
    // **** 0xd1: enter bootloader mode
    case 0xd1:
      switch (setup->b.wValue.w) {
        case 0:
          #ifdef ALLOW_DEBUG
            if (hardwired) {
              puts("-> entering bootloader\n");
              enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
              NVIC_SystemReset();
            }
          #endif
          break;
        case 1:
          puts("-> entering softloader\n");
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
          break;
        default:
          puts("Bootloader mode invalid\n");
          break;
      }
      break;
    // **** 0xd8: reset ST
    case 0xd8:
      NVIC_SystemReset();
      break;
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) break;
      while ((resp_len < MIN(setup->b.wLength.w, MAX_RESP_LEN)) &&
             getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    // **** 0xf1: Clear CAN ring buffer.
    case 0xf1:
      if (setup->b.wValue.w == 0xFFFFU) {
        puts("Clearing CAN Rx queue\n");
        can_clear(&can_rx_q);
      } else if (setup->b.wValue.w < BUS_MAX) {
        puts("Clearing CAN Tx queue\n");
        can_clear(can_queues[setup->b.wValue.w]);
      } else {
        puts("Clearing CAN CAN ring buffer failed: wrong bus number\n");
      }
      break;
    // **** 0xf2: Clear UART ring buffer.
    case 0xf2: {
        uart_ring * rb = get_ring_by_number(setup->b.wValue.w);
        if (rb != NULL) {
          puts("Clearing UART queue.\n");
          clear_uart_buff(rb);
        }
        break;
      }
    default:
      puts("NO HANDLER "); puth(setup->b.bRequest); puts("\n");
      break;
  }
  return resp_len;
}

// ***************************** can handlers *****************************
void CAN1_TX_IRQ_Handler(void) { process_can(0); }
void CAN2_TX_IRQ_Handler(void) { process_can(1); }
void CAN3_TX_IRQ_Handler(void) { process_can(2); }

// CAN1 (Car) -> CAN3 (EPS)
void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {
    CAN_FIFOMailBox_TypeDef to_fwd;
    // Set TXRQ here so transmission is guaranteed
    to_fwd.RIR  = CAN1->sFIFOMailBox[0].RIR | 1U;
    to_fwd.RDTR = CAN1->sFIFOMailBox[0].RDTR & 0xFU;   // DLC only
    to_fwd.RDLR = CAN1->sFIFOMailBox[0].RDLR;
    to_fwd.RDHR = CAN1->sFIFOMailBox[0].RDHR;

    can_send(&to_fwd, 2, false);  // forward to CAN3
    can_rx(0);                    // release RX FIFO
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  can_sce(CAN1);
  llcan_clear_send(CAN1);
}

void CAN2_RX0_IRQ_Handler(void) {
  // CAN2 not used in this passthrough
  while ((CAN2->RF0R & CAN_RF0R_FMP0) != 0) {
    can_rx(1);
  }
}

void CAN2_SCE_IRQ_Handler(void) {
  can_sce(CAN2);
  llcan_clear_send(CAN2);
}

// CAN3 (EPS) -> CAN1 (Car)
void CAN3_RX0_IRQ_Handler(void) {
  while ((CAN3->RF0R & CAN_RF0R_FMP0) != 0) {
    CAN_FIFOMailBox_TypeDef to_fwd;
    // Set TXRQ here so transmission is guaranteed
    to_fwd.RIR  = CAN3->sFIFOMailBox[0].RIR | 1U;
    to_fwd.RDTR = CAN3->sFIFOMailBox[0].RDTR & 0xFU;   // DLC only
    to_fwd.RDLR = CAN3->sFIFOMailBox[0].RDLR;
    to_fwd.RDHR = CAN3->sFIFOMailBox[0].RDHR;

    can_send(&to_fwd, 0, false);  // forward to CAN1
    can_rx(2);                    // release RX FIFO
  }
}

void CAN3_SCE_IRQ_Handler(void) {
  can_sce(CAN3);
  llcan_clear_send(CAN3);
}

// ***************************** main code *****************************

void loop(void) {
  // Empty loop - pure passthrough requires no processing
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  // Register CAN interrupts
  REGISTER_INTERRUPT(CAN1_TX_IRQn,  CAN1_TX_IRQ_Handler,  CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN2_TX_IRQn,  CAN2_TX_IRQ_Handler,  CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN2_RX0_IRQn, CAN2_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN2_SCE_IRQn, CAN2_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN3_TX_IRQn,  CAN3_TX_IRQ_Handler,  CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_RX0_IRQn, CAN3_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_SCE_IRQn, CAN3_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)

  disable_interrupts();

  // Init hardware
  clock_init();
  peripherals_init();
  detect_configuration();
  detect_board_type();

  // Init board
  current_board->init();

  // Force CAN PHYs awake/normal
  force_can_online();

  // Enable USB for debugging
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  usb_init();

  // Init ADC (required for board detection)
  adc_init();

  // Ensure controller is not in silent/listen-only (variable used by can_set_speed)
  can_silent = ALL_CAN_LIVE;

  // Init CAN buses at 500 kbps (5000 deci-kbps)
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) puts("Failed to set CAN1 speed\n");
  llcan_speed_set = llcan_set_speed(CAN2, 5000, false, false);
  if (!llcan_speed_set) puts("Failed to set CAN2 speed\n");
  llcan_speed_set = llcan_set_speed(CAN3, 5000, false, false);
  if (!llcan_speed_set) puts("Failed to set CAN3 speed\n");

  bool ret = llcan_init(CAN1); UNUSED(ret);
  ret = llcan_init(CAN2);      UNUSED(ret);
  ret = llcan_init(CAN3);      UNUSED(ret);

  // Set relay CLOSED (1) to match your working PQ bypass behavior initially
  set_gpio_mode(GPIOB, 0, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 0, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output(GPIOB, 0, 1);   // 1=closed (bypass), 0=open (bridge) -- flip if your HW is inverted

  puts("**** PURE CAN PASSTHROUGH READY ****\n");
  puts("**** CAN1 <-> CAN3 BIDIRECTIONAL ****\n");
  enable_interrupts();

  while (1) {
    loop();
  }

  return 0;
}
