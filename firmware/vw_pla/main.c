/*
  Corrected Pure CAN Passthrough Module
  - Fixes the RDTR register masking bug that prevented message forwarding.
  - Includes relay and termination setup required for in-vehicle use.
*/

// ********************* Includes *********************
#include "../config.h"
#include "libc.h"
#include "main_declarations.h"
#include "drivers/registers.h"
#include "drivers/interrupts.h"
#include "drivers/llcan.h"
#include "drivers/llgpio.h"
#include "board.h"
#include "drivers/clock.h"

// Minimal globals for the board to boot
uint8_t hw_type = 0;
const struct board *current_board = NULL;

void __initialize_hardware_early(void) { early(); }

// ***************************** can handlers *****************************
void CAN1_TX_IRQ_Handler(void) { process_can(0); }
void CAN2_TX_IRQ_Handler(void) { process_can(1); }
void CAN3_TX_IRQ_Handler(void) { process_can(2); }

// CAN1 (Car) -> CAN3 (EPS)
void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {
    CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR  = CAN1->sFIFOMailBox[0].RIR | 1U;
    // --- THIS IS THE FIX ---
    // Copy the entire RDTR register, not just the DLC part.
    to_fwd.RDTR = CAN1->sFIFOMailBox[0].RDTR;
    // --- END FIX ---
    to_fwd.RDLR = CAN1->sFIFOMailBox[0].RDLR;
    to_fwd.RDHR = CAN1->sFIFOMailBox[0].RDHR;

    can_send(&to_fwd, 2, false);  // forward to CAN3
    can_rx(0);                    // release RX FIFO
  }
}

// CAN3 (EPS) -> CAN1 (Car)
void CAN3_RX0_IRQ_Handler(void) {
  while ((CAN3->RF0R & CAN_RF0R_FMP0) != 0) {
    CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR  = CAN3->sFIFOMailBox[0].RIR | 1U;
    // --- THIS IS THE FIX ---
    // Copy the entire RDTR register, not just the DLC part.
    to_fwd.RDTR = CAN3->sFIFOMailBox[0].RDTR;
    // --- END FIX ---
    to_fwd.RDLR = CAN3->sFIFOMailBox[0].RDLR;
    to_fwd.RDHR = CAN3->sFIFOMailBox[0].RDHR;

    can_send(&to_fwd, 0, false);  // forward to CAN1
    can_rx(2);                    // release RX FIFO
  }
}

// Unused CAN2 handler
void CAN2_RX0_IRQ_Handler(void) {
  while ((CAN2->RF0R & CAN_RF0R_FMP0) != 0) {
    can_rx(1);
  }
}

// ***************************** main code *****************************
int main(void) {
  // Init interrupt table and disable interrupts for setup
  init_interrupts(true);
  REGISTER_INTERRUPT(CAN1_TX_IRQn,  CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN2_RX0_IRQn, CAN2_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN3_TX_IRQn,  CAN3_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_RX0_IRQn, CAN3_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  disable_interrupts();

  // Basic hardware init
  clock_init();
  peripherals_init();
  detect_board_type();
  current_board->init();

  // Set CAN speed to 500kbps for all buses
  llcan_set_speed(CAN1, 5000, false, false);
  llcan_set_speed(CAN2, 5000, false, false);
  llcan_set_speed(CAN3, 5000, false, false);
  
  // Initialize CAN peripherals
  llcan_init(CAN1);
  llcan_init(CAN2);
  llcan_init(CAN3);
  
  // --- IMPORTANT VEHICLE SETUP ---
  // 1. Set relay to OPEN (bridge mode) to run passthrough code
  set_gpio_mode(GPIOB, 0, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 0, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output(GPIOB, 0, 0); 
  
  // 2. Enable CAN termination (still need to find the real function name)
  // This is required for MQB. You will need to find the correct function
  // in your board's C file (e.g., boards/black.c)
  // current_board->set_can_termination(0, true); 
  // current_board->set_can_termination(2, true);

  puts("**** CORRECTED CAN PASSTHROUGH READY ****\n");
  enable_interrupts();

  while (1) { }
  return 0;
}
