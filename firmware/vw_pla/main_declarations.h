#ifndef MAIN_DECLARATIONS_H
#define MAIN_DECLARATIONS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// -------- Console helpers (provided by your lightweight libc) --------
void puts(const char *a);
void puth(unsigned int i);
void puth2(unsigned int i);

// -------- Forward declarations --------
// Forward typedefs needed by board_declarations.h before harness.h is included
typedef struct board board;
typedef struct harness_configuration harness_configuration;


// Utilities you call from main (implemented elsewhere in your tree)
void can_flip_buses(uint8_t bus1, uint8_t bus2);
void can_set_obd(uint8_t harness_orientation, bool obd);

// -------- Globals (declared here, defined once in main.c) --------
extern uint8_t hw_type;
extern const board *current_board;
extern volatile bool is_enumerated;
extern volatile uint32_t heartbeat_counter;
extern volatile uint32_t uptime_cnt;
extern volatile bool siren_enabled;
extern volatile bool green_led_enabled;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_DECLARATIONS_H */
