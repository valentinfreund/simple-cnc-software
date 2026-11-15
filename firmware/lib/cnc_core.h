/*
 * cnc_core.h - board-independent machine logic of the real-time board.
 *
 * Owns the protocol handling, the state machine (disabled / running / fault),
 * the communication watchdog and the step generator. A board port only has to
 *   - call cnc_core_frame() whenever a frame from the host has arrived,
 *   - call cnc_core_render() whenever the next block of step samples is due,
 *   - implement the four cnc_board_*() functions below.
 *
 * cnc_core_frame() and cnc_core_render() must not preempt each other (run
 * them from interrupts of the same priority, or from one loop).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CNC_CORE_H
#define CNC_CORE_H

#include <stdint.h>
#include "cncproto.h"
#include "stepgen.h"

typedef struct {
    stepgen_cfg_t stepgen;
    uint16_t block_samples;       /* samples rendered per cnc_core_render() call   */
    uint16_t watchdog_ms;         /* max. time without a valid frame while running */
    int8_t   estop_input;         /* input bit that is 1 while e-stop is active, -1 = none */
} cnc_core_cfg_t;

/* --- implemented by the board port ------------------------------------- */
uint32_t cnc_board_read_inputs(void);                 /* bit n = input n, 1 = active     */
void     cnc_board_write_outputs(uint32_t outputs);   /* bit n = output n                */
void     cnc_board_write_analog(unsigned ch, uint16_t value);
void     cnc_board_set_drive_enable(int on);          /* servo-on signal for all drives  */

/* --- called by the board port ------------------------------------------ */
int  cnc_core_init(const cnc_core_cfg_t *cfg);
uint32_t cnc_core_idle_sample(void);

/* Handle one received frame and write the next status frame to tx.
 * rx may be NULL if the transfer failed (tx is still refreshed). */
void cnc_core_frame(const uint8_t *rx, uint8_t *tx);

/* Render the next block_samples step samples; also runs the watchdog. */
void cnc_core_render(uint32_t *buf);

uint8_t cnc_core_state(void);

#endif /* CNC_CORE_H */
