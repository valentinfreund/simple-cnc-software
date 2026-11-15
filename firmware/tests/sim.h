/*
 * sim.h - model of a board port for host-side tests: a fake board plus the
 * same double-buffer pipeline the real firmware uses (a rendered block is
 * output one block later). The including file provides sim_on_sample().
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef SIM_H
#define SIM_H

#include <stdint.h>
#include "cnc_core.h"

#define FS      500000u
#define BLOCK   250u

static void sim_on_sample(uint32_t word);      /* called for every output sample */

static uint32_t fake_inputs, fake_outputs;
static int fake_enable;
static uint32_t sim_buf[2 * BLOCK];
static unsigned sim_idx;
static unsigned sim_last_half = 1;
static int sim_feedback_at_buffer_end;         /* 1 = emulate un-interpolated feedback */

uint32_t cnc_board_read_inputs(void) { return fake_inputs; }
void cnc_board_write_outputs(uint32_t o) { fake_outputs = o; }
void cnc_board_write_analog(unsigned ch, uint16_t v) { (void)ch; (void)v; }
void cnc_board_set_drive_enable(int on) { fake_enable = on; }

unsigned cnc_board_samples_played(void)
{
    unsigned half = sim_idx / BLOCK, off = sim_idx % BLOCK;
    if (sim_feedback_at_buffer_end)
        return 2 * BLOCK;
    return half != sim_last_half ? off : BLOCK + off;
}

static void sim_tick(void)
{
    sim_on_sample(sim_buf[sim_idx]);
    sim_idx++;
    if (sim_idx == BLOCK) {
        cnc_core_render(&sim_buf[0]);
        sim_last_half = 0;
    } else if (sim_idx == 2 * BLOCK) {
        sim_idx = 0;
        cnc_core_render(&sim_buf[BLOCK]);
        sim_last_half = 1;
    }
}

static void sim_advance(unsigned samples)
{
    while (samples--)
        sim_tick();
}

#endif /* SIM_H */
