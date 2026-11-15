/*
 * cnc_core.c - board-independent machine logic (see cnc_core.h).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "cnc_core.h"
#include <string.h>

_Static_assert(CNCP_POS_PER_STEP == STEPGEN_POS_PER_STEP, "position units");

static struct {
    cnc_core_cfg_t cfg;
    stepgen_t sg;
    uint8_t  state;
    uint16_t fault;
    uint16_t seq;
    uint8_t  enable_seen_low;    /* host sent ENABLE=0 since start-up / last fault */
    uint32_t wd_samples;         /* samples rendered since the last valid frame    */
    uint32_t wd_limit;           /* watchdog limit in samples                      */
} core;

static int estop_active(uint32_t inputs)
{
    return core.cfg.estop_input >= 0 && ((inputs >> core.cfg.estop_input) & 1u);
}

static void enter_safe(uint8_t state, uint16_t fault)
{
    stepgen_stop_all(&core.sg);
    cnc_board_set_drive_enable(0);
    cnc_board_write_outputs(0);
    for (unsigned ch = 0; ch < CNCP_NUM_ANALOG; ch++)
        cnc_board_write_analog(ch, 0);
    core.state = state;
    core.fault |= fault;
    core.enable_seen_low = 0;
}

int cnc_core_init(const cnc_core_cfg_t *cfg)
{
    memset(&core, 0, sizeof(core));
    core.cfg = *cfg;
    if (cfg->block_samples == 0 || stepgen_init(&core.sg, &cfg->stepgen) != 0)
        return -1;
    core.wd_limit = (uint32_t)((uint64_t)cfg->stepgen.sample_rate_hz * cfg->watchdog_ms / 1000u);
    enter_safe(CNCP_STATE_DISABLED, 0);
    return 0;
}

uint32_t cnc_core_idle_sample(void)
{
    return stepgen_idle_sample(&core.sg);
}

uint8_t cnc_core_state(void)
{
    return core.state;
}

void cnc_core_frame(const uint8_t *rx, uint8_t *tx)
{
    uint32_t inputs = cnc_board_read_inputs();
    cncp_cmd_t cmd;
    int valid = 0;

    if (rx) {
        memcpy(&cmd, rx, sizeof(cmd));
        valid = cncp_cmd_valid(&cmd);
    }

    if (valid) {
        int enable = (cmd.flags & CNCP_CMD_ENABLE) != 0;
        core.seq = cmd.seq;
        core.wd_samples = 0;

        if (!enable) {
            /* ENABLE low: leave RUNNING, and acknowledge a latched fault. */
            if (core.state != CNCP_STATE_DISABLED)
                enter_safe(CNCP_STATE_DISABLED, 0);
            core.fault = 0;
            core.enable_seen_low = 1;
        } else if (core.state == CNCP_STATE_DISABLED && core.enable_seen_low) {
            /* Rising edge of ENABLE: start, unless the e-stop is active. The
             * edge is consumed either way, so releasing the e-stop later can
             * never start the machine by itself. */
            core.enable_seen_low = 0;
            if (!estop_active(inputs)) {
                core.state = CNCP_STATE_RUNNING;
                cnc_board_set_drive_enable(1);
            }
        }

        if (core.state == CNCP_STATE_RUNNING) {
            if (estop_active(inputs)) {
                enter_safe(CNCP_STATE_FAULT, CNCP_FAULT_ESTOP);
            } else {
                for (unsigned i = 0; i < core.sg.cfg.num_axes; i++)
                    stepgen_set_velocity(&core.sg, i, cmd.vel_mhz[i]);
                cnc_board_write_outputs(cmd.outputs);
                for (unsigned ch = 0; ch < CNCP_NUM_ANALOG; ch++)
                    cnc_board_write_analog(ch, cmd.analog[ch]);
            }
        }
    }

    cncp_status_t st;
    memset(&st, 0, sizeof(st));
    st.state = core.state;
    st.seq = core.seq;
    unsigned played = cnc_board_samples_played();
    for (unsigned i = 0; i < core.sg.cfg.num_axes; i++)
        st.pos[i] = stepgen_position(&core.sg, i, played);
    st.inputs = inputs;
    st.fault = core.fault;
    st.live = estop_active(inputs) ? CNCP_LIVE_ESTOP : 0;
    cncp_status_seal(&st);
    memcpy(tx, &st, sizeof(st));
}

void cnc_core_render(uint32_t *buf)
{
    if (core.state == CNCP_STATE_RUNNING) {
        /* The e-stop and the watchdog are checked here as well, so they work
         * even when the host has stopped talking to us. */
        if (estop_active(cnc_board_read_inputs())) {
            enter_safe(CNCP_STATE_FAULT, CNCP_FAULT_ESTOP);
        } else {
            core.wd_samples += core.cfg.block_samples;
            if (core.wd_samples > core.wd_limit)
                enter_safe(CNCP_STATE_FAULT, CNCP_FAULT_COMM_TIMEOUT);
        }
    }
    stepgen_render(&core.sg, buf, core.cfg.block_samples);
}
