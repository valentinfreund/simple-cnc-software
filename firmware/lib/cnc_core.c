/*
 * cnc_core.c - board-independent machine logic (see cnc_core.h).
 *
 * Motion: every command frame carries a waypoint (a position per axis). The
 * board gives each waypoint a point in time on its own sample clock and moves
 * with constant speed from waypoint to waypoint. Two things make this exact:
 *
 *  - Clock recovery. The host sends one frame per servo period of *its*
 *    clock; the frames arrive with jitter. A slow PLL tracks the average
 *    arrival time and period, so waypoint times are evenly spaced and follow
 *    the host clock without following its jitter.
 *
 *  - Closed loop on the own position. For every piece that is rendered, the
 *    rate is "distance left to the waypoint / time left to the waypoint",
 *    computed from the exact position of the step generator. Rounding never
 *    accumulates and every waypoint is hit.
 *
 * Waypoint k is reached `delay` samples after its frame was expected:
 * one host period (the move itself) + the two blocks that are already in the
 * output buffer + a margin for frames that arrive late.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "cnc_core.h"
#include <string.h>

_Static_assert(CNCP_POS_PER_STEP == STEPGEN_POS_PER_STEP, "position units");

#define POS_UNIT      ((int64_t)1 << 24)   /* 1/256 step in 2^-32 steps */
#define WPQ           8                    /* waypoint ring size (power of 2) */
/* Clock recovery: a second-order PLL, critically damped (ki = kp^2 / 4). It
 * starts wide to lock quickly and narrows with every frame; the narrower it
 * is, the less of the host's jitter gets through (about sqrt(kp / 2)). */
#define PLL_KP_START  0.2
#define PLL_KP_MIN    0.01
#define PLL_KP_DECAY  0.99

typedef struct {
    double  t;                             /* sample time at which it is reached */
    int64_t p[CNCP_MAX_JOINTS];            /* position in 2^-32 steps            */
} waypoint_t;

static struct {
    cnc_core_cfg_t cfg;
    stepgen_t sg;
    uint8_t  state;
    uint16_t fault;
    uint16_t seq;
    uint8_t  enable_seen_low;    /* host sent ENABLE=0 since start-up / last fault */
    uint32_t wd_samples;         /* samples rendered since the last valid frame    */
    uint32_t wd_limit;           /* watchdog limit in samples                      */

    int64_t  t_next;             /* sample time at which the next rendered block starts playing */

    /* clock recovery */
    uint8_t  pll_locked;
    double   pll_t;              /* filtered arrival time of the latest frame      */
    double   pll_period;         /* filtered host period in samples                */
    double   pll_kp;             /* current loop gain                              */
    uint8_t  pll_outliers;       /* consecutive frames far off the expected time   */
    double   delay;              /* frame arrival -> waypoint reached, in samples  */

    /* waypoints */
    waypoint_t wp[WPQ];
    uint32_t wp_n;               /* number of waypoints pushed so far              */
    uint32_t wp_r;               /* index of the waypoint the renderer aims at     */
    int64_t  wp_pos[CNCP_MAX_JOINTS];     /* position of the newest waypoint       */
    int32_t  last_cmd[CNCP_MAX_JOINTS];   /* pos_cmd of the previous valid frame   */
    uint8_t  have_cmd;
    uint8_t  starving;
    uint8_t  late;
} core;

static int estop_active(uint32_t inputs)
{
    return core.cfg.estop_input >= 0 && ((inputs >> core.cfg.estop_input) & 1u);
}

static void enter_safe(uint8_t state, uint16_t fault)
{
    stepgen_stop_all(&core.sg);
    core.wp_r = core.wp_n;               /* forget pending waypoints */
    core.starving = 0;
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
    if (cfg->block_samples == 0 || cfg->host_period_samples == 0 ||
        stepgen_init(&core.sg, &cfg->stepgen) != 0)
        return -1;
    core.wd_limit = (uint32_t)((uint64_t)cfg->stepgen.sample_rate_hz * cfg->watchdog_ms / 1000u);
    core.t_next = 2 * (int64_t)cfg->block_samples;
    core.pll_period = cfg->host_period_samples;
    core.delay = (double)cfg->host_period_samples + 2.0 * cfg->block_samples + cfg->margin_samples;
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

/* ---- clock recovery ------------------------------------------------------ */

static void pll_update(double arrival)
{
    const double nominal = core.cfg.host_period_samples;

    if (core.pll_locked) {
        double e = arrival - (core.pll_t + core.pll_period);
        if (e > 0.5 * core.pll_period) {             /* frames were lost: skip them */
            double n = (double)(int64_t)(e / core.pll_period + 0.5);
            core.pll_t += n * core.pll_period;
            e -= n * core.pll_period;
        }
        if (e < 0.25 * core.pll_period && e > -0.25 * core.pll_period) {
            core.pll_t += core.pll_period + core.pll_kp * e;
            core.pll_period += 0.25 * core.pll_kp * core.pll_kp * e;
            if (core.pll_kp > PLL_KP_MIN)
                core.pll_kp *= PLL_KP_DECAY;
            if (core.pll_period > 1.02 * nominal) core.pll_period = 1.02 * nominal;
            if (core.pll_period < 0.98 * nominal) core.pll_period = 0.98 * nominal;
            core.pll_outliers = 0;
            return;
        }
        /* Far off the expected time: a latency spike of the host. Ignore it
         * and keep the schedule; only if it stays that way the host timing
         * has really moved. */
        if (++core.pll_outliers < 16) {
            core.pll_t += core.pll_period;
            return;
        }
    }
    /* first frame, or the host timing jumped for good: start over */
    core.pll_locked = 1;
    core.pll_t = arrival;
    core.pll_period = nominal;
    core.pll_kp = PLL_KP_START;
    core.pll_outliers = 0;
}

/* ---- waypoints --------------------------------------------------------------- */

static void waypoint_push(void)
{
    waypoint_t *w = &core.wp[core.wp_n % WPQ];
    double t = core.pll_t + core.delay;

    if (core.wp_n) {                                  /* keep the times increasing */
        double prev = core.wp[(core.wp_n - 1) % WPQ].t;
        if (t < prev + 1.0)
            t = prev + core.pll_period;
    }
    w->t = t;
    memcpy(w->p, core.wp_pos, sizeof(w->p));
    core.wp_n++;
    if (core.wp_n - core.wp_r > WPQ - 1)
        core.wp_r = core.wp_n - (WPQ - 1);
}

/* Where the plan says axis i should be at sample time t. Returns 0 if t is
 * outside of what the ring still knows. */
static int planned_position(unsigned i, double t, int64_t *out)
{
    uint32_t have = core.wp_n < WPQ ? core.wp_n : WPQ;

    for (uint32_t k = 0; k < have; k++) {
        const waypoint_t *b = &core.wp[(core.wp_n - 1 - k) % WPQ];
        if (t >= b->t) {
            if (k == 0) {                  /* past the newest waypoint */
                *out = b->p[i];
                return 1;
            }
            const waypoint_t *c = &core.wp[(core.wp_n - k) % WPQ];
            *out = b->p[i] + (int64_t)((double)(c->p[i] - b->p[i]) * (t - b->t) / (c->t - b->t));
            return 1;
        }
    }
    return 0;
}

/* ---- frames -------------------------------------------------------------------- */

void cnc_core_frame(const uint8_t *rx, uint8_t *tx)
{
    uint32_t inputs = cnc_board_read_inputs();
    unsigned played = cnc_board_samples_played();
    double now = (double)(core.t_next - 2 * (int64_t)core.cfg.block_samples) + (double)played;
    unsigned axes = core.sg.cfg.num_axes;
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
        pll_update(now);

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
                /* The first waypoint is where we are: no jump, whatever the
                 * host's position values look like. */
                for (unsigned i = 0; i < axes; i++)
                    core.wp_pos[i] = stepgen_pos_rendered(&core.sg, i);
                core.have_cmd = 0;
                core.state = CNCP_STATE_RUNNING;
                cnc_board_set_drive_enable(1);
            }
        }

        if (core.state == CNCP_STATE_RUNNING) {
            if (estop_active(inputs)) {
                enter_safe(CNCP_STATE_FAULT, CNCP_FAULT_ESTOP);
            } else {
                if (core.have_cmd)
                    for (unsigned i = 0; i < axes; i++)
                        core.wp_pos[i] += (int64_t)(int32_t)((uint32_t)cmd.pos_cmd[i] -
                                                             (uint32_t)core.last_cmd[i]) * POS_UNIT;
                waypoint_push();
                cnc_board_write_outputs(cmd.outputs);
                for (unsigned ch = 0; ch < CNCP_NUM_ANALOG; ch++)
                    cnc_board_write_analog(ch, cmd.analog[ch]);
            }
        }
        memcpy(core.last_cmd, cmd.pos_cmd, sizeof(core.last_cmd));
        core.have_cmd = 1;
    }

    cncp_status_t st;
    memset(&st, 0, sizeof(st));
    st.state = core.state;
    st.seq = core.seq;
    for (unsigned i = 0; i < axes; i++) {
        int64_t actual = stepgen_position_raw(&core.sg, i, played), plan;
        st.pos[i] = (int32_t)(uint32_t)((uint64_t)actual >> 24);
        if (core.state == CNCP_STATE_RUNNING && planned_position(i, now, &plan)) {
            int64_t e = (plan - actual) / POS_UNIT;
            st.err[i] = e > 32767 ? 32767 : e < -32768 ? -32768 : (int16_t)e;
        }
    }
    st.inputs = inputs;
    st.fault = core.fault;
    st.live = estop_active(inputs) ? CNCP_LIVE_ESTOP : 0;
    st.late = core.late;
    cncp_status_seal(&st);
    memcpy(tx, &st, sizeof(st));
}

/* ---- rendering -------------------------------------------------------------------- */

void cnc_core_render(uint32_t *buf)
{
    unsigned left = core.cfg.block_samples;
    unsigned axes = core.sg.cfg.num_axes;
    double t = (double)core.t_next;

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

    while (left) {
        unsigned n = left;

        if (core.state == CNCP_STATE_RUNNING) {
            /* aim at the first waypoint that is at least one sample ahead */
            while (core.wp_r != core.wp_n && core.wp[core.wp_r % WPQ].t < t + 1.0)
                core.wp_r++;
            if (core.wp_r != core.wp_n) {
                const waypoint_t *w = &core.wp[core.wp_r % WPQ];
                double dt = w->t - t;
                if (dt < (double)n)
                    n = (unsigned)dt;
                for (unsigned i = 0; i < axes; i++) {
                    int64_t dist = w->p[i] - stepgen_pos_rendered(&core.sg, i);
                    if (dist < POS_UNIT && dist > -POS_UNIT)
                        dist = 0;                    /* there: do not dither */
                    stepgen_set_inc(&core.sg, i, (int64_t)((double)dist / dt));
                }
                core.starving = 0;
            } else if (!core.starving && core.wp_n) {
                /* The next command is late: keep the current speed. When it
                 * arrives, the rate is computed from the real position, so
                 * nothing is lost. */
                core.starving = 1;
                core.late++;
            }
        }
        stepgen_render(&core.sg, buf, n);
        buf += n;
        t += n;
        left -= n;
    }
    stepgen_end_block(&core.sg, core.cfg.block_samples);
    core.t_next += core.cfg.block_samples;
}
