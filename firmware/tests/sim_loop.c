/*
 * sim_loop.c - timing simulation of the whole chain: LinuxCNC sends one
 * position command per servo period (1 kHz, with clock offset and jitter)
 * through the real protocol into the board library and its output pipeline.
 * Prints how far the pulses on the pins are from the commanded trajectory.
 * Run with `make -C firmware/tests sim`.
 *
 * Move: 0 -> 100 mm/s with 1000 mm/s^2, cruise, stop. 800 steps/mm.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim.h"

#define SCALE      800.0     /* steps per mm */
#define VMAX       100.0     /* mm/s */
#define ACC        1000.0    /* mm/s^2 */
#define T_CRUISE   0.4       /* s */
#define FRAME_END  210       /* samples from cycle start to end of SPI frame (0.42 ms) */

static uint16_t port;
static long pin_pos;         /* steps seen on the step/dir pins of axis 0 */

static void sim_on_sample(uint32_t w)
{
    uint16_t old = port;
    port = (uint16_t)((port | (w & 0xFFFF)) & ~(w >> 16));
    if ((port & 1) && !(old & 1))
        pin_pos += (port & 0x10) ? 1 : -1;
}

/* commanded position in mm at time t (trapezoidal velocity profile) */
static double traj(double t)
{
    double ta = VMAX / ACC, xa = 0.5 * ACC * ta * ta;
    if (t <= 0) return 0;
    if (t < ta) return 0.5 * ACC * t * t;
    if (t < ta + T_CRUISE) return xa + VMAX * (t - ta);
    if (t < 2 * ta + T_CRUISE) { double r = 2 * ta + T_CRUISE - t; return 2 * xa + VMAX * T_CRUISE - 0.5 * ACC * r * r; }
    return 2 * xa + VMAX * T_CRUISE;
}

static void run(const char *name, double clock_offset, int jitter_samples, int spike_every, int lose_every)
{
    cnc_core_cfg_t cfg = {
        .stepgen = { .sample_rate_hz = FS, .num_axes = 1, .step_len = 2, .step_space = 2,
                     .dir_setup = 5, .axis = { { .step_pin = 0, .dir_pin = 4 } } },
        .block_samples = BLOCK, .host_period_samples = 500, .margin_samples = 100,
        .watchdog_ms = 10, .estop_input = -1,
    };
    enum { N = 2900, START = 2000 };          /* the host idles 2 s before the move */
    static double x_true[N];
    uint8_t tx[CNCP_FRAME_SIZE], rx_host[CNCP_FRAME_SIZE];
    double host_ms = 500.0 * (1.0 + clock_offset);     /* host millisecond in board samples */
    double ferr_max = 0;
    long now = 0;
    uint16_t seq = 0;
    unsigned late = 0;

    port = 0; pin_pos = 0; sim_idx = 0; sim_last_half = 1;
    memset(sim_buf, 0, sizeof(sim_buf));
    srand(1);
    cnc_core_init(&cfg);
    sim_on_sample(cnc_core_idle_sample());
    cnc_core_frame(NULL, tx);

    for (int k = 0; k < N; k++) {
        long t_nom = (long)(k * host_ms);
        sim_advance((unsigned)(t_nom - now)); now = t_nom;
        x_true[k] = pin_pos / SCALE;

        /* the servo thread starts late by a random amount (host latency) */
        int j = jitter_samples ? rand() % (jitter_samples + 1) : 0;
        if (spike_every && k % spike_every == spike_every - 1)
            j = 200;                                   /* a 400 us latency spike */
        sim_advance((unsigned)(j + FRAME_END)); now += j + FRAME_END;

        cncp_cmd_t c; cncp_status_t st;
        memset(&c, 0, sizeof(c));
        c.flags = k >= 5 ? CNCP_CMD_ENABLE : 0;
        c.seq = ++seq;
        c.pos_cmd[0] = (int32_t)llround(traj((k - START) * 0.001) * SCALE * CNCP_POS_PER_STEP);
        cncp_cmd_seal(&c);
        if (lose_every && k % lose_every == lose_every - 1)
            c.seq ^= 1;                                /* frame damaged on the wire */
        memcpy(rx_host, tx, sizeof(tx));
        cnc_core_frame((const uint8_t *)&c, tx);
        memcpy(&st, rx_host, sizeof(st));              /* the host sees the previous status */
        double ferr = fabs(st.err[0] / (CNCP_POS_PER_STEP * SCALE));
        if (k > START && ferr > ferr_max) ferr_max = ferr;
        late = st.late;
    }

    /* cruise: constant lag = pure time shift; its variation is what matters */
    int c0 = START + 150, c1 = START + (int)(1000 * (VMAX / ACC + T_CRUISE)) - 20;
    double lag_sum = 0, lag_min = 1e9, lag_max = -1e9;
    for (int k = c0; k < c1; k++) {
        double lag = traj((k - START) * 0.001) - x_true[k];
        lag_sum += lag; if (lag < lag_min) lag_min = lag; if (lag > lag_max) lag_max = lag;
    }
    double shift = lag_sum / (c1 - c0) / VMAX;                /* seconds */

    /* whole move: deviation from the time-shifted command */
    double dyn_max = 0;
    for (int k = START; k < N; k++) {
        double e = fabs(x_true[k] - traj((k - START) * 0.001 - shift));
        if (e > dyn_max) dyn_max = e;
    }
    printf("%-38s %6.2f ms %8.2f um %9.2f um %9.2f um %6u\n", name, shift * 1e3,
           (lag_max - lag_min) * 1e3, dyn_max * 1e3, ferr_max * 1e3, late);
}

int main(void)
{
    printf("%-38s %9s %11s %12s %12s %6s\n", "100 mm/s, 1000 mm/s^2, 1.25 um/step",
           "delay", "ripple p-p", "max. error", "f-error", "late");
    run("ideal host",                          20e-6, 0,   0,  0);
    run("host jitter 0..50 us",                20e-6, 25,  0,  0);
    run("host jitter 0..200 us",               20e-6, 100, 0,  0);
    run("jitter 0..50 us + 400 us spikes",     20e-6, 25,  97, 0);
    run("jitter 0..50 us, 1 frame in 50 lost", 20e-6, 25,  0,  50);
    run("board clock 1 % off, jitter 0..50 us", 0.01, 25,  0,  0);
    printf("\ndelay      : constant delay of the whole trajectory (same for all axes)\n"
           "ripple p-p : variation of the position during cruise, includes +-1 step quantisation\n"
           "max. error : largest deviation from the delayed command over the whole move\n"
           "f-error    : largest following error LinuxCNC sees (reported by the board)\n"
           "late       : commands that arrived too late to be executed on time\n");
    return 0;
}
