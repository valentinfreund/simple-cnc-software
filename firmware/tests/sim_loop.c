/*
 * sim_loop.c - closed-loop timing simulation: LinuxCNC position loop (pid
 * with FF1/FF2, 1 kHz) driving the board library through the real protocol
 * and the real output pipeline. Prints how far the pulses on the pins are
 * from the commanded trajectory. Run with `make -C firmware/tests sim`.
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
#define FRAME_END  175       /* samples from cycle start to end of SPI frame (0.35 ms) */

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

static void run(const char *name, double drift, int jitter_samples, double pgain, double ff2, int old_feedback)
{
    cnc_core_cfg_t cfg = {
        .stepgen = { .sample_rate_hz = FS, .num_axes = 1, .step_len = 2, .step_space = 2,
                     .dir_setup = 5, .axis = { { .step_pin = 0, .dir_pin = 4 } } },
        .block_samples = BLOCK, .watchdog_ms = 10, .estop_input = -1,
    };
    enum { N = 900, START = 50 };
    static double x_true[N], cmd_at[N];
    uint8_t tx[CNCP_FRAME_SIZE], rx_host[CNCP_FRAME_SIZE];
    double host_ms = 500.0 * (1.0 + drift);      /* host millisecond in board samples */
    double fb = 0, cmd_prev = 0, cmd_d_prev = 0, ferr_max = 0;
    long now = 0;
    uint16_t seq = 0;

    port = 0; pin_pos = 0; sim_idx = 0; sim_last_half = 1;
    memset(sim_buf, 0, sizeof(sim_buf));
    sim_feedback_at_buffer_end = old_feedback;
    srand(1);
    cnc_core_init(&cfg);
    sim_on_sample(cnc_core_idle_sample());
    cnc_core_frame(NULL, tx);

    for (int k = 0; k < N; k++) {
        long t_nom = (long)(k * host_ms);
        sim_advance((unsigned)(t_nom - now)); now = t_nom;
        x_true[k] = pin_pos / SCALE;
        cmd_at[k] = traj((k - START) * 0.001);

        /* servo thread starts late by a random amount (host latency) */
        int j = jitter_samples ? rand() % (jitter_samples + 1) : 0;
        sim_advance((unsigned)j); now += j;

        /* LinuxCNC pid: error against the previous target, FF on the new one */
        double cmd = cmd_at[k];
        double cmd_d = (cmd - cmd_prev) / 0.001;
        double err = cmd_prev - fb;
        double out = pgain * err + cmd_d + ff2 * (cmd_d - cmd_d_prev) / 0.001;
        if (k > START && fabs(err) > ferr_max) ferr_max = fabs(err);
        cmd_prev = cmd; cmd_d_prev = cmd_d;

        /* SPI frame: the host gets the status the board prepared one frame ago */
        sim_advance(FRAME_END); now += FRAME_END;
        cncp_cmd_t c; cncp_status_t st;
        memset(&c, 0, sizeof(c));
        c.flags = k >= 5 ? CNCP_CMD_ENABLE : 0;
        c.seq = ++seq;
        c.vel_mhz[0] = (int32_t)(out * SCALE * 1000.0);
        cncp_cmd_seal(&c);
        memcpy(rx_host, tx, sizeof(tx));
        cnc_core_frame((const uint8_t *)&c, tx);
        memcpy(&st, rx_host, sizeof(st));
        fb = st.pos[0] / (CNCP_POS_PER_STEP * SCALE);
    }

    /* cruise: constant lag = pure time shift; its ripple is the jitter */
    int c0 = START + 150, c1 = START + (int)(1000 * (VMAX / ACC + T_CRUISE)) - 20;
    double lag_sum = 0, lag_min = 1e9, lag_max = -1e9;
    for (int k = c0; k < c1; k++) {
        double lag = cmd_at[k] - x_true[k];
        lag_sum += lag; if (lag < lag_min) lag_min = lag; if (lag > lag_max) lag_max = lag;
    }
    double lag = lag_sum / (c1 - c0), shift = lag / VMAX;     /* seconds */

    /* whole move: deviation from the time-shifted command = what is left
     * after the shift, i.e. what could show up as contour error */
    double dyn_max = 0;
    for (int k = START; k < N; k++) {
        double e = fabs(x_true[k] - traj((k - START) * 0.001 - shift));
        if (e > dyn_max) dyn_max = e;
    }
    printf("%-44s %7.2f ms %8.1f um %9.1f um %9.1f um\n", name, shift * 1e3,
           (lag_max - lag_min) * 1e3, dyn_max * 1e3, ferr_max * 1e3);
}

int main(int argc, char **argv)
{
    if (argc == 3) {            /* sim_loop <Pgain> <FF2>: try your own tuning */
        run("crystal, host jitter 0..50 us", 20e-6, 25, atof(argv[1]), atof(argv[2]), 0);
        return 0;
    }
    printf("%-44s %10s %11s %12s %12s\n", "100 mm/s, 1000 mm/s^2, 1.25 um/step",
           "time shift", "ripple p-p", "dyn. error", "f-error");
    run("old feedback, P=200, FF2=0, clock 1% off",   0.01,  0,  200, 0,      1);
    run("new feedback, P=200, FF2=0, clock 1% off",   0.01,  0,  200, 0,      0);
    run("new feedback, P=200, FF2=0, crystal",        20e-6, 0,  200, 0,      0);
    run("new feedback, P=100, FF2=0.0025, crystal",   20e-6, 0,  100, 0.0025, 0);
    run("  + host jitter 0..50 us",                   20e-6, 25, 100, 0.0025, 0);
    run("  + host jitter 0..200 us",                  20e-6, 100, 100, 0.0025, 0);
    printf("\ntime shift : constant delay of the whole trajectory (same for all axes)\n"
           "ripple p-p : variation of the position during cruise, includes +-1 step quantisation\n"
           "dyn. error : largest deviation from the time-shifted command over the whole move\n"
           "f-error    : largest following error LinuxCNC sees\n");
    return 0;
}
