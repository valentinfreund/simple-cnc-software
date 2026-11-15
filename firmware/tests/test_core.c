/*
 * Host-side tests for the board-independent firmware library.
 * Build and run with `make -C firmware/tests`.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim.h"

#define STEP_LEN   2
#define STEP_SPACE 1
#define DIR_SETUP  3
#define HOST_MS    503u     /* host period in board samples: the clocks are not in sync */
#define U          CNCP_POS_PER_STEP

static int failures;
#define CHECK(cond) do { if (!(cond)) { failures++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

/* ---- pin model: watches the output samples like a drive would ------------ */
static uint16_t port;
static long edges[2], min_high[2], min_low[2], run_len[2], run_known[2];
static long pos[2];                 /* position seen on the pins, in steps */
static long min_dir_setup, since_dir_change[2];

static void pins_reset(void)
{
    for (int a = 0; a < 2; a++) { edges[a] = 0; min_high[a] = min_low[a] = 1 << 30; run_len[a] = 0; run_known[a] = 0; }
    min_dir_setup = 1 << 30;
}

static void sim_on_sample(uint32_t w)
{
    uint16_t old = port;
    port = (uint16_t)((port | (w & 0xFFFF)) & ~(w >> 16));
    CHECK(((w & 0xFFFF) & (w >> 16)) == 0);   /* never set and reset one pin */
    for (int a = 0; a < 2; a++) {
        int step = (port >> a) & 1, was = (old >> a) & 1;
        int dir = (port >> (4 + a)) & 1, dir_was = (old >> (4 + a)) & 1;
        since_dir_change[a]++;
        if (dir != dir_was) { since_dir_change[a] = 0; CHECK(!was && !step); }
        if (step != was) {
            if (run_known[a]) {               /* length of the run that just ended */
                long *m = was ? &min_high[a] : &min_low[a];
                if (run_len[a] < *m) *m = run_len[a];
            }
            run_known[a] = 1;                 /* runs are measured from edge to edge */
            run_len[a] = 0;
            if (step) {
                edges[a]++;
                pos[a] += dir ? 1 : -1;
                if (since_dir_change[a] < min_dir_setup) min_dir_setup = since_dir_change[a];
            }
        }
        run_len[a]++;
    }
}

static const stepgen_cfg_t sg_cfg = {
    .sample_rate_hz = FS, .num_axes = 2, .step_len = STEP_LEN,
    .step_space = STEP_SPACE, .dir_setup = DIR_SETUP,
    .axis = { { .step_pin = 0, .dir_pin = 4 }, { .step_pin = 1, .dir_pin = 5 } },
};

/* ---- part A: the step generator alone --------------------------------------- */

static stepgen_t sg;

static void sg_run(unsigned samples, int32_t v0_mhz, int32_t v1_mhz)
{
    uint32_t buf[BLOCK];
    stepgen_set_velocity(&sg, 0, v0_mhz);
    stepgen_set_velocity(&sg, 1, v1_mhz);
    for (unsigned done = 0; done < samples; done += BLOCK) {
        stepgen_render(&sg, buf, BLOCK);
        stepgen_end_block(&sg, BLOCK);
        for (unsigned s = 0; s < BLOCK; s++) sim_on_sample(buf[s]);
    }
}

static void test_stepgen(void)
{
    CHECK(stepgen_init(&sg, &sg_cfg) == 0);
    port = 0; pos[0] = pos[1] = 0;
    sim_on_sample(stepgen_idle_sample(&sg));
    pins_reset();

    /* exact rates: 10 kHz and -3.3333 kHz for one second */
    sg_run(FS, 10000000, -3333300);
    CHECK(labs(pos[0] - 10000) <= 1 && labs(pos[1] + 3333) <= 1);
    CHECK(stepgen_count(&sg, 0) == pos[0] && stepgen_count(&sg, 1) == pos[1]);
    CHECK(min_high[0] == STEP_LEN && min_high[1] == STEP_LEN);
    CHECK(min_dir_setup > DIR_SETUP);

    /* and back: lands exactly where it started */
    sg_run(FS, -10000000, 3333300);
    sg_run(BLOCK, 0, 0);
    CHECK(labs(pos[0]) <= 1 && labs(pos[1]) <= 1);

    /* rate limit: 10 MHz request is clamped to FS / (len + space) */
    pins_reset();
    long before = pos[0];
    sg_run(FS / 10, 2000000000, 0);
    long want = (long)(FS / (STEP_LEN + STEP_SPACE)) / 10;
    CHECK(labs(pos[0] - before - want) <= 2);
    CHECK(min_high[0] == STEP_LEN && min_low[0] >= STEP_SPACE);

    /* rapid reversals: pulse shape and direction timing hold, count == pins */
    pins_reset();
    for (int i = 0; i < 300; i++)
        sg_run(BLOCK * (1 + i % 4), (i & 1) ? -150000000 : 150000000, (i & 1) ? 777000 : -777000);
    sg_run(BLOCK, 0, 0);
    CHECK(stepgen_count(&sg, 0) == pos[0] && stepgen_count(&sg, 1) == pos[1]);
    CHECK(min_high[0] == STEP_LEN && min_low[0] >= STEP_SPACE && min_dir_setup > DIR_SETUP);

    /* the fractional position agrees with the pins to within one step */
    double frac = (double)stepgen_pos_rendered(&sg, 0) / 4294967296.0;
    CHECK(fabs(frac - (double)pos[0]) <= 1.0);
}

/* ---- part B: the whole board logic behind the protocol ----------------------- */

static uint8_t tx[CNCP_FRAME_SIZE];
static uint16_t seq;
static double cmd_pos[2];           /* host position command in steps */
static int jitter;                  /* host starts each cycle 0..jitter samples late */
static double max_err;              /* largest tracking error the board reported, steps */
static double lag_min, lag_max;     /* command minus pins at the nominal cycle times */

static cncp_status_t send(int enable, int corrupt)
{
    cncp_cmd_t c;
    cncp_status_t st;
    memset(&c, 0, sizeof(c));
    c.flags = enable ? CNCP_CMD_ENABLE : 0;
    c.seq = ++seq;
    for (int a = 0; a < 2; a++)
        c.pos_cmd[a] = (int32_t)(uint32_t)(long long)llround(cmd_pos[a] * U);
    c.outputs = 0x5;
    cncp_cmd_seal(&c);
    if (corrupt) c.pos_cmd[0] ^= 1;
    cnc_core_frame((const uint8_t *)&c, tx);
    memcpy(&st, tx, sizeof(st));
    CHECK(cncp_status_valid(&st));
    for (int a = 0; a < 2; a++)
        if (fabs((double)st.err[a] / U) > max_err) max_err = fabs((double)st.err[a] / U);
    return st;
}

/* run host cycles; v0/v1 are the commanded speeds in steps per cycle */
static cncp_status_t cycles(int n, int enable, double v0, double v1)
{
    cncp_status_t st = {0};
    for (int i = 0; i < n; i++) {
        double lag = cmd_pos[0] - (double)pos[0];
        if (lag < lag_min) lag_min = lag;
        if (lag > lag_max) lag_max = lag;
        int j = jitter ? rand() % (jitter + 1) : 0;
        sim_advance((unsigned)j);
        cmd_pos[0] += v0; cmd_pos[1] += v1;
        st = send(enable, 0);
        sim_advance(HOST_MS - (unsigned)j);
    }
    return st;
}

static void stats_reset(void) { max_err = 0; lag_min = 1e30; lag_max = -1e30; }

static void test_core(void)
{
    cnc_core_cfg_t cfg = {
        .stepgen = sg_cfg, .block_samples = BLOCK, .host_period_samples = 500,
        .margin_samples = 100, .watchdog_ms = 10, .estop_input = 5,
    };
    cncp_status_t st;

    CHECK(cnc_core_init(&cfg) == 0);
    port = 0; pos[0] = pos[1] = 0;
    sim_on_sample(cnc_core_idle_sample());
    pins_reset();
    srand(1);

    /* the host's position values start anywhere, even close to the 32-bit wrap */
    cmd_pos[0] = 8388000.0; cmd_pos[1] = -1234.5;

    /* 1. ENABLE without a preceding ENABLE=0 frame must not start anything */
    st = cycles(5, 1, 50, 0);
    CHECK(st.state == CNCP_STATE_DISABLED && edges[0] == 0 && !fake_enable);

    /* 2. enabling does not move, whatever the commanded position is */
    cycles(3, 0, 0, 0);
    st = cycles(20, 1, 0, 0);
    CHECK(st.state == CNCP_STATE_RUNNING && fake_enable && fake_outputs == 0x5);
    CHECK(pos[0] == 0 && pos[1] == 0);

    cycles(600, 1, 0, 0);            /* let the clock recovery settle on the host clock */

    /* 3. a move with ramps, 100 kHz on axis 0, crossing the 32-bit wrap of the
     *    protocol; host clock 0.6 % off. Ends exactly on the commanded position. */
    double start0 = cmd_pos[0], start1 = cmd_pos[1];
    stats_reset();
    for (int i = 1; i <= 100; i++) cycles(1, 1, i * 1.006, -i * 0.3333);
    stats_reset();
    cycles(300, 1, 100.6, -33.33);                       /* cruise */
    printf("  cruise, no jitter : lag varies by %.2f steps\n", lag_max - lag_min);
    CHECK(lag_max - lag_min <= 1.5);                     /* constant delay, +- quantisation */
    CHECK(max_err <= 1.0);
    for (int i = 100; i >= 0; i--) cycles(1, 1, i * 1.006, -i * 0.3333);
    st = cycles(10, 1, 0, 0);
    CHECK(fabs((double)pos[0] - (cmd_pos[0] - start0)) <= 1.0);
    CHECK(fabs((double)pos[1] - (cmd_pos[1] - start1)) <= 1.0);
    CHECK(st.late == 0 && max_err <= 1.0);
    CHECK(min_high[0] == STEP_LEN && min_low[0] >= STEP_SPACE && min_dir_setup > DIR_SETUP);

    /* 4. the same with host jitter of 0..300 us: still on time, still exact */
    jitter = 150;
    cycles(1000, 1, 0, 0);           /* jitter is there from the start on a real host */
    uint8_t late0 = cycles(1, 1, 0, 0).late;
    for (int i = 1; i <= 100; i++) cycles(1, 1, -i * 1.0, i * 0.5);
    stats_reset();
    cycles(300, 1, -100, 50);
    printf("  cruise, 300 us jitter: lag varies by %.2f steps\n", lag_max - lag_min);
    CHECK(lag_max - lag_min <= 4.0);                     /* jitter is filtered down to a few steps */
    CHECK(max_err <= 2.0);
    for (int i = 100; i >= 0; i--) cycles(1, 1, -i * 1.0, i * 0.5);
    st = cycles(10, 1, 0, 0);
    CHECK(fabs((double)pos[0] - (cmd_pos[0] - start0)) <= 1.0);
    CHECK(fabs((double)pos[1] - (cmd_pos[1] - start1)) <= 1.0);
    CHECK(st.late == late0);
    jitter = 0;
    cycles(1000, 1, 0, 0);

    /* 5. lost frames while moving: no fault, the move is completed exactly */
    cycles(50, 1, 40, 0);
    for (int i = 0; i < 3; i++) {
        cmd_pos[0] += 40;
        send(1, 1);                                      /* corrupted on the way */
        sim_advance(HOST_MS);
        cycles(20, 1, 40, 0);
    }
    st = cycles(10, 1, 0, 0);
    CHECK(st.state == CNCP_STATE_RUNNING);
    CHECK(fabs((double)pos[0] - (cmd_pos[0] - start0)) <= 1.0);

    /* 6. a command the machine cannot follow (too fast): it falls behind,
     *    reports it, and still arrives */
    stats_reset();
    cycles(20, 1, 400, 0);                               /* 400 kHz requested, 166 kHz possible */
    CHECK(max_err >= 100.0);
    st = cycles(100, 1, 0, 0);
    CHECK(fabs((double)pos[0] - (cmd_pos[0] - start0)) <= 1.0);
    CHECK(min_high[0] == STEP_LEN && min_low[0] >= STEP_SPACE);

    /* 7. watchdog: no valid frame for more than 10 ms while running */
    cycles(5, 1, 10, 0);
    for (int i = 0; i < 12; i++) { st = send(1, 1); sim_advance(HOST_MS); }
    CHECK(st.state == CNCP_STATE_FAULT && (st.fault & CNCP_FAULT_COMM_TIMEOUT));
    CHECK(!fake_enable && fake_outputs == 0);
    long before = pos[0];
    st = cycles(20, 1, 10, 0);                           /* still ENABLE=1: stays in fault */
    CHECK(st.state == CNCP_STATE_FAULT && labs(pos[0] - before) <= 1);
    st = cycles(2, 0, 0, 0);                             /* ENABLE=0 acknowledges */
    CHECK(st.state == CNCP_STATE_DISABLED && st.fault == 0);

    /* 8. re-enabling after the host position moved on: no jump */
    before = pos[0];
    cmd_pos[0] += 5000;
    st = cycles(20, 1, 0, 0);
    CHECK(st.state == CNCP_STATE_RUNNING && pos[0] == before);

    /* 9. e-stop: stops while running, also without frames; cannot start while active */
    cycles(5, 1, 10, 0);
    fake_inputs = 1u << 5;
    sim_advance(BLOCK);
    CHECK(cnc_core_state() == CNCP_STATE_FAULT && !fake_enable);
    st = cycles(2, 0, 0, 0);
    CHECK(st.state == CNCP_STATE_DISABLED && (st.live & CNCP_LIVE_ESTOP));
    st = cycles(5, 1, 10, 0);
    CHECK(st.state == CNCP_STATE_DISABLED);
    fake_inputs = 0;                             /* releasing e-stop must not start by itself */
    st = cycles(5, 1, 10, 0);
    CHECK(st.state == CNCP_STATE_DISABLED && !fake_enable);
    cycles(2, 0, 0, 0);
    st = cycles(5, 1, 10, 0);
    CHECK(st.state == CNCP_STATE_RUNNING);
}

int main(void)
{
    test_stepgen();
    test_core();
    printf(failures ? "%d check(s) FAILED\n" : "all checks passed\n", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
