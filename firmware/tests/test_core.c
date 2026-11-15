/*
 * Host-side tests for the board-independent firmware library.
 * Build and run with `make -C firmware/tests`.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cnc_core.h"

#define FS      500000u
#define BLOCK   250u
#define STEP_LEN   2
#define STEP_SPACE 1
#define DIR_SETUP  3

static int failures;
#define CHECK(cond) do { if (!(cond)) { failures++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

/* ---- fake board -------------------------------------------------------- */
static uint32_t fake_inputs, fake_outputs;
static int fake_enable;
uint32_t cnc_board_read_inputs(void) { return fake_inputs; }
void cnc_board_write_outputs(uint32_t o) { fake_outputs = o; }
void cnc_board_write_analog(unsigned ch, uint16_t v) { (void)ch; (void)v; }
void cnc_board_set_drive_enable(int on) { fake_enable = on; }

/* ---- pin model: plays the sample buffer like the GPIO port would -------- */
static uint16_t port;
static long edges[2], min_high[2], min_low[2], run_len[2];
static long pos[2];                 /* position seen by a drive on the pins */
static long min_dir_setup, since_dir_change[2];

static void pins_reset(void)
{
    for (int a = 0; a < 2; a++) { edges[a] = 0; min_high[a] = min_low[a] = 1 << 30; run_len[a] = 0; }
    min_dir_setup = 1 << 30;
}

static void play(const uint32_t *buf, unsigned n)
{
    for (unsigned s = 0; s < n; s++) {
        uint16_t old = port;
        port = (uint16_t)((port | (buf[s] & 0xFFFF)) & ~(buf[s] >> 16));
        CHECK(((buf[s] & 0xFFFF) & (buf[s] >> 16)) == 0);   /* never set and reset one pin */
        for (int a = 0; a < 2; a++) {
            int step = (port >> a) & 1, was = (old >> a) & 1;
            int dir = (port >> (4 + a)) & 1, dir_was = (old >> (4 + a)) & 1;
            since_dir_change[a]++;
            if (dir != dir_was) { since_dir_change[a] = 0; CHECK(!was && !step); }
            if (step != was) {
                if (edges[a] || !step) {          /* length of the run that just ended */
                    long *m = was ? &min_high[a] : &min_low[a];
                    if (run_len[a] < *m) *m = run_len[a];
                }
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
}

static uint8_t tx[CNCP_FRAME_SIZE];
static uint16_t seq;

static cncp_status_t send(int enable, int32_t v0, int32_t v1, int corrupt)
{
    cncp_cmd_t c;
    cncp_status_t st;
    memset(&c, 0, sizeof(c));
    c.flags = enable ? CNCP_CMD_ENABLE : 0;
    c.seq = ++seq;
    c.vel_mhz[0] = v0;
    c.vel_mhz[1] = v1;
    c.outputs = 0x5;
    cncp_cmd_seal(&c);
    if (corrupt) c.vel_mhz[0] ^= 1;
    cnc_core_frame((const uint8_t *)&c, tx);
    memcpy(&st, tx, sizeof(st));
    CHECK(cncp_status_valid(&st));
    return st;
}

/* run for ms milliseconds, sending a frame every ms like the host does */
static cncp_status_t run_ms(int ms, int enable, int32_t v0, int32_t v1)
{
    uint32_t buf[BLOCK];
    cncp_status_t st = {0};
    for (int i = 0; i < ms; i++) {
        st = send(enable, v0, v1, 0);
        for (unsigned b = 0; b < FS / 1000 / BLOCK; b++) {
            cnc_core_render(buf);
            play(buf, BLOCK);
        }
    }
    return st;
}

int main(void)
{
    cnc_core_cfg_t cfg = {
        .stepgen = { .sample_rate_hz = FS, .num_axes = 2, .step_len = STEP_LEN,
                     .step_space = STEP_SPACE, .dir_setup = DIR_SETUP,
                     .axis = { { .step_pin = 0, .dir_pin = 4 }, { .step_pin = 1, .dir_pin = 5 } } },
        .block_samples = BLOCK, .watchdog_ms = 10, .estop_input = 5,
    };
    cncp_status_t st;
    uint32_t buf[BLOCK];

    CHECK(cnc_core_init(&cfg) == 0);
    uint32_t idle = cnc_core_idle_sample();
    play(&idle, 1);
    pins_reset();

    /* 1. ENABLE without a preceding ENABLE=0 frame must not start anything */
    st = run_ms(5, 1, 1000000, 0);
    CHECK(st.state == CNCP_STATE_DISABLED && edges[0] == 0 && !fake_enable);

    /* 2. normal start: 10 kHz and -3.3333 kHz for one second */
    run_ms(2, 0, 0, 0);
    st = run_ms(1000, 1, 10000000, -3333300);
    CHECK(st.state == CNCP_STATE_RUNNING && fake_enable && fake_outputs == 0x5);
    st = run_ms(3, 1, 0, 0);
    CHECK(labs(st.count[0] - 10000) <= 12);          /* +- start/stop latency */
    CHECK(labs(st.count[1] + 3333) <= 5);
    CHECK(st.count[0] == pos[0] && st.count[1] == pos[1]);   /* count == pulses on the pins */
    CHECK(min_high[0] == STEP_LEN && min_high[1] == STEP_LEN);
    CHECK(min_dir_setup > DIR_SETUP);

    /* 3. go back the same time: must land where we started (within latency) */
    run_ms(1000, 1, -10000000, 3333300);
    st = run_ms(3, 1, 0, 0);
    CHECK(labs(st.count[0]) <= 12 && labs(st.count[1]) <= 5);
    CHECK(st.count[0] == pos[0] && st.count[1] == pos[1]);

    /* 4. rate limit: 10 MHz request is clamped to FS / (len + space) */
    pins_reset();
    long before = pos[0];
    run_ms(100, 1, 2000000000, 0);
    long got = pos[0] - before;
    long want = (long)(FS / (STEP_LEN + STEP_SPACE)) / 10;
    CHECK(labs(got - want) <= want / 100 + 2);
    CHECK(min_high[0] == STEP_LEN && min_low[0] >= STEP_SPACE);
    run_ms(3, 1, 0, 0);

    /* 5. rapid reversals keep count and pins in sync, dir timing is kept */
    pins_reset();
    for (int i = 0; i < 200; i++)
        run_ms(1 + i % 3, 1, (i & 1) ? -150000000 : 150000000, (i & 1) ? 777000 : -777000);
    st = run_ms(3, 1, 0, 0);
    CHECK(st.count[0] == pos[0] && st.count[1] == pos[1]);
    CHECK(min_high[0] == STEP_LEN && min_low[0] >= STEP_SPACE && min_dir_setup > DIR_SETUP);

    /* 6. corrupted frames are ignored; watchdog trips after 10 ms without a valid one */
    run_ms(5, 1, 5000000, 0);
    for (int i = 0; i < 12; i++) {
        st = send(1, 5000000, 0, 1);
        cnc_core_render(buf); play(buf, BLOCK);
        cnc_core_render(buf); play(buf, BLOCK);
    }
    CHECK(st.state == CNCP_STATE_FAULT && (st.fault & CNCP_FAULT_COMM_TIMEOUT));
    CHECK(!fake_enable && fake_outputs == 0);
    before = pos[0];
    st = run_ms(20, 1, 5000000, 0);              /* still ENABLE=1: stays in fault */
    CHECK(st.state == CNCP_STATE_FAULT && pos[0] - before <= 1);
    st = run_ms(2, 0, 0, 0);                     /* ENABLE=0 acknowledges */
    CHECK(st.state == CNCP_STATE_DISABLED && st.fault == 0);

    /* 7. e-stop: stops while running, also without frames; cannot start while active */
    run_ms(5, 1, 5000000, 0);
    CHECK(cnc_core_state() == CNCP_STATE_RUNNING);
    fake_inputs = 1u << 5;
    cnc_core_render(buf); play(buf, BLOCK);
    CHECK(cnc_core_state() == CNCP_STATE_FAULT && !fake_enable);
    st = run_ms(2, 0, 0, 0);
    CHECK(st.state == CNCP_STATE_DISABLED && (st.live & CNCP_LIVE_ESTOP));
    st = run_ms(5, 1, 5000000, 0);
    CHECK(st.state == CNCP_STATE_DISABLED);
    fake_inputs = 0;                             /* releasing e-stop must not start by itself */
    st = run_ms(5, 1, 5000000, 0);
    CHECK(st.state == CNCP_STATE_DISABLED && !fake_enable);
    run_ms(2, 0, 0, 0);
    st = run_ms(5, 1, 5000000, 0);
    CHECK(st.state == CNCP_STATE_RUNNING);

    printf(failures ? "%d check(s) FAILED\n" : "all checks passed\n", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
