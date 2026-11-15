/*
 * stepgen.h - board-independent step/direction pulse generator.
 *
 * The generator renders pulses into a sample buffer instead of toggling pins
 * itself. One buffer entry is one output sample; the board plays the buffer
 * at a fixed sample rate with hardware timing (timer-triggered DMA on STM32,
 * a shift register on an FPGA, ...). That makes pulse timing jitter-free and
 * the step count exact, with no interrupt per step.
 *
 * Sample format: bits 0..15 = port pins to set, bits 16..31 = pins to reset
 * (this is the STM32 GPIO BSRR layout; other targets can translate it).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef STEPGEN_H
#define STEPGEN_H

#include <stdint.h>

#define STEPGEN_MAX_AXES 8

typedef struct {
    uint8_t step_pin;      /* bit number 0..15 within the output port */
    uint8_t dir_pin;       /* bit number 0..15 within the output port */
    uint8_t step_invert;   /* 1: step pin idles high, pulse is low    */
    uint8_t dir_invert;    /* 1: positive direction drives pin low    */
} stepgen_axis_cfg_t;

typedef struct {
    uint32_t sample_rate_hz;    /* output sample rate                          */
    uint8_t  num_axes;          /* 1..STEPGEN_MAX_AXES                         */
    uint8_t  step_len;          /* pulse length in samples (>= 1)              */
    uint8_t  step_space;        /* minimum gap between pulses in samples (>=1) */
    uint8_t  dir_setup;         /* samples between dir change and next step    */
    stepgen_axis_cfg_t axis[STEPGEN_MAX_AXES];
} stepgen_cfg_t;

typedef struct {
    uint32_t phase;             /* DDS phase accumulator (fraction of a step)  */
    uint32_t inc;               /* phase increment per sample, in use          */
    volatile int32_t inc_cmd;   /* commanded signed increment, taken at block start */
    uint8_t  dir;               /* direction currently on the pin              */
    uint8_t  pulse;             /* samples left until the step pin is released */
    uint8_t  holdoff;           /* samples left of dir setup time              */
    volatile int32_t count;     /* signed number of generated steps            */
    int64_t  pos;               /* rendered position in 2^-32 steps            */
    int64_t  d_last, d_prev;    /* position change of the last two blocks      */
    uint32_t step_on, step_off, dir_pos, dir_neg;   /* precomputed samples     */
} stepgen_axis_t;

typedef struct {
    stepgen_cfg_t  cfg;
    uint32_t       inc_max;     /* limit so that step_len + step_space is kept */
    unsigned       n_last, n_prev;   /* sizes of the last two rendered blocks  */
    stepgen_axis_t axis[STEPGEN_MAX_AXES];
} stepgen_t;

/* Returns 0 on success, -1 on an invalid configuration. */
int stepgen_init(stepgen_t *sg, const stepgen_cfg_t *cfg);

/* Sample that puts all step/dir pins into their idle level (write it once at start-up). */
uint32_t stepgen_idle_sample(const stepgen_t *sg);

/* Highest step rate the configuration can produce, in steps/s. */
uint32_t stepgen_max_rate_hz(const stepgen_t *sg);

/* Set the velocity of one axis in 1/1000 steps/s (clamped to the maximum rate).
 * Takes effect at the start of the next rendered block. */
void stepgen_set_velocity(stepgen_t *sg, unsigned axis, int32_t vel_mhz);

/* Stop all axes (takes effect at the start of the next rendered block). */
void stepgen_stop_all(stepgen_t *sg);

/* Render the next n samples into buf. */
void stepgen_render(stepgen_t *sg, uint32_t *buf, unsigned n);

/* Position of one axis in 1/256 steps at the sample that is being output
 * right now. The last two rendered blocks are still (partly) waiting in the
 * output buffer; `played` says how many of their samples are already out
 * (0 .. size of both blocks), counted from the start of the older block. */
#define STEPGEN_POS_PER_STEP 256
int32_t stepgen_position(const stepgen_t *sg, unsigned axis, unsigned played);

static inline int32_t stepgen_count(const stepgen_t *sg, unsigned axis)
{
    return sg->axis[axis].count;
}

#endif /* STEPGEN_H */
