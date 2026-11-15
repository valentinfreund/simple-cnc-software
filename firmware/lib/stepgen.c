/*
 * stepgen.c - board-independent step/direction pulse generator (see stepgen.h).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "stepgen.h"
#include <string.h>

#define PIN_SET(p)   (1u << (p))
#define PIN_RESET(p) (1u << ((p) + 16))

int stepgen_init(stepgen_t *sg, const stepgen_cfg_t *cfg)
{
    if (cfg->num_axes < 1 || cfg->num_axes > STEPGEN_MAX_AXES ||
        cfg->step_len < 1 || cfg->step_space < 1 || cfg->sample_rate_hz == 0)
        return -1;

    memset(sg, 0, sizeof(*sg));
    sg->cfg = *cfg;

    /* A step needs step_len + step_space samples, so the accumulator may
     * overflow at most once in that many samples. */
    sg->inc_max = (uint32_t)(0x100000000ull / (uint32_t)(cfg->step_len + cfg->step_space));
    if (sg->inc_max > 0x7FFFFFFFu)
        sg->inc_max = 0x7FFFFFFFu;

    for (unsigned i = 0; i < cfg->num_axes; i++) {
        const stepgen_axis_cfg_t *c = &cfg->axis[i];
        stepgen_axis_t *a = &sg->axis[i];
        if (c->step_pin > 15 || c->dir_pin > 15)
            return -1;
        a->step_on  = c->step_invert ? PIN_RESET(c->step_pin) : PIN_SET(c->step_pin);
        a->step_off = c->step_invert ? PIN_SET(c->step_pin)   : PIN_RESET(c->step_pin);
        a->dir_pos  = c->dir_invert  ? PIN_RESET(c->dir_pin)  : PIN_SET(c->dir_pin);
        a->dir_neg  = c->dir_invert  ? PIN_SET(c->dir_pin)    : PIN_RESET(c->dir_pin);
        a->dir = 1;
    }
    return 0;
}

uint32_t stepgen_idle_sample(const stepgen_t *sg)
{
    uint32_t w = 0;
    for (unsigned i = 0; i < sg->cfg.num_axes; i++)
        w |= sg->axis[i].step_off | (sg->axis[i].dir ? sg->axis[i].dir_pos : sg->axis[i].dir_neg);
    return w;
}

uint32_t stepgen_max_rate_hz(const stepgen_t *sg)
{
    return sg->cfg.sample_rate_hz / (uint32_t)(sg->cfg.step_len + sg->cfg.step_space);
}

void stepgen_set_velocity(stepgen_t *sg, unsigned axis, int32_t vel_mhz)
{
    if (axis >= sg->cfg.num_axes)
        return;

    uint64_t mag = vel_mhz < 0 ? (uint64_t)(-(int64_t)vel_mhz) : (uint64_t)vel_mhz;
    uint64_t inc = (mag << 32) / ((uint64_t)sg->cfg.sample_rate_hz * 1000u);
    if (inc > sg->inc_max)
        inc = sg->inc_max;
    sg->axis[axis].inc_cmd = vel_mhz < 0 ? -(int32_t)inc : (int32_t)inc;
}

void stepgen_stop_all(stepgen_t *sg)
{
    for (unsigned i = 0; i < sg->cfg.num_axes; i++)
        sg->axis[i].inc_cmd = 0;
}

void stepgen_render(stepgen_t *sg, uint32_t *buf, unsigned n)
{
    const uint8_t step_len = sg->cfg.step_len;
    const uint8_t dir_setup = sg->cfg.dir_setup;

    memset(buf, 0, n * sizeof(buf[0]));

    for (unsigned i = 0; i < sg->cfg.num_axes; i++) {
        stepgen_axis_t *a = &sg->axis[i];
        int32_t cmd = a->inc_cmd;
        uint32_t inc = cmd < 0 ? (uint32_t)(-cmd) : (uint32_t)cmd;
        uint8_t want_dir = inc ? (cmd > 0) : a->dir;   /* keep dir at standstill */
        int32_t count = a->count;
        unsigned moving = 0;       /* samples in which the position advanced */

        for (unsigned s = 0; s < n; s++) {
            uint32_t w = 0;
            uint8_t idle = (a->pulse == 0);

            if (!idle && --a->pulse == 0)
                w |= a->step_off;

            if (want_dir != a->dir) {
                /* Change direction only after the step pin has been released
                 * for a full sample, then wait dir_setup samples. */
                if (idle) {
                    a->dir = want_dir;
                    w |= want_dir ? a->dir_pos : a->dir_neg;
                    a->phase = ~a->phase;   /* keep the sub-step position */
                    a->holdoff = dir_setup;
                }
            } else if (a->holdoff) {
                a->holdoff--;
            } else {
                uint32_t p = a->phase + inc;
                if (p < a->phase) {
                    w |= a->step_on;
                    a->pulse = step_len;
                    count += a->dir ? 1 : -1;
                }
                a->phase = p;
                moving++;
            }
            buf[s] |= w;
        }
        a->inc = inc;
        a->count = count;
        a->d_prev = a->d_last;
        a->d_last = (int64_t)cmd * (int64_t)moving;
        a->pos += a->d_last;
    }
    sg->n_prev = sg->n_last;
    sg->n_last = n;
}

int32_t stepgen_position(const stepgen_t *sg, unsigned axis, unsigned played)
{
    const stepgen_axis_t *a = &sg->axis[axis];
    int64_t p = a->pos - a->d_last - a->d_prev;    /* start of the older block */

    if (played <= sg->n_prev) {
        if (sg->n_prev)
            p += a->d_prev * (int64_t)played / (int64_t)sg->n_prev;
    } else {
        unsigned q = played - sg->n_prev;
        if (q > sg->n_last)
            q = sg->n_last;
        p += a->d_prev;
        if (sg->n_last)
            p += a->d_last * (int64_t)q / (int64_t)sg->n_last;
    }
    /* 2^-32 steps -> 1/256 steps; wraps like the host expects */
    return (int32_t)(uint32_t)((uint64_t)p >> 24);
}
