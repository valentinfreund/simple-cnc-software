/*
 * cncproto.h - wire protocol between LinuxCNC (host) and the real-time board.
 *
 * Header-only on purpose: it is included by the firmware library and by the
 * LinuxCNC HAL component, so both sides always agree on the frame layout.
 *
 * One fixed-size frame travels in each direction per servo period (1 ms):
 *   host  -> board : cncp_cmd_t     (position commands, outputs)
 *   board -> host  : cncp_status_t  (positions, tracking error, inputs, state)
 * The transport (SPI, UDP, ...) only has to move CNCP_FRAME_SIZE bytes.
 * All fields are little-endian.
 *
 * Positions are absolute, in 1/CNCP_POS_PER_STEP steps, and wrap at 32 bit;
 * both sides only ever use differences. Each command is a waypoint: the
 * board moves in a straight line (constant speed) from the previous waypoint
 * to this one within one servo period of its own clock, which it keeps
 * locked to the average frame rate of the host.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CNCPROTO_H
#define CNCPROTO_H

#include <stdint.h>

#define CNCP_VERSION       2
#define CNCP_FRAME_SIZE    80
#define CNCP_MAX_JOINTS    8
#define CNCP_NUM_ANALOG    2
#define CNCP_POS_PER_STEP  256           /* resolution of the position feedback */

#define CNCP_MAGIC_CMD     0x32444D43u   /* "CMD2" */
#define CNCP_MAGIC_STATUS  0x32415453u   /* "STA2" */

/* cncp_cmd_t.flags */
#define CNCP_CMD_ENABLE    0x01u   /* host wants drives enabled and motion running */

/* cncp_status_t.state */
enum {
    CNCP_STATE_DISABLED = 0,   /* safe state: no steps, drives disabled            */
    CNCP_STATE_RUNNING  = 1,   /* executing velocity commands                      */
    CNCP_STATE_FAULT    = 2    /* latched; host must clear CNCP_CMD_ENABLE to reset */
};

/* cncp_status_t.fault (latched until the host drops CNCP_CMD_ENABLE) */
#define CNCP_FAULT_COMM_TIMEOUT  0x0001u  /* no valid frame within the watchdog time */
#define CNCP_FAULT_ESTOP         0x0002u  /* e-stop input opened while running       */

/* cncp_status_t.live (not latched) */
#define CNCP_LIVE_ESTOP          0x01u    /* e-stop input is active right now        */

typedef struct __attribute__((packed)) {
    uint32_t magic;                        /* CNCP_MAGIC_CMD                       */
    uint8_t  version;                      /* CNCP_VERSION                         */
    uint8_t  flags;                        /* CNCP_CMD_*                           */
    uint16_t seq;                          /* incremented by the host every frame  */
    int32_t  pos_cmd[CNCP_MAX_JOINTS];     /* position to reach one period from now */
    uint32_t outputs;                      /* digital outputs, bit n = output n    */
    uint16_t analog[CNCP_NUM_ANALOG];      /* 0..65535 (e.g. spindle speed)        */
    uint8_t  reserved[30];
    uint16_t crc;                          /* CRC16-CCITT over all previous bytes  */
} cncp_cmd_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;                        /* CNCP_MAGIC_STATUS                    */
    uint8_t  version;                      /* CNCP_VERSION                         */
    uint8_t  state;                        /* CNCP_STATE_*                         */
    uint16_t seq;                          /* seq of the last valid command frame  */
    int32_t  pos[CNCP_MAX_JOINTS];         /* position on the pins at the moment
                                              this frame was built                 */
    int16_t  err[CNCP_MAX_JOINTS];         /* planned minus actual position at that
                                              moment (saturating)                  */
    uint32_t inputs;                       /* digital inputs, bit n = input n      */
    uint16_t fault;                        /* CNCP_FAULT_*                         */
    uint8_t  live;                         /* CNCP_LIVE_*                          */
    uint8_t  late;                         /* counts commands that came too late
                                              to be executed on time (wraps)       */
    uint8_t  reserved[14];
    uint16_t crc;                          /* CRC16-CCITT over all previous bytes  */
} cncp_status_t;

_Static_assert(sizeof(cncp_cmd_t) == CNCP_FRAME_SIZE, "cncp_cmd_t size");
_Static_assert(sizeof(cncp_status_t) == CNCP_FRAME_SIZE, "cncp_status_t size");

/* CRC16-CCITT (poly 0x1021, init 0xFFFF), bitwise: 64 bytes per ms is cheap. */
static inline uint16_t cncp_crc16(const uint8_t *data, unsigned len)
{
    uint16_t crc = 0xFFFFu;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

static inline void cncp_cmd_seal(cncp_cmd_t *c)
{
    c->magic = CNCP_MAGIC_CMD;
    c->version = CNCP_VERSION;
    c->crc = cncp_crc16((const uint8_t *)c, CNCP_FRAME_SIZE - 2);
}

static inline int cncp_cmd_valid(const cncp_cmd_t *c)
{
    return c->magic == CNCP_MAGIC_CMD && c->version == CNCP_VERSION &&
           c->crc == cncp_crc16((const uint8_t *)c, CNCP_FRAME_SIZE - 2);
}

static inline void cncp_status_seal(cncp_status_t *s)
{
    s->magic = CNCP_MAGIC_STATUS;
    s->version = CNCP_VERSION;
    s->crc = cncp_crc16((const uint8_t *)s, CNCP_FRAME_SIZE - 2);
}

static inline int cncp_status_valid(const cncp_status_t *s)
{
    return s->magic == CNCP_MAGIC_STATUS && s->version == CNCP_VERSION &&
           s->crc == cncp_crc16((const uint8_t *)s, CNCP_FRAME_SIZE - 2);
}

#endif /* CNCPROTO_H */
