/*
 * cncboard_spi.c - LinuxCNC HAL driver for the real-time board, SPI transport.
 *
 * Runs on the LinuxCNC host (Raspberry Pi or any board with spidev) and
 * exchanges one frame per servo period with the board firmware:
 *   out: position command per joint, enable, digital/analog outputs
 *   in : position and tracking error per joint, digital inputs, board state
 *
 * The board follows the position commands by itself (it moves from each
 * commanded position to the next within one servo period of its own clock,
 * which it locks to the host). No pid is needed in HAL: connect
 * joint.N.motor-pos-cmd and joint.N.motor-pos-fb directly.
 *
 * The board executes every command a fixed time (about 2 ms) after it was
 * sent, the same for all joints. pos-fb hides this constant delay: it is the
 * command minus the tracking error the board measured, so the
 * following error in LinuxCNC is the real deviation from the planned path.
 * pos-board is the position on the step/dir pins at the last frame.
 *
 * Load:   loadrt cncboard_spi [device=/dev/spidev0.0] [speed_hz=2000000] [joints=4]
 * Thread: addf cncboard.update servo-thread      (after motion-controller)
 *
 * Pins and parameters:
 *   cncboard.joint.N.pos-cmd   float in   position command in machine units
 *   cncboard.joint.N.pos-fb    float out  position feedback in machine units
 *   cncboard.joint.N.pos-board float out  position on the pins (delayed), machine units
 *   cncboard.joint.N.counts    s32   out  position on the pins in steps
 *   cncboard.joint.N.scale     float rw   steps per machine unit (parameter)
 *   cncboard.enable            bit   in   run request; a rising edge starts the board
 *   cncboard.out.NN            bit   in   digital outputs
 *   cncboard.analog.N          float in   analog outputs, 0.0 .. 1.0
 *   cncboard.in.NN / -not      bit   out  digital inputs
 *   cncboard.ok                bit   out  link is up and the e-stop input is closed
 *   cncboard.ready             bit   out  board is running
 *   cncboard.fault             bit   out  board is in fault state
 *   cncboard.fault-code        u32   out  CNCP_FAULT_* bits
 *   cncboard.estop-active      bit   out  e-stop input of the board is open
 *   cncboard.comm-error        bit   out  too many consecutive bad frames
 *   cncboard.comm-errors       u32   out  total number of bad frames
 *   cncboard.late-commands     u32   out  commands that reached the board too late
 *   cncboard.comm-error-limit  u32   rw   consecutive bad frames tolerated (parameter)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "cncproto.h"

MODULE_AUTHOR("simple-cnc-software");
MODULE_DESCRIPTION("HAL driver for the cncboard real-time board (SPI)");
MODULE_LICENSE("GPL");

static char *device = "/dev/spidev0.0";
RTAPI_MP_STRING(device, "spidev device of the board");
static int speed_hz = 2000000;
RTAPI_MP_INT(speed_hz, "SPI clock in Hz");
static int joints = 4;
RTAPI_MP_INT(joints, "number of joints (1..8)");

#define NUM_IN   16
#define NUM_OUT  8

typedef struct {
    hal_float_t *pos_cmd;
    hal_float_t *pos_fb;
    hal_float_t *pos_board;
    hal_s32_t   *counts;
    hal_float_t  scale;
    int32_t      last_raw;
    long long    total;
} joint_t;

typedef struct {
    joint_t      joint[CNCP_MAX_JOINTS];
    hal_bit_t   *enable;
    hal_bit_t   *out[NUM_OUT];
    hal_float_t *analog[CNCP_NUM_ANALOG];
    hal_bit_t   *in[NUM_IN];
    hal_bit_t   *in_not[NUM_IN];
    hal_bit_t   *ok;
    hal_bit_t   *ready;
    hal_bit_t   *fault;
    hal_u32_t   *fault_code;
    hal_bit_t   *estop_active;
    hal_bit_t   *comm_error;
    hal_u32_t   *comm_errors;
    hal_u32_t   *late_commands;
    unsigned char last_late;
    hal_u32_t    comm_error_limit;
    unsigned     bad_in_a_row;
    int          have_counts;
    uint16_t     seq;
} board_t;

static int comp_id;
static int fd = -1;
static board_t *b;

static void update(void *arg, long period)
{
    board_t *d = arg;
    cncp_cmd_t cmd;
    cncp_status_t st;
    struct spi_ioc_transfer xfer;
    int i, good;
    (void)period;

    memset(&cmd, 0, sizeof(cmd));
    cmd.flags = *d->enable ? CNCP_CMD_ENABLE : 0;
    cmd.seq = ++d->seq;
    for (i = 0; i < joints; i++) {
        /* absolute position in 1/256 steps; only the low 32 bits travel */
        double p = *d->joint[i].pos_cmd * d->joint[i].scale * CNCP_POS_PER_STEP;
        cmd.pos_cmd[i] = (int32_t)(uint32_t)(unsigned long long)(long long)(p < 0 ? p - 0.5 : p + 0.5);
    }
    for (i = 0; i < NUM_OUT; i++)
        if (*d->out[i]) cmd.outputs |= 1u << i;
    for (i = 0; i < CNCP_NUM_ANALOG; i++) {
        double v = *d->analog[i];
        if (v < 0.0) v = 0.0;
        if (v > 1.0) v = 1.0;
        cmd.analog[i] = (uint16_t)(v * 65535.0 + 0.5);
    }
    cncp_cmd_seal(&cmd);

    memset(&xfer, 0, sizeof(xfer));
    memset(&st, 0, sizeof(st));
    xfer.tx_buf = (unsigned long)&cmd;
    xfer.rx_buf = (unsigned long)&st;
    xfer.len = CNCP_FRAME_SIZE;
    xfer.speed_hz = speed_hz;
    xfer.bits_per_word = 8;

    good = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer) >= 0 && cncp_status_valid(&st);

    if (good) {
        d->bad_in_a_row = 0;
        for (i = 0; i < joints; i++) {
            joint_t *j = &d->joint[i];
            double scale = j->scale != 0.0 ? j->scale : 1.0;
            if (!d->have_counts)
                j->last_raw = st.pos[i];        /* start at 0 wherever the board is */
            j->total += (int32_t)((uint32_t)st.pos[i] - (uint32_t)j->last_raw);
            j->last_raw = st.pos[i];
            *j->counts = (hal_s32_t)(j->total / CNCP_POS_PER_STEP);
            *j->pos_board = (double)j->total / (CNCP_POS_PER_STEP * scale);
            /* LinuxCNC compares this with the command it has just sent */
            *j->pos_fb = *j->pos_cmd - (double)st.err[i] / (CNCP_POS_PER_STEP * scale);
        }
        if (d->have_counts)
            *d->late_commands += (unsigned char)(st.late - d->last_late);
        d->last_late = st.late;
        d->have_counts = 1;
        for (i = 0; i < NUM_IN; i++) {
            int v = (st.inputs >> i) & 1u;
            *d->in[i] = v;
            *d->in_not[i] = !v;
        }
        *d->ready = st.state == CNCP_STATE_RUNNING;
        *d->fault = st.state == CNCP_STATE_FAULT;
        *d->fault_code = st.fault;
        *d->estop_active = (st.live & CNCP_LIVE_ESTOP) != 0;
    } else {
        (*d->comm_errors)++;
        if (d->bad_in_a_row < 1000000u)
            d->bad_in_a_row++;
    }

    if (!good)                      /* no news: assume the board is on plan */
        for (i = 0; i < joints; i++)
            *d->joint[i].pos_fb = *d->joint[i].pos_cmd;

    *d->comm_error = d->bad_in_a_row > d->comm_error_limit;
    if (*d->comm_error)
        *d->ready = 0;
    *d->ok = !*d->comm_error && !*d->estop_active;
}

static int export_pins(board_t *d)
{
    int i, r = 0;

    for (i = 0; i < joints && !r; i++) {
        r |= hal_pin_float_newf(HAL_IN, &d->joint[i].pos_cmd, comp_id, "cncboard.joint.%d.pos-cmd", i);
        r |= hal_pin_float_newf(HAL_OUT, &d->joint[i].pos_fb, comp_id, "cncboard.joint.%d.pos-fb", i);
        r |= hal_pin_float_newf(HAL_OUT, &d->joint[i].pos_board, comp_id, "cncboard.joint.%d.pos-board", i);
        r |= hal_pin_s32_newf(HAL_OUT, &d->joint[i].counts, comp_id, "cncboard.joint.%d.counts", i);
        r |= hal_param_float_newf(HAL_RW, &d->joint[i].scale, comp_id, "cncboard.joint.%d.scale", i);
        d->joint[i].scale = 1.0;
    }
    r |= hal_pin_bit_newf(HAL_IN, &d->enable, comp_id, "cncboard.enable");
    for (i = 0; i < NUM_OUT && !r; i++)
        r |= hal_pin_bit_newf(HAL_IN, &d->out[i], comp_id, "cncboard.out.%02d", i);
    for (i = 0; i < CNCP_NUM_ANALOG && !r; i++)
        r |= hal_pin_float_newf(HAL_IN, &d->analog[i], comp_id, "cncboard.analog.%d", i);
    for (i = 0; i < NUM_IN && !r; i++) {
        r |= hal_pin_bit_newf(HAL_OUT, &d->in[i], comp_id, "cncboard.in.%02d", i);
        r |= hal_pin_bit_newf(HAL_OUT, &d->in_not[i], comp_id, "cncboard.in.%02d-not", i);
    }
    r |= hal_pin_bit_newf(HAL_OUT, &d->ok, comp_id, "cncboard.ok");
    r |= hal_pin_bit_newf(HAL_OUT, &d->ready, comp_id, "cncboard.ready");
    r |= hal_pin_bit_newf(HAL_OUT, &d->fault, comp_id, "cncboard.fault");
    r |= hal_pin_u32_newf(HAL_OUT, &d->fault_code, comp_id, "cncboard.fault-code");
    r |= hal_pin_bit_newf(HAL_OUT, &d->estop_active, comp_id, "cncboard.estop-active");
    r |= hal_pin_bit_newf(HAL_OUT, &d->comm_error, comp_id, "cncboard.comm-error");
    r |= hal_pin_u32_newf(HAL_OUT, &d->comm_errors, comp_id, "cncboard.comm-errors");
    r |= hal_pin_u32_newf(HAL_OUT, &d->late_commands, comp_id, "cncboard.late-commands");
    r |= hal_param_u32_newf(HAL_RW, &d->comm_error_limit, comp_id, "cncboard.comm-error-limit");
    d->comm_error_limit = 3;
    return r;
}

int rtapi_app_main(void)
{
    unsigned char mode = SPI_MODE_0, bits = 8;
    unsigned int speed = speed_hz;

    if (joints < 1 || joints > CNCP_MAX_JOINTS) {
        rtapi_print_msg(RTAPI_MSG_ERR, "cncboard_spi: joints must be 1..%d\n", CNCP_MAX_JOINTS);
        return -EINVAL;
    }

    comp_id = hal_init("cncboard_spi");
    if (comp_id < 0)
        return comp_id;

    fd = open(device, O_RDWR);
    if (fd < 0 || ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "cncboard_spi: cannot set up %s: %s\n", device, strerror(errno));
        goto fail;
    }

    b = hal_malloc(sizeof(*b));
    if (!b)
        goto fail;
    memset(b, 0, sizeof(*b));

    if (export_pins(b) != 0 ||
        hal_export_funct("cncboard.update", update, b, 1, 0, comp_id) != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "cncboard_spi: exporting pins failed\n");
        goto fail;
    }

    /* Until the first good frame arrives, report "not ok". */
    *b->estop_active = 1;
    b->bad_in_a_row = 1000000u;

    hal_ready(comp_id);
    return 0;

fail:
    if (fd >= 0)
        close(fd);
    hal_exit(comp_id);
    return -1;
}

void rtapi_app_exit(void)
{
    if (fd >= 0)
        close(fd);
    hal_exit(comp_id);
}
