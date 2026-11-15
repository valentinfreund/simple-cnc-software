/*
 * board.h - configuration of the build "WeAct MiniSTM32H723 + SPI host".
 *
 * Board : WeAct Studio MiniSTM32H7xx core board with STM32H723VGT6 (LQFP100)
 * Host  : Raspberry Pi (or any Linux SBC) as SPI master, see docs/.
 *
 * Everything that is specific to this board/transport combination lives in
 * this directory; the machine logic comes from firmware/lib.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef BOARD_H
#define BOARD_H

/* ---- step generator ------------------------------------------------------
 * Output sample rate and pulse shape. With 500 kHz one sample is 2 us:
 *   pulse 4 us, gap >= 4 us  ->  max. 125 kHz step rate. */
#define BOARD_SAMPLE_RATE_HZ   500000u
#define BOARD_BLOCK_SAMPLES    250u      /* rendered per interrupt = 0.5 ms   */
#define BOARD_STEP_LEN         2         /* samples                            */
#define BOARD_STEP_SPACE       2         /* samples                            */
#define BOARD_DIR_SETUP        5         /* samples (10 us)                    */
#define BOARD_WATCHDOG_MS      10

/* ---- step/dir pins: all on GPIOB, all 5 V tolerant (FT) -------------------
 * Open drain: connect PUL+/DIR+ of the drive to +5 V and PUL-/DIR- to the
 * pin. The pin pulls low to light the drive's optocoupler, so a step pulse
 * is "pin low" (step_invert = 1). */
#define BOARD_NUM_AXES         4
#define BOARD_STEP_OPEN_DRAIN  1
#define BOARD_STEP_INVERT      1
#define BOARD_AXES { \
    /* joint 0: X  */ { .step_pin = 10, .dir_pin = 14, .step_invert = BOARD_STEP_INVERT }, \
    /* joint 1: Y1 */ { .step_pin = 11, .dir_pin = 15, .step_invert = BOARD_STEP_INVERT }, \
    /* joint 2: Y2 */ { .step_pin = 12, .dir_pin = 8,  .step_invert = BOARD_STEP_INVERT }, \
    /* joint 3: Z  */ { .step_pin = 13, .dir_pin = 9,  .step_invert = BOARD_STEP_INVERT }, \
}
#define BOARD_STEP_PORT        GPIOB
#define BOARD_STEP_PINS        (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | \
                                GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)

/* ---- digital inputs: 3.3 V only, internal pull-up, active when pulled to GND
 * (use optocouplers for 24 V switches and drive alarm outputs). */
#define BOARD_INPUTS { \
    /* in 0 */ { GPIOC, GPIO_PIN_0 }, \
    /* in 1 */ { GPIOC, GPIO_PIN_1 }, \
    /* in 2 */ { GPIOA, GPIO_PIN_0 }, \
    /* in 3 */ { GPIOA, GPIO_PIN_1 }, \
    /* in 4 */ { GPIOA, GPIO_PIN_2 }, \
    /* in 5 */ { GPIOA, GPIO_PIN_3 }, \
    /* in 6 */ { GPIOC, GPIO_PIN_4 }, \
    /* in 7 */ { GPIOC, GPIO_PIN_5 }, \
}

/* E-stop: normally-closed contact between this input and GND. Open contact
 * (or broken wire) = e-stop. Bit BOARD_ESTOP_BIT of the input word is 1 while
 * the e-stop is active. Set to -1 to build without an e-stop input. */
#define BOARD_ESTOP_INPUT      7         /* in 7 = PC5 */

/* ---- digital outputs: 3.3 V push-pull, active high ------------------------ */
#define BOARD_ENABLE_PORT      GPIOD     /* servo-on for all drives */
#define BOARD_ENABLE_PIN       GPIO_PIN_8
#define BOARD_OUTPUTS { \
    /* out 0 */ { GPIOD, GPIO_PIN_9  }, \
    /* out 1 */ { GPIOD, GPIO_PIN_10 }, \
    /* out 2 */ { GPIOD, GPIO_PIN_14 }, \
    /* out 3 */ { GPIOD, GPIO_PIN_15 }, \
}

/* ---- status LED (on-board, PE3) ------------------------------------------- */
#define BOARD_LED_PORT         GPIOE
#define BOARD_LED_PIN          GPIO_PIN_3

/* ---- host link: SPI1 slave, mode 0, 8 bit ---------------------------------
 * PA4 = NSS (CE0 of the Pi), PA5 = SCK, PA6 = MISO, PA7 = MOSI */

#endif /* BOARD_H */
