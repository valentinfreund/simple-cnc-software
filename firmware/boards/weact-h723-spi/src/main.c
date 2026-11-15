/*
 * main.c - board port "WeAct MiniSTM32H723 + SPI host".
 *
 * Hardware use:
 *   TIM2 update -> DMA1 stream 0 -> GPIOB->BSRR   step/dir samples, circular
 *   SPI1 slave  -> DMA1 stream 1 (rx) / 2 (tx)    one frame per servo period
 * All three interrupts run at the same priority, as cnc_core requires.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <string.h>
#include "stm32h7xx_hal.h"
#include "board.h"
#include "cnc_core.h"

#define IRQ_PRIO 1

typedef struct { GPIO_TypeDef *port; uint16_t pin; } pin_t;
static const pin_t input_pins[]  = BOARD_INPUTS;
static const pin_t output_pins[] = BOARD_OUTPUTS;
#define NUM_INPUTS  (sizeof(input_pins) / sizeof(input_pins[0]))
#define NUM_OUTPUTS (sizeof(output_pins) / sizeof(output_pins[0]))

/* DMA cannot reach DTCM RAM; the linker script puts .bss into AXI SRAM. The
 * data cache stays disabled, so no cache maintenance is needed. */
static uint32_t step_buf[2 * BOARD_BLOCK_SAMPLES] __attribute__((aligned(4)));
static uint8_t spi_rx[CNCP_FRAME_SIZE] __attribute__((aligned(4)));
static uint8_t spi_tx[CNCP_FRAME_SIZE] __attribute__((aligned(4)));

static SPI_HandleTypeDef hspi1;
static DMA_HandleTypeDef hdma_step, hdma_spi_rx, hdma_spi_tx;

void Error_Handler(void)
{
    __disable_irq();
    TIM2->CR1 = 0;                                   /* stop step output    */
    BOARD_ENABLE_PORT->BSRR = (uint32_t)BOARD_ENABLE_PIN << 16;   /* drives off */
    for (;;) {
        BOARD_LED_PORT->ODR ^= BOARD_LED_PIN;        /* fast blink          */
        for (volatile uint32_t i = 0; i < 2000000u; i++) { }
    }
}

/* ---- functions required by cnc_core ------------------------------------- */

uint32_t cnc_board_read_inputs(void)
{
    uint32_t v = 0;
    for (unsigned i = 0; i < NUM_INPUTS; i++) {
        int high = (input_pins[i].port->IDR & input_pins[i].pin) != 0;
        if ((int)i == BOARD_ESTOP_INPUT) {
            if (high) v |= 1u << i;      /* NC contact to GND: open (high) = e-stop */
        } else if (!high) {
            v |= 1u << i;                /* active when pulled to GND               */
        }
    }
    return v;
}

void cnc_board_write_outputs(uint32_t outputs)
{
    for (unsigned i = 0; i < NUM_OUTPUTS; i++)
        output_pins[i].port->BSRR = (outputs & (1u << i)) ? output_pins[i].pin
                                                          : (uint32_t)output_pins[i].pin << 16;
}

void cnc_board_write_analog(unsigned ch, uint16_t value)
{
    (void)ch; (void)value;               /* not available in this build yet */
}

void cnc_board_set_drive_enable(int on)
{
    BOARD_ENABLE_PORT->BSRR = on ? BOARD_ENABLE_PIN : (uint32_t)BOARD_ENABLE_PIN << 16;
}

static volatile uint8_t last_rendered_half = 1;   /* 0 = first, 1 = second half of step_buf */

unsigned cnc_board_samples_played(void)
{
    /* Position of the DMA read pointer in the circular buffer. */
    unsigned idx = 2u * BOARD_BLOCK_SAMPLES - __HAL_DMA_GET_COUNTER(&hdma_step);
    if (idx >= 2u * BOARD_BLOCK_SAMPLES)
        idx = 0;
    unsigned half = idx / BOARD_BLOCK_SAMPLES, off = idx % BOARD_BLOCK_SAMPLES;
    /* Normally the half that was rendered before the last one is playing. If
     * the read pointer is already in the last rendered half, the next render
     * interrupt is pending and we are one block further. */
    return half != last_rendered_half ? off : BOARD_BLOCK_SAMPLES + off;
}

/* ---- clock: 550 MHz CPU, 275 MHz bus/timers ----------------------------------
 * Runs from the 25 MHz crystal of the board, so the step rates are exact. If
 * the crystal does not start, the internal oscillator is used instead: the
 * board still works, but all rates are off by up to about 1 %, which shows
 * up as a small speed-dependent following error in LinuxCNC. */

static int pll_start(int use_crystal)
{
    RCC_OscInitTypeDef osc = {0};

    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLP = 1;
    osc.PLL.PLLQ = 3;
    osc.PLL.PLLR = 2;
    osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    if (use_crystal) {                       /* 25 MHz / 5 * 110 = 550 MHz */
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        osc.HSEState = RCC_HSE_ON;
        osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        osc.PLL.PLLM = 5;
        osc.PLL.PLLN = 110;
        osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
        osc.PLL.PLLFRACN = 0;
    } else {                                 /* 64 MHz / 4 * 34.375 = 550 MHz */
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState = RCC_HSI_DIV1;
        osc.HSICalibrationValue = 64;
        osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        osc.PLL.PLLM = 4;
        osc.PLL.PLLN = 34;
        osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
        osc.PLL.PLLFRACN = 3072;
    }
    return HAL_RCC_OscConfig(&osc) == HAL_OK;
}

static void clock_init(void)
{
    RCC_ClkInitTypeDef clk = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) { }

    if (!pll_start(1) && !pll_start(0))
        Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                    RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider = RCC_HCLK_DIV2;
    clk.APB3CLKDivider = RCC_APB3_DIV2;
    clk.APB1CLKDivider = RCC_APB1_DIV2;
    clk.APB2CLKDivider = RCC_APB2_DIV2;
    clk.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK)
        Error_Handler();
}

/* ---- GPIO ------------------------------------------------------------------ */

static void gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* Put every output into its safe level before it becomes an output. */
    BOARD_STEP_PORT->BSRR = cnc_core_idle_sample();
    cnc_board_set_drive_enable(0);
    cnc_board_write_outputs(0);

    g.Pin = BOARD_STEP_PINS;
    g.Mode = BOARD_STEP_OPEN_DRAIN ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(BOARD_STEP_PORT, &g);

    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = BOARD_ENABLE_PIN;
    HAL_GPIO_Init(BOARD_ENABLE_PORT, &g);
    g.Pin = BOARD_LED_PIN;
    HAL_GPIO_Init(BOARD_LED_PORT, &g);
    for (unsigned i = 0; i < NUM_OUTPUTS; i++) {
        g.Pin = output_pins[i].pin;
        HAL_GPIO_Init(output_pins[i].port, &g);
    }

    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    for (unsigned i = 0; i < NUM_INPUTS; i++) {
        g.Pin = input_pins[i].pin;
        HAL_GPIO_Init(input_pins[i].port, &g);
    }

    /* SPI1: PA4 NSS, PA5 SCK, PA6 MISO, PA7 MOSI */
    g.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &g);
    /* Keep NSS high while the host is not connected / not driving it. */
    g.Pin = GPIO_PIN_4;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);
}

/* ---- host link (SPI slave) -------------------------------------------------- */

static void spi_arm(void)
{
    /* If this fails, spi_poll() tries again; meanwhile the watchdog in
     * cnc_core keeps the machine safe. */
    (void)HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx, spi_rx, CNCP_FRAME_SIZE);
}

static void spi_frame_done(int ok)
{
    cnc_core_frame(ok ? spi_rx : NULL, spi_tx);
    spi_arm();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
    spi_frame_done(1);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
    spi_frame_done(0);
}

/* Called after every rendered block. Between two frames NSS is high; if a transfer is then
 * still pending, the frame either ended without the SPI end-of-transfer event
 * or the host sent a wrong number of bytes. Close it and listen again, so a
 * glitch can never leave the link out of step for good. */
static void spi_poll(void)
{
    if (!(GPIOA->IDR & GPIO_PIN_4))
        return;                                      /* frame in progress */
    if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY) {
        spi_arm();
        return;
    }
    uint32_t left = __HAL_DMA_GET_COUNTER(&hdma_spi_rx);
    if (left == CNCP_FRAME_SIZE)
        return;                                      /* idle, waiting      */
    HAL_SPI_Abort(&hspi1);
    spi_frame_done(left == 0);
}

static void spi_init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    hdma_spi_rx.Instance = DMA1_Stream1;
    hdma_spi_rx.Init.Request = DMA_REQUEST_SPI1_RX;
    hdma_spi_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi_rx.Init.Mode = DMA_NORMAL;
    hdma_spi_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi_rx) != HAL_OK)
        Error_Handler();
    __HAL_LINKDMA(&hspi1, hdmarx, hdma_spi_rx);

    hdma_spi_tx = hdma_spi_rx;
    hdma_spi_tx.Instance = DMA1_Stream2;
    hdma_spi_tx.Init.Request = DMA_REQUEST_SPI1_TX;
    hdma_spi_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    if (HAL_DMA_Init(&hdma_spi_tx) != HAL_OK)
        Error_Handler();
    __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi_tx);

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_SLAVE;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        Error_Handler();

    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    HAL_NVIC_SetPriority(SPI1_IRQn, IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);

    cnc_core_frame(NULL, spi_tx);        /* first status frame */
    spi_arm();
}

/* ---- step output: timer-clocked DMA writes samples to the GPIO port -------- */

static void step_half_done(DMA_HandleTypeDef *h)
{
    (void)h;
    cnc_core_render(&step_buf[0]);                   /* first half is free now  */
    last_rendered_half = 0;
    spi_poll();
}

static void step_full_done(DMA_HandleTypeDef *h)
{
    (void)h;
    cnc_core_render(&step_buf[BOARD_BLOCK_SAMPLES]); /* second half is free now */
    last_rendered_half = 1;
    spi_poll();
}

static void step_error(DMA_HandleTypeDef *h)
{
    (void)h;
    Error_Handler();
}

static void step_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    hdma_step.Instance = DMA1_Stream0;
    hdma_step.Init.Request = DMA_REQUEST_TIM2_UP;
    hdma_step.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_step.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_step.Init.MemInc = DMA_MINC_ENABLE;
    hdma_step.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_step.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_step.Init.Mode = DMA_CIRCULAR;
    hdma_step.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_step.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_step) != HAL_OK)
        Error_Handler();
    hdma_step.XferHalfCpltCallback = step_half_done;
    hdma_step.XferCpltCallback = step_full_done;
    hdma_step.XferErrorCallback = step_error;

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    memset(step_buf, 0, sizeof(step_buf));           /* 0 = leave pins alone */
    if (HAL_DMA_Start_IT(&hdma_step, (uint32_t)step_buf, (uint32_t)&BOARD_STEP_PORT->BSRR,
                         2 * BOARD_BLOCK_SAMPLES) != HAL_OK)
        Error_Handler();

    /* TIM2 runs from the APB1 timer clock (2 x PCLK1 because APB1 is divided). */
    uint32_t tim_clk = 2u * HAL_RCC_GetPCLK1Freq();
    TIM2->PSC = 0;
    TIM2->ARR = tim_clk / BOARD_SAMPLE_RATE_HZ - 1u;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->DIER = TIM_DIER_UDE;                       /* update event -> DMA request */
    TIM2->CR1 = TIM_CR1_CEN;
}

/* ---- interrupt handlers ------------------------------------------------------ */

void DMA1_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_step); }
void DMA1_Stream1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spi_rx); }
void DMA1_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spi_tx); }
void SPI1_IRQHandler(void)         { HAL_SPI_IRQHandler(&hspi1); }
void SysTick_Handler(void)         { HAL_IncTick(); }

void NMI_Handler(void)        { Error_Handler(); }
void HardFault_Handler(void)  { Error_Handler(); }
void MemManage_Handler(void)  { Error_Handler(); }
void BusFault_Handler(void)   { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }
void SVC_Handler(void)        { }
void DebugMon_Handler(void)   { }
void PendSV_Handler(void)     { }

/* ---- main ---------------------------------------------------------------------- */

int main(void)
{
    static const cnc_core_cfg_t cfg = {
        .stepgen = {
            .sample_rate_hz = BOARD_SAMPLE_RATE_HZ,
            .num_axes = BOARD_NUM_AXES,
            .step_len = BOARD_STEP_LEN,
            .step_space = BOARD_STEP_SPACE,
            .dir_setup = BOARD_DIR_SETUP,
            .axis = BOARD_AXES,
        },
        .block_samples = BOARD_BLOCK_SAMPLES,
        .host_period_samples = BOARD_SAMPLE_RATE_HZ / 1000000u * BOARD_HOST_PERIOD_US +
                               BOARD_SAMPLE_RATE_HZ % 1000000u * BOARD_HOST_PERIOD_US / 1000000u,
        .margin_samples = BOARD_SAMPLE_RATE_HZ / 100000u * BOARD_MARGIN_US / 10u,
        .watchdog_ms = BOARD_WATCHDOG_MS,
        .estop_input = BOARD_ESTOP_INPUT,
    };

    HAL_Init();
    clock_init();
    SCB_EnableICache();

    /* cnc_core_init() drives the board outputs to their safe level, so the
     * port clocks must already run; the pins become outputs in gpio_init(). */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    if (cnc_core_init(&cfg) != 0)
        Error_Handler();
    gpio_init();
    step_init();
    spi_init();

    /* LED: off = disabled, on = running, slow blink = fault */
    for (;;) {
        uint8_t state = cnc_core_state();
        int on = state == CNCP_STATE_RUNNING ||
                 (state == CNCP_STATE_FAULT && (HAL_GetTick() / 250u) % 2u);
        BOARD_LED_PORT->BSRR = on ? BOARD_LED_PIN : (uint32_t)BOARD_LED_PIN << 16;
    }
}
