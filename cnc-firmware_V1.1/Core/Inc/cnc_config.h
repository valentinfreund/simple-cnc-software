#ifndef CNC_CONFIG_H
#define CNC_CONFIG_H

#define CNC_TIMER_FREQ_HZ        100000ULL  // 100 kHz = 10µs pro Tick
#define CNC_MAX_SEGMENT_TICKS    4000       // Max 40ms bei 100kHz

#define X_STEP_PIN_MASK          (1 << 0)   // Pin E0
#define Y1_STEP_PIN_MASK         (1 << 2)   // Pin E1 (Linker Portal-Motor)
#define Y2_STEP_PIN_MASK         (1 << 3)   // Pin E2 (Rechter Portal-Motor)
#define Z_STEP_PIN_MASK          (1 << 1)   // Pin E3

#define ALL_STEP_PINS_MASK       (X_STEP_PIN_MASK | Y1_STEP_PIN_MASK | Y2_STEP_PIN_MASK | Z_STEP_PIN_MASK)

#endif // CNC_CONFIG_H
