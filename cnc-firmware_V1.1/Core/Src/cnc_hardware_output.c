#include "cnc_config.h"
#include "cnc_kinematics.h"
#include "cnc_ringbuffer.h"

// Globale Pufferstrukturen im D2-AxiSRAM oder D1-RAM
uint16_t DMA_Buffer_0[CNC_MAX_SEGMENT_TICKS]; // (Memory 0)
uint16_t DMA_Buffer_1[CNC_MAX_SEGMENT_TICKS]; // (Memory 1)

FDA_Axis_t axis_x, axis_z, axis_y1, axis_y2;
CNC_SPI_RingBuffer_t spi_input_buffer;
volatile uint8_t next_buffer_to_fill = 0;

/**
 * called on DMA buffer completion, called in DMA-Interrupt-Service-Routine (ISR).
 */
void CNC_DMA_Buffer_Callback(void) {
    SPI_Poly_Segment_t next_segment;
    uint16_t* target_buffer = (next_buffer_to_fill == 0) ? DMA_Buffer_0 : DMA_Buffer_1;

    // next segment?
    if (CNC_RingBuffer_Pop(&spi_input_buffer, &next_segment)) {
        
        // fda algorithm
        CNC_InitSegmentFDA(&next_segment, &axis_x, &axis_y1, &axis_y2, &axis_z);
        
        // hardware buffer in RAM
        CNC_GenerateSegmentBuffer(&next_segment, &axis_x,  &axis_y1, &axis_y2, &axis_z, target_buffer);
        
        //TODO missing direction pin
        
        next_buffer_to_fill = (next_buffer_to_fill == 0) ? 1 : 0;
        
    } else {
        // buffer underrun
        CNC_EmergencyStop("Buffer Underrun!");
    }
    
    // messaging how many free slots are available
    uint16_t free_slots = CNC_RingBuffer_GetFreeSpace(&spi_input_buffer);
    if (free_slots > 5) {
        // GPIO-Pin `BUFFER_NOT_FULL` to HIGH
        // HAL_GPIO_WritePin(GPIOB, BUFFER_READY_PIN, GPIO_PIN_SET);
    } else {
        // Setze GPIO-Pin `BUFFER_NOT_FULL` auf LOW -> RPi muss warten
        // HAL_GPIO_WritePin(GPIOB, BUFFER_READY_PIN, GPIO_PIN_RESET);
    }
}

void CNC_EmergencyStop(const char* reason) {
//TODO
    // 1. Timer stoppen, DMA stoppen
    // 2. Alle STEP-Pins sofort auf LOW ziehen
    // 3. Servos über ENABLE-Pin abschalten (falls konfiguriert)
    // 4. In eine Endlosschleife gehen und auf Reset warten
    while(1);
}
