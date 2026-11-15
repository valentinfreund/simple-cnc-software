#include "cnc_kinematics.h"
#include "cnc_ringbuffer.h"
#include "stm32h7xx_hal.h"

// if configured in non-Cacheable RAM
SPI_Poly_Segment_t rx_segment_buffer __attribute__((section(".dma_buffer")));
SPI_Status_Response_t tx_response_buffer __attribute__((section(".dma_buffer")));

extern CNC_SPI_RingBuffer_t spi_input_buffer;
extern uint8_t current_machine_state;
extern uint32_t last_executed_id;

uint16_t CalculateCRC16(const uint8_t* data, uint32_t length) {
    uint16_t crc = 0xFFFF; // Startwert
    for (uint32_t i = 0; i < length; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021; // Polynom
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

// fets called after reading 60 byte
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    
    uint16_t calculated_crc = CalculateCRC16((uint8_t*)&rx_segment_buffer, sizeof(SPI_Poly_Segment_t) - 2);
    
    tx_response_buffer.last_processed_id = last_executed_id;
    tx_response_buffer.free_segments = CNC_RingBuffer_GetFreeSpace(&spi_input_buffer);

    if (calculated_crc != rx_segment_buffer.crc16) {
        // NACK
        tx_response_buffer.status = (current_machine_state << 4) | HOST_STATUS_NACK;
    } 
    else {
        uint8_t cmd = rx_segment_buffer.flags & 0x0F;
        
        // Pings
        if (rx_segment_buffer.flags != 0xFF) {
            
            if (cmd == CNC_CMD_ABORT) {
                current_machine_state = STATE_IDLE;
                CNC_RingBuffer_Init(&spi_input_buffer);
            }
            else {
                CNC_RingBuffer_Push(&spi_input_buffer, &rx_segment_buffer);
            }
        }
        
        // ACK
        tx_response_buffer.status = (current_machine_state << 4) | HOST_STATUS_ACK;
    }

    // reactivate DMA
    HAL_SPI_TransmitReceive_DMA(hspi, (uint8_t*)&tx_response_buffer, (uint8_t*)&rx_segment_buffer, sizeof(SPI_Poly_Segment_t));
}
