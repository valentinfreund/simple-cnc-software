#ifndef CNC_KINEMATICS_H
#define CNC_KINEMATICS_H

#include <stdint.h>

// Protokoll-Flags
#define HOST_STATUS_ACK          0x01
#define HOST_STATUS_NACK         0x02

// 60-Byte spi poly segment
typedef struct __attribute__((packed)) {
    uint32_t segment_id;
    uint32_t total_ticks;
    
    float x_v0;  float x_a0;  float x_j;
    float z_v0;  float z_a0;  float z_j; // Z Achse
    float y1_v0; float y1_a0; float y1_j; // Y1 Achse
    float y2_v0; float y2_a0; float y2_j; // Y2 Achse     
 
    uint8_t  directions; // Bitmaske für DIR-Pins
    uint8_t  flags;      // 0xFF=Ping, 0x04=Homing, 0x05=Abort, 0x06=Probe
    uint16_t crc16;      // Prüfsumme
} SPI_Poly_Segment_t;

// 8-Byte response
typedef struct __attribute__((packed)) {
    uint32_t last_processed_id;
    uint16_t free_segments;
    uint8_t  status;     // Aufbau: (State << 4) | (ACK oder NACK)
    uint8_t  padding;    // Füllbyte für 8-Byte Alignment
} SPI_Status_Response_t;

typedef struct {
    int64_t pos;
    int64_t delta1;
    int64_t delta2;
    int64_t delta3;
} FDA_Axis_t;

void CNC_InitSegmentFDA(const SPI_Poly_Segment_t* msg, FDA_Axis_t* x, FDA_Axis_t* y1, FDA_Axis_t* y2, FDA_Axis_t* z);

#endif // CNC_KINEMATICS_H
