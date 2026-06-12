#ifndef CNC_RINGBUFFER_H
#define CNC_RINGBUFFER_H

#include "cnc_kinematics.h"
#include <stdbool.h>

#define SPI_RING_BUFFER_SIZE   32  // Platz für 32 Segmente (bei 20ms pro Segment = 640ms Pufferzeit)

typedef struct {
    SPI_Poly_Segment_t data[SPI_RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} CNC_SPI_RingBuffer_t;

void CNC_RingBuffer_Init(CNC_SPI_RingBuffer_t* rb);
bool CNC_RingBuffer_Push(CNC_SPI_RingBuffer_t* rb, const SPI_Poly_Segment_t* item);
bool CNC_RingBuffer_Pop(CNC_SPI_RingBuffer_t* rb, SPI_Poly_Segment_t* out_item);
uint16_t CNC_RingBuffer_GetFreeSpace(const CNC_SPI_RingBuffer_t* rb);
bool CNC_RingBuffer_IsEmpty(const CNC_SPI_RingBuffer_t* rb);

#endif // CNC_RINGBUFFER_H
