#include "cnc_ringbuffer.h"
#include <string.h>

void CNC_RingBuffer_Init(CNC_SPI_RingBuffer_t* rb) {
    rb->head = 0;
    rb->tail = 0;
}

uint16_t CNC_RingBuffer_GetFreeSpace(const CNC_SPI_RingBuffer_t* rb) {
    int32_t free_space = rb->tail - rb->head - 1;
    if (free_space < 0) {
        free_space += SPI_RING_BUFFER_SIZE;
    }
    return (uint16_t)free_space;
}

bool CNC_RingBuffer_IsEmpty(const CNC_SPI_RingBuffer_t* rb) {
    return (rb->head == rb->tail);
}

bool CNC_RingBuffer_Push(CNC_SPI_RingBuffer_t* rb, const SPI_Poly_Segment_t* item) {
    uint16_t next_head = (rb->head + 1) % SPI_RING_BUFFER_SIZE;
    
    if (next_head == rb->tail) {
        return false;
    }
    
    // copy to ring buffer
    memcpy(&(rb->data[rb->head]), item, sizeof(SPI_Poly_Segment_t));
    rb->head = next_head;
    return true;
}

bool CNC_RingBuffer_Pop(CNC_SPI_RingBuffer_t* rb, SPI_Poly_Segment_t* out_item) {
    if (CNC_RingBuffer_IsEmpty(rb)) {
        return false;
    }
    
    memcpy(out_item, &(rb->data[rb->tail]), sizeof(SPI_Poly_Segment_t));
    rb->tail = (rb->tail + 1) % SPI_RING_BUFFER_SIZE;
    return true;
}
