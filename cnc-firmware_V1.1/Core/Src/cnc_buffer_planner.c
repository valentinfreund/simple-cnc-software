#include "cnc_config.h"
#include "cnc_kinematics.h"

void CNC_GenerateSegmentBuffer(const SPI_Poly_Segment_t* msg, 
                               FDA_Axis_t* x, FDA_Axis_t* y1, FDA_Axis_t* y2, FDA_Axis_t* z,
                               uint16_t* dma_buffer) 
{
    int32_t last_pos_x  = 0; int32_t last_pos_y1 = 0;
    int32_t last_pos_y2 = 0; int32_t last_pos_z  = 0;

    uint32_t ticks = (msg->total_ticks > CNC_MAX_SEGMENT_TICKS) ? CNC_MAX_SEGMENT_TICKS : msg->total_ticks;

    for (uint32_t t = 0; t < ticks; t++) {
        uint16_t gpio_pattern = 0;

        int32_t curr_x = (int32_t)(x->pos >> 32);
        if (curr_x > last_pos_x) { gpio_pattern |= X_STEP_PIN_MASK; last_pos_x = curr_x; }
        x->pos += x->delta1; x->delta1 += x->delta2; x->delta1 += x->delta3;

        int32_t curr_y1 = (int32_t)(y1->pos >> 32);
        if (curr_y1 > last_pos_y1) { gpio_pattern |= Y1_STEP_PIN_MASK; last_pos_y1 = curr_y1; }
        y1->pos += y1->delta1; y1->delta1 += y1->delta2; y1->delta1 += y1->delta3;

        int32_t curr_y2 = (int32_t)(y2->pos >> 32);
        if (curr_y2 > last_pos_y2) { gpio_pattern |= Y2_STEP_PIN_MASK; last_pos_y2 = curr_y2; }
        y2->pos += y2->delta1; y2->delta1 += y2->delta2; y2->delta1 += y2->delta3;

        int32_t curr_z = (int32_t)(z->pos >> 32);
        if (curr_z > last_pos_z) { gpio_pattern |= Z_STEP_PIN_MASK; last_pos_z = curr_z; }
        z->pos += z->delta1; z->delta1 += z->delta2; z->delta1 += z->delta3;

        dma_buffer[t] = gpio_pattern;
    }
}
