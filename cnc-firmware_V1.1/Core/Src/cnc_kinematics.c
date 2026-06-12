#include "cnc_kinematics.h"

#define FIXED_POINT_SHIFT 32
#define TO_FIXED(f) ((int64_t)((f) * (double)(1ULL << FIXED_POINT_SHIFT)))

void CNC_InitSegmentFDA(const SPI_Poly_Segment_t* msg, FDA_Axis_t* x, FDA_Axis_t* y1, FDA_Axis_t* y2, FDA_Axis_t* z) {
    x->pos = 0;
    x->delta1 = TO_FIXED(msg->x_v0 + 0.5f * msg->x_a0 + (1.0f / 6.0f) * msg->x_j);
    x->delta2 = TO_FIXED(msg->x_a0 + msg->x_j);
    x->delta3 = TO_FIXED(msg->x_j);

    y1->pos = 0;
    y1->delta1 = TO_FIXED(msg->y1_v0 + 0.5f * msg->y1_a0 + (1.0f / 6.0f) * msg->y1_j);
    y1->delta2 = TO_FIXED(msg->y1_a0 + msg->y1_j);
    y1->delta3 = TO_FIXED(msg->y1_j);

    y2->pos = 0;
    y2->delta1 = TO_FIXED(msg->y2_v0 + 0.5f * msg->y2_a0 + (1.0f / 6.0f) * msg->y2_j);
    y2->delta2 = TO_FIXED(msg->y2_a0 + msg->y2_j);
    y2->delta3 = TO_FIXED(msg->y2_j);

    z->pos = 0;
    z->delta1 = TO_FIXED(msg->z_v0 + 0.5f * msg->z_a0 + (1.0f / 6.0f) * msg->z_j);
    z->delta2 = TO_FIXED(msg->z_a0 + msg->z_j);
    z->delta3 = TO_FIXED(msg->z_j);
}
