#ifndef LIBHECTOR_H
#define LIBHECTOR_H

#include <cstdint>

void hector_init();
void hector_service();

void hector_move(float x_mm, float y_mm, float v_mms);
bool hector_is_idle();

void brownian_init(float mean, float stddev, uint32_t seed=0);
float brownian_draw();

#endif // !LIBHECTOR_H
