#include "libhector.h"
#include "brownian/Brownian.h"
#include "omni/OmniBase.h"
#include "omni/OmniTypes.h"

static OmniBase base;
static Brownian brownian;

void hector_init() {
    base.init();
    base.enable();
}

void hector_move(float x_mm, float y_mm, float v_mms) {
    base.move(x_mm / 1000.0f, y_mm / 1000.0f, 0.0f, v_mms / 1000.0f, 0.0f);
}

bool hector_is_idle() {
    return base.telemetry().state != OmniCommandState::RUNNING;
}

void brownian_init(float mean, float stddev, uint32_t seed) {
    brownian = Brownian(mean, stddev, seed);
}

float brownian_draw() {
    return brownian.draw();
}
