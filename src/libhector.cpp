#include "libhector.h"
#include "omni/OmniBase.h"
#include "omni/OmniTypes.h"

static OmniBase base;

void hector_init() {
    base.init();
    base.enable();
}

void hector_move(float x_mm, float y_mm, float v_mms) {
    base.move(x_mm / 1000.0f, y_mm / 1000.0f, 0.0f, 0.5f, 0.0f);
}

bool hector_is_idle() {
    return base.telemetry().state != OmniCommandState::RUNNING;
}
