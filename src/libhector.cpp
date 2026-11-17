#include "libhector.h"
#include "omni/OmniBase.h"

static OmniBase base;

void hector_init() {
    base.init();
    base.enable();
}

void hector_move(float x_mm, float y_mm, float v_mms) {
    base.move(x_mm / 1000.0f, y_mm / 1000.0f, 0.0f, 0.5f, 0.0f);
}
