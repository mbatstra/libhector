#include "Brownian.h"
#include <math.h>

Brownian::Brownian(float mean, float stddev, uint32_t seed)
    : ud_(std::uniform_real_distribution<float>(0.0, 2.0 * M_PI)),
      nd_(std::normal_distribution(mean, stddev)),
      gen_(seed) {}

float Brownian::draw() {
    return nd_(gen_);
}
