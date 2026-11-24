#ifndef BROWNIAN_H
#define BROWNIAN_H

#include <cstdint>
#include <random>
#include <Eigen/Dense>

/**
 * @class Brownian
 * @brief brownian motion generator.
 */
class Brownian {
public:
    Brownian(float mean, float stddev, uint32_t seed=0);
    Brownian() = default;
    virtual ~Brownian() = default;

    float draw();

private:
    std::uniform_real_distribution<float> ud_;
    std::normal_distribution<float> nd_;
    std::mt19937 gen_;
};

#endif // !BROWNIAN_H
