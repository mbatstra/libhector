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
  Brownian(float mean, float stddev, float dt, float dr, float activity, float time_s, uint32_t seed=0);
  Brownian() = delete;
  virtual ~Brownian() = default;

  Eigen::Vector2d get_passive();
  Eigen::Vector2d get_active();

  float get_dt() const { return dt_; }
  float get_dr() const { return dr_; }
  float get_activity() const { return activity_; }
  float get_step_time_s() const { return time_s_; }
  void set_dt(float dt) { dt_ = dt; }
  void set_dr(float dr) { dr_ = dr; }
  void set_activity(float a) { activity_ = a; }
  void set_step_time_s(float t) { time_s_ = t; }

private:
  float draw_() { return nd_(gen_); }

  std::uniform_real_distribution<float> ud_;
  std::normal_distribution<float> nd_;
  std::mt19937 gen_;

  float dt_, dr_;   /* rotational and translational diffusion coefficients */
  float phi_;       /* angle for rotational diffusion */
  float activity_;  /* velocity of the active component */
  float time_s_;
};

#endif // !BROWNIAN_H
