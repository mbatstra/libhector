#include "Brownian.h"
#include <math.h>

/**
 * @brief Custom constructor. Always use this instead of the default constructor.
 * @param mean mean of the normal distribution to draw from
 * @param stddev standard deviation of the normal distribution to draw from
 * @param dt translational diffusion coefficient
 * @param dr rotational diffusion coefficient
 * @param activity 'strength' of activity
 * @param time_s step time in seconds
 * @param seed random number generator seed
 */
Brownian::Brownian(
  float mean, float stddev, float dt, float dr, float activity, float time_s, uint32_t seed)
  : ud_(std::uniform_real_distribution<float>(0.0, 2.0*M_PI)),
    nd_(std::normal_distribution(mean, stddev)),
    gen_(seed),
    dt_(dt),
    dr_(dr),
    phi_(0.0),  /* Should this also depend on the seed? Doesn't matter for passive Brownian */
    activity_(activity),
    time_s_(time_s)
{}

/**
 * @brief Get the next step of passive brownian motion.
 * @return 2d motion vector
 */
Eigen::Vector2d Brownian::get_passive() {
  /* this is technically only part of the noise term, but we do not have to take the sqrt twice */
  float noise_term = sqrt(2 * time_s_ * dt_);
  return Eigen::Vector2d(
    draw_() * noise_term * 52, /* multiply by robot radius so that displacement is in units of robot radii */
    draw_() * noise_term * 52
  );
}

/**
 * @brief Get the next step of active brownian motion.
 * @return 2d motion vector
 */
Eigen::Vector2d Brownian::get_active() {
  Eigen::Vector2d passive = get_passive();
  phi_ += draw_() * sqrt(2 * time_s_ * dr_);
  return Eigen::Vector2d(
    passive[0] * activity_ * time_s_ * cos(phi_),
    passive[1] * activity_ * time_s_ * sin(phi_)
  );
}
