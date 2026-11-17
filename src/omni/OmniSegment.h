#ifndef __OMNISEGMENT_H
#define __OMNISEGMENT_H

#include <stdint.h>

/*
	One piece of motion of the three wheels, in wheel space (no J, no robot frame: OmniMotion does that).
	OmniMotion owns one of each as members and points at the active one; start() reuses the object.

	- VelocitySegment: one ramp at a time: from speed omega_a (at angle theta_s, time _start_us) the wheel
	  speeds change linearly to omega_b in _ramp_us, then stay there. All wheels arrive together; the
	  busiest wheel ramps at WHEEL_ACCEL_MAX. Closed form in time: exact, independent of the passes.
	  A new target starts a new ramp from where the targets are at that moment (ramp-to-ramp).
	- MoveSegment: theta_i = theta0_i + d_i * s. The progress s goes 0 -> 1 with a trapezoid, integrated
	  every pass (so hold can slow it down later) and clamped at 1: exact endpoint. Starts and ends at
	  standstill; after the end the target stays at theta0 + d.
*/
class Segment
{
	public:
		virtual void targets(int64_t t_us, float angle[3], float velocity[3]) const = 0;	// where the wheels should be at t_us [rad, rad/s]; advances nothing
		virtual void advance(int64_t now_us) = 0;							// every pass, exactly once
		virtual void latch(const float measured[3], int64_t now_us) = 0;	// continue from the measured angles (ENABLE)
		virtual int64_t endTime() const = 0;								// target speed / endpoint reached; INT64_MAX = not yet
		virtual bool standingStill(int64_t t_us) const = 0;					// a MOVE may start
		virtual bool acceptsVelocity(int64_t t_us) const = 0;				// a new VELOCITY may start from here at t_us
};

class VelocitySegment : public Segment
{
	public:
		// ramp from (theta, omega) at start_us to omega_b
		void start(const float theta[3], const float omega[3], const float omega_b[3], int64_t start_us);

		void targets(int64_t t_us, float angle[3], float velocity[3]) const override;
		void advance(int64_t now_us) override;
		void latch(const float measured[3], int64_t now_us) override;
		int64_t endTime() const override;
		bool standingStill(int64_t t_us) const override;
		bool acceptsVelocity(int64_t t_us) const override;

	private:
		float _theta_s[3] = {};		// [rad] target at the start of the ramp
		float _omega_a[3] = {};		// [rad/s] speed at the start of the ramp
		float _omega_b[3] = {};		// [rad/s] target speed, reached at the end of the ramp
		int64_t _start_us = 0;		// start of the ramp
		int64_t _ramp_us = 0;		// duration of the ramp
};

class MoveSegment : public Segment
{
	public:
		// move the wheels by d [rad] from theta0; dxy, dr and the speeds are the robot-frame command (the limits).
		// false: a distance without a speed
		bool start(const float theta0[3], const float d[3], float dxy, float dr, float v_xy, float v_r, int64_t start_us);

		void targets(int64_t t_us, float angle[3], float velocity[3]) const override;
		void advance(int64_t now_us) override;
		void latch(const float measured[3], int64_t now_us) override;
		int64_t endTime() const override;
		bool standingStill(int64_t t_us) const override;
		bool acceptsVelocity(int64_t t_us) const override;

	private:
		float _theta0[3] = {};		// [rad] target at s = 0
		float _d[3] = {};			// [rad] wheel angles of the whole move
		float _s = 0.0f;			// progress 0 -> 1
		float _s_dot = 0.0f;		// progress speed [1/s]
		float _v_s = 0.0f;			// cruise speed of s [1/s]
		float _a_s = 0.0f;			// acceleration of s [1/s^2]
		int64_t _last_us = 0;		// time of the last step
		int64_t _end_us = 0;		// when s reached 1; INT64_MAX = still moving
};

#endif // __OMNISEGMENT_H
