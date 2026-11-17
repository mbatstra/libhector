#include "OmniSegment.h"
#include "config.h"

#include <math.h>

//*********************** VelocitySegment ****************************/

// New ramp from (theta, omega) at start_us to omega_b. The busiest wheel ramps at WHEEL_ACCEL_MAX,
// the others proportionally slower: all arrive together. The duration is whole microseconds and
// targets() uses exactly that, so a ramp up and the same ramp down are identical (exact distances)
void VelocitySegment::start(const float theta[3], const float omega[3], const float omega_b[3], int64_t start_us)
{
	float delta_max = 0.0f;
	for (int i = 0; i < 3; i++)
	{
		_theta_s[i] = theta[i];
		_omega_a[i] = omega[i];
		_omega_b[i] = omega_b[i];
		delta_max = fmaxf(delta_max, fabsf(omega_b[i] - omega[i]));
	};

	_start_us = start_us;
	_ramp_us = (int64_t)(delta_max / WHEEL_ACCEL_MAX * 1e6f);
};

// Closed form. tau = time since the start of the ramp, t_r = ramp time, delta = omega_b - omega_a:
//
//   tau < t_r:   omega = omega_a + delta * tau / t_r    theta = theta_s + omega_a * tau + 1/2 * delta * tau^2 / t_r
//   tau >= t_r:  omega = omega_b                        theta = theta_s + 1/2 * (omega_a + omega_b) * t_r + omega_b * (tau - t_r)
void VelocitySegment::targets(int64_t t_us, float angle[3], float velocity[3]) const
{
	int64_t elapsed_us = t_us - _start_us;
	if (elapsed_us < 0)
		elapsed_us = 0;

	float t = elapsed_us * 1e-6f;				// [s] since the start of the ramp
	float t_r = _ramp_us * 1e-6f;				// [s] ramp duration

	// on the ramp: the speed changes linearly from omega_a to omega_b
	if (elapsed_us < _ramp_us)
	{
		for (int i = 0; i < 3; i++)
		{
			float delta = _omega_b[i] - _omega_a[i];
			velocity[i] = _omega_a[i] + delta * t / t_r;
			angle[i] = _theta_s[i] + _omega_a[i] * t + 0.5f * delta * t * t / t_r;
		};
		return;
	};

	// after the ramp: constant speed omega_b
	for (int i = 0; i < 3; i++)
	{
		velocity[i] = _omega_b[i];
		angle[i] = _theta_s[i] + 0.5f * (_omega_a[i] + _omega_b[i]) * t_r + _omega_b[i] * (t - t_r);
	};
};

// Closed form: nothing to integrate
void VelocitySegment::advance(int64_t now_us)
{
};

// Continue from where the wheels stand, at the target speed of now: after DISABLE/STOP that is 0,
// while already enabled a running velocity continues without a jolt. The target speed stays
void VelocitySegment::latch(const float measured[3], int64_t now_us)
{
	float angle[3];
	float velocity[3];
	targets(now_us, angle, velocity);

	float omega_b[3];
	for (int i = 0; i < 3; i++)
		omega_b[i] = _omega_b[i];
	start(measured, velocity, omega_b, now_us);
};

int64_t VelocitySegment::endTime() const
{
	return _start_us + _ramp_us;			// the target speed is reached
};

// Target speed 0, ramp done
bool VelocitySegment::standingStill(int64_t t_us) const
{
	for (int i = 0; i < 3; i++)
	{
		if (_omega_b[i] != 0.0f)
			return false;
	};
	return t_us >= _start_us + _ramp_us;
};

// Ramp-to-ramp: a new speed can always take over
bool VelocitySegment::acceptsVelocity(int64_t t_us) const
{
	return true;
};

//*********************** MoveSegment ****************************/

// theta_i = theta0_i + d_i * s, s from 0 to 1 with a trapezoid.
// The limits in s units; each term only when its distance isn't 0, the lowest wins:
//
//   d_max = max|d_i|      dxy = hypot(dx, dy)
//   V   = min( WHEEL_SPEED_MAX / d_max,  v_xy / dxy,           v_r / |dr| )            [1/s]   speed of s
//   A   = min( WHEEL_ACCEL_MAX / d_max,  MOVE_ACCEL_XY / dxy,  MOVE_ACCEL_R / |dr| )   [1/s^2] acceleration of s
//
// s = 0, s_dot = 0 at start_us (chained: may lie in the past, the first step catches up).
// Nothing to move (d_max = 0): finished at its start
bool MoveSegment::start(const float theta0[3], const float d[3], float dxy, float dr, float v_xy, float v_r, int64_t start_us)
{
	// a distance needs a speed
	if (dxy > 0.0f && v_xy <= 0.0f)
		return false;
	if (dr != 0.0f && v_r <= 0.0f)
		return false;

	float d_max = 0.0f;
	for (int i = 0; i < 3; i++)
	{
		_theta0[i] = theta0[i];
		_d[i] = d[i];
		d_max = fmaxf(d_max, fabsf(d[i]));
	};

	_s_dot = 0.0f;
	_last_us = start_us;

	// nothing to move: finished at once
	if (d_max == 0.0f)
	{
		_s = 1.0f;
		_end_us = start_us;
		return true;
	};

	float v_s = WHEEL_SPEED_MAX / d_max;			// the busiest wheel
	float a_s = WHEEL_ACCEL_MAX / d_max;
	if (dxy > 0.0f)
	{
		v_s = fminf(v_s, v_xy / dxy);				// translation
		a_s = fminf(a_s, MOVE_ACCEL_XY / dxy);
	};
	if (dr != 0.0f)
	{
		v_s = fminf(v_s, v_r / fabsf(dr));			// rotation
		a_s = fminf(a_s, MOVE_ACCEL_R / fabsf(dr));
	};

	_v_s = v_s;
	_a_s = a_s;
	_s = 0.0f;
	_end_us = INT64_MAX;
	return true;
};

// Where the last step left it (t_us not used: advance() moves s)
//
//   theta = theta0 + d * s      omega = d * s_dot
void MoveSegment::targets(int64_t t_us, float angle[3], float velocity[3]) const
{
	for (int i = 0; i < 3; i++)
	{
		angle[i] = _theta0[i] + _d[i] * _s;
		velocity[i] = _d[i] * _s_dot;
	};
};

// One step of the trapezoid:
//
//   dt       = now - last step
//   v_target = min(V, sqrt(2 * A * (1 - s)))    never faster than it can still stop at s = 1 (later: * hold)
//   s_dot    towards v_target, at most A * dt per step (accelerate and decelerate)
//   s       += s_dot * dt
//   s >= 1 - 1e-6:  s = 1, s_dot = 0, end = now (exact endpoint; float: don't crawl through the last ulps)
void MoveSegment::advance(int64_t now_us)
{
	// finished: s stays at 1
	if (_end_us != INT64_MAX)
		return;

	float dt = (now_us - _last_us) * 1e-6f;		// measured: overruns don't matter
	_last_us = now_us;

	float v_target = _v_s;
	float v_brake = sqrtf(2.0f * _a_s * (1.0f - _s));	// braking curve: exactly deceleration _a_s
	if (v_brake < v_target)
		v_target = v_brake;

	if (_s_dot < v_target)
	{
		_s_dot += _a_s * dt;					// accelerate, limited
		if (_s_dot > v_target)
			_s_dot = v_target;
	}
	else
	{
		_s_dot -= _a_s * dt;					// decelerate, limited
		if (_s_dot < v_target)
			_s_dot = v_target;
	};

	_s += _s_dot * dt;

	if (_s >= 1.0f - 1e-6f)
	{
		_s = 1.0f;
		_s_dot = 0.0f;
		_end_us = now_us;
	};
};

// A running move continues: shift its start so the target is the measured angle now
void MoveSegment::latch(const float measured[3], int64_t now_us)
{
	for (int i = 0; i < 3; i++)
		_theta0[i] = measured[i] - _d[i] * _s;
};

int64_t MoveSegment::endTime() const
{
	return _end_us;							// INT64_MAX while moving
};

bool MoveSegment::standingStill(int64_t t_us) const
{
	return t_us >= _end_us;
};

// Only once finished
bool MoveSegment::acceptsVelocity(int64_t t_us) const
{
	return t_us >= _end_us;
};
