#ifndef __MAGNETICSENSORAS5047UVEL_H
#define __MAGNETICSENSORAS5047UVEL_H

#include <SimpleFOC.h>
#include <encoders/as5047u/MagneticSensorAS5047U.h>

/*
	MagneticSensorAS5047U with a different velocity calculation.

	Sensor::getVelocity() skips the update when the angle did not change and
	keeps returning the previous velocity. At standstill (angle flickering by
	1-2 LSB) that holds a ~1 LSB/Ts spike for many loops, which the velocity
	LPF can't average out -> low-frequency velocity noise and rattling at 0.

	This version always recomputes delta_angle / Ts (so 0 when nothing moved).
	The min_elapsed_time check is kept.
*/
class MagneticSensorAS5047UVel : public MagneticSensorAS5047U
{
public:
	using MagneticSensorAS5047U::MagneticSensorAS5047U;

	float getVelocity() override
	{
		float Ts = (angle_prev_ts - vel_angle_prev_ts) * 1e-6f;
		if (Ts < 0.0f)	// micros() overflow: resync and keep last value
		{
			vel_angle_prev = angle_prev;
			vel_full_rotations = full_rotations;
			vel_angle_prev_ts = angle_prev_ts;
			return velocity;
		};
		if (Ts < min_elapsed_time)
			return velocity;

		// split full rotations from angle to avoid float precision loss
		float delta_angle = (float)(full_rotations - vel_full_rotations) * _2PI + (angle_prev - vel_angle_prev);
		velocity = delta_angle / Ts;

		vel_angle_prev = angle_prev;
		vel_full_rotations = full_rotations;
		vel_angle_prev_ts = angle_prev_ts;
		return velocity;
	};
};

#endif // __MAGNETICSENSORAS5047UVEL_H
