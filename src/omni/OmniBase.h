#ifndef __OMNIBASE_H
#define __OMNIBASE_H

#include "OmniThread.h"

/*
	The three-wheel omni base, application side.
	All public methods are safe to call from the main thread.
	The motors live in OmniThread (private), which runs them in its own task.
	The constructor does not touch hardware; call init() from setup().
*/
class OmniBase
{
	public:
		OmniBase() = default;

		// init hardware + motors, then start the FOC task
		// MOTORS_DEBUG_MODE / SENSORS_DEBUG_MODE: does not return (see config.h)
		bool init();
		void enableDrivers();
		void disableDrivers();

		// recalibrate all motors and store the result. Blocks for several seconds
		// (alignment movement, wheels free). Drivers must be on; motors are disabled afterwards
		bool calibrate();

		// removes the stored calibration: the next boot calibrates (pauses the thread, motors disabled)
		bool clearCalibration();

		// Commands, executed in order by the FOC task. Return the seq of the (last) queued command;
		// block while the queue is full
		uint32_t enable();
		uint32_t disable();
		// Robot frame: x forward, y left, rotation CCW.
		// ramp to the speed (vx, vy [m/s], w [rad/s]), done when reached. With a duration [s]: back to 0
		// after the duration (counted from the speed change), done at standstill; covers exactly v * duration
		uint32_t velocity(float vx, float vy, float w, float duration = INFINITY);
		// move (dx, dy [m], dr [rad]) at most at v_xy [m/s] and v_r [rad/s], done at the (exact) endpoint.
		// Only from standstill: after a velocity brake() first (ERROR otherwise)
		uint32_t move(float dx, float dy, float dr, float v_xy, float v_r);
		uint32_t brake();			// ramp to standstill, done when reached
		uint32_t wait(float seconds);	// hold up the queue
		uint32_t setPose(float x, float y, float phi);	// the base stands at (x, y [m], phi [rad]) in the world frame now; done at once
		uint32_t stop();			// clears the queue, aborts the running command, depowers; within one pass

		OmniTelemetry telemetry();	// latest snapshot from the FOC task
		// command with seq 'seq' is finished, or cleared by stop()? Judged on snapshot t, so
		// decisions in one go use one moment (each telemetry() call is a new snapshot)
		static bool done(const OmniTelemetry& t, uint32_t seq);

	private:
		OmniThread _thread;

		uint32_t send(OmniCommand cmd);		// _thread.send(), with a warning when it is going to block

		// the primitive commands; chained: part of the same call as the command before it
		uint32_t sendVelocity(float vx, float vy, float w, bool chained);
		uint32_t sendMove(float dx, float dy, float dr, float v_xy, float v_r, bool chained);
		uint32_t sendWait(bool chained);
		uint32_t sendWaitTime(float seconds, bool chained);

	public: // Disable copying of these objects, they control hardware. Public for better error messages
		OmniBase(const OmniBase&) = delete;
		OmniBase& operator=(const OmniBase&) = delete;
};

#endif // __OMNIBASE_H
