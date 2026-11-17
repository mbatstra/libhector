#ifndef __OMNIMOTION_H
#define __OMNIMOTION_H

#include <stdint.h>

#include "OmniTypes.h"
#include "OmniSegment.h"

/*
	Motion targets of the three wheels. Thread side, no hardware: OmniThread passes the time
	and the measured angles in, and writes the targets into the motors.
	Commands are in the robot frame (x forward, y left, r CCW); the kinematics matrix J turns
	them into wheel angles and speeds. OmniMotion holds the rules (which command may start when)
	and points at the active segment (OmniSegment.h), which does the motion itself.
	It also integrates the pose from the measured wheel angles (wheel odometry).
*/
class OmniMotion
{
	public:
		OmniMotion();				// fills the kinematics matrix and its inverse from config.h

		void begin(const float measured[3], int64_t now_us);		// once, before the first update(): the wheels stand here; pose 0
		void latch(const float measured[3], int64_t now_us);		// continue from the measured angles (ENABLE), keep the current speed / move
		void stop(int64_t now_us);									// freeze the targets where they are now, no ramp (DISABLE, STOP)

		// ramp from the current speed to (vx, vy, vr) [m/s, m/s, rad/s]. false: a MOVE hasn't finished
		bool startVelocity(float vx, float vy, float vr, int64_t start_us);
		// move (dx, dy, dr) [m, m, rad] at most at v_xy [m/s] and v_r [rad/s].
		// false: not standing still, or no speed for a distance
		bool startMove(float dx, float dy, float dr, float v_xy, float v_r, int64_t start_us);

		// every pass, exactly once: advance the motion to now_us, then where each wheel should be [rad]
		// and how fast it should turn [rad/s] (the feed-forward). measured: the wheel angles now [rad]
		void update(int64_t now_us, const float measured[3], float angle[3], float velocity[3]);
		bool reached(int64_t t_us) const;	// the motion has reached what was commanded at t_us: the target speed (VELOCITY), the endpoint (MOVE)
		int64_t endTime() const;			// when that happened: valid once reached()

		OmniPose pose() const;				// the pose now
		void setPose(const OmniPose& pose);	// the base stands at pose now [m, m, rad]

	private:
		void toWheels(float x, float y, float r, float wheel[3]) const;		// robot frame -> wheels with J (distances or speeds)
		void toRobot(const float wheel[3], float& x, float& y, float& r) const;	// wheels -> robot frame with J^-1
		void integratePose(const float wheel[3]);							// add the step from _last_angle to wheel, then store wheel

		VelocitySegment _velocity;
		MoveSegment _move;
		Segment* _segment = &_velocity;		// the active one: always one of the two above

		float _J[3][3];				// kinematics: wheels = _J * (x, y, r)
		float _J_inv[3][3];			// and back: (x, y, r) = _J_inv * wheels

		OmniPose _pose = {};
		float _last_angle[3] = {};	// [rad] the measured wheel angles integrated last
};

#endif // __OMNIMOTION_H
