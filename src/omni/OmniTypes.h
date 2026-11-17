#ifndef __OMNITYPES_H
#define __OMNITYPES_H

#include <stdint.h>

/*
	Types shared by OmniBase (application side) and OmniThread (FOC task).
*/

// ENABLE, DISABLE, VELOCITY, MOVE, SET_POSE complete at once; WAIT (until the target speed is reached or
// the move has ended) and WAIT_TIME (duration) hold up the queue
enum class OmniCommandType : uint8_t { ENABLE, DISABLE, VELOCITY, MOVE, WAIT, WAIT_TIME, SET_POSE, STOP };

// Lifecycle of a command. INVALID: nothing published yet, or a bug (no state set). ERROR: refused, not executed
enum class OmniCommandState : uint8_t { INVALID, IDLE, RUNNING, DONE, ERROR };
inline const char* const OmniCommandStateStr[] = {"INVALID", "IDLE", "RUNNING", "DONE", "ERROR"};

// One entry in the command queue (main -> FOC task). Executed strictly in order; STOP overrules.
struct OmniCommand
{
	OmniCommandType	type;
	bool			chained;		// part of one OmniBase call: starts when the previous command ended (not when
									// taken from the queue), skipped (ERROR) when the previous one failed
	uint32_t		seq;			// assigned by OmniThread::send()
	float			x, y, r;		// robot frame. VELOCITY: speed to ramp to [m/s, m/s, rad/s]; MOVE: distance [m, m, rad].
									// SET_POSE: world frame, the new pose [m, m, rad]
	float			v_xy, v_r;		// MOVE: speed limits [m/s, rad/s]
	float			duration;		// WAIT_TIME: [s]
};

// Pose [m, m, rad], world frame: origin and heading 0 = at boot (or SET_POSE), phi CCW, not wrapped (one turn = 2 pi).
// Wheel odometry: integrated from the measured wheel angles, so it sees lag and pushes, not slip
struct OmniPose
{
	float x, y, phi;
};

// Snapshot of the FOC task (FOC task -> main, last-one-wins). All fields belong to one moment.
struct OmniTelemetry
{
	uint32_t			seq;		// last started command (0 = none yet)
	OmniCommandState	state;		// of that command
	OmniPose			pose;		// wheel odometry
};

#endif // __OMNITYPES_H
