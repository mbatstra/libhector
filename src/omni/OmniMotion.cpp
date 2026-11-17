#include "OmniMotion.h"
#include "config.h"

#include <math.h>

static const float DEG = (float)M_PI / 180.0f;		// degrees -> radians

// J must be invertible (toRobot). With the wheels 120 deg apart: det J ~ (1/r)^3 * L * cos(yaw)
static_assert(WHEEL_RADIUS > 0.0f, "WHEEL_RADIUS must be > 0");
static_assert(WHEEL_DISTANCE > 0.0f, "WHEEL_DISTANCE must be > 0: J not invertible");
static_assert(WHEEL_YAW > -90.0f && WHEEL_YAW < 90.0f, "WHEEL_YAW must lie between -90 and 90 deg: J not invertible");

// inv = adjugate / determinant. The cofactors written out: c[i][j] = (-1)^(i+j) * the 2x2 minor without row i, column j
static void invert3x3(const float m[3][3], float inv[3][3])
{
	float c[3][3];
	c[0][0] = m[1][1] * m[2][2] - m[1][2] * m[2][1];
	c[0][1] = m[1][2] * m[2][0] - m[1][0] * m[2][2];
	c[0][2] = m[1][0] * m[2][1] - m[1][1] * m[2][0];
	c[1][0] = m[0][2] * m[2][1] - m[0][1] * m[2][2];
	c[1][1] = m[0][0] * m[2][2] - m[0][2] * m[2][0];
	c[1][2] = m[0][1] * m[2][0] - m[0][0] * m[2][1];
	c[2][0] = m[0][1] * m[1][2] - m[0][2] * m[1][1];
	c[2][1] = m[0][2] * m[1][0] - m[0][0] * m[1][2];
	c[2][2] = m[0][0] * m[1][1] - m[0][1] * m[1][0];

	float det = m[0][0] * c[0][0] + m[0][1] * c[0][1] + m[0][2] * c[0][2];

	// the adjugate is the transposed cofactor matrix
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			inv[i][j] = c[j][i] / det;
	};
};

// Kinematics of yawed wheels: wheel i sits at angle alpha_i around the centre (symmetric, 120 deg
// apart), at distance L, and rolls along delta_i = alpha_i + 90 deg + yaw. Its rim moves with the body
// velocity projected on delta_i, plus the rotation times the lever arm L * sin(delta_i - alpha_i)
OmniMotion::OmniMotion()
{
	const float k = WHEEL_DIRECTION / WHEEL_RADIUS;		// rim distance [m] -> wheel angle [rad]

	for (int i = 0; i < 3; i++)
	{
		float alpha = (WHEEL0_ANGLE + WHEEL_NUMBERING * 120.0f * i) * DEG;
		float delta = alpha + (90.0f + WHEEL_YAW) * DEG;
		_J[i][0] = k * cosf(delta);
		_J[i][1] = k * sinf(delta);
		_J[i][2] = k * WHEEL_DISTANCE * sinf(delta - alpha);
	};

	invert3x3(_J, _J_inv);
};

// The wheels stand where initFOC() left them, motors disabled: targets and pose start there
void OmniMotion::begin(const float measured[3], int64_t now_us)
{
	const float zero[3] = {};
	_velocity.start(measured, zero, zero, now_us);
	_segment = &_velocity;

	for (int i = 0; i < 3; i++)
		_last_angle[i] = measured[i];
	_pose = {};
};

// Continue from where the wheels stand (ENABLE): the active segment keeps its speed / move
void OmniMotion::latch(const float measured[3], int64_t now_us)
{
	_segment->latch(measured, now_us);
};

// Freeze the targets where they are now, without a ramp (DISABLE, STOP). Also ends a move
void OmniMotion::stop(int64_t now_us)
{
	float angle[3];
	float velocity[3];
	_segment->targets(now_us, angle, velocity);

	const float zero[3] = {};
	_velocity.start(angle, zero, zero, now_us);	// from 0 to 0: no ramp
	_segment = &_velocity;
};

// Continue from the current targets (not the measured angle): errors don't add up over a sequence.
// start_us may lie in the past (chained): the ramp starts exactly there
bool OmniMotion::startVelocity(float vx, float vy, float vr, int64_t start_us)
{
	// a move must have finished
	if (!_segment->acceptsVelocity(start_us))
		return false;

	float angle[3];
	float velocity[3];
	_segment->targets(start_us, angle, velocity);	// where the targets are at the start: no jump

	float omega_b[3];
	toWheels(vx, vy, vr, omega_b);
	_velocity.start(angle, velocity, omega_b, start_us);
	_segment = &_velocity;
	return true;
};

// Move (dx, dy, dr) in the robot frame: d = J * (dx, dy, dr) in wheel angles; the profile is MoveSegment's.
// Starts from the current targets at standstill
bool OmniMotion::startMove(float dx, float dy, float dr, float v_xy, float v_r, int64_t start_us)
{
	// from standstill only: brake() first, or WAIT for the previous move
	if (!_segment->standingStill(start_us))
		return false;

	float angle[3];
	float velocity[3];
	_segment->targets(start_us, angle, velocity);	// standing still: velocity is 0

	float d[3];
	toWheels(dx, dy, dr, d);
	if (!_move.start(angle, d, hypotf(dx, dy), dr, v_xy, v_r, start_us))
		return false;

	_segment = &_move;
	return true;
};

void OmniMotion::update(int64_t now_us, const float measured[3], float angle[3], float velocity[3]) // every pass
{
	_segment->advance(now_us);
	_segment->targets(now_us, angle, velocity);

	// the measured angles, not the targets: they see how the wheels really got there (a held wheel
	// catching up, pushed or coasting while disabled), and with rotation the path decides the pose
	integratePose(measured);
};

bool OmniMotion::reached(int64_t t_us) const
{
	return t_us >= _segment->endTime();		// a running move: INT64_MAX, never
};

int64_t OmniMotion::endTime() const
{
	return _segment->endTime();
};

OmniPose OmniMotion::pose() const
{
	return _pose;
};

// Only the pose: _last_angle stays, the next step continues from the wheels as they are
void OmniMotion::setPose(const OmniPose& pose)
{
	_pose = pose;
};

// One pass of the pose (wheel odometry). The wheel increment, not velocity * dt: no ramp error, and the
// increments telescope (sensor noise doesn't add up; constant heading: the sum is J^-1 * total change).
// Each step is turned into the world frame with the heading halfway the step: path-dependent, as poses are.
//
//   (dx, dy, dr) = J^-1 * (wheel - last)       robot frame
//   phi_mid      = phi + dr / 2
//   x += cos(phi_mid) * dx - sin(phi_mid) * dy
//   y += sin(phi_mid) * dx + cos(phi_mid) * dy
//   phi += dr
void OmniMotion::integratePose(const float wheel[3])
{
	float step[3];
	for (int i = 0; i < 3; i++)
	{
		step[i] = wheel[i] - _last_angle[i];
		_last_angle[i] = wheel[i];
	};

	float dx, dy, dr;
	toRobot(step, dx, dy, dr);

	float phi_mid = _pose.phi + 0.5f * dr;
	float c = cosf(phi_mid);
	float s = sinf(phi_mid);
	_pose.x += c * dx - s * dy;
	_pose.y += s * dx + c * dy;
	_pose.phi += dr;
};

// Robot frame (x, y, r) -> wheels with the J-matrix: distances [m, m, rad] -> wheel angles [rad],
// or speeds [m/s, m/s, rad/s] -> wheel speeds [rad/s]
void OmniMotion::toWheels(float x, float y, float r, float wheel[3]) const
{
	const float body[3] = { x, y, r };
	for (int i = 0; i < 3; i++)
	{
		wheel[i] = 0.0f;
		for (int j = 0; j < 3; j++)
			wheel[i] += _J[i][j] * body[j];
	};
};

// Wheels -> robot frame with J^-1: wheel angles [rad] -> distances [m, m, rad]
void OmniMotion::toRobot(const float wheel[3], float& x, float& y, float& r) const
{
	float body[3];
	for (int i = 0; i < 3; i++)
	{
		body[i] = 0.0f;
		for (int j = 0; j < 3; j++)
			body[i] += _J_inv[i][j] * wheel[j];
	};

	x = body[0];
	y = body[1];
	r = body[2];
};
