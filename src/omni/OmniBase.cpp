#include "OmniBase.h"
#include "config.h"
#include "pins.h"
#include "log.h"

bool OmniBase::init()
{
	digitalWrite(PIN_MOTOR_nSLEEP, LOW);		// drivers off until we need them
	digitalWrite(PIN_MOTOR_ENABLE, LOW);
	pinMode(PIN_MOTOR_nSLEEP, OUTPUT);
	pinMode(PIN_MOTOR_ENABLE, OUTPUT);
	pinMode(PIN_MOTOR_nFAULT, INPUT);

#ifdef PIN_FOC_THREAD
	digitalWrite(PIN_FOC_THREAD, LOW);			// HIGH during each FOC pass (scope)
	pinMode(PIN_FOC_THREAD, OUTPUT);
#endif

	SimpleFOCDebug::enable(&Serial);			// all builds: init output + abnormal runtime messages

	OMNI_DEBUG_ATTACH(_thread);

	OMNIBASE_SPI.begin(PIN_SENSE_CLK, PIN_SENSE_MISO, PIN_SENSE_MOSI, -1);

#ifdef SENSORS_DEBUG_MODE
	OmniDebug::sensors();					// drivers still off, no motor init; does not return
#endif

	enableDrivers();

#if (PIN_CALIBRATE_ENABLED == 1)
	pinMode(PIN_CALIBRATE, INPUT_PULLUP);
	if(digitalRead(PIN_CALIBRATE) == LOW)
	{
		INFO("Calibration forced by PIN_CALIBRATE, clearing stored calibration");
		_thread.clearCalibration();
	};
#endif // PIN_CALIBRATE

	if(!_thread.init())
	{
		ERROR("Failed to initialize FOC thread.");
		return false;
	};

#ifdef MOTORS_DEBUG_MODE
	OmniDebug::motors();					// Commander + monitoring, starts the thread, monitor loop; does not return
#endif

	_thread.start();

	return true;
};

bool OmniBase::calibrate()
{
	return _thread.calibrate();
};

bool OmniBase::clearCalibration()
{
	return _thread.clearCalibration();
};

uint32_t OmniBase::enable()
{
	OmniCommand cmd = {};
	cmd.type = OmniCommandType::ENABLE;
	return send(cmd);
};

uint32_t OmniBase::disable()
{
	OmniCommand cmd = {};
	cmd.type = OmniCommandType::DISABLE;
	return send(cmd);
};

uint32_t OmniBase::velocity(float vx, float vy, float w, float duration)
{
	// INFINITY: ramp to the speed, done once it is reached
	if (isinf(duration))
	{
		sendVelocity(vx, vy, w, false);
		return sendWait(true);
	};

	// timed: the speed for 'duration' from the speed change on, then ramp down; done at standstill.
	// The ramp down mirrors the ramp up, so the base covers exactly v * duration
	sendVelocity(vx, vy, w, false);
	sendWaitTime(duration, true);
	sendVelocity(0, 0, 0, true);
	return sendWait(true);
};

uint32_t OmniBase::move(float dx, float dy, float dr, float v_xy, float v_r)
{
	sendMove(dx, dy, dr, v_xy, v_r, false);
	return sendWait(true);
};

uint32_t OmniBase::brake()
{
	sendVelocity(0, 0, 0, false);
	return sendWait(true);
};

uint32_t OmniBase::wait(float seconds)
{
	return sendWaitTime(seconds, false);
};

uint32_t OmniBase::setPose(float x, float y, float phi)
{
	OmniCommand cmd = {};
	cmd.type = OmniCommandType::SET_POSE;
	cmd.x = x;
	cmd.y = y;
	cmd.r = phi;
	return send(cmd);
};

uint32_t OmniBase::send(OmniCommand cmd)
{
	if (_thread.queueFull())
		WARNING("Command queue full (%d): waiting for room, raise FOC_COMMAND_QUEUE_LENGTH?", FOC_COMMAND_QUEUE_LENGTH);
	return _thread.send(cmd);
};

uint32_t OmniBase::sendVelocity(float vx, float vy, float w, bool chained)
{
	OmniCommand cmd = {};
	cmd.type = OmniCommandType::VELOCITY;
	cmd.chained = chained;
	cmd.x = vx;
	cmd.y = vy;
	cmd.r = w;
	return send(cmd);
};

uint32_t OmniBase::sendMove(float dx, float dy, float dr, float v_xy, float v_r, bool chained)
{
	OmniCommand cmd = {};
	cmd.type = OmniCommandType::MOVE;
	cmd.chained = chained;
	cmd.x = dx;
	cmd.y = dy;
	cmd.r = dr;
	cmd.v_xy = v_xy;
	cmd.v_r = v_r;
	return send(cmd);
};

uint32_t OmniBase::sendWait(bool chained)
{
	OmniCommand cmd = {};
	cmd.type = OmniCommandType::WAIT;
	cmd.chained = chained;
	return send(cmd);
};

uint32_t OmniBase::sendWaitTime(float seconds, bool chained)
{
	OmniCommand cmd = {};
	cmd.type = OmniCommandType::WAIT_TIME;
	cmd.chained = chained;
	cmd.duration = seconds;
	return send(cmd);
};

uint32_t OmniBase::stop()
{
	return _thread.stop();
};

OmniTelemetry OmniBase::telemetry()
{
	return _thread.telemetry();
};

bool OmniBase::done(const OmniTelemetry& t, uint32_t seq)
{
	if (t.seq > seq)
		return true;							// a later command started: this one finished or was cleared
	if (t.seq != seq)
		return false;
	return t.state == OmniCommandState::DONE || t.state == OmniCommandState::ERROR;	// ERROR: refused, also finished
};

void OmniBase::enableDrivers()
{
	DBG("Motor Drivers ON");
	digitalWrite(PIN_MOTOR_nSLEEP, HIGH);
	digitalWrite(PIN_MOTOR_ENABLE, HIGH);
	delay(1);									// driver wake-up time (check datasheet)
};

void OmniBase::disableDrivers()
{
	DBG("Motor Drivers OFF");
	digitalWrite(PIN_MOTOR_nSLEEP, LOW);
	digitalWrite(PIN_MOTOR_ENABLE, LOW);
};


