#include "OmniThread.h"
#include "config.h"
#include "log.h"

#include <Preferences.h>
#include <esp_rom_crc.h>
#include <esp_timer.h>
#include <math.h>

static const uint16_t CALIBRATION_VERSION = 1;		// bump when CalibrationPref changes: old records are rejected
static const char* NVS_NAMESPACE = "omnibase";
static const char* NVS_KEY = "cal";

OmniThread::OmniThread()
	: motors{OmniMotor(0), OmniMotor(1), OmniMotor(2)}
{
};

bool OmniThread::init()	// MAIN
{
	CalibrationPref cal = {};		// uncalibrated unless loaded: initFOC() aligns

	bool loaded = cal.load();
	if (loaded)
		INFO("Using calibration data from NVS");
	if (!loaded)
		INFO("No valid calibration data in NVS -> will trigger calibration.");

	if (!initMotors(cal))
		return false;

	if (!loaded)
		saveCalibration(cal);

	return true;
};

bool OmniThread::initMotors(const CalibrationPref& cal) // MAIN
{
	bool ok = true;
	for (int i = 0; i < 3; i++)
		ok &= motors[i].init(cal.motor[i]);

	return ok;
};

void OmniThread::saveCalibration(CalibrationPref& cal) // MAIN
{
	// Get the cal data initFOC() found
	for (int i = 0; i < 3; i++)
		cal.motor[i] = motors[i].calibration();

	if (!cal.save())
		ERROR("FAILed to save motor calibration data to NVS");		// not fatal: motors are calibrated, just not stored
};

bool OmniThread::clearCalibration() // MAIN
{
	bool paused = pause();		// flash write: no passes during it
	bool ok = CalibrationPref::clear();
	if (paused)
		resume();
	return ok;
};

bool OmniThread::calibrate() // MAIN
{
	bool paused = pause();		// motors disabled from here on

	bool ok = true;
	for (OmniMotor& m : motors)
		ok &= m.calibrate();

	if (ok)
	{
		CalibrationPref cal = {};
		saveCalibration(cal);
	};
	if (!ok)
		ERROR("Calibration failed, nothing saved");

	if (paused)
		resume();
	return ok;
};

// --- CalibrationPref: the record in NVS -------------------------------------

bool OmniThread::CalibrationPref::load() // MAIN
{
	if (read())
		return true;

	*this = CalibrationPref{};		// missing or invalid: back to uncalibrated
	return false;
};

bool OmniThread::CalibrationPref::read() // MAIN
{
	Preferences prefs;
	if (!prefs.begin(NVS_NAMESPACE, true))		// read-only; fails when the namespace doesn't exist yet
		return false;
	size_t len = prefs.getBytes(NVS_KEY, this, sizeof(*this));
	prefs.end();

	if (len != sizeof(*this))
		return false;
	if (version != CALIBRATION_VERSION)
		return false;
	if (crc != checksum())
		return false;
	for (const OmniMotor::Calibration& m : motor)
	{
		if (m.direction != Direction::CW && m.direction != Direction::CCW)
			return false;
		if (!(m.zero_angle >= 0.0f && m.zero_angle < TWO_PI))		// also rejects NaN and NOT_SET
			return false;
	};
	return true;
};

bool OmniThread::CalibrationPref::save() // MAIN
{
	version = CALIBRATION_VERSION;
	crc = checksum();

	Preferences prefs;
	if (!prefs.begin(NVS_NAMESPACE, false))
		return false;
	bool ok = prefs.putBytes(NVS_KEY, this, sizeof(*this)) == sizeof(*this);
	prefs.end();
	return ok;
};

bool OmniThread::CalibrationPref::clear() // MAIN
{
	Preferences prefs;
	if (!prefs.begin(NVS_NAMESPACE, false))
		return false;
	prefs.remove(NVS_KEY);		// false when there was nothing to remove: fine
	prefs.end();
	return true;
};

uint32_t OmniThread::CalibrationPref::checksum() const // MAIN
{
	return esp_rom_crc32_le(0, (const uint8_t*)this, offsetof(CalibrationPref, crc));
};

void OmniThread::start() // MAIN
{
	_parked = xSemaphoreCreateBinary(); 	// makes main wait for thread to park
	_resume = xSemaphoreCreateBinary();		// makes thread wait until main allows it to go again
	_commands = xQueueCreate(FOC_COMMAND_QUEUE_LENGTH, sizeof(OmniCommand));
	_telemetry = xQueueCreate(1, sizeof(OmniTelemetry));
	_state = State::RUNNING;
	xTaskCreatePinnedToCore(taskEntry, "foc", FOC_TASK_STACK, this, FOC_TASK_PRIORITY, &_task, FOC_TASK_CORE);
};

bool OmniThread::pause() // MAIN
{
	if (_state != State::RUNNING)
		return false;

	_pause_request = true;
	xSemaphoreTake(_parked, portMAX_DELAY);		// wait for the thread being parked
	_state = State::PAUSED;
	return true;
};

void OmniThread::resume() // MAIN
{
	if (_state != State::PAUSED)
		return;

	_pause_request = false;
	_state = State::RUNNING;
	xSemaphoreGive(_resume);					// Allow thread to resume
};

uint32_t OmniThread::send(OmniCommand cmd) // MAIN
{
	cmd.seq = ++_seq;
	xQueueSend(_commands, &cmd, portMAX_DELAY);	// full: wait until the thread made room (raise FOC_COMMAND_QUEUE_LENGTH when this hurts)
	return cmd.seq;
};

bool OmniThread::queueFull() // MAIN
{
	return uxQueueSpacesAvailable(_commands) == 0;
};

uint32_t OmniThread::stop() // MAIN
{
	// only producer: after the reset nothing else can be queued before STOP, so it is at the head
	xQueueReset(_commands);

	OmniCommand cmd = {};
	cmd.type = OmniCommandType::STOP;
	return send(cmd);
};

OmniTelemetry OmniThread::telemetry() // MAIN
{
	OmniTelemetry t = {};						// nothing published yet: {0, INVALID}
	xQueuePeek(_telemetry, &t, 0);				// non-destructive: always the latest snapshot
	return t;
};

/*********** THREAD METHODS/FUNCTION *****************************************************************************************************************/
void OmniThread::taskEntry(void* arg)  // THREAD
{
	static_cast<OmniThread*>(arg)->run();
};

// gptimer alarm (ISR): wake the FOC task
bool IRAM_ATTR OmniThread::onTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t* event, void* ctx) // ISR (On Thread core)
{
	BaseType_t woken = pdFALSE;
	vTaskNotifyGiveFromISR(static_cast<OmniThread*>(ctx)->_task, &woken);
	return woken == pdTRUE;
};

void OmniThread::run() // THREAD
{
	_task = xTaskGetCurrentTaskHandle();		// valid before the first timer interrupt

	// timer set up from within the task, so its interrupt is allocated on the same core as the thread
	gptimer_config_t timer_config = {};
	timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
	timer_config.direction = GPTIMER_COUNT_UP;
	timer_config.resolution_hz = 1000000;		// 1 tick = 1 us
	ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &_timer));

	gptimer_event_callbacks_t callbacks = {};
	callbacks.on_alarm = onTimer;
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(_timer, &callbacks, this));

	gptimer_alarm_config_t alarm = {};
	alarm.alarm_count = 1000000 / FOC_LOOP_FREQUENCY;
	alarm.reload_count = 0;
	alarm.flags.auto_reload_on_alarm = true;
	ESP_ERROR_CHECK(gptimer_set_alarm_action(_timer, &alarm));

	// the wheels stand where initFOC() left them (motors disabled): targets and pose start there
	float measured[3];
	measuredAngles(measured);
	_motion.begin(measured, esp_timer_get_time());

	ESP_ERROR_CHECK(gptimer_enable(_timer));
	ESP_ERROR_CHECK(gptimer_start(_timer));

	uint32_t command_downsample = 0;
	while (true)
	{
		// Wait for Notify from timer interrupt
		uint32_t count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (_pause_request)
		{
			park();
			continue;						// no pass, no stats for this wake-up
		};

		// Optional DEBUG code
		OMNI_PASS_BEGIN(_stats, count);		// for THREAD_DEBUG

#ifndef MOTORS_DEBUG_MODE
		updateTargets();					// before the motors: this pass uses its own targets (debug: Commander sets target)
#endif

		// Server the motors (at full rate)
		for (OmniMotor& m : motors)
			m.loop();

		// Process incoming commands once in a while
		if (++command_downsample >= FOC_COMMAND_DOWNSAMPLE)
		{
			command_downsample = 0;
			processCommands();
			publishTelemetry();				// every command tick: always the current state
		};


		// More Debug (closing)
		OMNI_DEBUG_PASS();					// for MOTORS_DEBUG_MODE

		OMNI_PASS_END(_stats);				// for THREAD_DEBUG
	};
};

void OmniThread::park() // THREAD
{
	ESP_ERROR_CHECK(gptimer_stop(_timer));
	for (OmniMotor& m : motors)
		m.motor.disable();					// PWM to 0: coast instead of holding the last voltage vector
	xSemaphoreGive(_parked);				// pause() may continue: no pass running, motors off

	xSemaphoreTake(_resume, portMAX_DELAY); // Will wait for main task until given
	ulTaskNotifyTake(pdTRUE, 0);			// drop a notification left from before the stop
	ESP_ERROR_CHECK(gptimer_start(_timer));
};

void OmniThread::processCommands() // THREAD
{
	// STOP overrules everything, also a running command
	int64_t now = esp_timer_get_time();

	OmniCommand peek_next;						// peek only: must not overwrite _current_command
	bool has_next = xQueuePeek(_commands, &peek_next, 0) == pdTRUE;
	if (has_next && peek_next.type == OmniCommandType::STOP)
	{
		startNextCommand(now); // 'starts' the STOP
		return;
	};

	// running command (a wait): wait for it to end
	int64_t start_us = now;
	if (_current_command_state == OmniCommandState::RUNNING)
	{
		// WAIT: its end is the moment the motion got there (not this command tick)
		if (_current_command.type == OmniCommandType::WAIT && _motion.reached(now))
			_command_end_us = _motion.endTime();

		if (now < _command_end_us)
			return;

		// The wait has finished: the next one starts now but we set its start
		// time to the end of the wait (exact timing, not rounded to command ticks)
		_current_command_state = OmniCommandState::DONE;
		start_us = _command_end_us;
	};

	startNextCommand(start_us);					// nothing when the queue is empty
};

void OmniThread::startNextCommand(int64_t start_us) // THREAD
{
	bool previous_failed = _current_command_state == OmniCommandState::ERROR;

	// the actual head: may differ from the peek when stop() ran in between
	if (xQueueReceive(_commands, &_current_command, 0) != pdTRUE)
		return;								// empty queue: keep the current command

	_current_command_state = OmniCommandState::INVALID; 			// if this ever shows up, one of the branches below failed to set the proper state.

	// part of one OmniBase call: start where the previous command ended (it may have been
	// taken a command tick earlier), and skip the rest of the call when a part failed
	if (_current_command.chained)
	{
		start_us = _command_end_us;			// where the previous command ended
		if (previous_failed)
		{
			_current_command_state = OmniCommandState::ERROR;
			return;							// _command_end_us stays: nothing happened
		};
	};

	switch (_current_command.type)
	{
		case OmniCommandType::ENABLE:
		{
			// write our _current_ wheel position to the motion engine
			// (a running velocity keeps its speed, a running move continues)
			float measured[3];
			measuredAngles(measured);
			_motion.latch(measured, start_us);

			// and enable motors
			for (OmniMotor& m : motors)
				m.motor.enable();
			_current_command_state = OmniCommandState::DONE;
			break;
		};

		case OmniCommandType::DISABLE:
		case OmniCommandType::STOP:
			// freeze the motion engine: no meaningful angle while disabled, 
			// the next ENABLE latches the then-current position anyway
			_motion.stop(start_us);		
			for (OmniMotor& m : motors)
				m.motor.disable();
			_current_command_state = OmniCommandState::DONE;
			break;

		case OmniCommandType::VELOCITY:
			// a velocity can only work if not disabled
			// (refused, not waited out: the ENABLE behind it would only start afterwards)
			if(!motors[0].motor.enabled)
			{
				_current_command_state = OmniCommandState::ERROR;
				break;
			};

			// refused during a move (no blending yet)
			if (!_motion.startVelocity(_current_command.x, _current_command.y, _current_command.r, start_us))
			{
				_current_command_state = OmniCommandState::ERROR;
				break;
			};

			// done now: the targets ramp to the new speed and keeps it until the next VELOCITY
			_current_command_state = OmniCommandState::DONE;
			break;

		case OmniCommandType::MOVE:
			// like VELOCITY: refused while disabled
			if(!motors[0].motor.enabled)
			{
				_current_command_state = OmniCommandState::ERROR;
				break;
			};

			// refused when not standing still, or a distance without a speed
			if (!_motion.startMove(_current_command.x, _current_command.y, _current_command.r,
								   _current_command.v_xy, _current_command.v_r, start_us))
			{
				_current_command_state = OmniCommandState::ERROR;
				break;
			};

			// done now: the WAIT behind it waits for the end
			_current_command_state = OmniCommandState::DONE;
			break;

		case OmniCommandType::WAIT:
			// waits for a state: the motion has reached what was commanded. Already reached
			// (also while disabled: the targets are frozen): done at once
			if (_motion.reached(start_us))
			{
				_current_command_state = OmniCommandState::DONE;
				break;
			};

			_command_end_us = INT64_MAX;		// set once the motion gets there (processCommands)
			_current_command_state = OmniCommandState::RUNNING;
			break;

		case OmniCommandType::WAIT_TIME:
			if (!isfinite(_current_command.duration) || _current_command.duration < 0.0f)
			{
				_current_command_state = OmniCommandState::ERROR;
				break;
			};

			_command_end_us = start_us + (int64_t)(_current_command.duration * 1e6f);
			_current_command_state = OmniCommandState::RUNNING;
			break;

		case OmniCommandType::SET_POSE:
		{
			// in the queue, so after what came before it (a move that is still running keeps moving)
			OmniPose pose = { _current_command.x, _current_command.y, _current_command.r };
			_motion.setPose(pose);
			_current_command_state = OmniCommandState::DONE;
			break;
		};
	};

	// completed at once: ends at its start
	if (_current_command_state != OmniCommandState::RUNNING)
		_command_end_us = start_us;
};

// Write new targets to the motors (during a move command, the angle changes in time)
void OmniThread::updateTargets() // THREAD, every pass
{
	float measured[3];
	measuredAngles(measured);					// for the pose

	float angle[3];
	float velocity[3];
	_motion.update(esp_timer_get_time(), measured, angle, velocity);

	for (int i = 0; i < 3; i++)
	{
		motors[i].motor.target = angle[i];
		motors[i].motor.feed_forward_velocity = velocity[i];
	};
};

void OmniThread::measuredAngles(float angle[3]) // THREAD
{
	for (int i = 0; i < 3; i++)
		angle[i] = motors[i].motor.shaft_angle;
};

void OmniThread::publishTelemetry() // THREAD
{
	OmniTelemetry t = {};
	t.seq = _current_command.seq;
	t.state = _current_command_state;
	t.pose = _motion.pose();
	xQueueOverwrite(_telemetry, &t);		// never blocks, replaces the previous snapshot
};
