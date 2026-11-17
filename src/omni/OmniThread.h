#ifndef __OMNITHREAD_H
#define __OMNITHREAD_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <driver/gptimer.h>

#include "config.h"
#include "OmniTypes.h"
#include "OmniMotor.h"
#include "OmniMotion.h"
#include "OmniDebug.h"

/*
	Thread side of the omni base: owns the 3 motors, the FOC task and its gptimer.
	(Driver pins and SPI bus: OmniBase. Debug modes: OmniDebug.)

	Rule: everything before start() is single-threaded (runs in main's context).
	After start(), only the FOC task touches this object - except while paused (pause()/resume()),
	and OmniDebug::motors() (main thread, MOTORS_DEBUG_MODE): monitor() on the motors and the pass statistics.
*/
class OmniThread
{
	friend class OmniDebug;

	public:
		//*********************** caller= Main Thread: ****************************/
		OmniThread();

		// pre-start: FOC init of all motors (SPI started and drivers powered by OmniBase)
		// restores the calibration from NVS, or calibrates (no valid data) and stores it
		bool init();
		void start();				// create the FOC task (core 1), which starts the timer

		// removes the stored calibration: the next init() calibrates.
		// When the thread runs it is paused for the flash write (motors stay disabled)
		bool clearCalibration();

		// recalibrate all motors and store the result (all-or-nothing). Pauses the thread
		// when it runs; blocks for several seconds; motors are disabled afterwards
		bool calibrate();

		// after start(); main is the only producer. Return the command's seq
		uint32_t send(OmniCommand cmd);	// queue a command (seq is assigned here); blocks while the queue is full
		bool queueFull();				// the next send() would block
		uint32_t stop();				// empty the queue, then queue STOP (can't fail)
		OmniTelemetry telemetry();		// latest snapshot; {0, INVALID} before the first command

		// TODO: needs to move private but requires solution in debug mode
		OmniMotor motors[3];		// Accessible to OmniBase but not outside it

	private:
		//*********************** caller= Main Thread: ****************************/
		enum class State { STOPPED, RUNNING, PAUSED };
		State _state = State::STOPPED;			// caller side only
		uint32_t _seq = 0;						// last assigned command seq

		// Calibration record as stored in NVS. Bump CALIBRATION_VERSION (OmniThread.cpp) when it changes.
		struct CalibrationPref
		{
			public:
				uint16_t				version;
				OmniMotor::Calibration	motor[3];
				uint32_t				crc;		// CRC32 over everything above

				bool load();				// false: missing or invalid, record reset to uncalibrated
				bool save();				// sets version + CRC, writes to NVS
				static bool clear();		// removes the stored record
			private:
				bool read();				// false: missing, wrong version/CRC or invalid values
				uint32_t checksum() const;	// CRC32 over everything before crc
		};
	
		bool pause();				// Main: Initiate pause negotiations, returns false when it wasn't running (yet or parked)
		void resume();				// Main: Un-pause the FOC task

		bool initMotors(const CalibrationPref& cal);
		void saveCalibration(CalibrationPref& cal);	// collect what initFOC() found and store it in NVS

		//*********************** caller= FOC Task Thread: ****************************/
		static void taskEntry(void* arg);
		void run();					// the FOC task itself, never returns
		void park();				// FOC task: stop timer, disable motors, wait for resume()
		gptimer_handle_t _timer = nullptr;

		void processCommands();						// every FOC_COMMAND_DOWNSAMPLE passes: process incoming commands
		void startNextCommand(int64_t start_us);	// receive the queue head into _current_command and start it at start_us; sets _current_command_state
		void publishTelemetry();					// _current_command.seq + _current_command_state; once per command tick, from run()
		OmniCommand _current_command = {};			// last started command (received straight into this)
		OmniCommandState _current_command_state = OmniCommandState::IDLE;
		int64_t _command_end_us = 0;				// when the current command ends; INT64_MAX = not known yet (WAIT: motion not there yet).
													// A chained next command starts here

		// Wheel targets: computed by _motion, written into the motors here
		void updateTargets();						// every pass, before the motor loop: writes target + feed_forward_velocity
		void measuredAngles(float angle[3]);		// shaft_angle of the motors [rad]: updated by move(), also while disabled
		OmniMotion _motion;

		static bool onTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t* event, void* ctx);	// ISR: Technically not FOC Thread, but on that core

		//*********************** Shared structures: ****************************/
		volatile bool _pause_request = false;	// caller -> FOC task
		SemaphoreHandle_t _parked = nullptr;	// FOC task -> pause(): parked, no pass running
		SemaphoreHandle_t _resume = nullptr;	// resume() -> FOC task
		QueueHandle_t _commands = nullptr;		// main -> FOC task (OmniCommand)
		QueueHandle_t _telemetry = nullptr;		// FOC task -> main (OmniTelemetry), length 1: mailbox
		TaskHandle_t _task = nullptr;
#ifdef THREAD_DEBUG
		PassStats _stats;
#endif

	public: // Disable copying of these objects, they control hardware. Public for better error messages
		OmniThread(const OmniThread&) = delete;
		OmniThread& operator=(const OmniThread&) = delete;
};

#endif // __OMNITHREAD_H
