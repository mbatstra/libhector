#include "OmniDebug.h"

#include "OmniThread.h"
#include "log.h"

#ifdef OMNI_DEBUG
// The (single) OmniThread, set by attach(). Also reached from the Commander callbacks (plain function pointers)
static OmniThread* instance = nullptr;

void OmniDebug::attach(OmniThread& thread)
{
	instance = &thread;
};
#endif

#ifdef MOTORS_DEBUG_MODE
static Commander command(Serial);
static void onMotor0(char* cmd) { command.motor(&instance->motors[0].motor, cmd); }
static void onMotor1(char* cmd) { command.motor(&instance->motors[1].motor, cmd); }
static void onMotor2(char* cmd) { command.motor(&instance->motors[2].motor, cmd); }

// 'X' and 'K' run in the FOC thread: only flag them, the work (which pauses the thread) happens in the monitor loop (main thread)
static volatile bool clear_calibration = false;
static void onClearCalibration(char*) { clear_calibration = true; }
static volatile bool run_calibration = false;
static void onCalibrate(char*) { run_calibration = true; }
#endif

#ifdef SENSORS_DEBUG_MODE
void OmniDebug::sensors()
{
	OmniThread& thread = *instance;

	for (OmniMotor& m : thread.motors)
		m.initSensor();

	INFO("SENSORS_DEBUG_MODE: motors unpowered, sensor dump every 500 ms");
	while (true)
	{
		for (OmniMotor& m : thread.motors)
			m.debugSensor();
		delay(500);
	};
};
#endif // SENSORS_DEBUG_MODE

#ifdef MOTORS_DEBUG_MODE
void OmniDebug::motors()
{
	OmniThread& thread = *instance;

	// setup before start(): the thread runs command.run() from then on
	for (int i = 0; i < 3; i++)
	{
		BLDCMotor& motor = thread.motors[i].motor;
		motor.useMonitoring(Serial);
		motor.monitor_downsample = 0;	// 0 = off; webcontroller needs >0 (AMD100)

		// webcontroller expects lines framed by the commander id on both ends
		motor.monitor_start_char = "ABC"[i];
		motor.monitor_end_char = "ABC"[i];
	};

	command.add('A', onMotor0, (char*)"MOTOR0");
	command.add('B', onMotor1, (char*)"MOTOR1");
	command.add('C', onMotor2, (char*)"MOTOR2");
	command.add('X', onClearCalibration, (char*)"clear calibration");
	command.add('K', onCalibrate, (char*)"calibrate");

	thread.start();

	INFO("HecTwor ready. Commands: A/B/C -> motor 1/2/3 (e.g. 'A5' = motor 1 target 5 rad/s), X -> clear calibration, K -> calibrate");

	while (true)
	{
		// monitor() only reads motor fields
		for (OmniMotor& m : thread.motors)
			m.motor.monitor();

		if (clear_calibration)
		{
			clear_calibration = false;
			if (thread.clearCalibration())
				INFO("calibration cleared, reset to recalibrate");
			else
				ERROR("clearing calibration failed");
		};

		if (run_calibration)
		{
			run_calibration = false;
			if (thread.calibrate())
				INFO("calibration done and saved, motors disabled");
			else
				ERROR("calibration failed");
		};

		reportPasses();								// once per second

		vTaskDelay(1);
	};
};

void OmniDebug::pass()
{
	command.run();
};
#endif // MOTORS_DEBUG_MODE

#ifdef THREAD_DEBUG
void OmniDebug::reportPasses()
{
	if (instance == nullptr)
		return;									// attach() not called yet

	PassStats& stats = instance->_stats;

	// state of the previous report; the first call starts the first 1 s window
	static uint32_t last_ms = millis();
	static uint32_t last_passes = stats.passes;
	static uint32_t last_busy_us = stats.busy_us;
	static uint32_t last_overruns = stats.overruns;

	uint32_t now_ms = millis();
	uint32_t elapsed_ms = now_ms - last_ms;		// unsigned: wrap-safe
	if (elapsed_ms < 1000)
		return;

	// one read per counter, the thread keeps counting (skew between them: about one pass)
	uint32_t passes = stats.passes;
	uint32_t busy_us = stats.busy_us;
	uint32_t overruns = stats.overruns;
	uint32_t max_pass_us = stats.max_pass_us;
	stats.max_pass_us = 0;						// benign race: at worst one max value is lost

	// rates over the window, measured with the main task's clock (not the gptimer)
	const uint32_t budget_us = 1000000 / FOC_LOOP_FREQUENCY;
	uint32_t rate = (uint64_t)(passes - last_passes) * 1000 / elapsed_ms;
	uint32_t load_pct = (uint64_t)(busy_us - last_busy_us) * 100 / ((uint64_t)elapsed_ms * 1000);
	uint32_t max_pct = max_pass_us * 100 / budget_us;
	uint32_t missed = overruns - last_overruns;

	Serial.printf("FOC: %lu passes/s (%d), avg. load %lu%%, max pass %lu/%lu us (=%lu%%), overruns +%lu (total %lu)\n",
		(unsigned long)rate, FOC_LOOP_FREQUENCY, (unsigned long)load_pct,
		(unsigned long)max_pass_us, (unsigned long)budget_us, (unsigned long)max_pct,
		(unsigned long)missed, (unsigned long)overruns);
	if (missed > 0)
		WARNING("FOC: %lu passes missed", (unsigned long)missed);

	last_ms = now_ms;
	last_passes = passes;
	last_busy_us = busy_us;
	last_overruns = overruns;
};
#endif // THREAD_DEBUG

