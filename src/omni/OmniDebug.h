#ifndef __OMNIDEBUG_H
#define __OMNIDEBUG_H

#include <Arduino.h>

#include "config.h"
#include "pins.h"

class OmniThread;

/*
	FOC pass statistics (THREAD_DEBUG only): overruns, longest pass, PIN_FOC_THREAD
	HIGH during each pass. Used through OMNI_PASS_BEGIN/END below, which expand to
	nothing without THREAD_DEBUG. begin()/end() run in the FOC thread; the counters
	are read from the main thread (32-bit aligned reads are atomic).
*/
#ifdef THREAD_DEBUG
class PassStats
{
	public:
		inline void begin(uint32_t notify_count)
		{
			// count > 1: the timer fired again before this pass -> missed passes, no catch-up
			if (notify_count > 1)
				overruns += notify_count - 1;
			passes++;
#ifdef PIN_FOC_THREAD
			digitalWrite(PIN_FOC_THREAD, HIGH);
#endif
			_t0 = micros();
		};

		inline void end()
		{
			uint32_t pass_us = micros() - _t0;
			busy_us += pass_us;
			if (pass_us > max_pass_us)
				max_pass_us = pass_us;
#ifdef PIN_FOC_THREAD
			digitalWrite(PIN_FOC_THREAD, LOW);
#endif
		};

		volatile uint32_t overruns = 0;			// missed passes since boot
		volatile uint32_t max_pass_us = 0;		// longest pass since last reset
		volatile uint32_t passes = 0;			// executed passes since boot (wraps; use deltas)
		volatile uint32_t busy_us = 0;			// summed pass time since boot (wraps after ~71 min; use deltas)

	private:
		uint32_t _t0 = 0;
};

#define OMNI_PASS_BEGIN(stats, count)	(stats).begin(count)
#define OMNI_PASS_END(stats)			(stats).end()
#else
#define OMNI_PASS_BEGIN(stats, count)	((void)(count))		// no code; just marks count as used
#define OMNI_PASS_END(stats)
#endif

/*
	Debug modes (see config.h). Not part of the FOC thread: sensors() and motors()
	run in the main task and never return. Only pass() runs in the FOC thread.
	OmniBase::init() first calls attach(): from then on OmniDebug knows the (single) thread.
*/
#if defined(MOTORS_DEBUG_MODE) || defined(SENSORS_DEBUG_MODE) || defined(THREAD_DEBUG)
#define OMNI_DEBUG								// any debug mode: OmniDebug exists
#endif

#ifdef OMNI_DEBUG
class OmniDebug
{
	public:
		static void attach(OmniThread& thread);		// OmniBase::init(), before anything below
#ifdef THREAD_DEBUG
		static void reportPasses();					// main task, call often: prints the pass statistics once per second
#endif
#ifdef SENSORS_DEBUG_MODE
		static void sensors();						// pre-start, drivers off: sensor dump loop
#endif // SENSORS_DEBUG_MODE
#ifdef MOTORS_DEBUG_MODE
		static void motors();						// Commander + monitoring, starts the thread, monitor loop
		static void pass();							// FOC thread, once per pass: command.run()
#endif // MOTORS_DEBUG_MODE
};
#define OMNI_DEBUG_ATTACH(x)	OmniDebug::attach(x)
#else // !OMNI_DEBUG
#define OMNI_DEBUG_ATTACH(x)
#endif // OMNI_DEBUG

// hook in the FOC loop: OmniDebug::pass() in MOTORS_DEBUG_MODE, nothing otherwise
#ifdef MOTORS_DEBUG_MODE
#define OMNI_DEBUG_PASS()	OmniDebug::pass()
#else
#define OMNI_DEBUG_PASS()
#endif

#endif // __OMNIDEBUG_H
