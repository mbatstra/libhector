#ifndef __OMNIMOTOR_H
#define __OMNIMOTOR_H

#include <SPI.h>
#include <SimpleFOC.h>

#include "MagneticSensorAS5047UVel.h"

/*
	One omni wheel: AS5047U sensor + 3PWM driver + BLDC motor.
	Shared hardware (SPI bus, nSLEEP/ENABLE/nFAULT) is NOT handled here.
	The constructor does not touch hardware; call init() from setup().
*/
class OmniMotor
{
	public:
		// What initFOC() measures for this motor. The defaults mean "uncalibrated":
		// with UNKNOWN and NOT_SET, initFOC() runs the alignment.
		struct Calibration
		{
			int8_t	direction = Direction::UNKNOWN;	// SimpleFOC Direction: CW = 1, CCW = -1
			float	zero_angle = NOT_SET;			// zero_electric_angle [rad]
		};

		OmniMotor(int id);			// id indexes PINS[] in OmniMotor.cpp

		// SPI bus must already be started. Calibrated cal: restored, uncalibrated: initFOC() aligns
		bool init(const Calibration& cal);
		void initSensor();			// sensor only, part of init()
		Calibration calibration() const;	// what initFOC() found or restored, for saving
		bool calibrate();			// after init(), not while loop() runs: forget the calibration and align again; disabled afterwards
		void loop();				// loopFOC + move

		// declaration order matters: motor links to sensor + driver
		MagneticSensorAS5047UVel 	sensor;
		BLDCDriver3PWM 				driver;
		BLDCMotor 					motor;
		void debugSensor();		// raw SPI / error register dump

	private:
		const int _id;

	public: // Disable copying of these objects, they control hardware. Public for better error messages
		OmniMotor(const OmniMotor&) = delete;
		OmniMotor& operator=(const OmniMotor&) = delete;
};

#endif // __OMNIMOTOR_H
