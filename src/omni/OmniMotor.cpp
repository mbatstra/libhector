#include "OmniMotor.h"
#include "config.h"
#include "pins.h"

// Indexed by motor id
static const struct
{
	int PWM_A;
	int PWM_B;
	int PWM_C;
	int SENSE_CS;
} PINS[] =
{
	//	PWM_A				PWM_B				PWM_C				SENSE_CS
	{	PIN_MOTOR0_PWM_A,	PIN_MOTOR0_PWM_B,	PIN_MOTOR0_PWM_C,	PIN_SENSE0_CS	},
	{	PIN_MOTOR1_PWM_A,	PIN_MOTOR1_PWM_B,	PIN_MOTOR1_PWM_C,	PIN_SENSE1_CS	},
	{	PIN_MOTOR2_PWM_A,	PIN_MOTOR2_PWM_B,	PIN_MOTOR2_PWM_C,	PIN_SENSE2_CS	},
};

OmniMotor::OmniMotor(int id)
	: sensor(PINS[id].SENSE_CS, true, SPISettings(8000000, AS5047U_BITORDER, SPI_MODE1))
	, driver(PINS[id].PWM_A, PINS[id].PWM_B, PINS[id].PWM_C)
	, motor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE, MOTOR_KV, MOTOR_PHASE_INDUCTANCE, MOTOR_PHASE_INDUCTANCE)
	, _id(id)
{
};

bool OmniMotor::init(const Calibration& cal)
{
	Serial.printf("-- Starting Motor %d\n", _id);

	initSensor();

	driver.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
	driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;
	driver.pwm_frequency = MOTOR_PWM_FREQUENCY;
	if (!driver.init())
	{
		Serial.printf("Driver %d init failed!\n", _id);
		return false;
	};

	motor.linkSensor(&sensor);
	motor.linkDriver(&driver);

	// torque via voltage (no current sensing on this build)
	motor.torque_controller = TorqueControlType::estimated_current;
	motor.controller = MotionControlType::angle;		// target = angle, feed_forward_velocity = speed (computed by OmniMotion, written by OmniThread::updateTargets())
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
	motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
	motor.velocity_limit = MOTOR_VELOCITY_LIMIT;

	// velocity PID + low-pass filter - tune for your motor
	motor.PID_velocity.P = MOTOR_PID_P;
	motor.PID_velocity.I = MOTOR_PID_I;
	motor.PID_velocity.D = MOTOR_PID_D;
	motor.LPF_velocity.Tf = MOTOR_PID_LPF;
	motor.PID_velocity.limit = MOTOR_PID_LIMIT;
	motor.PID_velocity.output_ramp = MOTOR_PID_RAMPRATE;

	// angle loop: P only, corrects the position error on top of the feed-forward velocity
	motor.P_angle.P = MOTOR_ANGLE_P;
	motor.P_angle.I = 0.0f;
	motor.P_angle.D = 0.0f;
	motor.P_angle.output_ramp = 0.0f;
	motor.P_angle.limit = MOTOR_ANGLE_LIMIT;

	// move() every FOC_MOTION_DOWNSAMPLE FOC passes (SimpleFOC runs it every N+1 calls)
	motor.motion_downsample = FOC_MOTION_DOWNSAMPLE - 1;
	motor.LPF_current_q.Tf = FOC_CURRENT_LPF_TF;

	if (!motor.init())
	{
		Serial.printf("Motor %d init failed!\n", _id);
		return false;
	};

	// known values: initFOC() skips the direction search / zero-angle alignment
	motor.sensor_direction = (Direction)cal.direction;
	motor.zero_electric_angle = cal.zero_angle;

	if (!motor.initFOC())
	{
		Serial.printf("Motor %d FOC alignment failed!\n", _id);
		return false;
	};

	motor.target = 0.0f;
	motor.disable();
	return true;
};

OmniMotor::Calibration OmniMotor::calibration() const
{
	return { (int8_t)motor.sensor_direction, motor.zero_electric_angle };
};

// not init() again: driver.init() would allocate the PWM channels a second time
bool OmniMotor::calibrate()
{
	Serial.printf("-- Calibrating Motor %d\n", _id);

	motor.enable();									// initFOC() drives the phases
	motor.sensor_direction = Direction::UNKNOWN;	// forget the old values: initFOC() aligns
	motor.zero_electric_angle = NOT_SET;

	bool ok = motor.initFOC();
	if (!ok)
		Serial.printf("Motor %d FOC alignment failed!\n", _id);

	motor.target = 0.0f;
	motor.disable();
	return ok;
};

void OmniMotor::initSensor()
{
	sensor.init(&OMNIBASE_SPI);
	sensor.clearErrorFlag();
};

void OmniMotor::loop()
{
	motor.loopFOC();
	motor.move();
};

void OmniMotor::debugSensor()
{
	uint16_t raw = sensor.readRawAngle();
	uint16_t mag = sensor.readMagnitude();
	float angle = sensor.getCurrentAngle() * 360 / (2*PI);
	bool err = sensor.isErrorFlag();
	bool warn = sensor.isWarningFlag();
	Serial.printf("Sensor %d: raw=0x%04X angle=%.1f mag=%u err=%d warn=%d ", _id, raw, angle, mag, err, warn);

	// decode + clear the error register so we see which bit is actually set
	if(err)
	{
		AS5047UError error = sensor.clearErrorFlag();
		if(error.cordic_ovf)		Serial.print("CORDIC_OVF ");
		if(error.off_notfinished)	Serial.print("OFF_NOTFINISHED ");
		if(error.wdtst)				Serial.print("WDTST ");
		if(error.crc_error)			Serial.print("CRC_ERROR ");
		if(error.cmd_error)			Serial.print("CMD_ERROR ");
		if(error.frame_error)		Serial.print("FRAME_ERROR ");
		if(error.p2ram_error)		Serial.print("P2RAM_ERROR ");
		if(error.p2ram_warning)		Serial.print("P2RAM_WARNING ");
		if(error.maghalf)			Serial.print("MAGHALF ");
		if(error.agc_warning)		Serial.print("AGC_WARNING ");

		// datasheet: "In case of an error flag, a read of the DIA register is mandatory"
		AS5047UDiagnostics diag = sensor.readDiagnostics();
		if(!diag.agc_finished)		Serial.print("AGC_NOT_FINISHED ");
		if(!diag.off_finished)		Serial.print("OFF_NOT_FINISHED ");
		if(!diag.sin_finished)		Serial.print("SIN_NOT_FINISHED ");
		if(!diag.cos_finished)		Serial.print("COS_NOT_FINISHED ");
		if(diag.maghalf_flag)		Serial.print("MAGHALF_FLAG ");
		if(diag.comp_h)				Serial.print("COMP_H ");
		if(diag.comp_l)				Serial.print("COMP_L ");
		if(diag.cordic_ovf)			Serial.print("CORDIC_OVF ");
		if(!diag.loops_finished)	Serial.print("LOOPS_NOT_FINISHED ");
		if(diag.vdd_mode)			Serial.print("VDD5.0V ");
		else 						Serial.print("VDD3.3V ");
	};
	Serial.println();
};
