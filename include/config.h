#ifndef __CONFIG_H
#define __CONFIG_H

// --- Build modes and Debug flags (all off for speedy-Gonzales ------------
// #define MOTORS_DEBUG_MODE                   // Commander (A/B/C) + monitoring for PID tuning; init() does not return
// #define SENSORS_DEBUG_MODE                     // Only init sensors, drivers depowered, dump sensor debug; init() does not return
// #define THREAD_DEBUG                           // Thread cycle time checking and monitoring

#define LOG_DEVICE         Serial              // Where to print debug messages

// --- Motor / power stage parameters --------------------------------------
/*
    Motor: SMC PM2205 (http://shop.smc-powers.com/PM2205.html)
    Nom. Voltage        12      V
    Nom. Current        0.66    A
    Nom. Torque         0.04    Nm
    Nom. Speed          792     rpm (~13 rot/sec = 82 rad/sec)
    Max. Speed          1200    rpm (~20 rot/sec = 125 rad/sec)
    Stall Torque        0.06    Nm
    Stall Current       1.00    A
    Resistance          7.1     Ohms
    Inductance          1.45    mH
    Speed Const         100     rpm/V       (Stated, TW measured more, 480 something?)
    Torque Const        0.06    Nm/A
    Rotor Internia      42      gcm^2
    Pole Pairs          7      
    Motor Weight        20      g
    Working Temp        -20/80  C
    Demagnet [Temp]     120     C
    Encoder Type        AS5048A/AS5600 (When bought, internally. We use:)
    Encoder USED(!)     AS5047U
*/ 
#define MOTOR_POLE_PAIRS            7                   // T-DoubleU told me so, datasheet didn't call him bluff.
#define MOTOR_PHASE_RESISTANCE      (7.1f / 2.0f)       // From datasheet (0 disables current limiting [Ohms], phase res is half the measured resistance
#define MOTOR_PHASE_INDUCTANCE      (0.00145f / 2.0f)   // From datasheet (0 sets Ud estimate to zero) [H]
#define MOTOR_KV                    100 //470           // TODO: Hard guess by soft-skills, in lack of a random number generator
#define MOTOR_SUPPLY_VOLTAGE        5.0f                // Volts, Volts, it must be Volts.
#define MOTOR_VOLTAGE_LIMIT         5.0f                // V, keep low until the setup is verified. So max it is.
#define MOTOR_VELOCITY_LIMIT        30.0f	            // rad/s -> 82 = 10 rot/sec, but 30 is about the max for 5V
#define MOTOR_PWM_FREQUENCY         22000               // Right above Mels' annoyance level. Factually checked. [Hz]

#define MOTOR_PID_P                 0.1f            //
#define MOTOR_PID_I                 2.0f            //
#define MOTOR_PID_D                 0.0f            //
#define MOTOR_PID_LPF               0.01f           // Velocity measurement low-pass rate
#define MOTOR_PID_LIMIT             1.0f            // Limit PID output rate (current), limits wind-up
#define MOTOR_PID_RAMPRATE          1000.0f         // rad/s/s? <185 and it starts to oscillate. Higher numbers don't seeem much of problem

#define MOTOR_ANGLE_P               20.0f           // P_angle [rad/s per rad]: position error -> extra velocity (library default, to be tuned)
#define MOTOR_ANGLE_LIMIT           10.0f           // max correction velocity from P_angle [rad/s], on top of the feed-forward

// --- Wheel geometry (robot frame: x forward, y left, angles CCW) ------------
// Symmetric frame: wheel i sits at alpha_i = WHEEL0_ANGLE + WHEEL_NUMBERING * 120 * i around the centre
// and rolls along alpha_i + 90 + WHEEL_YAW. Defaults (0, +1, yaw 0) give positions 0/120/240 and match
// the rolling directions the hexagon test assumes (90/210/330).
#define WHEEL_RADIUS                0.019f          // [m] (38 mm wheels)
#define WHEEL_DISTANCE              0.031f          // L [m], centre to wheel contact point. TODO measure
#define WHEEL0_ANGLE                0.0f            // [deg] position of wheel 0 around the centre (alpha_0)
#define WHEEL_NUMBERING             1.0f            // +1: wheels 0 -> 1 -> 2 counter-clockwise, -1: clockwise
#define WHEEL_YAW                   40.15f          // [deg] wheels turned around Z; 0 = axles through the centre. TODO measure
#define WHEEL_DIRECTION             1.0f            // +1 / -1: positive motor angle = rolling along the rolling direction
#define WHEEL_SPEED_MAX             (MOTOR_VELOCITY_LIMIT - MOTOR_ANGLE_LIMIT)  // [rad/s] max feed-forward: headroom for the P_angle correction
#define WHEEL_ACCEL_MAX             50.0f           // [rad/s^2] ramp rate of the busiest wheel. Placeholder, to be measured
#define MOVE_ACCEL_XY               0.5f            // [m/s^2] MOVE: translation acceleration. Placeholder, to be measured
#define MOVE_ACCEL_R                4.0f            // [rad/s^2] MOVE: rotation acceleration. Placeholder, to be measured

#define OMNIBASE_SPI                SPI                 // The (Arduino) SPI interface to use, the sensor bus.

#define PIN_CALIBRATE_ENABLED       (1)                 // Enable (1) to enable pin-high-to-force-calibration logic (define PIN_CALIBRATE as well in pins.h)

// --- FOC thread ---------------------------------------------------------------
#define FOC_MOTION_DOWNSAMPLE       1                   // move() every Nth FOC pass (1 = every pass)
// With downsampling the current setpoint (velocity PID output) is a staircase with steps of
// N/f_FOC; filtering it with Tf ~ 0.5..1 step smooths it into a near-ramp at a comparable lag.
#define FOC_CURRENT_LPF_TF          (0.75f * FOC_MOTION_DOWNSAMPLE / FOC_LOOP_FREQUENCY)   // [s]

#define FOC_LOOP_FREQUENCY          5000                // Fixed FOC pass rate [Hz]; lower it if overruns show up
#define FOC_TASK_CORE               1                   // core the FOC task is pinned to (WiFi/BT live on core 0)
#define FOC_TASK_PRIORITY           20                  // well above loopTask (1)
#define FOC_TASK_STACK              4096                // [bytes]
#define FOC_COMMAND_QUEUE_LENGTH    16                  // commands that can wait in the queue
#define FOC_COMMAND_DOWNSAMPLE      5                   // process commands every Nth FOC pass (5 -> 1 kHz)

#if defined(MOTORS_DEBUG_MODE) && defined(SENSORS_DEBUG_MODE)
#error "MOTORS_DEBUG_MODE and SENSORS_DEBUG_MODE are mutually exclusive"
#endif

#ifdef MOTORS_DEBUG_MODE
    #define THREAD_DEBUG
#endif

#endif // __CONFIG_H
