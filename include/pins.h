#ifndef __PINS_H__
#define __PINS_H__

// --- General -------------------------------------------------------------
#define PIN_MOTOR_nSLEEP     GPIO_NUM_1
#define PIN_MOTOR_nFAULT     GPIO_NUM_2
#define PIN_MOTOR_ENABLE     GPIO_NUM_18
//ToBeUsed: #define PIN_CHARGER_nINT    GPIO_NUM_42
//ToBeUsed: #define PIN_LED_DATA        GPIO_NUM_46
//ToBeUsed: #define PIN_HECTOR_GO       GPIO_NUM_47

// #define PIN_USB_D_N         GPIO_NUM_19      // USB: Keep your digits home
// #define PIN_USB_D_P         GPIO_NUM_20      // USB: Keep your digits home
// #define PIN_GPIO3_FREE      GPIO_NUM_3
// #define PIN_GPIO35_PSRAM GPIO_NUM35    // PSRAM: Keep your digits home
// #define PIN_GPIO36_PSRAM GPIO_NUM36    // PSRAM: Keep your digits home
// #define PIN_GPIO37_PSRAM GPIO_NUM37    // PSRAM: Keep your digits home
// #define PIN_GPIO43_FREE     GPIO_NUM_43
#define PIN_FOC_THREAD      GPIO_NUM_43         // Thread pace monitor, undef to remove code
// #define PIN_GPIO44_FREE     GPIO_NUM_44
#define PIN_CALIBRATE       GPIO_NUM_44         // Hold LOW at boot to recalibrate
// #define PIN_GPIO45_FREE     GPIO_NUM_45
// #define PIN_GPIO48_FREE     GPIO_NUM_48

// --- Buses ---------------------------------------------------------------
// SPI for SENSE0..2
#define PIN_SENSE_CLK           GPIO_NUM_5
#define PIN_SENSE_MISO          GPIO_NUM_6
#define PIN_SENSE_MOSI          GPIO_NUM_7

#define PIN_CHARGER_OLED_SCL    GPIO_NUM_38
#define PIN_CHARGER_OLED_SDA    GPIO_NUM_39
    #define I2C_ADDRESS_CHARGER     0x6B
    #define I2C_ADDRESS_OLED        0x3C

#define PIN_SENSORS_SCL         GPIO_NUM_40
#define PIN_SENSORS_SDA         GPIO_NUM_41

// --- Motor 0 -------------------------------------------------------------
#define PIN_MOTOR0_PWM_A        GPIO_NUM_10
#define PIN_MOTOR0_PWM_B        GPIO_NUM_9
#define PIN_MOTOR0_PWM_C        GPIO_NUM_8

#define PIN_SENSE0_CS           GPIO_NUM_11

// --- Motor 1 -------------------------------------------------------------
#define PIN_MOTOR1_PWM_A        GPIO_NUM_17
#define PIN_MOTOR1_PWM_B        GPIO_NUM_16
#define PIN_MOTOR1_PWM_C        GPIO_NUM_15

#define PIN_SENSE1_CS           GPIO_NUM_4

// --- Motor 2 -------------------------------------------------------------
#define PIN_MOTOR2_PWM_A        GPIO_NUM_21
#define PIN_MOTOR2_PWM_B        GPIO_NUM_14
#define PIN_MOTOR2_PWM_C        GPIO_NUM_13

#define PIN_SENSE2_CS           GPIO_NUM_12

#endif // __PINS_H__
