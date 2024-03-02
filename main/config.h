/*
 * config.h
 *
 *  Created on: 26-Sep-2023
 *      Author: aniru
 */

#ifndef MAIN_CONFIG_H_
#define MAIN_CONFIG_H_

//#include "config.h"
//#include "AHRS.h"
//#include "Flight_Control.h"
//#include "RF.h"

/*Freame Type*/
//#define SINGLE_AXIS_RIG //single axis PID trainer
//#define QUADX //Quadcopter X configuration
#define ROLL_CTRL_DART

/*Constant Defines*/
#define RAD_TO_DEG 57.29577951308
#define PI 3.14159265359

/*Task Defines*/
//Task Priorities
#define AHRS_task_priority 3
#define flight_control_task_priority 3
#define RF_task_priority 2
#define debug_task_priority 1

//Task rates (Hz) Available rates: 1000, 500, 333.33, 250, 200, 100, 50
#define AHRS_task_rate 400
#define flight_control_task_rate 200
#define RF_task_rate 50
#define debug_task_rate 1

/*GPIO defines */
#define M1_GPIO 1//32
#define M2_GPIO 2//12
#define M3_GPIO 3//16
#define M4_GPIO 4
#define ESP_FC_SCL_PIN 19
#define ESP_FC_SDA_PIN 21

/*Peripheral Sensor Defines*/
#define IMU_MPU6050

#define IMU_ICM20948

//#define MAG_QMC5883L

#define BARO_BMP180

//Magnetometer calibration on boot
//#define CALIB_MAG_BOOOT

/*Sensor Fusion Defines*/
#define COMPLIMENTARY_FILTER
#define COMP_GYRO_WT 0.98
#define COMP_ACCEL_WT 0.02
//#define FLTR_GYRO

/*Filter Coefficients */
#define GYROX_LPF_a 500.0 //check units
#define GYROY_LPF_a 500.0 //check units
#define GYROZ_LPF_a 500.0 //check units
#define PIDD_LPF_a 100.0

/*I2C Defines */
#define ESP_FC_I2C_PORT 0
#define ESP_FC_I2C_CLK_FREQ 400000
#define ESP_FC_I2C_TX_BUF_LEN 1024
#define ESP_FC_I2C_RX_BUF_LEN 1024

/*PID Constants*/
//Proportional angle controller constants
#define ROLL_ANGLE_P 0.8
#define PITCH_ANGLE_P 0.8
#define YAW_ANGLE_P 0.8
#define ALT_ANGLE_P 0.8

//PID rate controller constants
#define ROLL_KP 1.0
#define ROLL_KI 0
#define ROLL_KD 0
#define PITCH_KP 1.0
#define PITCH_KI 0
#define PITCH_KD 0
#define YAW_KP 1.0
#define YAW_KI 0
#define YAW_KD 0
#define ALT_KP 1.0
#define ALT_KI 0
#define ALT_KD 0

//PID Constants
#define PID_MAX 50//don't change
#define I_MAX PID_MAX/2
#define ACT_DEF_SPD 50//(%)

/*RF Type*/
//WIFI_defines
#define RF_WIFI 1
#define UDP_PORT 5832
#define udp_task_priority 2

//#define RF_BT
//#define RF_BLE
//#define RF_LORA

/*PWM Driver Defines*/
#define PWM_TYPE 2 //LEDCPWM = 1, MCPWM = 2, DSHOT = 3
#define PWM_FREQ 50000 //Hz

#endif /* MAIN_CONFIG_H_ */
