#include "easy_main.h"

#include "common_includes.h"
#include "../dRobot/general_msgs/geometry_msgs.h"
#include "../dRobot/general_odrs/numeric_odrs.h"

#include "../dRobot/Keyboard.h"
#include "../dRobot/Kinematics.h"
#include "../dRobot/LED.h"
#include "../dRobot/Odometer.h"
#include "../dRobot/DeadReckoning.h"
#include "../dRobot/PathGuider.h"
#include "../dRobot/PIDController.h"
#include "../dRobot/PosePoint.h"
#include "../dRobot/MotorDriver.h"
#include "../dRObot/WheelObserver.h"

/* External global variables */

// Timer for Ticker interrupt
extern TIM_HandleTypeDef htim6;

// TImer for PWM generation
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

// Timer counters for encoder
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// UART connection for PC serial
extern UART_HandleTypeDef huart3;


/* All devices */

// Input device
dRobot::Keyboard keyboard(&huart3);

// Point device
dRobot::PosePoint wheel_point1;
dRobot::PosePoint wheel_point2;
dRobot::PosePoint wheel_point3;
dRobot::PosePoint wheel_point4;

// Sensor device
dRobot::Odometer odometer_r(TIM2, 4800, 0.030);
dRobot::Odometer odometer_f(TIM3, 4800, 0.030);
dRobot::Odometer odometer_l(TIM4, 4800, 0.030);

// System device
dRobot::Kinematics kinematics;
dRobot::DeadReckoning dead_reckoning;
dRobot::PIDController pid_controller1(1.0, 0.001, 0.00001);
dRobot::PIDController pid_controller2(1.0, 0.001, 0.00001);
dRobot::PIDController pid_controller3(1.0, 0.001, 0.00001);
dRobot::PIDController pid_controller4(1.0, 0.001, 0.00001);
dRobot::WheelObserver wheel_observer;
dRobot::PathGuider path_guider;

// Output device
dRobot::LED led(GPIOB,GPIO_PIN_14);
dRobot::MotorDriver motor_driver1(&(TIM1->CCR1), &(TIM1->CCR2), 0, 1000, 0.75);
dRobot::MotorDriver motor_driver2(&(TIM8->CCR1), &(TIM8->CCR2), 0, 1000, 0.75);
dRobot::MotorDriver motor_driver3(&(TIM8->CCR3), &(TIM8->CCR4), 0, 1000, 0.75);
dRobot::MotorDriver motor_driver4(&(TIM1->CCR3), &(TIM1->CCR4), 0, 1000, 0.75);


/* Setup */

void easy_setup(){
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

	ticker_init(&htim6, 16, 'm');

	// setup wheel_point
	wheel_point1.setPosition(0.29915, 0.29915, 0.0);
	wheel_point1.setAngle_z(3.0*M_PI/4.0);

	wheel_point2.setPosition(-0.29915, 0.29915, 0.0);
	wheel_point2.setAngle_z(-3.0*M_PI/4.0);

	wheel_point3.setPosition(-0.29915, -0.29915, 0.0);
	wheel_point3.setAngle_z(-M_PI/4.0);

	wheel_point4.setPosition(0.29915, -0.29915, 0.0);
	wheel_point4.setAngle_z(M_PI/4.0);

	// setup odometer_r
	odometer_r.setPosition(0.0, -0.28837, 0.0);
	odometer_r.setAngle_z(M_PI);
	odometer_r.setup();

	// setup odometer_l
	odometer_f.setPosition(0.28837, 0.0, 0.0);
	odometer_f.setAngle_z(-M_PI/2.0);
	odometer_f.setup();

	// setup odometer_r
	odometer_l.setPosition(0.0, 0.28837, 0.0);
	odometer_l.setAngle_z(0.0);
	odometer_l.setup();

	// setup kinematics
	odometer_r >> &kinematics.wheel[0];
	odometer_f >> &kinematics.wheel[1];
	odometer_l >> &kinematics.wheel[2];
	kinematics.setup(0.020, 1);

	// setup dead_reckoning
	kinematics >> &dead_reckoning.kinematics;
	dead_reckoning.setup(0.020, 2);

	dead_reckoning >> &wheel_observer.positioning;

	// setup pid_controller
	motor_driver1 >> &pid_controller1.output;
	pid_controller1.setup();

	motor_driver2 >> &pid_controller2.output;
	pid_controller2.setup();

	motor_driver3 >> &pid_controller3.output;
	pid_controller3.setup();

	motor_driver4 >> &pid_controller4.output;
	pid_controller4.setup();

	// setup wheel_observer
	kinematics >> &wheel_observer.obs;
	pid_controller1 >> &wheel_observer.err[0];
	pid_controller2 >> &wheel_observer.err[1];
	pid_controller3 >> &wheel_observer.err[2];
	pid_controller4 >> &wheel_observer.err[3];
	wheel_point1 >> &wheel_observer.wheel[0];
	wheel_point2 >> &wheel_observer.wheel[1];
	wheel_point3 >> &wheel_observer.wheel[2];
	wheel_point4 >> &wheel_observer.wheel[3];
	wheel_observer.setup(0.020, 2);

	// setup keyboard
	led >> &keyboard.toggle_dev[1];
	//wheel_observer >> &keyboard.twist_dev;
	keyboard.setup("KeyI");

	// setup path_guider
	dead_reckoning >> &path_guider.positioner;
	wheel_observer >> &path_guider.controller;

	dRobot::float_array_odr path_odr;
	float arr[12];

	arr[0] = -2;
	arr[1] = 3;
	arr[2] = 0;
	arr[3] = 0;

	arr[4] = 2;
	arr[5] = -3;
	arr[6] = 3;
	arr[7] = 0;

	arr[8] = 0.0;
	arr[9] = 0.0;
	arr[10] = 3.14;
	arr[11] = 0;

	path_odr.arr = arr;
	path_odr.size = 12;

	path_guider.shareOdr(path_odr);

	path_guider.setup(0.020, 3);

	ticker_schedule();
	ticker_start();
}


void easy_loop(){
	float_msg msg = path_guider.shareMsg();

	if (msg.val > 1.0){

	}
}

#if 0
#include "../dRobot2/general_msgs/geometry_msgs.h"

#include "../dRobot2/DeadReckoning.h"
#include "../dRobot2/InvKinematics.h"
#include "../dRobot2/KeyboardWASD.h"
#include "../dRobot2/Kinematics.h"
#include "../dRobot2/LED.h"
#include "../dRobot2/Observer.h"
#include "../dRobot2/Odometer.h"
#include "../dRobot2/OmniChassisRobot.h"
#include "../dRobot2/PIDController.h"
#include "../dRobot2/PosePoint.h"
#include "../dRobot2/MotorDriver.h"

/* External global variables */

// Timer for Ticker interrupt
extern TIM_HandleTypeDef htim6;

// TImer for PWM generation
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

// Timer counters for encoder
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// UART connection for PC serial
extern UART_HandleTypeDef huart3;


/* All devices */

// Input device
dRobot2::KeyboardWASD keypad(&huart3);

// Point device
dRobot2::PosePoint wheel_point1;
dRobot2::PosePoint wheel_point2;
dRobot2::PosePoint wheel_point3;
dRobot2::PosePoint wheel_point4;

// Sensor device
dRobot2::Odometer odometer_r(TIM2, 4800, 0.030);
dRobot2::Odometer odometer_f(TIM3, 4800, 0.030);
dRobot2::Odometer odometer_l(TIM4, 4800, 0.030);

// System device
dRobot2::InvKinematics inv_kinematics_w1_i;
dRobot2::InvKinematics inv_kinematics_w1_o;
dRobot2::InvKinematics inv_kinematics_w2_i;
dRobot2::InvKinematics inv_kinematics_w2_o;
dRobot2::InvKinematics inv_kinematics_w3_i;
dRobot2::InvKinematics inv_kinematics_w3_o;
dRobot2::InvKinematics inv_kinematics_w4_i;
dRobot2::InvKinematics inv_kinematics_w4_o;
dRobot2::Kinematics kinematics;
dRobot2::DeadReckoning dead_reckoning;
dRobot2::Observer observer_w1;
dRobot2::Observer observer_w2;
dRobot2::Observer observer_w3;
dRobot2::Observer observer_w4;
dRobot2::OmniChassisRobot omni_chassis_robot;
dRobot2::PIDController pid_controller_w1(1.0, 0.001, 0.00001);
dRobot2::PIDController pid_controller_w2(1.0, 0.001, 0.00001);
dRobot2::PIDController pid_controller_w3(1.0, 0.001, 0.00001);
dRobot2::PIDController pid_controller_w4(1.0, 0.001, 0.00001);

// Output device
dRobot2::LED led(GPIOB,GPIO_PIN_14);
dRobot2::MotorDriver motor_driver1(&(TIM1->CCR1), &(TIM1->CCR2), 0, 1000, 0.2);
dRobot2::MotorDriver motor_driver2(&(TIM8->CCR1), &(TIM8->CCR2), 0, 1000, 0.2);
dRobot2::MotorDriver motor_driver3(&(TIM8->CCR3), &(TIM8->CCR4), 0, 1000, 0.2);
dRobot2::MotorDriver motor_driver4(&(TIM1->CCR3), &(TIM1->CCR4), 0, 1000, 0.2);

/* External devices */
//dRobot2::PathGuider path_guider;


/* Setup */

void easy_setup(){
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

	ticker_init(&htim6, 16, 'm');

	// setup wheel_point
	wheel_point1.translate(0.29915, 0.29915, 0.0);
	wheel_point1.rotate_z(3.0*M_PI/4.0);

	wheel_point2.translate(-0.29915, 0.29915, 0.0);
	wheel_point2.rotate_z(-3.0*M_PI/4.0);

	wheel_point3.translate(-0.29915, -0.29915, 0.0);
	wheel_point3.rotate_z(-M_PI/4.0);

	wheel_point4.translate(0.29915, -0.29915, 0.0);
	wheel_point4.rotate_z(M_PI/4.0);

	// setup odometer_r
	odometer_r.translate(0.0, -0.28837, 0.0);
	odometer_r.rotate_z(M_PI);

	// setup odometer_l
	odometer_f.translate(0.28837, 0.0, 0.0);
	odometer_f.rotate_z(-M_PI/2.0);

	// setup odometer_r
	odometer_l.translate(0.0, 0.28837, 0.0);
	odometer_l.rotate_z(0.0);

	// setup kinematics
	odometer_r >> &kinematics.wheel[0];
	odometer_f >> &kinematics.wheel[1];
	odometer_l >> &kinematics.wheel[2];

	// setup dead_reckoning
	kinematics >> &dead_reckoning.kinematics;

	dead_reckoning >> &omni_chassis_robot.positioning;

	// setup inv_kinematics
	omni_chassis_robot >> &inv_kinematics_w1_i.forward;
	wheel_point1       >> &inv_kinematics_w1_i.wheel;
	inv_kinematics_w1_i.addAutoReload(&pid_controller_w1);
	inv_kinematics_w1_i.setup(0.020, 2);

	omni_chassis_robot >> &inv_kinematics_w2_i.forward;
	wheel_point2       >> &inv_kinematics_w2_i.wheel;
	inv_kinematics_w2_i.addAutoReload(&pid_controller_w2);
	inv_kinematics_w2_i.setup(0.020, 2);

	omni_chassis_robot >> &inv_kinematics_w3_i.forward;
	wheel_point3       >> &inv_kinematics_w3_i.wheel;
	inv_kinematics_w3_i.addAutoReload(&pid_controller_w3);
	inv_kinematics_w3_i.setup(0.020, 2);

	omni_chassis_robot >> &inv_kinematics_w4_i.forward;
	wheel_point4       >> &inv_kinematics_w4_i.wheel;
	inv_kinematics_w4_i.addAutoReload(&pid_controller_w4);
	inv_kinematics_w4_i.setup(0.020, 2);

	kinematics   >> &inv_kinematics_w1_i.forward;
	wheel_point1 >> &inv_kinematics_w1_o.wheel;

	kinematics   >> &inv_kinematics_w2_i.forward;
	wheel_point2 >> &inv_kinematics_w2_o.wheel;

	kinematics   >> &inv_kinematics_w3_i.forward;
	wheel_point3 >> &inv_kinematics_w3_o.wheel;

	kinematics   >> &inv_kinematics_w4_i.forward;
	wheel_point4 >> &inv_kinematics_w4_o.wheel;

	// setup wheel_observer
	inv_kinematics_w1_i >> &observer_w1.target;
	inv_kinematics_w1_o >> &observer_w1.obs;

	inv_kinematics_w2_i >> &observer_w2.target;
	inv_kinematics_w2_o >> &observer_w2.obs;

	inv_kinematics_w3_i >> &observer_w3.target;
	inv_kinematics_w3_o >> &observer_w3.obs;

	inv_kinematics_w4_i >> &observer_w4.target;
	inv_kinematics_w4_o >> &observer_w4.obs;

	// setup pid_controller
	observer_w1 >> &pid_controller_w1.obs;
	observer_w2 >> &pid_controller_w1.obs;
	observer_w3 >> &pid_controller_w1.obs;
	observer_w4 >> &pid_controller_w1.obs;

	// setup motordriver
	pid_controller_w1 >> &motor_driver1.volumer;
	pid_controller_w2 >> &motor_driver2.volumer;
	pid_controller_w3 >> &motor_driver3.volumer;
	pid_controller_w4 >> &motor_driver4.volumer;

	// setup keyboard
	keypad >> &omni_chassis_robot.controller;
	omni_chassis_robot.addAutoReload(&inv_kinematics_w1_i);
	omni_chassis_robot.addAutoReload(&inv_kinematics_w2_i);
	omni_chassis_robot.addAutoReload(&inv_kinematics_w3_i);
	omni_chassis_robot.addAutoReload(&inv_kinematics_w4_i);

	/* outer setup */

	// setup path_guider


	ticker_schedule();
	ticker_start();
}


void easy_loop(){

}
#endif
