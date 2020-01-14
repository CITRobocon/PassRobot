
#ifndef WHEELOBSERVER_H_
#define WHEELOBSERVER_H_

/*
 *   Name: WheelObserver
 *   About: A PID-controller
 *
 *   Upward output: None
 *   Upward input:	Target value
 *
 *   Downward output: Input feedback
 *   Downward input:  Feedback value observing
 *
 *                ___________________
 *     float --> |                   | --> WheelObserver/empty
 *               |  WheelObserver  |
 *     float <-- |___________________| <-- WheelObserver/float
 *
 */


#include <cstdint>
#include "common_includes.h"

#include "device.h"
#include "PIDController.h"
#include "general_msgs/empty_msgs.h"
#include "general_msgs/numeric_msgs.h"
#include "general_msgs/geometry_msgs.h"
#include "general_odrs/empty_odrs.h"
#include "general_odrs/numeric_odrs.h"
#include "general_odrs/geometry_odrs.h"

namespace dRobot {


/* Message structure */

// common_msgs/empty_msgs/empty_msg


/* Order structure */

// common_odrs/numeric_odrs/float_odr


/* device class */

class WheelObserver: public device<empty_msg, twist_odr> {
public:
	/* Pointers of child device */
	device<pose2d_msg, pose2d_odr> *positioning;
	device<twist_msg, empty_odr> *obs;
	device<empty_msg, float_odr> *err[4];
	device<empty_msg, empty_odr> *wheel[4];

protected:
	/* Override: Called back before share a message */
	void msgCallback(empty_msg msg){
	}

	/* Override: Called back after be shared an order */
	void odrCallback(twist_odr odr){
		for (int i = 0; i < 4; i++){
			pose2d_msg msg;

			if (wheel[i] != nullptr){
				msg = positioning->shareMsg();

				if (odr.vx == 0.0 && odr.vy == 0.0 && odr.omega == 0.0){
					target[i] = 0.0;
				}
				else{
					float vx = odr.vx * cos(msg.theta) + odr.vy * sin(msg.theta);
					float vy = -odr.vx * sin(msg.theta) + odr.vy * cos(msg.theta);

					target[i] = calcInvKinematics(wheel[i]->getRelativePose(), vx, vy, odr.omega);
				}
			}
		}
	}

	/* Override: Update myself */
	void selfTickerUpdate(ticker_args targs){
		//float period = static_cast<float>(targs.period_ms) * 0.0010;
		const float alpha = 0.90;

		twist_msg msg = obs->shareMsg();
		float_odr odr;

		//const float thresh = 0.50;

		for (int i = 0; i < 4; i++){
			float observed_vel;

			if (wheel[i] != nullptr && err[i] != nullptr){
				if(target[i] == 0.0){
					odr.val = 0.0;
				}
				else{
					observed_vel = calcInvKinematics(wheel[i]->getRelativePose(), msg.vx, msg.vy, msg.omega);
					odr.val = target[i] - (alpha * (prev_observed[i]) + (1.0 - alpha) * (observed_vel));
					prev_observed[i] = observed_vel;
				}

				/*
				if (fabs((target[i] - observed_vel) / target[i]) > thresh){
					odr.val = odr.val * 0.1;
				}
				*/

				err[i]->shareOdr(odr);
			}
		}
	}

	/* Override: Initialize */
	void initialize(){
	}

	/* Personal private variables */
	float target[4];
	float prev_observed[4];

	/* Personal private functions */
	float calcInvKinematics(tf, float, float, float);

public:
	/* Constructor */
	WheelObserver(){
		obs = nullptr;
		for (int i = 0; i < 4; i++){
			wheel[i] = nullptr;
			err[i] = nullptr;
			prev_observed[i] = 0.0;
		}
	}
};


} /* namespace dRobot */

#endif /* WHEELOBSERVER_H_ */
