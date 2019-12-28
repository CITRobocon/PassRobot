#if 0
#ifndef COMMANDER_H_
#define COMMANDER_H_

/*
 *   Name: Commander
 *   About: Commander of Robot with three wheels
 *
 *   Upward output:   2D translation velocity and z-axis rotation of Robot
 *   Upward input:	  None
 *
 *   Downward output: None
 *   Downward input:  Three displacements of wheel
 *                   _____________
 *     float x3 --> |             | --> Commander/twist
 *                  |  Commander  |
 *     empty x3 <-- |_____________| <-- Commander/empty
 *
 */

#include "device.h"
#include "tf/tf.h"
#include "general_msgs/empty_msgs.h"
#include "general_msgs/geometry_msgs.h"
#include "general_odrs/empty_odrs.h"
#include "general_odrs/geometry_odrs.h"


namespace dRobot {


/* Message structure */

// general_msgs/geometry_msgs/twist_msg


/* Order structure */

// general_odrs/empty_msgs/empty_msg


/* device class */

class Commander: public device<empty_msg, empty_odr> {
public:
	/* Pointers of child device */
	device<float_msg, empty_odr> *wheel[3];

protected:
	/* Override: Called back after be shared an order */
	void odrCallback(empty_odr odr){
	}

	/* Override: Update myself */
	void selfTickerUpdate(ticker_args targs){
		float period = (float)targs.period_ms * 0.0010;

		if (period > 0.0000001){
			float_msg msg1 = wheel[0]->shareMsg();
			float_msg msg2 = wheel[1]->shareMsg();
			float_msg msg3 = wheel[2]->shareMsg();

			b.set(msg1.val, msg2.val, msg3.val);

			vec3d delta = inverted_A * b;

			myMsg.vx = delta(0) / period;
			myMsg.vy = delta(1) / period;
			myMsg.omega = delta(2) / period;
		}
	}

	/* Override: Initialize */
	void initialize(){
		tf pose1 = wheel[0]->getRelativePose();
		tf pose2 = wheel[1]->getRelativePose();
		tf pose3 = wheel[2]->getRelativePose();

		A.set(pose1.posture(0,0), pose1.posture(1,0), -pose1.posture(0,0)*pose1.position(1) + pose1.posture(1,0)*pose1.position(0),
			  pose2.posture(0,0), pose2.posture(1,0), -pose2.posture(0,0)*pose2.position(1) + pose2.posture(1,0)*pose2.position(0),
			  pose3.posture(0,0), pose3.posture(1,0), -pose3.posture(0,0)*pose3.position(1) + pose3.posture(1,0)*pose3.position(0));

		inverted_A = inverse(A);
	}

	/* Personal private variables */
	mat3x3 A;
	mat3x3 inverted_A;
	vec3d b;

	/* Personal private functions */
	// None

public:
	/* Constructor */
	Commander(){
	}
};


} /* namespace dRobot */

#endif /* COMMANDER_H_ */
#endif
