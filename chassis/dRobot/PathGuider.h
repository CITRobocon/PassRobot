
#ifndef PATHGUIDER_H_
#define PATHGUIDER_H_

/*
 *   Name: PathGuider
 *   About: An interface to positioning system using lidar with PC
 *
 *   Upward output: 2D pose from attached origin
 *   Upward input:	2D pose as attached origin
 *
 *   Downward output: None
 *   Downward input:  Dead reckoning system as initial value
 *                        ______________
 *     DeadReckoning --> |              | --> PathGuider/float
 *                       |  PathGuider  |
 *              None <-- |______________| <-- PathGuider/none
 */


#include <cstdint>
#include "common_includes.h"
#include "../connection/scom.h"

#include "device.h"
#include "general_msgs/empty_msgs.h"
#include "general_msgs/numeric_msgs.h"
#include "general_msgs/geometry_msgs.h"
#include "general_odrs/empty_odrs.h"
#include "general_odrs/numeric_odrs.h"
#include "general_odrs/geometry_odrs.h"


namespace dRobot {


/* Message structure */

// general_msgs/geometry_msgs/pose2d_msg


/* Order structure */

//


/* device class */

class PathGuider: public device<float_msg, float_array_odr> {
public:
	/* Pointers of child device */
	device<pose2d_msg, pose2d_odr> *positioner;
	device<empty_msg, twist_odr> *controller;

private:
	/* Override: Called back before share a message */
	void msgCallback(float_msg msg){
		myMsg.val = u;
	}

	/* Override: Called back after be shared an order */
	void odrCallback(float_array_odr odr){
		once = true;
		u = 0.0;

		if (odr.size == 12){
			int i, j;

			for (i = 0; i < 3; i++){
				for (j = 0; j < 4; j++){
					coes[i][j] = *(odr.arr + i*4 + j);
				}
			}

			coes[3][0] = 0.0;
			coes[3][1] = 0.0;
			coes[3][2] = 0.0;
			coes[3][3] = vt;
		}
		else if (odr.size == 16){
			int i, j;

			for (i = 0; i < 4; i++){
				for (j = 0; j < 4; j++){
					coes[i][j] = *(odr.arr + i*4 + j);
				}
			}
		}
	}

	/* Override: Update myself using ticker */
	void selfTickerUpdate(ticker_args targs){
		if (u != -100){
			pose2d_msg msg = positioner->shareMsg();
			twist_odr tw_odr;

			u = calcNearestPoint(msg.x, msg.y);

			if (u < 1.0){
				float ang = atan2(fy_prime(u + du), fx_prime(u + du));
				float err_n = (fx(u + du) - msg.x) * sin(-ang) + (fy(u + du) - msg.y) * cos(-ang);
				float err_r = ftheta(u + du) - msg.theta;

				if(err_r > M_PI){
					err_r -= 2.0*M_PI;
				}
				else if(err_r < -M_PI){
					err_r += 2.0*M_PI;
				}

				tw_odr.vx = fv(u) * cos(ang) - kn * err_n * sin(ang);
				tw_odr.vy = fv(u) * sin(ang) + kn * err_n * cos(ang);
				tw_odr.omega = kr * err_r;
			}
			else{
				tw_odr.vx = 0.0;
				tw_odr.vy = 0.0;
				tw_odr.omega = 0.0;
			}

			controller->shareOdr(tw_odr);
		}
	}

	void initialize(){
	}

	/* Personal private variables */
	bool once;
	float coes[4][4];
	float u;

	float vt;
	float kn, kr;
	float du;

	/* Personal private functions */
	float calcNearestPoint(float, float);
	float fx(float);
	float fy(float);
	float ftheta(float);
	float fv(float);
	float fx_prime(float);
	float fy_prime(float);

public:
	/* Constructor */
	PathGuider(float gain_n, float gain_r){
		kn = gain_n;
		kr = gain_r;

		du = 0.010;
		vt = 0.30;
		u = -100;
	}
};


} /* namespace dRobot */

#endif /* PATHGUIDER_H_ */
