
#include "PathGuider.h"

namespace dRobot {


/* PRIVATE */

/* Personal private functions */

float PathGuider::fx(float v){
	return coes[0][0]*v*v*v + coes[0][1]*v*v + coes[0][2]*v + coes[0][3];
}

float PathGuider::fy(float v){
	return coes[1][0]*v*v*v + coes[1][1]*v*v + coes[1][2]*v + coes[1][3];
}

float PathGuider::ftheta(float v){
	return coes[2][0]*v*v*v + coes[2][1]*v*v + coes[2][2]*v + coes[2][3];
}

float PathGuider::fv(float v){
	return coes[3][0]*v*v*v + coes[3][1]*v*v + coes[3][2]*v + coes[3][3];
}

float PathGuider::fx_prime(float v){
	return 3.0*coes[0][0]*v*v + 2.0*coes[0][1]*v + coes[0][2];
}

float PathGuider::fy_prime(float v){
	return 3.0*coes[1][0]*v*v + 2.0*coes[1][1]*v + coes[1][2];
}

float PathGuider::calcNearestPoint(float x, float y){
	float v = u;
	float kc[6] = {};
	float c_prime;

	kc[0] = 3.0*(coes[0][0]*coes[0][0] + coes[1][0]*coes[1][0]);
	kc[1] = 5.0*(coes[0][0]*coes[0][1] + coes[1][0]*coes[1][1]);
	kc[2] = (4.0*coes[0][0]*coes[0][2] + 2.0*coes[0][1]*coes[0][1]
			     + 4.0*coes[1][0]*coes[1][2] + 2.0*coes[1][1]*coes[1][1]);
	kc[3] = 3.0*(coes[0][0]*(coes[0][3] - x) + coes[0][1]*coes[0][2]
			     + coes[1][0]*(coes[1][3] - y) + coes[1][1]*coes[1][2]);
	kc[4] = 2.0*coes[0][1]*(coes[0][3] - x) + coes[0][2]*coes[0][2]
			 + 2.0*coes[1][1]*(coes[1][3] - y) + coes[1][2]*coes[1][2];
	kc[5] = coes[0][2]*(coes[0][3] - x) + coes[1][2]*(coes[1][3] - y);

	for (int i = 0; i < 10; i++){
		c_prime = 5.0*kc[0]*v*v*v*v + 4.0*kc[1]*v*v*v + 3.0*kc[2]*v*v + 2.0*kc[3]*v + kc[4];
		if (c_prime != 0.0)
			v -= (kc[0]*v*v*v*v*v + kc[1]*v*v*v*v + kc[2]*v*v*v + kc[3]*v*v + kc[4]*v + kc[5])/c_prime;
	}

	return v;
}


} /* namespace dRobot */
