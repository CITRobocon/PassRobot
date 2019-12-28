/*
 * numeric_odrs.h
 *
 *  Created on: 2019/12/16
 *      Author: shohei
 */

#ifndef GENERAL_ODRS_NUMERIC_ODRS_H_
#define GENERAL_ODRS_NUMERIC_ODRS_H_

#include "cstdint"


namespace dRobot{


typedef struct{
	float val;
} float_odr;


typedef struct{
	uint8_t val;
} uint8_odr;


typedef struct{
	size_t size;
	float *arr;
} float_array_odr;


}; // namespace dRobot


#endif /* GENERAL_ODRS_NUMERIC_ODRS_H_ */
