
#ifndef BUTTON_H_
#define BUTTON_H_

/*
 *   Name: Button
 *   About: Button interface connecting to debuging PC
 *
 *   Upward output: None
 *   Upward input:	None
 *
 *   Downward output: Float value volumed keyboard input
 *   Downward input:  None
 *
 *                ____________
 *     empty --> |            | --> Button/empty
 *               |  Button  |
 *     float <-- |____________| <-- Button/empty
 *
 */


#include <cstdint>
#include "common_includes.h"

#include "device.h"
#include "general_msgs/empty_msgs.h"
#include "general_msgs/numeric_msgs.h"
#include "general_odrs/empty_odrs.h"
#include "general_odrs/numeric_odrs.h"
#include "general_odrs/geometry_odrs.h"


namespace dRobot {


/* Message structure */
// common_msgs/empty_msgs/empty_msg


/* Order structure */
// common_odrs/numeric_odrs/float_odr


/* Device class */

class Button: public device<uint8_msg, empty_odr> {
public:
	/* Pointers of child device */
	device<empty_msg, uint8_odr> *out;

protected:
	/* Override: Update myself */
	void selfInterUpdate(){
		uint8_odr odr;
		odr.val = 1 - (HAL_GPIO_ReadPin(gpio, gpio_pin) & 0x01);
		myMsg.val = odr.val;
		out->shareOdr(odr);
	}

	/* Personal private variables */
	GPIO_TypeDef *gpio;
	uint16_t gpio_pin;


	/* Personal private functions */

public:
	/* Constructor */
	Button(GPIO_TypeDef *gpiot, uint16_t pin){
		myMsg.val = 0;
		gpio = gpiot;
		gpio_pin = pin;
	}
};


} /* namespace dRobot */

#endif /* BUTTON_H_ */
