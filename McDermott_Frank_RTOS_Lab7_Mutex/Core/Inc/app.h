/*
 * app.h
 *
 *  Created on: Mar 13, 2024
 *      Author: fjmcd
 */

#ifndef INC_APP_H_
#define INC_APP_H_

/************************************
 * 		Header Includes		*
 ************************************/
/* C-STD Headers */
#include <limits.h>
/* Core System Headers */
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
/* Application Headers */
#include "Gyro_Driver.h"
#include "LCD_Driver.h"


/************************************
 * 		Macro Definitions			*
 ************************************/
/* GPIO Definitions */
#define BUTTON_PIN					GPIO_PIN_0
#define BUTTON_PORT					GPIOA
#define BUTTON_IRQn					EXTI0_IRQn
#define RED_LED_PIN					GPIO_PIN_14
#define RED_LED_PORT				GPIOG
#define GREEN_LED_PIN				GPIO_PIN_13
#define GREEN_LED_PORT				GPIOG
/* Timer Definitions */
#define BTN_HOLD_TIMER_TICKS_1S		(uint32_t)1000
/* Event Flag Definitions */
#define SPEED_UPDATE_EVENT_FLAG		(uint32_t)0x01
#define DIRECTION_UPDATE_EVENT_FLAG	(uint32_t)0x02
#define ALERT_UPDATE_EVENT_FLAG		(uint32_t)0x04
/* Semaphore Definitions */
#define MAKE_BINARY_SEMAPHORE		1	/* A max count value of 1 creates a binary semaphore */
#define SEMAPHORE_ONE_INIT_TOKEN	1	/* Initialize sempahore token count to zero; timer will release it */


/************************************
 * 			Enumerations			*
************************************/

typedef enum
{
	counterClockwiseFast 	= -15000,	/* Faster counter-clockwise (-) rotation */
	counterClockwiseSlow 	= -2000,	/* Slow but affirmative counter-clockwise (-) rotation*/
	nearlyZero 				= 0,		/* Nearly zero clockwise (+) rotation */
	clockwiseSlow 			= 2000,		/* Slow but affirmative clockwise (+) rotation */
	clockwiseFast 			= 15000,	/* Faster clockwise (+) rotation */
}gyroRotationRate;


#endif /* INC_APP_H_ */
