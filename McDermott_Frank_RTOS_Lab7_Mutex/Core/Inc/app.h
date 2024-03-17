/*
 * app.h
 *
 *  Created on: Mar 13, 2024
 *      Author: Frank McDermott
 */

#ifndef INC_APP_H_
#define INC_APP_H_

/************************************
 * 			Header Includes			*
 ************************************/
/* C-STD Headers */
#include <limits.h>
#include <stdbool.h>
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
#define BUTTON_PIN							GPIO_PIN_0
#define BUTTON_PORT							GPIOA
#define BUTTON_IRQn							EXTI0_IRQn
#define RED_LED_PIN							GPIO_PIN_14
#define RED_LED_PORT						GPIOG
#define GREEN_LED_PIN						GPIO_PIN_13
#define GREEN_LED_PORT						GPIOG
/* Timer Definitions */
#define HOLD_BTN_TIMER_TICKS_1S				(uint32_t)1000	/* 1 second timer period */
#define VEHICLE_DIR_WAKEUP_TIMER_TICKS		(uint32_t)100	/* 100ms timer period */
#define DIRECTION_ALERT_TIMER_TICKS			(uint32_t)5000	/* 5 second timer period */
#define LCD_DISPLAY_TIMER_TICKS				(uint32_t)200	/* 100ms timer period */
/* Semaphore Definitions */
#define MAKE_BINARY_SEMAPHORE				1	/* A max count value of 1 creates a binary semaphore */
#define SEMAPHORE_ZERO_INIT_TOKENS			0	/* Initialize sempahore token count to zero */
#define SEMAPHORE_ONE_INIT_TOKEN			1	/* Initialize sempahore token count to one */

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


typedef enum
{
	hardLeftTurn,
	gradualLeftTurn,
	drivingStraight,
	gradualRightTurn,
	hardRightTurn,

}vehicleDirection;


typedef enum
{
	/* Hold Button Event Flags */
	buttonPressedEventFlag			= 0x001,	/* Reserved: For ITC options */
	buttonHeldEventFlag 			= 0x002,	/* Reserved: For ITC options */
	buttonReleasesEventFlag			= 0x004,	/* Reserved: For ITC options */
	holdButtonAllEventFlags			= 0x007,	/* Reserved: For ITC options */
	/* Vehicle Monitor Event Flag Group */
	speedUpdateEventFlag 			= 0x008,	/* Flag to signal the vehicle speed has been updated */
	directionUpdateEventFlag		= 0x010, 	/* Flag to signal the vehicle direction has been updated */
	vehicleMonitorBothFlags			= 0x018,	/* Both Vehicle Monitor Event Flags */
	/* LED Output Event Flag Group */
	activateSpeedAlertEventFlag		= 0x020, /* Flag to signal the speed alert is in effect */
	activateDirAlertEventFlag		= 0x040, /* Flag to signal the direction alert is in effect */
	deactivateSpeedAlertEventFlag	= 0x080, /* Flag to signal the speed alert is no longer in effect */
	deactivateDirAlertEventFlag		= 0x100, /* Flag to signal the direction alert is no longer in effect */
	ledOutputEventAllFlag			= 0x1E0	 /* All four LED OutPut event flags */
}vehicleEventFlags;


/****************************************
 *		Public Function Prototypes		*
 ****************************************/

void appInit(void);


#endif /* INC_APP_H_ */
