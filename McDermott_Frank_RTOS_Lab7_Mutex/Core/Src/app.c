/*
 * app.c
 *
 *  Created on: Mar 13, 2024
 *      Author: Frank McDermott
 */

#include "app.h"

typedef gyroRotationRate Direction_t;


/************************************
 * 			Static Variables		*
 ************************************/

static struct VehicleSpeedData_t
{
	uint8_t speed;					/* Current speed of the vehicle */
	uint8_t speedIncrementCount;	/* Count of how many times the speed has been incremented */
	uint8_t speedDecrementCount;	/* Count of how many times the speed has been decremented */
}speedData;

static struct VehicleDirectionData_t
{
	Direction_t direction;			/*  */
	uint8_t leftTurnCount;			/* Number of left turns the vehicle has made */
	uint8_t rightTurnCount;			/* Number of right turns the vehicle has made */
}directionData;


/* Static Task: Speed Setpoint */
static osThreadId_t speedSetpointTask;
static const osThreadAttr_t speedSetpointTaskAttr = { .name = "speedSetpointTask" };
/* Static Task: Vehicle Direction */
static osThreadId_t vehicleDirectionTask;
static const osThreadAttr_t vehicleDirectionTaskAttr = { .name = "vehicleDirectionTask" };
/* Static Task: Veicle Monitor */
static osThreadId_t vehicleMonitorTask;
static const osThreadAttr_t vehicleMonitorTaskAttr = { .name = "vehicleMonitorTask" };
/* Static Task: LED Output */
static osThreadId_t ledOutputTask;
static const osThreadAttr_t ledOutputTaskAttr = { .name = "ledOutputTask" };
/* Static Task: LCD Display */
static osThreadId_t lcdDisplayTask;
static const osThreadAttr_t lcdDisplayTaskAttr = { .name = "lcdDisplayTask" };

/* Static Timer: Speed Setpoint */
static osTimerId_t speedSetpointTimer;
static const osTimerAttr_t speedSetpointTimerAttr = { .name = "speedSetpointTimer" };
/* Static Timer: Button Hold */
static osTimerId_t buttonHoldTimer;
static const osTimerAttr_t buttonHoldTimerAttr = { .name = "buttonHoldTimer" };

/* Static Semaphore */
static osSemaphoreId_t buttonStateSemaphorID;
static const osSemaphoreAttr_t buttonStateSemaphorAttr = { .name = "buttonStateSemaphor" };

/* Static Semaphore */
static osMutexId_t speedDataMutexID;
static osMutexAttr_t speedDataMutexAttr = { .name = "speedDataMutex" };

/****************************************
 * 			Forward Declarations		*
 ****************************************/

/* Static Task Functions */
static void speedSetpoint(void* arg);
static void vehicleDirection(void* arg);
static void monitor(void* arg);
static void ledOutput(void* arg);
static void lcdDisplay(void* arg);




/****************************************
 * 			Function Definitions		*
 ****************************************/

void appInit(void)
{
	// Create hold button timer as a oneshot timer
	buttonHoldTimer = osTimerNew(buttonHoldTimerCallback, osTimerPeriodic, NULL, &buttonHoldTimerAttr);

	// Verify that the timer was created sucessfully
	while(buttonHoldTimer == NULL){}
}


/*
 * @brief
 */
void speedSetpoint(void* arg)
{
	while(1)
	{
		// Acquire the button state sempahore
		osStatus_t semaphoreStatus = osSemaphoreAcquire(buttonStateSemaphorID, osWaitForever);

		// Validate semaphore status
		while(semaphoreStatus != osOK){}

		// Start the oneshot hold button timer
		osStatus_t timerStatus = osTimerStart(buttonHoldTimer, BTN_HOLD_TIMER_TICKS_1S);

		// Validate the timer status
		while(timerStatus !=  osOK){}

	}

}


/*
 * @brief
 */
void vehicleDirection(void* arg)
{

}


/*
 * @brief
 */
void monitor(void* arg)
{

}



/*
 * @brief
 */
void ledOutput(void* arg)
{

}


/************************************************
 * 		Function Definitions: Callbacks			*
 ************************************************/

/*
 * @brief Hold button timer callback
 */
static void buttonHoldTimerCallback(void* arg)
{

}



/************************************************
 * 		Function Definitions: IRQ Handlers		*
 ************************************************/


/*
 * @brief User Button (GPIO) ISR
 *
 * @details
 * @retval none
 */
void EXTI0_IRQHandler(void)
{
	// Disable interrupts
	HAL_NVIC_DisableIRQ(BUTTON_IRQn);

	osStatus_t status = osSemaphoreRelease(buttonStateSemaphorID);

	// verify status
	while(status != osOK){}

	// Clear interrupt flag
	__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_PIN);

	// Re-enable interrupts
	HAL_NVIC_EnableIRQ(BUTTON_IRQn);
}
