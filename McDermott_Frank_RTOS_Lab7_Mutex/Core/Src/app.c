/*
 * app.c
 *
 *  Created on: Mar 13, 2024
 *      Author: Frank McDermott
 */

#include "app.h"


/************************************
 * 			Static Data				*
 ************************************/

static struct VehicleSpeedData_t
{
	uint8_t speed;					/* Current speed of the vehicle */
	uint8_t speedIncrementCount;	/* Count of how many times the speed has been incremented */
	uint8_t speedDecrementCount;	/* Count of how many times the speed has been decremented */
}speedData;

static struct VehicleDirectionData_t
{
	vehicleDirection direction;		/* Current vehicle direction */
	uint8_t leftTurnCount;			/* Number of left turns the vehicle has made */
	uint8_t rightTurnCount;			/* Number of right turns the vehicle has made */
}directionData;


/* ------------------ TASKS ------------------------ */
/* Static Task: Speed Setpoint */
static osThreadId_t speedSetpointTaskID;
static const osThreadAttr_t speedSetpointTaskAttr = { .name = "speedSetpointTask",
													  .priority = osPriorityHigh };
/* Static Task: Vehicle Direction */
static osThreadId_t vehicleDirectionTaskID;
static const osThreadAttr_t vehicleDirectionTaskAttr = { .name = "vehicleDirectionTask",
		   	   	   	   	   	   	   	   	   	   	   	   	 .priority = osPriorityHigh };
/* Static Task: Veicle Monitor */
static osThreadId_t vehicleMonitorTaskID;
static const osThreadAttr_t vehicleMonitorTaskAttr = { .name = "vehicleMonitorTask",
													   .priority = osPriorityAboveNormal };
/* Static Task: LED Output */
static osThreadId_t ledOutputTaskID;
static const osThreadAttr_t ledOutputTaskAttr = { .name = "ledOutputTask",
												  .priority = osPriorityNormal };
/* Static Task: LCD Display */
static osThreadId_t lcdDisplayTaskID;
static const osThreadAttr_t lcdDisplayTaskAttr = { .name = "lcdDisplayTask",
												   .priority = osPriorityNormal };

/* ------------------ TIMERS ------------------------ */
/* Static Timer: Hold Button */
static osTimerId_t holdButtonTimerID;
static const osTimerAttr_t holdButtonTimerAttr = { .name = "holdButtonTimer" };
/* Static Timer: Vehicle Direction Task */
static osTimerId_t vehicleDirWakeupTimerID;
static const osTimerAttr_t vehicleDirWakeupTimerAttr = { .name = "vehicleDirWakeupTimer" };
/* Static Timer: LCD Display Task */
static osTimerId_t lcdDisplayWakeupTimerID;
static const osTimerAttr_t lcdDisplayWakeupTimerAttr = { .name = "lcdDisplayWakeupTimer" };
/* Static Timer: Direction Alert */
static osTimerId_t directionAlertTimerID;
static const osTimerAttr_t directionAlertTimerAttr = { .name = "directionAlertTimer" };

/* ------------------ SEMAPHORES ------------------------ */
/* Static Semaphore: Button State */
static StaticTask_t buttonStateSemaphoreTCB;
static osSemaphoreId_t buttonStateSemaphorID;
static const osSemaphoreAttr_t buttonStateSemaphorAttr = { .name = "buttonStateSemaphor",
														   .attr_bits = 0,
														   .cb_mem = &buttonStateSemaphoreTCB,
														   .cb_size = sizeof(buttonStateSemaphoreTCB)};
/* Static Semaphore: Vehicle Direction */
static StaticTask_t vehicleDirSemaphoreTCB;
static osSemaphoreId_t vehicleDirSemaphoreID;
static const osSemaphoreAttr_t vehicleDirSemaphoreAttr = { .name = "vehicleDirSemaphore",
														   .attr_bits = 0,
														   .cb_mem = &vehicleDirSemaphoreTCB,
														   .cb_size = sizeof(vehicleDirSemaphoreTCB)};;
/* Static Semaphore: LCD Display */
static StaticTask_t lcdDisplaySemaphoreTCB;
static osSemaphoreId_t lcdDisplaySemaphoreID;
static const osSemaphoreAttr_t lcdDisplaySemaphoreAttr = { .name = "lcdDisplaySemaphore",
													       .attr_bits = 0,
														   .cb_mem = &lcdDisplaySemaphoreTCB,
														   .cb_size = sizeof(lcdDisplaySemaphoreTCB)};

/* ------------------ MUTEXES ------------------------ */
/* Static Mutex: Speed Data */
static osMutexId_t speedDataMutexID;
static const osMutexAttr_t speedDataMutexAttr = { .name = "speedDataMutex" };
/* Static Mutex: Vehicle Direction Data */
static osMutexId_t vehicleDirDataMutexID;
static const osMutexAttr_t vehicleDirDataMutexAttr = { .name = "vehicleDirDataMutex" };

/* ------------------ EVENT FLAGS ------------------------ */
/* Static Event Flag: Vehicle Monitor */
static osEventFlagsId_t vehicleMonitorEventFlagID;
static const osEventFlagsAttr_t vehicleMonitorEventFlagAttr = { .name = "vehicleMonitorEventFlag" };
/* Static Event Flag: LCD Output */
static osEventFlagsId_t ledOutputEventFlagID;
static const osEventFlagsAttr_t ledOutputEventFlagAttr = { .name = "ledOutputEventFlag" };

/* ------------------ BOOLEANS ------------------------ */
/* Static Bool: Buttons */
static volatile bool buttonPressed		= false;
static volatile bool buttonHeld			= false;


/****************************************
 * 			Forward Declarations		*
 ****************************************/

/* Static Task Functions */
static void speedSetpointTask(void* arg);
static void vehicleDirectionTask(void* arg);
static void vehicleMonitorTask(void* arg);
static void ledOutputTask(void* arg);
static void lcdDisplayTask(void* arg);
/* Static Callback Functions */
static void holdButtonTimerCallback(void* arg);
static void vehicleDirWakeupTimerCallback(void* arg);
static void lcdDisplayWakeupTimerCallback(void* arg);
static void directionAlertTimerCallback(void* arg);
/* Static Helper Functions */
static void initTasks(void);
static void initTimers(void);
static void initSempahores(void);
static void initMutexes(void);
static void initEventFlags(void);
static void startTimers(void);
static void lcdInit(void);
static gyroRotationRate getGyroRateOfRotation(void);
static vehicleDirection determineVehicleDirection(gyroRotationRate gyro);
static void updateLCD(uint8_t speed, vehicleDirection direction);

/****************************************
 *		Function Definitions: Public	*
 ****************************************/

/*
 * @brief Initialize the application
 */
void appInit(void)
{
	// Initialize Tasks, Timers, and ITC structures
	initTimers();
	initSempahores();
	initMutexes();
	initEventFlags();

	// Initialize the LTCD
	lcdInit();

	// Initialize the Gyro
	Gyro_Init();

	// Start timers
	initTasks();
	startTimers();
}


/********************************************
 *	Function Definitions: Private Helpers	*
 ********************************************/


/*
 * @brief Initialize all tasks
 */
void initTasks(void)
{
	// Create new task threads
	speedSetpointTaskID = osThreadNew(speedSetpointTask, NULL, &speedSetpointTaskAttr);
	vehicleDirectionTaskID = osThreadNew(vehicleDirectionTask, NULL, &vehicleDirectionTaskAttr);
	vehicleMonitorTaskID = osThreadNew(vehicleMonitorTask, NULL, &vehicleMonitorTaskAttr);
	lcdDisplayTaskID = osThreadNew(lcdDisplayTask, NULL, &lcdDisplayTaskAttr);
	ledOutputTaskID = osThreadNew(ledOutputTask, NULL, &ledOutputTaskAttr);

	// Verify that all task threads were created successfully
	assert(speedSetpointTaskID != NULL);
	assert(vehicleDirectionTaskID != NULL);
	assert(vehicleMonitorTaskID != NULL);
	assert(lcdDisplayTaskID != NULL);
	assert(ledOutputTaskID != NULL);
}


/*
 * @brief Initalize app timers
 */
void initTimers(void)
{
	// Create timers
	holdButtonTimerID = osTimerNew(holdButtonTimerCallback, osTimerOnce, NULL, &holdButtonTimerAttr);
	vehicleDirWakeupTimerID = osTimerNew(vehicleDirWakeupTimerCallback, osTimerPeriodic, NULL, &vehicleDirWakeupTimerAttr);
	lcdDisplayWakeupTimerID = osTimerNew(lcdDisplayWakeupTimerCallback, osTimerPeriodic, NULL, &lcdDisplayWakeupTimerAttr);
	directionAlertTimerID = osTimerNew(directionAlertTimerCallback, osTimerOnce, NULL, &directionAlertTimerAttr);

	// Verify each of the timers was setup properly
	assert(holdButtonTimerID != NULL);
	assert(vehicleDirWakeupTimerID != NULL);
	assert(lcdDisplayWakeupTimerID != NULL);
	assert(directionAlertTimerID != NULL);
}


/*
 * @brief Initialize app semaphores
 */
void initSempahores(void)
{
	// Create new app semaphores
	buttonStateSemaphorID = osSemaphoreNew(MAKE_BINARY_SEMAPHORE, SEMAPHORE_ZERO_INIT_TOKENS,
										   &buttonStateSemaphorAttr);
	vehicleDirSemaphoreID = osSemaphoreNew(MAKE_BINARY_SEMAPHORE, SEMAPHORE_ZERO_INIT_TOKENS,
										   &vehicleDirSemaphoreAttr);
	lcdDisplaySemaphoreID = osSemaphoreNew(MAKE_BINARY_SEMAPHORE, SEMAPHORE_ZERO_INIT_TOKENS,
										   &lcdDisplaySemaphoreAttr);

	// Verify each semaphore was initialized sucessfully
	assert(buttonStateSemaphorID != NULL);
	assert(vehicleDirSemaphoreID != NULL);
	assert(lcdDisplaySemaphoreID != NULL);
}


/*
 * @brief Initialize app mutexes
 */
void initMutexes(void)
{
	// Create app mutexes
	speedDataMutexID = osMutexNew(&speedDataMutexAttr);
	vehicleDirDataMutexID = osMutexNew(&vehicleDirDataMutexAttr);

	// Verifiy that the mutexes were created successfully
	assert(speedDataMutexID != NULL);
	assert(vehicleDirDataMutexID != NULL);
}


/*
 * @brief Initialize app event flags
 */
void initEventFlags(void)
{
	// Create app event flags
	vehicleMonitorEventFlagID = osEventFlagsNew(&vehicleMonitorEventFlagAttr);
	ledOutputEventFlagID = osEventFlagsNew(&ledOutputEventFlagAttr);

	// Verify that the event flags were created successfully
	assert(vehicleMonitorEventFlagID != NULL);
	assert(ledOutputEventFlagID != NULL);
}


/*
 * @brief Starts the task wakeup timers
 */
void startTimers(void)
{
	// Start task wakeup timers
	osStatus_t dirWakeupTimerStatus = osTimerStart(vehicleDirWakeupTimerID, VEHICLE_DIR_WAKEUP_TIMER_TICKS);
	osStatus_t lcdWakeupTimerStatus = osTimerStart(lcdDisplayWakeupTimerID, LCD_DISPLAY_TIMER_TICKS);

	// Verify that the timers were started successfully
	assert(dirWakeupTimerStatus == osOK);
	assert(lcdWakeupTimerStatus == osOK);
}


/*
 * @brief Initialize the LCD
 */
void lcdInit(void)
{
	LTCD__Init();
	LTCD_Layer_Init(0);

	LCD_Clear(0,LCD_COLOR_CYAN);
	LCD_SetTextColor(LCD_COLOR_BLACK);
	LCD_SetFont(&Font16x24);
}


/*
 *  @brief Get Gyro Rate of Rotation
 *  @details	Samples the gyro and uses the raw velocity data to categorize
 *  			the velocity as clockwise/counterclockwise and slow/fast.
 *
 *  @return Returns the categorized gyro velocity rate
 * */
gyroRotationRate getGyroRateOfRotation(void)
{
	// Variable to store and return the gyro rotation rate
	gyroRotationRate gyroRate;

	// Get the gyro velocity
	int16_t rawVelocity = Gyro_Get_Velocity();

	// Set the gyro rate based upon where the raw value
	// of the gyro velocity fits within the enumerated
	// gyroRotationRate ranges:
	// 		velocity <= -15000 			= counterClockwiseFast
	//		-15000 < velocity <= -2000 	= counterClockwiseSlow
	//		-2000 < velocity < 2000		= nearlyZero (treated as clockwise)
	//		150 <= velocity < 15000		= clockwiseSlow
	//		velocity >= 15000			= clockwiseFast
	if(rawVelocity <= counterClockwiseFast)
	{
		gyroRate = counterClockwiseFast;
	}
	else if(rawVelocity <= counterClockwiseSlow)
	{
		gyroRate = counterClockwiseSlow;
	}
	else if(rawVelocity < clockwiseSlow)
	{
		gyroRate = nearlyZero;
	}
	else if(rawVelocity < clockwiseFast)
	{
		gyroRate = clockwiseSlow;
	}
	else // rawVelocity > clockwiseFast
	{
		gyroRate = clockwiseFast;
	}

	return gyroRate;
}


/*
 * @brief Determines which direction the vehicle is moving
 *
 * @param[in] gyro Categorized gyro data into left/right
 * 				   turns or driving straight.
 *
 * @return Returns the direction that the vehicle is moving
 */
vehicleDirection determineVehicleDirection(gyroRotationRate gyro)
{
	vehicleDirection direction;

	// Convert gyro rotation data to vehicle direction data
	switch(gyro)
	{
		case counterClockwiseFast:
			direction = hardLeftTurn;
			break;
		case counterClockwiseSlow:
			direction = gradualLeftTurn;
			break;
		case clockwiseSlow:
			direction = gradualRightTurn;
			break;
		case clockwiseFast:
			direction = hardRightTurn;
			break;
		default:
			direction = drivingStraight;
			break;
	}

	return direction;
}


/*
 * @brief Update LCD with speed an direction data
 *
 * @param[in] speed Speed data to diplay on the LCD
 * @param[in] direction Direction data to display on the LCD
 */
void updateLCD(uint8_t speed, vehicleDirection direction)
{
	char* speedText = "Speed: ";
	char* dirText;

	while(1)
	{
		switch(direction)
		{
			case drivingStraight:
				dirText = "Dir: Straight";
				break;
			case gradualLeftTurn:
				dirText = "Dir: Grad Left";
				break;
			case gradualRightTurn:
				dirText = "Dir: Grad Right";
				break;
			case hardLeftTurn:
				dirText = "Dir: Hard Left";
				break;
			case hardRightTurn:
				dirText = "Dir: Hard Right";
				break;
			default:
				dirText = "";
				break;
		}

		LCD_Clear(0,LCD_COLOR_CYAN);
		LCD_DisplayString(10, 130, speedText);
		LCD_DisplayNumber(100, 130, speed);
		LCD_DisplayString(120, 130, "MPH");
		LCD_DisplayString(10, 180, dirText);
	}
}


/********************************************
 *	Function Definitions: Private Tasks		*
 ********************************************/


/*
 * @brief Speed Setpoint Task Function
 *
 * @details Task is woken up by pending on the Button State Semaphore.
 * 			Updates the vehicle speed data when the button is pressed
 * 			(accelerate) or held (decelerate). Signals a speed change
 * 			by posting to the Vehicle Monitor Event Flag in order to
 * 			signal this change to the Vehicle Monitor Task.
 *
 * @param[in] arg Dummy parameter for use with osThreadNew()
 */
void speedSetpointTask(void* arg)
{
	while(1)
	{
		// Acquire the button state sempahore
		osStatus_t semaphoreStatus = osSemaphoreAcquire(buttonStateSemaphorID, osWaitForever);
		assert(semaphoreStatus == osOK);

		// Start the oneshot hold button timer
		osStatus_t status = osTimerStart(holdButtonTimerID, HOLD_BTN_TIMER_TICKS_1S);
		assert(status == osOK);

		// Wait until the button is released
		while(buttonPressed){}

		// Since the button was pressed, accelerate by 5
		int acceleration = 5;

		// If the button was held sufficiently long (1 second), flip the sign
		// of acceleration (from 5 to -5) so signal deceleration
		if(buttonHeld == true)
		{
			buttonHeld = false;
			acceleration *= -1;
		}

		// Acquire the speed data mutex to set the new speed data
		osStatus_t mutexStatus = osMutexAcquire(speedDataMutexID, osWaitForever);
		assert(mutexStatus == osOK);

		// Update vehicle speed
		speedData.speed += acceleration;

		// Speed should never be negative
		if(acceleration < 0)
		{
			speedData.speed = 0;
			speedData.speedDecrementCount++;
		}
		else
		{
			speedData.speedIncrementCount++;
		}


		// speedData updated, release the mutex
		mutexStatus = osMutexRelease(speedDataMutexID);
		assert(mutexStatus == osOK);

		// Raise the speed update event flag to signal to the Vehicle Monitor
		// Task that the speed has been updated
		osEventFlagsSet(vehicleMonitorEventFlagID, speedUpdateEventFlag);

		// Verify that the event flag was set successfully
//		assert(eventStatus & speedAndDirectionEventFlags);
	}
}


/*
 * @brief Vehicle Direction Task
 *
 * #details Task is woken up by pending on the Vehicle Direction Semaphore.
 * 			Samples the gyro and updates the vehicle direction data. Posts
 * 			on the Vehicle Monitor Event Flag to signal a direction change
 * 			to the Vehicle Monitor Task.
 *
 * @param[in] arg Dummy parameter for use with osThreadNew()
 */
void vehicleDirectionTask(void* arg)
{
	while(1)
	{
		// Wakeup task when the semaphore is released by
		// the Vehicle Direction Timer Callback
		osStatus_t status = osSemaphoreAcquire(vehicleDirSemaphoreID, osWaitForever);
		assert(status == osOK);

		// Sample the gyro
		gyroRotationRate gyro = getGyroRateOfRotation();

		// Determine which direction the vehicle is moving
		vehicleDirection newDirection = determineVehicleDirection(gyro);

		// Acquire the vehicle direction data mutex
		osStatus_t mutexStatus = osMutexAcquire(vehicleDirDataMutexID, osWaitForever);
		assert(mutexStatus == osOK);

		// Update the vehicle direction data
		directionData.direction = newDirection;

		// Update direction data turn counts
		switch(newDirection)
		{
			case hardLeftTurn:
				directionData.leftTurnCount += 2;
				break;
			case gradualLeftTurn:
				directionData.leftTurnCount++;
				break;
			case gradualRightTurn:
				directionData.rightTurnCount++;
				break;
			case hardRightTurn:
				directionData.rightTurnCount += 2;
				break;
			default:
				break;
		}

		// Release the vehicle direction data mutex
		mutexStatus = osMutexRelease(vehicleDirDataMutexID);
		assert(mutexStatus == osOK);

		// Raise the Direction Update Flag to signal to the Vehicle
		// Monitor Task that new vehicle direction data is available
		osEventFlagsSet(vehicleMonitorEventFlagID, directionUpdateEventFlag);
//		assert(eventStatus & speedAndDirectionEventFlags);
	}
}


/*
 * @brief Vehicle Monitor Task
 *
 * @details Task is woken up by a pend on the Vehicle Monitor Event Flag.
 * 			Checks for speed and/or direction violations and posts on the
 * 			LED Output Event Flag to signal to the LED Output Task which
 * 			violation has occured.
 *
 * @param[in] arg Dummy parameter for use with osThreadNew()
 */
void vehicleMonitorTask(void* arg)
{
	uint8_t currentSpeed = 0;
	vehicleDirection currentDirection = drivingStraight;
	vehicleDirection previousDirection;

	while(1)
	{
		// Pend on the Vehicle Monitor Event Flag
		uint32_t eventStatus = osEventFlagsWait(vehicleMonitorEventFlagID, speedAndDirectionEventFlags,
												osFlagsWaitAny, osWaitForever);

		osStatus_t status;

		// Check if the speed update event flag is set
		if(eventStatus & speedUpdateEventFlag)
		{
			// Acquire the vehicle speed data mutex before trying to
			// read vehicle speed data
		    osStatus_t mutexStatus = osMutexAcquire(speedDataMutexID, osWaitForever);
		    assert(mutexStatus == osOK);

		    // Update speed data
		    currentSpeed = speedData.speed;

		    // Done reading vehicle speed data; release the vehicle
		    // speed data mutex
		    mutexStatus = osMutexRelease(speedDataMutexID);
		    assert(mutexStatus == osOK);
		}


		// Check if the direction update event flag is set
		if(eventStatus & directionUpdateEventFlag)
		{
			// Acquire the vehicle direction data mutex before trying to
			// read vehicle direction data
			osStatus_t mutexStatus = osMutexAcquire(vehicleDirDataMutexID, osWaitForever);
		    assert(mutexStatus == osOK);

		    // Update directional data
		    previousDirection = currentDirection;
		    currentDirection = directionData.direction;

		    // Done reading vehicle direction data; release the vehicle
		    // direction data mutex
		    mutexStatus = osMutexRelease(vehicleDirDataMutexID);
		    assert(mutexStatus == osOK);
		}

		/* Speed Violation – Light LED3 (green) for the following warnings:
		 * - Over limit, regardless of direction. Suggested limit is 75 mph.
		 * - Over limit, when making a turn. Suggested limit is 45 mph. */
		if((currentSpeed > 75) || ((currentSpeed > 45) && (currentDirection != drivingStraight)))
		{
			osEventFlagsSet(ledOutputEventFlagID, speedUpdateEventFlag);
//			assert(flagStatus & speedAndDirectionEventFlags);
		}
		else
		{
			osEventFlagsSet(ledOutputEventFlagID, deactivateSpeedAlertEventFlag);
//			assert(flagStatus & deactivateBothAlertEventFlags);
		}

		// Makes the compound conditional easier to read
		bool currentLeft = (currentDirection < drivingStraight);
		bool previouslyNotLeft = (previousDirection >= drivingStraight);
		bool currentRight = (currentDirection > drivingStraight);
		bool previouslyNotRight = (previousDirection <= drivingStraight);


		// Check if the vehicle has changed direction
		// NOTE: The direction alert timer is started/restarted when the
		//		 vehicle changes direction. It is only stopped when driving
		//		 straight.
		if((currentDirection == drivingStraight) 	||	/* Vehicle is now driving straight */
		   (currentLeft && previouslyNotLeft) 		||	/* Vehicle is now turning left; but wasn't previously */
		   (currentRight && previouslyNotRight))		/* Vehicle is now turning right; but wasn't previously */
		{
			// Direction changed, so set the deactivate direction alert flag
			osEventFlagsSet(ledOutputEventFlagID, deactivateDirAlertEventFlag);
//			assert(flagStatus & deactivateBothAlertEventFlags);

			// Since the vehicle is driving straight, it is not in danger
			// of committing a direction violation. Therefore, stop the
			// direction alert timer
			if(currentDirection == drivingStraight)
			{
				// Stop timer if running
				// NOTE: osTimerStop will return osErrorResource if
				// 		 the timer is not running
				if(osTimerIsRunning(directionAlertTimerID))
				{
					status = osTimerStop(directionAlertTimerID);
					assert(status == osOK);
				}
			}
			// While the vehicle did change direction, it is still in the process
			// of taking a turn. Therefore, restart the direction alert timer
			else
			{
				// NOTE: If the callback is called (after 5 seconds), then the direction
				// 		 alert event flag is raised and the proper LED is turned on via
				//		 the LED Output Task.
				status = osTimerStart(directionAlertTimerID, DIRECTION_ALERT_TIMER_TICKS);
				assert(status == osOK);
			}
		}
	}
}



/*
 * @brief LED Output Task
 *
 * @details Task is woken up by a pend on the LED Output Event Flag.
 * 			Turns LEDs on or off depending on which event flag was raised.
 * 			A speed violation is linked to the green LED (LED3) and a
 * 			direction violation is linked to the red LED (LED4).
 *
 * @param[in] arg Dummy parameter for use with osThreadNew()
 */
void ledOutputTask(void* arg)
{
	while(1)
	{
		// Pend on the alert update event flag
		uint32_t flags = osEventFlagsWait(ledOutputEventFlagID, LedOutputEventAllFlag,
										  osFlagsWaitAny, osWaitForever);

//		assert(flags & LedOutputEventAllFlag);

		// Drive the LEDs according to which flag was set
		switch(flags)
		{
			// Activate Speed Violation: Turn on green LED (LED3)
			case activateSpeedAlertEventFlag:
				HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
				break;
			// Activate Direction Violation: Turn on red LED (LED4)
			case activateDirAlertEventFlag:
				HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
				break;
			// Deactivate Speed Violation: Turn off green LED (LED3)
			case deactivateSpeedAlertEventFlag:
				HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
				break;
			// Deactivate Direction Violation: Turn off red LED (LED4)
			case deactivateDirAlertEventFlag:
				HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
				break;
			default:
				break;
		}
	}
}


/*
 * @brief LCD Display Task
 *
 * @details
 *
 * @param[in] arg Dummy parameter for use with osThreadNew()
 */
void lcdDisplayTask(void* arg)
{
	while(1)
	{
		// Pend on the LCD Display Semaphore
		osStatus_t status = osSemaphoreAcquire(lcdDisplaySemaphoreID, osWaitForever);
		assert(status == osOK);

		// Acquire the Speed Data Mutex
		osStatus_t mutexStatus = osMutexAcquire(speedDataMutexID, osWaitForever);
		assert(mutexStatus == osOK);

		uint8_t currentSpeed = speedData.speed;

		// Release the Speed Data Mutex
		mutexStatus = osMutexRelease(speedDataMutexID);
		assert(mutexStatus == osOK);

		// Acquire the Vehicle Direction Data Mutex
		mutexStatus = osMutexAcquire(vehicleDirDataMutexID, osWaitForever);
		assert(mutexStatus == osOK);

		vehicleDirection currentDirection = directionData.direction;

		// Release the Vehicle Direction Data Mutex
		mutexStatus = osMutexRelease(vehicleDirDataMutexID);
		assert(mutexStatus == osOK);

		// Update the LCD with the current speed and direction
		updateLCD(currentSpeed, currentDirection);
	}
}


/********************************************************
 * 		Function Definitions: Private Callbacks			*
 ********************************************************/

/*
 * @brief Hold Button Timer Callback
 *
 * @details If this callback executes, it mean that the button
 * 			was held for more than 1 second.
 *
 * @param[in] arg Dummy parameter for use with osTimerNew()
 */
void holdButtonTimerCallback(void* arg)
{
	// Avoids a compiler warning for unused parameter
	(void) &arg;

	// Notify the speedSetpoint task that the button was held
	buttonHeld = true;
}


/*
 * @brief Vehicle Direction Wakeup Timer Callback
 *
 * @details Posts on the Vehicle Direction Semaphore in order
 * 			to wakeup the Vehicle Direction Task.
 *
 * @param[in] arg Dummy parameter for use with osTimerNew()
 */
void vehicleDirWakeupTimerCallback(void* arg)
{
	// Avoids a compiler warning for unused parameter
	(void) &arg;

	osStatus_t status = osSemaphoreRelease(vehicleDirSemaphoreID);
	assert(status == osOK);
}


/*
 * @brief LCD Display Wakeup Timer Callback
 *
 * @details
 *
 * @param[in] arg Dummy parameter for use with osTimerNew()
 */
void lcdDisplayWakeupTimerCallback(void* arg)
{
	// Avoids a compiler warning for unused parameter
	(void) &arg;

	osStatus_t status = osSemaphoreRelease(lcdDisplaySemaphoreID);
	assert(status == osOK);
}


/*
 * @brief Direction Alert Timer Callback
 *
 * @details Posts on the LED Output Event Flag in order
 * 			to signal that a direction violation has occured.
 *
 * @param[in] arg Dummy parameter for use with osTimerNew()
 */
void directionAlertTimerCallback(void* arg)
{
	// Avoids a compiler warning for unused parameter
	(void) &arg;

	osEventFlagsSet(ledOutputEventFlagID, directionUpdateEventFlag);
//	assert(flagStatus & speedAndDirectionEventFlags);
}


/************************************************
 * 		Function Definitions: IRQ Handlers		*
 ************************************************/


/*
 * @brief User Button (GPIO) ISR
 *
 * @details When the user button is pressed/released, this ISR will
 * 			post on the Button State Semaphore inorder to wake up the
 * 			Speed Setpoint Task. Toggles the buttonPressed global variable
 * 			on press/release.
 */
void EXTI0_IRQHandler(void)
{
	// Disable interrupts
	HAL_NVIC_DisableIRQ(BUTTON_IRQn);

	osStatus_t status = osSemaphoreRelease(buttonStateSemaphorID);

	// verify status
	assert(status == osOK);

	// Toggle buttonPressed
	// - On press: true
	// - On release false
	buttonPressed = !buttonPressed;

	// Clear interrupt flag
	__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_PIN);

	// Re-enable interrupts
	HAL_NVIC_EnableIRQ(BUTTON_IRQn);
}

