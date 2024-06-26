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
/* Static Task: LCD Display */
static osThreadId_t lcdDisplayTaskID;
static const osThreadAttr_t lcdDisplayTaskAttr = { .name = "lcdDisplayTask",
												   .priority = osPriorityAboveNormal };

/* Static Task: LED Output */
static osThreadId_t ledOutputTaskID;
static const osThreadAttr_t ledOutputTaskAttr = { .name = "ledOutputTask" };

/* ------------------ TIMERS ------------------------ */
/* Static Timer: Hold Button */
static osTimerId_t holdButtonTimerID;
static const osTimerAttr_t holdButtonTimerAttr = { .name = "holdButtonTimer" };
/* Static Timer: Vehicle Direction Task */
static osTimerId_t vehicleDirWakeupTimerID;
static const osTimerAttr_t vehicleDirWakeupTimerAttr = { .name = "vehicleDirWakeupTimer" };
/* Static Timer: Direction Alert */
static osTimerId_t directionAlertTimerID;
static const osTimerAttr_t directionAlertTimerAttr = { .name = "directionAlertTimer" };
/* Static Timer: LCD Display Task */
static osTimerId_t lcdDisplayWakeupTimerID;
static const osTimerAttr_t lcdDisplayWakeupTimerAttr = { .name = "lcdDisplayWakeupTimer" };


/* ------------------ SEMAPHORES ------------------------ */
/* Static Semaphore: Button State */
//static StaticTask_t buttonStateSemaphoreTCB;
static osSemaphoreId_t buttonStateSemaphorID;
static const osSemaphoreAttr_t buttonStateSemaphorAttr = { .name = "buttonStateSemaphor" };
/* Static Semaphore: Vehicle Direction */
//static StaticTask_t vehicleDirSemaphoreTCB;
static osSemaphoreId_t vehicleDirSemaphoreID;
static const osSemaphoreAttr_t vehicleDirSemaphoreAttr = { .name = "vehicleDirSemaphore" };
/* Static Semaphore: LCD Display */
static osSemaphoreId_t lcdDisplaySemaphoreID;
static const osSemaphoreAttr_t lcdDisplaySemaphoreAttr = { .name = "lcdDisplaySemaphore" };

/* ------------------ MUTEXES ------------------------ */
/* Static Mutex: Speed Data */
static osMutexId_t speedDataMutexID;
static const osMutexAttr_t speedDataMutexAttr = { .name = "speedDataMutex",
												  .attr_bits = osMutexPrioInherit };

/* Static Mutex: Vehicle Direction Data */
static osMutexId_t vehicleDirDataMutexID;
static const osMutexAttr_t vehicleDirDataMutexAttr = { .name = "vehicleDirDataMutex",
		  	  	  	  	  	  	  	  	  	  	  	   .attr_bits = osMutexPrioInherit };

/* ------------------ EVENT FLAGS ------------------------ */
/* Static Event Flag: Vehicle Monitor */
static osEventFlagsId_t vehicleMonitorEventFlagID;
static const osEventFlagsAttr_t vehicleMonitorEventFlagAttr = { .name = "vehicleMonitorEventFlag" };
/* Static Event Flag: LCD Output */
static osEventFlagsId_t ledOutputEventFlagID;
static const osEventFlagsAttr_t ledOutputEventFlagAttr = { .name = "ledOutputEventFlag" };

/* ------------------ BOOLEANS ------------------------ */
/* Static Bool: Buttons */
static volatile bool buttonHeld = false;


/****************************************
 * 			Forward Declarations		*
 ****************************************/

/* Static Task Functions */
static void speedSetpointTask(void* arg);
static void vehicleDirectionTask(void* arg);
static void vehicleMonitorTask(void* arg);
static void lcdDisplayTask(void* arg);
static void ledOutputTask(void* arg);
/* Static Callback Functions */
static void holdButtonTimerCallback(void* arg);
static void vehicleDirWakeupTimerCallback(void* arg);
static void directionAlertTimerCallback(void* arg);
static void lcdDisplayWakeupTimerCallback(void* arg);
/* Static Helper Functions: Initialization */
static void initTasks(void);
static void initTimers(void);
static void initSempahores(void);
static void initMutexes(void);
static void initEventFlags(void);
static void startTimers(void);
static void lcdInit(void);
/* Static Helper Functions: Vehicle Data */
static void updateVehicleSpeedData(const int acceleration);
static void updateVehicleDirectionData(const vehicleDirection newDirection);
static gyroRotationRate getGyroRateOfRotation(void);
static vehicleDirection determineVehicleDirection(const gyroRotationRate gyro);
static uint8_t getVehicleSpeed(void);
static void getVehicleDirection(vehicleDirection* currentDirection, vehicleDirection* previousDirection);
static void checkForVehicleSpeedViolation(uint8_t currentSpeed, vehicleDirection currentDirection);
static void checkForVehicleDirectionViolation(vehicleDirection previousDirection, vehicleDirection currentDirection);
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

	// Start tasks/timers
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
	directionAlertTimerID = osTimerNew(directionAlertTimerCallback, osTimerOnce, NULL, &directionAlertTimerAttr);
	lcdDisplayWakeupTimerID = osTimerNew(lcdDisplayWakeupTimerCallback, osTimerPeriodic, NULL, &lcdDisplayWakeupTimerAttr);

	// Verify each of the timers was setup properly
	assert(holdButtonTimerID != NULL);
	assert(vehicleDirWakeupTimerID != NULL);
	assert(directionAlertTimerID != NULL);
	assert(lcdDisplayWakeupTimerID != NULL);
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
 *
 */
void updateVehicleSpeedData(const int acceleration)
{
	// Acquire the speed data mutex before reading the vehicle speed data
	osStatus_t mutexStatus = osMutexAcquire(speedDataMutexID, osWaitForever);
	assert(mutexStatus == osOK);

	// Update vehicle speed
	uint8_t currentSpeed = speedData.speed;

	// Update vehicle speed
	// NOTE: Speed should never be negative
	if((currentSpeed + acceleration) < 0){ speedData.speed = 0; }
	else { speedData.speed += acceleration; }

	// Update vehicle increment/decrement counts
	if(acceleration > 0) { speedData.speedIncrementCount++; }
	else { speedData.speedDecrementCount++; }


	// speedData updated, release the mutex
	mutexStatus = osMutexRelease(speedDataMutexID);
	assert(mutexStatus == osOK);
}


/*
 *
 */
void updateVehicleDirectionData(const vehicleDirection newDirection)
{
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
vehicleDirection determineVehicleDirection(const gyroRotationRate gyro)
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
 *
 */
uint8_t getVehicleSpeed(void)
{
	// Acquire the vehicle speed data mutex before trying to
	// read vehicle speed data
	osStatus_t mutexStatus = osMutexAcquire(speedDataMutexID, osWaitForever);
	assert(mutexStatus == osOK);

	// Update speed data
	uint8_t currentSpeed = speedData.speed;

	// Done reading vehicle speed data; release the vehicle
	// speed data mutex
	mutexStatus = osMutexRelease(speedDataMutexID);
	assert(mutexStatus == osOK);

	return currentSpeed;
}


/*
 *
 */
void getVehicleDirection(vehicleDirection* currentDirection, vehicleDirection* previousDirection)
{
	// Acquire the vehicle direction data mutex before trying to
	// read vehicle direction data
	osStatus_t mutexStatus = osMutexAcquire(vehicleDirDataMutexID, osWaitForever);
	assert(mutexStatus == osOK);

	// Update directional data
	*previousDirection = *currentDirection;
	*currentDirection = directionData.direction;

	// Done reading vehicle direction data; release the vehicle
	// direction data mutex
	mutexStatus = osMutexRelease(vehicleDirDataMutexID);
	assert(mutexStatus == osOK);
}


/*
 *
 */
void checkForVehicleSpeedViolation(uint8_t currentSpeed, vehicleDirection currentDirection)
{
	/* Speed Violation – Light LED3 (green) for the following warnings:
	 * - Over limit, regardless of direction. Suggested limit: 75 mph.
	 * - Over limit, when making a turn. Suggested limit: 45 mph. */
	if((currentSpeed > 75) || ((currentSpeed > 45) && (currentDirection != drivingStraight)))
	{
		uint32_t flags = osEventFlagsSet(ledOutputEventFlagID, activateSpeedAlertEventFlag);
		assert(flags & ledOutputEventAllFlags);
	}
	else
	{
		uint32_t flags = osEventFlagsSet(ledOutputEventFlagID, deactivateSpeedAlertEventFlag);
		assert(flags & ledOutputEventAllFlags);
	}
}



/*
 *
 */
void checkForVehicleDirectionViolation(vehicleDirection previousDirection, vehicleDirection currentDirection)
{
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
		uint32_t flags = osEventFlagsSet(ledOutputEventFlagID, deactivateDirAlertEventFlag);
		assert(flags & ledOutputEventAllFlags);

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
				osStatus_t stopStatus = osTimerStop(directionAlertTimerID);
				assert(stopStatus == osOK);
			}
		}
		// While the vehicle did change direction, it is still in the process
		// of taking a turn. Therefore, restart the direction alert timer
		else
		{
			// NOTE: If the callback is called (after 5 seconds), then the direction
			// 		 alert event flag is raised and the proper LED is turned on via
			//		 the LED Output Task.

			/* Direction Violation – Light LED4 (red) for the following warnings:
			 * - Potential collision alert if constantly turning for more
			 *   than a predefined time limit. Suggested limit: 5 seconds.
			 * - Treat both gradual and hard turns as the same direction.	*/
			if(osTimerIsRunning(directionAlertTimerID))
			{
				osStatus_t stopStatus = osTimerStop(directionAlertTimerID);
				assert(stopStatus == osOK);
			}

			osStatus_t startStatus = osTimerStart(directionAlertTimerID, DIRECTION_ALERT_TIMER_TICKS);
			assert(startStatus == osOK);
		}
	}
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
	char* dirText = "";

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
			break;
	}

	LCD_Clear(0,LCD_COLOR_CYAN);
	LCD_SetTextColor(LCD_COLOR_BLACK);
	LCD_SetFont(&Font16x24);
	LCD_DisplayString(10, 130, speedText);
	LCD_DisplayNumber(100, 130, speed);
	LCD_DisplayString(140, 130, "MPH");
	LCD_DisplayString(10, 180, dirText);
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
		// Acquire the button state sempahore to wake up the task
		osStatus_t semaphoreStatus = osSemaphoreAcquire(buttonStateSemaphorID, osWaitForever);
		assert(semaphoreStatus == osOK);

		if(!osTimerIsRunning(holdButtonTimerID))
		{
			// Start the oneshot hold button timer
			osStatus_t status = osTimerStart(holdButtonTimerID, HOLD_BTN_TIMER_TICKS_1S);
			assert(status == osOK);
		}

		// Pend on the button state semaphore (wait for the button to be released)
		semaphoreStatus = osSemaphoreAcquire(buttonStateSemaphorID, osWaitForever);
		assert(semaphoreStatus == osOK);

		// Stop the timer if it hasn't yet expired
		if(osTimerIsRunning(holdButtonTimerID) == true)
		{
			osStatus_t status = osTimerStop(holdButtonTimerID);
			assert(status == osOK);
		}

		// Since the button was pressed, accelerate by 5
        int acceleration = 5;

        // If the button was held sufficiently long (1 second), flip the sign
        // of acceleration (from 5 to -5) to signal deceleration
        if(buttonHeld == true) { buttonHeld = false; acceleration *= -1; }

		updateVehicleSpeedData(acceleration);

		// Raise the speed update event flag to signal to the Vehicle Monitor
		// Task that the speed has been updated
		uint32_t flags = osEventFlagsSet(vehicleMonitorEventFlagID, speedUpdateEventFlag);

		// Verify that the event flag was set successfully
		assert(flags & speedUpdateEventFlag);
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

		updateVehicleDirectionData(newDirection);

		// Raise the Direction Update Flag to signal to the Vehicle
		// Monitor Task that new vehicle direction data is available
		uint32_t flags = osEventFlagsSet(vehicleMonitorEventFlagID, directionUpdateEventFlag);
		assert(flags & directionUpdateEventFlag);
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
		uint32_t eventStatus = osEventFlagsWait(vehicleMonitorEventFlagID, vehicleMonitorBothFlags,
												osFlagsNoClear, osWaitForever);
		assert(eventStatus & vehicleMonitorBothFlags);


		// Check if the speed update event flag is set
		if(eventStatus & speedUpdateEventFlag)
		{
			// Manually clear the speed update event flag
			uint32_t flags = osEventFlagsClear(vehicleMonitorEventFlagID, speedUpdateEventFlag);
			assert(flags & vehicleMonitorBothFlags);

			currentSpeed = getVehicleSpeed();
		}


		// Check if the direction update event flag is set
		if(eventStatus & directionUpdateEventFlag)
		{
			// Manually clear the direction update event flag
			uint32_t flags = osEventFlagsClear(vehicleMonitorEventFlagID, directionUpdateEventFlag);
			assert(flags & vehicleMonitorBothFlags);

			getVehicleDirection(&previousDirection, &currentDirection);
		}

		// Check for vehicle speed and/or direction violations
		checkForVehicleSpeedViolation(currentSpeed, currentDirection);
		checkForVehicleDirectionViolation(previousDirection, currentDirection);
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
		uint32_t flags = osEventFlagsWait(ledOutputEventFlagID, ledOutputEventAllFlags,
										  osFlagsNoClear, osWaitForever);

		assert(flags & ledOutputEventAllFlags);

		// Drive the LEDs according to which flag was set
		if(flags & activateSpeedAlertEventFlag)
		{
			flags = osEventFlagsClear(ledOutputEventFlagID, activateSpeedAlertEventFlag);
			HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
		}
		else if(flags & activateDirAlertEventFlag)
		{
			flags = osEventFlagsClear(ledOutputEventFlagID, activateDirAlertEventFlag);
			HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
		}
		else if(flags & deactivateSpeedAlertEventFlag)
		{
			flags = osEventFlagsClear(ledOutputEventFlagID, deactivateSpeedAlertEventFlag);
			HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
		}
		else if(flags & deactivateDirAlertEventFlag)
		{
			flags = osEventFlagsClear(ledOutputEventFlagID, deactivateDirAlertEventFlag);
			HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
		}

		assert(flags & ledOutputEventAllFlags);
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

	SEGGER_SYSVIEW_RecordEnterTimer((uint32_t)holdButtonTimerID);

	// Notify the speedSetpoint task that the button was held
	buttonHeld = true;

	SEGGER_SYSVIEW_RecordExitTimer();
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

	SEGGER_SYSVIEW_RecordEnterTimer((uint32_t)vehicleDirWakeupTimerID);

	osStatus_t status = osSemaphoreRelease(vehicleDirSemaphoreID);
	assert(status == osOK);

	SEGGER_SYSVIEW_RecordExitTimer();
}


/*
 * @brief Direction Alert Timer Callback
 *
 * @details Posts on the LED Output Event Flag in order
 * 			to signal that a direction violation has occured.
 *
 * 			Direction Violation – Light LED4 (red) for the following warnings:
 * 			- Potential collision alert if constantly turning
 * 			  for more than a predefined time limit.
 * 			- Both gradual and hard turns are treated as same direction.
 *
 * @param[in] arg Dummy parameter for use with osTimerNew()
 */
void directionAlertTimerCallback(void* arg)
{
	// Avoids a compiler warning for unused parameter
	(void) &arg;

	SEGGER_SYSVIEW_RecordEnterTimer((uint32_t)directionAlertTimerID);

	uint32_t flags = osEventFlagsSet(ledOutputEventFlagID, activateDirAlertEventFlag);
	assert(flags & ledOutputEventAllFlags);

	SEGGER_SYSVIEW_RecordExitTimer();

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

	SEGGER_SYSVIEW_RecordEnterTimer((uint32_t)lcdDisplayWakeupTimerID);

	osStatus_t status = osSemaphoreRelease(lcdDisplaySemaphoreID);
	assert(status == osOK);

	SEGGER_SYSVIEW_RecordEnterTimer((uint32_t)lcdDisplayWakeupTimerID);
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
	SEGGER_SYSVIEW_RecordEnterISR();

	// Clear interrupt flag
	__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_PIN);

	// POST on the button state sempahore
	osStatus_t status = osSemaphoreRelease(buttonStateSemaphorID);
	assert(status == osOK);


	SEGGER_SYSVIEW_RecordExitISR();

	// Re-enable interrupts
	HAL_NVIC_EnableIRQ(BUTTON_IRQn);
}

