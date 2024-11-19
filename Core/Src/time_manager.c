
#include "time_manager.h"

//variables
extern RTC_HandleTypeDef hrtc;//defined in main.c
static TimeManager timeManager;




void RTC_initialize(uint32_t alarmTime, uint8_t timerMode) {
    // Update the time manager with the provided current time and date
    HAL_RTC_GetTime(&hrtc, &timeManager._currentTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &timeManager._currentDate, RTC_FORMAT_BIN);

    if(timerMode == MODE_RUNNING){

		timeManager._setTime = alarmTime;  // Store the alarm offset (in seconds)

    }
    timeManager._timerMode = timerMode;    // Set the initial mode
}
/**
 * @brief Sets the current RTC time.
 * @param[in,out] time Pointer to the RTC_TimeTypeDef structure containing the time to set.
 */
void RTC_SetTime(RTC_TimeTypeDef *time) {
    HAL_RTC_SetTime(&hrtc, time, RTC_FORMAT_BIN);
    HAL_RTC_GetTime(&hrtc, &timeManager._currentTime, RTC_FORMAT_BIN);

}

/**
 * @brief Updates the alarm time based on the current time and set time.
 * This function recalculates the remaining time for the alarm based on the elapsed time and the current time.
 */
RTC_TimeTypeDef update(void) {
    HAL_RTC_GetTime(&hrtc, &timeManager._currentTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &timeManager._currentDate, RTC_FORMAT_BIN); // Necessary to read RTC correctly.

    // Compute remaining time for alarm
    timeManager._elapsedTime = timeManager._currentTime.Seconds +
                               timeManager._currentTime.Minutes * 60 +
                               timeManager._currentTime.Hours * 3600;

    uint32_t remainingTime = timeManager._setTime - timeManager._elapsedTime;

    timeManager._alarmTime.AlarmTime.Seconds = (timeManager._setTime - timeManager._elapsedTime) % 60;
    timeManager._alarmTime.AlarmTime.Minutes = ((timeManager._setTime - timeManager._elapsedTime) / 60) % 60;
    timeManager._alarmTime.AlarmTime.Hours = ((timeManager._setTime - timeManager._elapsedTime) / 3600);

    return timeManager._currentTime;
}
uint8_t getMode(void){
	return timeManager._timerMode;
}
/**
 * @brief Starts the timer by setting the RTC mode to running and triggering the alarm handler.
 */
int startstop(void) {
    if ((timeManager._timerMode == MODE_IDLE) ||(timeManager._timerMode == MODE_INITIALIZE)) {//IF IDLE THEN START
        HAL_RTC_GetTime(&hrtc, &timeManager._currentTime, RTC_FORMAT_BIN);
    	timeManager._startTime = timeManager._currentTime;
    	timeManager._timerMode = MODE_RUNNING;


		RTC_AlarmTypeDef sAlarm;
		uint32_t totalSeconds = timeManager._currentTime.Seconds +
								timeManager._currentTime.Minutes * 60 +
								timeManager._currentTime.Hours * 3600 + timeManager._setTime;
		// Set the alarm time using the provided RTC_AlarmTypeDef structure


		 // Calculate the new alarm time
		sAlarm.AlarmTime.Seconds = totalSeconds % 60;
		sAlarm.AlarmTime.Minutes = (totalSeconds / 60) % 60;
		sAlarm.AlarmTime.Hours = (totalSeconds / 3600) % 24;
		sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM; // Assuming 12-hour format; adjust as needed
		sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;

		sAlarm.Alarm = RTC_ALARM_A;

		// Update time manager settings
		timeManager._alarmTime = sAlarm; // Store calculated alarm time in time manager



		if (HAL_RTC_SetAlarm_IT(&hrtc, &timeManager._alarmTime, RTC_FORMAT_BIN) != HAL_OK) {
			RTC_ErrorHandler(); // Handle error if alarm setting fails
		}
       // RTC_SetMode(MODE_RUNNING);
        //RTC_Alarm_IRQHandler();
        //activate alarm.
    }else if (timeManager._timerMode == MODE_RUNNING) {//IF RUNNING, THEN PAUSE
    	//timeManager._timerMode = MODE_IDLE;
       // RTC_SetMode(MODE_IDLE);
        //HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
        //update();
    }
    return timeManager._timerMode;
}

/**
 * @brief Resets the elapsed time and alarm time to zero.
 */
void reset(void) {
    timeManager._elapsedTime = 0;
    timeManager._alarmTime.AlarmTime.Seconds = 0;
    timeManager._alarmTime.AlarmTime.Minutes = 0;
    timeManager._alarmTime.AlarmTime.Hours = 0;
}

/**
 * @brief Increments the alarm time by a specified amount.
 * @param hours Number of hours to increment.
 * @param minutes Number of minutes to increment.
 * @param seconds Number of seconds to increment.
 */
void increment(uint8_t hours, uint8_t minutes, uint8_t seconds) {
    timeManager._alarmTime.AlarmTime.Seconds += seconds;
    timeManager._alarmTime.AlarmTime.Minutes += minutes;
    timeManager._alarmTime.AlarmTime.Hours += hours;
    // Ensure proper RTC time format with carry-over
    if (timeManager._alarmTime.AlarmTime.Seconds >= 60) {
        timeManager._alarmTime.AlarmTime.Seconds -= 60;
        timeManager._alarmTime.AlarmTime.Minutes += 1;
    }
    if (timeManager._alarmTime.AlarmTime.Minutes >= 60) {
        timeManager._alarmTime.AlarmTime.Minutes -= 60;
        timeManager._alarmTime.AlarmTime.Hours += 1;
    }
    if (timeManager._alarmTime.AlarmTime.Hours >= 24) {
        timeManager._alarmTime.AlarmTime.Hours -= 24;
    }
}

/**
 * @brief Sets the RTC mode of the stopwatch and manages transitions between modes.
 * @param newMode The new mode to set (IDLE, RUNNING, CONFIGURE, ERROR).
 */
void RTC_SetMode(uint8_t newMode) {
    timeManager._timerMode = newMode;
    switch (newMode) {
        case MODE_RUNNING:
            startstop();
            break;
        case MODE_IDLE:
            startstop();
            break;
        case MODE_INITIALIZE:
            // Logic for configuration mode
            break;
        case MODE_ERROR:
            RTC_ErrorHandler();
            break;
        default:
            break;
    }
}

/**
 * @brief RTC alarm interrupt handler.
 * This function handles the RTC alarm interrupt and updates the elapsed time if the stopwatch is running.
 */
//void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
//	RTC_AlarmTypeDef sAlarm;
//	HAL_RTC_GetAlarm(hrtc,&sAlarm,RTC_ALARM_A,FORMAT_BIN);
//	if(sAlarm.AlarmTime.Seconds>58) {
//		sAlarm.AlarmTime.Seconds=0;
//	}else{
//		sAlarm.AlarmTime.Seconds=sAlarm.AlarmTime.Seconds+1;
//	}
//    while(HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN)!=HAL_OK){}
    //HAL_GPIO_TogglePin(Indication_light_GPIO_Port, Indication_light_Pin);
//}
/**
 * @brief Handles errors related to RTC operations.
 * This function switches the mode to error and enters an infinite loop to prevent further operations.
 */
void RTC_ErrorHandler(void) {
    // Handle RTC errors, switch to error mode if needed
    timeManager._timerMode = MODE_ERROR;
    while (1) {
        // Error state, potential reset or handling logic here
    }
}

