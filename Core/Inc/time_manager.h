

/*Chat GPT prompt
 *
 * This is a time class used to represent time for my stopwatch project.
 *
 * I will be using the RTC on the STM32WB55 as my main way of keeping track of time.
 * The way this RTC will be used is to keep track of current time like a clock.
 * A feature that will also be implemented is a pomodoro timer, which will run in conjunction with the RTC
 * Where this alarm will be triggered 30mins after RTC time for example.
 *
 * There will be 3 modes for my project, and time must account for it accordingly
 	 	const int MODE_IDLE = 0;
		const int MODE_RUNNING = 1;
		const int MODE_CONFIGURE = 2;
		const int MODE_ERROR = 3;


Here are some ideas for
	Variables:
		"_currentTime" - RTC data type - for general time keeping
		"_alarmTime" - RTC data type for when the next alarm will be activated
		"_setTime" - time set for the final alarm
		"_elapsedTime" - RTC time where  _elapsedTime - _currentTime
	Functions:
		"update" - where we calculate alarmTime = _setTime-(_elapsedTime - _currentTime)
		"start/stop" - used to start the timer by setting an alarm at _currentTime
					 - or to stop the timer where
					 	 1. use "update" function
					 	 2. disable the alarm
					 	 3. Enter configuration mode
		"reset" - reset the _alarmTime
		"increment(hours,minutes,seconds)" - used to increase the _alarmTime by h,m,s



 * Follow my series of instructions sequentially while maintaining accuracy and simplicity, and backup all decisions with evidence for me to look at.
 * Spread out into several prompts if needed to ensure quality responses
 * 1. Contemplate the specific desires and needs given information given for this c and h file
 * 2. Plan out specific requirements that emphasises the ability to easily implement the class required
 * 3. Using the implementation structure you just created. Implement it
 *
 *
 *
 * */


#ifndef TIME_MANAGER_H
#define TIME_MANAGER_H

#include "stm32wbxx_hal.h"


// Mode definitions
#define MODE_IDLE       0
#define MODE_RUNNING    1
#define MODE_INITIALIZE  2
#define MODE_ERROR      3

// TimeManager class for stopwatch functionality
typedef struct {
    RTC_TimeTypeDef _currentTime;   // Holds the current RTC time
    RTC_DateTypeDef _currentDate;   // Holds the current RTC date
    RTC_AlarmTypeDef _alarmTime;    // Configured RTC alarm time
    RTC_TimeTypeDef _startTime;     // Time when the alarm was initially set
    RTC_DateTypeDef _startDate;     // Date when the alarm was initially set
    uint8_t _timerMode;                   // Current mode of the stopwatch
    uint32_t _setTime;              // Time initially set (in seconds)
    uint32_t _remainingTime;        // Time remaining until alarm (in seconds)
    uint32_t _elapsedTime;          // Elapsed time since the start (in seconds)
} TimeManager;



// Function prototypes
/**
 * @brief This function is called every pomodoro session
 * Initializes time, mode = idle. Start time, ect.
 * @param[in] alarmTime, this is the amount of time that'll occur until alarm ends.E.G 10 seconds.
 * 			An initial alarm will be set to x seconds away from current time.
 * @param[in] timerMode
 *
 */
void RTC_initialize(uint32_t alarmTime, uint8_t timerMode);
RTC_TimeTypeDef update(void);
int startstop(void);
void reset(void);
void increment(uint8_t hours, uint8_t minutes, uint8_t seconds);
void RTC_SetMode(uint8_t newMode);
void RTC_ErrorHandler(void);
uint8_t getMode(void);
#endif // TIME_MANAGER_H

