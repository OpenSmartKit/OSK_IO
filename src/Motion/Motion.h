#ifndef MOTION_h
#define MOTION_h

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <functional>
#include <IO.h>

typedef std::function<void()> MotionHandlerFunction;

/*!
    \class Motion Motion.h <Motion.h>
    \brief Class to handle motion sensors. 

    Connect motion sensor to input pin and use this class
    for reliable handling of different scenarios.
*/
class Motion
{
public:
    /*!
		Motion constructor. Creates new Motion instance
		\param[in] pin Pin. For example OSK_IO1
	*/
    Motion(uint8_t pin);

    /*!
		Motion constructor. Creates new Motion instance and allow to set active state of pin
		\param[in] pin Pin. For example OSK_IO1
        \param[in] isActiveHigh Set false if active state of motion sensor is LOW
	*/
    Motion(uint8_t pin, boolean isActiveHigh);

    /*!
		Motion constructor. Creates new Motion instance
		\param[in] pin Pin. For example OSK_IO1
	*/
    ~Motion();

    /*!
		Handle motion detection event
		\param[in] fn Callback function
	*/
    void onCallback(MotionHandlerFunction fn);

    /*!
		Handle some time after stop motion detection 
		\param[in] fn Callback function
	*/
    void offCallback(MotionHandlerFunction fn);

    /*!
		Start motion detection
		\param[in] offDelay Delay after stop motion detection in seconds.
	*/
    void begin(int offDelay);

    /*!
		Set delay after stop motion detection
		\param[in] fn Callback function
	*/
    void changeOffDelay(int offDelay);

    /*!
		Current pin value
	*/
    uint8_t pinState = LOW;

private:
    static void _timerCallback(TimerHandle_t handle);
    void _onPinChange(uint8_t state);
    void _onTimerEnd();

    IO *_io = nullptr;
    TimerHandle_t _timer = nullptr;
    uint8_t _pin = 0;
    MotionHandlerFunction _onCallback = nullptr;
    MotionHandlerFunction _offCallback = nullptr;
    bool _isActiveHigh = true;
    bool _isActive = false;
};

#endif