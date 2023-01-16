#ifndef MOTION_h
#define MOTION_h

#define RELIABILITY_TIME 60
#define MOTION_X_BLOCK_TIME 100

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <OSK_config.h>
#include <functional>
#include <Button.h>
#include <Debug.h>

typedef std::function<void()> MotionHandlerFunction;

/*!
  \class Motion Motion.h <Motion.h>
  \brief Class to handle motion sensors. 

  Connect motion sensor to input pin and use this class
  for reliable handling of different scenarios.
*/
class Motion : protected Button
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
      Debug output. Current state
    */
    void debug(String debugContext = "");

    /*!
      Set debug context to differentiate debug outputs for different instances
      Is it was set the debug will output the data on every state change
    */
    void startDebug(String debugContext);

    /*!
		  Current pin value
	  */
    uint8_t pinState = LOW;

private:
    static void _keepOnTimerCallback(TimerHandle_t handle);
    void _onPinChange();
    void _onKeepOnTimerEnd();

    TimerHandle_t _keepOnTimer = nullptr;
    MotionHandlerFunction _onCallback = nullptr;
    MotionHandlerFunction _offCallback = nullptr;
    bool _isActiveHigh = true;
    bool _isActive = false;
    String _debugContext = "";
};

#endif