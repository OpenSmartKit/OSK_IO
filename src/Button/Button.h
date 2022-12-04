#ifndef BUTTON_h
#define BUTTON_h

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <functional>
#include <IO.h>

#define CLICK_PERIOD 150
#define LONG_CLICK_PERIOD 1500

typedef std::function<void()> ButtonHandlerFunction;

/*!
    \class Button Button.h <Button.h>
    \brief Class to handle buttons actions. 

    Connect buttons or switches to input pin and use this class
    for reliable handling of different scenarios.
*/
class Button
{
public:
    /*!
		Button constructor. Creates new button instance.
		\param[in] pin Pin. For example OSK_IO1
	*/
    Button(uint8_t pin);

    ~Button();

    /*!
		Handle single click on button
		\param[in] fn Callback function
	*/
    void click(ButtonHandlerFunction fn);

    /*!
		Handle long click on button
		\param[in] fn Callback function
	*/
    void longClick(ButtonHandlerFunction fn);
    /*!
		Handle switch on HIGH position
		\param[in] fn Callback function
	*/
    void onHigh(ButtonHandlerFunction fn);

    /*!
		Handle switch on LOW position
		\param[in] fn Callback function
	*/
    void onLow(ButtonHandlerFunction fn);

private:
    static void _callback(TimerHandle_t handle);
    void _onPinChange(uint8_t state);
    void _onTimerEnd();

    IO *_io = nullptr;
    TimerHandle_t _timer = nullptr;
    uint8_t _pin = 0;
    uint8_t _state = 0;
    uint64_t _timeStart = 0;
    ButtonHandlerFunction _click = nullptr;
    ButtonHandlerFunction _longClick = nullptr;
    ButtonHandlerFunction _onHigh = nullptr;
    ButtonHandlerFunction _onLow = nullptr;
};

#endif