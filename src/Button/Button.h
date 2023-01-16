#ifndef BUTTON_h
#define BUTTON_h

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <OSK_config.h>
#include <functional>
#include <IO.h>

#ifndef RELIABILITY_PERIOD
    #define RELIABILITY_PERIOD 60
#endif
#ifndef CLICK_PERIOD
    #define CLICK_PERIOD 100
#endif
#ifndef LONG_CLICK_PERIOD
    #define LONG_CLICK_PERIOD 1500
#endif

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

    /*!
		Button constructor. Creates new button instance.
		\param[in] pin Pin. For example OSK_IO1
        \param[in] reliabilityPeriod Delay in milliseconds to prevent unexpected triggers and remove noises
	*/
    Button(uint8_t pin, uint8_t reliabilityPeriod);

    ~Button();

    /*!
		Handle single click on button
		\param[in] fn Callback function
	*/
    void onClick(ButtonHandlerFunction fn);

    /*!
		Handle long click on button
		\param[in] fn Callback function
	*/
    void onLongClick(ButtonHandlerFunction fn);

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

    /*!
		Handle switch change position
		\param[in] fn Callback function
	*/
    void onChange(ButtonHandlerFunction fn);

    /*!
		Default button state: LOW or HIGH. For click functions only
	*/
    uint8_t defaultState = LOW;

protected:
    IO *_io = nullptr;
    uint8_t _pin = 0;

private:
    static void _changeCallback(TimerHandle_t handle);
    static void _clickCallback(TimerHandle_t handle);
    static void _longClickCallback(TimerHandle_t handle);
    void _onPinChange(uint8_t state);
    void _onTimerEnd();
    void _onChangeCallback();

    TimerHandle_t _longClickTimer = nullptr;
    TimerHandle_t _clickTimer = nullptr;
    TimerHandle_t _rTimer = nullptr;
    uint8_t _state = 0;
    uint8_t _isClickDone = 0;
    uint8_t _isLongClickDone = 0;
    ButtonHandlerFunction _click = nullptr;
    ButtonHandlerFunction _longClick = nullptr;
    ButtonHandlerFunction _onHigh = nullptr;
    ButtonHandlerFunction _onLow = nullptr;
    ButtonHandlerFunction _onChange = nullptr;
};

#endif