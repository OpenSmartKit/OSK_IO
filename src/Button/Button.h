#ifndef BUTTON_h
#define BUTTON_h

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <functional>
#include <IO.h>

#define RELIABILITY_PERIOD 60
#define CLICK_PERIOD 100
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

private:
    static void _changeCallback(TimerHandle_t handle);
    static void _clickCallback(TimerHandle_t handle);
    static void _longClickCallback(TimerHandle_t handle);
    void _onPinChange(uint8_t state);
    void _onTimerEnd();

    IO *_io = nullptr;
    TimerHandle_t _longClickTimer = nullptr;
    TimerHandle_t _clickTimer = nullptr;
    TimerHandle_t _rTimer = nullptr;
    uint8_t _pin = 0;
    uint8_t _state = 0;
    uint8_t _clickEnoughTime = 0;
    uint8_t _longClickEnoughTime = 0;
    ButtonHandlerFunction _click = nullptr;
    ButtonHandlerFunction _longClick = nullptr;
    ButtonHandlerFunction _onHigh = nullptr;
    ButtonHandlerFunction _onLow = nullptr;
    ButtonHandlerFunction _onChange = nullptr;
};

#endif