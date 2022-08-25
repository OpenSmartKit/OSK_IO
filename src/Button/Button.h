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

class Button
{
public:
    Button(uint8_t pin);
    ~Button();
    void click(ButtonHandlerFunction fn);
    void longClick(ButtonHandlerFunction fn);

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
};

#endif