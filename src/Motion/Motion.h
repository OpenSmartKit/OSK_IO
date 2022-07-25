#ifndef MOTION_h
#define MOTION_h

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <functional>
#include <IO\IO.h>

typedef std::function<void()> MotionHandlerFunction;

class Motion
{
public:
    Motion(uint8_t pin);
    Motion(uint8_t pin, boolean isActiveHigh);
    ~Motion();
    void onOn(MotionHandlerFunction fn);
    void onOff(MotionHandlerFunction fn);
    void begin(int offDelay);

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