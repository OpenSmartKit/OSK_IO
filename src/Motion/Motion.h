#ifndef MOTION_h
#define MOTION_h

#define RELIABILITY_TIME 60
#define MOTION_X_BLOCK_TIME 100

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <functional>
#include <IO.h>
#include <Debug.h>

typedef std::function<void()> MotionHandlerFunction;

class Motion
{
public:
    Motion(uint8_t pin);
    Motion(uint8_t pin, boolean isActiveHigh);
    ~Motion();
    void onCallback(MotionHandlerFunction fn);
    void offCallback(MotionHandlerFunction fn);
    void begin(int offDelay);
    void changeOffDelay(int offDelay);
    uint8_t pinState = LOW;

private:
    static void _keepOnTimerCallback(TimerHandle_t handle);
    static void _reliabilityTimerCallback(TimerHandle_t handle);
    void _onPinChange(uint8_t state);
    void _onPinReliableChange();
    void _onKeepOnTimerEnd();

    IO *_io = nullptr;
    TimerHandle_t _keepOnTimer = nullptr;
    TimerHandle_t _reliabilityTimer = nullptr;
    uint8_t _pin = 0;
    MotionHandlerFunction _onCallback = nullptr;
    MotionHandlerFunction _offCallback = nullptr;
    bool _isActiveHigh = true;
    bool _isActive = false;
};

#endif