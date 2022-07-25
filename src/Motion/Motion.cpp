#include <Motion\Motion.h>

Motion::Motion(uint8_t pin, boolean isActiveHigh)
{
  _io = IO::getInstance();
  _pin = pin;
  _isActiveHigh = isActiveHigh;
}

Motion::Motion(uint8_t pin)
{
  Motion(pin, true);
}

Motion::~Motion()
{
  _io->off(_pin);
}

void Motion::begin(int offDelay)
{
  if (_onCallback != nullptr || _offCallback != nullptr)
  {
    _timer = xTimerCreate("timer", pdMS_TO_TICKS(offDelay * 1000), pdFALSE, this, _timerCallback);
    _io->mode(_pin, INPUT);
    _io->on(_pin, CHANGE, [this](uint8_t state)
            { _onPinChange(state); });
    _onPinChange(_io->get(_pin));
  }
}

void Motion::onOn(MotionHandlerFunction fn)
{
  _onCallback = fn;
}

void Motion::onOff(MotionHandlerFunction fn)
{
  _offCallback = fn;
}

void Motion::_timerCallback(TimerHandle_t handle)
{
  Motion *p = static_cast<Motion *>(pvTimerGetTimerID(handle));
  p->_onTimerEnd();
}

void Motion::_onPinChange(uint8_t state)
{
  if (!_isActiveHigh)
  {
    state = state == LOW ? HIGH : LOW;
  }
  if (state == HIGH)
  {
    if (!_isActive)
    {
      _onCallback();
      _isActive = true;
    }
    if (xTimerIsTimerActive(_timer) != pdFALSE)
    {
      xTimerStop(_timer, 0);
    }
  }
  else if (_isActive)
  {
    xTimerStart(_timer, 0);
  }
}

void Motion::_onTimerEnd()
{
  xTimerStop(_timer, 0);
  _isActive = false;
  _offCallback();
}