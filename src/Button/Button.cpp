#include <Button/Button.h>

Button::Button(uint8_t pin)
{
  _io = IO::getInstance();
  _pin = pin;
  _timer = xTimerCreate("timer", pdMS_TO_TICKS(LONG_CLICK_PERIOD), pdFALSE, this, _callback);
  _io->mode(_pin, INPUT);
  _state = _io->get(_pin);
  _io->on(_pin, CHANGE, [this](uint8_t state)
          { _onPinChange(state); });
}

Button::~Button()
{
  _io->off(_pin);
}

void Button::click(ButtonHandlerFunction fn)
{
  _click = fn;
}

void Button::longClick(ButtonHandlerFunction fn)
{
  _longClick = fn;
}

void Button::_callback(TimerHandle_t handle)
{
  Button *p = static_cast<Button *>(pvTimerGetTimerID(handle));
  p->_onTimerEnd();
}

void Button::_onPinChange(uint8_t state)
{
  if (_timeStart == 0)
  {
    if (state == LOW)
    {
      _timeStart = millis();
      _state = state;
      xTimerStart(_timer, 0);
    }
    return;
  }
  else
  {
    if (_state == state)
      return;
    else
    {
      uint64_t period = millis() - _timeStart;
      if (period >= CLICK_PERIOD && period < LONG_CLICK_PERIOD)
      {
        if (_click)
          _click();
      }
      xTimerStop(_timer, 0);
      _state = state;
      _timeStart = 0;
    }
  }
}

void Button::_onTimerEnd()
{
  if (_longClick)
    _longClick();
}