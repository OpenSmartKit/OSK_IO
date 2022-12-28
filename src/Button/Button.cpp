#include <Button.h>

Button::Button(uint8_t pin)
{
  _io = IO::getInstance();
  _pin = pin;
  _rTimer = xTimerCreate("rTimer", pdMS_TO_TICKS(RELIABILITY_PERIOD), pdFALSE, this, _changeCallback);
  _io->mode(_pin, INPUT);
  _state = _io->get(_pin);
  defaultState = _state;
  _io->on(_pin, CHANGE, [this](uint8_t state)
          { _onPinChange(state); });
}

Button::~Button()
{
  _io->off(_pin);
}

void Button::onHigh(ButtonHandlerFunction fn)
{
  _onHigh = fn;
}

void Button::onLow(ButtonHandlerFunction fn)
{
  _onLow = fn;
}

void Button::onChange(ButtonHandlerFunction fn)
{
  _onChange = fn;
}

void Button::onClick(ButtonHandlerFunction fn)
{
  if (!_clickTimer) {
    _clickTimer = xTimerCreate("clickTimer", pdMS_TO_TICKS(CLICK_PERIOD), pdFALSE, this, _clickCallback);
  }
  _click = fn;
}

void Button::onLongClick(ButtonHandlerFunction fn)
{
  if (!_longClickTimer) {
    _longClickTimer = xTimerCreate("longClickTimer", pdMS_TO_TICKS(LONG_CLICK_PERIOD), pdFALSE, this, _longClickCallback);
  }
  _longClick = fn;
}

void Button::_changeCallback(TimerHandle_t handle)
{
  Button *p = static_cast<Button *>(pvTimerGetTimerID(handle));
  p->_state = p->_io->get(p->_pin);

  // Handle switch change event
  if (p->_onChange) {
    p->_onChange();
  }

  // Handle switch OFF event
  if (p->_onHigh && p->_state == HIGH) {
    p->_onHigh();
  }

  // Handle switch ON event
  if (p->_onLow && p->_state == LOW) {
    p->_onLow();
  }

  // Handle button click event
  if (p->_click) {
    if (p->_state != p->defaultState && xTimerIsTimerActive(p->_clickTimer) == pdFALSE && p->_isClickDone == 0)
    {
      xTimerStart(p->_clickTimer, 0);
    } else if (p->_state == p->defaultState && xTimerIsTimerActive(p->_clickTimer) != pdFALSE)
    {
      xTimerStop(p->_clickTimer, 0);
    }

    // call click callback
    if (p->_isClickDone == 1 && p->_state == p->defaultState && p->_isLongClickDone == 0)
    {
      p->_click();
      p->_isClickDone = 0;
    }
  }

  // Handle button long click event
  if (p->_longClick) {
    if (p->_state != p->defaultState && xTimerIsTimerActive(p->_longClickTimer) == pdFALSE && p->_isLongClickDone == 0)
    {
      xTimerStart(p->_longClickTimer, 0);
    } else if (p->_state == p->defaultState && xTimerIsTimerActive(p->_longClickTimer) != pdFALSE)
    {
      xTimerStop(p->_longClickTimer, 0);
    }

    // call long click callback
    if (p->_isLongClickDone == 1 && p->_state == p->defaultState)
    {
      p->_longClick();
      p->_isLongClickDone = 0;
    }
  }
}

void Button::_longClickCallback(TimerHandle_t handle)
{
  Button *p = static_cast<Button *>(pvTimerGetTimerID(handle));
  p->_isLongClickDone = 1;
}

void Button::_clickCallback(TimerHandle_t handle)
{
  Button *p = static_cast<Button *>(pvTimerGetTimerID(handle));
  p->_isClickDone = 1;
}

void Button::_onPinChange(uint8_t state)
{
  if (state != _state && xTimerIsTimerActive(_rTimer) == pdFALSE)
  {
    xTimerStart(_rTimer, 0);
  }
  else if (state == _state && xTimerIsTimerActive(_rTimer) != pdFALSE)
  {
    xTimerStop(_rTimer, 0);
  }
}
