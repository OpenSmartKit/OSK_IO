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

uint8_t Button::getState()
{
  return _state;
}

void Button::_changeCallback(TimerHandle_t handle)
{
  Button *p = static_cast<Button *>(pvTimerGetTimerID(handle));
  p->_onChangeCallback();
}

void Button::_onChangeCallback()
{
  _state = _io->get(_pin);

  // Handle switch change event
  if (_onChange) {
    _onChange();
  }

  // Handle switch OFF event
  if (_onHigh && _state == HIGH) {
    _onHigh();
  }

  // Handle switch ON event
  if (_onLow && _state == LOW) {
    _onLow();
  }

  // Handle button click event
  if (_click) {
    if (_state != defaultState && xTimerIsTimerActive(_clickTimer) == pdFALSE && _isClickDone == 0)
    {
      xTimerStart(_clickTimer, 0);
    } else if (_state == defaultState && xTimerIsTimerActive(_clickTimer) != pdFALSE)
    {
      xTimerStop(_clickTimer, 0);
    }

    // call click callback
    if (_isClickDone == 1 && _state == defaultState && _isLongClickDone == 0)
    {
      _click();
      _isClickDone = 0;
    }
  }

  // Handle button long click event
  if (_longClick) {
    if (_state != defaultState && xTimerIsTimerActive(_longClickTimer) == pdFALSE && _isLongClickDone == 0)
    {
      xTimerStart(_longClickTimer, 0);
    } else if (_state == defaultState && xTimerIsTimerActive(_longClickTimer) != pdFALSE)
    {
      xTimerStop(_longClickTimer, 0);
    }

    // call long click callback
    if (_isLongClickDone == 1 && _state == defaultState)
    {
      _longClick();
      _isLongClickDone = 0;
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
