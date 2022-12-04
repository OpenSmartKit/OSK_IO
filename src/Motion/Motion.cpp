#include <Motion.h>

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
    _keepOnTimer = xTimerCreate("motion", pdMS_TO_TICKS(offDelay * 1000), pdFALSE, this, _keepOnTimerCallback);
    _reliabilityTimer = xTimerCreate("mr", pdMS_TO_TICKS(RELIABILITY_TIME), pdFALSE, this, _reliabilityTimerCallback);
    _io->mode(_pin, INPUT);
    _io->on(_pin, CHANGE, [this](uint8_t state)
            { _onPinChange(state); });
    _onPinReliableChange();
  }
}

void Motion::onCallback(MotionHandlerFunction fn)
{
  _onCallback = fn;
}

void Motion::offCallback(MotionHandlerFunction fn)
{
  _offCallback = fn;
}

void Motion::changeOffDelay(int offDelay)
{
  bool isTimerActive = false;
  if (xTimerIsTimerActive(_keepOnTimer)) {
    isTimerActive = true;
  }
  xTimerChangePeriod(_keepOnTimer, pdMS_TO_TICKS(offDelay * 1000), 100);
  if (!isTimerActive) {
    xTimerStop(_keepOnTimer, MOTION_X_BLOCK_TIME);
  }
}

void Motion::_keepOnTimerCallback(TimerHandle_t handle)
{
  Motion *p = static_cast<Motion *>(pvTimerGetTimerID(handle));
  p->_onKeepOnTimerEnd();
}

void Motion::_onPinChange(uint8_t state)
{
  if (state == pinState && xTimerIsTimerActive(_reliabilityTimer) != pdFALSE) {
    xTimerStop(_reliabilityTimer, MOTION_X_BLOCK_TIME);
  } else if (xTimerIsTimerActive(_reliabilityTimer) == pdFALSE) {
    xTimerStart(_reliabilityTimer, MOTION_X_BLOCK_TIME);
  }
}

void Motion::_reliabilityTimerCallback(TimerHandle_t handle)
{
  Motion *p = static_cast<Motion *>(pvTimerGetTimerID(handle));
  p->_onPinReliableChange();
}

void Motion::_onPinReliableChange()
{
  pinState = _io->get(_pin);
  //DEBUG_MSG("Pin changed: ");
  //DEBUG_MSG(pinState == LOW ? "Pin is LOW" : "Pin is HIGH");
  //DEBUG_MSG_NL(_isActive ? "Is Active" : "Not Active");
  if (!_isActiveHigh)
  {
    pinState = pinState == LOW ? HIGH : LOW;
  }
  if (pinState == HIGH)
  {
    if (!_isActive)
    {
      _onCallback();
      _isActive = true;
    }
    if (xTimerIsTimerActive(_keepOnTimer) != pdFALSE)
    {
      //DEBUG_MSG_NL("Motion stop timer.");
      xTimerStop(_keepOnTimer, MOTION_X_BLOCK_TIME);
    }
  }
  else if (_isActive)
  {
    //DEBUG_MSG_NL("Motion start timer.");
    xTimerStart(_keepOnTimer, MOTION_X_BLOCK_TIME);
  }
}

void Motion::_onKeepOnTimerEnd()
{
  //DEBUG_MSG_NL("Motion end timer.");
  xTimerStop(_keepOnTimer, MOTION_X_BLOCK_TIME);
  _isActive = false;
  _offCallback();
}