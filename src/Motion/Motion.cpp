#include <Motion.h>

Motion::Motion(uint8_t pin, boolean isActiveHigh) : Button(pin)
{
  _isActiveHigh = isActiveHigh;
}

Motion::Motion(uint8_t pin) : Button(pin)
{
  Motion(pin, true);
}

void Motion::begin(int offDelay)
{
  if (_onCallback != nullptr || _offCallback != nullptr)
  {
    _keepOnTimer = xTimerCreate("motion", pdMS_TO_TICKS(offDelay * 1000), pdFALSE, this, _keepOnTimerCallback);
    onChange([this]()
            { _onPinChange(); });
    _onPinChange();
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

void Motion::_onPinChange()
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

void Motion::debug(String debugContext)
{
  #if OSK_DEBUG_ON
    if (debugContext.length() == 0) {
      debugContext = _debugContext;
    }
    if (debugContext.length() > 0) {
      bool currentPinState = _io->get(_pin);
      DBG("Motion: %s; Pin is : %s; Light is: %s.", debugContext, currentPinState == LOW ? "LOW" : "HIGH", _isActive ? "ON" : "OFF");
    }
	#endif 
}

void Motion::startDebug(String debugContext)
{
  _debugContext = debugContext;
}
