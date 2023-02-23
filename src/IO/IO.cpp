#include <IO.h>

IO *IO::_instance = nullptr;

IO *IO::getInstance()
{
	if (!_instance)
		_instance = new IO();

	return _instance;
}

IO::IO()
{
	_exp = new PCF8574(IO_PCF8574_ADDR);

#ifdef MAIN_PCA9685_ADDR
	_led = new PCA9685(MAIN_PCA9685_ADDR);
#endif
#ifdef RELAY_PCA9685_ADDR
	_relay = new PCA9685(RELAY_PCA9685_ADDR);
#endif

	begin();
}

IO::~IO()
{
	detachInterrupt(digitalPinToInterrupt(IO_INT_PIN));
	delete _exp;
#ifdef MAIN_PCA9685_ADDR
	for (uint8_t i = 0; i < OSK_DC_COUNT; i++)
	{
		xTimerDelete(_ledTimer[i], portMAX_DELAY);
	}
	vTaskDelete(_ledTask);
	vQueueDelete(_ledQueue);
	delete _led;
#endif
#ifdef RELAY_PCA9685_ADDR
	delete _relay;
#endif

	vTaskDelete(_task);
	vQueueDelete(_queue);
}

void IO::begin()
{
	_queue = xQueueCreate(64, sizeof(uint8_t));
	xTaskCreatePinnedToCore(this->taskHandler, "task", TASK_STACK, this, TASK_PRIORITY, &_task, TASK_CORE);

	for (uint8_t i = 0; i < sizeof(_subscribers) / sizeof(_subscribers[0]); i++)
	{
		_subscribers[i] = {0};
	}

	pinMode(IO_INT_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(IO_INT_PIN), expIsr, FALLING);

	_exp->begin();
	_exp->setButtonMask(0b11111111);
	_expPinsState = _exp->readButton8();

	if (_exp->isConnected())
	{
		DBG("IO Expander PCF8574 is OK on address: %X", IO_PCF8574_ADDR);
	} else {
		DBG("IO Expander PCF8574 ERROR on address: %X", IO_PCF8574_ADDR);
	}

	ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);

#ifdef MAIN_PCA9685_ADDR
	_led->resetDevices();
	_led->init();
	_led->setPWMFrequency(PWM_FREQUENCY);

	_ledQueue = xQueueCreate(24, sizeof(uint8_t));
	xTaskCreatePinnedToCore(this->_ledTaskHandler, "ledTask", TASK_STACK, this, TASK_PRIORITY, &_ledTask, TASK_CORE);
	for (uint8_t i = 0; i < OSK_DC_COUNT; i++)
	{
		_ledConfig[i] = {i, 0, 0, 0};
		_ledTimer[i] = xTimerCreate("t", pdMS_TO_TICKS(1000), pdTRUE, &_ledConfig[i].id, _ledTimerHandle);
	}
#endif
#ifdef RELAY_PCA9685_ADDR
	_relay->resetDevices();
	_relay->init();
#endif
}

void IO::mode(uint16_t input, uint8_t mode)
{
	if (mode != INPUT && mode != OUTPUT)
		return;

	if (_isNativePort(input))
	{
		pinMode(input, mode);
	}
	else
	{
		uint8_t io = _getExpanderPort(input);
		if (io == INCORRECT_PIN)
			return;

		uint8_t mask = _exp->getButtonMask();
		if (mode == INPUT)
			mask |= 1UL << io;
		else
			mask &= ~(1UL << io);
		_exp->setButtonMask(mask);
	}
}

bool IO::get(uint16_t input)
{
	if (_isNativePort(input))
		return digitalRead(input);

	uint8_t io = _getExpanderPort(input);
	if (io != INCORRECT_PIN)
		return _exp->read(io);

	return false;
}

void IO::set(uint16_t input, bool state)
{
	if (_isNativePort(input))
	{
		digitalWrite(input, state);
	}
	else
	{
		uint8_t port = _getExpanderPort(input);
		if (port != INCORRECT_PIN)
			_exp->write(port, state);

#ifdef MAIN_PCA9685_ADDR
		uint8_t channel = _getLedChannel(input);
		if (channel != INCORRECT_PIN)
		{
			if (xTimerIsTimerActive(_ledTimer[channel]))
			{
				xTimerStop(_ledTimer[channel], portMAX_DELAY);
				_ledConfig[channel].state = 0;
			}

			if (state)
				_led->setChannelOn(channel);
			else
				_led->setChannelOff(channel);
		}
#endif
#ifdef RELAY_PCA9685_ADDR
		uint8_t relay = _getRelayChannel(input);
		if (relay != INCORRECT_PIN)
		{
			if (state)
				_relay->setChannelOn(relay);
			else
				_relay->setChannelOff(relay);
		}
#endif
	}
}

void IO::pwmWrite(uint16_t input, uint16_t value)
{
	if (_isNativePort(input))
	{
		ledcAttachPin(input, PWM_CHANNEL);
		ledcWrite(PWM_CHANNEL, value);
	}
	else
	{
#ifdef MAIN_PCA9685_ADDR
		uint8_t channel = _getLedChannel(input);
		if (channel == INCORRECT_PIN)
			return;

		if (xTimerIsTimerActive(_ledTimer[channel]))
		{
			xTimerStop(_ledTimer[channel], portMAX_DELAY);
			_ledConfig[channel].state = 0;
		}
		_led->setChannelPWM(channel, value);
#endif
	}
}

uint16_t IO::pwmRead(uint16_t input)
{
	if (_isNativePort(input))
	{
		return ledcRead(input);
	}
	else
	{
#ifdef MAIN_PCA9685_ADDR
		uint8_t channel = _getLedChannel(input);
		if (channel == INCORRECT_PIN)
			return 0;

		return _led->getChannelPWM(channel);
#endif
	}
	return 0;
}

void IO::on(uint16_t pin, int mode, PinChangeHandlerFunction method)
{
	if (_isNativePort(pin))
	{
		uint8_t isrPin = digitalPinToInterrupt(pin);
		switch (pin)
		{
		case OSK_IO1:
			attachInterrupt(isrPin, inp1Isr, mode);
			break;
		case OSK_IO2:
			attachInterrupt(isrPin, inp2Isr, mode);
			break;
		case OSK_IO3:
			attachInterrupt(isrPin, inp3Isr, mode);
			break;
		case OSK_IO4:
			attachInterrupt(isrPin, inp4Isr, mode);
			break;
		case OSK_IO5:
			attachInterrupt(isrPin, inp5Isr, mode);
			break;
		case OSK_IO6:
			attachInterrupt(isrPin, inp6Isr, mode);
			break;
		case OSK_IO7:
			attachInterrupt(isrPin, inp7Isr, mode);
			break;
		case OSK_IO8:
			attachInterrupt(isrPin, inp8Isr, mode);
			break;
		case OSK_IO9:
			attachInterrupt(isrPin, inp9Isr, mode);
			break;
		}

		uint8_t value = digitalRead(pin);
		uint8_t index = _getExpanderIndexByInput(pin);
		_prevValues[index] = value;
	}
	else
	{
		uint8_t io = _getExpanderPort(pin);
		if (io == INCORRECT_PIN)
			return;

		uint8_t state = _exp->readButton(io);
		if (state)
			_expPinsState |= 1UL << io;
		else
			_expPinsState &= ~(1UL << io);
	}

	_subscribers[_getExpanderIndexByInput(pin)] = {1, method, mode};
}

void IO::off(uint16_t pin)
{
	if (_isNativePort(pin))
	{
		uint8_t isrPin = digitalPinToInterrupt(pin);
		switch (pin)
		{
		case 1:
			detachInterrupt(isrPin);
			break;
		case 2:
			detachInterrupt(isrPin);
			break;
		case 3:
			detachInterrupt(isrPin);
			break;
		case 4:
			detachInterrupt(isrPin);
			break;
		case 5:
			detachInterrupt(isrPin);
			break;
		case 6:
			detachInterrupt(isrPin);
			break;
		case 7:
			detachInterrupt(isrPin);
			break;
		case 8:
			detachInterrupt(isrPin);
			break;
		case 9:
			detachInterrupt(isrPin);
			break;
		}
	}
	else
	{
		uint8_t io = _getExpanderPort(pin);
		if (io == INCORRECT_PIN)
			return;
	}

	_subscribers[pin] = {0, nullptr};
}

void IO::taskHandler(void *pvParameters)
{
	IO *self = (IO *)pvParameters;
	uint8_t input;
	uint8_t io;
	EventSubscriber subscriber;

	for (;;)
	{
		if (xQueueReceive(self->_queue, &input, QUEUE_RECEIVE_DELAY))
		{
			if (input == EXPANDER_INPUT)
			{
				uint8_t states = self->_exp->readButton8();
				uint8_t newState, prevState;

				for (uint8_t i = 0; i <= 6; i++)
				{
					uint8_t pin = self->_getExpanderInput(i);
					subscriber = self->_subscribers[self->_getExpanderIndexByInput(pin)];
					if (!subscriber.active || !subscriber.fn)
						continue;

					newState = (states >> i) & 1;
					prevState = (self->_expPinsState >> i) & 1;

					if (prevState == newState)
						continue;

					if (subscriber.mode == CHANGE || (subscriber.mode == FALLING && newState == LOW) || (subscriber.mode == RISING && newState == HIGH))
						subscriber.fn(newState);
				}
				self->_expPinsState = states;
			}
			else
			{
				subscriber = self->_subscribers[self->_getExpanderIndexByInput(input)];
				if (!subscriber.active || !subscriber.fn)
					continue;

				subscriber.fn(digitalRead(input));
			}
		}
	}
}

void IO::expIsr()
{
	if (_instance)
		_instance->_expanderInterrupt();
}

void IO::inp1Isr()
{
	if (_instance)
		_instance->_inputInterrupt(OSK_IO1);
}
void IO::inp2Isr()
{
	if (_instance)
		_instance->_inputInterrupt(OSK_IO2);
}
void IO::inp3Isr()
{
	if (_instance)
		_instance->_inputInterrupt(OSK_IO3);
}
void IO::inp4Isr()
{
	if (_instance)
		_instance->_inputInterrupt(OSK_IO4);
}
void IO::inp5Isr()
{
	if (_instance)
		_instance->_inputInterrupt(OSK_IO5);
}
void IO::inp6Isr()
{
	if (_instance)
		_instance->_inputInterrupt(OSK_IO6);
}
void IO::inp7Isr()
{
	if (_instance)
		_instance->_inputInterrupt(OSK_IO7);
}
void IO::inp8Isr()
{
	if (_instance)
		_instance->_inputInterrupt(OSK_IO8);
}
void IO::inp9Isr()
{
	if (_instance)
		_instance->_inputInterrupt(OSK_IO9);
}

void IO::_inputInterrupt(uint16_t input)
{
	uint8_t value = digitalRead(input);
	uint8_t index = _getExpanderIndexByInput(input);
	
	if (_prevValues[index] != value) {
		_prevValues[index] = value;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR(_queue, &input, &xHigherPriorityTaskWoken);
	}
}

void IO::_expanderInterrupt()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint8_t input = EXPANDER_INPUT;
	xQueueSendFromISR(_queue, &input, &xHigherPriorityTaskWoken);
}

bool IO::_isNativePort(uint16_t pin)
{
	return pin < 100;
}

uint8_t IO::_getExpanderPort(uint16_t input)
{
	if (input >= 100 && input < 1000)
		return input - 100;
	else
		return INCORRECT_PIN;
}

#ifdef MAIN_PCA9685_ADDR
void IO::ledDim(uint16_t input, uint8_t value, uint16_t duration)
{
	if (value > 100)
		value = 100;
	
	uint8_t channel = _getLedChannel(input);
	if (channel == INCORRECT_PIN)
		return;

	uint8_t currentValue = ledRead(input);
	if (currentValue == value)
	{
		_ledConfig[channel].state = 0;
		return;
	}

	if (duration == 0)
	{
		pwmWrite(channel, pgm_read_word(&cie[value]));
		_ledConfig[channel].state = 0;
	}
	else
	{
		uint8_t stepsCount = abs(value - currentValue);
		uint16_t relativeDuration = (stepsCount / 100.0) * duration;
		uint16_t stepDuration = relativeDuration / stepsCount;
		if (stepDuration < 1)
			stepDuration = 1;
		
		_ledConfig[channel].state = value > currentValue ? 1 : 2;
		_ledConfig[channel].current = currentValue;
		_ledConfig[channel].target = value;
		
		xTimerChangePeriod(_ledTimer[channel], pdMS_TO_TICKS(stepDuration), portMAX_DELAY);
	}
}

uint8_t IO::ledRead(uint16_t input)
{
	uint16_t value = pwmRead(input);
	//TODO: Think about a more optimal way performance-wise (e.g., binary search)
	for (uint8_t i = 0; i < sizeof(cie)/sizeof(*cie); i++)
	{
		if (pgm_read_word(&cie[i]) >= value)
			return i;
	}
	return 0;
}

void IO::_ledTaskHandler(void *pvParameters)
{
	IO *self = (IO *)pvParameters;
	uint8_t channel;

	for (;;)
	{
		if (xQueueReceive(self->_ledQueue, &channel, QUEUE_RECEIVE_DELAY))
		{
			LedConfig cfg = self->_ledConfig[channel];
			uint8_t nextValue = cfg.state == 1 ? cfg.current + 1 : cfg.current - 1;

			if (nextValue == cfg.target)
			{
				xTimerStop(self->_ledTimer[channel], portMAX_DELAY);
				self->_ledConfig[channel].state = 0;
			}
			
			self->_led->setChannelPWM(channel, pgm_read_word(&cie[nextValue]));
			self->_ledConfig[channel].current = nextValue;
		}
	}
}

void IO::_ledTimerHandle(TimerHandle_t handle)
{
	uint8_t *channel = static_cast<uint8_t *>(pvTimerGetTimerID(handle));
	IO *self = IO::getInstance();

	xQueueSend(self->_ledQueue, channel, 0);
}

uint8_t IO::_getLedChannel(uint16_t input)
{
	if (input >= 1000 && input < 2000)
		return input - 1000;
	else
		return INCORRECT_PIN;
}
#endif
#ifdef RELAY_PCA9685_ADDR
uint8_t IO::_getRelayChannel(uint16_t input)
{
	if (input >= 2000 && input < 3000)
		return input - 2000;
	else
		return INCORRECT_PIN;
}
#endif

uint16_t IO::_getExpanderInput(uint8_t expanderPort)
{
	switch (expanderPort)
	{
	case 0:
		return OSK_IO10;
	case 1:
		return OSK_IO11;
	case 2:
		return OSK_IO12;
	case 3:
		return OSK_IO13;
	case 4:
		return OSK_IO14;
	case 5:
		return OSK_IO15;
	case 6:
		return OSK_IO16;
	default:
		return INCORRECT_PIN;
	}
}

uint8_t IO::_getExpanderIndexByInput(uint16_t pin)
{
	switch (pin)
	{
	case OSK_IO1:
		return 1;
	case OSK_IO2:
		return 2;
	case OSK_IO3:
		return 3;
	case OSK_IO4:
		return 4;
	case OSK_IO5:
		return 5;
	case OSK_IO6:
		return 6;
	case OSK_IO7:
		return 7;
	case OSK_IO8:
		return 8;
	case OSK_IO9:
		return 9;
	case OSK_IO10:
		return 10;
	case OSK_IO11:
		return 11;
	case OSK_IO12:
		return 12;
	case OSK_IO13:
		return 13;
	case OSK_IO14:
		return 14;
	case OSK_IO15:
		return 15;
	case OSK_IO16:
		return 16;
	default:
		return 0;
	}
}

