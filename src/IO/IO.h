#ifndef IO_h
#define IO_h

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <functional>
#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>
#include "OSK_config.h"
#include <Debug.h>

#if defined MAIN_PCA9685_ADDR || defined RELAY_PCA9685_ADDR
#include <PCA9685.h>
#endif
#if defined MAIN_PCA9685_ADDR
#include <cie1931.h>
#endif

#define INCORRECT_PIN 255
#define EXPANDER_INPUT 245

#define PWM_FREQUENCY 1526
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 12

#define TASK_PRIORITY 3
#define TASK_CORE 1
#define TASK_STACK 10240
#define QUEUE_RECEIVE_DELAY 10

typedef std::function<void(bool state)> PinChangeHandlerFunction;

struct EventSubscriber
{
	bool active;
	PinChangeHandlerFunction fn;
	int mode;
};

struct LedConfig
{
	uint8_t id;
	uint8_t state;
	uint8_t current;
	uint8_t target;
};

class IO
{
public:
	static IO *getInstance();
	~IO();
	void mode(uint16_t input, uint8_t mode);
	bool get(uint16_t input);
	void set(uint16_t input, bool state);
	void pwmWrite(uint16_t input, uint16_t value);
	uint16_t pwmRead(uint16_t input);
	void on(uint16_t input, int mode, PinChangeHandlerFunction method);
	void off(uint16_t input);

#ifdef MAIN_PCA9685_ADDR
	void ledDim(uint16_t input, uint8_t value, uint16_t duration = 1000);
	uint8_t ledRead(uint16_t input);
#endif

private:
	IO();
	void begin();
	static void taskHandler(void *pvParameters);
	static void IRAM_ATTR expIsr();
	static void IRAM_ATTR inp1Isr();
	static void IRAM_ATTR inp2Isr();
	static void IRAM_ATTR inp3Isr();
	static void IRAM_ATTR inp4Isr();
	static void IRAM_ATTR inp5Isr();
	static void IRAM_ATTR inp6Isr();
	static void IRAM_ATTR inp7Isr();
	static void IRAM_ATTR inp8Isr();
	static void IRAM_ATTR inp9Isr();
	static IO *_instance;

	void IRAM_ATTR _inputInterrupt(uint16_t input);
	void IRAM_ATTR _expanderInterrupt();
	bool _isNativePort(uint16_t input);
	uint8_t _getExpanderPort(uint16_t input);
	uint16_t _getExpanderInput(uint8_t expanderPort);
	uint8_t _getExpanderIndexByInput(uint16_t input);
	uint8_t _prevValues[10];
	void _initPrevValues();

#ifdef MAIN_PCA9685_ADDR
	static void _ledTaskHandler(void *pvParameters);
	static void _ledTimerHandle(TimerHandle_t handle);
	uint8_t _getLedChannel(uint16_t input);
#endif

#ifdef RELAY_PCA9685_ADDR
	uint8_t _getRelayChannel(uint16_t input);
#endif

	PCF8574 *_exp = nullptr;
	uint8_t _expPinsState = 0;
	TaskHandle_t _task = nullptr;
	QueueHandle_t _queue = nullptr;
	EventSubscriber _subscribers[20];

#ifdef MAIN_PCA9685_ADDR
	PCA9685 *_led = nullptr;
	QueueHandle_t _ledQueue = nullptr;
	TaskHandle_t _ledTask = nullptr;
	TimerHandle_t _ledTimer[OSK_DC_COUNT];
	LedConfig _ledConfig[OSK_DC_COUNT];
#endif
#ifdef RELAY_PCA9685_ADDR
	PCA9685 *_relay = nullptr;
#endif
};

#endif