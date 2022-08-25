/*
  OSK_IOxx - OSK Module Input/Output number
  OSK_DCOx - OSK Module Direct Current Output number
  OSK_RELAYx - OSK Module Relay number

  To define pins of PCF8574 use 100-108 numbers
*/

// Default Main board
#ifndef OSK_MAIN_BOARD
#define OSK_MAIN_BOARD        OSK_MAIN_DC_2_0
#endif
// Default Controller board
#ifndef OSK_CONTROLLER_BOARD
#define OSK_CONTROLLER_BOARD  OSK_ESP32_1_0
#endif
// Default IO board
#ifndef OSK_IO_BOARD
#define OSK_IO_BOARD          OSK_IO_2_0
#endif
// Default Extension board
#ifndef OSK_EXT_BOARD
#define OSK_EXT_BOARD         OSK_EXT_1_0
#endif
// Default Face board
#ifndef OSK_FACE_BOARD
#define OSK_FACE_BOARD        OSK_FACE_1_0
#endif

#define IO_PCF8574_ADDR 0x27
#define IO_INT_PIN 13

// IO config
#if OSK_IO_BOARD == OSK_IO_2_0
  #define OSK_IO1 26
  #define OSK_IO2 32
  #define OSK_IO3 25
  #define OSK_IO4 33
  #define OSK_IO5 34
  #define OSK_IO6 39
  #define OSK_IO7 36
  #define OSK_IO8 35
  #define OSK_IO9 14
  #define OSK_IO10 100
  #define OSK_IO11 101
  #define OSK_IO12 102
  #define OSK_IO13 103
  #define OSK_IO14 104
  #define OSK_IO15 105
  #define OSK_IO16 106
#endif

// Main board config
#if OSK_MAIN_BOARD == OSK_MAIN_DC_2_0
  #define OSK_DCO1 1000
  #define OSK_DCO2 1001
  #define OSK_DCO3 1002
  #define OSK_DCO4 1003
  #define OSK_DCO5 1004
  #define OSK_DCO6 1005

  #define MAIN_PCA9685_ADDR 0x40
  #define OSK_DC_COUNT 6
  //#define MAIN_PCA9685_ADDR 0x7f

#elif OSK_MAIN_BOARD == OSK_RELAY_1_2
  #define OSK_RELAY1 2000
  #define OSK_RELAY2 2001
  #define OSK_RELAY3 2002
  #define OSK_RELAY4 2003
  #define OSK_RELAY5 2004
  #define OSK_RELAY6 2005
  #define OSK_RELAY7 2006
  #define OSK_RELAY8 2007

#elif OSK_MAIN_BOARD == OSK_RELAY2_1_0
  #define OSK_RELAY1 2000
  #define OSK_RELAY2 2001
  #define OSK_RELAY3 2002
  #define OSK_RELAY4 2003
  #define OSK_RELAY5 2004
  #define OSK_RELAY6 2005

  #if OSK_IO_BOARD == OSK_IO_2_0
    #define OSK_IO8 14
    #define OSK_IO9 100
    #define OSK_IO10 101
    #define OSK_IO11 102
    #define OSK_IO12 103
    #define OSK_IO13 104
    #define OSK_IO14 105
  #endif
#endif

#if OSK_MAIN_BOARD == OSK_RELAY_1_2 || OSK_MAIN_BOARD == OSK_RELAY2_1_0
  #define RELAY_PCA9685_ADDR 0x40
#endif

// Controller board config
#if OSK_CONTROLLER_BOARD == OSK_ESP32_1_0
  #define OSK_GREEN_LED 2
#endif