/*
  OSK_IOxx - OSK Module Input/Output number
  OSK_DCOx - OSK Module Direct Current Output number
  OSK_RELAYx - OSK Module Relay number

  To define pins of PCF8574 use 100-108 numbers
*/


/* ============= *
   Supported OSK boards.
   Define related to used hardware.
* ============= */

// Main Boards
// #define OSK_MAIN_DC_S1
// #define OSK_MAIN_RELAY_S1
// #define OSK_MAIN_RELAY_S2

// IO Boards
// #define OSK_IO_1

// Controller
// #define OSK_CONTROLLER_ESP32

// Extension Boards
// #define OSK_EXT_1

/*******************
       CONFIG
********************/

#ifndef OSK_CONFIG_h
#define OSK_CONFIG_h

/* ============= *
   General config for IO boards
   In case of using not default board
   User must define specific OSK_IO_* in main.cpp
* ============= */
#ifdef OSK_IO_2
  // Specific config here for OSK_IO_3_0 (this is example for the future)

#else // default config is for OSK_IO_1
  #define OSK_IO10 100
  #define OSK_IO11 101
  #define OSK_IO12 102
  #define OSK_IO13 103
  #define OSK_IO14 104
  #define OSK_IO15 105
  #define OSK_IO16 106

  #ifndef IO_PCF8574_ADDR
    #define IO_PCF8574_ADDR 0x20 // 0x27
  #endif
  #define IO_INT_PIN 13
#endif


/* ============= *
   Config for Controller boards
   In case of using not default board
   User must define specific OSK_CONTROLLER_* in main.cpp
* ============= */
#ifdef OSK_CONTROLLER_OMEGA2
  // Specific config here for OSK_CONTROLLER_OMEGA2 (this is example for the future)

#elif OSK_CONTROLLER_STM32
  // Specific config here for OSK_CONTROLLER_STM32 (this is example for the future)

#else // default config for OSK_CONTROLLER_ESP32 board based on ESP32
  #define OSK_GREEN_LED 2

  // Specific config for using IO board OSK_IO_2_0 with controller board OSK_CONTROLLER_ESP32_1_0
  #ifdef OSK_IO_3_0 // OSK_IO_3_0 = IO 2.7
    #define OSK_IO1 36
    #define OSK_IO2 32
    #define OSK_IO3 39
    #define OSK_IO4 33
    #define OSK_IO5 34
    #define OSK_IO6 25
    #define OSK_IO7 26
    #define OSK_IO8 35
    #define OSK_IO9 14

  #else // default config is for OSK_IO_2_0
    #define OSK_IO1 26
    #define OSK_IO2 32
    #define OSK_IO3 25
    #define OSK_IO4 33
    #define OSK_IO5 34
    #define OSK_IO6 39
    #define OSK_IO7 36
    #define OSK_IO8 35
    #define OSK_IO9 14
  #endif
#endif


/* ============= *
   Config for Main bards
   There is no default main board, so
   one of supported board OSK_MAIN_* must be defined in main.cpp 
* ============= */
#ifdef OSK_MAIN_DC_S1
  #define OSK_DCO1 1000
  #define OSK_DCO2 1001
  #define OSK_DCO3 1002
  #define OSK_DCO4 1003
  #define OSK_DCO5 1004
  #define OSK_DCO6 1005

  #define MAIN_PCA9685_ADDR 0x40 // 0x7f
  #define OSK_DC_COUNT 6

#elif OSK_MAIN_RELAY_S1
  #define OSK_RELAY1 2000
  #define OSK_RELAY2 2001
  #define OSK_RELAY3 2002
  #define OSK_RELAY4 2003
  #define OSK_RELAY5 2004
  #define OSK_RELAY6 2005
  #define OSK_RELAY7 2006
  #define OSK_RELAY8 2007

  #define RELAY_PCA9685_ADDR 0x40 // 0x7f

#elif OSK_MAIN_RELAY_S2
  #define OSK_RELAY1 2000
  #define OSK_RELAY2 2001
  #define OSK_RELAY3 2002
  #define OSK_RELAY4 2003
  #define OSK_RELAY5 2004
  #define OSK_RELAY6 2005

  #define RELAY_PCA9685_ADDR 0x40 // 0x7f

  // Specific config for using IO board OSK_IO_2_0 with controller board OSK_CONTROLLER_ESP32_1_0 with OSK_RELAY2_1_0
  // Add configuration for other cases if needed
    #define OSK_IO8 14
    #define OSK_IO9 100
    #define OSK_IO10 101
    #define OSK_IO11 102
    #define OSK_IO12 103
    #define OSK_IO13 104
    #define OSK_IO14 105
    #undef OSK_IO15
    #undef OSK_IO16

#endif

#endif