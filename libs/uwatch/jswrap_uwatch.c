/*
 * This file is part of Espruino, a JavaScript interpreter for Microcontrollers
 *
 * Copyright (C) 2019 Gordon Williams <gw@pur3.co.uk>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * ----------------------------------------------------------------------------
 * This file is designed to be parsed during the build process
 *
 * Contains JavaScript interface for Uwatch2 (and probably lots of other
 * chinese watches compatible with dafit software)
 * ----------------------------------------------------------------------------
 */

#include <jswrap_uwatch.h>
#include "jsinteractive.h"
#include "jsdevices.h"
#include "jsnative.h"
#include "jshardware.h"
#include "jsdevices.h"
#include "jspin.h"
#include "jstimer.h"
#include "jswrap_promise.h"
#include "jswrap_bluetooth.h"
#include "jswrap_date.h"
#include "jswrap_math.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_soc.h"
#include "nrf_nvic.h"
#include "nrf5x_utils.h"
#include "jsflash.h" // for jsfRemoveCodeFromFlash
#include "bluetooth.h" // for self-test
#include "jsi2c.h" // accelerometer/etc

#include "jswrap_graphics.h"
#include "lcd_spilcd.h"
#include "nmea.h"

#include "accel_config.h"

/*JSON{
  "type": "class",
  "class" : "Uwatch",
  "ifdef" : "UWATCH"
}
Class containing utility functions for the [Bangle.js Smart Watch](http://www.espruino.com/Bangle.js)
*/


/*JSON{
  "type" : "variable",
  "name" : "VIBRATE",
  "generate_full" : "VIBRATE_PIN",
  "ifdef" : "UWATCH",
  "return" : ["pin",""]
}
The Bangle.js's vibration motor.
*/
/*JSON{
  "type" : "event",
  "class" : "Uwatch",
  "name" : "lcdPower",
  "params" : [["on","bool","`true` if screen is on"]],
  "ifdef" : "UWATCH"
}
Has the screen been turned on or off? Can be used to stop tasks that are no longer useful if nothing is displayed.
*/
/*JSON{
  "type" : "event",
  "class" : "Uwatch",
  "name" : "accel",
  "params" : [["xyz","JsVar","`{x,y,z}`"]],
  "ifdef" : "UWATCH"
}
Accelerometer data available with `{x,y,z,diff,mag}` object as a parameter.
You can also retrieve the most recent reading with `Bangle.getAccel()`.
 */
/*JSON{
  "type" : "event",
  "class" : "Uwatch",
  "name" : "touch",
  "params" : [["data","JsVar","`{gesture, x, y}`"]],
  "ifdef" : "UWATCH"
}
Emitted when the touchscreen is touched.
*/

#define BTN1_LOAD_TIMEOUT 1500 // in msec
#define TIMER_MAX 60000 // 60 sec - enough to fit in uint16_t without overflow if we add ACCEL_POLL_INTERVAL
/// Internal I2C used for Accelerometer/Pressure
JshI2CInfo internalI2C;
/// Is I2C busy? if so we'll skip one reading in our interrupt so we don't overlap
bool i2cBusy;
/// How often should be poll for accelerometer/compass data?
volatile uint16_t pollInterval; // in ms
/// counter that counts up if watch has stayed face up or down
volatile unsigned char faceUpCounter;
/// time since LCD contents were last modified
volatile uint16_t flipTimer; // in ms
/// How long has BTN1 been held down for
volatile uint16_t btn1Timer; // in ms
/// Is LCD power automatic? If true this is the number of ms for the timeout, if false it's 0
int lcdPowerTimeout = 30*1000; // in ms
/// Is the LCD on?
bool lcdPowerOn;
/// Current step counter
uint32_t stepCounter;

/// Promise when buzz is finished
JsVar *promiseBuzz;
/// Promise when pressure is requested
JsVar *promisePressure;

typedef struct {
  uint8_t gesture;
  uint8_t x;
  uint8_t y;
} JsTouchEvent;
JsTouchEvent lastTouchEvent;

typedef struct {
  uint16_t intStatus;
} JsAccelEvent;
JsAccelEvent lastAccelEvent;

typedef enum {
  JSBT_NONE,
  JSBT_LCD_ON = 1,
  JSBT_LCD_OFF = 2,
  JSBT_TOUCH_EVENT = 4,
  JSBT_ACCEL_DATA = 8,
  JSBT_STEP_EVENT = 16,
  JSBT_RESET = 128, ///< reset the watch and reload code from flash
} JsBangleTasks;
JsBangleTasks bangleTasks;



/// Send buffer contents to the screen. Usually only the modified data will be output, but if all=true then the whole screen contents is sent
void lcd_flip(JsVar *parent, bool all) {
  JsGraphics gfx;
  if (!graphicsGetFromVar(&gfx, parent)) return;
  if (all) {
    gfx.data.modMinX = 0;
    gfx.data.modMinY = 0;
    gfx.data.modMaxX = LCD_WIDTH-1;
    gfx.data.modMaxY = LCD_HEIGHT-1;
  }
  if (lcdPowerTimeout && !lcdPowerOn) {
    // LCD was turned off, turn it back on
    jswrap_uwatch_setLCDPower(1);
  }
  flipTimer = 0;
  lcdFlip_SPILCD(&gfx);
  graphicsSetVar(&gfx);
}

char clipi8(int x) {
  if (x<-128) return -128;
  if (x>127) return 127;
  return x;
}

/* Scan peripherals for any data that's needed
 * Also, holding down both buttons will reboot */
void peripheralPollHandler() {
  // Handle watchdog
  jshKickWatchDog();

  // Power on display if a button is pressed
  if (lcdPowerTimeout &&
      (jshPinGetValue(BTN1_PININDEX) || jshPinGetValue(BTN2_PININDEX))) {
    flipTimer = 0;
    if (!lcdPowerOn)
      bangleTasks |= JSBT_LCD_ON;
  }
  if (flipTimer < TIMER_MAX)
    flipTimer += pollInterval;

  // If BTN1 is held down, trigger a reset
  if (jshPinGetValue(BTN1_PININDEX)) {
    if (btn1Timer < TIMER_MAX)
      btn1Timer += pollInterval;
  } else {
    if (btn1Timer > BTN1_LOAD_TIMEOUT) {
      bangleTasks |= JSBT_RESET;
      // execInfo.execute |= EXEC_CTRL_C|EXEC_CTRL_C_WAIT; // set CTRLC
    }
    btn1Timer = 0;
  }

  if (lcdPowerTimeout && lcdPowerOn && flipTimer>=lcdPowerTimeout) {
    // 10 seconds of inactivity, turn off display
    bangleTasks |= JSBT_LCD_OFF;
  }
  //jshPinOutput(LED1_PININDEX, 0);

  if (i2cBusy) {
    return;
  }

  // Read steps
  unsigned char buf[4];
  buf[0] = 0x1E; // step counter 0
  jsi2cWrite(&internalI2C, ACCEL_ADDR, 1, buf, false);
  jsi2cRead(&internalI2C, ACCEL_ADDR, 4, buf, true);
  uint32_t steps = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
  if (stepCounter != steps) {
    bangleTasks |= JSBT_STEP_EVENT;
    stepCounter = steps;
  }
}

void touchInterruptHandler(bool state, IOEventFlags flags) {
  // Trigger only on falling edge
  if (state) {
    return;
  }

  unsigned char buf[6];
  buf[0] = 0x01; // read 6 bytes starting from second
  jsi2cWrite(&internalI2C, TOUCH_ADDR, 1, buf, false);
  jsi2cRead(&internalI2C, TOUCH_ADDR, 6, buf, true);
  lastTouchEvent.gesture = buf[0];
  lastTouchEvent.x = ((buf[2] & 0xf) << 8) | buf[3];
  lastTouchEvent.y = ((buf[4] & 0xf) << 8) | buf[5];
  bangleTasks |= JSBT_TOUCH_EVENT;
}

void accelInterruptHandler(bool state, IOEventFlags flags) {
  // Trigger only on falling edge
  if (state) {
    return;
  }

  unsigned char buf[2];
  buf[0] = 0x1C; // int0 status register
  jsi2cWrite(&internalI2C, ACCEL_ADDR, 1, buf, false);
  jsi2cRead(&internalI2C, ACCEL_ADDR, 2, buf, true);
  lastAccelEvent.intStatus = (buf[0] << 8) | buf[1];
  bangleTasks |= JSBT_ACCEL_DATA;
}

/*JSON{
    "type" : "staticmethod",
    "class" : "Uwatch",
    "name" : "setLCDPower",
    "generate" : "jswrap_uwatch_setLCDPower",
    "params" : [
      ["isOn","bool","True if the LCD should be on, false if not"]
    ]
}
This function can be used to turn Bangle.js's LCD off or on.
*/
void jswrap_uwatch_setLCDPower(bool isOn) {
  if (isOn) {
    lcdCmd_SPILCD(0x11, 0, NULL); // SLPOUT
    jshPinOutput(LCD_BL, 0); // backlight
  } else {
    lcdCmd_SPILCD(0x10, 0, NULL); // SLPIN
    jshPinOutput(LCD_BL, 1); // backlight
  }
  if (lcdPowerOn != isOn) {
    JsVar *bangle = jsvObjectGetChild(execInfo.root, "Uwatch", 0);
    if (bangle) {
      JsVar *v = jsvNewFromBool(isOn);
      jsiQueueObjectCallbacks(bangle, JS_EVENT_PREFIX"lcdPower", &v, 1);
      jsvUnLock(v);
    }
    jsvUnLock(bangle);
  }
  flipTimer = 0;
  lcdPowerOn = isOn;
}

/*JSON{
    "type" : "staticmethod",
    "class" : "Uwatch",
    "name" : "setLCDTimeout",
    "generate" : "jswrap_uwatch_setLCDTimeout",
    "params" : [
      ["isOn","float","The timeout of the display in seconds, or `0`/`undefined` to turn power saving off. Default is 10 seconds."]
    ]
}
This function can be used to turn Bangle.js's LCD power saving on or off.

With power saving off, the display will remain in the state you set it with `Bangle.setLCDPower`.

With power saving on, the display will turn on if a button is pressed, the watch is turned face up, or the screen is updated. It'll turn off automatically after the given timeout.
*/
void jswrap_uwatch_setLCDTimeout(JsVarFloat timeout) {
  if (!isfinite(timeout)) lcdPowerTimeout=0;
  else lcdPowerTimeout = timeout*1000;
  if (lcdPowerTimeout<0) lcdPowerTimeout=0;
}

/*JSON{
    "type" : "staticmethod",
    "class" : "Uwatch",
    "name" : "setPollInterval",
    "generate" : "jswrap_uwatch_setPollInterval",
    "params" : [
      ["interval","float","Polling interval in milliseconds"]
    ]
}
Set how often the watch should poll for new acceleration/gyro data
*/
void jswrap_uwatch_setPollInterval(JsVarFloat interval) {
  if (!isfinite(interval) || interval<10) {
    jsExceptionHere(JSET_ERROR, "Invalid interval");
    return;
  }
  pollInterval = (uint16_t)interval;
  JsSysTime t = jshGetTimeFromMilliseconds(pollInterval);
  jstStopExecuteFn(peripheralPollHandler, 0);
  jstExecuteFn(peripheralPollHandler, NULL, jshGetSystemTime()+t, t);
}


/*JSON{
    "type" : "staticmethod",
    "class" : "Uwatch",
    "name" : "setLCDPalette",
    "generate" : "jswrap_uwatch_setLCDPalette",
    "params" : [
      ["palette","JsVar","An array of 24 bit 0xRRGGBB values"]
    ]
}
Bangle.js's LCD can display colours in 12 bit, but to keep the offscreen
buffer to a reasonable size it uses a 4 bit paletted buffer.

With this, you can change the colour palette that is used.
*/
void jswrap_uwatch_setLCDPalette(JsVar *palette) {
  if (jsvIsIterable(palette)) {
    uint16_t pal[16];
    JsvIterator it;
    jsvIteratorNew(&it, palette, JSIF_EVERY_ARRAY_ELEMENT);
    int idx = 0;
    while (idx<16 && jsvIteratorHasElement(&it)) {
      unsigned int rgb = jsvIteratorGetIntegerValue(&it);
      unsigned int r = rgb>>16;
      unsigned int g = (rgb>>8)&0xFF;
      unsigned int b = rgb&0xFF;
      pal[idx++] = ((r&0xF0)<<4) | (g&0xF0) | (b>>4);
      jsvIteratorNext(&it);
    }
    jsvIteratorFree(&it);
    lcdSetPalette_SPILCD(pal);
  } else
    lcdSetPalette_SPILCD(0);
}

/*JSON{
    "type" : "staticmethod",
    "class" : "Uwatch",
    "name" : "isLCDOn",
    "generate" : "jswrap_uwatch_isLCDOn",
    "return" : ["bool","Is the display on or not?"]
}
*/
bool jswrap_uwatch_isLCDOn() {
  return lcdPowerOn;
}

/*JSON{
    "type" : "staticmethod",
    "class" : "Uwatch",
    "name" : "isCharging",
    "generate" : "jswrap_uwatch_isCharging",
    "return" : ["bool","Is the battery charging or not?"]
}
*/
bool jswrap_uwatch_isCharging() {
  return !jshPinGetValue(BAT_PIN_CHARGING);
}

/// get battery percentage
JsVarInt jswrap_uwatch_getBattery() {
  JsVarFloat v = jshPinAnalog(BAT_PIN_VOLTAGE);
  const JsVarFloat vlo = 0.51;
  const JsVarFloat vhi = 0.62;
  int pc = (v-vlo)*100/(vhi-vlo);
  if (pc>100) pc=100;
  if (pc<0) pc=0;
  return pc;
}

/*JSON{
    "type" : "staticmethod",
    "class" : "Uwatch",
    "name" : "lcdWr",
    "generate" : "jswrap_uwatch_lcdWr",
    "params" : [
      ["cmd","int",""],
      ["data","JsVar",""]
    ]
}
Writes a command directly to the ST7735 LCD controller
*/
void jswrap_uwatch_lcdWr(JsVarInt cmd, JsVar *data) {
  JSV_GET_AS_CHAR_ARRAY(dPtr, dLen, data);
  lcdCmd_SPILCD(cmd, dLen, dPtr);
}



/*JSON{
    "type" : "staticmethod",
    "class" : "Uwatch",
    "name" : "accelWr",
    "generate" : "jswrap_uwatch_accelWr",
    "params" : [
      ["reg","int",""],
      ["data","int",""]
    ]
}
Writes a register on the KX023 Accelerometer
*/
void jswrap_uwatch_accelWr(JsVarInt reg, JsVarInt data) {
  unsigned char buf[2];
  buf[0] = (unsigned char)reg;
  buf[1] = (unsigned char)data;
  i2cBusy = true;
  jsi2cWrite(&internalI2C, ACCEL_ADDR, 2, buf, true);
  i2cBusy = false;
}



/*JSON{
    "type" : "staticmethod",
    "class" : "Uwatch",
    "name" : "accelRd",
    "generate" : "jswrap_uwatch_accelRd",
    "params" : [
      ["reg","int",""]
    ],
    "return" : ["int",""]
}
Reads a register from the KX023 Accelerometer
*/
int jswrap_uwatch_accelRd(JsVarInt reg) {
  unsigned char buf[1];
  buf[0] = (unsigned char)reg;
  i2cBusy = true;
  jsi2cWrite(&internalI2C, ACCEL_ADDR, 1, buf, true);
  jsi2cRead(&internalI2C, ACCEL_ADDR, 1, buf, true);
  i2cBusy = false;
  return buf[0];
}

/*JSON{
    "type" : "staticmethod",
    "class" : "Uwatch",
    "name" : "reboot",
    "generate" : "jswrap_uwatch_reboot",
    "params" : []
}
Reboot to bootloader
*/
void jswrap_uwatch_reboot() {
  sd_power_gpregret_set(0, 0x42);
  sd_nvic_SystemReset();
}


/*JSON{
  "type" : "init",
  "generate" : "jswrap_uwatch_init"
}*/
void jswrap_uwatch_init() {
  jshPinOutput(VIBRATE_PIN,0); // vibrate off
  jshPinOutput(LED1_PININDEX,0); // LED off

  jswrap_ble_setTxPower(4);

  // Set up I2C
  i2cBusy = true;
  jshI2CInitInfo(&internalI2C);
  internalI2C.bitrate = 0x7FFFFFFF; // make it as fast as we can go
  internalI2C.pinSDA = ACCEL_PIN_SDA;
  internalI2C.pinSCL = ACCEL_PIN_SCL;
  jshPinSetValue(internalI2C.pinSCL, 1);
  jshPinSetState(internalI2C.pinSCL, JSHPINSTATE_GPIO_OUT_OPENDRAIN_PULLUP);
  jshPinSetValue(internalI2C.pinSDA, 1);
  jshPinSetState(internalI2C.pinSDA, JSHPINSTATE_GPIO_OUT_OPENDRAIN_PULLUP);

  lcdPowerOn = true;
  // Create backing graphics for LCD
  JsVar *graphics = jspNewObject(0, "Graphics");
  if (!graphics) return; // low memory
  JsGraphics gfx;
  graphicsStructInit(&gfx, LCD_WIDTH, LCD_HEIGHT, LCD_BPP);
  gfx.data.type = JSGRAPHICSTYPE_SPILCD;
  gfx.graphicsVar = graphics;

  lcdInit_SPILCD(&gfx);
  graphicsSetVar(&gfx);
  jsvObjectSetChild(execInfo.root, "g", graphics);
  jsvObjectSetChild(execInfo.hiddenRoot, JS_GRAPHICS_VAR, graphics);
  graphicsGetFromVar(&gfx, graphics);

  // Create 'flip' fn
  JsVar *fn;
  fn = jsvNewNativeFunction((void (*)(void))lcd_flip, JSWAT_VOID|JSWAT_THIS_ARG|(JSWAT_BOOL << (JSWAT_BITS*1)));
  jsvObjectSetChildAndUnLock(graphics,"flip",fn);

  graphicsClear(&gfx);
  int h=6,y=0;
  jswrap_graphics_drawCString(&gfx,0,y+h*1," ____                 _ ");
  jswrap_graphics_drawCString(&gfx,0,y+h*2,"|  __|___ ___ ___ _ _|_|___ ___ ");
  jswrap_graphics_drawCString(&gfx,0,y+h*3,"|  __|_ -| . |  _| | | |   | . |");
  jswrap_graphics_drawCString(&gfx,0,y+h*4,"|____|___|  _|_| |___|_|_|_|___|");
  jswrap_graphics_drawCString(&gfx,0,y+h*5,"         |_| espruino.com");
  jswrap_graphics_drawCString(&gfx,0,y+h*6," "JS_VERSION" (c) 2019 G.Williams");
  // Write MAC address in bottom right
  JsVar *addr = jswrap_ble_getAddress();
  char buf[20];
  jsvGetString(addr, buf, sizeof(buf));
  jsvUnLock(addr);
  jswrap_graphics_drawCString(&gfx,(LCD_WIDTH-1)-strlen(buf)*6,y+h*8,buf);
  lcdFlip_SPILCD(&gfx);
  graphicsSetVar(&gfx);

  jsvUnLock(graphics);

  // Initialize touch controller
  IOEventFlags channel;
  jshSetPinShouldStayWatched(TOUCH_PIN_INTERRUPT, true);
  channel = jshPinWatch(TOUCH_PIN_INTERRUPT, true);
  if (channel != EV_NONE) jshSetEventCallback(channel, touchInterruptHandler);

  // Initialize accelerometer
  jshSetPinShouldStayWatched(ACCEL_PIN_INTERRUPT, true);
  channel = jshPinWatch(ACCEL_PIN_INTERRUPT, true);
  if (channel != EV_NONE) jshSetEventCallback(channel, accelInterruptHandler);
  jswrap_uwatch_accelWr(0x7E, 0xB6); // cmd, soft reset
  jshDelayMicroseconds(50000);
  jswrap_uwatch_accelWr(0x7C, 0x02); // power config, sleep disable
  jshDelayMicroseconds(1000);
  jswrap_uwatch_accelWr(0x59, 0); // init ctrl, disable config loading
  for (int i = 0; i < sizeof(BMA421_CONFIG_FILE); i += 8) {
    jswrap_uwatch_accelWr(0x5B, (i / 2));
    jswrap_uwatch_accelWr(0x5C, (i / 2) >> 4);
    unsigned char buf[9];
    buf[0] = 0x5E;
    for (int j = 0; j < 8; j++) {
      buf[j + 1] = BMA421_CONFIG_FILE[i + j];
    }
    jsi2cWrite(&internalI2C, ACCEL_ADDR, 9, buf, true);
    jshKickWatchDog();
  }
  jswrap_uwatch_accelWr(0x59, 1); // init ctrl, enable config loading
  jshDelayMicroseconds(150000);
  jswrap_uwatch_accelRd(0x2A); // should be 0x01 when config loading ok
  jswrap_uwatch_accelWr(0x40, 0xA7); // accel config, ?
  jswrap_uwatch_accelWr(0x41, 0x01); // accel range, range_4g
  jswrap_uwatch_accelWr(0x7D, 0x05); // power control, accel enable, aux enable (??)
  for (int i = 0; i < sizeof(BMA421_FEATURES); i += 8) {
    int offset = 0x200 + i;
    jswrap_uwatch_accelWr(0x5B, (offset / 2));
    jswrap_uwatch_accelWr(0x5C, (offset / 2) >> 4);
    unsigned char buf[9];
    buf[0] = 0x5E;
    for (int j = 0; j < 8; j++) {
      buf[j + 1] = BMA421_FEATURES[i + j];
    }
    jsi2cWrite(&internalI2C, ACCEL_ADDR, 9, buf, true);
  }
  jswrap_uwatch_accelWr(0x7C, 0x03); // power config, fifo wakeup, sleep enable
  jswrap_uwatch_accelWr(0x7D, 0x04); // power control, accel enable, aux disable
  jswrap_uwatch_accelWr(0x56, 0x20); // int1 map, enable wakeup interrupt
  jswrap_uwatch_accelWr(0x58, 0x00); // int map data, don't map anything
  jswrap_uwatch_accelWr(0x53, 0x08); // int1 io control, enable int1 pin
  jswrap_uwatch_accelWr(0x70, 0x06); // nvm config, i2c watchdog 40ms
  jswrap_uwatch_accelWr(0x40, 0x27); // accel config, norm_avg4, 50hz
  i2cBusy = false;

  // Add watchdog timer to ensure watch always stays usable (hopefully!)
  // This gets killed when _kill / _init happens
  //  - the bootloader probably already set this up so the
  //    enable will do nothing - but good to try anyway
  jshEnableWatchDog(10); // 5 second watchdog
  // This timer kicks the watchdog, and does some other stuff as well
  pollInterval = 1000;
  JsSysTime t = jshGetTimeFromMilliseconds(pollInterval);
  jstExecuteFn(peripheralPollHandler, NULL, jshGetSystemTime()+t, t);
}

/*JSON{
  "type" : "kill",
  "generate" : "jswrap_uwatch_kill"
}*/
void jswrap_uwatch_kill() {
  jstStopExecuteFn(peripheralPollHandler, 0);
  jsvUnLock(promisePressure);
  promisePressure = 0;
  jsvUnLock(promiseBuzz);
  promiseBuzz = 0;
}

/*JSON{
  "type" : "idle",
  "generate" : "jswrap_uwatch_idle"
}*/
bool jswrap_uwatch_idle() {
  if (bangleTasks == JSBT_NONE) return false;
  JsVar *bangle =jsvObjectGetChild(execInfo.root, "Uwatch", 0);
  /* if (bangleTasks & JSBT_LCD_OFF) jswrap_uwatch_setLCDPower(0); */
  if (bangleTasks & JSBT_LCD_ON) jswrap_uwatch_setLCDPower(1);
  if (bangleTasks & JSBT_RESET)
    jsiStatus |= JSIS_TODO_FLASH_LOAD;
  if (bangleTasks & JSBT_TOUCH_EVENT) {
    JsVar *o = jsvNewObject();
    const char *string = "unknown";
    if (lastTouchEvent.gesture == 0) string = "none";
    if (lastTouchEvent.gesture == 1) string = "swipe-down";
    if (lastTouchEvent.gesture == 2) string = "swipe-up";
    if (lastTouchEvent.gesture == 3) string = "swipe-left";
    if (lastTouchEvent.gesture == 4) string = "swipe-right";
    if (lastTouchEvent.gesture == 5) string = "tap";
    if (lastTouchEvent.gesture == 12) string = "long-press";
    if (lastTouchEvent.gesture == 14) string = "home";
    jsvObjectSetChildAndUnLock(o, "gesture", jsvNewFromString(string));
    jsvObjectSetChildAndUnLock(o, "x", jsvNewFromInteger(lastTouchEvent.x));
    jsvObjectSetChildAndUnLock(o, "y", jsvNewFromInteger(lastTouchEvent.y));
    jsiQueueObjectCallbacks(bangle, JS_EVENT_PREFIX"touch", &o, 1);
    jsvUnLock(o);
  }
  if (bangleTasks & JSBT_ACCEL_DATA) {
    JsVar *o = jsvNewObject();
    jsvObjectSetChildAndUnLock(o, "status", jsvNewFromInteger(lastAccelEvent.intStatus));
    jsiQueueObjectCallbacks(bangle, JS_EVENT_PREFIX"accel", &o, 1);
    jsvUnLock(o);
  }
  if (bangleTasks & JSBT_STEP_EVENT) {
    JsVar *steps = jsvNewFromInteger(stepCounter);
    jsiQueueObjectCallbacks(bangle, JS_EVENT_PREFIX"step", &steps, 1);
    jsvUnLock(steps);
  }
  jsvUnLock(bangle);
  bangleTasks = JSBT_NONE;
  return false;
}



/*JSON{
    "type" : "staticmethod",
    "class" : "Uwatch",
    "name" : "buzz",
    "generate" : "jswrap_uwatch_buzz",
    "params" : [
      ["time","int","Time in ms (default 200)"],
      ["strength","float","Power of vibration from 0 to 1 (Default 1)"]
    ],
    "return" : ["JsVar","A promise, completed when beep is finished"],
    "return_object":"Promise"
}
Perform a Spherical [Web Mercator projection](https://en.wikipedia.org/wiki/Web_Mercator_projection)
of latitude and longitude into `x` and `y` coordinates, which are roughly
equivalent to meters from `{lat:0,lon:0}`.

This is the formula used for most online mapping and is a good way
to compare GPS coordinates to work out the distance between them.
*/
void jswrap_uwatch_buzz_callback() {
  jshPinOutput(VIBRATE_PIN,0); // vibrate off

  jspromise_resolve(promiseBuzz, 0);
  jsvUnLock(promiseBuzz);
  promiseBuzz = 0;
}

JsVar *jswrap_uwatch_buzz(int time, JsVarFloat amt) {
  if (!isfinite(amt)|| amt>1) amt=1;
  if (amt<0) amt=0;
  if (time<=0) time=200;
  if (time>5000) time=5000;
  if (promiseBuzz) {
    jsExceptionHere(JSET_ERROR, "Buzz in progress");
    return 0;
  }
  promiseBuzz = jspromise_create();
  if (!promiseBuzz) return 0;

  jshPinAnalogOutput(VIBRATE_PIN, 0.4 + amt*0.6, 1000, JSAOF_NONE);
  jsiSetTimeout(jswrap_uwatch_buzz_callback, time);
  return jsvLockAgain(promiseBuzz);
}

/*JSON{
    "type" : "staticmethod",
    "class" : "Uwatch",
    "name" : "off",
    "generate" : "jswrap_uwatch_off"
}
Turn Bangle.js off. It can only be woken by pressing BTN1.
*/
void jswrap_uwatch_off() {
  jsiKill();
  jsvKill();
  jshKill();
  jshPinOutput(VIBRATE_PIN,0); // vibrate off
  jshPinOutput(LCD_BL,1); // backlight off
  jshPinOutput(LED1_PININDEX,0); // LED off
  lcdCmd_SPILCD(0x28, 0, NULL); // display off


  nrf_gpio_cfg_sense_set(BTN2_PININDEX, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg_sense_set(BTN1_PININDEX, NRF_GPIO_PIN_SENSE_LOW);
  sd_power_system_off();
}
