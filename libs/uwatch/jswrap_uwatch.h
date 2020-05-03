/*
 * This file is part of Espruino, a JavaScript interpreter for Microcontrollers
 *
 * Copyright (C) 2016 Gordon Williams <gw@pur3.co.uk>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * ----------------------------------------------------------------------------
* Contains JavaScript interface for Bangle.js (http://www.espruino.com/Bangle.js)
 * ----------------------------------------------------------------------------
 */
#include "jspin.h"

void jswrap_uwatch_lcdWr(JsVarInt cmd, JsVar *data);
void jswrap_uwatch_setLCDPower(bool isOn);
void jswrap_uwatch_setLCDTimeout(JsVarFloat timeout);
void jswrap_uwatch_setLCDPalette(JsVar *palette);
void jswrap_uwatch_setPollInterval(JsVarFloat interval);
void jswrap_uwatch_setGestureOptions(JsVar *options);
bool jswrap_uwatch_isLCDOn();
int jswrap_uwatch_accelRd(JsVarInt cmd);
void jswrap_uwatch_accelWr(JsVarInt cmd, JsVarInt data);
bool jswrap_uwatch_isCharging();
void jswrap_uwatch_reboot();
JsVarInt jswrap_uwatch_getBattery();

JsVar *jswrap_uwatch_buzz(int time, JsVarFloat amt);
void jswrap_uwatch_off();

void jswrap_uwatch_init();
void jswrap_uwatch_kill();
bool jswrap_uwatch_idle();
