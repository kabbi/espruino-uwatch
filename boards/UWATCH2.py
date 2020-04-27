#!/bin/false
# This file is part of Espruino, a JavaScript interpreter for Microcontrollers
#
# Copyright (C) 2013 Gordon Williams <gw@pur3.co.uk>
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
#
# ----------------------------------------------------------------------------------------
# This file contains information for a specific board - the available pins, and where LEDs,
# Buttons, and other in-built peripherals are. It is used to build documentation as well
# as various source and header files for Espruino.
# ----------------------------------------------------------------------------------------

import pinutils

info = {
    'name': "Umidigi Uwatch2",
    'link':  ["http://www.espruino.com/Bangle.js"],
    'espruino_page_link': 'uwatch2',
    'default_console': "EV_BLUETOOTH",
    # How many variables are allocated for Espruino to use. RAM will be overflowed if this number is too high and code won't compile.
    'variables': 2100,
    'bootloader': 0,
    'binary_name': 'espruino_%v_uwatch.hex',
    'build': {
        'optimizeflags': '-Os',
        'libraries': [
            'BLUETOOTH',
            'GRAPHICS',
            'LCD_SPI'
            # 'TERMINAL',
        ],
        'makefile': [
            'DEFINES += -DCONFIG_NFCT_PINS_AS_GPIOS',  # Allow the reset pin to work
            'DEFINES += -DBUTTONPRESS_TO_REBOOT_BOOTLOADER',
            'DEFINES+=-DBLUETOOTH_NAME_PREFIX=\'"Uwatch2.js"\'',
            'DEFINES+=-DCUSTOM_GETBATTERY=jswrap_uwatch_getBattery',
            'DEFINES+=-DDUMP_IGNORE_VARIABLES=\'"g\\0"\'',
            'DEFINES+=-DUSE_FONT_6X8 -DGRAPHICS_PALETTED_IMAGES',
            'DFU_PRIVATE_KEY=targets/nrf5x_dfu/dfu_private_key.pem',
            'DFU_SETTINGS=--application-version 0xff --hw-version 52 --sd-req 0x8C',
            'INCLUDE += -I$(ROOT)/libs/uwatch -I$(ROOT)/libs/misc',
            'WRAPPERSOURCES += libs/uwatch/jswrap_uwatch.c',
            'SOURCES += libs/misc/nmea.c',
            'JSMODULESOURCES += libs/js/graphical_menu.min.js',
            'NRF_BL_DFU_INSECURE=1',
            'LINKER_BOOTLOADER=targetlibs/nrf5x_12/nrf5x_linkers/uwatch_dfu.ld',
            'LINKER_ESPRUINO=targetlibs/nrf5x_12/nrf5x_linkers/uwatch_espruino.ld'
        ]
    }
}


chip = {
    'part': "NRF52832",
    'family': "NRF52",
    'package': "QFN48",
    'ram': 64,
    'flash': 512,
    'speed': 64,
    'usart': 1,
    'spi': 2,
    'i2c': 1,
    'adc': 1,
    'dac': 0,
    'saved_code': {
        # Bootloader takes pages 120-127, FS takes 118-119
        'address': ((118 - 10) * 4096),
        'page_size': 4096,
        'pages': 10,
        # Softdevice uses 31 pages of flash, bootloader 8, FS 2, code 10. Each page is 4 kb.
        'flash_available': 512 - ((31 + 8 + 2 + 10)*4)
    },
}

devices = {
    'BTN1' : { 'pin' : 'D29', 'pinstate' : 'IN_PULLDOWN' },
    'BTN2' : { 'pin' : 'D30', 'pinstate' : 'IN_PULLDOWN' },
    'LED1': {'pin': 'D27'},
    'VIBRATE': {'pin': 'D16'},
    'LCD': {
        'width': 240, 'height': 240, 'bpp': 16,
        'controller': 'st7789u',
        'pin_dc': 'D18',
        'pin_cs': 'D25',
        'pin_rst': 'D26',
        'pin_sck': 'D2',
        'pin_mosi': 'D3',
        'pin_bl': 'D23',
    },
    'ACCEL': {
        'device': 'KX023', 'addr': 0x1e,
        'pin_sda': 'D6',
        'pin_scl': 'D7',
    },
    'TOUCH': {
        'device': 'CST816S', 'addr': 0x15,
        'pin_sda': 'D6',
        'pin_scl': 'D7',
        'pin_interrupt': 'D28',
    },
    'BAT': {
        'pin_charging': 'D19',
        'pin_voltage': 'D31',
    },
}

board = {
    'left': [],
    'right': [],
    '_notes': {}
}
board["_css"] = ""


def get_pins():
    pins = pinutils.generate_pins(0, 31)  # 32 General Purpose I/O Pins.
    pinutils.findpin(pins, "PD0", True)["functions"]["XL1"] = 0
    pinutils.findpin(pins, "PD1", True)["functions"]["XL2"] = 0

    # everything is non-5v tolerant
    for pin in pins:
        pin["functions"]["3.3"] = 0

    return pins
