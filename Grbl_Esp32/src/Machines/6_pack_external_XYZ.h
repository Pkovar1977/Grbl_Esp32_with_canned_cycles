#pragma once
// clang-format off

/*
    6_pack_external_XYZ.h

    Covers all V1 versions V1p0, V1p1, etc

    Part of Grbl_ESP32
    Pin assignments for the ESP32 I2S 6-axis board

    2021    - Bart Dring
    
    Grbl_ESP32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    Grbl is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with Grbl_ESP32.  If not, see <http://www.gnu.org/licenses/>.


    6 Pack Jumpers for External Drivers
    The only jumper you set is the Vcc on 5V
    Stallguard jumpers must not be connected
    MS/SPI Do not need to be installed. It is OK to put them oonm the MS side
    TMC5160 Is does not matter if this is installed or not on any side.


*/
#define MACHINE_NAME            "6 Pack External XYZ"

#define N_AXIS 6

// === Special Features

// I2S (steppers & other output-only pins)
#define USE_I2S_OUT
#define USE_I2S_STEPS
//#define DEFAULT_STEPPER ST_I2S_STATIC

#define I2S_OUT_BCK             GPIO_NUM_22
#define I2S_OUT_WS              GPIO_NUM_5
#define I2S_OUT_DATA            GPIO_NUM_21


// Motor Socket #1
#define X_STEP_PIN              I2SO(1)
#define X_DIRECTION_PIN         I2SO(2)
//#define X_DISABLE_PIN           I2SO(0)

// Motor Socket #2
#define Y_STEP_PIN              I2SO(3)
#define Y_DIRECTION_PIN         I2SO(4)
//#define Y_DISABLE_PIN           I2SO(3)

// Motor Socket #3
#define Z_STEP_PIN              I2SO(5)
#define Z_DIRECTION_PIN         I2SO(6)
//#define Z_DISABLE_PIN           I2SO(8)

//
// Motor Socket #4
#define A_STEP_PIN              I2SO(7)
#define A_DIRECTION_PIN         I2SO(8)
//#define A_DISABLE_PIN           I2SO(7)

// Motor Socket #5
#define B_STEP_PIN              I2SO(9)
#define B_DIRECTION_PIN         I2SO(10)
//#define B_DISABLE_PIN           I2SO(16)

// Motor Socket #5
#define C_STEP_PIN              I2SO(11)
#define C_DIRECTION_PIN         I2SO(12)
//#define C_DISABLE_PIN           I2SO(11)
//

// OK to comment out to use pin for other features
//#define STEPPERS_DISABLE_PIN    GPIO_NUM_27
//#define STEPPERS_DISABLE_PIN    I2SO(0)  // Shared disable for XYZ TMC drivers SPI
//#define ROTARY_DISABLE_PIN    I2SO(19) // Shared disable for ABC TMC drivers SPI
#define STEPPERS_DISABLE_PIN    I2SO(13) // Shared disable for XYZ drivers SD
#define ROTARY_DISABLE_PIN      I2SO(14) // Shared disable for XYZ drivers SD
#define LUBRICATION_DISABLE_PIN I2SO(15) // Lubrication disable for external lubrication controller

//#define SPINDLE_TYPE            SpindleType::RELAY
#define SPINDLE_TYPE            SpindleType::PWM
#define SPINDLE_ENABLE_PIN      GPIO_NUM_4  // labeled SpinEnbl
#define SPINDLE_OUTPUT_PIN      GPIO_NUM_16  // labeled SpinPWM

#define COOLANT_FLOOD_PIN       I2SO(20)  // labeled Flood
#define COOLANT_MIST_PIN        I2SO(21)  // labeled Mist
//#define PROBE_PIN               GPIO_NUM_32  // labeled Probe



// 4x Input Module in Socket 


#define CONTROL_SAFETY_DOOR_PIN     GPIO_NUM_36  // labeled Door,  needs external pullup
#define PROBE_PIN                   GPIO_NUM_39  // needs external pullup
#define CONTROL_CYCLE_START_PIN     GPIO_NUM_34  // needs external pullup
#define CONTROL_FEED_HOLD_PIN       GPIO_NUM_35  // needs external pullup

#define X_LIMIT_PIN                 GPIO_NUM_33
#define Y_LIMIT_PIN                 GPIO_NUM_25
#define Z_LIMIT_PIN                 GPIO_NUM_26



// ================= Setting Defaults ==========================

// see wiki https://github.com/bdring/Grbl_Esp32/wiki/External-Stepper-Drivers
#define DEFAULT_STEP_ENABLE_DELAY        5 // how long after enable do we wait for 
#define DEFAULT_STEP_PULSE_MICROSECONDS  4 // length of step pulse. Must be greater than I2S_OUT_USEC_PER_PULSE (4) with I2S
#define STEP_PULSE_DELAY                 6 // gap between enable and dir changes before step

//#define DEFAULT_STEPPING_INVERT_MASK     (bit(X_AXIS) | bit(Y_AXIS) | bit(Z_AXIS))
//#define DEFAULT_DIRECTION_INVERT_MASK    (bit(X_AXIS) | bit(Y_AXIS) | bit(Z_AXIS))
//#define DEFAULT_INVERT_ST_ENABLE         true



