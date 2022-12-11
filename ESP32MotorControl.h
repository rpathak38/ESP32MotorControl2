/* Header for ESP32MotorControl
 *
 * Copyright (C) 2021  Karol Pieniacy https://github.com/pieniacy/ESP32MotorControl
 *
 * Copyright (C) 2018  Joao Lopes https://github.com/JoaoLopesF/ESP32MotorControl
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This header file describes the public API for SerialDebug.
 *
 */

#ifndef ESP32MotorControl_H
#define ESP32MotorControl_H

#include "Arduino.h"

#ifndef ESP32
#error "this library is only for ESP32"
#endif

class ESP32MotorControl {
public:
	// Fields:
	uint16_t mMotorSpeed[2] = {0, 0};
	boolean mMotorForward[2] = {true, true};

    // constructor: pass in whether you want the robot to coast and the frequency you'd like to use for control
    ESP32MotorControl(uint8_t gpioNSleep, boolean coast = false, uint32_t frequencyHz = 1000);

	// Methods:
    // set the frequency of the DRV8833
    void setFrequency(uint32_t frequency);

    // wake up the DRV8833 from sleep. power will be consumed quickly
    void wake();

    // put the DRV8833 to sleep. power consumption is almost 0
    void sleep();

    // control the DRV8833 in coast mode or quick braking mode.
    void setCoast(bool coast);

	// Attach one motor
	void attachMotor(uint8_t gpioIn1, uint8_t gpioIn2);

	// Attach two motors
	void attachMotors(uint8_t gpioIn1, uint8_t gpioIn2, uint8_t gpioIn3, uint8_t gpioIn4);

	// Set speed values of [-100 : 100] to both motors, set to 0 to stop
	void motorsSet(int8_t speed0, int8_t speed1);

    //set motors to either fully forwards or backwards movment, no speed control
    void setMotorNoPWM(uint8_t motor, int8_t dir);

    //set motor to either forwards or backwards movement, with speed control
    void setMotorPWM(uint8_t motor, int8_t speed);

    //status getters
    uint8_t getMotorSpeed(uint8_t motor);
	boolean isMotorStopped(uint8_t motor);
    boolean isAwake();
    boolean isCoast();
    uint32_t getFrequency();

private:
	// Fields:
	boolean mMotorAttached_[2] = {false, false};
    boolean coast = false;
    boolean awake = false;
    uint32_t frequencyHz = 1000;
    uint8_t gpioNSleep;

    // Methods:
	void setMotor_(uint8_t motor, int8_t speed);
	boolean isMotorValid_(uint8_t motor);
};

#endif  // ESP32MotorControl_H