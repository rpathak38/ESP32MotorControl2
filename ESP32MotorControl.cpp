/*****************************************
 * Library   : ESP32MotorControl2 - Library for dual motor driver carrier for Arduino. For use with DRV8833.
 * Programmers: Rishi Pathak, Karol Pieniacu, Joao Lopes
 * Comments  : This library is to use with dual motors ICs, as DRV8833, DRV8825 and L298.
 * 			   And uses the MCPWM for control motors
 * Versions  :
 * ------ 	---------- 		-------------------------
 * 0.1.0  	2018-12-26		First beta
 * 0.2.0  	2021-11-21		Revision by Karol Pieniacy
 * 0.3.0    2022-12-08      Fast/Slow Delay Modes by Rishi Pathak
 *****************************************/

/*
 * Source for ESP32MotorControl
 *
 * Copyright (C) 2022  Rishi Pathak https://github.com/rpathak38/ESP32MotorControl2
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
 * This file contains the code for ESP32MotorControl library.
 *
 */

#include "ESP32MotorControl.h"


#include "Arduino.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

namespace {
    inline constexpr mcpwm_unit_t MCPWM_UNIT(uint8_t motor) {
    	return motor == 0 ? MCPWM_UNIT_0 : MCPWM_UNIT_1;
    }
    inline constexpr mcpwm_timer_t MCPWM_TIMER(uint8_t motor) {
    	return motor == 0 ? MCPWM_TIMER_0 : MCPWM_TIMER_1;
    }

}  // namespace

ESP32MotorControl::ESP32MotorControl(uint8_t gpioNSleep, boolean coast, uint32_t frequencyHz) {
    this->gpioNSleep = gpioNSleep;
    this->coast = coast;
    this->frequencyHz = frequencyHz;
    pinMode(gpioNSleep, OUTPUT);
}

void ESP32MotorControl::setFrequency(uint32_t frequency) {
    this->frequencyHz = frequency;
}

void ESP32MotorControl::wake() {
    digitalWrite(gpioNSleep, HIGH);
}

void ESP32MotorControl::sleep() {
    digitalWrite(gpioNSleep, LOW);
}

void ESP32MotorControl::setCoast(bool coast) {
    this->coast = coast;
}

void ESP32MotorControl::setMotorNoPWM(uint8_t motor, int8_t dir) {
    if (dir > 0) {
        // Full forward
        mcpwm_set_signal_low(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_B);
        mcpwm_set_signal_high(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_A);
    } else if (dir < 0) {
        // Full reverse
        mcpwm_set_signal_low(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_A);
        mcpwm_set_signal_high(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_B);
    } else {
        // Stop
        if (coast) {
            mcpwm_set_signal_low(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_A);
            mcpwm_set_signal_low(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_B);
        }
        else {
            mcpwm_set_signal_high(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_A);
            mcpwm_set_signal_high(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_B);
        }
    }
}

void ESP32MotorControl::setMotorPWM(uint8_t motor, int8_t speed)
{
    if (speed == 0 || speed == -100 || speed == 100) {
        setMotorNoPWM(motor, speed);
        return;
    }
    if (coast) {
        if (speed > 0) {
            mcpwm_set_signal_low(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_B);
            mcpwm_set_duty(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_A, speed);
            // call the following each time, in case we used mcpwm_set_signal_low/high
            mcpwm_set_duty_type(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        } else {
            mcpwm_set_signal_low(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_A);
            mcpwm_set_duty(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_B, -speed);
            // call the following each time, in case we used mcpwm_set_signal_low/high
            mcpwm_set_duty_type(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        }
    }
    else {
        if (speed > 0) {
            mcpwm_set_signal_high(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_A);
            mcpwm_set_duty(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_B, speed);
            // call the following each time, in case we used mcpwm_set_signal_low/high
            mcpwm_set_duty_type(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        } else {
            mcpwm_set_signal_high(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_B);
            mcpwm_set_duty(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_A, -speed);
            // call the following each time, in case we used mcpwm_set_signal_low/high
            mcpwm_set_duty_type(MCPWM_UNIT(motor), MCPWM_TIMER(motor), MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        }
    }
}

void ESP32MotorControl::attachMotor(uint8_t gpioIn1, uint8_t gpioIn2)
{
	attachMotors(gpioIn1, gpioIn2, 0, 0);
}

void ESP32MotorControl::attachMotors(uint8_t gpioIn1, uint8_t gpioIn2, uint8_t gpioIn3,
                                     uint8_t gpioIn4)
{
	// Attach motor 0 input pins.
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, gpioIn1);
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, gpioIn2);

	// Indicate the motor 0 is attached.
	this->mMotorAttached_[0] = true;

	if (!(gpioIn3 == 0 && gpioIn4 == 0)) {
		// Attach motor 1 input pins.
		mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, gpioIn3);
		mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, gpioIn4);

		// Indicate the motor 1 is attached.
		this->mMotorAttached_[1] = true;
	}

	// Initial MCPWM configuration
	mcpwm_config_t cfg;
	cfg.frequency = this->frequencyHz;
	cfg.cmpr_a = 0;
	cfg.cmpr_b = 0;
	cfg.counter_mode = MCPWM_UP_COUNTER;
	cfg.duty_mode = MCPWM_DUTY_MODE_0;

	// Configure PWM0A & PWM0B with above settings
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);

	// Configure PWM1A & PWM1B with above settings
	mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &cfg);
}

void ESP32MotorControl::motorsSet(int8_t speed0, int8_t speed1)
{
	setMotor_(0, speed0);
	setMotor_(1, speed1);
}

uint8_t ESP32MotorControl::getMotorSpeed(uint8_t motor)
{
	if (!isMotorValid_(motor))
		return false;

	return mMotorSpeed[motor];
}

boolean ESP32MotorControl::isMotorStopped(uint8_t motor)
{
	if (!isMotorValid_(motor))
		return true;

	return (mMotorSpeed[motor] == 0);
}

void ESP32MotorControl::setMotor_(uint8_t motor, int8_t speed)
{
	if (!isMotorValid_(motor) || speed > 100 || speed < -100)
		return;

	setMotorPWM(motor, speed);

	mMotorSpeed[motor] = std::abs(speed);
	mMotorForward[motor] = speed > 0;
}

boolean ESP32MotorControl::isAwake() {
    if (this->awake) {
        return true;
    }
    return false;
}

boolean ESP32MotorControl::isCoast() {
    if (this->coast) {
        return true;
    }
    return false;
}

uint32_t ESP32MotorControl::getFrequency() {
   return this->frequencyHz;
}


boolean ESP32MotorControl::isMotorValid_(uint8_t motor)
{
	if (motor > 1)
		return false;

	return mMotorAttached_[motor];
}
