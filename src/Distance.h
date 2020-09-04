/*
 * RobotCar.h
 *
 *  Created on: 29.09.2016
 *  Copyright (C) 2016-2020  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-RobotCar https://github.com/ArminJo/Arduino-RobotCar.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#ifndef DISTANCE_H_
#define DISTANCE_H_

#include <Arduino.h>

void initDistance();

#ifdef CAR_HAS_TOF_DISTANCE_SENSOR
#include <SparkFun_VL53L1X.h>
extern SFEVL53L1X sToFDistanceSensor;
uint8_t getToFDistanceAsCentimeter();
uint8_t readToFDistanceAsCentimeter(); // no start of measurement, just read result.
#define OFFSET_MILLIMETER 10 // The offset measured manually or by calibrateOffset(). Offset = RealDistance - MeasuredDistance
#endif

#ifdef CAR_HAS_IR_DISTANCE_SENSOR
uint8_t getIRDistanceAsCentimeter();
#endif

#endif //  DISTANCE_H_

#pragma once
