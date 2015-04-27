/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA AGENT
 *
 * CAMBADA AGENT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA AGENT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef KICKCALIBDATA_H_
#define KICKCALIBDATA_H_

namespace cambada {

class KickCalibAppData {
public:
	int robotNumber; 	// robot number to be calibrated
	int power; 			// power to test
	float distance; 	// distance to goal to test
	bool active; 		// activate robot to calibration
	bool kick; 			// kick instruction
};

class KickCalibRobData {
public:
	bool stopped; 		// true when robot is stopped preparing for kicking
	bool kicked; 		// true when robot has kicked
	float distance; 	// distance measured by the robot
	int power; 			// power tested by the robot
};

} /* namespace cambada */
#endif /* KICKCALIBDATA_H_ */
