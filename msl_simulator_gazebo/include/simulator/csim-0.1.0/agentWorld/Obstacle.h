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

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "simulator/csim-0.1.0/geom/Vec.h"

using namespace cambada::geom;

class ObstacleInfo	//Estimated information, class for sharing
{
	public:
		ObstacleInfo();
		void clear();
		bool isTeamMate();

		Vec absCenter;		//absolute coordinates
		unsigned char id;
};


class Obstacle
{
	public:
		Obstacle();
		void clear();

		Vec limitCenter;	//relative ccordinates
		Vec leftPoint;		//relative coordinates
		Vec rightPoint;		//relative coordinates
		float obstacleWidth;
		ObstacleInfo obstacleInfo;	//information estimated in integration (sharing information)		
};

#endif

