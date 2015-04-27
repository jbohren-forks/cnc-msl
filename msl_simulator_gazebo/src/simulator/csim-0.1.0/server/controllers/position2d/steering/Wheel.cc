/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: General Wheel
 * Author: Jordi Polo
 * Date: 18 Dec 2007
 */

#include "simulator/csim-0.1.0/server/Global.hh"
#include "simulator/csim-0.1.0/server/Model.hh"
#include "simulator/csim-0.1.0/server/physics/Joint.hh"
#include "simulator/csim-0.1.0/server/physics/HingeJoint.hh"
#include "simulator/csim-0.1.0/server/physics/Hinge2Joint.hh"
#include "simulator/csim-0.1.0/server/World.hh"
#include "simulator/csim-0.1.0/libgazebo/gazebo.h"
#include "simulator/csim-0.1.0/server/GazeboError.hh"
#include "simulator/csim-0.1.0/server/controllers/ControllerFactory.hh"
#include "simulator/csim-0.1.0/server/controllers/position2d/steering/Steering_Position2d.hh"
#include "simulator/csim-0.1.0/server/controllers/position2d/steering/Wheel.hh"
#include <string>

using namespace gazebo;


enum {DRIVE, STEER, FULL};


////////////////////////////////////////////////////////////////////////////////
// Constructor
Wheel::Wheel( )
{

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Wheel::~Wheel( )
{

}


void Wheel::SetSuspension(float spring, float damping, float step)
{
}


void Wheel::Connect(Joint *joint, int type)
{

}

void Wheel::Stop()
{

}


void Wheel::SetTorque( float newTorque)
{
}

void Wheel::Update(float speed, float steer)
{

}



