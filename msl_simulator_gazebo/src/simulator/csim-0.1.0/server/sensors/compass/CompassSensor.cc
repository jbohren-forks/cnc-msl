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

/* Desc: CAMABADA compass sensor
 * Author: Eurico F. Pedrosa <efp@ua.pt>
 * Date: 25 Fev 2010
 */

#include <sstream>
#include "simulator/csim-0.1.0/server/Global.hh"
#include "simulator/csim-0.1.0/server/GazeboError.hh"
#include "simulator/csim-0.1.0/server/World.hh"
#include "simulator/csim-0.1.0/server/Model.hh"
#include "simulator/csim-0.1.0/server/physics/Body.hh"
#include "simulator/csim-0.1.0/server/Field.hh"
#include "simulator/csim-0.1.0/server/Rand.hh"

#include "simulator/csim-0.1.0/server/sensors/SensorFactory.hh"
#include "simulator/csim-0.1.0/server/sensors/compass/CompassSensor.hh"

// CAMBADA include
#include "simulator/libs/rtdb/rtdb_sim.h"
#include "simulator/libs/rtdb/rtdb_user.h"
#include "simulator/libs/rtdb/HWcomm_rtdb.h"

using namespace csim;
using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("compass", CompassSensor);

//////////////////////////////////////////////////////////////////////////////
// Constructor
CompassSensor::CompassSensor(Body *body)
    : Sensor(body)
{
  this->typeName = "compass";
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
CompassSensor::~CompassSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
// Load the sensor
void CompassSensor::LoadChild( XMLConfigNode *node )
{
  this->theNorth = World::Instance()->GetField()->north ;// node->GetFloat("theNorth",  0.0, 0 );
  this->selfID = this->GetParentModel()->GetSelfID();

  this->noisyCompass = false; //node->GetBool("noise", false, 0);
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the compass
void CompassSensor::InitChild()
{
  // Without ID define it will not init
  if ( this->selfID < 1 ) { 
    gzerr(0) << "Agent ID is missing, Compass will not update\n";
    return;
  }
  
  this->adjustNorth = DTOR(this->theNorth) - M_PI/2;
  
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the compass
void CompassSensor::FiniChild()
{
}

//////////////////////////////////////////////////////////////////////////////
// Update the compass
void CompassSensor::UpdateChild()
{
  
  if ( this->selfID < 1  ) return;
  // Parent pose
  Pose3d ppose = this->body->GetAbsPose();

  double value = ppose.rot.GetYaw() + M_PI/2;
  if ( this->noisyCompass )
      value += Rand::GetDblNormal(0.0, 0.038818);

  int compass = (int) (RTOD( value ) - (90.0 - this->theNorth ) );
  
  CMD_Imu info;
  info.compass = compass;
  info.yaw = compass;
  info.rawYaw = compass;
  //DB_put_in(this->selfID, this->selfID, CMD_COMPASS , (void*)(&compass) , 0);
  //DB_put_in(this->selfID, this->selfID, CMD_IMU, (void*)(&compass), 0); 
  DB_put_in(this->selfID, this->selfID, CMD_IMU, (void*)(&info), 0); 

}
