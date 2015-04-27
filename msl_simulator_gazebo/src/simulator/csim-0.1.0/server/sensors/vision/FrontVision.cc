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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Desc: Frontal camera.
 * Author: Eurico Pedrosa
 * Date: 09 Sept 2011
 */

#include <sstream>
#include <cmath>

#include "simulator/csim-0.1.0/server/Global.hh"
#include "simulator/csim-0.1.0/server/GazeboError.hh"
#include "simulator/csim-0.1.0/server/physics/Body.hh"
#include "simulator/csim-0.1.0/server/World.hh"

#include "simulator/csim-0.1.0/server/sensors/SensorFactory.hh"
#include "simulator/csim-0.1.0/server/sensors/vision/FrontVision.hh"
#include "simulator/libs/rtdb/rtdb_sim.h"
#include "simulator/libs/rtdb/rtdb_user.h"

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("frontvision", FrontVision);

//////////////////////////////////////////////////////////////////////////////
// Constructor
FrontVision::FrontVision(Body *body)
    : Sensor(body)
{
	this->ball = NULL;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
FrontVision::~FrontVision()
{
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void FrontVision::LoadChild( XMLConfigNode *node )
{
	this->ballModelName = std::string();
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void FrontVision::InitChild()
{
	this->selfID = this->GetParentModel()->GetSelfID();

	// Without ID defined it will not run.
	if ( this->selfID < 1 ) {
		gzerr(0) << "Agent ID is missing, FrontVision will not update\n";
		return;
	}

	Model* ballModel = World::Instance()->GetModelByName( this->ballModelName );

	if ( ballModel == NULL ){
		this->ball = NULL;
		gzerr(0) << "Ball model not found. Ball detection disabled" << std::endl;

	}else{
		// I assume there is only one body, makes no sense otherwise.
		this->ball = ballModel->GetBody();
	}

	fvinfo.clear();
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void FrontVision::FiniChild()
{
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void FrontVision::UpdateChild()
{
	// Without ID and ball defined it will not update
	if ( this->selfID < 1 )  return;
	if ( this->ball == NULL) return;

	Pose3d ppose = this->body->GetAbsPose();
	Pose3d bpose = this->ball->GetAbsPose();
	// Resolve ball position
	Pose3d ballRelPosition = bpose - ppose;

	double hFOV = 0.6640;
	double vFOV = 0.5595;

	double angle;
	// verify if the ball is in the camera FOV
	angle = std::atan2( ballRelPosition.pos.x, ballRelPosition.pos.y );
	if ( std::fabs(angle) > hFOV  ){
		if ( fvinfo.ball[0].cyclesNotVisible < INT_MAX )
			fvinfo.ball[0].cyclesNotVisible++;
		return;
	}

	double height = bpose.pos.z - 0.49; // subtract sensor height
	//height = height < 0.0 ?  0.0 : height;

	angle = std::atan2( height, ballRelPosition.pos.y );
	if ( angle < 0.0 || angle > vFOV  ){
		if ( fvinfo.ball[0].cyclesNotVisible < INT_MAX )
			fvinfo.ball[0].cyclesNotVisible++;
		return;
	}

	fvinfo.ball[0].position = Vec(ballRelPosition.pos.x,ballRelPosition.pos.y);
	fvinfo.ball[0].height = bpose.pos.z;
	fvinfo.ball[0].cyclesNotVisible = 0;

	DB_put_in(this->selfID, this->selfID, FRONT_VISION_INFO, &(this->fvinfo), 0);
}




