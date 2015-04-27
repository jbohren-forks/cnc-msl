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
 */
/* Desc: Base class for all sensors
 * Author: Nathan Koenig
 * Date: 25 May 2007
 * SVN: $Id: Sensor.cc 8474 2009-12-18 17:26:23Z natepak $
 */

#include "simulator/csim-0.1.0/server/Timer.hh"
#include "simulator/csim-0.1.0/server/controllers/Controller.hh"
#include "simulator/csim-0.1.0/libgazebo/gazebo.h"
#include "simulator/csim-0.1.0/server/GazeboError.hh"
#include "simulator/csim-0.1.0/server/GazeboMessage.hh"
#include "simulator/csim-0.1.0/server/physics/Body.hh"
#include "simulator/csim-0.1.0/server/World.hh"
#include "simulator/csim-0.1.0/server/controllers/ControllerFactory.hh"
#include "simulator/csim-0.1.0/server/Simulator.hh"
#include "simulator/csim-0.1.0/server/sensors/Sensor.hh"
#include "simulator/csim-0.1.0/server/physics/PhysicsEngine.hh"
#include "simulator/csim-0.1.0/server/Global.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Sensor::Sensor(Body *body)
    : Entity(body)
{
  this->body = body;
  this->controller = NULL;
  this->active = true;

  this->world = World::Instance();
  this->simulator = Simulator::Instance();

  Param::Begin(&this->parameters);
  this->updateRateP = new ParamT<double>("updateRate", 0, 0);
  this->logDataP  = new ParamT<bool>("logData", false, 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Sensor::~Sensor()
{
  delete this->updateRateP;
  delete this->logDataP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the sensor
void Sensor::Load()
{
  this->nameP->Load();
  this->updateRateP->Load();
  this->logDataP->Load();

  if (**(this->updateRateP) == 0)
    this->updatePeriod = 0.0;
  else
    this->updatePeriod = 1.0 / **(updateRateP);

  //TODO: XML STUFF
  this->LoadController();
  this->LoadChild();

  double updateRate  = 0; //node->GetDouble("updateRate", 0, 0);
  if (updateRate == 0)
    this->updatePeriod = 0.0; // no throttling if updateRate is 0
  else
    this->updatePeriod = 1.0 / updateRate;
  this->lastUpdate = Simulator::Instance()->GetSimTime();

}

////////////////////////////////////////////////////////////////////////////////
/// Save the sensor info in XML format
void Sensor::Save(std::string &prefix, std::ostream &stream)
{
  std::string p = prefix + "  ";

  stream << prefix << "<sensor:" << this->typeName << " name=\"" << this->nameP->GetValue() << "\">\n";

  stream << prefix << *(this->updateRateP) << "\n";

  this->SaveChild(prefix, stream);

  if (this->controller)
    this->controller->Save(p, stream);

  stream << prefix << "</sensor:" << this->typeName << ">\n";
}
 
////////////////////////////////////////////////////////////////////////////////
/// Initialize the sensor
void Sensor::Init()
{
  if (this->controller)
    this->controller->Init();

  this->lastUpdate = Simulator::Instance()->GetSimTime();

  this->InitChild();
}

////////////////////////////////////////////////////////////////////////////////
/// Update the sensor
void Sensor::Update()
{
  //DiagnosticTimer timer("Sensor[" + this->GetName() + "] Update");

  Time physics_dt = this->world->GetPhysicsEngine()->GetStepTime();

  if (((this->simulator->GetSimTime() - this->lastUpdate - this->updatePeriod)/physics_dt) >= 0)
  {
    this->UpdateChild();
    this->lastUpdate = this->simulator->GetSimTime();
  }

  // update any controllers that are children of sensors, e.g. ros_bumper
  if (this->controller)
    this->controller->Update();
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the sensor
void Sensor::Fini()
{
  if (this->controller)
    this->controller->Fini();
  this->FiniChild();
}

////////////////////////////////////////////////////////////////////////////////
/// Load a controller helper function
void Sensor::LoadController()
{
	//TODO:XML STUFF
//  if (!node)
//  {
//    gzmsg(9) << this->GetName() << " sensor has no controller.\n";
//    return;
//  }


  //Iface *iface;
  //XMLConfigNode *childNode;
  std::ostringstream stream;

  // Get the controller's type
  //TODO XML STUFF
 std::string controllerType = "";//node->GetName();

  // Get the unique name of the controller
  std::string controllerName = "";//node->GetString("name","",1);

  // Create the interface
  /*if ( (childNode = node->GetChildByNSPrefix("interface")) )
  {
    // Get the type of the interface (eg: laser)
    std::string ifaceType = childNode->GetName();

    // Get the name of the iface
    std::string ifaceName = childNode->GetString("name","",1);

    // Use the factory to get a new iface based on the type
    iface = IfaceFactory::NewIface(ifaceType);

    // Create the iface
    iface->Create(World::Instance()->GetGzServer(), ifaceName);

  }
  else
  {
    stream << "No interface defined for " << controllerName << "controller";
    gzthrow(stream.str());
  }*/
  
  // See if the controller is in a plugin
  std::string pluginName = ""; //node->GetString("plugin","",0);
  if (pluginName != "")
    ControllerFactory::LoadPlugin(pluginName, controllerType);

  // Create the controller based on it's type
  this->controller = ControllerFactory::NewController(controllerType, this);

  // Load the controller
  this->controller->Load();
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Set whether the sensor is active or not
void Sensor::SetActive(bool value)
{
  this->active = value;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Set whether the sensor is active or not
bool Sensor::IsActive()
{
  return this->active;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the current pose
Pose3d Sensor::GetPose() const
{
  return this->body->GetAbsPose();
}
///////////////////////////////////////////////////////////////////////////////
/// Get the name of the interfaces define in the sensor controller
void Sensor::GetInterfaceNames(std::vector<std::string>& list) const
{
  controller->GetInterfaceNames(list);
}


