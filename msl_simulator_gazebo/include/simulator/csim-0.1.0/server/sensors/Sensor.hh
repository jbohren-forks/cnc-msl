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
/* Desc: Base class for all sensors
 * Author: Nathan Koenig
 * Date: 25 May 2007
 * SVN: $Id: Sensor.hh 8474 2009-12-18 17:26:23Z natepak $
 */

#ifndef SENSOR_HH
#define SENSOR_HH

#include "simulator/csim-0.1.0/server/Param.hh"
#include "simulator/csim-0.1.0/server/Pose3d.hh"
#include "simulator/csim-0.1.0/server/Entity.hh"
#include "simulator/csim-0.1.0/server/Time.hh"

namespace gazebo
{
//TODO XML STUFF
//  class XMLConfigNode;
  class Body;
  class World;
  class Simulator;
  class Controller;

  /// \addtogroup gazebo_sensor
  /// \brief Base class for sensors
  /// \{
  
  /// \brief Base class for sensors
  class Sensor : public Entity
  {
    /// \brief  Constructor
    public: Sensor(Body *body);
  
    /// \brief  Destructor
    public: virtual ~Sensor();
  
    ///  \brief Load the sensor
    /// \param node XMLConfigNode pointer
    //TODO XML STUFF
    public: virtual void Load();

    /// \brief Save the sensor info in XML format
    public: void Save(std::string &prefix, std::ostream &stream);

    /// \brief Child save function
    protected: virtual void SaveChild(std::string &prefix,std::ostream &stream) {}
  
    /// \brief  Initialize the sensor
    public: void Init();
  
    /// \brief  Update the sensor
    public: void Update();
  
    /// \brief  Finalize the sensor
    public: void Fini();

    /// \brief Get the type of the sensor
    public: std::string GetSensorType(){return typeName;}

    /// \brief Get the current pose
    public: virtual Pose3d GetPose() const;

    /// \brief Get the name of the interfaces define in the sensor controller
    public: void GetInterfaceNames(std::vector<std::string>& list) const;
 
    /// \brief Set whether the sensor is active or not
    public: virtual void SetActive(bool value);
    public: bool IsActive();

    /// \brief  Load the child sensor
    //TODO XML STUFF
    protected: virtual void LoadChild(/*XMLConfigNode * /*node*/) {};
  
    /// \brief  Initialize the child
    protected: virtual void InitChild() {};
  
    /// \brief  Update the child
    protected: virtual void UpdateChild() {};
  
    /// \brief Finalize the child
    protected: virtual void FiniChild() {};
  
    /// \brief Load a controller for this sensor
    /// \param node XML configure parameter node
    //TODO XML STUFF
    private: void LoadController();
  
    /// The body this sensor is attached to
    protected: Body *body;
    protected: World *world;
    protected: Simulator *simulator;

    /// \brief Pointer to the controller of the sensor
    protected: Controller *controller;

    /// \brief True if active
    protected: bool active;

    protected: ParamT<double> *updateRateP;
    protected: ParamT<bool> *logDataP;
    protected: Time updatePeriod;
    protected: Time lastUpdate;
    protected: std::string typeName;
  };
  /// \}
}
#endif
