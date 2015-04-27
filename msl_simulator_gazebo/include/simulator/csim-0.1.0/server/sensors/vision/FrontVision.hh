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
/* Desc: Stubbed out sensor
 * Author: Nate Koenig
 * Date: 05 Aug 2007
 * SVN: $Id: SensorStub.hh 7038 2008-09-24 18:04:30Z natepak $
 */

#ifndef FRONTVISION_HH
#define FRONTVISION_HH

#include <string>

#include "simulator/csim-0.1.0/server/Model.hh"
#include "simulator/csim-0.1.0/server/physics/Body.hh"
#include "simulator/csim-0.1.0/server/sensors/Sensor.hh"

// cambada includes
#include "simulator/csim-0.1.0/agentWorld/VisionInfo.h"

namespace gazebo
{
/// \addtogroup gazebo_sensor
/// \brief Stubbed out sensor
/// \{
/// \defgroup gazebo_sensor_stub Sensor Stub
/// \brief Stubbed out sensor
// \{


/// \brief Stubbed out  sensor
///
/// Copy this sensor to create your own
class FrontVision : public Sensor
{
  /// \brief Constructor
  public: FrontVision(Body *body);

  /// \brief Destructor
  public: virtual ~FrontVision();

  /// \brief Load the camera using parameter from an XMLConfig node
  /// \param node The XMLConfig node
  protected: virtual void LoadChild( XMLConfigNode *node );

  /// \brief Initialize the camera
  protected: virtual void InitChild();

  /// \brief Update the sensor information
  protected: virtual void UpdateChild();

  /// Finalize the camera
  protected: virtual void FiniChild();

  private:
  	  // Robot ID
  	  int selfID;
  	  FrontVisionInfo fvinfo;

  	  std::string ballModelName;
  	  Body* ball;


};

/// \}
/// \}
}
#endif

