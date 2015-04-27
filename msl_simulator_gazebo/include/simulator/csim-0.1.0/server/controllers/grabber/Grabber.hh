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
 
/* Desc: CAMABADA barrier sensor
 * Author: Eurico F. Pedrosa <efp@ua.pt>
 * Date: 8 March 2010
 */
 
#ifndef GRABBER_HH
#define GRABBER_HH

#include "simulator/csim-0.1.0/server/controllers/Controller.hh"
#include "simulator/csim-0.1.0/server/Entity.hh"

namespace csim
{
/// \addtogroup gazebo_controller
/// \{
/** \defgroup controller_stub controller_stub

  \brief A stubbed out controller.

  Copy this example code when creating a new controller
  \{
*/

    using namespace gazebo;

/// \brief A stubbed out controller.
class Grabber : public Controller
{ 
     
  /// Constructor
  public: Grabber(Entity *parent );

  /// Destructor
  public: virtual ~Grabber();

  /// Load the controller
  /// \param node XML config node
  /// \return 0 on success
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// Init the controller
  /// \return 0 on success
  protected: virtual void InitChild();

  /// Update the controller
  /// \return 0 on success
  protected: virtual void UpdateChild();

  /// Finalize the controller
  /// \return 0 on success
  protected: virtual void FiniChild();

  /// The parent Model
private:
  BarrierSensor *barrier;
  
  // Grabber area
  float angle; 
  float distance;
  
  int selfID;
};

/** \} */
/// \}

}

#endif

