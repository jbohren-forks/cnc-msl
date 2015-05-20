/*
 * Ball.h
 *
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_BALL_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_BALL_H_

#include "container/CNPoint2D.h"
#include "container/CNVelocity2D.h"

#include <memory>

#include "SystemConfig.h"

using namespace supplementary;

using namespace std;

namespace msl
{

	class MSLWorldModel;
	class Ball
	{
	public:
		Ball(MSLWorldModel* wm);
		virtual ~Ball();
		bool haveBall();
		shared_ptr<CNPoint2D> getAlloBallPosition();
		shared_ptr<CNPoint2D> getEgoBallPosition();
		shared_ptr<CNPoint2D> getEgoRawBallPosition();
		shared_ptr<CNVelocity2D> getEgoBallVelocity();
		void updateOnWorldModelData();
	private:
		shared_ptr<CNPoint2D> lastKnownBallPos;
		double HAVE_BALL_TOLERANCE_DRIBBLE;
		double KICKER_DISTANCE;
		double KICKER_ANGLE;
		double HAVE_BALL_MAX_ANGLE_DELTA;
		int hasBallIteration;
		bool hadBefore;
		bool hasBall;
		double haveBallDistanceDynamic;
		MSLWorldModel* wm;
		SystemConfig* sc;

	};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_BALL_H_ */
