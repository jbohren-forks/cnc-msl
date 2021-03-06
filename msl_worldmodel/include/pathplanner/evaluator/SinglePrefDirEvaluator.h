/*
 * SinglePrefDirEvaluator.h
 *
 *  Created on: May 14, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_SINGLEPREFDIREVALUATOR_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_SINGLEPREFDIREVALUATOR_H_

#include <pathplanner/evaluator/PathEvaluator.h>

namespace msl
{

	class SinglePrefDirEvaluator : public PathEvaluator
	{
	public:
		SinglePrefDirEvaluator(PathPlanner* planner);
		virtual ~SinglePrefDirEvaluator();
		double eval(double costsSoFar, shared_ptr<VoronoiNet> voronoi, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> path,
					geometry::CNPoint2D startPos, geometry::CNPoint2D goal, shared_ptr<SearchNode> currentNode, shared_ptr<SearchNode> nextNode);
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_PATHPLANNER_EVALUATOR_SINGLEPREFDIREVALUATOR_H_ */
