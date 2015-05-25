/*
 * GeometryTransformer.h
 *
 *  Created on: 15.11.2014
 *      Author: tobi
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_GEOMETRYTRANSFORMER_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_GEOMETRYTRANSFORMER_H_

#include <tuple>

#include "SystemConfig.h"
#include "container/CNPoint2D.h"
using namespace std;

namespace msl {

	class GeometryCalculator {
	public:

		virtual ~GeometryCalculator(){};
//		static pair<double, double> allo2Ego(pair<double, double>& p, tuple<double, double, double>& ownPos);
		static double deltaAngle(double angle1, double angle2);
		static bool isInsideRectangle(CNPoint2D rectPointA, CNPoint2D rectPointB, CNPoint2D point);
		static bool isInsidePolygon(vector<CNPoint2D> polygon, int n, CNPoint2D point);

	private:
		GeometryCalculator();
		static bool onSegment(CNPoint2D p, CNPoint2D q, CNPoint2D r);
		static int orientation(CNPoint2D p, CNPoint2D q, CNPoint2D r);
		static bool doIntersect(CNPoint2D p1, CNPoint2D q1, CNPoint2D p2, CNPoint2D q2);
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_GEOMETRYTRANSFORMER_H_ */
