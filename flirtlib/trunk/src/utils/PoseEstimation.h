/* *
 * FLIRTLib - Fast Laser Interesting Region Transform Library
 * Copyright (C) 2009-2010 Gian Diego Tipaldi and Kai O. Arras
 *
 * This file is part of FLIRTLib.
 *
 * FLIRTLib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FLIRTLib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FLIRTLib.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef POSEESTIMATION_H_
#define POSEESTIMATION_H_

#include <geometry/point.h>
#include <vector>
#include <utility>


/** Function to compute the Euclidean transformation between two set of points. The transformation is computed in closed form minimizing the squared residual reprojection error. */
double compute2DPose(const std::vector< std::pair<Point2D, Point2D> > &correspondences, OrientedPoint2D& transformation);

/** Function to compute the Euclidean transformation between two set of points. The transformation is computed in closed form minimizing the squared residual reprojection error. */
inline OrientedPoint2D compute2DPose(const std::vector< std::pair<Point2D, Point2D> > &correspondences)
    {OrientedPoint2D result; compute2DPose(correspondences, result); return result;}

#endif
