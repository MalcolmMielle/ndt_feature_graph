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

#ifndef REGRESSION_H_
#define REGRESSION_H_

#include <vector>
#include <geometry/point.h>
#include <cmath>

/** 
 * Simple functions to perform line fitting 
 *
 * @author Gian Diego Tipaldi
 */


/**
 * Struct defining the parameters of a line;
 *
 * @author Gian Diego Tipaldi
 */

struct LineParameters{
    double rho; /**< The distance of the line from the origin of the axis. */
    double alpha; /**< The angle of the normal vector of the line. */
    double error; /**< The residual error of the fitted line. */
};

/** 
 * Fits a line in the weighted least squares sense to the points, in cartesian coordinates, with the corresponding weights, 
 * minimizing the weighted perpendicular errors from the points to the line. 
 */

LineParameters computeNormals(const std::vector<Point2D>& points, const std::vector<double>& weights);

/** 
 * Fits a line in the weighted least squares sense to the points, in cartesian coordinates, 
 * minimizing the perpendicular errors from the points to the line. 
 */

LineParameters computeNormals(const std::vector<Point2D>& points);

/** 
 * Fits a line in the weighted least squares sense at the center with the points, in cartesian coordinates, 
 * minimizing the perpendicular errors from the points to the line. 
 */


LineParameters computeNormals(const Point2D& center, const std::vector<Point2D>& _points);

#endif
