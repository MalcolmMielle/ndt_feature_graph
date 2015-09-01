//
//
// FLIRTLib - Fast Laser Interesting Region Transform Library
// Copyright (C) 2009-2010 Gian Diego Tipaldi and Kai O. Arras
//
// This file is part of FLIRTLib.
//
// FLIRTLib is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// FLIRTLib is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with FLIRTLib.  If not, see <http://www.gnu.org/licenses/>.
//

#include "point.h"

Point2D::Point2D(): 
    x(0.0),
    y(0.0) 
{ 

}

Point2D::Point2D(double _x, double _y):
    x(_x),
    y(_y) 
{ 

}

OrientedPoint2D::OrientedPoint2D():
    Point2D(0.0,0.0),
    theta(0.0)
{

}

OrientedPoint2D::OrientedPoint2D(double _x, double _y, double _theta):
    Point2D(_x,_y),
    theta(normAngle(_theta))
{
    
}

OrientedPoint2D OrientedPoint2D::ominus() const
{
    double ctheta = cos(theta), stheta = sin(theta);
    return OrientedPoint2D(-x * ctheta - y * stheta,
			    x * stheta - y * ctheta,
			    normAngle(-theta));
}
    
OrientedPoint2D OrientedPoint2D::ominus(const OrientedPoint2D& point) const
{
    double ctheta = cos(theta), stheta = sin(theta);
    return OrientedPoint2D( (point.x - x) * ctheta + (point.y - y) * stheta,
			   -(point.x - x) * stheta + (point.y - y) * ctheta,
			     normAngle(point.theta - theta));

}
    
Point2D OrientedPoint2D::ominus(const Point2D& point) const
{
    double ctheta = cos(theta), stheta = sin(theta);
    return Point2D( (point.x - x) * ctheta + (point.y - y) * stheta,
		   -(point.x - x) * stheta + (point.y - y) * ctheta);

}
    
OrientedPoint2D OrientedPoint2D::oplus(const OrientedPoint2D& point) const
{
    double ctheta = cos(theta), stheta = sin(theta);
    return OrientedPoint2D(x + point.x * ctheta - point.y * stheta,
			   y + point.x * stheta + point.y * ctheta,
			   normAngle(theta + point.theta));
}

Point2D OrientedPoint2D::oplus(const Point2D& point) const
{
    double ctheta = cos(theta), stheta = sin(theta);
    return Point2D(x + point.x * ctheta - point.y * stheta,
		   y + point.x * stheta + point.y * ctheta);
}


double normAngle(double angle, double base) {
    double pi2 = 2*M_PI;
    double min2pi = base + pi2;
    while(angle>=min2pi) angle -= pi2;
    while(angle<base) angle += pi2;
    return angle;
}

