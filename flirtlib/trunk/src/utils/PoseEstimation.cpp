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

#include "PoseEstimation.h"

double compute2DPose(const std::vector< std::pair<Point2D, Point2D> > &correspondences, OrientedPoint2D& transformation)
{
    Point2D point1mean, point2mean;
    for(unsigned int i = 0; i < correspondences.size(); i++){
	point1mean = point1mean + correspondences[i].first;
	point2mean = point2mean + correspondences[i].second;
    }
    point1mean = point1mean * (1./double(correspondences.size()));
    point2mean = point2mean * (1./double(correspondences.size()));
    
    double A = 0, B = 0;
    for(unsigned int i = 0; i < correspondences.size(); i++){
	Point2D delta1 = correspondences[i].first - point1mean;
	Point2D delta2 = correspondences[i].second - point2mean;
	A += delta1 * delta2;
	B += delta1.ortho() * delta2;
    }
    
    A /= double(correspondences.size());
    B /= double(correspondences.size());
    
    double denom = sqrt(A*A + B*B);
    double sinalpha1 =  B/denom;
    double cosalpha1 = -A/denom;
    double sinalpha2 = -sinalpha1;
    double cosalpha2 = -cosalpha1;
    
    Point2D point1rotated(cosalpha1 * point1mean.x - sinalpha1 * point1mean.y,
			  sinalpha1 * point1mean.x + cosalpha1 * point1mean.y);
    Point2D point2rotated(cosalpha2 * point1mean.x - sinalpha2 * point1mean.y,
			  sinalpha2 * point1mean.x + cosalpha2 * point1mean.y);
    
    Point2D translation1(point2mean - point1rotated);
    Point2D translation2(point2mean - point2rotated);

    double error1 = 0, error2 = 0;
    for(unsigned int i = 0; i < correspondences.size(); i++){
	const Point2D& point1 = correspondences[i].first;
	const Point2D& point2 = correspondences[i].second;
	Point2D delta1 = point2 - Point2D(cosalpha1 * point1.x - sinalpha1 * point1.y, sinalpha1 * point1.x + cosalpha1 * point1.y) - translation1;
	Point2D delta2 = point2 - Point2D(cosalpha2 * point1.x - sinalpha2 * point1.y, sinalpha2 * point1.x + cosalpha2 * point1.y) - translation2;
	error1 = error1 + delta1 * delta1;
	error2 = error2 + delta2 * delta2;
    }
    if(error1 <= error2){
	transformation.x = translation1.x; transformation.y = translation1.y; transformation.theta = atan2(sinalpha1, cosalpha1);
	return error1;
    } else {
	transformation.x = translation2.x; transformation.y = translation2.y; transformation.theta = atan2(sinalpha2, cosalpha2);
	return error2;
    }
}
