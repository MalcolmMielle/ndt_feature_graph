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

#include "Regression.h"

LineParameters computeNormals(const std::vector<Point2D>& _points, const std::vector<double>& _weights){
    if (_weights.size()!=_points.size()) return computeNormals(_points);
    
    LineParameters result;
    
    double sumWeights = 0;
    for(unsigned int i = 0; i < _weights.size(); i++) sumWeights += _weights[i];
    
    double meanWeightX = 0, meanWeightY = 0;
    for(unsigned int i = 0; i < _weights.size(); i++) {
	meanWeightX += 1.0/(_weights[i]*_weights[i])* _points[i].x;
	meanWeightY += 1.0/(_weights[i]*_weights[i])* _points[i].y;
    }
    meanWeightX = meanWeightX / sumWeights;
    meanWeightY = meanWeightY / sumWeights;
    
    // alpha
    double nominator = 0, denominator = 0, errorTerm = 0;
    for(unsigned int i = 0; i < _weights.size(); i++) {
	nominator += 1.0/(_weights[i]*_weights[i])*(_points[i].x - meanWeightX)*(_points[i].y - meanWeightY);
	denominator += 1.0/(_weights[i]*_weights[i])*((_points[i].y - meanWeightY)*(_points[i].y - meanWeightY) - 
	    (_points[i].x - meanWeightX)*(_points[i].x - meanWeightX));
	errorTerm += 1.0/(_weights[i]*_weights[i])*((_points[i].y - meanWeightY)*(_points[i].y - meanWeightY) + 
	    (_points[i].x - meanWeightX)*(_points[i].x - meanWeightX));
    }
    
    result.alpha = 0.5 * atan2(-2*nominator,denominator);

    // rho
    result.rho = meanWeightX * cos(result.alpha) + meanWeightY * sin(result.alpha);
    
    // normalize to get the solution facing the robot
    if(result.rho < 0){
	result.alpha += M_PI;
	result.rho = -result.rho;
    }
    // Compute the error of fit
    result.error = 0.5 * (errorTerm - sqrt(4 * nominator * nominator + denominator * denominator));
    
    return result;  
}

LineParameters computeNormals(const std::vector<Point2D>& _points) {
    return computeNormals(_points, std::vector<double>(_points.size(),1.0));
}

LineParameters computeNormals(const Point2D& center, const std::vector<Point2D>& _points) {
    LineParameters result;
        
    //TODO Check with the distance as weight;
    std::vector<double> _weights(_points.size(), 1.);
    
    double sumWeights = 0;
    for(unsigned int i = 0; i < _weights.size(); i++){
// 	_weights[i] = 1./((_points[i] - center) * (_points[i] - center));
	sumWeights += _weights[i];
    }

    double meanWeightX = 0, meanWeightY = 0;
    for(unsigned int i = 0; i < _weights.size(); i++) {
	meanWeightX += 1.0/(_weights[i]*_weights[i])* _points[i].x;
	meanWeightY += 1.0/(_weights[i]*_weights[i])* _points[i].y;
    }
    meanWeightX = center.x; // meanWeightX / sumWeights;
    meanWeightY = center.y; // meanWeightY / sumWeights;

    // alpha
    double nominator = 0, denominator = 0, errorTerm = 0;
    for(unsigned int i = 0; i < _weights.size(); i++) {
	nominator += 1.0/(_weights[i]*_weights[i])*(_points[i].x - meanWeightX)*(_points[i].y - meanWeightY);
	denominator += 1.0/(_weights[i]*_weights[i])*((_points[i].y - meanWeightY)*(_points[i].y - meanWeightY) - 
	    (_points[i].x - meanWeightX)*(_points[i].x - meanWeightX));
	errorTerm += 1.0/(_weights[i]*_weights[i])*((_points[i].y - meanWeightY)*(_points[i].y - meanWeightY) + 
	    (_points[i].x - meanWeightX)*(_points[i].x - meanWeightX));
	
    }

    result.alpha = 0.5 * atan2(-2*nominator,denominator);

    // rho
    result.rho = meanWeightX * cos(result.alpha) + meanWeightY * sin(result.alpha);

    // normalize to get the solution facing the robot
    if(result.rho < 0){
	result.alpha += M_PI;
	result.rho = -result.rho;
    }

    // Compute the error of fit
    result.error = 0.5 * (errorTerm - sqrt(4 * nominator * nominator + denominator * denominator));

    return result;  

}