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

#include "RangeDetector.h"
#include <utils/Regression.h>


RangeDetector::RangeDetector(const PeakFinder* peak, unsigned int scales, double sigma, double step, SmoothingFilterFamily filterType):
    MultiScaleDetector(peak, scales, sigma, step, filterType)
{
    computeDifferentialBank();
}

void RangeDetector::computeDifferentialBank(){
    m_differentialBank.resize(m_scaleNumber, std::vector<double>(3));
    for(unsigned int i = 0; i < m_differentialBank.size(); i++){
		m_differentialBank[i][0] = m_scales[i]*1; 
		m_differentialBank[i][1] = -m_scales[i]*2; 
		m_differentialBank[i][2] = m_scales[i]*1;
    }
}

void RangeDetector::computeSignal(const LaserReading& reading, std::vector<double>& signal, std::vector<unsigned int>& maxRangeMapping) const{
    signal.reserve(reading.getRho().size());
    maxRangeMapping.reserve(reading.getRho().size());
    for(unsigned int i = 0; i < reading.getRho().size(); i++){
		if(reading.getRho()[i] < reading.getMaxRange()){ 
			signal.push_back(reading.getRho()[i]);
			maxRangeMapping.push_back(i);
		} else if (m_useMaxRange){
			signal.push_back(reading.getMaxRange());
			maxRangeMapping.push_back(i);
		}
    }
}

unsigned int RangeDetector::computeInterestPoints(const LaserReading& reading, const std::vector<double>& signal, std::vector<InterestPoint*>& point, 
						  std::vector< std::vector<unsigned int> >& indexes, std::vector<unsigned int>& maxRangeMapping) const
{
    point.clear();
    point.reserve(reading.getRho().size());
    const std::vector<Point2D>& worldPoints = reading.getWorldCartesian();
    for(unsigned int i = 0; i < indexes.size(); i++){
		for(unsigned int j = 0; j < indexes[i].size(); j++){
			OrientedPoint2D pose;
			unsigned int pointIndex = maxRangeMapping[indexes[i][j]];
			
			// Reomoving the detection in the background and pushing it to the foreground
			double rangeBefore = (pointIndex > 1)? reading.getRho()[pointIndex - 1] : reading.getMaxRange();
			double rangeAfter = (pointIndex < worldPoints.size() - 1)? reading.getRho()[pointIndex + 1] : reading.getMaxRange();
			double rangeCurrent = reading.getRho()[pointIndex];
			if(rangeBefore < rangeAfter){
				if(rangeBefore < rangeCurrent){
					pointIndex = pointIndex - 1;
				}
			} else if(rangeAfter < rangeCurrent){
				pointIndex = pointIndex + 1;
			}
			
			// Removing max range reading
			if(reading.getRho()[pointIndex] >= reading.getMaxRange()){
				continue;
			}

			pose.x =  (reading.getWorldCartesian()[pointIndex]).x;
			pose.y =  (reading.getWorldCartesian()[pointIndex]).y;
			pose.theta = 0.0;
			
			bool exists = false;
			for(unsigned int k = 0; !exists && k < point.size(); k++){
				exists = exists || (fabs(pose.x - point[k]->getPosition().x) <= 0.2 &&  fabs(pose.y - point[k]->getPosition().y) <= 0.2);
			}
			if(exists) continue;

	    unsigned int first = indexes[i][j] - floor((int)m_filterBank[i].size()/2.0);
	    unsigned int last = indexes[i][j] + floor((int)m_filterBank[i].size()/2.0);
	    std::vector<Point2D> support(last - first + 1);
	    for(unsigned int p = 0; p < support.size(); p++) {
		support[p] = Point2D(worldPoints[maxRangeMapping[p + first]]);
	    }
			
			LineParameters param = computeNormals(support);
			pose.theta = normAngle(param.alpha, - M_PI);

			double maxDistance = -1e20;
			for(unsigned int k = 0; k < support.size(); k++){
				double distance = sqrt((pose.x - support[k].x)*(pose.x - support[k].x) + (pose.y - support[k].y)*(pose.y - support[k].y));
				maxDistance = maxDistance < distance ? distance : maxDistance;
			}
			InterestPoint *interest = new InterestPoint(pose, maxDistance);
	// 	    InterestPoint *interest = new InterestPoint(pose, m_scales[i]);
			interest->setScaleLevel(i);
			interest->setSupport(support);
			point.push_back(interest);
		}
    }
    return point.size();
}

