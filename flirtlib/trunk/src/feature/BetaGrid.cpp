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

#include "BetaGrid.h"

#include <sstream>
#include <cstdio>
#include <sys/types.h>
#include <sys/stat.h>


Descriptor* BetaGrid::clone() const{
    return new BetaGrid(*this);
}
	
void BetaGrid::getFlatDescription(std::vector<double>& description) const {
    description.clear();
    double sum = 0.;
    for(unsigned int i = 0; i < m_histogram.size(); i++){
	for(unsigned int j = 0; j < m_histogram[i].size(); j++){
	    double mean = (getHit()[i][j]+1)/(getHit()[i][j] + getMiss()[i][j]+2);
	    double variance = (getHit()[i][j]+1) * (getMiss()[i][j]+1)/
			      ( (getHit()[i][j] + getMiss()[i][j]+2) * (getHit()[i][j] + getMiss()[i][j]+2) *
			      (getHit()[i][j] + getMiss()[i][j] + 3));
			mean = getHistogram()[i][j];
	    description.push_back(mean);
// 	    description.push_back(m_hit[i][j]);
	    sum += description.back();
	}
    }
    if(sum < 10e-6){
	for(std::vector<double>::iterator it = description.begin(); it != description.end(); it++){
	    *it = 10e-10;
	}
    } else {
      	for(std::vector<double>::iterator it = description.begin(); it != description.end(); it++){
	    *it /= sum;
	}
    }
}

void BetaGrid::getWeightedFlatDescription(std::vector<double>& description, std::vector<double>& weight) const {
    description.clear();
    double sum = 0.;
    for(unsigned int i = 0; i < m_histogram.size(); i++){
	for(unsigned int j = 0; j < m_histogram[i].size(); j++){
	    double mean = (getHit()[i][j]+1)/(getHit()[i][j] + getMiss()[i][j]+2);
	    double variance = (getHit()[i][j]+1) * (getMiss()[i][j]+1)/
			      ( (getHit()[i][j] + getMiss()[i][j]+2) * (getHit()[i][j] + getMiss()[i][j]+2) *
			      (getHit()[i][j] + getMiss()[i][j] + 3));
	    description.push_back(mean);
	    weight.push_back(1./variance);
// 	    description.push_back(m_hit[i][j]);
	    sum += description.back();
	}
    }
/*    if(sum < 10e-6){
	for(std::vector<double>::iterator it = description.begin(); it != description.end(); it++){
	    *it = 10e-10;
	}
    } else {
      	for(std::vector<double>::iterator it = description.begin(); it != description.end(); it++){
	    *it /= sum;
	}
    }*/
}

double BetaGrid::distance(const Descriptor* descriptor) const {
    const BetaGrid *betaGrid = dynamic_cast<const BetaGrid *>(descriptor);
    if(!m_distanceFunction || !betaGrid){
	return 10e16;
    }
    return m_distanceFunction->distance(this->getHistogram(), betaGrid->getHistogram());
}

BetaGridGenerator::BetaGridGenerator(double minRho, double maxRho, unsigned int binRho, unsigned int binPhi)
{
    setEdges(minRho, maxRho, binRho, binPhi);
}

BetaGridGenerator::BetaGridGenerator(const std::vector<double>& rhoEdges, const std::vector<double>& phiEdges):
    m_rhoEdges(rhoEdges),
    m_phiEdges(phiEdges)
{

}

void BetaGridGenerator::setEdges(double minRho, double maxRho, unsigned int binRho, unsigned int binPhi){
    m_rhoEdges.resize(binRho+1);
    m_phiEdges.resize(binPhi+1);
    double minPhi = -M_PI, maxPhi = M_PI;
    for(unsigned int i = 0; i <= binRho; i++){
	m_rhoEdges[i] = minRho + double(i)*(maxRho - minRho)/double(binRho);
    }
    for(unsigned int i = 0; i <= binPhi; i++){
	m_phiEdges[i] = minPhi + double(i)*(maxPhi - minPhi)/double(binPhi);
    }
}


Descriptor* BetaGridGenerator::describe(const InterestPoint& point, const LaserReading& reading){
    return describe(point.getPosition(), reading);
}
	
Descriptor* BetaGridGenerator::describe(const OrientedPoint2D& point, const LaserReading& reading){
    BetaGrid * shape = new BetaGrid();
    shape->getHistogram().resize(m_phiEdges.size() - 1, std::vector<double>(m_rhoEdges.size() - 1, 0.));
    shape->getVariance().resize(m_phiEdges.size() - 1, std::vector<double>(m_rhoEdges.size() - 1, 0.));
    shape->getHit().resize(m_phiEdges.size() - 1, std::vector<double>(m_rhoEdges.size() - 1, 0.));
    shape->getMiss().resize(m_phiEdges.size() - 1, std::vector<double>(m_rhoEdges.size() - 1, 0.));
    
    const OrientedPoint2D& position = reading.getLaserPose();
    const Point2D delta = position.ominus(point);
    double alpha = atan2(delta.y, delta.x), alphaDiff = atan2(m_rhoEdges.back(), hypot(delta.x, delta.y));
    double minAlpha = alpha - alphaDiff, maxAlpha = alpha + alphaDiff;
    
    const std::vector<double>& readingPhi = reading.getPhi();
    double normalizer = double(readingPhi.size())/double(readingPhi.back() - readingPhi.front());
    unsigned int minIndex = normalizer * (minAlpha - readingPhi.front());
    minIndex = std::max(0, int(minIndex) - 1);
    unsigned int maxIndex = normalizer * (maxAlpha - readingPhi.front());
    maxIndex = std::min(readingPhi.size(), size_t(maxIndex + 2));
    
    unsigned int pointIndex = normalizer * double(alpha - readingPhi.front());
/*    std::cout << "min phi: " << readingPhi.front() << ", max phi = " << readingPhi.back() << std::endl;
    std::cout << "index double no norm= " << double(alpha - readingPhi.front())/double(readingPhi.front() - readingPhi.back()) << std::endl;
    std::cout << "index double norm = " << normalizer * double(alpha - readingPhi.front()) << std::endl;
    std::cout << "Point alpha: " << alpha << ", index = " << pointIndex << std::endl;*/
    
    
	for(unsigned int phi = 0; phi < m_phiEdges.size() - 1; phi++){
	double angleBorder = M_PI/180.*5.;
	double angle1 = m_phiEdges[phi] + point.theta - angleBorder;
	double angle2 = m_phiEdges[phi+1] + point.theta + angleBorder;
	double cos1 = cos(angle1), sin1 = sin(angle1), cos2 = cos(angle2), sin2 = sin(angle2);
/*	    shape->getMiss()[phi][rho] = 1;
	    shape->getHit()[phi][rho] = 1;*/
	double border = 0.05;
	const Point2D point1(point.x + cos1 * (m_rhoEdges[0]), point.y + sin1 * (m_rhoEdges[0]));
	const Point2D point2(point.x + cos1 * (m_rhoEdges[1] + border), point.y + sin1 * (m_rhoEdges[1] + border));
	const Point2D point3(point.x + cos2 * (m_rhoEdges[1] + border), point.y + sin2 * (m_rhoEdges[1] + border));
	const Point2D point4(point.x + cos2 * (m_rhoEdges[0]), point.y + sin2 * (m_rhoEdges[0]));
//  	    for(unsigned int i = 0; i < reading.getWorldCartesian().size(); i++){
	for(unsigned int i = minIndex; i < maxIndex; i++){
	    int accumulator = 0;
	    const Point2D& endPoint = reading.getWorldCartesian()[i];
	    if(reading.getRho()[i] >= reading.getMaxRange()) continue;
	    accumulator += intersectSegment2Segment(position, endPoint, point1, point2);
	    accumulator += intersectSegment2Arc(position, endPoint, point2, point3, point);
	    accumulator += intersectSegment2Segment(position, endPoint, point4, point3);
	    accumulator += intersectSegment2Arc(position, endPoint, point1, point4, point);
	    shape->getMiss()[phi][0] += accumulator >= 2.;
	    shape->getHit()[phi][0] += accumulator == 1.;
	}
	for(unsigned int rho = 1; rho < m_rhoEdges.size() - 1; rho++){
/*	    shape->getMiss()[phi][rho] = 1;
	    shape->getHit()[phi][rho] = 1;*/
// 	    double border = 0.05;
	    const Point2D point1(point.x + cos1 * (m_rhoEdges[rho] - border), point.y + sin1 * (m_rhoEdges[rho] - border));
	    const Point2D point2(point.x + cos1 * (m_rhoEdges[rho+1] + border), point.y + sin1 * (m_rhoEdges[rho+1] + border));
	    const Point2D point3(point.x + cos2 * (m_rhoEdges[rho+1] + border), point.y + sin2 * (m_rhoEdges[rho+1] + border));
	    const Point2D point4(point.x + cos2 * (m_rhoEdges[rho] - border), point.y + sin2 * (m_rhoEdges[rho] - border));
//  	    for(unsigned int i = 0; i < reading.getWorldCartesian().size(); i++){
	    for(unsigned int i = minIndex; i < maxIndex; i++){
		int accumulator = 0;
		const Point2D& endPoint = reading.getWorldCartesian()[i];
		if(reading.getRho()[i] >= reading.getMaxRange()) continue;
		accumulator += intersectSegment2Segment(position, endPoint, point1, point2);
		accumulator += intersectSegment2Arc(position, endPoint, point2, point3, point);
		accumulator += intersectSegment2Segment(position, endPoint, point4, point3);
		accumulator += intersectSegment2Arc(position, endPoint, point1, point4, point);
		shape->getMiss()[phi][rho] += accumulator >= 2.;
		shape->getHit()[phi][rho] += accumulator == 1.;
	    }
	}
    }
    
    for(unsigned int rho = 0; rho < m_rhoEdges.size() - 1; rho++){
	for(unsigned int phi = 0; phi < m_phiEdges.size() - 1; phi++){
	    if(shape->getHit()[phi][rho] != 0 || shape->getMiss()[phi][rho] != 0){
		shape->getHistogram()[phi][rho] = (shape->getHit()[phi][rho])/(shape->getHit()[phi][rho] + shape->getMiss()[phi][rho]);
		shape->getVariance()[phi][rho] = (shape->getHit()[phi][rho] * shape->getMiss()[phi][rho])/
						( (shape->getHit()[phi][rho] + shape->getMiss()[phi][rho]) * (shape->getHit()[phi][rho] + shape->getMiss()[phi][rho]) *
						(shape->getHit()[phi][rho] + shape->getMiss()[phi][rho] + 1));
	    } else{
		shape->getHistogram()[phi][rho] = 0.5;
		shape->getVariance()[phi][rho] = 1./(2 * 2 * 3);
	    }
	}
    }
    
    // Normalizing histogram
    double accumulator = 0.;
/*    double accumulator2 = 0.;
    double min = 1e16, max = 0.;
    double min2 = 1e16, max2 = 0.;*/
    for(unsigned int rho = 0; rho < m_rhoEdges.size() - 1; rho++){
	for(unsigned int phi = 0; phi < m_phiEdges.size() - 1; phi++){
	    accumulator += shape->getHistogram()[phi][rho];
/*	    min = min < shape->getHistogram()[phi][rho]? min : shape->getHistogram()[phi][rho];
	    max = max > shape->getHistogram()[phi][rho]? max : shape->getHistogram()[phi][rho];*/
	}
    }
//     std::cout << "BEFORE: accumumlator: " << accumulator << ", min: " << min << ", max: " << max << std::endl;
    
    for(unsigned int rho = 0; rho < m_rhoEdges.size() - 1; rho++){
	for(unsigned int phi = 0; phi < m_phiEdges.size() - 1; phi++){
	    shape->getHistogram()[phi][rho] = shape->getHistogram()[phi][rho]/accumulator;
/*	    accumulator2 += shape->getHistogram()[phi][rho];
	    min2 = min2 < shape->getHistogram()[phi][rho]? min2 : shape->getHistogram()[phi][rho];
	    max2 = max2 > shape->getHistogram()[phi][rho]? max2 : shape->getHistogram()[phi][rho];*/
	}
    }
//     std::cout << "AFTER : accumumlator: " << accumulator2 << ", min: " << min2 << ", max: " << max2 << std::endl;
    
    shape->setDistanceFunction(m_distanceFunction);
    return shape;
}

bool BetaGridGenerator::intersectSegment2Segment(const Point2D& segment1Start, const Point2D& segment1End, const Point2D& segment2Start, const Point2D& segment2End)
{
    Point2D segment1Direction = segment1End - segment1Start;
    Point2D segment2Direction = segment2End - segment2Start;
    Point2D delta = segment2Start - segment1Start;
    double dotPerpProduct = segment1Direction * segment2Direction.ortho();
    if (dotPerpProduct != 0.){
	double ascissa1 = (delta * segment1Direction.ortho()) / dotPerpProduct;
	double ascissa2 = (delta * segment2Direction.ortho()) / dotPerpProduct;
	return (ascissa1 >= 0 && ascissa1 <= 1 && ascissa2 >= 0 && ascissa2 <= 1 );
    } else if(delta * segment1Direction == 0.){
	double ascissa1 = (segment1Direction * delta) / (segment1Direction * segment1Direction);
	Point2D delta2 = segment2Direction - segment1Start;
	double ascissa2 = (segment1Direction * delta2) / (segment1Direction * segment1Direction);
	return (ascissa1 >= 0. && ascissa1 <= 1.) || (ascissa2 >= 0. && ascissa2 <= 1.);
    }
    return false;
}

int BetaGridGenerator::intersectSegment2Arc(const Point2D& segmentStart, const Point2D& segmentEnd, const Point2D& arcStart, const Point2D& arcEnd, const Point2D& arcCenter)
{
    Point2D segmentDirection = segmentEnd - segmentStart;
    Point2D delta = segmentStart - arcCenter;
    double radiusSquare = (arcStart - arcCenter) * (arcStart - arcCenter);
    double discriminant = (segmentDirection * delta) * (segmentDirection * delta) - (segmentDirection * segmentDirection) * (delta * delta - radiusSquare);
    if(discriminant == 0.){
	double ascissa = -(segmentDirection * delta)/(segmentDirection * segmentDirection);
	Point2D intersectionDelta = segmentStart + ascissa * segmentDirection - arcStart;
	Point2D arcDelta = arcEnd - arcStart;
	return (intersectionDelta.x * arcDelta.y >= intersectionDelta.y * arcDelta.x) * (ascissa >= 0. && ascissa <= 1.);
    } else if(discriminant > 0.){
	double ascissa1 = (-(segmentDirection * delta) + sqrt(discriminant))/(segmentDirection * segmentDirection);
	double ascissa2 = (-(segmentDirection * delta) - sqrt(discriminant))/(segmentDirection * segmentDirection);
	int result = 0;
	Point2D intersectionDelta1 = segmentStart + ascissa1 * segmentDirection - arcStart;
	Point2D intersectionDelta2 = segmentStart + ascissa2 * segmentDirection - arcStart;
	Point2D arcDelta = arcEnd - arcStart;
	result += (intersectionDelta1.x * arcDelta.y >= intersectionDelta1.y * arcDelta.x) * (ascissa1 >= 0. && ascissa1 <= 1.);
	result += (intersectionDelta2.x * arcDelta.y >= intersectionDelta2.y * arcDelta.x) * (ascissa2 >= 0. && ascissa2 <= 1.);
	return result;
    }
    return 0;
}

