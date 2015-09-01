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

#include "ShapeContext.h"


Descriptor* ShapeContext::clone() const{
    return new ShapeContext(*this);
}
	
void ShapeContext::getFlatDescription(std::vector<double>& description) const {
	description.clear();
	for(unsigned int i = 0; i < m_histogram.size(); i++){
		for(unsigned int j = 0; j < m_histogram[i].size(); j++){
	    description.push_back(m_histogram[i][j]);
		}
	}
}

double ShapeContext::distance(const Descriptor* descriptor) const {
    const ShapeContext *shapeContext = dynamic_cast<const ShapeContext *>(descriptor);
    if(!m_distanceFunction || !shapeContext){
	return 10e16;
    }
    return m_distanceFunction->distance(this->getHistogram(), shapeContext->getHistogram());
}

ShapeContextGenerator::ShapeContextGenerator(double minRho, double maxRho, unsigned int binRho, unsigned int binPhi)
{
    setEdges(minRho, maxRho, binRho, binPhi);
}

ShapeContextGenerator::ShapeContextGenerator(const std::vector<double>& rhoEdges, const std::vector<double>& phiEdges):
    m_rhoEdges(rhoEdges),
    m_phiEdges(phiEdges)
{

}

void ShapeContextGenerator::setEdges(double minRho, double maxRho, unsigned int binRho, unsigned int binPhi){
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


Descriptor* ShapeContextGenerator::describe(const InterestPoint& point, const LaserReading& reading){
    return describe(point.getPosition(), reading);
}
	
Descriptor* ShapeContextGenerator::describe(const OrientedPoint2D& point, const LaserReading& reading){
    unsigned int accumulator = 0;
    ShapeContext * shape = new ShapeContext();
    shape->getHistogram().resize(m_phiEdges.size() - 1, std::vector<double>(m_rhoEdges.size() - 1, 0.));
    for(unsigned int i = 0; i < reading.getWorldCartesian().size(); i++){
		Point2D difference = reading.getWorldCartesian()[i] - point;
		double distance = hypot(difference.x, difference.y);
		if ((distance >= m_rhoEdges[0] && distance < m_rhoEdges[m_rhoEdges.size() - 1])){
			for(unsigned int rho = 0; rho < m_rhoEdges.size() - 1; rho++){
				if((distance < m_rhoEdges[rho + 1] && distance >= m_rhoEdges[rho])){
					double angle = atan2(difference.y, difference.x);
					angle = normAngle(angle - point.theta, -M_PI);
					for(unsigned int phi = 0; phi < m_phiEdges.size() - 1; phi++){
						if(angle < m_phiEdges[phi + 1] && angle >= m_phiEdges[phi]){
							shape->getHistogram()[phi][rho] += 1.;
							accumulator += 1;
						}
					}
				}
			}
		}
    }
    int size = shape->getHistogram().size() * shape->getHistogram().front().size();
    for(unsigned int i = 0; i < shape->getHistogram().size(); i++){
	for(unsigned int j = 0; j < shape->getHistogram()[i].size(); j++){
	    shape->getHistogram()[i][j] = accumulator ? shape->getHistogram()[i][j]/double(accumulator) : 1./double(size);
	}
    }
    shape->setDistanceFunction(m_distanceFunction);
    return shape;
}
	
