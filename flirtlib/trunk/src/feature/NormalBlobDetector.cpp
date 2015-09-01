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

#include "NormalBlobDetector.h"


NormalBlobDetector::NormalBlobDetector(const PeakFinder* peak, unsigned int scales, double sigma, double step, unsigned int window, SmoothingFilterFamily filterType):
    NormalDetector(peak, scales, sigma, step, window, filterType)
{
    computeDifferentialBank();
}

void NormalBlobDetector::computeDifferentialBank(){
    m_differentialBank.resize(m_scaleNumber, std::vector<double>(3));
    for(unsigned int i = 0; i < m_differentialBank.size(); i++){
	m_differentialBank[i][0] = m_scales[i]*1; 
	m_differentialBank[i][1] = -m_scales[i]*2; 
	m_differentialBank[i][2] = m_scales[i]*1;
    }
}

