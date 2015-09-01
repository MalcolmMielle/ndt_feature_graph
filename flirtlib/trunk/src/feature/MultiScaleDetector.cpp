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

#include "MultiScaleDetector.h"


MultiScaleDetector::MultiScaleDetector(const PeakFinder* _peak, unsigned int _scales, double _sigma, double step, SmoothingFilterFamily _filterType):
    m_peakFinder(_peak),
    m_scaleNumber(_scales),
    m_baseSigma(_sigma),
    m_sigmaStep(step),
    m_useMaxRange(false),
    m_filterType(_filterType)
{
    computeFilterBank();
}

unsigned int MultiScaleDetector::detect(const LaserReading& reading, std::vector<InterestPoint*>& point) const{
    std::vector<double> signal;
    std::vector< std::vector<double> > signalSmooth;
    std::vector< std::vector<double> > signalDiff;
    std::vector< std::vector<unsigned int> > indexes;
    return detect(reading, point, signal, signalSmooth, signalDiff, indexes);
}

unsigned int MultiScaleDetector::detect(const LaserReading& reading, std::vector<InterestPoint*>& point,
				   std::vector< double >& signal,
				   std::vector< std::vector<double> >& signalSmooth,
				   std::vector< std::vector<double> >& signalDiff,
				   std::vector< std::vector<unsigned int> >& indexes) const
{
    std::vector<unsigned int> maxRangeMapping;
    computeSignal(reading, signal, maxRangeMapping);
    detect(signal, signalSmooth, signalDiff, indexes);
    return computeInterestPoints(reading, signal, point, indexes, maxRangeMapping);
}

void MultiScaleDetector::detect(const std::vector<double>& signal,
				   std::vector< std::vector<double> >& signalSmooth,
				   std::vector< std::vector<double> >& signalDiff,
				   std::vector< std::vector<unsigned int> >& indexes) const
{
    signalSmooth.resize(m_scaleNumber);
    signalDiff.resize(m_scaleNumber);
    indexes.resize(m_scaleNumber);
    for(unsigned int i = 0; i < m_filterBank.size(); i++){
	int offsetRange = floor((int)m_filterBank[i].size()/2.0);
	if(offsetRange > signal.size()) continue;
	signalSmooth[i] = convolve1D(signal, m_filterBank[i], -offsetRange); 
	signalDiff[i] = convolve1D(signalSmooth[i], m_differentialBank[i], -1);
	for(unsigned int j = offsetRange + 1; j < signal.size() - offsetRange - 1; j++){
	    if(m_peakFinder->isPeak(signalDiff[i], j)){
		indexes[i].push_back(j);
	    }
	}
    }
}

void MultiScaleDetector::computeFilterBank(){
    m_filterBank.resize(m_scaleNumber);
    m_scales.resize(m_scaleNumber);
    for(unsigned int i = 0; i < m_filterBank.size(); i++){
	m_scales[i] = m_baseSigma * pow(m_sigmaStep, i);
	unsigned int kernelSize = 2*ceil(m_scales[i])+1;
	if(kernelSize < MIN_KERNEL_SIZE)
	    kernelSize = MIN_KERNEL_SIZE;
	else if(kernelSize > MAX_KERNEL_SIZE)
	    kernelSize = MAX_KERNEL_SIZE;
	switch(m_filterType){
	    case GAUSSIAN:
		m_filterBank[i] = gaussianKernel1D(m_scales[i], kernelSize); break;
	    case BESSEL:
		m_filterBank[i] = besselKernel1D(m_scales[i], kernelSize); break;
	    default:
		m_filterBank[i] = besselKernel1D(m_scales[i], kernelSize); break;
	}
    }
}
