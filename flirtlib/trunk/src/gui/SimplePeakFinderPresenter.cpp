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

#include "SimplePeakFinderPresenter.h"
#include "SimplePeakFinderPresenter.moc"

#include <iostream>

SimplePeakFinderPresenter::SimplePeakFinderPresenter(SimplePeakFinder* peakFinder, ParameterWidget* peakParameter):
    PeakFinderPresenter(peakFinder, peakParameter)
{
    m_peakFinderParameter->addDoubleParameter("minValue", "Minimum value", 0., 0.);
    m_peakFinderParameter->addDoubleParameter("minDifference", "Minimum difference", 0., 0., 10e16, 4, 0.01);
    syncronize();
    reconnect();
}

void SimplePeakFinderPresenter::setPeakFinder(PeakFinder* peakFinder){
    m_peakFinder = peakFinder;
    syncronize();
}

void SimplePeakFinderPresenter::setPeakFinderParameter(ParameterWidget* peakParameter){
    disconnect(m_peakFinderParameter, 0, this, 0);
    m_peakFinderParameter = peakParameter;
    m_peakFinderParameter->clearParameterMap();
    m_peakFinderParameter->addDoubleParameter("minValue", "Minimum value", 0., 0.);
    m_peakFinderParameter->addDoubleParameter("minDifference", "Minimum difference", 0., 0., 10e16, 4, 0.01);
    syncronize();
    reconnect();
}

void SimplePeakFinderPresenter::changeParameter(const QString& name){
    if(!QString::compare(name, "minValue")){
	double guiValue;
	bool valid = m_peakFinderParameter->getDoubleValue("minValue", guiValue);
	if(valid) {changeMinValue(guiValue);}
    } else if(!QString::compare(name, "minDifference")){
	double guiValue;
	bool valid = m_peakFinderParameter->getDoubleValue("minDifference", guiValue);
	if(valid) {changeMinDifference(guiValue);}
    }
}


void SimplePeakFinderPresenter::changeMinValue(double value){
    SimplePeakFinder *peakFinder = (SimplePeakFinder *) m_peakFinder;
    double guiValue; 
    m_peakFinderParameter->getDoubleValue(QString("minValue"), guiValue);
    if(peakFinder->getMinValue() != value ||  guiValue != value){
	peakFinder->setMinValue(value);
	m_peakFinderParameter->setDoubleValue("minValue", peakFinder->getMinValue());
	emit peakFinderChanged();
    }
}

void SimplePeakFinderPresenter::changeMinDifference(double value){
    SimplePeakFinder *peakFinder = (SimplePeakFinder *) m_peakFinder;
    double guiValue; 
    m_peakFinderParameter->getDoubleValue("minDifference", guiValue);
    if(peakFinder->getMinDifference() != value || guiValue != value){
	peakFinder->setMinDifference(value);
	m_peakFinderParameter->setDoubleValue("minDifference", peakFinder->getMinDifference());
	emit peakFinderChanged();
    }
}

void SimplePeakFinderPresenter::syncronize(){
    SimplePeakFinder *peakFinder = (SimplePeakFinder *) m_peakFinder;
    m_peakFinderParameter->setDoubleValue("minValue", peakFinder->getMinValue());
    m_peakFinderParameter->setDoubleValue("minDifference", peakFinder->getMinDifference());
}

void SimplePeakFinderPresenter::reconnect(){
    connect(m_peakFinderParameter, SIGNAL(parameterChanged(const QString&)), this, SLOT(changeParameter(const QString&)));
}
