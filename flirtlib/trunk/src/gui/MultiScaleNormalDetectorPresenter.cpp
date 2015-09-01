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

#include "MultiScaleNormalDetectorPresenter.h"
#include "MultiScaleNormalDetectorPresenter.moc"

#include <iostream>

MultiScaleNormalDetectorPresenter::MultiScaleNormalDetectorPresenter(NormalDetector* detector, ParameterWidget* detectorParameter):
    MultiScaleDetectorPresenter(detector, detectorParameter)
{
    m_detectorParameter->addIntParameter("windowSize", "Size of the normal window", 3, 3, 11, 2);

    syncronize();
    reconnect();
}

void MultiScaleNormalDetectorPresenter::setDetectorParameter(ParameterWidget* detectorParameter){
    disconnect(m_detectorParameter, 0, this, 0);
    MultiScaleDetectorPresenter::setDetectorParameter(detectorParameter);
    m_detectorParameter->addIntParameter("windowSize", "Size of the normal window", 3, 3, 11, 2);
    m_detectorParameter = detectorParameter;
    syncronize();
    reconnect();
}



void MultiScaleNormalDetectorPresenter::changeParameter(const QString& name){
    if(!QString::compare(name, "windowSize")){
	int guiValue;
	bool valid = m_detectorParameter->getIntValue("windowSize", guiValue);
	if(valid) {changeWindowSize(guiValue);}
    } else {
	MultiScaleDetectorPresenter::changeParameter(name);
    }
}

void MultiScaleNormalDetectorPresenter::changeWindowSize(int size){
    NormalDetector *detector = dynamic_cast<NormalDetector *>(m_detector);
    if(!detector || !m_detectorParameter) return;
    if((int)detector->getWindowSize() != size || m_detectorParameter->getIntValue("windowSize") != size){
	detector->setWindowSize(size);
	m_detectorParameter->setIntValue("windowSize", detector->getWindowSize());
	emit detectorChanged();
    }
}

void MultiScaleNormalDetectorPresenter::syncronize(){
    NormalDetector *detector = dynamic_cast<NormalDetector *>(m_detector);
    if(detector && m_detectorParameter){
	m_detectorParameter->setIntValue("windowSize", detector->getWindowSize());
    }
    MultiScaleDetectorPresenter::syncronize();
}

void MultiScaleNormalDetectorPresenter::reconnect(){
    disconnect(m_detectorParameter, 0, this, 0);
    connect(m_detectorParameter, SIGNAL(parameterChanged(const QString&)), this, SLOT(changeParameter(const QString&)));
}
