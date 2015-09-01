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

#include "BetaGridPresenter.h"
#include "BetaGridPresenter.moc"

#include <iostream>

BetaGridPresenter::BetaGridPresenter(BetaGridGenerator* descriptor, ParameterWidget* peakParameter):
    DescriptorPresenter(descriptor, peakParameter)
{
    m_descriptorParameter->addDoubleParameter("minRho", "Minimum radial distance", 0.05, 0.);
    m_descriptorParameter->addDoubleParameter("maxRho", "Maximum radial distance", 0.4, 0.);
    m_descriptorParameter->addIntParameter("binRho", "Number of radial bins", 4, 1);
    m_descriptorParameter->addIntParameter("binPhi", "Number of angular bins", 12, 1);
    m_descriptorParameter->addEnumParameter("distanceFunction", "Histogram distance function");
    syncronize();
    reconnect();
}

void BetaGridPresenter::setDescriptor(DescriptorGenerator* descriptor){
    m_descriptor = descriptor;
    syncronize();
}

void BetaGridPresenter::setDescriptorParameter(ParameterWidget* peakParameter){
    disconnect(m_descriptorParameter, 0, this, 0);
    m_descriptorParameter = peakParameter;
    m_descriptorParameter->clearParameterMap();
    m_descriptorParameter->addDoubleParameter("minRho", "Minimum radial distance", 0.05, 0.);
    m_descriptorParameter->addDoubleParameter("maxRho", "Maximum radial distance", 0.4, 0.);
    m_descriptorParameter->addIntParameter("binRho", "Number of radial bins", 4, 1);
    m_descriptorParameter->addIntParameter("binPhi", "Number of angular bins", 12, 1);
    m_descriptorParameter->addEnumParameter("distanceFunction", "Histogram distance function");
    syncronize();
    reconnect();
}

void BetaGridPresenter::insertDistanceFunction(const QString& name, const HistogramDistance<double>* distanceFunction){
    m_distanceFunctionNames.append(name);
    m_distanceFunctions.push_back(distanceFunction);
    m_descriptorParameter->insertEnumValue("distanceFunction", name);
    if(m_distanceFunctions.size() == 1){ 
	m_currentDistanceFunction = distanceFunction;
	m_currentDistanceFunctionIndex = 0;
	BetaGridGenerator *descriptor = (BetaGridGenerator *) m_descriptor;
	descriptor->setDistanceFunction(m_currentDistanceFunction);
    }
}

void BetaGridPresenter::changeParameter(const QString& name){
    if(!QString::compare(name, "minRho")){
	double guiValue;
	bool valid = m_descriptorParameter->getDoubleValue("minRho", guiValue);
	if(valid) {changeMinRho(guiValue);}
    } else if(!QString::compare(name, "maxRho")){
	double guiValue;
	bool valid = m_descriptorParameter->getDoubleValue("maxRho", guiValue);
	if(valid) {changeMaxRho(guiValue);}
    }else if(!QString::compare(name, "binRho")){
	int guiValue;
	bool valid = m_descriptorParameter->getIntValue("binRho", guiValue);
	if(valid) {changeBinRho(guiValue);}
    }else if(!QString::compare(name, "binPhi")){
	int guiValue;
	bool valid = m_descriptorParameter->getIntValue("binPhi", guiValue);
	if(valid) {changeBinPhi(guiValue);}
    }else if(!QString::compare(name, "distanceFunction")){
	int guiValue;
	bool valid = m_descriptorParameter->getEnumValue("distanceFunction", guiValue);
	if(valid) {changeDistanceFunction(guiValue);}
    }
}


void BetaGridPresenter::changeMinRho(double value){
    BetaGridGenerator *descriptor = (BetaGridGenerator *) m_descriptor;
    double guiValue; 
    m_descriptorParameter->getDoubleValue(QString("minRho"), guiValue);
    if(descriptor->getMinRho() != value ||  guiValue != value){
	descriptor->setEdges(value, descriptor->getMaxRho(), descriptor->getBinRho(), descriptor->getBinPhi());
	m_descriptorParameter->setDoubleValue("minRho", descriptor->getMinRho());
	emit descriptorChanged();
    }
}

void BetaGridPresenter::changeMaxRho(double value){
    BetaGridGenerator *descriptor = (BetaGridGenerator *) m_descriptor;
    double guiValue; 
    m_descriptorParameter->getDoubleValue("maxRho", guiValue);
    if(descriptor->getMaxRho() != value || guiValue != value){
	descriptor->setEdges(descriptor->getMinRho(), value, descriptor->getBinRho(), descriptor->getBinPhi());
	m_descriptorParameter->setDoubleValue("maxRho", descriptor->getMaxRho());
	emit descriptorChanged();
    }
}

void BetaGridPresenter::changeBinRho(int value){
    BetaGridGenerator *descriptor = (BetaGridGenerator *) m_descriptor;
    int guiValue; 
    m_descriptorParameter->getIntValue(QString("binRho"), guiValue);
    if(descriptor->getBinRho() != (unsigned int)value ||  guiValue != value){
	descriptor->setEdges(descriptor->getMinRho(), descriptor->getMaxRho(), value, descriptor->getBinPhi());
	m_descriptorParameter->setIntValue("binRho", descriptor->getBinRho());
	emit descriptorChanged();
    }
}

void BetaGridPresenter::changeBinPhi(int value){
    BetaGridGenerator *descriptor = (BetaGridGenerator *) m_descriptor;
    int guiValue; 
    m_descriptorParameter->getIntValue(QString("binPhi"), guiValue);
    if(descriptor->getBinPhi() != (unsigned int)value ||  guiValue != value){
	descriptor->setEdges(descriptor->getMinRho(), descriptor->getMaxRho(), descriptor->getBinRho(), value);
	m_descriptorParameter->setIntValue("binPhi", descriptor->getBinPhi());
	emit descriptorChanged();
    }
}

void BetaGridPresenter::changeDistanceFunction(int value){
    BetaGridGenerator *descriptor = (BetaGridGenerator *) m_descriptor;
    if(m_currentDistanceFunctionIndex != value){
	m_currentDistanceFunctionIndex = value;
	m_currentDistanceFunction = m_distanceFunctions[m_currentDistanceFunctionIndex];
	descriptor->setDistanceFunction(m_currentDistanceFunction);
	m_descriptorParameter->setEnumValue("distanceFunction", m_currentDistanceFunctionIndex);
	emit descriptorChanged();
    }
}

void BetaGridPresenter::syncronize(){
    BetaGridGenerator *descriptor = (BetaGridGenerator *) m_descriptor;
    m_descriptorParameter->setDoubleValue("minRho", descriptor->getMinRho());
    m_descriptorParameter->setDoubleValue("maxRho", descriptor->getMaxRho());
    m_descriptorParameter->setIntValue("binRho", descriptor->getBinRho());
    m_descriptorParameter->setIntValue("binPhi", descriptor->getBinPhi());
    if(m_distanceFunctions.size()){
	m_descriptorParameter->clearEnumParameter("distanceFunction");
	for(int i = 0; i < m_distanceFunctionNames.size(); i++){
	    m_descriptorParameter->insertEnumValue("distanceFunction", m_distanceFunctionNames[i]);
	}
	m_descriptorParameter->setEnumValue("peakFinder", m_currentDistanceFunctionIndex);
	descriptor->setDistanceFunction(m_currentDistanceFunction);
    }
}

void BetaGridPresenter::reconnect(){
    connect(m_descriptorParameter, SIGNAL(parameterChanged(const QString&)), this, SLOT(changeParameter(const QString&)));
}
