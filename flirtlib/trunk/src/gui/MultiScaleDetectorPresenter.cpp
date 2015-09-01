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

#include "MultiScaleDetectorPresenter.h"
#include "MultiScaleDetectorPresenter.moc"

#include <iostream>

MultiScaleDetectorPresenter::MultiScaleDetectorPresenter(MultiScaleDetector* detector, ParameterWidget* detectorParameter):
    DetectorPresenter(detector, detectorParameter),
    m_currentPeakPresenter(NULL)
{
    QStringList filters;
    filters << "Gaussian" << "Bessel";

    m_detectorParameter->addBoolParameter("useMaxRange", "Use max range values", false);
    m_detectorParameter->addIntParameter("scale", "Number of scales", 1, 1, 10);
    m_detectorParameter->addDoubleParameter("sigma", "Base value of Sigma", 1.6, 0.0, 10e16, 2, 0.1);
    m_detectorParameter->addDoubleParameter("sigmaStep", "Step value of Sigma", 1.4, 0.0, 10e16, 2, 0.1);
    m_detectorParameter->addEnumParameter("filterType", "Type of smoothing operator", filters, 1);
    m_detectorParameter->addEnumParameter("peakFinder", "Type of peak finder");
    syncronize();
    reconnect();
}

void MultiScaleDetectorPresenter::activate(){
    DetectorPresenter::activate();
    if(m_currentPeakPresenter){
	m_currentPeakPresenter->activate();
    }
}

void MultiScaleDetectorPresenter::deactivate(){
    if(m_currentPeakPresenter){
	m_currentPeakPresenter->deactivate();
    }
    DetectorPresenter::deactivate();
}

void MultiScaleDetectorPresenter::insertPeakFinder(const QString& name, PeakFinderPresenter* peak){
    m_peakPresenterNames.append(name);
    m_peakPresenters.push_back(peak);
    m_detectorParameter->insertEnumValue("peakFinder", name);
    if(m_peakPresenters.size() == 1){ 
	m_currentPeakPresenter = peak;
	m_currentPeakPresenterIndex = 0;
	connect(m_currentPeakPresenter, SIGNAL(peakFinderChanged()), this, SIGNAL(detectorChanged()));
	m_currentPeakPresenter->activate();
    }else{
	peak->deactivate();
    }
}

void MultiScaleDetectorPresenter::setDetector(Detector* detector){
    m_detector = detector;
    syncronize();
}
	
void MultiScaleDetectorPresenter::setDetectorParameter(ParameterWidget* detectorParameter){
    QStringList filters;
    filters << "Gaussian" << "Bessel";

    m_detectorParameter->clearParameterMap();
    m_detectorParameter->addBoolParameter("useMaxRange", "Use max range values", false);
    m_detectorParameter->addIntParameter("scale", "Number of scales", 1, 1, 10);
    m_detectorParameter->addDoubleParameter("sigma", "Base value of Sigma", 1.6, 0.0, 10e16, 2, 0.1);
    m_detectorParameter->addDoubleParameter("sigmaStep", "Step value of Sigma", 1.4, 0.0, 10e16, 2, 0.1);
    m_detectorParameter->addEnumParameter("filterType", "Type of smoothing operator", filters, 1);
    m_detectorParameter->addEnumParameter("peakFinder", "Type of peak finder");

    disconnect(m_detectorParameter, 0, this, 0);
    m_detectorParameter = detectorParameter;
    syncronize();
    reconnect();
}

void MultiScaleDetectorPresenter::changeParameter(const QString& name){
    if(!QString::compare(name, "scale")){
	int guiValue;
	bool valid = m_detectorParameter->getIntValue("scale", guiValue);
	if(valid) {changeScale(guiValue);}
    } else if(!QString::compare(name, "useMaxRange")){
	bool guiValue;
	bool valid = m_detectorParameter->getBoolValue("useMaxRange", guiValue);
	if(valid) {changeMaxRange(guiValue ? Qt::Checked : Qt::Unchecked);}
    } else if(!QString::compare(name, "sigma")){
	double guiValue;
	bool valid = m_detectorParameter->getDoubleValue("sigma", guiValue);
	if(valid) {changeSigma(guiValue);}
    } else if(!QString::compare(name, "sigmaStep")){
	double guiValue;
	bool valid = m_detectorParameter->getDoubleValue("sigmaStep", guiValue);
	if(valid) {changeSigmaStep(guiValue);}
    } else if(!QString::compare(name, "filterType")){
	int guiValue;
	bool valid = m_detectorParameter->getEnumValue("filterType", guiValue);
	if(valid) {changeFilter(guiValue);}
    } else if(!QString::compare(name, "peakFinder")){
	int guiValue;
	bool valid = m_detectorParameter->getEnumValue("peakFinder", guiValue);
	if(valid) {changePeakFinder(guiValue);}
    }
}

void MultiScaleDetectorPresenter::changeScale(int scale){
    MultiScaleDetector *detector = (MultiScaleDetector *) m_detector;
    if((int)detector->getScaleNumber() != scale || m_detectorParameter->getIntValue("scale") != scale){
	detector->setScaleNumber(scale);
	m_detectorParameter->setIntValue("scale", detector->getScaleNumber());
	emit detectorChanged();
    }
}

void MultiScaleDetectorPresenter::changeMaxRange(int value){
    MultiScaleDetector *detector = (MultiScaleDetector *) m_detector;
    bool check = value == Qt::Checked;
    if(detector->getUseMaxRange() != check || m_detectorParameter->getBoolValue("useMaxRange") != check){
	detector->setUseMaxRange(check);
	m_detectorParameter->setBoolValue("useMaxRange", value);
	emit detectorChanged();
    }
}

void MultiScaleDetectorPresenter::changeSigma(double sigma){
    MultiScaleDetector *detector = (MultiScaleDetector *) m_detector;
    if(detector->getBaseSigma() != sigma || m_detectorParameter->getDoubleValue("sigma") != sigma){
	detector->setBaseSigma(sigma);
	m_detectorParameter->setDoubleValue("sigma", detector->getBaseSigma());
	emit detectorChanged();
    }
}

void MultiScaleDetectorPresenter::changeSigmaStep(double step){
    MultiScaleDetector *detector = (MultiScaleDetector *) m_detector;
    if(detector->getSigmaStep() != step || m_detectorParameter->getDoubleValue("sigmaStep") != step){
	detector->setSigmaStep(step);
	m_detectorParameter->setDoubleValue("sigmaStep", detector->getSigmaStep());
	emit detectorChanged();
    }
}

void MultiScaleDetectorPresenter::changeFilter(int filter){
    MultiScaleDetector *detector = (MultiScaleDetector *) m_detector;
    if(((int) detector->getFilterType()) != filter || m_detectorParameter->getEnumValue("filterType") != filter){
	detector->setFilterType((SmoothingFilterFamily) filter);
	m_detectorParameter->setEnumValue("filterType", detector->getFilterType());
	emit detectorChanged();
    }    
}

void MultiScaleDetectorPresenter::changePeakFinder(int peakFinder){
    MultiScaleDetector *detector = (MultiScaleDetector *) m_detector;
    if(m_currentPeakPresenterIndex != peakFinder){
	disconnect(m_currentPeakPresenter, SIGNAL(peakFinderChanged()), this, SIGNAL(detectorChanged()));
	m_currentPeakPresenter->deactivate();
	m_currentPeakPresenterIndex = peakFinder;
	m_currentPeakPresenter = m_peakPresenters[m_currentPeakPresenterIndex];
	detector->setPeakFinder(m_currentPeakPresenter->getPeakFinder());
	m_detectorParameter->setEnumValue("peakFinder", m_currentPeakPresenterIndex);
	m_currentPeakPresenter->activate();
	connect(m_currentPeakPresenter, SIGNAL(peakFinderChanged()), this, SIGNAL(detectorChanged()));
	emit detectorChanged();
    }
}


void MultiScaleDetectorPresenter::syncronize(){
    MultiScaleDetector *detector = (MultiScaleDetector *) m_detector;
    m_detectorParameter->setIntValue("scale", detector->getScaleNumber());
    m_detectorParameter->setDoubleValue("sigma", detector->getBaseSigma());
    m_detectorParameter->setDoubleValue("sigmaStep", detector->getSigmaStep());
    m_detectorParameter->setEnumValue("filterType", detector->getFilterType());
    m_detectorParameter->setBoolValue("useMaxRange", detector->getUseMaxRange());
    if(m_peakPresenters.size()){
	m_detectorParameter->clearEnumParameter("peakFinder");
	for(int i = 0; i < m_peakPresenterNames.size(); i++){
	    m_detectorParameter->insertEnumValue("peakFinder", m_peakPresenterNames[i]);
	}
	m_detectorParameter->setEnumValue("peakFinder", m_currentPeakPresenterIndex);
	detector->setPeakFinder(m_currentPeakPresenter->getPeakFinder());
	m_currentPeakPresenter->activate();
    }
}

void MultiScaleDetectorPresenter::reconnect(){
    disconnect(m_detectorParameter, 0, this, SLOT(changeParameter(const QString&)));
    connect(m_detectorParameter, SIGNAL(parameterChanged(const QString&)), this, SLOT(changeParameter(const QString&)));
}
