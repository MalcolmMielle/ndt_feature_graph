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

#include "RansacPresenter.h"
#include "RansacPresenter.moc"

#include <iostream>

RansacPresenter::RansacPresenter(RansacFeatureSetMatcher* featureSetMatcher, ParameterWidget* featureSetMatcherParameter):
    FeatureSetMatcherPresenter(featureSetMatcher, featureSetMatcherParameter)
{
    m_featureSetMatcherParameter->addDoubleParameter("successProbability", "Probability of success", 0.95, 0., 1., 2, 0.01);
    m_featureSetMatcherParameter->addDoubleParameter("inlierProbability", "Probability of inlier", 0.5, 0., 1., 2, 0.01);
    m_featureSetMatcherParameter->addDoubleParameter("distanceThreshold", "Threshold on the descriptor", 0.4, 0., 1., 2, 0.01);
    m_featureSetMatcherParameter->addBoolParameter("adaptive", "Adaptive iteration estimation", false);
    syncronize();
    reconnect();
}

void RansacPresenter::setFeatureSetMatcher(AbstractFeatureSetMatcher* featureSetMatcher){
    m_featureSetMatcher = featureSetMatcher;
    syncronize();
}

void RansacPresenter::setFeatureSetMatcherParameter(ParameterWidget* featureSetMatcherParameter){
    disconnect(m_featureSetMatcherParameter, 0, this, 0);
    m_featureSetMatcherParameter = featureSetMatcherParameter;
    m_featureSetMatcherParameter->clearParameterMap();
    m_featureSetMatcherParameter->addDoubleParameter("successProbability", "Probability of success", 0.95, 0., 1., 2, 0.01);
    m_featureSetMatcherParameter->addDoubleParameter("inlierProbability", "Probability of inlier", 0.5, 0., 1., 2, 0.01);
    m_featureSetMatcherParameter->addDoubleParameter("distanceThreshold", "Threshold on the descriptor", 0.4, 0., 1., 2, 0.01);
    m_featureSetMatcherParameter->addBoolParameter("adaptive", "Adaptive iteration estimation", false);
    syncronize();
    reconnect();
}

void RansacPresenter::changeParameter(const QString& name){
    if(!QString::compare(name, "successProbability")){
	double guiValue;
	bool valid = m_featureSetMatcherParameter->getDoubleValue("successProbability", guiValue);
	if(valid) {changeSuccessProbability(guiValue);}
    } else if(!QString::compare(name, "inlierProbability")){
	double guiValue;
	bool valid = m_featureSetMatcherParameter->getDoubleValue("inlierProbability", guiValue);
	if(valid) {changeInlierProbability(guiValue);}
    } else if(!QString::compare(name, "distanceThreshold")){
	double guiValue;
	bool valid = m_featureSetMatcherParameter->getDoubleValue("distanceThreshold", guiValue);
	if(valid) {changeDistanceThreshold(guiValue);}
    } else if(!QString::compare(name, "adaptive")){
	bool guiValue;
	bool valid = m_featureSetMatcherParameter->getBoolValue("adaptive", guiValue);
	if(valid) {changeAdaptive(guiValue ? Qt::Checked : Qt::Unchecked);}
    } else {
	FeatureSetMatcherPresenter::changeParameter(name);
    }
}


void RansacPresenter::changeSuccessProbability(double value){
    RansacFeatureSetMatcher *featureSetMatcher = (RansacFeatureSetMatcher *) m_featureSetMatcher;
    double guiValue; 
    m_featureSetMatcherParameter->getDoubleValue("successProbability", guiValue);
    if(featureSetMatcher->getSuccessProbability() != value ||  guiValue != value){
	featureSetMatcher->setSuccessProbability(value);
	m_featureSetMatcherParameter->setDoubleValue("successProbability", featureSetMatcher->getSuccessProbability());
	emit featureSetMatcherChanged();
    }
}

void RansacPresenter::changeInlierProbability(double value){
    RansacFeatureSetMatcher *featureSetMatcher = (RansacFeatureSetMatcher *) m_featureSetMatcher;
    double guiValue; 
    m_featureSetMatcherParameter->getDoubleValue("inlierProbability", guiValue);
    if(featureSetMatcher->getInlierProbability() != value || guiValue != value){
	featureSetMatcher->setInlierProbability(value);
	m_featureSetMatcherParameter->setDoubleValue("inlierProbability", featureSetMatcher->getInlierProbability());
	emit featureSetMatcherChanged();
    }
}

void RansacPresenter::changeDistanceThreshold(double value){
    RansacFeatureSetMatcher *featureSetMatcher = (RansacFeatureSetMatcher *) m_featureSetMatcher;
    double guiValue; 
    m_featureSetMatcherParameter->getDoubleValue("distanceThreshold", guiValue);
    if(featureSetMatcher->getDistanceThreshold() != value || guiValue != value){
	featureSetMatcher->setDistanceThreshold(value);
	m_featureSetMatcherParameter->setDoubleValue("distanceThreshold", featureSetMatcher->getDistanceThreshold());
	emit featureSetMatcherChanged();
    }
}

void RansacPresenter::changeAdaptive(int value){
    RansacFeatureSetMatcher *featureSetMatcher = (RansacFeatureSetMatcher *) m_featureSetMatcher;
    bool check = value == Qt::Checked;
    if(featureSetMatcher->getAdaptive() != check || m_featureSetMatcherParameter->getBoolValue("adaptive") != check){
	featureSetMatcher->setAdaptive(check);
	m_featureSetMatcherParameter->setBoolValue("adaptive", value);
	emit featureSetMatcherChanged();
    }
}

void RansacPresenter::syncronize(){
    RansacFeatureSetMatcher *featureSetMatcher = (RansacFeatureSetMatcher *) m_featureSetMatcher;
    m_featureSetMatcherParameter->setDoubleValue("successProbability", featureSetMatcher->getSuccessProbability());
    m_featureSetMatcherParameter->setDoubleValue("distanceThreshold", featureSetMatcher->getDistanceThreshold());
    m_featureSetMatcherParameter->setDoubleValue("inlierProbability", featureSetMatcher->getInlierProbability());
    m_featureSetMatcherParameter->setBoolValue("adaptive", featureSetMatcher->getAdaptive());
    FeatureSetMatcherPresenter::syncronize();
}

void RansacPresenter::reconnect(){
    disconnect(m_featureSetMatcherParameter, 0, this, 0);
    connect(m_featureSetMatcherParameter, SIGNAL(parameterChanged(const QString&)), this, SLOT(changeParameter(const QString&)));
}
