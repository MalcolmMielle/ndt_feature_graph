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

#include "DetectorChooserPresenter.h"
#include "DetectorChooserPresenter.moc"
/*
DetectorChooserWidget::DetectorChooserWidget(QWidget* parent):
    QWidget(parent),
    m_detector(this),
    m_detectorLabel("Type of detector",this),
    m_layout(this)
{
    buildGui();
}

void DetectorChooserWidget::changeDetector(int detector){
    m_detector.setCurrentIndex(detector);
}

void DetectorChooserWidget::buildGui(){
    m_layout.setSpacing(3);
    m_layout.setMargin(3);
    m_layout.addWidget(&m_detectorLabel,m_layout.rowCount(),0);
    m_layout.addWidget(&m_detector,m_layout.rowCount() - 1,1);
    
    m_detector.setInsertPolicy(QComboBox::NoInsert);
    m_detector.setEditable(false);

    connect(&m_detector, SIGNAL(activated(int)), this, SIGNAL(detectorChanged(int)));
}

void DetectorChooserWidget::resetDetector(){
    while(m_detector.count()) m_detector.removeItem(0);
}
*/

DetectorChooserPresenter::DetectorChooserPresenter(ParameterWidget* chooser):
    m_chooser(chooser),
    m_currentDetectorPresenter(NULL)
{
    m_chooser->addEnumParameter("detector", "Type of detector");
    syncronize();
    reconnect();
}


void DetectorChooserPresenter::insertDetector(const QString& name, DetectorPresenter* detector){
    m_detectorPresenterNames.push_back(name);
    m_detectorPresenters.push_back(detector);
    m_chooser->insertEnumValue("detector", name);
    if(m_detectorPresenters.size() == 1){ 
	m_currentDetectorPresenter = detector;
	m_currentDetectorPresenterIndex = 0;
	connect(m_currentDetectorPresenter, SIGNAL(detectorChanged()), this, SIGNAL(detectorChanged()));
	m_currentDetectorPresenter->activate();
    } else{
	detector->deactivate();
	m_currentDetectorPresenter->activate();
    }
}

void DetectorChooserPresenter::setChooser(ParameterWidget* chooser){
    disconnect(m_chooser, 0, this, 0);
    m_chooser = chooser;
    m_chooser->clearParameterMap();
    m_chooser->addEnumParameter("detector", "Type of detector");
    syncronize();
    reconnect();
}

void DetectorChooserPresenter::changeParameter(const QString& name){
    if(!QString::compare(name, "detector")){
	int guiValue;
	bool valid = m_chooser->getEnumValue("detector", guiValue);
	if(valid) {changeDetector(guiValue);}
    }
}

void DetectorChooserPresenter::changeDetector(int detector){
    if(m_currentDetectorPresenterIndex != detector){
	disconnect(m_currentDetectorPresenter, SIGNAL(detectorChanged()), this, SIGNAL(detectorChanged()));
	m_currentDetectorPresenter->deactivate();
	m_currentDetectorPresenterIndex = detector;
	m_currentDetectorPresenter = m_detectorPresenters[m_currentDetectorPresenterIndex];
	m_chooser->setEnumValue("detector", m_currentDetectorPresenterIndex);
	m_currentDetectorPresenter->activate();
	connect(m_currentDetectorPresenter, SIGNAL(detectorChanged()), this, SIGNAL(detectorChanged()));
	emit detectorChanged();
    }
}


void DetectorChooserPresenter::syncronize(){
    if(m_detectorPresenters.size()){
	m_chooser->clearEnumParameter("detector");
	for(int i = 0; i < m_detectorPresenterNames.size(); i++){
	    m_chooser->insertEnumValue("detector", m_detectorPresenterNames[i]);
	}
	m_chooser->setEnumValue("detector", m_currentDetectorPresenterIndex);
	m_currentDetectorPresenter->activate();
    }
}

void DetectorChooserPresenter::reconnect(){
    connect(m_chooser, SIGNAL(parameterChanged(const QString&)), this, SLOT(changeParameter(const QString&)));
}
