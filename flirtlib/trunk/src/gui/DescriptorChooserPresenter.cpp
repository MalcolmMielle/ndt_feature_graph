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

#include "DescriptorChooserPresenter.h"
#include "DescriptorChooserPresenter.moc"
/*
DescriptorChooserWidget::DescriptorChooserWidget(QWidget* parent):
    QWidget(parent),
    m_descriptor(this),
    m_descriptorLabel("Type of descriptor",this),
    m_layout(this)
{
    buildGui();
}

void DescriptorChooserWidget::changeDescriptor(int descriptor){
    m_descriptor.setCurrentIndex(descriptor);
}

void DescriptorChooserWidget::buildGui(){
    m_layout.setSpacing(3);
    m_layout.setMargin(3);
    m_layout.addWidget(&m_descriptorLabel,m_layout.rowCount(),0);
    m_layout.addWidget(&m_descriptor,m_layout.rowCount() - 1,1);
    
    m_descriptor.setInsertPolicy(QComboBox::NoInsert);
    m_descriptor.setEditable(false);

    connect(&m_descriptor, SIGNAL(activated(int)), this, SIGNAL(descriptorChanged(int)));
}

void DescriptorChooserWidget::resetDescriptor(){
    while(m_descriptor.count()) m_descriptor.removeItem(0);
}
*/

DescriptorChooserPresenter::DescriptorChooserPresenter(ParameterWidget* chooser):
    m_chooser(chooser),
    m_currentDescriptorPresenter(NULL)
{
    m_chooser->addEnumParameter("descriptor", "Type of descriptor");
    syncronize();
    reconnect();
}


void DescriptorChooserPresenter::insertDescriptor(const QString& name, DescriptorPresenter* descriptor){
    m_descriptorPresenterNames.push_back(name);
    m_descriptorPresenters.push_back(descriptor);
    m_chooser->insertEnumValue("descriptor", name);
    if(m_descriptorPresenters.size() == 1){ 
	m_currentDescriptorPresenter = descriptor;
	m_currentDescriptorPresenterIndex = 0;
	connect(m_currentDescriptorPresenter, SIGNAL(descriptorChanged()), this, SIGNAL(descriptorChanged()));
	m_currentDescriptorPresenter->activate();
    } else{
	descriptor->deactivate();
	m_currentDescriptorPresenter->activate();
    }
}

void DescriptorChooserPresenter::setChooser(ParameterWidget* chooser){
    disconnect(m_chooser, 0, this, 0);
    m_chooser = chooser;
    m_chooser->clearParameterMap();
    m_chooser->addEnumParameter("descriptor", "Type of descriptor");
    syncronize();
    reconnect();
}

void DescriptorChooserPresenter::changeParameter(const QString& name){
    if(!QString::compare(name, "descriptor")){
	int guiValue;
	bool valid = m_chooser->getEnumValue("descriptor", guiValue);
	if(valid) {changeDescriptor(guiValue);}
    }
}

void DescriptorChooserPresenter::changeDescriptor(int descriptor){
    if(m_currentDescriptorPresenterIndex != descriptor){
	disconnect(m_currentDescriptorPresenter, SIGNAL(descriptorChanged()), this, SIGNAL(descriptorChanged()));
	m_currentDescriptorPresenter->deactivate();
	m_currentDescriptorPresenterIndex = descriptor;
	m_currentDescriptorPresenter = m_descriptorPresenters[m_currentDescriptorPresenterIndex];
	m_chooser->setEnumValue("descriptor", m_currentDescriptorPresenterIndex);
	m_currentDescriptorPresenter->activate();
	connect(m_currentDescriptorPresenter, SIGNAL(descriptorChanged()), this, SIGNAL(descriptorChanged()));
	emit descriptorChanged();
    }
}


void DescriptorChooserPresenter::syncronize(){
    if(m_descriptorPresenters.size()){
	m_chooser->clearEnumParameter("descriptor");
	for(int i = 0; i < m_descriptorPresenterNames.size(); i++){
	    m_chooser->insertEnumValue("descriptor", m_descriptorPresenterNames[i]);
	}
	m_chooser->setEnumValue("descriptor", m_currentDescriptorPresenterIndex);
	m_currentDescriptorPresenter->activate();
    }
}

void DescriptorChooserPresenter::reconnect(){
    connect(m_chooser, SIGNAL(parameterChanged(const QString&)), this, SLOT(changeParameter(const QString&)));
}
