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

#include "SensorStreamWidget.h"


SensorStreamWidget::SensorStreamWidget(QWidget * _parent):
    QFrame(_parent),
    m_isSeekable(false),
    m_position(0)
{
    m_layout = new QGridLayout(this);
    m_nextButton = new QPushButton("Next", this);
    m_sensorSlider = new QSlider(this);
    m_sensorBox = new QSpinBox(this);
    m_sensorBox->setMinimum(0);
    
    buildGui();
}

SensorStreamWidget::~SensorStreamWidget(){
    delete m_layout;
	delete m_nextButton;
	delete m_sensorSlider;
	delete m_sensorBox;
}
    
void SensorStreamWidget::seekPosition(int _position){
    if(_position == m_position){
		return;
    }
    m_position = _position;
    if(m_isSeekable){
		m_sensorSlider->setValue(_position);
		m_sensorBox->setValue(_position);
    }
    emit newReading(_position);
}

void SensorStreamWidget::nextPosition(){
    if(m_isSeekable){
	seekPosition(m_sensorSlider->value() + m_sensorSlider->singleStep());
    } else {    
	emit newReading();
    }
}

void SensorStreamWidget::seekable(bool _seek, unsigned int _size){
    m_isSeekable = _seek;
    m_sensorSlider->setEnabled(_seek);
    m_sensorBox->setEnabled(_seek);
    if(_size) {
    m_sensorSlider->setMaximum(_size);
    m_sensorBox->setMaximum(_size);
	}
}

void SensorStreamWidget::streamReady(){
    m_nextButton->setEnabled(true);
}


void SensorStreamWidget::buildGui(){
    m_layout->setSpacing(3);
    m_layout->setMargin(3);
    m_layout->addWidget(m_sensorSlider, 0,0);
    m_layout->addWidget(m_nextButton, 0,1);
    m_layout->addWidget(m_sensorBox, 0,2);
    
    m_sensorSlider->setEnabled(false);
    m_sensorSlider->setOrientation(Qt::Horizontal);
    
    m_nextButton->setEnabled(false);

    connect(m_nextButton, SIGNAL(clicked()), this, SLOT(nextPosition()));
    connect(m_sensorSlider, SIGNAL(valueChanged(int)), this, SLOT(seekPosition(int)));
    connect(m_sensorBox, SIGNAL(valueChanged(int)), this, SLOT(seekPosition(int)));
}

#include "SensorStreamWidget.moc"