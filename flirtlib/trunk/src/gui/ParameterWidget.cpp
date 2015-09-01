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

#include "ParameterWidget.h"
#include "ParameterWidget.moc"

#include <iostream>

ParameterWidget::ParameterWidget(const QString& name, QWidget *parent):
    QWidget(parent),
    m_name(name),
    m_mapper(this)
{
    m_layout = new QGridLayout(this);
    m_layout->setSpacing(3);
    m_layout->setMargin(3);
    connect(&m_mapper, SIGNAL(mapped(const QString&)), this, SIGNAL(parameterChanged(const QString&)));
}

void ParameterWidget::addDoubleParameter(const QString& name, const QString& description, double initialValue, double minValue, double maxValue, int decimals, double step){
    ParameterPair pair;
    pair.description = new QLabel(description, this);
    QDoubleSpinBox *parameter = new QDoubleSpinBox(this);
    
    parameter->setValue(initialValue);
    parameter->setMinimum(minValue);
    parameter->setMaximum(maxValue);
    parameter->setDecimals(decimals);
    parameter->setSingleStep(step);

    pair.parameter = parameter;
    m_layout->addWidget(pair.description,m_layout->rowCount(), 0);
    m_layout->addWidget(pair.parameter,m_layout->rowCount() - 1, 1);
    
    connect(pair.parameter, SIGNAL(valueChanged(double)), &m_mapper, SLOT(map()));
    m_mapper.setMapping(pair.parameter, name);
    m_parameterMap.insert(name, pair);
}

void ParameterWidget::addIntParameter(const QString& name, const QString& description, int initialValue, int minValue, int maxValue, int step){
    ParameterPair pair;
    pair.description = new QLabel(description, this);
    QSpinBox *parameter = new QSpinBox(this);

    parameter->setValue(initialValue);
    parameter->setMinimum(minValue);
    parameter->setMaximum(maxValue);
    parameter->setSingleStep(step);

    pair.parameter = parameter;
    m_layout->addWidget(pair.description,m_layout->rowCount(),0);
    m_layout->addWidget(pair.parameter,m_layout->rowCount() - 1,1);
    
    connect(pair.parameter, SIGNAL(valueChanged(int)), &m_mapper, SLOT(map()));
    m_mapper.setMapping(pair.parameter, name);
    m_parameterMap.insert(name, pair);
}

void ParameterWidget::addBoolParameter(const QString& name, const QString& description, bool initialValue){
    ParameterPair pair;
    pair.description = new QLabel(description, this);
    QCheckBox *parameter = new QCheckBox(this);

    parameter->setCheckState(initialValue ? Qt::Checked : Qt::Unchecked);

    pair.parameter = parameter;
    m_layout->addWidget(pair.description,m_layout->rowCount(),0);
    m_layout->addWidget(pair.parameter,m_layout->rowCount() - 1,1);
    
    connect(pair.parameter, SIGNAL(stateChanged(int)), &m_mapper, SLOT(map()));
    m_mapper.setMapping(pair.parameter, name);
    m_parameterMap.insert(name, pair);
}

void ParameterWidget::addEnumParameter(const QString& name, const QString& description, QStringList& values, int initialValue){
    ParameterPair pair;
    pair.description = new QLabel(description, this);
    QComboBox *parameter = new QComboBox(this);

    parameter->insertItems(0, values);
    parameter->setInsertPolicy(QComboBox::NoInsert);
    parameter->setEditable(false);

    pair.parameter = parameter;
    m_layout->addWidget(pair.description,m_layout->rowCount(),0);
    m_layout->addWidget(pair.parameter,m_layout->rowCount() - 1,1);
    
    connect(pair.parameter, SIGNAL(activated(int)), &m_mapper, SLOT(map()));
    m_mapper.setMapping(pair.parameter, name);
    m_parameterMap.insert(name, pair);
}

void ParameterWidget::insertEnumValue(const QString& name, const QString& value){
    ParameterPair pair = m_parameterMap.value(name);
    QComboBox *parameter = dynamic_cast<QComboBox *>(pair.parameter);
    if(parameter){
	parameter->addItem(value);
    }
}

void ParameterWidget::removeEnumValue(const QString& name, const QString& value){
    ParameterPair pair = m_parameterMap.value(name);
    QComboBox *parameter = dynamic_cast<QComboBox *>(pair.parameter);
    if(parameter){
	int index = parameter->findText(value);
	if(index >= 0){
	    parameter->removeItem(index);
	}
    }
}

void ParameterWidget::clearEnumParameter(const QString& name){
    ParameterPair pair = m_parameterMap.value(name);
    QComboBox *parameter = dynamic_cast<QComboBox *>(pair.parameter);
    if(parameter){
	while(parameter->count()) parameter->removeItem(0);
    }
}

void ParameterWidget::removeParameter(QString& name){
    ParameterPair pair = m_parameterMap.value(name);
    disconnect(pair.parameter, 0, &m_mapper, 0);
    delete pair.parameter;
    delete pair.description;
    m_parameterMap.remove(name);
}

void ParameterWidget::clearParameterMap(){
    for(QHash<QString, ParameterPair>::iterator it = m_parameterMap.begin(); it != m_parameterMap.end(); ++it){
	ParameterPair pair = it.value();
	delete pair.parameter;
	delete pair.description;
    }
    m_parameterMap.clear();
}

bool ParameterWidget::getDoubleValue(const QString& name, double& result) const{
    ParameterPair pair = m_parameterMap.value(name);
    const QDoubleSpinBox *parameter = dynamic_cast<const QDoubleSpinBox *>(pair.parameter);
    if(parameter){
	result = parameter->value();
    }
    return parameter != NULL;
}

bool ParameterWidget::getIntValue(const QString& name, int& result) const{
    ParameterPair pair = m_parameterMap.value(name);
    const QSpinBox *parameter = dynamic_cast<const QSpinBox *>(pair.parameter);
    if(parameter){
	result = parameter->value();
    }
    return parameter != NULL;
}

bool ParameterWidget::getBoolValue(const QString& name, bool& result) const{
    ParameterPair pair = m_parameterMap.value(name);
    const QCheckBox *parameter = dynamic_cast<const QCheckBox *>(pair.parameter);
    if(parameter){
	result = parameter->checkState() == Qt::Checked;
    }
    return parameter != NULL;
}

bool ParameterWidget::getEnumValue(const QString& name, int& result) const{
    ParameterPair pair = m_parameterMap.value(name);
    const QComboBox *parameter = dynamic_cast<const QComboBox *>(pair.parameter);
    if(parameter){
	result = parameter->currentIndex();
    }
    return parameter != NULL;
}

void ParameterWidget::setDoubleValue(const QString& name, double value){
    ParameterPair pair = m_parameterMap.value(name);
    QDoubleSpinBox *parameter = dynamic_cast<QDoubleSpinBox *>(pair.parameter);
    if(parameter){
	parameter->setValue(value);
    }    
}

void ParameterWidget::setIntValue(const QString& name, int value){
    ParameterPair pair = m_parameterMap.value(name);
    QSpinBox *parameter = dynamic_cast<QSpinBox *>(pair.parameter);
    if(parameter){
	parameter->setValue(value);
    }    
}

void ParameterWidget::setBoolValue(const QString& name, bool value){
    ParameterPair pair = m_parameterMap.value(name);
    QCheckBox *parameter = dynamic_cast<QCheckBox *>(pair.parameter);
    if(parameter){
	parameter->setCheckState(value ? Qt::Checked : Qt::Unchecked);
    }    
}

void ParameterWidget::setEnumValue(const QString& name, int value){
    ParameterPair pair = m_parameterMap.value(name);
    QComboBox *parameter = dynamic_cast<QComboBox *>(pair.parameter);
    if(parameter){
	parameter->setCurrentIndex(value);
    }
}
