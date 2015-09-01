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

#include "gui/TabbedParameterWidget.h"

TabbedParameterWidget::TabbedParameterWidget(const QString &name, QTabWidget *parent, int position):
    ParameterWidget(name, parent),
    m_tabWidget(parent),
    m_tabPosition(position)
{
    if(m_tabWidget && position == -1)
	m_tabPosition = m_tabWidget->count();
}

void TabbedParameterWidget::activate()
{
    ParameterWidget::activate(); 
    if(m_tabWidget){
	int position = m_tabWidget->insertTab(m_tabPosition, this, m_name);
// 	m_tabWidget->setCurrentIndex(position);
    }
}
    
void TabbedParameterWidget::deactivate()
{
    ParameterWidget::deactivate(); 
    if(m_tabWidget){
	int position = m_tabWidget->indexOf(this);
	m_tabWidget->removeTab(position);
    }
}
