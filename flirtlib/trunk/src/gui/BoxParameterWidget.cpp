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

#include "BoxParameterWidget.h"


BoxParameterWidget::BoxParameterWidget(const QString &name, QWidget *parent):
    ParameterWidget(name, parent)
{
    QGridLayout *layout = m_layout;
    m_groupBox = new QGroupBox(name, this);
    layout->addWidget(m_groupBox,0,0);
    m_layout = new QGridLayout(m_groupBox);
}

