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

#include "InterestPoint.h"

InterestPoint::InterestPoint(const OrientedPoint2D& _position, double _scale, const Descriptor* _descriptor):
    m_position(_position),
    m_scale(_scale),
    m_scaleLevel(0),
    m_descriptor(NULL)
{
    if(_descriptor) m_descriptor = _descriptor->clone();
}

InterestPoint::InterestPoint(const InterestPoint& _point):
    m_position(_point.getPosition()),
    m_scale(_point.getScale()),
    m_scaleLevel(_point.getScaleLevel())
{
    if(_point.getDescriptor()) 
	m_descriptor = _point.getDescriptor()->clone();
    else
	m_descriptor = 0;
}

InterestPoint& InterestPoint::operator=(const InterestPoint& _point){
    m_position = _point.getPosition();
    m_scale = _point.getScale();
    m_scaleLevel = _point.getScaleLevel();
    delete m_descriptor;
    m_descriptor = _point.getDescriptor()->clone();
    return *this;
}

InterestPoint::~InterestPoint(){
    delete m_descriptor;
}
