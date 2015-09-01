/* *
 * FLIRTLib - Fast Laser Interesting Region Transform Library
 * Copyright (C) 2009-2010 Gian Diego Tipaldi and Kai O. Arras
 *
 * This file is part of FLIRTLib.
 *
 * FLIRTLib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FLIRTLib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with FLIRTLib.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DESCRIPTORPRESENTER_H_
#define DESCRIPTORPRESENTER_H_

#include <feature/Descriptor.h>
#include <QtGui/QWidget>
#include <QtCore/QObject>
#include <gui/ParameterWidget.h>

class DescriptorPresenter: public QObject{
    public:
	DescriptorPresenter(DescriptorGenerator* descriptor, ParameterWidget* peakParameter):
	    m_descriptor(descriptor),
	    m_descriptorParameter(peakParameter) 
	    { }

	inline void activate()
	    {if(m_descriptorParameter) m_descriptorParameter->activate();}
	inline void deactivate()
	    {if(m_descriptorParameter) m_descriptorParameter->deactivate();}

	virtual void setDescriptor(DescriptorGenerator* descriptor) = 0;

	virtual void setDescriptorParameter(ParameterWidget* peakParameter) = 0;
    
	inline const DescriptorGenerator* getDescriptor() const
	    {return m_descriptor;}
	
	inline const ParameterWidget* getDescriptorParameter() const
	    {return m_descriptorParameter;}
    
	inline DescriptorGenerator* getDescriptor() 
	    {return m_descriptor;}
	
	inline ParameterWidget* getDescriptorParameter() 
	    {return m_descriptorParameter;}
	
    protected:
	DescriptorGenerator* m_descriptor;
	ParameterWidget* m_descriptorParameter;
};

#endif

