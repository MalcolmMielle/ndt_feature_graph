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

#ifndef DETECTORPRESENTER_H_
#define DETECTORPRESENTER_H_

#include <feature/Detector.h>
#include <gui/ParameterWidget.h>
#include <QtGui/QWidget>
#include <QtCore/QObject>

#include <iostream>

class DetectorPresenter: public QObject{
    public:
	DetectorPresenter(Detector* detector, ParameterWidget* detectorParameter): 
	    m_detector(detector), 
	    m_detectorParameter(detectorParameter)
	    { }
	
	virtual inline void activate()
	    {if(m_detectorParameter){m_detectorParameter->activate();}}
	virtual inline void deactivate()
	    {if(m_detectorParameter) m_detectorParameter->deactivate();}

	virtual void setDetector(Detector* detector) = 0;

	virtual void setDetectorParameter(ParameterWidget* detectorParameter) = 0;
	
	inline const Detector* getDetector() const
	    {return m_detector;}

	inline const ParameterWidget* getDetectorParameter() const
	    {return m_detectorParameter;}

	inline Detector* getDetector() 
	    {return m_detector;}

	inline ParameterWidget* getDetectorParameter() 
	    {return m_detectorParameter;}


    protected:
	virtual void syncronize() = 0;
	virtual void reconnect() = 0;
	
	Detector* m_detector;
	ParameterWidget* m_detectorParameter;
};

#endif

