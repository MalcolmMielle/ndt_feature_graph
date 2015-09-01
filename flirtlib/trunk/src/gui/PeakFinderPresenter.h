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

#ifndef PEAKFINDERPRESENTER_H_
#define PEAKFINDERPRESENTER_H_

#include <utils/PeakFinder.h>
#include <QtGui/QWidget>
#include <QtCore/QObject>
#include <gui/ParameterWidget.h>

class PeakFinderPresenter: public QObject{
    public:
	PeakFinderPresenter(PeakFinder* peakFinder, ParameterWidget* peakParameter):
	    m_peakFinder(peakFinder),
	    m_peakFinderParameter(peakParameter) 
	    { }

	inline void activate()
	    {if(m_peakFinderParameter) m_peakFinderParameter->activate();}
	inline void deactivate()
	    {if(m_peakFinderParameter) m_peakFinderParameter->deactivate();}

	virtual void setPeakFinder(PeakFinder* peakFinder) = 0;

	virtual void setPeakFinderParameter(ParameterWidget* peakParameter) = 0;

	    
	inline const PeakFinder* getPeakFinder() const
	    {return m_peakFinder;}
	inline const ParameterWidget* getPeakFinderParameter() const
	    {return m_peakFinderParameter;}
	
	inline PeakFinder* getPeakFinder() 
	    {return m_peakFinder;}
	inline ParameterWidget* getPeakFinderParameter() 
	    {return m_peakFinderParameter;}
	
    protected:
	PeakFinder* m_peakFinder;
	ParameterWidget* m_peakFinderParameter;
};

#endif

