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

#ifndef FEATURESETMATCHERPRESENTER_H_
#define FEATURESETMATCHERPRESENTER_H_

#include <feature/AbstractFeatureSetMatcher.h>
#include <QtGui/QWidget>
#include <QtCore/QObject>
#include <gui/ParameterWidget.h>

class FeatureSetMatcherPresenter: public QObject{
    Q_OBJECT
    
    public:
	FeatureSetMatcherPresenter(AbstractFeatureSetMatcher* featureSetMatcher, ParameterWidget* featureSetMatcherParameter);

	inline virtual void activate()
	    {if(m_featureSetMatcherParameter) m_featureSetMatcherParameter->activate();}
	inline virtual void deactivate()
	    {if(m_featureSetMatcherParameter) m_featureSetMatcherParameter->deactivate();}

	virtual void setFeatureSetMatcher(AbstractFeatureSetMatcher* featureSetMatcher) = 0;

	virtual void setFeatureSetMatcherParameter(ParameterWidget* peakParameter) = 0;

	    
	inline const AbstractFeatureSetMatcher* getFeatureSetMatcher() const
	    {return m_featureSetMatcher;}
	inline const ParameterWidget* getFeatureSetMatcherParameter() const
	    {return m_featureSetMatcherParameter;}
	
	inline AbstractFeatureSetMatcher* getFeatureSetMatcher() 
	    {return m_featureSetMatcher;}
	inline ParameterWidget* getFeatureSetMatcherParameter() 
	    {return m_featureSetMatcherParameter;}
    
    signals:
	void featureSetMatcherChanged();
	    
    public slots:
	void changeParameter(const QString& name);
	void changeAcceptanceThreshold(double value);

    protected:
	virtual void syncronize();
	virtual void reconnect();

	AbstractFeatureSetMatcher* m_featureSetMatcher;
	ParameterWidget* m_featureSetMatcherParameter;
};

#endif

