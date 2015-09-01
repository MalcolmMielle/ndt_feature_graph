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

#ifndef RANSACPRESENTER_H_
#define RANSACPRESENTER_H_

#include <gui/FeatureSetMatcherPresenter.h>
#include <gui/ParameterWidget.h>
#include <feature/RansacFeatureSetMatcher.h>
#include <QtGui/QWidget>
#include <QtCore/QObject>

class RansacPresenter: public FeatureSetMatcherPresenter{
    Q_OBJECT
    
    public:
	RansacPresenter(RansacFeatureSetMatcher* featureSetMatcher, ParameterWidget* featureSetMatcherParameter);

	virtual void setFeatureSetMatcher(AbstractFeatureSetMatcher* featureSetMatcher);
	
	virtual void setFeatureSetMatcherParameter(ParameterWidget* peakParameter);
	
/*    signals:
	void featureSetMatcherChanged();*/
	    
    public slots:
	void changeParameter(const QString& name);
	void changeSuccessProbability(double value);
	void changeInlierProbability(double value);
	void changeDistanceThreshold(double value);
	void changeAdaptive(int value);
	
    protected:
	virtual void syncronize();
	virtual void reconnect();
};

#endif

