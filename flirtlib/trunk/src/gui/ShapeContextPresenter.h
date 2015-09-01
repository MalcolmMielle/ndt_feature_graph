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

#ifndef SHAPECONTEXTPRESENTER_H_
#define SHAPECONTEXTPRESENTER_H_

#include <gui/DescriptorPresenter.h>
#include <gui/ParameterWidget.h>
#include <feature/ShapeContext.h>
#include <QtGui/QWidget>
#include <QtCore/QObject>

class ShapeContextPresenter: public DescriptorPresenter{
    Q_OBJECT
    
    public:
	ShapeContextPresenter(ShapeContextGenerator* descriptor, ParameterWidget* peakParameter);

	virtual void setDescriptor(DescriptorGenerator* descriptor);
	
	virtual void setDescriptorParameter(ParameterWidget* peakParameter);
	
	void insertDistanceFunction(const QString& name, const HistogramDistance<double>* distanceFunction);
	
    signals:
	void descriptorChanged();
	    
    public slots:
	void changeParameter(const QString& name);
	void changeMinRho(double value);
	void changeMaxRho(double value);
	void changeBinRho(int value);
	void changeBinPhi(int value);
	void changeDistanceFunction(int value);
	
    protected:
	void syncronize();
	void reconnect();
	
	const HistogramDistance<double>* m_currentDistanceFunction;
	int m_currentDistanceFunctionIndex;
	QVector< const HistogramDistance<double>* > m_distanceFunctions;
	QStringList m_distanceFunctionNames;
};

#endif

