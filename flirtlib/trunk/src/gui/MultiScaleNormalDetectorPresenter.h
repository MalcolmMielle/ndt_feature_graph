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

#ifndef MULTISCALENORMALDETECTORPRESENTER_H_
#define MULTISCALENORMALDETECTORPRESENTER_H_

#include <gui/PeakFinderPresenter.h>
#include <gui/MultiScaleDetectorPresenter.h>
#include <gui/ParameterWidget.h>
#include <feature/NormalDetector.h>
#include <QtCore/QVector>
#include <QtCore/QObject>
#include <QtCore/QString>

class MultiScaleNormalDetectorPresenter: public MultiScaleDetectorPresenter{
    Q_OBJECT

    public:
	MultiScaleNormalDetectorPresenter(NormalDetector* detector, ParameterWidget* detectorParameter);
	
	void setDetectorParameter(ParameterWidget* detectorParameter);

    public slots:
	void changeParameter(const QString& name);
	
	void changeWindowSize(int size);

    protected:
	virtual void syncronize();
	virtual void reconnect();
};

#endif

