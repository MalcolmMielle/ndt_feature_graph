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

#ifndef MULTISCALEDETECTORPRESENTER_H_
#define MULTISCALEDETECTORPRESENTER_H_

#include <gui/PeakFinderPresenter.h>
#include <gui/DetectorPresenter.h>
#include <gui/ParameterWidget.h>
#include <feature/MultiScaleDetector.h>
#include <QtCore/QVector>
#include <QtCore/QObject>
#include <QtCore/QString>

class MultiScaleDetectorPresenter: public DetectorPresenter{
    Q_OBJECT

    public:
	MultiScaleDetectorPresenter(MultiScaleDetector* detector, ParameterWidget* detectorParameter);
	
	virtual void activate();
	
	virtual void deactivate();
	
	void insertPeakFinder(const QString& name, PeakFinderPresenter* peak);

	void setDetector(Detector* detector);
	
	void setDetectorParameter(ParameterWidget* detectorParameter);
	
    signals:
	void detectorChanged();
	
    public slots:
	void changeParameter(const QString& name);
    
	void changeMaxRange(int check);
	void changeScale(int scale);
	void changeSigma(double sigma);
	void changeSigmaStep(double step);
	void changeFilter(int filter);
	void changePeakFinder(int peakFinder);	

    protected:
	virtual void syncronize();
	virtual void reconnect();
	
	PeakFinderPresenter* m_currentPeakPresenter;
	int m_currentPeakPresenterIndex;
	QVector< PeakFinderPresenter* > m_peakPresenters;
	QStringList m_peakPresenterNames;
};

#endif

