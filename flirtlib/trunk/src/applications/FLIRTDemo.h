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

#ifndef FLIRTDEMO_H_
#define FLIRTDEMO_H_

#include <gui/SensorStreamWidget.h>
#include <gui/LaserReadingRenderer.h>
#include <gui/PolarGridRenderer.h>
#include <gui/InterestPointRenderer.h>
#include <gui/RendererWidget.h>
#include <gui/DetectorChooserPresenter.h>
#include <gui/DescriptorChooserPresenter.h>
#include <gui/MultiScaleDetectorPlotWidget.h>
#include <gui/MultiScaleDetectorPresenter.h>
#include <gui/MultiScaleCurvatureDetectorPresenter.h>
#include <gui/MultiScaleNormalDetectorPresenter.h>
#include <gui/ShapeContextPresenter.h>
#include <gui/BetaGridPresenter.h>
#include <gui/ParameterWidget.h>
#include <gui/TabbedParameterWidget.h>
#include <gui/SimplePeakFinderPresenter.h>
#include <feature/Detector.h>
#include <feature/ShapeContext.h>
#include <feature/BetaGrid.h>
#include <feature/RangeDetector.h>
#include <feature/CurvatureDetector.h>
#include <feature/NormalBlobDetector.h>
#include <feature/NormalEdgeDetector.h>
#include <sensorstream/CarmenLog.h>
#include <sensorstream/LogSensorStream.h>
#include <sensorstream/SensorStream.h>
#include <utils/SimpleMinMaxPeakFinder.h>

#include <QtGui/QApplication>
#include <QtGui/QGridLayout>
#include <QtGui/QVBoxLayout>
#include <QtGui/QTabWidget>
#include <QtGui/QScrollArea>

#include <string>
#include <iostream>

inline Color toColor(const QColor& _color) { return Color(_color.redF(), _color.greenF(), _color.blueF(), _color.alphaF());}

class FLIRTDemo: public QWidget {
    Q_OBJECT
    public:
	FLIRTDemo(QWidget* parent, SensorStream& sensor);
	
	virtual ~FLIRTDemo();
	
	SensorStreamWidget *m_sensorWidget;
	RendererWidget *m_rendererWidget;
	RendererWidget *m_descriptorRendererWidget;
	LaserReadingRenderer* m_laserRenderer;
	LaserReadingRenderer* m_supportRenderer;
	
	RangeDetector* m_detectorR;
	CurvatureDetector* m_detectorC;
	NormalBlobDetector* m_detectorNB;
	NormalEdgeDetector* m_detectorNE;
	MultiScaleDetectorPresenter* m_detectorPresenterR;
	MultiScaleCurvatureDetectorPresenter* m_detectorPresenterC;
	MultiScaleNormalDetectorPresenter* m_detectorPresenterNB;
	MultiScaleNormalDetectorPresenter* m_detectorPresenterNE;
	
	QWidget* m_chooserW;
	DetectorChooserPresenter* m_chooser;
	DescriptorChooserPresenter* m_chooserD;
	
	SimpleMinMaxPeakFinder* m_peakMR;
	SimplePeakFinder* m_peakR;
	SimplePeakFinderPresenter* m_peakMRPresenter;
	SimplePeakFinderPresenter* m_peakRPresenter;
	
	SimpleMinMaxPeakFinder* m_peakMNE;
	SimplePeakFinder* m_peakNE;
	SimplePeakFinderPresenter* m_peakMNEPresenter;
	SimplePeakFinderPresenter* m_peakNEPresenter;
	
	SimpleMinMaxPeakFinder* m_peakMNB;
	SimplePeakFinder* m_peakNB;
	SimplePeakFinderPresenter* m_peakMNBPresenter;
	SimplePeakFinderPresenter* m_peakNBPresenter;
	
	SimplePeakFinder* m_peakC;
	SimplePeakFinderPresenter* m_peakCPresenter;
	
	InterestPointRenderer* m_interestRenderer;
	PolarGridRenderer* m_polarRenderer;
	
	ShapeContextGenerator *m_descriptorGeneratorS;
	ShapeContextPresenter *m_descriptorPresenterS;
	
	BetaGridGenerator *m_descriptorGeneratorB;
	BetaGridPresenter *m_descriptorPresenterB;
	
	QGridLayout *m_layout;
	QGridLayout *m_paramLayout;
	MultiScaleDetectorPlotWidget * m_plotWidget;
	SensorStream& m_sensor;
	QTabWidget* m_parameter;
// 	QScrollArea *m_parameterScroll;
	
	std::vector<InterestPoint *> m_points;
	std::vector<const OrientedPoint2D *> m_interestPoints;
	InterestPoint * m_currentPoint;
	std::vector<double> m_scales;
	std::vector<Color> m_colors;
	const LaserReading *m_reading;

    public slots:
	virtual void next();

	virtual void next(int pos);
	
	virtual void update();
	
	virtual void updateDescriptor();
	
	virtual void mousePressedGL(int button, int x, int y);
	
    protected:
	void processReading(const AbstractReading* _read);
	void drawDescriptor();
};

#endif

