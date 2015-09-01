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

#include "FLIRTDemo.h"

#include "FLIRTDemo.moc"

#include <typeinfo>
#include <iostream>
#include <string>
#include <sstream>

FLIRTDemo::FLIRTDemo(QWidget* parent, SensorStream& sensor): 
    QWidget(parent), 
    m_sensor(sensor),
	m_currentPoint(NULL),
	m_reading(NULL)
{ 
    m_sensorWidget = new SensorStreamWidget(this);
    m_rendererWidget = new RendererWidget(this);
    m_descriptorRendererWidget = new RendererWidget(this);
    m_peakMR = new SimpleMinMaxPeakFinder(0.2, 0.04);
    m_peakR = new SimplePeakFinder(0.2, 0.04);
    m_peakC = new SimplePeakFinder(0.3, 0.004);
    m_peakMNE = new SimpleMinMaxPeakFinder(0.2, 0.04);
    m_peakNE = new SimplePeakFinder(0.2, 0.04);
    m_peakMNB = new SimpleMinMaxPeakFinder(0.2, 0.04);
    m_peakNB = new SimplePeakFinder(0.2, 0.04);
    m_detectorR = new RangeDetector(m_peakMR,1);
    m_detectorC = new CurvatureDetector(m_peakC,1);
    m_detectorNB = new NormalBlobDetector(m_peakMNB,3,1);
    m_detectorNE = new NormalEdgeDetector(m_peakMNE,3,1);
    m_layout = new QGridLayout(this);
    m_parameter = new QTabWidget(this);
    m_chooserW = new QWidget(0);
    m_parameter->insertTab(1,m_chooserW,"Chooser");
    QVBoxLayout *chooserLayout = new QVBoxLayout(m_chooserW);
//     m_paramLayout = new QGridLayout(m_parameter);
    m_plotWidget = new MultiScaleDetectorPlotWidget(this, m_detectorR->getScaleNumber());

    m_detectorPresenterR = new MultiScaleDetectorPresenter(m_detectorR, new TabbedParameterWidget("RangeDetector", m_parameter,0));
    m_peakMRPresenter = new SimplePeakFinderPresenter(m_peakMR, new TabbedParameterWidget("PeakFinder", m_parameter,2));
    m_peakRPresenter = new SimplePeakFinderPresenter(m_peakR, new TabbedParameterWidget("PeakFinder", m_parameter,2));
    m_detectorPresenterR->insertPeakFinder("SimpleMinMax", m_peakMRPresenter);
    m_detectorPresenterR->insertPeakFinder("Simple", m_peakRPresenter);
    
    m_detectorPresenterNB = new MultiScaleNormalDetectorPresenter(m_detectorNB, new TabbedParameterWidget("NormalBlobDetector", m_parameter,1));
    m_peakMNBPresenter = new SimplePeakFinderPresenter(m_peakMNB, new TabbedParameterWidget("PeakFinder", m_parameter,2));
    m_peakNBPresenter = new SimplePeakFinderPresenter(m_peakNB, new TabbedParameterWidget("PeakFinder", m_parameter,2));
    m_detectorPresenterNB->insertPeakFinder("SimpleMinMax", m_peakMNBPresenter);
    m_detectorPresenterNB->insertPeakFinder("Simple", m_peakNBPresenter);
    
    m_detectorPresenterNE = new MultiScaleNormalDetectorPresenter(m_detectorNE, new TabbedParameterWidget("NormalEdgeDetector", m_parameter,1));
    m_peakMNEPresenter = new SimplePeakFinderPresenter(m_peakMNE, new TabbedParameterWidget("PeakFinder", m_parameter,2));
    m_peakNEPresenter = new SimplePeakFinderPresenter(m_peakNE, new TabbedParameterWidget("PeakFinder", m_parameter,2));
    m_detectorPresenterNE->insertPeakFinder("SimpleMinMax", m_peakMNEPresenter);
    m_detectorPresenterNE->insertPeakFinder("Simple", m_peakNEPresenter);
    
    m_detectorPresenterC = new MultiScaleCurvatureDetectorPresenter(m_detectorC, new TabbedParameterWidget("CurvatureDetector", m_parameter,1)); 
    m_peakCPresenter = new SimplePeakFinderPresenter(m_peakC, new TabbedParameterWidget("PeakFinder", m_parameter,2));
    m_detectorPresenterC->insertPeakFinder("Simple", m_peakCPresenter);
    
    m_chooser = new DetectorChooserPresenter(new ParameterWidget("DetectorChooser", m_chooserW));
    m_chooser->insertDetector("CurvatureDetector", m_detectorPresenterC);
    m_chooser->insertDetector("RangeDetector", m_detectorPresenterR);
    m_chooser->insertDetector("NormalBlobDetector", m_detectorPresenterNB);
    m_chooser->insertDetector("NormalEdgeDetector", m_detectorPresenterNE);
    
    m_descriptorGeneratorS = new ShapeContextGenerator(0.02, 0.5, 4, 12);
    m_descriptorPresenterS = new ShapeContextPresenter(m_descriptorGeneratorS, new TabbedParameterWidget("ShapeContext", m_parameter,3));
    
    m_descriptorGeneratorB = new BetaGridGenerator(0.02, 0.5, 4, 12);
    m_descriptorPresenterB = new BetaGridPresenter(m_descriptorGeneratorB, new TabbedParameterWidget("BetaGrid", m_parameter,3));
    
    m_chooserD = new DescriptorChooserPresenter(new ParameterWidget("DescriptorChooser", m_chooserW));
    m_chooserD->insertDescriptor("BetaGrid", m_descriptorPresenterB);
    m_chooserD->insertDescriptor("ShapeContext", m_descriptorPresenterS);
    
    m_interestRenderer = 0;
    m_laserRenderer = 0;
    m_supportRenderer = 0;
    m_polarRenderer = 0;

    m_layout->setSpacing(3);
    m_layout->setMargin(3);
    m_layout->addWidget(m_sensorWidget,0,0);
    m_layout->addWidget(m_rendererWidget,1,0);
    m_layout->addWidget(m_descriptorRendererWidget,2,1);
    m_layout->setRowStretch(1,1);
    m_layout->addWidget(m_plotWidget,2,0);
    m_layout->addWidget(m_parameter,1,1);
    chooserLayout->addWidget(m_chooser->getChooserParameter());
    chooserLayout->addWidget(m_chooserD->getChooserParameter());
/*    m_paramLayout->setSpacing(3);
    m_paramLayout->setMargin(3);
    m_paramLayout->addWidget(m_detectorPresenterC->getDetectorParameter(),0,0);
    m_paramLayout->addWidget(m_detectorPresenterR->getDetectorParameter(),0,0);
    m_paramLayout->addWidget(m_detectorPresenterNB->getDetectorParameter(),0,0);
    m_paramLayout->addWidget(m_detectorPresenterNE->getDetectorParameter(),0,0);
    m_paramLayout->addWidget(m_peakMRPresenter->getPeakFinderParameter(),1,0);
    m_paramLayout->addWidget(m_peakRPresenter->getPeakFinderParameter(),1,0);
    m_paramLayout->addWidget(m_peakCPresenter->getPeakFinderParameter(),1,0);
    m_paramLayout->addWidget(m_peakMNEPresenter->getPeakFinderParameter(),1,0);
    m_paramLayout->addWidget(m_peakNEPresenter->getPeakFinderParameter(),1,0);
    m_paramLayout->addWidget(m_peakMNBPresenter->getPeakFinderParameter(),1,0);
    m_paramLayout->addWidget(m_peakNBPresenter->getPeakFinderParameter(),1,0);
    m_paramLayout->addWidget(m_descriptorPresenter->getDescriptorParameter(),2,0);*/
    m_sensor.seek(0,END);
    unsigned int end = m_sensor.tell();
    m_sensor.seek(0,BEGIN);
    m_sensorWidget->seekable(m_sensor.isSeekable(), end);
    m_sensorWidget->streamReady();
    m_sensorWidget->setMaximumHeight(32);
    m_plotWidget->setMaximumHeight(256);
    m_plotWidget->setMinimumHeight(256);
    m_parameter->setMaximumWidth(350);
    m_parameter->setMinimumWidth(350);

    m_parameter->show();

    connect(m_sensorWidget, SIGNAL(newReading()), this, SLOT(next()));
    connect(m_sensorWidget, SIGNAL(newReading(int)), this, SLOT(next(int)));
    connect(m_chooser, SIGNAL(detectorChanged()), this, SLOT(update()));
    connect(m_chooserD, SIGNAL(descriptorChanged()), this, SLOT(updateDescriptor()));
    connect(m_descriptorPresenterS, SIGNAL(descriptorChanged()), this, SLOT(updateDescriptor()));
    connect(m_descriptorPresenterB, SIGNAL(descriptorChanged()), this, SLOT(updateDescriptor()));
    connect(m_rendererWidget, SIGNAL(mousePressedGL(int, int, int)), this, SLOT(mousePressedGL(int, int, int)));

	const AbstractReading* read = m_sensor.current();
    processReading(read);
}
	
FLIRTDemo::~FLIRTDemo() { 
    delete m_sensorWidget;
    delete m_rendererWidget;
    delete m_peakR;
    delete m_detectorR;
    delete m_layout;
    delete m_plotWidget;
    if(m_interestRenderer) delete m_interestRenderer;
    if(m_laserRenderer) delete m_laserRenderer;
    if(m_supportRenderer) delete m_supportRenderer;
    delete m_sensorWidget;
}

void FLIRTDemo::next() {
    m_sensor.next();
    const AbstractReading* read = m_sensor.current();
    processReading(read);
}

void FLIRTDemo::next(int pos) {
    m_sensor.seek((unsigned int) pos);
    const AbstractReading* read = m_sensor.current();
    processReading(read);
}

void FLIRTDemo::update(){
    if(dynamic_cast<const MultiScaleDetector*>(m_chooser->getCurrentDetector())){
		m_plotWidget->setScales(static_cast<const MultiScaleDetector*>(m_chooser->getCurrentDetector())->getScaleNumber());
    } else if(dynamic_cast<const CurvatureDetector*>(m_chooser->getCurrentDetector())){
		m_plotWidget->setScales(static_cast<const CurvatureDetector*>(m_chooser->getCurrentDetector())->getScaleNumber());
    }
    const AbstractReading* read = m_sensor.current();
    processReading(read);    
}

void FLIRTDemo::mousePressedGL(int button, int x, int y){
    if(button & Qt::LeftButton){
		GLdouble worldX, worldY, worldZ;
		m_rendererWidget->unprojectCoordinates(x, y, &worldX, &worldY, &worldZ);

		if(!m_points.size()) return;
		double minDistance = 10e16;
		unsigned int minIndex = 0;
		Point2D clickedPoint(worldX, worldY);
		for(unsigned int i = 0; i < m_points.size(); i ++){
			Point2D difference = clickedPoint - m_points[i]->getPosition();
			double distance = hypot(difference.x, difference.y);
			if(distance < minDistance){
				minDistance = distance;
				minIndex = i;
			}
		}
		if(minDistance < 0.2){
			m_currentPoint = m_points[minIndex];
			m_supportRenderer->setLaserPoints(&m_currentPoint->getSupport());
			m_supportRenderer->setColor(m_colors[minIndex]);
			m_polarRenderer->setColor(m_colors[minIndex]);
		} else {
			m_currentPoint = NULL;
		}
    }
    drawDescriptor();
}

void FLIRTDemo::updateDescriptor(){
    for(unsigned int i = 0; i < m_points.size(); i++){
		m_points[i]->setDescriptor(m_chooserD->getCurrentDescriptor()->describe(*m_points[i], *m_reading));
    }
    drawDescriptor();
}

void FLIRTDemo::drawDescriptor(){
    if(m_currentPoint){
		if(const ShapeContext * desc = dynamic_cast<const ShapeContext *>(m_currentPoint->getDescriptor())){
			m_polarRenderer->setGrid(&desc->getHistogram(), &m_descriptorGeneratorS->getPhiEdges(), &m_descriptorGeneratorS->getRhoEdges());
			m_polarRenderer->setPosition(&m_currentPoint->getPosition());
			m_descriptorRendererWidget->setLaserPose(m_currentPoint->getPosition().x, m_currentPoint->getPosition().y);
		} else if(const BetaGrid * desc = dynamic_cast<const BetaGrid *>(m_currentPoint->getDescriptor())){
			m_polarRenderer->setGrid(&desc->getHistogram(), &m_descriptorGeneratorB->getPhiEdges(), &m_descriptorGeneratorB->getRhoEdges());
			m_polarRenderer->setPosition(&m_currentPoint->getPosition());
			m_descriptorRendererWidget->setLaserPose(m_currentPoint->getPosition().x, m_currentPoint->getPosition().y);
		}
    } else {
		m_polarRenderer->setGrid(NULL, NULL, NULL);
		m_polarRenderer->setPosition(NULL);
		m_supportRenderer->setLaserPoints(NULL);
    }
    m_descriptorRendererWidget->updateGL();
    m_rendererWidget->updateGL();
}


void FLIRTDemo::processReading(const AbstractReading* _read){
    const LaserReading* lread = dynamic_cast<const LaserReading*>(_read);
    if (lread){
		const std::vector<Point2D>& points = lread->getCartesian();
		std::vector< double > signal;
		std::vector< std::vector<double> > smooth;
		std::vector< std::vector<double> > diff;
		std::vector< std::vector<unsigned int> > idx;
		QVector<double> xpoints(points.size());
		for(int i = 0; i < xpoints.size(); i++) xpoints[i] = (double)i;
		m_chooser->getCurrentDetector()->detect(*lread, m_points, signal, smooth, diff, idx);
		const std::vector<QColor>& colors = m_plotWidget->getColors();
		for(unsigned int i = 0; i < smooth.size(); i++){
			QVector<double> smoothV = QVector<double>::fromStdVector(smooth[i]);
			QVector<double> diffV = QVector<double>::fromStdVector(diff[i]);
			m_plotWidget->setSmoothData(xpoints, smoothV, i);
			m_plotWidget->setDifferentialData(xpoints, diffV, i);
			QVector<double> smoothMarker(idx[i].size());
			QVector<double> diffMarker(idx[i].size());
			QVector<double> indexes(idx[i].size());
			for(unsigned int j = 0; j < idx[i].size(); j++){
				smoothMarker[j] = smooth[i][idx[i][j]];
				diffMarker[j] = diff[i][idx[i][j]];
				indexes[j] = idx[i][j];
			} 
			m_plotWidget->setSmoothMarker(indexes, smoothMarker, i);
			m_plotWidget->setDifferentialMarker(indexes, diffMarker, i);
		}
		m_interestPoints.resize(m_points.size());
		m_scales.resize(m_points.size());
		m_colors.resize(m_points.size(),Color(1.,1.,1.,1.));
		for(unsigned int i = 0; i < m_points.size(); i++){
			m_points[i]->setDescriptor(m_chooserD->getCurrentDescriptor()->describe(*m_points[i], *lread));
			m_interestPoints[i] = &(m_points[i]->getPosition());
			m_scales[i] = m_points[i]->getScale();
			m_colors[i] = toColor(colors[m_points[i]->getScaleLevel()]);
		}
		if(!m_laserRenderer){ 
			m_interestRenderer = new InterestPointRenderer(&m_interestPoints, &m_scales);
			m_laserRenderer = new LaserReadingRenderer(&lread->getCartesian());
			m_supportRenderer = new LaserReadingRenderer(NULL);
			m_polarRenderer = new PolarGridRenderer(NULL, NULL, NULL);
			m_supportRenderer->setSize(m_laserRenderer->getSize() + 0.01f);
			m_descriptorRendererWidget->addRenderer(m_polarRenderer);
			m_descriptorRendererWidget->addRenderer(m_laserRenderer);
			m_rendererWidget->addRenderer(m_laserRenderer);
			m_rendererWidget->addRenderer(m_supportRenderer);
			m_rendererWidget->addRenderer(m_interestRenderer);
			m_laserRenderer->setLaserPose(&lread->getLaserPose());
		}else {
			m_laserRenderer->setLaserPoints(&points);
			m_interestRenderer->setInterestPoints(&m_interestPoints, &m_scales);
			m_laserRenderer->setLaserPose(&lread->getLaserPose());
		}
		m_polarRenderer->setGrid(NULL, NULL, NULL);
		m_polarRenderer->setPosition(NULL);
		m_supportRenderer->setLaserPoints(NULL);
		m_interestRenderer->setColors(m_colors);
		m_rendererWidget->setLaserPose(lread->getLaserPose().x, lread->getLaserPose().y);
		m_reading = lread;
    }
    m_plotWidget->replot();
    m_rendererWidget->updateGL();
    m_descriptorRendererWidget->updateGL();
}


int main(int argc, char **argv){

	if(argc < 2){
		std::cerr << "Usage : flirtDemo <logfile>" << std::endl;
		exit(-1);
	}
	std::string filename(argv[1]);
    
    QApplication app(argc, argv);
    
    CarmenLogWriter writer;
    CarmenLogReader reader;
    
    LogSensorStream log(&reader, &writer);
    
    log.load(filename);
    
    FLIRTDemo* widget = new FLIRTDemo(0, log);
    widget->setGeometry(100, 100, 500, 500);
    widget->show();
    
    return app.exec();
}
