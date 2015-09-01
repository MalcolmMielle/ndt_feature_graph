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

#include <feature/Detector.h>
#include <feature/ShapeContext.h>
#include <feature/BetaGrid.h>
#include <feature/RangeDetector.h>
#include <feature/CurvatureDetector.h>
#include <feature/NormalBlobDetector.h>
#include <feature/NormalEdgeDetector.h>
#include <feature/RansacFeatureSetMatcher.h>
#include <feature/RansacMultiFeatureSetMatcher.h>
#include <sensorstream/CarmenLog.h>
#include <sensorstream/LogSensorStream.h>
#include <sensorstream/SensorStream.h>
#include <utils/SimpleMinMaxPeakFinder.h>
#include <utils/HistogramDistances.h>

#include <cairo.h>
#include <cairo-pdf.h>
#include <cairo-svg.h>

#include <iostream>
#include <string>
#include <string.h>
#include <sstream>
#include <utility>
#include <sys/stat.h>
#include <sys/types.h>


LogSensorStream m_sensorReference(NULL,NULL);

CurvatureDetector *m_detectorCurvature = NULL;
NormalBlobDetector *m_detectorNormalBlob = NULL;
NormalEdgeDetector *m_detectorNormalEdge = NULL;
RangeDetector *m_detectorRange = NULL;
Detector* m_detector = NULL;

BetaGridGenerator *m_betaGenerator = NULL;
ShapeContextGenerator *m_shapeGenerator = NULL;
DescriptorGenerator *m_descriptor = NULL;

RansacFeatureSetMatcher *m_ransac = NULL;

std::vector< std::vector<InterestPoint *> > m_pointsReference;
std::vector< OrientedPoint2D > m_posesReference;

cairo_t * cairoMap = 0;
cairo_t * cairoOut = 0;

int offsetX = 0, offsetY = 0, border = 10;
double scaleFactor = 1.;
int sizeX = 1024, sizeY = 800;

unsigned int corresp = 9;

double acceptanceSigma = 0.1;

void help(){
	std::cerr << "FLIRTLib version 0.9b - authors Gian Diego Tipaldi and Kai O. Arras" << std::endl
			  << "Usage: ransacLoopClosureDraw -filename <logfile> [options] " << std::endl
			  << "Options:" << std::endl

			  << " -filename          \t The logfile in CARMEN format to process (mandatory)." << std::endl
			  << " -corresp           \t The number of minimum correspondences to assess a match (default=9)." << std::endl
			  << " -scale             \t The number of scales to consider (default=5)." << std::endl
			  << " -dmst              \t The number of spanning tree for the curvature detector (deafult=2)." << std::endl
			  << " -window            \t The size of the local window for estimating the normal signal (default=3)." << std::endl
			  << " -detectorType      \t The type of detector to use. Available options are (default=0):" << std::endl
			  << "                    \t     0 - Curvature detector;" << std::endl
			  << "                    \t     1 - Normal edge detector;" << std::endl
			  << "                    \t     2 - Normal blob detector;" << std::endl
			  << "                    \t     3 - Range detector." << std::endl
			  << " -descriptorType    \t The type of descriptor to use. Available options are (default=0):" << std::endl
			  << "                    \t     0 - Beta - grid;" << std::endl
			  << "                    \t     1 - Shape context." << std::endl
			  << " -distanceType      \t The distance function to compare the descriptors. Available options are (default=2):" << std::endl
			  << "                    \t     0 - Euclidean distance;" << std::endl
			  << "                    \t     1 - Chi square distance;" << std::endl
			  << "                    \t     2 - Symmetric Chi square distance;" << std::endl
			  << "                    \t     3 - Bhattacharyya distance;" << std::endl
			  << "                    \t     4 - Kullback Leibler divergence;" << std::endl
			  << "                    \t     5 - Jensen Shannon divergence." << std::endl
			  << " -baseSigma         \t The initial standard deviation for the smoothing operator (default=0.2)." << std::endl
			  << " -sigmaStep         \t The incremental step for the scales of the smoothing operator. (default=1.4)." << std::endl
			  << " -minPeak           \t The minimum value for a peak to be detected (default=0.34)." << std::endl
			  << " -minPeakDistance   \t The minimum difference with the neighbors for a peak to be detected (default=0.001)." << std::endl
			  << " -acceptanceSigma   \t The standard deviation of the detection error (default=0.1)." << std::endl
			  << " -success           \t The success probability of RANSAC (default=0.95)." << std::endl
			  << " -inlier            \t The inlier probability of RANSAC (default=0.4)." << std::endl
			  << " -matchingThreshold \t The threshold two descriptors are considered matched (default=0.4)." << std::endl
			  << " -strategy          \t The strategy for matching two descriptors. Available options are (default=0):" << std::endl
			  << "                    \t     0 - Nearest Neighbour. Only the closest match above the threshold is considered;" << std::endl
			  << "                    \t     1 - Threshold. All the matches above the threshold are considered." << std::endl
			  << std::endl
			  << "The program matches every scan with all the others in the logfile. To work properly, the logfile must"  << std::endl
			  << "be corrected by a SLAM algorithm beforehand (the provided logfiles are already corrected). The program writes" << std::endl
			  << "one image for each scan in the following directory:" << std::endl
			  << "    <logfile>_<detector>_<descriptor>_<distance>/" << std::endl
			  << std::endl
			  << std::endl;
}

void match(unsigned int position)
{
//     cairo_set_line_cap(cairoOut, CAIRO_LINE_CAP_ROUND);
    
    m_sensorReference.seek(position);
    cairo_matrix_t m1;
    cairo_get_matrix(cairoOut, &m1);
    cairo_identity_matrix(cairoOut);
    cairo_set_source_surface(cairoOut, cairo_get_target(cairoMap), 0., 0.);
    cairo_paint(cairoOut);
    cairo_set_matrix(cairoOut, &m1);
//     cairo_set_line_width(cairoOut, 1./(2.*scaleFactor));
    
    std::vector<InterestPoint *> pointsLocal(m_pointsReference[position].size());
    const LaserReading* lreadReference = dynamic_cast<const LaserReading*>(m_sensorReference.current());
    for(unsigned int j = 0; j < m_pointsReference[position].size(); j++){
	InterestPoint * point = new InterestPoint(*m_pointsReference[position][j]);
	point->setPosition(lreadReference->getLaserPose().ominus(point->getPosition()));
	pointsLocal[j] = point;
    }
    
    
    for(unsigned int i = 0; i < m_pointsReference.size(); i++){
	if(i == position) {
	    continue;
	}
	OrientedPoint2D transform;
	std::vector< std::pair<InterestPoint*, InterestPoint* > > correspondences;
	double result = m_ransac->matchSets(m_pointsReference[i], pointsLocal, transform, correspondences);
	if(correspondences.size() >= corresp) {
	    cairo_matrix_t m;
	    cairo_get_matrix(cairoOut, &m);
	    cairo_translate(cairoOut, transform.x, transform.y);
	    cairo_rotate(cairoOut, transform.theta);
	    
	    cairo_set_source_rgba(cairoOut, 1., 0., 0., 1. - result/(acceptanceSigma * acceptanceSigma * 5.99 * double(pointsLocal.size())));
	    cairo_move_to(cairoOut, 0., -0.3);
	    cairo_line_to(cairoOut, 0.6, 0.);
	    cairo_line_to(cairoOut, 0., 0.3);
	    cairo_close_path(cairoOut);
	    cairo_fill(cairoOut);
	    cairo_set_matrix(cairoOut, &m);
	}
    }
    
    cairo_matrix_t m;
    cairo_get_matrix(cairoOut, &m);
    cairo_translate(cairoOut, lreadReference->getLaserPose().x, lreadReference->getLaserPose().y);
    cairo_rotate(cairoOut, lreadReference->getLaserPose().theta);
    cairo_set_source_rgba(cairoOut, 0., 0., 1., 1.);
    cairo_move_to(cairoOut, 0., -0.3);
    cairo_line_to(cairoOut, 0.6, 0.);
    cairo_line_to(cairoOut, 0., 0.3);
    cairo_close_path(cairoOut);
    cairo_stroke(cairoOut);
    cairo_set_matrix(cairoOut, &m);
//     cairo_show_page(cairoOut);
}

void detectLog(){
    unsigned int i = 0;
    unsigned int position = m_sensorReference.tell();
    m_sensorReference.seek(0,END);
    unsigned int last = m_sensorReference.tell();
    m_sensorReference.seek(0);
    
    std::string bar(50, ' ');
    bar[0] = '#';
    unsigned int progress = 0;
    
    while(!m_sensorReference.end()){
	unsigned int currentProgress = (m_sensorReference.tell()*100)/last;
	if (progress < currentProgress){
	    progress = currentProgress;
	    bar[progress/2] = '#';
	    std::cout << "\rDetecting points  [" << bar << "] " << (m_sensorReference.tell()*100)/last << "%" << std::flush;
	}
	const LaserReading* lreadReference = dynamic_cast<const LaserReading*>(m_sensorReference.next());
	if (lreadReference){
	    m_detector->detect(*lreadReference, m_pointsReference[i]);
	    m_posesReference[i] = lreadReference->getLaserPose();
	    i++;
	}
    }
    m_sensorReference.seek(position);
    std::cout << " done." << std::endl;
}

void describeLog(){
    unsigned int i = 0;
    unsigned int position = m_sensorReference.tell();
    m_sensorReference.seek(0,END);
    unsigned int last = m_sensorReference.tell();
    m_sensorReference.seek(0);
    
    std::string bar(50, ' ');
    bar[0] = '#';
    unsigned int progress = 0;
    
    while(!m_sensorReference.end()){
	unsigned int currentProgress = (m_sensorReference.tell()*100)/last;
	if (progress < currentProgress){
	    progress = currentProgress;
	    bar[progress/2] = '#';
	    std::cout << "\rDescribing points  [" << bar << "] " << (m_sensorReference.tell()*100)/last << "%" << std::flush;
	}
	const LaserReading* lreadReference = dynamic_cast<const LaserReading*>(m_sensorReference.next());
	if (lreadReference){
	    for(unsigned int j = 0; j < m_pointsReference[i].size(); j++){
		m_pointsReference[i][j]->setDescriptor(m_descriptor->describe(*m_pointsReference[i][j], *lreadReference));
	    }
	    i++;
	}
    }
    m_sensorReference.seek(position);
    std::cout << " done." << std::endl;
}

void computeBoundingBox(){
    double minX =  1e17, minY =  1e17;
    double maxX = -1e17, maxY = -1e17;
    unsigned int position = m_sensorReference.tell();
    m_sensorReference.seek(0,END);
    unsigned int last = m_sensorReference.tell();
    m_sensorReference.seek(0);
    
    std::string bar(50, ' ');
    bar[0] = '#';
    unsigned int progress = 0;
    
    while(!m_sensorReference.end()){
	unsigned int currentProgress = (m_sensorReference.tell()*100)/last;
	if (progress < currentProgress){
	    progress = currentProgress;
	    bar[progress/2] = '#';
	    std::cout << "\rComputing Bounding Box  [" << bar << "] " << (m_sensorReference.tell()*100)/last << "%" << std::flush;
	}
	const LaserReading* lreadReference = dynamic_cast<const LaserReading*>(m_sensorReference.next());
	if (lreadReference){
	    const std::vector<Point2D>& points = lreadReference->getWorldCartesian();
	    const std::vector<double>& rho = lreadReference->getRho();
	    for(unsigned int i = 0; i < points.size(); i++){
		if(rho[i] >= lreadReference->getMaxRange()) continue;
		minX = minX < points[i].x ? minX : points[i].x;
		minY = minY < points[i].y ? minY : points[i].y;
		maxX = maxX > points[i].x ? maxX : points[i].x;
		maxY = maxY > points[i].y ? maxY : points[i].y;
	    }
	}
    }
    double bbX = maxX - minX, bbY = maxY - minY;
    if(bbX < bbY) { // making it landscape
	uint tmp = sizeX;
	sizeX = sizeY;
	sizeY = tmp;
// 	bbX = maxY - minY, bbY = maxX - minX;
    }
    double scaleX = double(sizeX - 2*border)/bbX;
    double scaleY = double(sizeY - 2*border)/bbY;
    scaleFactor = scaleX > scaleY ? scaleY : scaleX;
    offsetX = minX * scaleFactor - border, offsetY = minY * scaleFactor - border;
    m_sensorReference.seek(position);
    sizeX = bbX*scaleFactor + 2*border;
    sizeY = bbY*scaleFactor + 2*border;
    std::cout << " done." << std::endl;
    std::cout << "Bounding Box = " << bbX << "x" << bbY << std::endl;
}

void computeMap(){
    unsigned int position = m_sensorReference.tell();
    m_sensorReference.seek(0,END);
    unsigned int last = m_sensorReference.tell();
    m_sensorReference.seek(0);
    
    std::string bar(50, ' ');
    bar[0] = '#';
    unsigned int progress = 0;
    
    cairo_translate(cairoMap, -offsetX, -offsetY);
    cairo_scale(cairoMap, scaleFactor, scaleFactor);
    cairo_set_line_cap(cairoMap, CAIRO_LINE_CAP_ROUND);
    cairo_set_line_width(cairoMap, 1./scaleFactor);
    while(!m_sensorReference.end()){
		unsigned int currentProgress = (m_sensorReference.tell()*100)/last;
		if (progress < currentProgress){
			progress = currentProgress;
			bar[progress/2] = '#';
			std::cout << "\rComputing Map  [" << bar << "] " << (m_sensorReference.tell()*100)/last << "%" << std::flush;
		}
		const LaserReading* lreadReference = dynamic_cast<const LaserReading*>(m_sensorReference.next());
		if (lreadReference){
			const std::vector<Point2D>& points = lreadReference->getWorldCartesian();
			const std::vector<double>& rho = lreadReference->getRho();
			for(unsigned int i = 0; i < points.size(); i+=2){
				if(rho[i] >= lreadReference->getMaxRange()) continue;
		/*		cairo_move_to(cairoMap, points[i].x * scaleFactor - offsetX, points[i].y * scaleFactor - offsetY);
				cairo_line_to(cairoMap, points[i].x * scaleFactor - offsetX, points[i].y * scaleFactor - offsetY);*/
				cairo_move_to(cairoMap, points[i].x, points[i].y);
				cairo_line_to(cairoMap, points[i].x, points[i].y);
				cairo_stroke(cairoMap);
			}
		}
    }
    m_sensorReference.seek(position);
    std::cout << " done." << std::endl;
}

int main(int argc, char **argv){

    std::string filename("");
	unsigned int scale = 5, dmst = 2, window = 3, detectorType = 0, descriptorType = 0, distanceType = 2, strategy = 0;
    double baseSigma = 0.2, sigmaStep = 1.4, minPeak = 0.34, minPeakDistance = 0.001, success = 0.95, inlier = 0.4, matchingThreshold = 0.4;
	bool useMaxRange = false;
	
	int i = 1;
	while(i < argc){
		if(strncmp("-filename", argv[i], sizeof("-filename")) == 0 ){
			filename = argv[++i];
			i++;
		} else if(strncmp("-detector", argv[i], sizeof("-detector")) == 0 ){
			detectorType = atoi(argv[++i]);
			i++;
		} else if(strncmp("-descriptor", argv[i], sizeof("-descriptor")) == 0 ){
			descriptorType = atoi(argv[++i]);
			i++;
		} else if(strncmp("-corresp", argv[i], sizeof("-corresp")) == 0 ){
			corresp = atoi(argv[++i]);
			i++;
		}  else if(strncmp("-distance", argv[i], sizeof("-distance")) == 0 ){
			distanceType = atoi(argv[++i]);
			i++;
		} else if(strncmp("-baseSigma", argv[i], sizeof("-baseSigma")) == 0 ){
			baseSigma = strtod(argv[++i], NULL);
			i++;
		} else if(strncmp("-sigmaStep", argv[i], sizeof("-sigmaStep")) == 0 ){
			sigmaStep = strtod(argv[++i], NULL);
			i++;
		} else if(strncmp("-minPeak", argv[i], sizeof("-minPeak")) == 0 ){
			minPeak = strtod(argv[++i], NULL);
			i++;
		} else if(strncmp("-minPeakDistance", argv[i], sizeof("-minPeakDistance")) == 0 ){
			minPeakDistance = strtod(argv[++i], NULL);
			i++;
		} else if(strncmp("-scale", argv[i], sizeof("-scale")) == 0 ){
			scale = atoi(argv[++i]);
			i++;
		} else if(strncmp("-dmst", argv[i], sizeof("-dmst")) == 0 ){
			dmst = atoi(argv[++i]);
			i++;
		} else if(strncmp("-window", argv[i], sizeof("-window")) == 0 ){
			scale = atoi(argv[++i]);
			i++;
		} else if(strncmp("-acceptanceSigma", argv[i], sizeof("-acceptanceSigma")) == 0 ){
			acceptanceSigma = strtod(argv[++i], NULL);
			i++;
		} else if(strncmp("-success", argv[i], sizeof("-success")) == 0 ){
			success = strtod(argv[++i], NULL);
			i++;
		} else if(strncmp("-inlier", argv[i], sizeof("-inlier")) == 0 ){
			inlier = strtod(argv[++i], NULL);
			i++;
		} else if(strncmp("-matchingThreshold", argv[i], sizeof("-matchingThreshold")) == 0 ){
			matchingThreshold = strtod(argv[++i], NULL);
			i++;
		} else {
			i++;
		}
    }
    
    if(!filename.size()){
		help();
		exit(-1);
    }
    
    CarmenLogWriter writer;
    CarmenLogReader reader;
    
    m_sensorReference = LogSensorStream(&reader, &writer);
    
    m_sensorReference.load(filename);
    
    SimpleMinMaxPeakFinder *m_peakMinMax = new SimpleMinMaxPeakFinder(minPeak, minPeakDistance);
    
    
    std::string detector("");
    switch(detectorType){
	case 0:
	    m_detectorCurvature = new CurvatureDetector(m_peakMinMax, scale, baseSigma, sigmaStep, dmst);
		m_detectorCurvature->setUseMaxRange(useMaxRange);
	    m_detector = m_detectorCurvature;
	    detector = "curvature";
	    break;
	case 1:
	    m_detectorNormalEdge = new NormalEdgeDetector(m_peakMinMax, scale, baseSigma, sigmaStep, window);
	    m_detectorNormalEdge->setUseMaxRange(useMaxRange);
		m_detector = m_detectorNormalEdge;
	    detector = "edge";
	    break;
	case 2:
	    m_detectorNormalBlob = new NormalBlobDetector(m_peakMinMax, scale, baseSigma, sigmaStep, window);
		m_detectorNormalBlob->setUseMaxRange(useMaxRange);
	    m_detector = m_detectorNormalBlob;
	    detector = "blob";
	    break;
	case 3:
	    m_detectorRange = new RangeDetector(m_peakMinMax, scale, baseSigma, sigmaStep);
		m_detectorRange->setUseMaxRange(useMaxRange);
	    m_detector = m_detectorRange;
	    detector = "range";
	    break;
	default:
	    std::cerr << "Wrong detector type" << std::endl;
	    exit(-1);
    }
    
    HistogramDistance<double> *dist = NULL;
    
    std::string distance("");
    switch(distanceType){
	case 0:
	    dist = new EuclideanDistance<double>();
	    distance = "euclid";
	    break;
	case 1:
	    dist = new Chi2Distance<double>();
	    distance = "chi2";
	    break;
	case 2:
	    dist = new SymmetricChi2Distance<double>();
	    distance = "symchi2";
	    break;
	case 3:
	    dist = new BatthacharyyaDistance<double>();
	    distance = "batt";
	    break;
	case 4:
	    dist = new KullbackLeiblerDistance<double>();
	    distance = "kld";
	    break;
	case 5:
	    dist = new JensenShannonDistance<double>();
	    distance = "jsd";
	    break;
	default:
	    std::cerr << "Wrong distance type" << std::endl;
	    exit(-1);
    }
    
    std::string descriptor("");
    switch(descriptorType){
	case 0:
	    m_betaGenerator = new BetaGridGenerator(0.02, 0.5, 4, 12);
	    m_betaGenerator->setDistanceFunction(dist);
	    m_descriptor = m_betaGenerator;
	    descriptor = "beta";
	    break;
	case 1:
	    m_shapeGenerator = new ShapeContextGenerator(0.02, 0.5, 4, 12);
	    m_shapeGenerator->setDistanceFunction(dist);
	    m_descriptor = m_shapeGenerator;
	    descriptor = "shape";
	    break;
	default:
	    std::cerr << "Wrong descriptor type" << std::endl;
	    exit(-1);
    }
    
    std::cerr << "Processing file:\t" << filename << "\nDetector:\t\t" << detector << "\nDescriptor:\t\t" << descriptor << "\nDistance:\t\t" << distance << std::endl;
    
    switch(strategy){
	case 0:
	    m_ransac = new RansacFeatureSetMatcher(acceptanceSigma * acceptanceSigma * 5.99, success, inlier, matchingThreshold, acceptanceSigma * acceptanceSigma * 3.84, false);
	    break;
	case 1:
	    m_ransac = new RansacMultiFeatureSetMatcher(acceptanceSigma * acceptanceSigma * 5.99, success, inlier, matchingThreshold, acceptanceSigma * acceptanceSigma * 3.84, false);
	    break;
	default:
	    std::cerr << "Wrong strategy type" << std::endl;
	    exit(-1);
    }

    m_sensorReference.seek(0,END);
    unsigned int end = m_sensorReference.tell();
    m_sensorReference.seek(0,BEGIN);

    m_pointsReference.resize(end + 1);
    m_posesReference.resize(end + 1);
        
    computeBoundingBox();
    
    detectLog();
    describeLog();
    
    std::stringstream mapFile;
    mapFile << filename << "_map.png";
/*    cairo_surface_t * cairoMapSurface = cairo_pdf_surface_create(mapFile.str().c_str(), sizeX, sizeY);
    cairo_surface_t * cairoOutSurface = cairo_pdf_surface_create(outFile.str().c_str(), sizeX, sizeY);*/
    cairo_surface_t * cairoMapSurface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, sizeX, sizeY);
//     cairo_surface_t * cairoMapSurface = cairo_svg_surface_create(mapFile.str().c_str(), sizeX, sizeY);
    cairoMap = cairo_create(cairoMapSurface);
    
    std::stringstream outDir;
    outDir << filename << "_" << detector << "_" << descriptor << "_" << distance << "/";
    mkdir(outDir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    
    computeMap();

    std::string bar(50,' ');
    bar[0] = '#';
    unsigned int progress = 0;
    
    
    for(unsigned int i =0; i < m_pointsReference.size(); i++){
	unsigned int currentProgress = (i*100)/(m_pointsReference.size() - 1);
	if (progress < currentProgress){
	    progress = currentProgress;
	    bar[progress/2] = '#';
	    std::cout << "\rMatching  [" << bar << "] " << progress << "%" << std::flush;
	}
	std::stringstream outFile;
	outFile << outDir.str() << "frame_";
	outFile.width(4);
	outFile.fill('0');
	outFile << std::right << i << ".png";
	cairo_surface_t * cairoOutSurface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, sizeX, sizeY);
// 	cairo_surface_t * cairoOutSurface = cairo_svg_surface_create(outFile.str().c_str(), sizeX, sizeY);
	cairoOut = cairo_create(cairoOutSurface);
	cairo_translate(cairoOut, -offsetX, -offsetY);
	cairo_scale(cairoOut, scaleFactor, scaleFactor);
	cairo_set_line_width(cairoOut, 1./scaleFactor);
	match(i);
 	cairo_surface_write_to_png(cairoOutSurface, outFile.str().c_str());
	cairo_surface_destroy(cairoOutSurface);
	cairo_destroy(cairoOut);
    }
    std::cout << " done." << std::endl;
    
    cairo_surface_write_to_png(cairoMapSurface, mapFile.str().c_str());
    
    cairo_surface_destroy(cairoMapSurface);
    cairo_destroy(cairoMap);
//     cairo_show_page(cairoOut);
}

