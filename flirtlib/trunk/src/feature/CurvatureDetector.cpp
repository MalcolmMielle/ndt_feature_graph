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

#include "CurvatureDetector.h"

#include <utils/Regression.h>

CurvatureDetector::CurvatureDetector(const PeakFinder* peak, unsigned int scales, double sigma, double step, unsigned int dmst):
    m_peakFinder(peak),
    m_scaleNumber(scales),
    m_baseSigma(sigma),
    m_sigmaStep(step),
    m_useMaxRange(false),
    m_dmst(dmst)
{
    computeScaleBank();
}

	
unsigned int CurvatureDetector::detect(const LaserReading& reading, std::vector<InterestPoint*>& _point) const
{
    Graph graph;
    std::vector< std::vector<Point2D> > operatorA;
    std::vector< std::vector<double> > signalDiff;
    std::vector< std::vector<unsigned int> > indexes;
    return detect(reading, _point, graph, operatorA, signalDiff, indexes);
}
	
unsigned int CurvatureDetector::detect(const LaserReading& reading, std::vector<InterestPoint*>& point,
				       Graph& graph,
				       std::vector< std::vector<Point2D> >& operatorA, 
				       std::vector< std::vector<double> >& signalDiff,
				       std::vector< std::vector<unsigned int> >& indexes) const
{
    std::vector<unsigned int> maxRangeMapping;
    std::vector<Point2D> graphPoints;
    computeGraph(reading, graphPoints, graph, maxRangeMapping);
    detect(graph, graphPoints, operatorA, signalDiff, indexes);
    return computeInterestPoints(reading, operatorA, point, indexes, maxRangeMapping);
}

unsigned int CurvatureDetector::detect( const LaserReading& reading, std::vector<InterestPoint*>& point,
					std::vector< double >& signal,
					std::vector< std::vector<double> >& signalSmooth,
					std::vector< std::vector<double> >& signalDiff,
					std::vector< std::vector<unsigned int> >& indexes) const
{
    std::vector<unsigned int> maxRangeMapping;
    std::vector<Point2D> graphPoints;
    Graph graph;
    
    std::vector< std::vector<Point2D> > operatorA;
    computeGraph(reading, graphPoints, graph, maxRangeMapping);
    detect(graph, graphPoints, operatorA, signalDiff, indexes);
    signal.resize(graphPoints.size());
    signalSmooth.resize(m_scales.size(), std::vector<double> (graphPoints.size()));
    for(unsigned int i = 0; i < graphPoints.size(); i++){
		Point2D difference = graphPoints[i] - reading.getLaserPose();
		signal[i] = hypot(difference.x, difference.y);
		for(unsigned int scale = 0; scale < m_scales.size(); scale++){
			difference = operatorA[scale][i] - reading.getLaserPose();
			signalSmooth[scale][i] = hypot(difference.x, difference.y);
		}
    }
    return computeInterestPoints(reading, operatorA, point, indexes, maxRangeMapping);
}

void CurvatureDetector::computeGraph(const LaserReading& reading, std::vector<Point2D>& graphPoints, Graph& graph, std::vector<unsigned int>& maxRangeMapping) const
{
    const std::vector<Point2D>& worldPoints = reading.getWorldCartesian();
    graphPoints.reserve(worldPoints.size());
    std::vector<GraphEdge> edges;
    std::vector< boost::property < boost::edge_weight_t, double > > weights;
    edges.reserve(worldPoints.size()*worldPoints.size());
    weights.reserve(worldPoints.size()*worldPoints.size());
    unsigned int currentVertexNumber = 0;
    for(unsigned int i = 0; i < worldPoints.size(); i++){
		if(m_useMaxRange || reading.getRho()[i] < reading.getMaxRange()){
			graphPoints.push_back(worldPoints[i]);
			maxRangeMapping.push_back(i);
			unsigned int targetVertexNumber = currentVertexNumber + 1;
			for(unsigned int j = i + 1; j < worldPoints.size(); j++){
				if(m_useMaxRange || reading.getRho()[j] < reading.getMaxRange()){
					Point2D difference = worldPoints[i] - worldPoints[j];
		    double weight = hypot(difference.x, difference.y);
					edges.push_back(GraphEdge(currentVertexNumber,targetVertexNumber));
		    weights.push_back(weight);
					targetVertexNumber++;
				}
			}
			currentVertexNumber++;
		}
    }
    
    MatrixGraph costGraph(currentVertexNumber);
    for(unsigned int i = 0; i < edges.size(); i++){
		boost::add_edge(edges[i].first, edges[i].second, weights[i], costGraph);
    }
    boost::remove_edge(0, currentVertexNumber - 1, costGraph);
    
    for(unsigned int iter = 0; iter <= m_dmst; iter++){
		std::vector < boost::graph_traits < Graph >::vertex_descriptor > predecessor(boost::num_vertices(costGraph));
                boost::prim_minimum_spanning_tree(costGraph, &predecessor[0]);
                for(unsigned int i = 1; i < predecessor.size(); i++){
			boost::add_edge(predecessor[i], i, boost::get(boost::edge_weight, costGraph, boost::edge(predecessor[i], i, costGraph).first), graph);
			boost::remove_edge(predecessor[i], i, costGraph);
		}
    }
}

void CurvatureDetector::detect(const Graph& graph, const std::vector<Point2D>& graphPoints, std::vector< std::vector<Point2D> >& operatorA, std::vector< std::vector<double> >& signalDiff, 
				       std::vector< std::vector<unsigned int> >& indexes) const
{
    operatorA.resize(m_scales.size());
    signalDiff.resize(m_scales.size());
    indexes.resize(m_scales.size());
    
    unsigned int vertexNumber = boost::num_vertices(graph);
    std::vector< std::vector< double > > distances(vertexNumber, std::vector<double>(vertexNumber));
    boost::johnson_all_pairs_shortest_paths(graph, distances);
   
    for(unsigned int scale = 0; scale < m_scales.size(); scale++){
		double currentScale = m_scales[scale];
		double normalizer = sqrt(2*M_PI)*currentScale;
		std::vector<double> densities(vertexNumber, 0.);
		operatorA[scale].resize(vertexNumber, Point2D());
		signalDiff[scale].resize(vertexNumber,0.);
		double weights[vertexNumber][vertexNumber];
		double weightNormalizer[vertexNumber];
		for(unsigned int vertex1 = 0; vertex1 < vertexNumber; vertex1++){
			for(unsigned int vertex2 = 0; vertex2 < vertexNumber; vertex2++){
				weights[vertex1][vertex2] = normalizer * exp(-distances[vertex1][vertex2] * distances[vertex1][vertex2]/(2 * currentScale * currentScale));
				densities[vertex1] += weights[vertex1][vertex2];
			}
		}
		for(unsigned int vertex1 = 0; vertex1 < vertexNumber; vertex1++){
			weightNormalizer[vertex1] = 0.;
			for(unsigned int vertex2 = 0; vertex2 < vertexNumber; vertex2++){
				weights[vertex1][vertex2] /= densities[vertex1] * densities[vertex2];
				weightNormalizer[vertex1] += weights[vertex1][vertex2];
			}
		}
		for(unsigned int vertex1 = 0; vertex1 < vertexNumber; vertex1++){
			for(unsigned int vertex2 = 0; vertex2 < vertexNumber; vertex2++){
				operatorA[scale][vertex1] = operatorA[scale][vertex1] + weights[vertex1][vertex2] * graphPoints[vertex2];
			}
			operatorA[scale][vertex1] = operatorA[scale][vertex1] * ( 1. / weightNormalizer[vertex1]);
			Point2D pointDifference = operatorA[scale][vertex1] - graphPoints[vertex1];
			double temporary = 2 * (1/currentScale) * hypot(pointDifference.x, pointDifference.y);
			signalDiff[scale][vertex1] =  temporary * exp(-temporary);
		}
		for(unsigned int j = 2; j < signalDiff[scale].size() - 2; j++){
			if(m_peakFinder->isPeak(signalDiff[scale], j)){
				indexes[scale].push_back(j);
			}
		}
    }
}

unsigned int CurvatureDetector::computeInterestPoints(const LaserReading& reading, const std::vector< std::vector<Point2D> >& operatorA, std::vector<InterestPoint*>& point, 
						      const std::vector< std::vector<unsigned int> >& indexes, std::vector<unsigned int>& maxRangeMapping) const
{
    point.clear();
    point.reserve(reading.getRho().size());
    const std::vector<Point2D>& worldPoints = reading.getWorldCartesian();
    for(unsigned int i = 0; i < indexes.size(); i++){
		for(unsigned int j = 0; j < indexes[i].size(); j++){
			OrientedPoint2D pose;
			unsigned int pointIndex = maxRangeMapping[indexes[i][j]];
			
			// Reomoving the detection in the background and pushing it to the foreground
			double rangeBefore = (pointIndex > 1)? reading.getRho()[pointIndex - 1] : reading.getMaxRange();
			double rangeAfter = (pointIndex < worldPoints.size() - 1)? reading.getRho()[pointIndex + 1] : reading.getMaxRange();
			double rangeCurrent = reading.getRho()[pointIndex];
			if(rangeBefore < rangeAfter){
				if(rangeBefore < rangeCurrent){
					pointIndex = pointIndex - 1;
				}
			} else if(rangeAfter < rangeCurrent){
				pointIndex = pointIndex + 1;
			}
			
			// Removing max range reading
			if(reading.getRho()[pointIndex] >= reading.getMaxRange()){
				continue;
			}

			
			pose.x =  (worldPoints[pointIndex]).x;
			pose.y =  (worldPoints[pointIndex]).y;
			Point2D difference = operatorA[i][indexes[i][j]] - worldPoints[pointIndex];
			pose.theta = atan2(difference.y, difference.x);
			
			bool exists = false;
			for(unsigned int k = 0; !exists && k < point.size(); k++){
				exists = exists || (fabs(pose.x - point[k]->getPosition().x) <= 0.2 &&  fabs(pose.y - point[k]->getPosition().y) <= 0.2);
			}
			if(exists) continue;

			double distance = 2. * m_scales[i];
			Point2D diffStart = pose - worldPoints.front();
			Point2D diffEnd = pose - worldPoints.back();
			
			if(hypot(diffStart.x, diffStart.y) < distance || hypot(diffEnd.x, diffEnd.y) < distance){
				continue;
			}
			
			std::vector<Point2D> support;
			for(unsigned int k = 0; k < worldPoints.size(); k++){
				Point2D diff2 = pose - worldPoints[k];
				if(hypot(diff2.x, diff2.y) < distance) support.push_back(worldPoints[k]);
			}

// 	    LineParameters param = computeNormals(support);
// 	    pose.theta = normAngle(param.alpha, - M_PI);    
	    
			InterestPoint *interest = new InterestPoint(pose, distance);
		//  InterestPoint *interest = new InterestPoint(pose, m_scales[i]);
			interest->setSupport(support);
			interest->setScaleLevel(i);
			point.push_back(interest);
		}
    }
    return point.size();
    
}

void CurvatureDetector::computeScaleBank()
{
    m_scales.resize(m_scaleNumber);
    for(unsigned int i = 0; i < m_scales.size(); i++){
		m_scales[i] = m_baseSigma * pow(m_sigmaStep, i);
    }
}
