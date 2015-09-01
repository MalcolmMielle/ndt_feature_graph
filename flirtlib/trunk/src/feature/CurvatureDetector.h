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

#ifndef CURVATUREDETECTOR_H_
#define CURVATUREDETECTOR_H_

#include <feature/InterestPoint.h>
#include <feature/Detector.h>
#include <utils/Convolution.h>
#include <utils/PeakFinder.h>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/graph/graph_utility.hpp>
#include <vector>

 
typedef boost::property<boost::vertex_distance_t, double> DistanceVertexProperty;
typedef boost::property < boost::edge_weight_t, double > WeightEdgeProperty;

typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS, DistanceVertexProperty, WeightEdgeProperty> Graph;
typedef boost::adjacency_matrix < boost::undirectedS, DistanceVertexProperty, WeightEdgeProperty > MatrixGraph;
typedef std::pair<unsigned int, unsigned int> GraphEdge;



/**
 * Representation of the multi scale curvature detector of Unnikrishnan and Hebert.
 * The class represents the curvature based detector of Unnikrishnan and Hebert. It provides the interface to set 
 * the parameters of the detector and to detect interest points in laser readings. 
 *
 * @author Gian Diego Tipaldi
 */
 
class CurvatureDetector: public Detector {
    public:
		/** 
		 * Constructor. Constructs and initialize the curvature based detector.
		 *
		 * @param peak The peak finder used to detect maxima in the signal.
		 * @param scales The number of different scales to consider.
		 * @param sigma The standard deviation of the smoothing kernel for the initial scale (\f$ t_0 \f$ in the paper). 
		 * @param step The scale increment at every new scale (\f$ t_i \f$ in the paper). The standard deviation of the kernel at scale \f$ s \f$ is \f$ t_0 \cdot (t_i)^s \f$
		 * @param dmst The number of minimum spanning tree to approximate the geodesic coordinate (see the paper for more details).
		 */
		CurvatureDetector(const PeakFinder* peak, unsigned int scales = 5, double sigma = 0.2, double step = 1.4, unsigned int dmst = 4);
		
		/** Default destructor. */
		virtual ~CurvatureDetector() { }
	
		virtual unsigned int detect(const LaserReading& reading, std::vector<InterestPoint*>& point) const;
		
		/** 
		 * Detects the interesting points given the laser reading. It also returns the signals used for the computation.
		 *
		 * @return The number of interest points detected.
		 *
		 * @param reading The laser reading.
		 * @param points The detected interest points.
		 * @param graph The graph approximation for the geodesic coordinate.
		 * @param operatorA The result of the smoothing operator on the geodesic coordinate at the different scales..
		 * @param signalDiff The exponential dump applied to the smoothing operator at the different scales.
		 * @param indexes The indexes of the differential operator maxima at the different scales.
		 */
		virtual unsigned int detect(const LaserReading& reading, std::vector<InterestPoint*>& points,
						Graph& graph,
						std::vector< std::vector<Point2D> >& operatorA,
						std::vector< std::vector<double> >& signalDiff,
						std::vector< std::vector<unsigned int> >& indexes) const;
		
		virtual unsigned int detect(const LaserReading& reading, std::vector<InterestPoint*>& point,
						std::vector< double >& signal,
						std::vector< std::vector<double> >& signalSmooth,
						std::vector< std::vector<double> >& signalDiff,
						std::vector< std::vector<unsigned int> >& indexes) const;

		/** 
		 * Detects the interesting points given the graph. It also returns the signals used for the computation.
		 *
		 * @return The number of interest points detected.
		 *
		 * @param graph The graph approximation for the geodesic coordinate.
		 * @param graphPoints The point cloud in Cartesian coordinates.
		 * @param operatorA The result of the smoothing operator on the geodesic coordinate at the different scales..
		 * @param signalDiff The exponential dump applied to the smoothing operator at the different scales.
		 * @param indexes The indexes of the differential operator maxima at the different scales.
		 */
		virtual void detect(const Graph& graph, const std::vector<Point2D>& graphPoints, std::vector< std::vector<Point2D> >& operatorA, 
						std::vector< std::vector<double> >& signalDiff, std::vector< std::vector<unsigned int> >& indexes) const;
		
		/** Sets the number of scales */
		inline void setScaleNumber(unsigned int scales)
			{m_scaleNumber = scales; computeScaleBank();}
		
		/** Sets the number of minimum spanning tree */
		inline void setDmst(unsigned int dmst)
			{m_dmst = dmst;}
		
		/** Sets the standard deviation of the initial scale */
		inline void setBaseSigma(double sigma)
			{m_baseSigma = sigma; computeScaleBank();}
			
		/** Sets the scale increment at each new scale */
		inline void setSigmaStep(double step) 
			{m_sigmaStep = step; computeScaleBank();}
		
		/** Sets the peak finder */
		inline void setPeakFinder(const PeakFinder* peak)
			{m_peakFinder = peak;}
			
		/** Sets the "use max range" flag. The flag enable or disable the use of max range readings. */
		inline void setUseMaxRange(bool use)
			{m_useMaxRange = use;}
		
		/** Gets the number of scales */
		inline unsigned int getScaleNumber() const
			{return m_scaleNumber;}
		
		/** Gets the number of minimum spanning trees */
		inline unsigned int getDmst() const
			{return m_dmst;}
		
		/** Gets the actual scales */
		inline const std::vector<double>& getScales() const
			{return m_scales;}
		
		/** Gets the standard deviation of the initial scale */
		inline double getBaseSigma() const
			{return m_baseSigma;}
		
		/** Gets the sigma increment step */
		inline double getSigmaStep() const
			{return m_sigmaStep;}
		
		/** Gets the actual peak finder */
		inline const PeakFinder* getPeakFinder() const
			{return m_peakFinder;}
		
		/** Gets the "use the max range" flag. */
		inline bool getUseMaxRange()
			{return m_useMaxRange;}
		
    protected:
		/** Computes the graph approximation of the geodesic coordinate. */
		virtual void computeGraph(const LaserReading& reading, std::vector<Point2D>& graphPoints, Graph& graph, std::vector<unsigned int>& maxRangeMapping) const;
		
		/** Computes the interest points given the laser reading and the smoothed operator. It should not be used directly. Use the public functions instead. */
		virtual unsigned int computeInterestPoints(const LaserReading& reading, const std::vector< std::vector<Point2D> >& operatorA, std::vector<InterestPoint*>& point, 
							const std::vector< std::vector<unsigned int> >& indexes, std::vector<unsigned int>& maxRangeMapping) const;
		
		/** Computes the scale bank for the smoothing operators. It is provided for efficiency issues. */
		void computeScaleBank();
		
		const PeakFinder* m_peakFinder; /**< The peak finder used to detect maxima in the signal. */
		unsigned int m_scaleNumber; /**< The numebr of scales */
		double m_baseSigma; /**< The standard deviation of the smoothing kernel for the initial scale (\f$ t_0 \f$ in the paper). */
		double m_sigmaStep; /**< The scale increment at every new scale (\f$ t_i \f$ in the paper). The standard deviation of the kernel at scale \f$ s \f$ is \f$ t_0 \cdot (t_i)^s \f$ */
		bool m_useMaxRange; /**< The "use max range" flag. The flag enable or disable the use of max range readings. */
		unsigned int m_dmst; /**< The number of minimum spanning tree to approximate the geodesic coordinate (see the paper for more details). */
		std::vector<double> m_scales; /**< The precomputed scales for the smoothing operators. It is provided for efficiency issues. */
};

#endif

