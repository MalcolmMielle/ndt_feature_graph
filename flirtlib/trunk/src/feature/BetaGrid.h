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



#ifndef BETAGRID_H_
#define BETAGRID_H_

#include <feature/Descriptor.h>
#include <feature/InterestPoint.h>
#include <sensors/LaserReading.h>
#include <utils/HistogramDistances.h>
#include <vector>

/**
 * Representation of the Beta grid descriptor.
 * The class represents the Beta grid descriptor presented in the paper. It provides the interface
 * to compare different descriptors, as well as to inspect the internal histogram representation.
 * 
 *
 * @author Gian Diego Tipaldi
 */


class BetaGrid: public Descriptor{
    public:
		virtual Descriptor* clone() const;
		
		/** Default destructor. */
		virtual ~BetaGrid() { }

		/** 
		 * Returns \f$ \beta \f$-Grid in the form of a onedimensional histogram. The resulting vector represents the feature descriptor.
		 *
		 */
		virtual void getFlatDescription(std::vector<double>& description) const;

		/** 
		 * Returns \f$ \beta \f$-Grid and its variance in the form of onedimensional histograms. The resulting vectors represent the feature descriptor and the variance.
		 * @param description the descriptor vector
		 * @param weight the descriptor variance
		 *
		 */
		virtual void getWeightedFlatDescription(std::vector<double>& description, std::vector<double>& weight) const;

		/** 
		 * Implements the distance function between two shape context descriptors. 
		 * The actual distance is computed using the histogram distance defined in #m_distanceFunction .
		 * 
		 */
		virtual double distance(const Descriptor* descriptor) const;
		
		/** Returns the \f$ \beta \f$-Grid in the form of a bidimensional histogram. The first dimension represents the angle and the second dimension the distance. */
		inline const std::vector< std::vector< double > >& getHistogram() const
			{return m_histogram;}
		
		/** Returns the \f$ \beta \f$-Grid in the form of a bidimensional histogram. The first dimension represents the angle and the second dimension the distance. */
		inline std::vector< std::vector< double > >& getHistogram() 
			{return m_histogram;}
		
		/** Returns the \f$ \beta \f$-Grid variance in the form of a bidimensional histogram. The first dimension represents the angle and the second dimension the distance. */
		inline const std::vector< std::vector< double > >& getVariance() const
			{return m_variance;}
		
		/** Returns the \f$ \beta \f$-Grid variance in the form of a bidimensional histogram. The first dimension represents the angle and the second dimension the distance. */
		inline std::vector< std::vector< double > >& getVariance() 
			{return m_variance;}
		
		/** Returns the histogram of occupied cells computed to obtain the \f$ \beta \f$-Grid descriptor. The first dimension represents the angle and the second dimension the distance. */
		inline const std::vector< std::vector< double > >& getHit() const
			{return m_hit;}
		
		/** Returns the histogram of occupied cells computed to obtain the \f$ \beta \f$-Grid descriptor. The first dimension represents the angle and the second dimension the distance. */
		inline std::vector< std::vector< double > >& getHit() 
			{return m_hit;}
		
		/** Returns the histogram of free cells computed to obtain the \f$ \beta \f$-Grid descriptor. The first dimension represents the angle and the second dimension the distance. */
		inline const std::vector< std::vector< double > >& getMiss() const
			{return m_miss;}
		
		/** Returns the histogram of free cells computed to obtain the \f$ \beta \f$-Grid descriptor. The first dimension represents the angle and the second dimension the distance. */
		inline std::vector< std::vector< double > >& getMiss() 
			{return m_miss;}
		
		/** Returns the distance function used to compeare two \f$ \beta \f$-Grids. */
		inline const HistogramDistance<double>* getDistanceFunction() const
			{return m_distanceFunction;}
		
		/** Sets the distance function used to compeare two \f$ \beta \f$-Grids. */
		inline void setDistanceFunction(const HistogramDistance<double>* distanceFunction)
			{m_distanceFunction = distanceFunction;}
		
	protected:
		friend class boost::serialization::access;

		/** Serializes the class using boost::serialization. */ 
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version);

		const HistogramDistance<double> *m_distanceFunction; /**< The distance function used to compeare two \f$ \beta \f$-Grids. */ 
		std::vector< std::vector< double > > m_histogram; /**< The histogram representing the \f$ \beta \f$-Grid. */
		std::vector< std::vector< double > > m_variance; /**< The histogram representing the variance of the \f$ \beta \f$-Grid. */
		std::vector< std::vector< double > > m_hit; /** The histogram of occupied cells. */
		std::vector< std::vector< double > > m_miss; /** The histogram of free  cells. */
};

BOOST_CLASS_EXPORT(BetaGrid);

template <class Archive>
void BetaGrid::serialize(Archive & ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Descriptor);
    ar & BOOST_SERIALIZATION_NVP(m_histogram);
    ar & BOOST_SERIALIZATION_NVP(m_variance);
    ar & BOOST_SERIALIZATION_NVP(m_hit);
    ar & BOOST_SERIALIZATION_NVP(m_miss);
}

/**
 * Representation of the Beta grid descriptor generator. 
 * The class generates the Beta grid descriptor presented in the paper. It provides the interface
 * to generate the descriptor from a laser reading and set the descriptor size.
 * 
 *
 * @author Gian Diego Tipaldi
 */

class BetaGridGenerator: public DescriptorGenerator {
    public:
		/** 
		 * Constructor. It sets the size of the generated \f$ \beta \f$-Grid and the number of bins.
		 * @param minRho The minimum distance for a point to be included in the descriptor.
		 * @param maxRho The maximum distance for a point to be included in the descriptor.
		 * @param binRho The number of bins in the radial coordinate of the descriptor.
		 * @param binPhi The number of bins in the angular coordinate of the descriptor.
		 */
		BetaGridGenerator(double minRho, double maxRho, unsigned int binRho, unsigned int binPhi);
		
		/** 
		 * Constructor. It sets the size of the generated \f$ \beta \f$-Grid and the number of bins.
		 * @param rhoEdges Vector containing the edges of the radial coordinate of the desciptor.
		 * @param phiEdges Vector containing the edges of the angular coordinate of the desciptor.
		 *
		 * \note
		 * A point is in the bin i,j if its radial distance is between rhoEdges[i] and rhoEdges[i + 1] and
		 * its angular distance between phiEdges[j] and phiEdges[j + 1]
		 */
		BetaGridGenerator(const std::vector<double>& rhoEdges, const std::vector<double>& phiEdges);

		/** Default destructor. */
	virtual ~BetaGridGenerator() { }
		
		virtual Descriptor* describe(const InterestPoint& point, const LaserReading& reading);
		
		virtual Descriptor* describe(const OrientedPoint2D& point, const LaserReading& reading);

		/** Sets the radial and angular edges of the descriptor. 
		 * @param minRho The minimum distance for a point to be included in the descriptor.
		 * @param maxRho The maximum distance for a point to be included in the descriptor.
		 * @param binRho The number of bins in the radial coordinate of the descriptor.
		 * @param binPhi The number of bins in the angular coordinate of the descriptor.
		 */
		void setEdges(double minRho, double maxRho, unsigned int binRho, unsigned int binPhi);
	
		/** Gets the minimum distance for a point to be included in the descriptor. */
		inline double getMinRho() const
			{return m_rhoEdges.front();}
	
		/** Gets the maximum distance for a point to be included in the descriptor. */
		inline double getMaxRho() const
			{return m_rhoEdges.back();}
	
		/** Gets the number of bins in the radial coordinate of the descriptor. */
		inline unsigned int getBinRho() const
			{return m_rhoEdges.size() - 1;}
	
		/** Gets the number of bins in the angular coordinate of the descriptor. */
		inline unsigned int getBinPhi() const
			{return m_phiEdges.size() - 1;}
	
		/** Gets the vector containing the edges of the radial coordinate of the desciptor. */
		inline const std::vector<double>& getRhoEdges() const
			{return m_rhoEdges;}
	
		/** Gets the vector containing the edges of the angular coordinate of the desciptor. */
		inline const std::vector<double>& getPhiEdges() const
			{return m_phiEdges;}
		
		/** Sets the radial and angular edges of the descriptor. 
		 * @param rhoEdges Vector containing the edges of the radial coordinate of the desciptor.
		 * @param phiEdges Vector containing the edges of the angular coordinate of the desciptor.
		 *
		 * \note
		 * A point is in the bin i,j if its radial distance is between rhoEdges[i] and rhoEdges[i + 1] and
		 * its angular distance between phiEdges[j] and phiEdges[j + 1]
		 */
		inline void setEdges(const std::vector<double>& rhoEdges, const std::vector<double>& phiEdges)
			{m_rhoEdges = rhoEdges; m_phiEdges = phiEdges;}
		
		/** Gets the distance function used to compare two descriptors */
		inline const HistogramDistance<double>* getDistanceFunction() const
			{return m_distanceFunction;}
		
		/** Sets the distance function used to compare two descriptors */
		inline void setDistanceFunction(const HistogramDistance<double>* distanceFunction)
			{m_distanceFunction = distanceFunction;}
		
    protected:
		/** Checks if two segments intersect each other. It is used to perform ray-tracing in the polar grid. */
		bool intersectSegment2Segment(const Point2D& segment1Start, const Point2D& segment1End, const Point2D& segment2Start, const Point2D& segment2End);
		
		/** Checks if a segment intersect an arc. It is used to perform ray-tracing in the polar grid. */
		int intersectSegment2Arc(const Point2D& segmentStart, const Point2D& segmentEnd, const Point2D& arcStart, const Point2D& arcEnd, const Point2D& arcCenter);
		
		std::vector<double> m_rhoEdges; /**< The vector containing the edges of the radial coordinate of the desciptor. */
		std::vector<double> m_phiEdges; /**< The vector containing the edges of the angular coordinate of the desciptor. */
		const HistogramDistance<double> *m_distanceFunction; /**< The distance function used to compare two descriptors */
};

#endif
