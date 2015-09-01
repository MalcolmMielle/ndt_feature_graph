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

#ifndef RANSACFEATURESETMATCHER_H_
#define RANSACFEATURESETMATCHER_H_

#include <feature/AbstractFeatureSetMatcher.h>
#include <utils/PoseEstimation.h>
#include <vector>
#include <utility>

/**
 * Representation of the RANSAC algorithm for feature matching.
 * The class represents the RANSAC algorithm for matching two different feature sets. 
 * The matching result is an Euclidean transformation (rotation + translation) encoded as a point in \f$ \mathcal{SO}(2) \f$.
 * The algorithm uses the Nearest Neighbour strategy to obtain the possible correspondences from the features' descriptors. 
 *
 * @author Gian Diego Tipaldi
 */

class RansacFeatureSetMatcher: public AbstractFeatureSetMatcher {
    public:
	/**
	 * Constructor. Constructs and initializes the RANSAC algorithm.
	 *
	 * @param acceptanceThreshold The maximum distance (in meters) for a point to be considered in the inlier set.
	 * @param successProbability The probability of finding a correct match if exists.
	 * @param inlierProbability The probability of a generic correspondence to be an inlier.
	 * @param distanceThreshold The maximum distance (dimensionless) for two descriptors to be considered as a valid match. This threshold depends on the actual distance employed.
	 * @param rigidityThreshold The maximum value (in meters) of difference between the relative distance of two interest points. This implements a rigidity check in the RANSAC hypothesis generation.
	 * @param adaptive The flag to set the adaptive strategy to compute the number of RANSAC iterations (EXPERIMENTAL!!!).
	 */
	RansacFeatureSetMatcher(double acceptanceThreshold, double successProbability, double inlierProbability, double distanceThreshold, double rigidityThreshold, bool adaptive = false);
	
	/** Default destructor. */
	virtual ~RansacFeatureSetMatcher() { }
	
	virtual double matchSets(const std::vector<InterestPoint *> &reference, const std::vector<InterestPoint *> &data, OrientedPoint2D& transformation) const;
	
	virtual double matchSets(const std::vector<InterestPoint *> &reference, const std::vector<InterestPoint *> &data, OrientedPoint2D& transformation,
				 std::vector< std::pair<InterestPoint *, InterestPoint *> > &correspondences) const;
	
	/** Gets the flag for the adaptive RANSAC */
	inline bool getAdaptive()
	    {return m_adaptive;}
	
	/** Gets the probability of finding a correct match if exists. */ 
	inline double getSuccessProbability()
	    {return m_successProbability;}
	
	/** Gets the probability of a generic correspondence to be an inlier. */ 
	inline double getInlierProbability()
	    {return m_inlierProbability;}
	
	/** Gets the maximum distance (dimensionless) for two descriptors to be considered as a valid match */ 
	inline double getDistanceThreshold()
	    {return m_distanceThreshold;}

	/** Sets the flag for the adaptive RANSAC */
	inline void setAdaptive(bool adaptive)
	    {m_adaptive = adaptive;}
	
	/** Sets the probability of finding a correct match if exists. */ 
	inline void setSuccessProbability(double successProbability)
	    {m_successProbability = successProbability;}
	
	/** Sets the probability of a generic correspondence to be an inlier. */
	inline void setInlierProbability(double inlierProbability)
	    {m_inlierProbability = inlierProbability;}
	
	/** Sets the maximum distance (dimensionless) for two descriptors to be considered as a valid match */ 
	inline void setDistanceThreshold(double distanceThreshold)
	    {m_distanceThreshold = distanceThreshold;}
    
    protected:
	double m_successProbability; /**< The probability of finding a correct match if exists. */
	double m_inlierProbability; /**< The probability of a generic correspondence to be an inlier. */
	double m_distanceThreshold; /**< The maximum distance (dimensionless) for two descriptors to be considered as a valid match. This threshold depends on the actual distance employed. */
	double m_rigidityThreshold; /**< The maximum value (in meters) of difference between the relative distance of two interest points. This implements a rigidity check in the RANSAC hypothesis generation. */
	bool m_adaptive; /**< The flag to set the adaptive strategy to compute the number of RANSAC iterations (EXPERIMENTAL!!!). */
};

#endif

