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



#ifndef ABSTRACTFEATURESETMATCHER_H_
#define ABSTRACTFEATURESETMATCHER_H_

#include <feature/InterestPoint.h>
#include <geometry/point.h>
#include <vector>
#include <utility>


/**
 * Representation of an abstract algorithm for feature matching.
 * The class represents an abstract algorithm for matching two different feature sets. 
 * The matching result is an Euclidean transformation (rotation + translation) encoded as a point in \f$ \mathcal{SO}(2) \f$.
 *
 *
 * @author Gian Diego Tipaldi
 */

class AbstractFeatureSetMatcher {
    public:
		/** 
		 * Constructor. This is an abstract class, so the constructor only creates the shared part of any specialized class.
		 *
		 * @param acceptanceThreshold The maximum distance (in meters) for a point to be considered in the inlier set.
		 */
		AbstractFeatureSetMatcher(double acceptanceThreshold);

		/** 
		 * Default destructor.
		 *
		 */
		virtual ~AbstractFeatureSetMatcher() { }
		
		/**
		 * Matches two features sets, returning the matching error and the transformation.
		 *
		 * @return The matching error.
		 *
		 * @param reference The reference feature set. It is used as reference frame for the transformation.
		 * @param data The feature set to match. 
		 * @param transformation The restulting transformation. It is used to register the points in data to the points in reference.
		 */
		virtual double matchSets(const std::vector<InterestPoint *> &reference, const std::vector<InterestPoint *> &data, OrientedPoint2D& transformation) const = 0;
		
		/**
		 * Matches two features sets, returning the matching error and the transformation. It also returns the set of corresponding inlier points.
		 *
		 * @return The matching error.
		 *
		 * @param reference The reference feature set. It is used as reference frame for the transformation.
		 * @param data The feature set to match. 
		 * @param transformation The restulting transformation. It is used to register the points in data to the points in reference.
		 * @param correspondences The set of corresponding inlier points. Each individual correspondence is in the format (reference, data).
		 */
		virtual double matchSets(const std::vector<InterestPoint *> &reference, const std::vector<InterestPoint *> &data, OrientedPoint2D& transformation,
					std::vector< std::pair<InterestPoint *, InterestPoint *> > &correspondences) const = 0;
					
		/** Sets the maximum distance (in meters) for a point to be considered in the inlier set. */
		inline void setAcceptanceThreshold(double acceptanceThreshold) 
			{m_acceptanceThreshold = acceptanceThreshold;}
		
		/** Gets the maximum distance (in meters) for a point to be considered in the inlier set. */
		inline double getAcceptanceThreshold() 
			{return m_acceptanceThreshold;}
		
		/** 
		 * Computes the Euclidean transformation given two point correspondences. 
		 * This is the minimum set of correspondence to obtain a solution in Cartesian space. 
		 * 
		 * @param correspondences The two correspondences. The individual correspondence is in the format (reference, data).
		 */
		virtual OrientedPoint2D generateHypothesis(const std::pair< std::pair<InterestPoint *, InterestPoint *>, std::pair<InterestPoint *, InterestPoint *> > &correspondences) const;
		
		/** Verifies a transformation hypothesis. It returns the matching error and the set of corresponding inlier points. 
		 * 
		 * @param reference The reference feature set. It is used as reference frame for the transformation.
		 * @param data The feature set to match. 
		 * @param transformation The transformation hypothesis. 
		 * @param inlierSet The set of corresponding inlier points. Each individual correspondence is in the format (reference, data).
		 */
		virtual double verifyHypothesis(const std::vector<InterestPoint *> &reference, const std::vector<InterestPoint *> &data, const OrientedPoint2D& transformation,
						std::vector< std::pair<InterestPoint *, InterestPoint *> > &inlierSet) const;
		
    protected:
		double m_acceptanceThreshold; /**< The maximum distance (in meters) for a point to be considered in the inlier set. */
};

#endif
