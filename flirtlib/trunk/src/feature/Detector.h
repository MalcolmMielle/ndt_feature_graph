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

#ifndef DETECTOR_H_
#define DETECTOR_H_

#include <feature/InterestPoint.h>
#include <sensors/LaserReading.h>

/**
 * Representation of an abstract detector.
 * This class represents an abstract detector, defining the interface for detecting interest points in laser readings.
 *
 * @author Gian Diego Tipaldi
 */


class Detector {
    public:
		/** Default destructor. */
		virtual ~Detector() { }

		/** 
		 * Detects the interesting points given the laser reading. 
		 *
		 * @return The number of interest points detected.
		 *
		 * @param reading The laser reading.
		 * @param points The detected interest points.
		 */
		virtual unsigned int detect(const LaserReading& reading, std::vector<InterestPoint*>& points) const = 0;
		
		/** 
		 * Detects the interesting points given the laser reading. It also returns the signals used for the computation.
		 *
		 * @return The number of interest points detected.
		 *
		 * @param reading The laser reading.
		 * @param points The detected interest points.
		 * @param signal The signal used for computing the interest points.
		 * @param signalSmooth The smoothed signal at the different scales.
		 * @param signalDiff The differential operator applied to the signal at the different scales.
		 * @param indexes The indexes of the differential operator maxima at the different scales.
		 */
		virtual unsigned int detect(const LaserReading& reading, std::vector<InterestPoint*>& points,
						std::vector< double >& signal,
						std::vector< std::vector<double> >& signalSmooth,
						std::vector< std::vector<double> >& signalDiff,
						std::vector< std::vector<unsigned int> >& indexes) const = 0;

};

#endif

