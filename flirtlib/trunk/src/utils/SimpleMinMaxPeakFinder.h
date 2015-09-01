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

#ifndef SIMPLEMINMAXPEAKFINDER_H_
#define SIMPLEMINMAXPEAKFINDER_H_

#include <utils/SimplePeakFinder.h>
#include <vector>

/**
 * Representation of a simple algorithm for peak finding.
 * The class represents a simple algorithm for finding peaks in discrete signals.
 * The algorithm finds both positive and negative peaks.
 *
 * @author Gian Diego Tipaldi
 */

class SimpleMinMaxPeakFinder: public SimplePeakFinder{
    public:
	/**
	 * Constructor. Creates and initializes the peak finder.
	 *
	 * @param minValue The minimum value for a peak to be considerated valid. The negative value is used for the negative peak. 
	 * @param minDifference The minimum difference a peak should have with respect to its immediate neighbours. The negative value is used for the negative peak.
	 */
	SimpleMinMaxPeakFinder(double minValue = 0.0, double minDifference = 0.0);
	
	/** Default destructor. */
	virtual ~SimpleMinMaxPeakFinder() { }

	virtual bool isPeak(const std::vector<double>& signal, unsigned int index) const;
};

#endif

