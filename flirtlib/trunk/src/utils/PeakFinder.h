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

#ifndef PEAKFINDER_H_
#define PEAKFINDER_H_

#include <vector>

/**
 * Representation of an abstract algorithm for peak finding.
 * The class represents an abstract algorithm for finding peaks in discrete signals.
 *
 * @author Gian Diego Tipaldi
 */

class PeakFinder{
    public:
	/** Default destructor. */
	virtual ~PeakFinder() { }

	/** Finds the indexes of the peaks in a onedimensional signal. */
	virtual void findPeaks(const std::vector<double>& signal, std::vector<unsigned int>& indexes) const = 0;
	
	/** Finds the indexes of the peaks in a bidimensional signal. */
	virtual void findPeaks(const std::vector< std::vector<double> >& signal, std::vector< std::vector<unsigned int> >& indexes) const = 0;
	
	/** Checks if the given index represents a peak in the monodimensional signal */
	virtual bool isPeak(const std::vector<double>& signal, unsigned int index) const = 0;
	
	/** Checks if the given indexes represent a peak in the bidimensional signal */
	virtual bool isPeak(const std::vector< std::vector<double> >& signal, unsigned int index1, unsigned int index2) const = 0;
};

#endif

