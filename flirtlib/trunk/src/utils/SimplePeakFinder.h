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

#ifndef SIMPLEPEAKFINDER_H_
#define SIMPLEPEAKFINDER_H_

#include <utils/PeakFinder.h>
#include <vector>

/**
 * Representation of a simple algorithm for peak finding.
 * The class represents a simple algorithm for finding peaks in discrete signals.
 *
 * @author Gian Diego Tipaldi
 */

class SimplePeakFinder: public PeakFinder{
    public:
	/**
	 * Constructor. Creates and initializes the peak finder.
	 *
	 * @param minValue The minimum value for a peak to be considerated valid. 
	 * @param minDifference The minimum difference a peak should have with respect to its immediate neighbours.
	 */
	SimplePeakFinder(double minValue = 0.0, double minDifference = 0.0);

	/** Default destructor. */
	virtual ~SimplePeakFinder() { } 
	
	virtual void findPeaks(const std::vector<double>& signal, std::vector<unsigned int>& indexes) const;
	
	virtual void findPeaks(const std::vector< std::vector<double> >& signal, std::vector< std::vector<unsigned int> >& indexes) const;
	
	virtual bool isPeak(const std::vector<double>& signal, unsigned int index) const;
	
	virtual bool isPeak(const std::vector< std::vector<double> >& signal, unsigned int index1, unsigned int index2) const;
	
	/** Gets the minimum value for a peak to be considerated valid. */
	inline double getMinValue() const
	    {return m_minValue;}
	
	/** Gets the minimum difference a peak should have with respect to its immediate neighbours. */
	inline double getMinDifference() const
	    {return m_minDifference;}
	
	/** Sets the minimum value for a peak to be considerated valid. */
	inline void setMinValue(double value)
	    {m_minValue = value;}
	
	/** Sets the minimum difference a peak should have with respect to its immediate neighbours. */
	inline void setMinDifference(double value)
	    {m_minDifference = value;}
    protected:
	double m_minValue; /**< The minimum value for a peak to be considerated valid.  */
	double m_minDifference; /**< The minimum difference a peak should have with respect to its immediate neighbours. */
};

#endif

