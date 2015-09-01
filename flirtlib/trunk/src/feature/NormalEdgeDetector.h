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

#ifndef NORMALEDGEDETECTOR_H_
#define NORMALEDGEDETECTOR_H_

#include <feature/NormalDetector.h>

/**
 * Representation of the normal edge detector.
 * The class represents the normal edge detector defined in the paper. It extracts edges (extrema of the first derivative) on the normal signal.
 *
 * @author Gian Diego Tipaldi
 */

class NormalEdgeDetector: public NormalDetector {
    public:
		/** 
		 * Constructor. Constructs and initialize the normal edge detector. 
		 *
		 * @param peak The peak finder used to detect maxima in the signal.
		 * @param scales The number of different scales to consider.
		 * @param sigma The standard deviation of the smoothing kernel for the initial scale (\f$ t_0 \f$ in the paper). 
		 * @param step The scale increment at every new scale (\f$ t_i \f$ in the paper). The standard deviation of the kernel at scale \f$ s \f$ is \f$ t_0 \cdot (t_i)^s \f$
		 * @param window The window size for the local line fitting.
		 * @param filterType The smoothing kernel used in the detector.
		 */
		NormalEdgeDetector(const PeakFinder* peak, unsigned int scales = 5, double sigma = 1.6, double step = 1.4, unsigned int window = 3, SmoothingFilterFamily filterType = BESSEL);
	
	/** Virtual Default destructor. */
	virtual ~NormalEdgeDetector() { }

    protected:
		/** Computes the bank for the first derivative at different scales. */
		virtual void computeDifferentialBank();
};

#endif

