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

#ifndef MULTISCALEDETECTOR_H_
#define MULTISCALEDETECTOR_H_

#include <feature/InterestPoint.h>
#include <feature/Detector.h>
#include <utils/Convolution.h>
#include <utils/PeakFinder.h>

#include <vector>

#define MIN_KERNEL_SIZE 3
#define MAX_KERNEL_SIZE 50


/** 
 * The smoothing kernel used in the detector.
 *
 */
enum SmoothingFilterFamily {
    GAUSSIAN, /**< The Gaussian smoothing kernel. It is the optimal kernel in the continuous case. */
    BESSEL /**< The Bessel smoothing kernel. It is the optimal kernel in the discrete case. */
};


/**
 * Representation of an abstract multi scale detector for monodimensional signals.
 * The class represents an abstract multi scale detector for monodimensional signals. It provides the interface to set 
 * the parameters of the detector and to detect interest points in laser readings. 
 *
 * @author Gian Diego Tipaldi
 */

class MultiScaleDetector: public Detector {
    public:
		/** 
		 * Constructor. Constructs and initialize the shared part of a general multi scale detector for monodimensional signals.
		 *
		 * @param peak The peak finder used to detect maxima in the signal.
		 * @param scales The number of different scales to consider.
		 * @param sigma The standard deviation of the smoothing kernel for the initial scale (\f$ t_0 \f$ in the paper). 
		 * @param step The scale increment at every new scale (\f$ t_i \f$ in the paper). The standard deviation of the kernel at scale \f$ s \f$ is \f$ t_0 \cdot (t_i)^s \f$
		 * @param filterType The smoothing kernel used in the detector.
		 */
		MultiScaleDetector(const PeakFinder* peak, unsigned int scales = 5, double sigma = 1.6, double step = 1.4, SmoothingFilterFamily filterType = BESSEL);

		/** Virtual Default destructor. */
		virtual ~MultiScaleDetector() { }

		virtual unsigned int detect(const LaserReading& reading, std::vector<InterestPoint*>& point) const;

		virtual unsigned int detect(const LaserReading& reading, std::vector<InterestPoint*>& point,
						std::vector< double >& signal,
						std::vector< std::vector<double> >& signalSmooth,
						std::vector< std::vector<double> >& signalDiff,
						std::vector< std::vector<unsigned int> >& indexes) const;
		
		/** 
		* Detects the interesting points given the laser reading as a 1D signal. It also returns the signals used for the computation.
		 *
		 * @return The number of interest points detected.
		 *
		 * @param signal The signal used for computing the interest points.
		 * @param signalSmooth The smoothed signal at the different scales.
		 * @param signalDiff The differential operator applied to the signal at the different scales.
		 * @param indexes The indexes of the differential operator maxima at the different scales.
		 */
		virtual void detect(const std::vector<double>& signal,
						std::vector< std::vector<double> >& signalSmooth,
						std::vector< std::vector<double> >& signalDiff,
						std::vector< std::vector<unsigned int> >& indexes) const;
		
		/** Sets the type of smoothing kernel*/
		inline void setFilterType(SmoothingFilterFamily filterType)
			{m_filterType = filterType; computeFilterBank(); computeDifferentialBank();}
		
		/** Sets the number of scales */
		inline void setScaleNumber(unsigned int scales)
			{m_scaleNumber = scales; computeFilterBank(); computeDifferentialBank();}
		
		/** Sets the standard deviation of the initial scale */
		inline void setBaseSigma(double sigma)
			{m_baseSigma = sigma; computeFilterBank(); computeDifferentialBank();}
			
		/** Sets the scale increment at each new scale */
		inline void setSigmaStep(double step) 
			{m_sigmaStep = step; computeFilterBank(); computeDifferentialBank();}
		
		/** Sets the peak finder */
		inline void setPeakFinder(const PeakFinder* peak)
			{m_peakFinder = peak;}
			
		/** Sets the "use max range" flag. The flag enable or disable the use of max range readings. */
		inline void setUseMaxRange(bool use)
			{m_useMaxRange = use;}
		
		/** Gets the type of smoothing kernel*/
		inline SmoothingFilterFamily getFilterType() const
			{return m_filterType;}
		
		/** Gets the number of scales */
		inline unsigned int getScaleNumber() const
			{return m_scaleNumber;}
		
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
		/** Precomputes the bank of smoothing operators. It is provided for efficiency issues. */
		virtual void computeFilterBank();
		
		/** Precomputes the bank of differential operators. It defines which family of interest point detector to use (blob, edge, ridge, ...).*/
		virtual void computeDifferentialBank() = 0;
		
		/** Computes the monodimensional signal from which extracts the features. */
		virtual void computeSignal(const LaserReading& reading, std::vector<double>& signal, std::vector<unsigned int>& maxRangeMapping) const = 0;
		
		/** Computes the interst point locations and (when possible) orientation. The points are extracted from the laser reading according to the indexes.*/
		virtual unsigned int computeInterestPoints(const LaserReading& reading, const std::vector<double>& signal, std::vector<InterestPoint*>& point, 
							std::vector< std::vector<unsigned int> >& indexes, std::vector<unsigned int>& maxRangeMapping) const = 0;
		
		const PeakFinder* m_peakFinder; /**< The peak finder used to detect maxima in the signal. */
		unsigned int m_scaleNumber; /**< The numebr of scales */
		double m_baseSigma; /**< The standard deviation of the smoothing kernel for the initial scale (\f$ t_0 \f$ in the paper). */
		double m_sigmaStep; /**< The scale increment at every new scale (\f$ t_i \f$ in the paper). The standard deviation of the kernel at scale \f$ s \f$ is \f$ t_0 \cdot (t_i)^s \f$ */
		bool m_useMaxRange; /**< The "use max range" flag. The flag enable or disable the use of max range readings. */
		SmoothingFilterFamily m_filterType;
		std::vector< std::vector<double> > m_filterBank; /**< The precomputed bank of smoothing operators. It is provided for efficiency issues. */
		std::vector< std::vector<double> > m_differentialBank; /**< The precomputed bank of differential operators. It defines which family of interest point detector to use (blob, edge, ridge, ...).*/
		std::vector<double> m_scales; /**< The precomputed scales for the smoothing operators. It is provided for efficiency issues. */
};

#endif

