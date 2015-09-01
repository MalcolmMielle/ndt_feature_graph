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

#ifndef HISTOGRAMDISTANCES_H_
#define HISTOGRAMDISTANCES_H_

#include <vector>
#include <cmath>

/** 
 * Representation of an abstract distance function between histograms.
 * This class represents an abstract function to compute the distance between histograms.
 *
 */

template<class Numeric>
class HistogramDistance{
    public:
	/** Default destructor. */
	virtual ~HistogramDistance() { }

	/** Computes the distance between the first and last histogram (1D). */
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& last) const = 0;
	
	/** Computes the distance between the first and last histogram (2D). */
	virtual double distance(const std::vector< std::vector<Numeric> >& first, const std::vector< std::vector<Numeric> >& last) const;

	/** Computes the weighted distance between the first and last histogram (1D). */
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& weightFirst,
				const std::vector<Numeric>& last, const std::vector<Numeric>& weightLast) const = 0;

	/** Computes the weighted distance between the first and last histogram (2D). */
	virtual double distance(const std::vector< std::vector<Numeric> >& first, const std::vector< std::vector<Numeric> >& weightFirst, 
				const std::vector< std::vector<Numeric> >& last, const std::vector< std::vector<Numeric> >& weightLast) const;
};

/** 
 * Representation of the Euclidean distance function between histograms.
 * This class represents the function to compute the Euclidean distance between histograms.
 * The result is normalized to be in the range [0,1].
 *
 */

template<class Numeric>
class EuclideanDistance: public HistogramDistance<Numeric>{
    public:
	/** Default destructor. */
	virtual ~EuclideanDistance() { }
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& last) const;
	virtual double distance(const std::vector< std::vector<Numeric> >& first, const std::vector< std::vector<Numeric> >& last) const;
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& weightFirst,
				const std::vector<Numeric>& last, const std::vector<Numeric>& weightLast) const;
};

/** 
 * Representation of the \f$ \chi^2 \f$ distance function between histograms.
 * This class represents the function to compute the \f$ \chi^2 \f$ distance between histograms.
 *
 */

template<class Numeric>
class Chi2Distance: public HistogramDistance<Numeric>{
    public:
	/** Default destructor. */
	virtual ~Chi2Distance() { }
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& last) const;
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& weightFirst,
				const std::vector<Numeric>& last, const std::vector<Numeric>& weightLast) const;
};

/** 
 * Representation of the symmetric \f$ \chi^2 \f$ distance function between histograms.
 * This class represents the function to compute the symmetric \f$ \chi^2 \f$  distance between histograms.
 * The result is normalized to be in the range [0,1].
 *
 */

template<class Numeric>
class SymmetricChi2Distance: public HistogramDistance<Numeric>{
    public:
	/** Default destructor. */
	virtual ~SymmetricChi2Distance() { }
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& last) const;
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& weightFirst,
				const std::vector<Numeric>& last, const std::vector<Numeric>& weightLast) const;
};

/** 
 * Representation of the Batthacharyya distance function between histograms.
 * This class represents the function to compute the Batthacharyya distance between histograms.
 *
 */

template<class Numeric>
class BatthacharyyaDistance: public HistogramDistance<Numeric>{
    public:
	/** Default destructor. */
	virtual ~BatthacharyyaDistance() { }
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& last) const;
	virtual double distance(const std::vector< std::vector<Numeric> >& first, const std::vector< std::vector<Numeric> >& last) const;
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& weightFirst,
				const std::vector<Numeric>& last, const std::vector<Numeric>& weightLast) const;
};

/** 
 * Representation of the Kullback Leibler divergence between histograms.
 * This class represents the function to compute the Kullback Leibler divergence between histograms.
 *
 */

template<class Numeric>
class KullbackLeiblerDistance: public HistogramDistance<Numeric>{
    public:
	/** Default destructor. */
	virtual ~KullbackLeiblerDistance() { }
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& last) const;
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& weightFirst,
				const std::vector<Numeric>& last, const std::vector<Numeric>& weightLast) const;
};

/** 
 * Representation of the Jensen Shannon divergence between histograms.
 * This class represents the function to compute the Jensen Shannon divergence between histograms.
 * The result is normalized to be in the range [0,1].
 *
 */

template<class Numeric>
class JensenShannonDistance: public HistogramDistance<Numeric>{
    public:
	/** Default destructor. */
	virtual ~JensenShannonDistance() { }
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& last) const;
	virtual double distance(const std::vector<Numeric>& first, const std::vector<Numeric>& weightFirst,
				const std::vector<Numeric>& last, const std::vector<Numeric>& weightLast) const;
};
static EuclideanDistance<double> standardEuclideanDistance;

#include <utils/HistogramDistances.hpp>

#endif

