//
//
// FLIRTLib - Fast Laser Interesting Region Transform Library
// Copyright (C) 2009-2010 Gian Diego Tipaldi and Kai O. Arras
//
// This file is part of FLIRTLib.
//
// FLIRTLib is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// FLIRTLib is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with FLIRTLib.  If not, see <http://www.gnu.org/licenses/>.
//

#include <limits>
#define PSEUDOZERO std::numeric_limits<double>::min()

#include <iostream>

template<class Numeric>
double HistogramDistance<Numeric>::distance(const std::vector< std::vector<Numeric> >& first, const std::vector< std::vector<Numeric> >& last) const{
    if (first.size() != last.size()) return 10e16;
    double accumulator = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	accumulator += distance(first[i], last[i]);
    }
    return accumulator;
}
template<class Numeric>
double HistogramDistance<Numeric>::distance(const std::vector< std::vector<Numeric> >& first, 
					    const std::vector< std::vector<Numeric> >& weightFirst, 
					    const std::vector< std::vector<Numeric> >& last, 
					    const std::vector< std::vector<Numeric> >& weightLast) const
{
    if (first.size() != last.size()) return 10e16;
    if (first.size() != weightFirst.size()) return 10e16;
    if (last.size() != weightLast.size()) return 10e16;
    double accumulator = 0.;
    double normalizer = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	accumulator += distance(first[i], weightFirst[i], last[i],  weightLast[i]);
    }
    return accumulator;
}

/// Euclidean
template<class Numeric>
double EuclideanDistance<Numeric>::distance(const std::vector< std::vector<Numeric> >& first, const std::vector< std::vector<Numeric> >& last) const{
    if (first.size() != last.size()) return 10e16;
    double accumulator = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	accumulator += distance(first[i], last[i]) * distance(first[i], last[i]);
    }
    return sqrt(accumulator);
}

template<class Numeric>
double EuclideanDistance<Numeric>::distance(const std::vector<Numeric>& first, const std::vector<Numeric>& weightFirst,
					    const std::vector<Numeric>& last, const std::vector<Numeric>& weightLast) const{
    if (first.size() != last.size()) return 10e16;
    if (first.size() != weightFirst.size()) return 10e16;
    if (last.size() != weightLast.size()) return 10e16;
    double accumulator = 0.;
    double normalizer = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	accumulator += (first[i] - last[i])*(first[i] - last[i])*(weightFirst[i]+weightLast[i]);
	normalizer += (weightFirst[i]+weightLast[i]);
    }
    return sqrt(accumulator/normalizer);
}

template<class Numeric>
double EuclideanDistance<Numeric>::distance(const std::vector<Numeric>& first, const std::vector<Numeric>& last) const{
    if (first.size() != last.size()) return 10e16;
    double accumulator = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	accumulator += (first[i] - last[i])*(first[i] - last[i]);
    }
    return sqrt(accumulator);
}

/// Chi2
template<class Numeric>
double Chi2Distance<Numeric>::distance(const std::vector<Numeric>& first, const std::vector<Numeric>& last) const{
    if (first.size() != last.size()) return 10e16;
    double accumulator = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	double p = last[i] == 0 ? PSEUDOZERO : last[i];
	double q = first[i] == 0 ? PSEUDOZERO : first[i];
	accumulator += (q - p)*(q - p)/q;
    }
    return accumulator;
}

template<class Numeric>
double Chi2Distance<Numeric>::distance(const std::vector<Numeric>& first, const std::vector<Numeric>& weightFirst,
				       const std::vector<Numeric>& last, const std::vector<Numeric>& weightLast) const
{
    if (first.size() != last.size()) return 10e16;
    if (first.size() != weightFirst.size()) return 10e16;
    if (last.size() != weightLast.size()) return 10e16;
    double accumulator = 0.;
    double normalizer = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	double p = last[i] == 0 ? PSEUDOZERO : last[i];
	double q = first[i] == 0 ? PSEUDOZERO : first[i];
	accumulator += ((q - p)*(q - p)/q)*(weightFirst[i]+weightLast[i]);
	normalizer += (weightFirst[i]+weightLast[i]);
    }
    return accumulator/normalizer;
}

/// Symmetric Chi2
template<class Numeric>
double SymmetricChi2Distance<Numeric>::distance(const std::vector<Numeric>& first, const std::vector<Numeric>& last) const{
    if (first.size() != last.size()) return 10e16;
    double accumulator = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	double p = last[i] == 0 ? PSEUDOZERO : last[i];
	double q = first[i] == 0 ? PSEUDOZERO : first[i];
	accumulator += (q - p)*(q - p)/(q + p);    
    }
    return 0.5 * accumulator; //TOCHECK Some books says to use 2 other 0.5
}

template<class Numeric>
double SymmetricChi2Distance<Numeric>::distance(const std::vector<Numeric>& first, const std::vector<Numeric>& weightFirst,
				       const std::vector<Numeric>& last, const std::vector<Numeric>& weightLast) const
{
    if (first.size() != last.size()) return 10e16;
    if (first.size() != weightFirst.size()) return 10e16;
    if (last.size() != weightLast.size()) return 10e16;    
    double accumulator = 0.;
    double normalizer = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	double p = last[i] == 0 ? PSEUDOZERO : last[i];
	double q = first[i] == 0 ? PSEUDOZERO : first[i];
	accumulator += ((q - p)*(q - p)/(q + p))*(weightFirst[i]+weightLast[i]);
	normalizer += (weightFirst[i]+weightLast[i]);
    }
    return accumulator/normalizer; 
}
///Batthacharyya
template<class Numeric>
double BatthacharyyaDistance<Numeric>::distance(const std::vector< std::vector<Numeric> >& first, const std::vector< std::vector<Numeric> >& last) const{
    if (first.size() != last.size()) return 10e16;
    double accumulator = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	double current = distance(first[i], last[i]);
	accumulator += 1. - current * current;
    }
    return sqrt(1. - accumulator);
}

template<class Numeric>
double BatthacharyyaDistance<Numeric>::distance(const std::vector<Numeric>& first, const std::vector<Numeric>& last) const{
    if (first.size() != last.size()) return 10e16;
    double accumulator = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	double p = last[i];// <= 0 ? PSEUDOZERO : last[i];
	double q = first[i];// <= 0 ? PSEUDOZERO : first[i];
	accumulator += sqrt(q * p);
    }
    return sqrt(1. - accumulator);
}

template<class Numeric>
double BatthacharyyaDistance<Numeric>::distance(const std::vector<Numeric>& first, const std::vector<Numeric>& weightFirst,
				       const std::vector<Numeric>& last, const std::vector<Numeric>& weightLast) const
{
    if (first.size() != last.size()) return 10e16;
    if (first.size() != weightFirst.size()) return 10e16;
    if (last.size() != weightLast.size()) return 10e16;    
    double accumulator = 0.;
    double normalizer = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	double p = last[i];// <= 0 ? PSEUDOZERO : last[i];
	double q = first[i];// <= 0 ? PSEUDOZERO : first[i];
	accumulator += sqrt(q * p)*(weightFirst[i]+weightLast[i]);
	normalizer += (weightFirst[i]+weightLast[i]);
    }
    return sqrt(1. - accumulator/normalizer);
}

///KullbackLeibler
template<class Numeric>
double KullbackLeiblerDistance<Numeric>::distance(const std::vector<Numeric>& first, const std::vector<Numeric>& last) const{
    if (first.size() != last.size()) return 10e16;
    double accumulator = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	double p = last[i] <= 0 ? PSEUDOZERO : last[i];;
	double q = first[i] <= 0 ? PSEUDOZERO : first[i];
	accumulator += p * log (p/q);
    }
    return accumulator;
}

template<class Numeric>
double KullbackLeiblerDistance<Numeric>::distance(const std::vector<Numeric>& first, const std::vector<Numeric>& weightFirst,
				       const std::vector<Numeric>& last, const std::vector<Numeric>& weightLast) const
{
    if (first.size() != last.size()) return 10e16;
    if (first.size() != weightFirst.size()) return 10e16;
    if (last.size() != weightLast.size()) return 10e16;    
    double accumulator = 0.;
    double normalizer = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	double p = last[i] <= 0 ? PSEUDOZERO : last[i];;
	double q = first[i] <= 0 ? PSEUDOZERO : first[i];
	accumulator += p * log (p/q)*(weightFirst[i]+weightLast[i]);
	normalizer += (weightFirst[i]+weightLast[i]);
    }
    return accumulator/normalizer;
}
///JensenShannon
template<class Numeric>
double JensenShannonDistance<Numeric>::distance(const std::vector<Numeric>& first, const std::vector<Numeric>& last) const{
    if (first.size() != last.size()) return 10e16;
    double accumulator = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	double p = last[i] <= 0 ? PSEUDOZERO : last[i];
	double q = first[i] <= 0 ? PSEUDOZERO : first[i];
	accumulator += p * log (2*p/(p + q)) + q * log (2*q/(p + q));
    }
    return 0.5 * accumulator / log(2);
}
template<class Numeric>
double JensenShannonDistance<Numeric>::distance(const std::vector<Numeric>& first, const std::vector<Numeric>& weightFirst,
				       const std::vector<Numeric>& last, const std::vector<Numeric>& weightLast) const
{
    if (first.size() != last.size()) return 10e16;
    if (first.size() != weightFirst.size()) return 10e16;
    if (last.size() != weightLast.size()) return 10e16;    
    double accumulator = 0.;
    double normalizer = 0.;
    for (unsigned int i = 0; i < first.size(); i++){
	double p = last[i] <= 0 ? PSEUDOZERO : last[i];
	double q = first[i] <= 0 ? PSEUDOZERO : first[i];
	accumulator += (p * log (2*p/(p + q)) + q * log (2*q/(p + q)))*(weightFirst[i]+weightLast[i]);
	normalizer += (weightFirst[i]+weightLast[i]);
    }
    return 0.5 * accumulator /normalizer / log(2);
}
