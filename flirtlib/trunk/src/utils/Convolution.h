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

#ifndef CONVOLUTION_H_
#define CONVOLUTION_H_

#include <vector>
#include <cmath>
#include <boost/math/special_functions/bessel.hpp>
#include <iostream>

/**
 * Simple functions for performing convolution on discrete signals.
 *
 * @author Gian Diego Tipaldi
 */

/**
 * The padding strategy for the convolution operation.
 *
 */
enum ConvolutionPadding{ 
    ZERO, /**< The zero padding strategy. The original signal is augmented with zeros at the borders. */
    SPECULAR, /**< The specular padding strategy. The original signal is replicated in a specular way at the borders. */
    CIRCULAR /**< The circular padding strategy. It implements the circular convolution. */
};

/**
 * The size of the convolution result
 *
 */
enum ConvolutionResult{
    SAME, /**< The convolution result has the same size of the original signal. */
    FULL /**< The convolution result has the full size. */
};


/** 
 * Convolve the kernel over the source, if the size of the source is bigger than the size of the kernel, the opposite otherwise. 
 * The padding is defined according to padding and can be ZERO, SPECULAR or CIRCULAR. The size of the result is defined by resultType 
 * and can be FULL or SAME. The parameter offset define the offset of the kernel with respect to the source.
 *
 * @param source The signal to convolve.
 * @param kernel The convolution kernel. If the kernel is bigger than the source kernel and source are inverted.
 * @param offset The offset of the convolution. It is useful to shifting the signal to the left or right.
 * @param padding The type of padding. See the #ConvolutionPadding enum.
 * @param resultType The size of the convolution result. See the #ConvolutionResult enum.
 */

template<class Numeric>
std::vector<Numeric> convolve1D(const std::vector<Numeric>& source, const std::vector<Numeric>& kernel, int offset = 0, ConvolutionPadding padding = SPECULAR, ConvolutionResult resultType = SAME);

/** 
 * Compute the Bessel kernel for smoothing 
 *
 * @param sigma The standard deviation of the kernel.
 * @param kernelSize The size of the kernel.
 */

template<class Numeric>
std::vector<Numeric> besselKernel1D(Numeric sigma, unsigned int kernelSize);

/** 
 * Compute the Gaussian kernel for smoothing 
 *
 * @param sigma The standard deviation of the kernel.
 * @param kernelSize The size of the kernel.
 */

template<class Numeric>
std::vector<Numeric> gaussianKernel1D(Numeric sigma, unsigned int kernelSize);

/** Print the signal to a stream */
template<class Numeric>
std::ostream& operator<<(std::ostream& out, const std::vector<Numeric>& vector);

// real implementation
#include <utils/Convolution.hpp>

#endif

