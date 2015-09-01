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


template<class Numeric>
std::ostream& operator<<(std::ostream& out, const std::vector<Numeric>& vector){
    for(uint i = 0; i < vector.size(); i++){
	out << vector[i] << " "; 
    }
    return out;
}

template<class Numeric>
std::vector<Numeric> convolve1D(const std::vector<Numeric>& _source, const std::vector<Numeric>& _kernel, int _offset, ConvolutionPadding _padding, ConvolutionResult _resultType){
    _resultType = (_padding == CIRCULAR) ? SAME : _resultType;
    bool inverted = (_source.size() < _kernel.size());
    const std::vector<Numeric>& source = (inverted) ? _kernel : _source;
    const std::vector<Numeric>& kernel = (inverted) ? _source : _kernel;
    unsigned int size  = _source.size() + (_resultType == FULL) * (_kernel.size() - 1);
    std::vector<Numeric> result(size);
    
    unsigned int i = 0;
    for( ; i < result.size(); i++){
	result[i] = 0;
	
	int j = (int)i - (int)kernel.size() + 1 ;
	for(; j <= (int)i && j < _offset; j++){
	    Numeric pad = 0;
	    switch(_padding) {
		case ZERO:
		    pad = 0; break;
		case SPECULAR:
		    pad = source[_offset - j]; break;
		case CIRCULAR:
		    pad = source[source.size() + j -_offset]; break;
	    }
	    result[i] += pad * kernel[i - j];
	}
	
	for( ; j <= (int)i && j < (int)source.size() + _offset; j++){
	    result[i] += source[j - _offset] * kernel[i - j];
	}
	
	for( ; j <= (int)i; j++){
	    Numeric pad = 0;
	    switch (_padding){
		case ZERO:
		    pad = 0; break;
		case SPECULAR:
		    pad = source[2*source.size() - 1 - j + _offset]; break;
		case CIRCULAR:
		    pad = source[j - _offset -source.size()]; break;
	    }
	    result[i] += pad * kernel[i - j];
	}
    }
/*    for( ; i < result.size(); i++){
	result[i] = 0;
	
	int j = (int)i - (int)kernel.size() + 1;
	for(; j <= (int)i; j++){
	    Numeric pad = 0;
	    switch (_padding){
		case ZERO:
		    pad = 0; break;
		case SPECULAR:
		    pad = source[source.size() - j]; break;
		case CIRCULAR:
		    pad = source[j]; break;
	    }
	    result[i] += pad * kernel[i - j];
	}
    }*/
    
    return result;
}

template<class Numeric>
std::vector<Numeric> besselKernel1D(Numeric _sigma, unsigned int _kernelSize){
    unsigned int size = (_kernelSize % 2) ? _kernelSize : _kernelSize + 1;
    std::vector<Numeric> result(size);
    Numeric accumulator = 0;
    
    for(unsigned int i = 0; i < size; i++){
	result[i] = exp(_sigma) * boost::math::cyl_bessel_i(i - 0.5*(size-1),_sigma);
	accumulator += result[i];
    }
    
    for(unsigned int i = 0; i < size; i++){
	result[i] = result[i] / accumulator;
    }
    
    return result;
}

template<class Numeric>
std::vector<Numeric> gaussianKernel1D(Numeric sigma, unsigned int _kernelSize){
    unsigned int size = (_kernelSize % 2) ? _kernelSize : _kernelSize + 1;
    std::vector<Numeric> result(size);
    Numeric accumulator = 0;
    if(sigma < 1e-10) sigma = 1e-6;
    
    for(unsigned int i = 0; i < size; i++){
	Numeric x = i - 0.5*(size-1);
	result[i] = exp(-x*x/(2*sigma * sigma));
	accumulator += result[i];
    }
    
    for(unsigned int i = 0; i < size; i++){
	result[i] = result[i] / accumulator;
    }
    
    return result;
}
