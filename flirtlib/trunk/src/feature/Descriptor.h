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

#ifndef DESCRIPTOR_H_
#define DESCRIPTOR_H_

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/export.hpp>

class InterestPoint;
class LaserReading;
class OrientedPoint2D;

/**
 * Representation of an abstract descriptor.
 * The class represents an abstract descriptor, defining the interface for comparing two descriptors.
 *
 * @author Gian Diego Tipaldi
 */

class Descriptor {
	public:
		/** Clone function for prototyping. It implements the Prototype pattern. */
		virtual Descriptor* clone() const = 0;

		/** Default destructor. */
		virtual ~Descriptor() { }
		
		/** 
		 * Abstract interface for computing the distance between two descriptors. 
		 * The implementation of the actual distance is left to the inherited classes. 
		 * 
		 */
		virtual double distance(const Descriptor* descriptor) const = 0;
	
		/** 
		 * Returns the descriptor in the form of onedimensional histogram. The resulting vector represents the feature descriptor.
		 *
		 */
		virtual void getFlatDescription(std::vector<double>& description) const = 0;
	
		/** 
		 * Returns the descriptor in the form of a weighted onedimensional histogram. The resulting vectors represent the feature descriptor and the importance of each dimension.
		 * @param description the descriptor vector
		 * @param weight the descriptor variance
		 *
		 */
		virtual void getWeightedFlatDescription(std::vector<double>& description, std::vector<double>& weight) const
	    {getFlatDescription(description); weight.resize(description.size(),1.);}
   
    protected:
		friend class boost::serialization::access;
	
		/** Serializes the class using boost::serialization. */ 
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version);
};

template<class Archive>
void Descriptor::serialize(Archive& ar, const unsigned int version)
{
    BOOST_SERIALIZATION_ASSUME_ABSTRACT(Descriptor);
}

#if BOOST_VERSION > 104000
BOOST_CLASS_EXPORT_KEY(Descriptor);
#endif

/**
 * Representation of an abstract descriptor generator.
 * The class represents an abstract descriptor generator, defining the interface for generating the description of an interest point.
 *
 * @author Gian Diego Tipaldi
 */

class DescriptorGenerator{
    public:

		/** Default destructor. */
		virtual ~DescriptorGenerator() { }

		/** Abstract interface for generating a descriptors given an interest point and a laser reading. */
		virtual Descriptor* describe(const InterestPoint& _point, const LaserReading& reading) = 0;
		
		/** Abstract interface for generating a descriptors given a general point in \f$ \mathcal{SO}(2) \f$  and a laser reading. */
		virtual Descriptor* describe(const OrientedPoint2D& _point, const LaserReading& reading) = 0;
};

#endif

