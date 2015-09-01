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

#ifndef INTERESTPOINT_H_
#define INTERESTPOINT_H_

#include <geometry/point.h>
#include <feature/Descriptor.h>
#include <vector>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>

/**
 * Representation of an interesting point.
 * The class represents an interesting point. It defines the interface for obtaining the position of the interest point
 * and its descriptor. 
 *
 * @author Gian Diego Tipaldi
 */

class InterestPoint {
    public:
	/** 
	 * Constructor. It creates the interest point providing the position, scale and descriptor.
	 *
	 * @param position The position of the interest point as a point in \f$ \mathcal{SO}(2) \f$.
	 * @param scale The scale at which the interest point was detected.
	 * @param descriptor The descriptor of the interest point. The descriptor will be internally cloned.
	 */
	InterestPoint(const OrientedPoint2D& position = OrientedPoint2D(0.,0.,0.), double scale = 1., const Descriptor* descriptor = 0);
	
	/** Copy Constructor. */
	InterestPoint(const InterestPoint& point);
	
	/** Assignament operator. */
	InterestPoint& operator=(const InterestPoint& _point);
	
	/** Destructor. */
	virtual ~InterestPoint();
    
	/** Gets the position and orientation of the interest point. */
	inline const OrientedPoint2D& getPosition() const 
	    {return m_position;}

	/** Gets the scale at which the interest point was detected. */
	inline double getScale() const
	    {return m_scale;}
	
	/** Gets the index of the scale at which the interest point is detected. It is used for plotting purposes. */
	inline double getScaleLevel() const
	    {return m_scaleLevel;}
	
	/** Gets the descriptor of the interest point. */
	inline const Descriptor* getDescriptor() const
	    {return m_descriptor;}

	/** Get the descriptor of the interest point (non const version). */
	inline Descriptor* getDescriptor() 
	    {return m_descriptor;}
	
	/** Gets the points in the support region of the interest point. See the paper for more details. */
	inline const std::vector<Point2D>& getSupport() const
	    {return m_supportPoints;}
	    
	/** Sets the position and orientation of the interest point */
	inline void setPosition(const OrientedPoint2D& _position)
	    {m_position = _position;}
	
	/** Sets the scale at which the interest point was detected. */
	inline void setScale(double _scale)
	    {m_scale = _scale;}
	
	/** Sets the index of the scale at which the interest point is detected. It is used for plotting purposes. */
	inline void setScaleLevel(unsigned int _scale)
	    {m_scaleLevel = _scale;}
	
	/** Sets the descriptor of the interest point. The descriptor will be internally cloned. */
	inline void setDescriptor(const Descriptor* _descriptor)
	    {if(m_descriptor) delete m_descriptor; if(_descriptor) m_descriptor = _descriptor->clone(); else m_descriptor = NULL;} 
	
	/** Sets the points in the support region of the interest point. See the paper for more details. */
	inline void setSupport(const std::vector<Point2D>& points)
	    {m_supportPoints = points;}
	    
    protected:
		friend class boost::serialization::access;

		/** Serializes the class using boost::serialization. */ 
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version);

	OrientedPoint2D m_position; /**< The position of the interest point as a point in \f$ \mathcal{SO}(2) \f$. */
	std::vector<Point2D> m_supportPoints; /**< The points in the support region of the interest point. See the paper for more details. */
	double m_scale; /**< The scale at which the interest point was detected. */
	unsigned int m_scaleLevel; /**< The index of the scale at which the interest point is detected. It is used for plotting purposes. */
	Descriptor* m_descriptor; /**< The descriptor of the interest point. */
};

template<class Archive>
void InterestPoint::serialize(Archive& ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(m_position);
    ar & BOOST_SERIALIZATION_NVP(m_supportPoints);
    ar & BOOST_SERIALIZATION_NVP(m_scale);
    ar & BOOST_SERIALIZATION_NVP(m_scaleLevel);
    ar & BOOST_SERIALIZATION_NVP(m_descriptor);
}

#endif
