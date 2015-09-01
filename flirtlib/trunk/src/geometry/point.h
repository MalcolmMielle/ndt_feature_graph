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

#ifndef POINT_H_
#define POINT_H_

#include <cmath>
#include <iostream>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
/**
 * Representation of a point in \f$ \mathbb{R}^2 \f$ .
 * The class represents 2D points in terms of their position (#x, #y) in in \f$ \mathbb{R}^2 \f$ .
 *
 * @author Gian Diego Tipaldi
 */

struct Point2D {
    /** Default constructor. It sets the point to be in the origin (0,0) */
    Point2D(); 
    /** Constructor. It sets the point to be in (_x, _y) */
    Point2D(double _x, double _y); 
    
    /** Returns the orthogonal vector (rotation of 90 deg) */
    inline Point2D ortho()
	{return Point2D(y, -x);}

    double x; /**< The x coordinate.  */
    double y; /**< The y coordinate. */

	/** Serializes the class using boost::serialization. */ 
    template<class Archive>  
    void serialize(Archive & ar, const unsigned int version);
    
    virtual ~Point2D() { }
};
template<class Archive>
void Point2D::serialize(Archive& ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(x);
    ar & BOOST_SERIALIZATION_NVP(y);
}

/**
 * Representation of a point in \f$ \mathcal{SO}(2) \f$ .
 * The class represents points in \f$ \mathcal{SO}(2) \f$ in terms of their position (#x, #y) and their orientation #theta.
 *
 * @author Gian Diego Tipaldi
 */

struct OrientedPoint2D: public Point2D {
    /** Default constructor. It sets the point to be in the origin (0,0,0) */
    OrientedPoint2D(); 
    /** Constructor. It sets the point to be in (_x, _y, _theta) */
    OrientedPoint2D(double _x, double _y, double _theta); 
    
    virtual ~OrientedPoint2D() { }
    /** Inverse compound operator \f$ (\ominus this) \f$*/
    OrientedPoint2D ominus() const;
    
    /** Inverse compound operator \f$ (\ominus this) \oplus point \f$ */
    OrientedPoint2D ominus(const OrientedPoint2D& point) const;
    
    /** Inverse compound operator \f$ (\ominus this) \oplus point \f$ */
    Point2D ominus(const Point2D& point) const;
    
    /** Compound operator \f$ this \oplus point \f$ */
    OrientedPoint2D oplus(const OrientedPoint2D& point) const;
    
    /** Compound operator \f$ this \oplus point \f$ */
    Point2D oplus(const Point2D& point) const;
    
    double theta; /**< The orientation of the point. */
    
	/** Serializes the class using boost::serialization. */ 
	template<class Archive>  
    void serialize(Archive & ar, const unsigned int version);
};

template<class Archive>
void OrientedPoint2D::serialize(Archive& ar, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Point2D);
    ar & BOOST_SERIALIZATION_NVP(theta);
}

/** Simple function for convrting degrees into radians */
inline double deg2rad(double _angle) 
    {return _angle/180.0*M_PI;}

/** Simple function for converting radians into degrees */
inline double rad2deg(double _angle) 
    {return _angle/M_PI*180.0;}

/** Simple function for normalizing the angle into the interval \f$(base, 2\pi + base)\f$  (default base = \f$ -\pi \f$)*/
double normAngle(double angle, double base = -M_PI); 

/** Compute the difference between two points */
inline Point2D operator-(const Point2D& first, const Point2D& last)
    {return Point2D(first.x - last.x, first.y - last.y);}

/** Compute the sum between two points */
inline Point2D operator+(const Point2D& first, const Point2D& last)
    {return Point2D(first.x + last.x, first.y + last.y);}
    
/** Compute the analitical difference between two oriented points */
inline OrientedPoint2D operator-(const OrientedPoint2D& first, const OrientedPoint2D& last)
    {return OrientedPoint2D(first.x - last.x, first.y - last.y, normAngle(first.theta - last.theta, -M_PI));}

/** Compute the analitical sum between two oriented points */
inline OrientedPoint2D operator+(const OrientedPoint2D& first, const OrientedPoint2D& last)
    {return OrientedPoint2D(first.x + last.x, first.y + last.y, normAngle(first.theta + last.theta, -M_PI));}
    
/** Compute the product of a point with a scalar */
inline Point2D operator*(const Point2D& first, const double last)
    {return Point2D(first.x * last, first.y * last);}

/** Compute the product of a point with a scalar */
inline Point2D operator*(const double last, const Point2D& first)
    {return Point2D(first.x * last, first.y * last);}
    
/** Compute the dot product of two points*/
inline double operator*(const Point2D& first, const Point2D& last)
    {return first.x * last.x + first.y * last.y;}

/** Print the Point2D */
inline std::ostream& operator << (std::ostream& out, const Point2D& point)
    {out << "(" << point.x << ", " << point.y << ")"; return out;}

/** Print the OrientedPoint2D */
inline std::ostream& operator << (std::ostream& out, const OrientedPoint2D& point)
    {out << "(" << point.x << ", " << point.y << ", " << point.theta << ")"; return out;}

#endif

