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

#ifndef LASERREADING_H_
#define LASERREADING_H_

#include <sensors/AbstractReading.h>
#include <geometry/point.h>
#include <string>
#include <vector>
#include <cmath>

/**
 * Representation of a laser measurement.
 * This class represent a general laser reading in terms of both polar and Cartesian coordinates.
 *
 * @author Gian Diego Tipaldi
 */
 
class LaserReading: public AbstractReading {
     public:
	/** 
	 * Constructor. It creates the object by giving the reading in polar coordinates (_phi, _rho).
	 * @param _phi The angles where the reading is taken.
	 * @param _rho The distances measured by the laser.
	 * @param _time The timestamp of the reading.
	 * @param _name The name of the sensor.
	 * @param _robot The name of the robot.
	 */
	LaserReading(const std::vector<double>& _phi, const std::vector<double>& _rho,
		     double _time = 0.0, const std::string& _name = "ROBOTLASER1", const std::string& _robot = "");
	
	/** Destructor. It destructs the object. */
	virtual ~LaserReading();
	
	/** Clone function for prototyping. It implements the Prototype pattern. */
	virtual AbstractReading* clone() const;


	// Getter Methods
	/** Get the angles. It returns the angles where the reading is taken. */
	inline const std::vector<double>& getPhi() const 
	    {return m_phi;} 
	/** Get the distances. It returns the distances measured by the laser*/
	inline const std::vector<double>& getRho() const 
	    {return m_rho;}
	/** Get angles and distances. It returns both angles, _phi, and distances, _rho, of the reading.*/
	inline unsigned int getPolar(const std::vector<double>*& _phi, const std::vector<double>*&  _rho) const 
	    {_phi = &m_phi; _rho = &m_rho; return m_phi.size();}
	/** 
	 * Get local Cartesian coordinates. It returns the points measured by the laser in Cartesian coordinates.
	 * The points are considered in the sensor reference frame. 
	 */
	inline const std::vector<Point2D>& getCartesian() const
	    {return m_cartesian;}
	/** 
	 * Get global Cartesian coordinates. It returns the points measured by the laser in Cartesian coordinates.
	 * The points are transormed in the world reference frame by using #m_laserPose. 
	 */
	inline const std::vector<Point2D>& getWorldCartesian() const
	    {return m_worldCartesian;}
	/** 
	 * Get local Cartesian coordinates. It returns the points measured by the laser in Cartesian coordinates.
	 * The points are considered in the sensor reference frame and max range readings are omitted. 
	 */
	inline const std::vector<Point2D>& getCartesianNoMax() const
	    {return m_cartesianNoMax;}
	/** 
	 * Get global Cartesian coordinates. It returns the points measured by the laser in Cartesian coordinates.
	 * The points are transormed in the world reference frame by using #m_laserPose and max range readings are omitted. 
	 */
	inline const std::vector<Point2D>& getWorldCartesianNoMax() const
	    {return m_worldCartesianNoMax;}
	/// Get reminiscence values 
	/** Get remission values. It returns the remission values of the reading, if present. */	inline const std::vector<double>& getRemission() const
	    {return m_remission;}
	/** Get maximum range. It returns the maximum range of the sensor. */
	inline double getMaxRange() const 
	    {return m_maxRange;}
	/** Get laser pose. It returns the pose of the laser in the world. */
	inline const OrientedPoint2D& getLaserPose() const 
	    {return m_laserPose;}
	
	/** Get robot pose. It returns the pose of the robot in the world. */
	inline const OrientedPoint2D& getRobotPose() const 
	    {return m_robotPose;}

	// Setter Methods
	/** 
	 * Set reminiscence values. It sets the reminiscence values of the reading. It fails if the size of _remi 
	 * is different than the size of #m_phi . 
	 */
	void setRemission(const std::vector<double>& _remi);
	/** Set maximum range. It sets the maximum range of the sensor. */
	inline void setMaxRange(double _max)
	    {m_maxRange = _max; computeWorldCartesian(); computeLocalCartesian();}
	/** Set laser pose. It sets the pose of the laser in the world. */
	void setLaserPose(const OrientedPoint2D& _pose); 
	/** Set robot pose. It sets the pose of the robot in the world. */
	void setRobotPose(const OrientedPoint2D& _pose); 
	/// Set phi values (polar coordinates)
	inline void setPhi(const std::vector<double>& phi)
	    {m_phi = phi; computeWorldCartesian(); computeLocalCartesian();}
	/// Set rho values (polar coordinates)
	inline void setRho(const std::vector<double>& rho)
	    {m_rho = rho; computeWorldCartesian(); computeLocalCartesian();}

    protected:
	/** Compute world points. It transforms the polar reading into a cartesian one, considering the pose of the laser. */
	void computeWorldCartesian();
	/** 
	 * Compute world points. It transforms the polar reading into a cartesian one, considering the sensor 
	 * being in (0,0,0). 
	 */
	void computeLocalCartesian();
	/** The maximum range of the sensor. */
	double m_maxRange;
	/** The pose of the laser. */
	OrientedPoint2D m_laserPose;
	/** The pose of the robot. */
	OrientedPoint2D m_robotPose;
	/** The angles of the reading. */
	std::vector<double> m_phi;
	/** The distances measured. */
	std::vector<double> m_rho;
	/** The points in the world reference frame. */
	std::vector<Point2D> m_worldCartesian;
	/** The points in the sensor reference frame. */
	std::vector<Point2D> m_cartesian;
	/** The points in the world reference frame, without max range readings. */
	std::vector<Point2D> m_worldCartesianNoMax;
	/** The points in the sensor reference frame, without max range readings. */
	std::vector<Point2D> m_cartesianNoMax;
	/** The reminiscence measured. */
	std::vector<double> m_remission;
};

#endif 

