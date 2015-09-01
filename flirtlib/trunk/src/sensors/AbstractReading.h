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

#ifndef ABSTRACTREADING_H_
#define ABSTRACTREADING_H_

#include <string>

/**
 * Representation of an abstract sensor measurement.
 * This class represent a general sensor reading in terms of timestamp, sensor name and robot name. Other sensor modality should inherit from this base class.
 *
 * @author Gian Diego Tipaldi
 */

class AbstractReading {
    public:
	/** 
	 * Constructor. This is an abstract class, so the constructor only creates the shared part of any specialized class.
	 * @param _time The timestamp of the reading.
	 * @param _name The name of the sensor.
	 * @param _robot The name of the robot.
	*/
	AbstractReading(double _time, const std::string& _name, const std::string& _robot);
	/** Destructor. */
	virtual ~AbstractReading() {};
	
	/** Clone function for prototyping. It implements the Prototype pattern. */
	virtual AbstractReading* clone() const = 0;

	/** Get the timestamp of the reading. */
	inline double getTime() const {return m_time;}
	/** Get the sensor name. */
	inline const std::string& getName() const {return m_name;}
	/** Get the robot name. */
	inline const std::string& getRobot() const {return m_robot;}
	

    protected:
	/** The timestamp of the reading (in seconds). */
	double m_time; // maybe using a better timing representation
	/** The name of the sensor */
	std::string m_name;
	/** The name of the robot (hostname of the machine) */
	std::string m_robot;
};

#endif

