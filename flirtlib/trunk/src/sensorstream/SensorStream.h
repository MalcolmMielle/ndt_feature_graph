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

#ifndef SENSORSTREAM_H_
#define SENSORSTREAM_H_

#include <sensors/AbstractReading.h>

/** 
 * \enum SensorStreamOffset The offset from where to seek, values are END or BEGIN.
 *
 * @author Gian Diego Tipaldi
 */
 
enum SensorStreamOffset {
    END,
    BEGIN
};

/** 
 * Representation of an abstract stream of sensor readings.
 * The class represents an abstract sensor stream, providing the interface for obtaining sensor
 * readings, as well as seeking within the stream and checking if the stream is over.
 *
 * @author Gian Diego Tipaldi
 */

class SensorStream {
    public:
	/** Virtual Default destructor */
	virtual ~SensorStream() { }

	/** Get the next reading and advance the stream (const reading) */
	virtual const AbstractReading* next() const = 0;
	
	/** Get the current reading without advancing the stream (const reading) */
	virtual const AbstractReading* current() const = 0;

	/** Get the next reading and advance the stream (non const reading) */
	virtual AbstractReading* next() = 0;

	/** Get the current reading without advancing the stream (nonconst reading) */
	virtual AbstractReading* current() = 0;

	/** Get the n-th reading if the stream is seekable without advancing the the stream (const reading) */
	virtual const AbstractReading* operator[](unsigned int n) const = 0;

	/** Get the n-th reading if the stream is seekable without advancing the the stream (non const reading) */
	virtual AbstractReading* operator[](unsigned int n) = 0;
	
	/** Seek the stream to a given sensor position. Return false if is not possible */
	virtual bool seek(const unsigned int _position = 0, SensorStreamOffset _offset = BEGIN) = 0;
	
	/** Get the current sensor position of the stream. Return 0 if it is not seekable */
	virtual unsigned int tell() const = 0;
	
	/** Check if the stream is seekable */
	virtual inline bool isSeekable() const {return false;}
	
	/** Check if the stream is finished */
	virtual bool end() const = 0 ;
};

#endif

