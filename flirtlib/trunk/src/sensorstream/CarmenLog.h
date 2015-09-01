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

#ifndef CARMENLOG_H_
#define CARMENLOG_H_

#include <sensorstream/LogReader.h>
#include <sensorstream/LogWriter.h>
#include <sensors/AbstractReading.h>
#include <sensors/LaserReading.h>
#include <geometry/point.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

/** 
 * Representation of a CARMEN log reader. It implements the LogReader class for the CARMEN log file format.
 *
 * @author Gian Diego Tipaldi
 */

class CarmenLogReader: public LogReader{
    public:
	/** Virtual Default destructor. */
	virtual ~CarmenLogReader() { }
	/** Read a log from an inputstream. */
	virtual void readLog(std::istream& _stream, std::vector<AbstractReading*>& _log) const;
	/** Read a single line from the stream. */
	virtual AbstractReading* readLine(std::istream& _stream) const;

    protected:
	/** Parse the old laser structure (FLASER). */
	LaserReading* parseFLaser(std::istream& _stream) const;
	/** Parse the new laser structure (ROBOTLASERX). */
	LaserReading* parseRobotLaser(std::istream& _stream) const;
	/** Parse the raw laser structure (RAWLASERX). */
	LaserReading* parseRawLaser(std::istream& _stream) const;
};


/** 
 * Representation of a CARMEN log writer. It implements the LogWriter class for the CARMEN log file format.
 *
 * @author Gian Diego Tipaldi
 */

class CarmenLogWriter: public LogWriter{
    public:
	/** Virtual Default destructor */
	virtual ~CarmenLogWriter() { }
	/** Write a log to an outputstream */
	virtual void writeLog(std::ostream& _stream, const std::vector<AbstractReading*>& _log) const;
	/** Write a single line to the stream. */
	virtual void writeLine(std::ostream& _stream, const AbstractReading* _reading) const;

    protected:
	/** Write the old laser structure (FLASER). */
	void writeFLaser(std::ostream& _stream, const LaserReading* _reading) const;
	/** Write the new laser structure (ROBOTLASERX). */
	void writeRobotLaser(std::ostream& _stream, const LaserReading* _reading) const;
	/** Write the raw laser structure (RAWLASERX). */
	void writeRawLaser(std::ostream& _stream, const LaserReading* _reading) const;
};

#endif

