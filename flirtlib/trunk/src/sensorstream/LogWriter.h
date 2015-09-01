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

#ifndef LOGWRITER_H_
#define LOGWRITER_H_

#include <sensors/AbstractReading.h>

#include <iostream>
#include <vector>

/** 
 * Representation of an abstract log writer. It defines the interface for writing a log file to a stream.
 *
 *
 * @author Gian Diego Tipaldi
 */
 
class LogWriter{
    public:
	/** Virtual Default destructor */
	virtual ~LogWriter() { }
	
	/** Write a log to an outputstream */
	virtual void writeLog(std::ostream& _stream, const std::vector<AbstractReading*>& _log) const = 0;

	/** Write a reading to an outputstream */
	virtual void writeLine(std::ostream& _stream, const AbstractReading* _reading) const = 0;
};

#endif

