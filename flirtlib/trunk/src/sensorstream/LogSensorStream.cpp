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

#include "LogSensorStream.h"

LogSensorStream::LogSensorStream(const LogReader* _reader, const LogWriter* _writer):
    m_index(0),
    m_logReader(_reader),
    m_logWriter(_writer)
{
    m_log.reserve(AVG_SIZE_LOG);
}

LogSensorStream::LogSensorStream(const LogSensorStream& _stream):
    m_log(_stream.getLog().size()),
    m_index(0),
    m_logReader(_stream.getReader()),
    m_logWriter(_stream.getWriter())
{
    const std::vector<AbstractReading*> log = _stream.getLog();
    for(unsigned int i = 0; i < log.size(); i++){
	m_log[i] = log[i]->clone();
    }
}

LogSensorStream& LogSensorStream::operator=(const LogSensorStream& _stream){
    m_index = 0;
    m_logReader = _stream.getReader();
    m_logWriter = _stream.getWriter();
    
    for(unsigned int i = 0; i < m_log.size(); i++){
	delete m_log[i];
    }
    
    const std::vector<AbstractReading*> log = _stream.getLog();
    m_log.resize(log.size());
    for(unsigned int i = 0; i < log.size(); i++){
	m_log[i] = log[i]->clone();
    }
    return *this;
}

LogSensorStream::~LogSensorStream(){
    for(unsigned int i=0; i < m_log.size(); i++){
	delete m_log[i];
    }
}

AbstractReading* LogSensorStream::next(){
    return m_log[m_index++];
}


AbstractReading* LogSensorStream::current(){
    return m_log[m_index];
}

const AbstractReading* LogSensorStream::next() const{
    return m_log[m_index++];
}


const AbstractReading* LogSensorStream::current() const{
    return m_log[m_index];
}

bool LogSensorStream::seek(const unsigned int _position, SensorStreamOffset _offset){
    unsigned int position = (_offset == BEGIN)? _position : m_log.size() -_position - 1;
    bool result = position >= 0 && position < m_log.size();
    m_index = result ? position : m_index;
    return result;
}

bool LogSensorStream::end() const{
    return m_index >= m_log.size();
}


void LogSensorStream::load(const std::string& _filename){
    std::ifstream infile;
    infile.open(_filename.c_str());
    load(infile);
    infile.close();
}

void LogSensorStream::save(const std::string& _filename){
    std::ofstream outfile;
    outfile.open(_filename.c_str());
    save(outfile);
    outfile.close();
}

void LogSensorStream::load(std::istream& _stream){
    m_logReader->readLog(_stream, m_log);
}

void LogSensorStream::save(std::ostream& _stream){
    m_logWriter->writeLog(_stream, m_log);
}
