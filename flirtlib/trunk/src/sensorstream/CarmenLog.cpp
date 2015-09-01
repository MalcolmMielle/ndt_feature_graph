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

#include "CarmenLog.h"

#include <boost/algorithm/string/predicate.hpp>
#include <math.h>

/** \def MAX_LINE_SIZE The maximum length of a line. */
#define MAX_LINE_SIZE 8192

void CarmenLogReader::readLog(std::istream& _stream, std::vector<AbstractReading*>& _log) const{
    char buffer[MAX_LINE_SIZE];

    while(_stream){
	_stream.getline(buffer,MAX_LINE_SIZE);
	std::istringstream instream(buffer);
	AbstractReading *reading = readLine(instream);
	if (reading)
	    _log.push_back(reading);
    }
}

AbstractReading* CarmenLogReader::readLine(std::istream& _stream) const{
    std::string sensorName;
    _stream >> sensorName;
    _stream.seekg(0, std::ios_base::beg);
    
    if (boost::iequals(sensorName,"FLASER")){
	return parseFLaser(_stream);
    } else if (boost::istarts_with(sensorName,"ROBOTLASER")){
	return parseRobotLaser(_stream);
    } else if (boost::istarts_with(sensorName,"RAWLASER")){
	return parseRawLaser(_stream);
    }
    return 0;
}

LaserReading* CarmenLogReader::parseRobotLaser(std::istream& _stream) const{
    std::string sensorName, robotName;
    std::vector<double> phi,rho,remission;
    unsigned int number=0, remissionNumber=0;
    OrientedPoint2D laserPose, robotPose;
    double timestamp;
    double start, fov, resolution, maxRange, accuracy;
    int laserType, remissionMode;
    double tv, rv, forward_safety_dist, side_safety_dist, turn_axis;
    
    _stream >> sensorName >> laserType >> start >> fov >> resolution >> maxRange >> accuracy >> remissionMode; // Laser sensor parameters
    
    _stream >> number;
    phi.resize(number);
    rho.resize(number);
    
    for(uint i=0; i < number; i++){
	phi[i] = start + i*resolution;
	_stream >> rho[i];
    }

    _stream >> remissionNumber;
    remission.resize(remissionNumber);
    
    for(uint i=0; i<remissionNumber; i++){
	_stream >> remission[i];
    }
    
    _stream >> laserPose.x >> laserPose.y >> laserPose.theta;
    _stream >> robotPose.x >> robotPose.y >> robotPose.theta;
    
    _stream >> tv >> rv >> forward_safety_dist >> side_safety_dist >> turn_axis;
    
    _stream >> timestamp >> robotName;
    
    LaserReading *result = new LaserReading(phi, rho, timestamp, sensorName, robotName);
    result->setMaxRange(maxRange);
    result->setRemission(remission);
    result->setLaserPose(laserPose);
    result->setRobotPose(robotPose);
    
    return result;
}

LaserReading* CarmenLogReader::parseRawLaser(std::istream& _stream) const{
    std::string sensorName, robotName;
    std::vector<double> phi,rho,remission;
    unsigned int number, remissionNumber;
    OrientedPoint2D laserPose, robotPose;
    double timestamp;
    double start, fov, resolution, maxRange, accuracy;
    int laserType, remissionMode;
    
    _stream >> sensorName >> laserType >> start >> fov >> resolution >> maxRange >> accuracy >> remissionMode; // Laser sensor parameters
    
    _stream >> number;
    phi.resize(number);
    rho.resize(number);
    
    for(uint i=0; i < number; i++){
	phi[i] = start + i*resolution;
	_stream >> rho[i];
    }

    _stream >> remissionNumber;
    remission.resize(remissionNumber);
    
    for(uint i=0; i<remissionNumber; i++){
	_stream >> remission[i];
    }
        
    _stream >> timestamp >> robotName;
    
    LaserReading *result = new LaserReading(phi, rho, timestamp, sensorName, robotName);
    result->setMaxRange(maxRange);
    result->setRemission(remission);
//     result->setLaserPose(laserPose);
    
    return result;
}

LaserReading* CarmenLogReader::parseFLaser(std::istream& _stream) const{
    std::string sensorName, robotName;
    std::vector<double> phi,rho;
    unsigned int number;
    OrientedPoint2D laserPose, robotPose;
    double timestamp;
    double start, fov, resolution, maxRange;
    
    _stream >> sensorName >> number;
    
    phi.resize(number);
    rho.resize(number);
    start = -M_PI_2;
    fov = M_PI;
    resolution = fov/number;
    maxRange = 81.9;
    
    for(uint i=0; i<number; i++){
	phi[i] = start + i*resolution;
	_stream >> rho[i];
    }

    _stream >> laserPose.x >> laserPose.y >> laserPose.theta;
    _stream >> robotPose.x >> robotPose.y >> robotPose.theta;
    
    _stream >> timestamp >> robotName;
    
    LaserReading *result = new LaserReading(phi, rho, timestamp, sensorName, robotName);
    result->setMaxRange(maxRange);
    result->setLaserPose(laserPose);
    result->setRobotPose(robotPose);
    
    return result;
}


void CarmenLogWriter::writeLog(std::ostream& _stream, const std::vector<AbstractReading*>& _log) const{
    for(uint i = 0; i < _log.size(); i++){
	writeLine(_stream, _log[i]);
    }
}

void CarmenLogWriter::writeLine(std::ostream& _stream, const AbstractReading* _reading) const{
    if (boost::iequals(_reading->getName(),"FLASER")){
	writeFLaser(_stream, dynamic_cast<const LaserReading*>(_reading));
    } else if (boost::istarts_with(_reading->getName(),"ROBOTLASER")){
	writeRobotLaser(_stream, dynamic_cast<const LaserReading*>(_reading));
    } else if (boost::istarts_with(_reading->getName(),"RAWLASER")){
	writeRawLaser(_stream, dynamic_cast<const LaserReading*>(_reading));
    }
}

void CarmenLogWriter::writeFLaser(std::ostream& _stream, const LaserReading* _reading) const{
    _stream << std::fixed;
    _stream << _reading->getName() << " ";
    
    const std::vector<double>& rho = _reading->getRho();
    _stream << rho.size() << " ";
    
    _stream.precision(3);
    for(uint i = 0; i < rho.size(); i++){
	_stream << rho[i] << " ";
    }
    
    const OrientedPoint2D &laserPose = _reading->getLaserPose();
    const OrientedPoint2D &robotPose = _reading->getRobotPose();
    
    _stream.precision(6);
    _stream << laserPose.x << " " << laserPose.y << " " << laserPose.theta << " ";
    _stream << robotPose.x << " " << robotPose.y << " " << robotPose.theta << " ";
    
    _stream << _reading->getTime() << " " << _reading->getRobot() << " " << _reading->getTime() << std::endl;
}

void CarmenLogWriter::writeRobotLaser(std::ostream& _stream, const LaserReading* _reading) const{
    _stream << std::fixed;
    _stream << _reading->getName() << " " << "0 ";
    
    const std::vector<double>& rho = _reading->getRho();
    const std::vector<double>& phi = _reading->getPhi();

    _stream.precision(6);
    _stream << phi.front() << " " << phi.back() - phi.front() << " " << (phi[1] - phi[0]) << " " << _reading->getMaxRange() << " " << "0.010000 "<< "0 ";
    
    _stream << rho.size() << " ";
    
    _stream.precision(3);
    for(uint i = 0; i < rho.size(); i++){
	_stream << rho[i] << " ";
    }
    
    const std::vector<double>& remission = _reading->getRemission();
    _stream << remission.size() << " ";
    
    for(uint i = 0; i < remission.size(); i++){
	_stream << remission[i] << " ";
    }
    
    const OrientedPoint2D &laserPose = _reading->getLaserPose();
    const OrientedPoint2D &robotPose = _reading->getRobotPose();
    
    _stream.precision(6);
    _stream << laserPose.x << " " << laserPose.y << " " << laserPose.theta << " ";
    _stream << robotPose.x << " " << robotPose.y << " " << robotPose.theta << " ";
    
    _stream << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " ";
    
    _stream << _reading->getTime() << " " << _reading->getRobot() << " " << _reading->getTime() << std::endl;
}

void CarmenLogWriter::writeRawLaser(std::ostream& _stream, const LaserReading* _reading) const{
    _stream << std::fixed;
    _stream << _reading->getName() << " " << "0 ";
    
    const std::vector<double>& rho = _reading->getRho();
    const std::vector<double>& phi = _reading->getPhi();

    _stream.precision(6);
    _stream << phi.front() << " " << phi.back() - phi.front() << " " << (phi[1] - phi[0]) << " " << _reading->getMaxRange() << " " << "0.010000 "<< "0 ";
    
    _stream << rho.size() << " ";
    
    _stream.precision(3);
    for(uint i = 0; i < rho.size(); i++){
	_stream << rho[i] << " ";
    }
    
    const std::vector<double>& remission = _reading->getRemission();
    _stream << remission.size() << " ";
    
    for(uint i = 0; i < remission.size(); i++){
	_stream << remission[i] << " ";
    }
        
    _stream << _reading->getTime() << " " << _reading->getRobot() << " " << _reading->getTime() << std::endl;
}
