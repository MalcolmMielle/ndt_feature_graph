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

#include "LaserReading.h"


using namespace std;

LaserReading::LaserReading(const vector<double>& _phi, const vector<double>& _rho, double _time, const string& _name, const string& _robot):
    AbstractReading(_time, _name, _robot),
    m_maxRange(80),
    m_phi(_phi),
    m_rho(_rho)
{
    m_remission.reserve(m_rho.size());
    m_laserPose.x = 0;
    m_laserPose.y = 0;
    m_laserPose.theta = 0;
    m_robotPose.x = 0;
    m_robotPose.y = 0;
    m_robotPose.theta = 0;
    computeLocalCartesian();
    m_worldCartesian = m_cartesian;
}

LaserReading::~LaserReading(){

}

void LaserReading::setRemission(const vector<double>& _remi){
    if(_remi.size() == m_phi.size())
	m_remission = _remi;
}

void LaserReading::setLaserPose(const OrientedPoint2D& _pose){
    m_laserPose = _pose;
    computeWorldCartesian();
}

void LaserReading::setRobotPose(const OrientedPoint2D& _pose){
    m_robotPose = _pose;
}

void LaserReading::computeWorldCartesian(){
    m_worldCartesian.resize(m_phi.size());
    m_worldCartesianNoMax.clear();
    for(unsigned int i = 0; i < m_phi.size(); i++){
	if(m_rho[i]<m_maxRange){
	    m_worldCartesian[i].x = cos(m_phi[i] + m_laserPose.theta)*m_rho[i] + m_laserPose.x;
	    m_worldCartesian[i].y = sin(m_phi[i] + m_laserPose.theta)*m_rho[i] + m_laserPose.y;
	    m_worldCartesianNoMax.push_back(m_worldCartesian[i]);
	} else {
	    m_worldCartesian[i].x = cos(m_phi[i] + m_laserPose.theta)*m_maxRange + m_laserPose.x;
	    m_worldCartesian[i].y = sin(m_phi[i] + m_laserPose.theta)*m_maxRange + m_laserPose.y;
/*	    m_worldCartesian[i].x = 0;
	    m_worldCartesian[i].y = 0;*/
	}
    }
}

void LaserReading::computeLocalCartesian(){
    m_cartesian.resize(m_phi.size());
    m_cartesianNoMax.clear();
    for(unsigned int i = 0; i < m_phi.size(); i++){
	if(m_rho[i]<m_maxRange){
	    m_cartesian[i].x = cos(m_phi[i])*m_rho[i];
	    m_cartesian[i].y = sin(m_phi[i])*m_rho[i];
	    m_cartesianNoMax.push_back(m_cartesian[i]);
	} else {
	    m_cartesian[i].x = cos(m_phi[i])*m_maxRange;
	    m_cartesian[i].y = sin(m_phi[i])*m_maxRange;
	}
    }
}

AbstractReading* LaserReading::clone() const{
    return new LaserReading(*this);
}
