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

#include "InterestPointRenderer.h"

InterestPointRenderer::InterestPointRenderer(const std::vector<const OrientedPoint2D *> *_points, 
					     const std::vector<double> * _scales):
    m_interestPoints(_points),
    m_scales(_scales),
    m_laserPose(NULL)
{ 
    if(!_points || (_scales && _points->size() != _scales->size())){
	m_interestPoints = NULL;
	m_scales = NULL;	
    }
    if(m_interestPoints){
	m_GLUPoints.resize(m_interestPoints->size());
	for(unsigned int i = 0; i < m_GLUPoints.size(); i++){
	    m_GLUPoints[i] = gluNewQuadric();
	}
	m_colors.resize(m_interestPoints->size(),Color(1.f,0.f,0.f,1.f));
    }
    m_scaleFactor = 1.f;
    m_depth = 0.f;
    setSubdivision(80, 12);
}

InterestPointRenderer::~InterestPointRenderer(){
    for(unsigned int i = 0; i < m_GLUPoints.size(); i++){
	gluDeleteQuadric(m_GLUPoints[i]);
    }
}

InterestPointRenderer::InterestPointRenderer(const InterestPointRenderer& _renderer):
    m_interestPoints(_renderer.getInterestPoints()),
    m_scales(_renderer.getScales()),
    m_GLUPoints(0),
    m_colors(_renderer.m_colors)
{
    if(m_interestPoints){
	m_GLUPoints.resize(m_interestPoints->size());
    }
    for(unsigned int i = 0; i < m_GLUPoints.size(); i++){
	m_GLUPoints[i] = gluNewQuadric();
    }
    m_depth = _renderer.m_depth;
    m_scaleFactor = _renderer.getScaleFactor();
    _renderer.getSubdivision(m_subdivision[0], m_subdivision[1]);
}

InterestPointRenderer& InterestPointRenderer::operator=(const InterestPointRenderer& _renderer){
    setInterestPoints(_renderer.getInterestPoints(), _renderer.getScales());
    m_scaleFactor = _renderer.getScaleFactor();
    m_colors = _renderer.getColors();
    m_depth = _renderer.m_depth;
    _renderer.getSubdivision(m_subdivision[0], m_subdivision[1]);
    return *this;
}

void InterestPointRenderer::setInterestPoints(const std::vector<const OrientedPoint2D *> *_points,
					      const std::vector<double> * _scales) 
{
    if(_points && _scales && _points->size() == _scales->size()){
	m_interestPoints = _points;
	m_scales = _scales;
    } else {
	m_interestPoints = NULL;
	m_scales = NULL;
	return;
    }
    
    if(m_interestPoints->size() == m_GLUPoints.size()) return;
    
    unsigned int i = m_interestPoints->size();
    for(; i < m_GLUPoints.size(); i++){
	gluDeleteQuadric(m_GLUPoints[i]);
    }
    
    i = m_GLUPoints.size();
    m_GLUPoints.resize(m_interestPoints->size());
    for(; i < m_GLUPoints.size(); i++){
	m_GLUPoints[i] = gluNewQuadric();
    }
}
	

void InterestPointRenderer::render(){
    if(!m_interestPoints) return;
    glPushMatrix();
    glTranslated(0., 0., m_depth);
    if(m_laserPose) {
// 	std::cout << "Laser pose = " << *m_laserPose << std::endl;
	glTranslated(m_laserPose->x, m_laserPose->y, 0.);
	glRotated(rad2deg(m_laserPose->theta), 0., 0., 1.);
    }
    for (unsigned int index = 0; index < m_interestPoints->size(); index++) {
	glPushMatrix();
// 	std::cout << "\t" << *(*m_interestPoints)[index] << ", transf = " << m_laserPose->oplus(*(*m_interestPoints)[index]) << std::endl;
	glTranslated((*m_interestPoints)[index]->x, (*m_interestPoints)[index]->y, 0);
	glRotated(rad2deg((*m_interestPoints)[index]->theta), 0., 0., 1.);
	glColor4f(m_colors[index].red(), m_colors[index].green(), m_colors[index].blue(), m_colors[index].alpha());
	double size = m_scaleFactor * (*m_scales)[index];
	
	glBegin(GL_LINES);
	    glVertex3f(0.0f,0.0f,0.0f);
	    glVertex3f(size, 0.f ,0.0f);
	glEnd();
	
	glBegin(GL_LINE_STRIP);
	    for(double i = -M_PI; i <= M_PI; i += 2.0*M_PI/(double)m_subdivision[0])
		glVertex3f(cos(i)*size, sin(i)*size,0.0f);
	glEnd();
	
	gluSphere(m_GLUPoints[index], 0.02f, m_subdivision[0], m_subdivision[1]);
	glPopMatrix();
    }
    glPopMatrix();
}
