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

#include "LaserReadingRenderer.h"

LaserReadingRenderer::LaserReadingRenderer(const std::vector<Point2D> *_points, const std::vector<double> &normals):
    m_laserPoints(_points),
    m_laserNormals(normals),
    m_GLUPoints(0),
    m_laserPose(NULL),
    m_color(0.0f, 0.0f, 0.0f, 1.0f)
{ 
    m_list = glGenLists(1);
    if(m_laserPoints){
	m_GLUPoints.resize(m_laserPoints->size());
	for(unsigned int i = 0; i < m_GLUPoints.size(); i++){
	    m_GLUPoints[i] = gluNewQuadric();
	}
    }

    m_pointSize = 0.02f;
    setSubdivision(6, 3);
    m_depth = 0.f;
    m_listValid=false;
}

LaserReadingRenderer::~LaserReadingRenderer(){
    for(unsigned int i = 0; i < m_GLUPoints.size(); i++){
	gluDeleteQuadric(m_GLUPoints[i]);
    }
    glDeleteLists(m_list,1);
}

LaserReadingRenderer::LaserReadingRenderer(const LaserReadingRenderer& _renderer):
    m_laserPoints(_renderer.getLaserPoints()),
    m_laserNormals(_renderer.getLaserNormals()),
    m_GLUPoints(0),
    m_laserPose(_renderer.m_laserPose),
    m_color(_renderer.m_color)
{
    m_list = glGenLists(1);
    if(m_laserPoints){
	m_GLUPoints.resize(m_laserPoints->size());
	for(unsigned int i = 0; i < m_GLUPoints.size(); i++){
	    m_GLUPoints[i] = gluNewQuadric();
	}
    }
    m_pointSize = _renderer.getSize();
    _renderer.getSubdivision(m_subdivision[0], m_subdivision[1]);
    m_depth = _renderer.m_depth;
    m_listValid=false;
}

LaserReadingRenderer& LaserReadingRenderer::operator=(const LaserReadingRenderer& _renderer){
    setLaserPoints(_renderer.getLaserPoints(), _renderer.getLaserNormals());
    m_laserPose = _renderer.m_laserPose;
    m_color = _renderer.getColor();
    m_pointSize = _renderer.getSize();
    _renderer.getSubdivision(m_subdivision[0], m_subdivision[1]);
    m_depth = _renderer.m_depth;
    m_listValid=false;
    return *this;
}

void LaserReadingRenderer::setLaserPoints(const std::vector<Point2D> *_points, const std::vector<double>& normals){
    m_laserPoints = _points;
    
    if(!m_laserPoints) return;
    
    if (normals.size() == m_laserPoints->size()){
	m_laserNormals = normals;
    } else {
	m_laserNormals.clear();
    }
    if(m_laserPoints->size() == m_GLUPoints.size()) return;
    
    unsigned int i = m_laserPoints ? m_laserPoints->size() : 0;
    for(; i < m_GLUPoints.size(); i++){
	gluDeleteQuadric(m_GLUPoints[i]);
    }
    
    i = m_GLUPoints.size();
    if(m_laserPoints){
	m_GLUPoints.resize(m_laserPoints->size());
	for(; i < m_GLUPoints.size(); i++){
	    m_GLUPoints[i] = gluNewQuadric();
	}
    }
    m_listValid=false;
}
	
void LaserReadingRenderer::render(){
    if(!m_listValid)
	makeList();
    glCallList(m_list);
}

void LaserReadingRenderer::makeList(){
//     std::cout << "Making the list" << std::endl;
    glNewList(m_list, GL_COMPILE);
    if(!m_laserPoints) {
	glEndList();
// 	std::cout << "\tNo points found!" << std::endl;
	return;
    }
    glPushMatrix();
    glColor4f(m_color.red(), m_color.green(), m_color.blue(), m_color.alpha());
    glTranslatef(0.f, 0.f, m_depth);
    if(m_laserPose){
// 	glPushMatrix();
	glTranslatef(m_laserPose->x, m_laserPose->y, 0.f);
	glRotatef(rad2deg(m_laserPose->theta),0.f,0.f,1.f);
	glBegin(GL_TRIANGLES);
	    glVertex3f( 0.0f, -0.2f, 0.f);
	    glVertex3f( 0.4f,  0.0f, 0.f);
	    glVertex3f( 0.0f,  0.2f, 0.f);
	glEnd();
// 	glPopMatrix();
    }
    
    glBegin(GL_POINTS);
    for (unsigned int index = 0; m_laserPoints && index < m_laserPoints->size(); index++) {
	glVertex3f((*m_laserPoints)[index].x, (*m_laserPoints)[index].y, 0.f);
    }
    glEnd();
    
    for (unsigned int index = 0; m_laserPoints && m_laserNormals.size() && index < m_laserPoints->size(); index++) {
	glPushMatrix();
	if(m_laserNormals.size()){
	    glTranslated((*m_laserPoints)[index].x, (*m_laserPoints)[index].y, 0.f);
	    glBegin(GL_LINES);
		glVertex3f(0.f, 0.f, 0.f);
		glVertex3f(cos(m_laserNormals[index]), sin(m_laserNormals[index]), 0.f);
	    glEnd();
	}
// 	gluSphere(m_GLUPoints[index], m_pointSize, m_subdivision[0], m_subdivision[1]);
	glPopMatrix();
    }
    glPopMatrix();
    glEndList();
//     std::cout << "List Completed" << std::endl;
}
