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

#include "PolarGridRenderer.h"

PolarGridRenderer::PolarGridRenderer(const std::vector< std::vector<double> > *grid, const std::vector<double> *phiEdges, const std::vector<double> *rhoEdges):
    m_grid(NULL),
    m_phiEdges(NULL),
    m_rhoEdges(NULL),
    m_GLUGrids(0),
    m_GLUSectors(0),
    m_position(0),
    m_maxValue(0),
    m_color(0.4f, 1.f, 0.5f, 1.0f)
{ 
    if(grid && phiEdges && rhoEdges && grid->size() && grid->size() == phiEdges->size() - 1 && (*grid)[0].size() == rhoEdges->size() - 1){
	m_grid = grid;
	m_phiEdges = phiEdges;
	m_rhoEdges = rhoEdges;
	for(unsigned int i = 0; i < m_grid->size(); i ++){
	    for(unsigned int j = 0; j < (*m_grid)[i].size(); j++){
		if((*grid)[i][j] > m_maxValue ) m_maxValue = (*grid)[i][j];
	    }
	}
    }
    
    if(m_grid){
	m_GLUGrids.resize(m_grid->size() * (*m_grid)[0].size());
	m_GLUSectors.resize(m_grid->size() * (*m_grid)[0].size());
	for(unsigned int i = 0; i < m_GLUGrids.size(); i++){
	    m_GLUGrids[i] = gluNewQuadric();
	    m_GLUSectors[i] = gluNewQuadric();
	}
    }

    m_depth = 0.f;
    setSubdivision(6, 3);
}

PolarGridRenderer::~PolarGridRenderer(){
    for(unsigned int i = 0; i < m_GLUGrids.size(); i++){
	gluDeleteQuadric(m_GLUGrids[i]);
	gluDeleteQuadric(m_GLUSectors[i]);
    }
}

PolarGridRenderer::PolarGridRenderer(const PolarGridRenderer& _renderer):
    m_grid(_renderer.m_grid),
    m_phiEdges(_renderer.m_phiEdges),
    m_rhoEdges(_renderer.m_rhoEdges),
    m_GLUGrids(0),
    m_GLUSectors(0),
    m_maxValue(_renderer.m_maxValue),
    m_color(_renderer.m_color)
{
    if(m_grid){
	m_GLUGrids.resize(m_grid->size() * (*m_grid)[0].size());
	m_GLUSectors.resize(m_grid->size() * (*m_grid)[0].size());
	for(unsigned int i = 0; i < m_GLUGrids.size(); i++){
	    m_GLUGrids[i] = gluNewQuadric();
	    m_GLUSectors[i] = gluNewQuadric();
	}
    }
    m_depth = _renderer.m_depth;
    _renderer.getSubdivision(m_subdivision[0], m_subdivision[1]);
}

PolarGridRenderer& PolarGridRenderer::operator=(const PolarGridRenderer& _renderer){
    setGrid(_renderer.getGrid(), _renderer.getPhiEdges(), _renderer.getRhoEdges());
    m_color = _renderer.m_color;
    _renderer.getSubdivision(m_subdivision[0], m_subdivision[1]);
    m_depth = _renderer.m_depth;
    return *this;
}

void PolarGridRenderer::setGrid(const std::vector< std::vector<double> > *grid, const std::vector<double> *phiEdges, const std::vector<double> *rhoEdges){
    if(grid && phiEdges && rhoEdges && grid->size() && grid->size() == phiEdges->size() - 1 && (*grid)[0].size() == rhoEdges->size() - 1){
	m_grid = grid;
	m_phiEdges = phiEdges;
	m_rhoEdges = rhoEdges;
	m_maxValue = 0.;
	for(unsigned int i = 0; i < m_grid->size(); i ++){
	    for(unsigned int j = 0; j < (*m_grid)[i].size(); j++){
		if((*grid)[i][j] > m_maxValue ) m_maxValue = (*grid)[i][j];
	    }
	}
    } else {
	m_grid = NULL;
	m_phiEdges = NULL;
	m_rhoEdges = NULL;
	return;
    }
    
    if(m_grid->size() * (*m_grid)[0].size() == m_GLUGrids.size()) return;
    
    unsigned int i = m_grid->size() * (*m_grid)[0].size();
    for(; i < m_GLUGrids.size(); i++){
	gluDeleteQuadric(m_GLUGrids[i]);
	gluDeleteQuadric(m_GLUSectors[i]);
    }
    
    i = m_GLUGrids.size();
    m_GLUGrids.resize(m_grid->size() * (*m_grid)[0].size());
    m_GLUSectors.resize(m_grid->size() * (*m_grid)[0].size());
    for(; i < m_GLUGrids.size(); i++){
	m_GLUGrids[i] = gluNewQuadric();
	m_GLUSectors[i] = gluNewQuadric();
    }

}
	

void PolarGridRenderer::render(){
    if(m_grid){
	glPushMatrix();
	glTranslatef(0.f, 0.f, m_depth);
	if(m_position){
	    glTranslatef(m_position->x, m_position->y, 0.f);
	    glRotatef(rad2deg(m_position->theta), 0.f, 0.f, 1.f);
	}
	double sweep = 2*M_PI/(m_phiEdges->size() - 1);
	for(unsigned int i = 0; i < m_grid->size(); i ++){
	    for(unsigned int j = 0; j < (*m_grid)[i].size(); j++){
		glPushMatrix();
		m_color.lightness() = m_maxValue == 0.? 1. : 1. - 0.5*(*m_grid)[i][j]/m_maxValue;
		Color color(HSL2RGB(m_color));
		double inner = (*m_rhoEdges)[j];
		double outer = (*m_rhoEdges)[j+1];
		double start = -(*m_phiEdges)[i] + M_PI_2;
		glColor4f(color.red(), color.green(), color.blue(), color.alpha());
		gluQuadricDrawStyle(m_GLUSectors[i * (*m_grid)[i].size() + j], GLU_FILL);
 		gluPartialDisk(m_GLUSectors[i * (*m_grid)[i].size() + j], inner, outer, m_subdivision[0], m_subdivision[1], rad2deg(start), rad2deg(-sweep));
		glColor4f(0.f, 0.f, 0.f, 1.f);
		gluQuadricDrawStyle(m_GLUGrids[i * (*m_grid)[i].size() + j], GLU_SILHOUETTE);
 		gluPartialDisk(m_GLUGrids[i * (*m_grid)[i].size() + j], inner, outer, m_subdivision[0], m_subdivision[1], rad2deg(start), rad2deg(-sweep));
		glPopMatrix();
	    }
	}
	glColor4f(0.f,0.f,0.f,1.f);
	glBegin(GL_LINES);
	    glVertex3f(0.f, 0.f, 0.f);
	    glVertex3f((*m_rhoEdges)[m_rhoEdges->size() - 1] + 0.1f, 0.f, 0.f);
	glEnd();	
	glPopMatrix();
    }
}
