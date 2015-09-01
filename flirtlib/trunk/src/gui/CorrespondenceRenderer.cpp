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

#include "CorrespondenceRenderer.h"

CorrespondenceRenderer::CorrespondenceRenderer(const std::vector< std::pair<Point2D, Point2D> > *correspondences, const std::vector< double > *distances):
    m_correspondences(correspondences),
    m_distances(NULL),
    m_referenceDepth(0),
    m_dataDepth(0),
    m_maxDistance(0)
{
    if(m_correspondences){
	m_colors.resize(m_correspondences->size(),Color(1.f,0.f,0.f,1.f));
	if(distances && m_correspondences->size() == distances->size()){
	    m_distances = distances;
	    for(unsigned int i = 0; i < m_distances->size(); i++){
		m_maxDistance = m_maxDistance > (*m_distances)[i] ? m_maxDistance : (*m_distances)[i];
	    }
	}
    }
}

void CorrespondenceRenderer::setCorrespondences(const std::vector< std::pair<Point2D, Point2D> > *correspondences, const std::vector< double > *distances)
{
    m_correspondences = correspondences;
    m_distances = distances;
    if(m_correspondences){
	m_colors.resize(m_correspondences->size(),Color(1.f,0.f,0.f,1.f));
	if(distances && m_correspondences->size() == distances->size()){
	    m_distances = distances;
	    for(unsigned int i = 0; i < m_distances->size(); i++){
		m_maxDistance = m_maxDistance > (*m_distances)[i] ? m_maxDistance : (*m_distances)[i];
	    }
	}
    }
}

void CorrespondenceRenderer::render()
{
    if(!m_correspondences) return;
    glPushMatrix();
    float width;
    glGetFloatv(GL_LINE_WIDTH, &width);
    glLineWidth(5.0);
    glBegin(GL_LINES);
    for(unsigned int i = 0; i < m_correspondences->size(); i++){
	float alpha = m_distances ? 1.f - (*m_distances)[i]/m_maxDistance : 1.f;
// 	glColor4f(m_colors[i].red(), m_colors[i].green(), m_colors[i].blue(), alpha);
	glColor4f(alpha, alpha, alpha, 1.f);
	glVertex3f((*m_correspondences)[i].first.x, (*m_correspondences)[i].first.y, m_referenceDepth);
	glVertex3f((*m_correspondences)[i].second.x, (*m_correspondences)[i].second.y, m_dataDepth);
    }
    glEnd();
    glLineWidth(width);
    glPopMatrix();
}