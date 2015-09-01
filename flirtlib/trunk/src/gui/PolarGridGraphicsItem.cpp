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

#include "PolarGridGraphicsItem.h"

#include <cmath>
#include <geometry/point.h>


PolarGridGraphicsItem::PolarGridGraphicsItem(const std::vector< std::vector<double> > *grid, const std::vector<double> *phiEdges, const std::vector<double> *rhoEdges, QGraphicsItem* parent):
    QGraphicsItem(parent),
    m_grid(NULL),
    m_flatGrid(NULL),
    m_phiEdges(NULL),
    m_rhoEdges(NULL),
    m_maxValue(0),
    m_color(Qt::black)
{
    if(grid && phiEdges && rhoEdges && grid->size() == phiEdges->size() - 1 && (*grid)[0].size() == rhoEdges->size() - 1){
	m_grid = grid;
	m_phiEdges = phiEdges;
	m_rhoEdges = rhoEdges;
	for(unsigned int i = 0; i < m_grid->size(); i ++){
	    for(unsigned int j = 0; j < (*m_grid)[i].size(); j++){
		if((*grid)[i][j] > m_maxValue ) m_maxValue = (*grid)[i][j];
	    }
	}
    }
    
    setFlag(QGraphicsItem::ItemIsSelectable);
}

PolarGridGraphicsItem::PolarGridGraphicsItem(const std::vector< double > *grid, const std::vector<double> *phiEdges, const std::vector<double> *rhoEdges, QGraphicsItem* parent):
    QGraphicsItem(parent),
    m_grid(NULL),
    m_flatGrid(NULL),
    m_phiEdges(NULL),
    m_rhoEdges(NULL),
    m_maxValue(0),
    m_color(Qt::black)
{
    if(grid && phiEdges && rhoEdges && grid->size() == (phiEdges->size() - 1) * (rhoEdges->size() - 1)){
	m_flatGrid = grid;
	m_phiEdges = phiEdges;
	m_rhoEdges = rhoEdges;
	for(unsigned int i = 0; i < grid->size(); i ++){
	    if((*grid)[i] > m_maxValue ) m_maxValue = (*grid)[i];
	}
    }
    
    setFlag(QGraphicsItem::ItemIsSelectable);
}

void PolarGridGraphicsItem::setGrid(const std::vector< std::vector<double> > *grid, const std::vector<double> *phiEdges, const std::vector<double> *rhoEdges){
    if(grid && phiEdges && rhoEdges && grid->size() == phiEdges->size() - 1 && (*grid)[0].size() == rhoEdges->size() - 1){
	m_grid = grid;
	m_flatGrid = NULL;
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
}

void PolarGridGraphicsItem::setFlatGrid(const std::vector< double > *grid, const std::vector<double> *phiEdges, const std::vector<double> *rhoEdges){
    if(grid && phiEdges && rhoEdges && grid->size() == (phiEdges->size() - 1) * (rhoEdges->size() - 1) ){
	m_flatGrid = grid;
	m_grid = NULL;
	m_phiEdges = phiEdges;
	m_rhoEdges = rhoEdges;
	m_maxValue = 0.;
	for(unsigned int i = 0; i < grid->size(); i ++){
		if((*grid)[i] > m_maxValue ) m_maxValue = (*grid)[i];
	}
    } else {
	m_flatGrid = NULL;
	m_phiEdges = NULL;
	m_rhoEdges = NULL;
	return;
    }
}

void PolarGridGraphicsItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){
    painter->translate(64,64);
    if(m_grid){
	painter->setPen(Qt::black);
	m_color.setHsvF(m_color.hueF(),1.f,m_color.valueF());
	double sweep = normAngle(2*M_PI/(m_phiEdges->size() - 1), 0.);
	double maxEdge = m_rhoEdges->back();
	double minEdge = m_rhoEdges->front();
	for(unsigned int i = 0; i < m_grid->size(); i ++){
	    double start = normAngle((*m_phiEdges)[i], 0.);
	    for(unsigned int j = 0; j < (*m_grid)[i].size(); j++){
		float value = m_maxValue == 0.? 0. : (*m_grid)[i][j]/(m_maxValue + 0.5);
		m_color.setHsvF(m_color.hueF(), m_color.saturationF(), 1 - value); 
		double inner = double(128) * ((*m_rhoEdges)[j] - minEdge)/(maxEdge - minEdge);
		double outer = double(128) * ((*m_rhoEdges)[j + 1] - minEdge)/(maxEdge - minEdge);
		
		QPainterPath path;
		path.moveTo(inner/2. * cos(start), -inner/2. * sin(start));
		path.arcTo(-outer/2., -outer/2., outer, outer, rad2deg(start), rad2deg(sweep));
		path.arcTo(-floor(inner/2), -floor(inner/2), floor(inner), floor(inner), rad2deg(start + sweep), rad2deg(-sweep));
		path.closeSubpath();
		painter->setBrush(m_color);
		painter->drawPath(path);
	    }
	}
	painter->drawText(QRectF(-64, 135 - 64, 128, 25), Qt::AlignCenter, m_text);
    } else if(m_flatGrid) {
	painter->setPen(Qt::black);
	m_color.setHsvF(m_color.hueF(),1.f,m_color.valueF());
	double sweep = normAngle(2*M_PI/(m_phiEdges->size() - 1), 0.);
	double maxEdge = m_rhoEdges->back();
	double minEdge = m_rhoEdges->front();
	for(unsigned int i = 0; i < m_phiEdges->size() - 1; i ++){
	    double start = normAngle((*m_phiEdges)[i], 0.);
	    unsigned int edges = (*m_rhoEdges).size() - 1;
	    for(unsigned int j = 0; j < edges; j++){
		unsigned int index = i * edges + j;
		float value = m_maxValue == 0.? 0. : (*m_flatGrid)[index]/(m_maxValue + 0.5);
		m_color.setHsvF(m_color.hueF(), m_color.saturationF(), 1 - value); 
		double inner = double(128) * ((*m_rhoEdges)[j] - minEdge)/(maxEdge - minEdge);
		double outer = double(128) * ((*m_rhoEdges)[j + 1] - minEdge)/(maxEdge - minEdge);
		
		QPainterPath path;
		path.moveTo(inner/2. * cos(start), -inner/2. * sin(start));
		path.arcTo(-outer/2., -outer/2., outer, outer, rad2deg(start), rad2deg(sweep));
		path.arcTo(-floor(inner/2), -floor(inner/2), floor(inner), floor(inner), rad2deg(start + sweep), rad2deg(-sweep));
		path.closeSubpath();
		painter->setBrush(m_color);
		painter->drawPath(path);
	    }
	}
	painter->drawText(QRectF(-64, 135 - 64, 128, 25), Qt::AlignCenter, m_text);
    }
}

QRectF PolarGridGraphicsItem::boundingRect() const{
     return QRectF(0, 0, 128, 160);
}

QPainterPath PolarGridGraphicsItem::shape() const{
    QPainterPath path;
    path.addEllipse(boundingRect());
    return path; 
}

// void PolarGridGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent *event);