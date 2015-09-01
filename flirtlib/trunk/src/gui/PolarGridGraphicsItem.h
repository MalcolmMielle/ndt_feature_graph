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

#ifndef POLARGRIDGRAPHICSITEM_H_
#define POLARGRIDGRAPHICSITEM_H_

#include <QtGui/QGraphicsItem>
#include <QtGui/QPainter>
#include <QtGui/QPainterPath>
#include <vector>


class PolarGridGraphicsItem: public QGraphicsItem {
    public:
	PolarGridGraphicsItem(const std::vector< std::vector<double> > *grid, const std::vector<double> *phiEdges, const std::vector<double> *rhoEdges, QGraphicsItem* parent = 0); 
	
	PolarGridGraphicsItem(const std::vector< double > *grid, const std::vector<double> *phiEdges, const std::vector<double> *rhoEdges, QGraphicsItem* parent = 0); 
	
	inline void setColor(float _red, float _green, float _blue, float _alpha = 1.0f)
	    {m_color = QColor(_red, _green, _blue, _alpha);}
	inline void setColor(const QColor& color)
	    {m_color = color;}

	inline const QColor& getColor() const
	    {return m_color; }
	
	void setGrid(const std::vector< std::vector<double> > *grid, const std::vector<double> *phiEdges, const std::vector<double> *rhoEdges);
	
	void setFlatGrid(const std::vector< double > *grid, const std::vector<double> *phiEdges, const std::vector<double> *rhoEdges);
	
	inline const std::vector< std::vector<double> > * getGrid() const
	    {return m_grid;}
	
	inline const std::vector< double > * getFlatGrid() const
	    {return m_flatGrid;}
	
	inline const std::vector<double> * getPhiEdges() const
	    {return m_phiEdges;}
	    
	inline const std::vector<double> * getRhoEdges() const
	    {return m_rhoEdges;}
	    
	inline const QString& getText() const
	    {return m_text;}
	    
	inline void setText(const QString& text)
	    {m_text = text;}

	virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0);
	
	QRectF boundingRect() const;
	
	QPainterPath shape() const;
	
    protected:
// 	void mousePressEvent(QGraphicsSceneMouseEvent *event);
	
	const std::vector< std::vector<double> >* m_grid;
	const std::vector< double >* m_flatGrid;
	const std::vector<double> * m_phiEdges;
	const std::vector<double> * m_rhoEdges;
	double m_maxValue;
	QColor m_color;
	QString m_text;
};


#endif

