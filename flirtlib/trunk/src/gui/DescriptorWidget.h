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

#ifndef DESCRIPTORWIDGET_H_
#define DESCRIPTORWIDGET_H_

#include <QtGui/QGraphicsView>
#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsItem>
#include <QtGui/QMouseEvent>
#include <vector>


class DescriptorWidget: public QGraphicsView {
    Q_OBJECT
    
    public:
	DescriptorWidget(QWidget *parent = 0); 
	
	void addDescriptor(QGraphicsItem * item);

	void addSeparator();

	void addNewLine();

	void clear();
	
    signals:
	void descriptorSelected(int index);
	
    protected:
	virtual void mousePressEvent(QMouseEvent *event);
	
	int findDescriptor(const QGraphicsItem * item);
	
	QGraphicsScene * m_scene;
	QVector<QGraphicsItem * > m_items;
	QPointF m_position;


};


#endif

