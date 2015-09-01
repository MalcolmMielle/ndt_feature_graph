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

#include "DescriptorWidget.h"
#include "DescriptorWidget.moc"

#include <iostream>

DescriptorWidget::DescriptorWidget(QWidget *parent):
    QGraphicsView(parent),
    m_items(0)
{
    m_scene = new QGraphicsScene(this);
    setScene(m_scene);
    setCacheMode(CacheBackground);
    setViewportUpdateMode(BoundingRectViewportUpdate);
    setRenderHint(QPainter::Antialiasing);
}
	
void DescriptorWidget::addDescriptor(QGraphicsItem * item){
    if(!item) return;
    QRectF bounding = item->boundingRect();
    float adjust = 10.;
    float width = adjust + bounding.width();
    float height = adjust + bounding.height();
    item->setPos(m_position.x(), m_position.y());
    m_items.push_back(item);
    m_scene->addItem(item);
    m_position.rx() += width + adjust;
    update();
}

void DescriptorWidget::addSeparator() {
    if(!m_items.size()) return;
    QRectF bounding = m_items.front()->boundingRect();
    float adjust = 10.;
    float height = adjust + bounding.height();
    QLineF line(m_position.x(), m_position.y(), m_position.x(), m_position.y() + height);
    QPen pen;
    pen.setWidth(3);
    m_scene->addLine(line, pen);
    m_position.rx() += adjust + pen.widthF();
}

void DescriptorWidget::addNewLine() {
    QRectF bounding = m_items.front()->boundingRect();
    float adjust = 10.;
    float width = adjust + bounding.width();
    float height = adjust + bounding.height();
    m_position.rx() = m_items.front()->x();
    m_position.ry() += height;
}    

void DescriptorWidget::clear(){
    if(m_scene) delete m_scene;
    m_scene = new QGraphicsScene(this);
    setScene(m_scene);
    setCacheMode(CacheBackground);
    setViewportUpdateMode(BoundingRectViewportUpdate);
    setRenderHint(QPainter::Antialiasing);
/*    foreach(QGraphicsItem * item, m_items){
	m_scene->removeItem(item);
	delete item;
    }*/	
    m_items.clear();
//     scene->set
}
	
void DescriptorWidget::mousePressEvent(QMouseEvent *event){
    const QGraphicsItem * selected = itemAt(event->pos());
    int index = findDescriptor(selected);
    if(index > -1) emit descriptorSelected(index);
}
	
int DescriptorWidget::findDescriptor(const QGraphicsItem * item){
    int index = -1;
    for(int i = 0; i < m_items.size(); i++){
	if(m_items[i] == item){
	    index = i;
	    break;
	}
    }
    return index;
}
