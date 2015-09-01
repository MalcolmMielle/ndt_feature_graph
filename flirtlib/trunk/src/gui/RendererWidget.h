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

#ifndef RENDERERWIDGET_H_
#define RENDERERWIDGET_H_

#include <gui/AbstractRenderer.h>

#include <QtOpenGL/QGLWidget>
#include <QtGui/QMouseEvent>
#include <vector>
#include <algorithm>
#include <iostream>

class RendererWidget: public QGLWidget{
    Q_OBJECT
    public:
	RendererWidget(QWidget *_parent = 0);
	inline void addRenderer(AbstractRenderer* _renderer)
	    {m_renderObject.push_back(_renderer);}
	inline void removeRenderer(AbstractRenderer* _renderer)
	    {m_renderObject.erase(std::find(m_renderObject.begin(), m_renderObject.end(), _renderer));}
	    
	void unprojectCoordinates(GLdouble viewX, GLdouble viewY, GLdouble viewZ, GLdouble* worldX, GLdouble* worldY, GLdouble* worldZ);
	void unprojectCoordinates(GLdouble viewX, GLdouble viewY, GLdouble* worldX, GLdouble* worldY, GLdouble* worldZ);
    
    signals:
	void mousePressedGL(int button, int x, int y);
	void mouseMovedGL(int button, int x1, int y1, int x2, int y2);
    
    public slots:
	void setLaserPose(float _x, float _y, float _theta = 0);
	void setRotation(float _x, float _y, float _z);
	void setOffset(float _x, float _y, float _z);
	void setScale(float _scale);
	void makeSnapshot(const char * _filename);
    
    protected:
	virtual void initializeGL();
	virtual void resizeGL(int _width, int _height);
	virtual void paintGL();
	
	virtual void mousePressEvent(QMouseEvent *_event);
	virtual void mouseReleaseEvent(QMouseEvent *_event);
	virtual void mouseMoveEvent(QMouseEvent *_event);
	virtual void wheelEvent(QWheelEvent *_event);

	std::vector<AbstractRenderer * > m_renderObject;
	
	GLdouble m_GLdefaultDepth;
	
	float m_laserPoseX;
	float m_laserPoseY;
	float m_laserPoseTheta;

	float m_rotationX;
	float m_rotationY;
	float m_rotationZ;
	float m_offsetX;
	float m_offsetY;
	float m_offsetZ;
	float m_scale;
	
	QPoint m_lastMousePosition;
	QPoint m_lastMousePressPosition;
};

#endif

