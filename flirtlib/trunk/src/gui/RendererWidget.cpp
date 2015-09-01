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

#include "RendererWidget.h"
#include "RendererWidget.moc"

#include <geometry/point.h>

static GLfloat LightAmb[] = { 0.3f, 0.3f, 0.3f, 1.0f }; // Ambient Light
static GLfloat LightDif[] = { 0.6f, 0.6f, 0.6f, 1.0f }; // Diffuse Light
static GLfloat LightPos[] = { 6.0f, 6.0f, 8.0f, 1.0f }; // Light Position


RendererWidget::RendererWidget(QWidget *_parent): 
    QGLWidget(_parent),
    m_laserPoseX(0),
    m_laserPoseY(0),
    m_laserPoseTheta(0),
    m_rotationX(0.0),
    m_rotationY(0.0),
    m_rotationZ(0.0),
    m_offsetX(0.0),
    m_offsetY(0.0),
    m_offsetZ(-10.0),
    m_scale(1.0)
{

}

void RendererWidget::unprojectCoordinates(GLdouble viewX, GLdouble viewY, GLdouble* worldX, GLdouble* worldY, GLdouble* worldZ){
    makeCurrent();
/*    
    GLint viewPort[4];
    glGetIntegerv( GL_VIEWPORT, viewPort);

    GLdouble viewZ;
    glReadPixels( viewX, viewY, 1, 1, GL_DEPTH_COMPONENT, GL_DOUBLE, &viewZ );*/
    unprojectCoordinates(viewX, viewY, m_GLdefaultDepth, worldX, worldY, worldZ);
}


void RendererWidget::unprojectCoordinates(GLdouble viewX, GLdouble viewY, GLdouble viewZ, GLdouble* worldX, GLdouble* worldY, GLdouble* worldZ){
    makeCurrent();

    GLdouble modelView[16];/* = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 10.0, 1.0};*/
    glGetDoublev( GL_MODELVIEW_MATRIX, modelView);
    GLdouble projection[16];
    glGetDoublev( GL_PROJECTION_MATRIX, projection);
    GLint viewPort[4];
    glGetIntegerv( GL_VIEWPORT, viewPort);
    gluUnProject( viewX, viewY, viewZ, modelView, projection, viewPort, worldX, worldY, worldZ);
}


void RendererWidget::initializeGL(){
    glShadeModel(GL_SMOOTH); // Enable Smooth Shading
    glClearColor(0.9f, 0.9f, 0.9f, 1.0f); // Background
    glClearDepth(1.0f); // Depth Buffer Setup
    glClearStencil(0); // Clear The Stencil Buffer To 0
    glEnable(GL_DEPTH_TEST); // Enables Depth Testing
    glDepthFunc(GL_LEQUAL); // The Type Of Depth Testing To Do
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // Enable Alpha Blending
    glEnable(GL_BLEND); // Enable Blending
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Really Nice Perspective Calculations
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST); // anti aliased lines
    
    glEnable(GL_LINE_SMOOTH); // anti aliased lines
    
    // glEnable(GL_TEXTURE_2D);
    
    glEnable(GL_CULL_FACE);
    
    glEnable(GL_POINT_SMOOTH); //anti aliased points
    glPointSize(3.0);



    glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDif);
    glLightfv(GL_LIGHT0, GL_POSITION, LightPos);
}

void RendererWidget::resizeGL(int width, int height){
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    GLfloat x = (GLfloat) width / (GLfloat) height;
    glFrustum(-x, x, -1.0, 1.0, 4.0, 15.0);
    glMatrixMode(GL_MODELVIEW);
}

void RendererWidget::paintGL(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glColor4f(0.f,0.f,0.f,1.0f);
    
    glTranslatef(m_offsetX, m_offsetY, m_offsetZ);
    glRotatef(m_rotationX, 1.0, 0.0, 0.0);
    glRotatef(m_rotationY, 0.0, 1.0, 0.0);
    glRotatef(m_rotationZ, 0.0, 0.0, 1.0);
    glScalef(m_scale, m_scale, m_scale);
    
    glTranslatef(-m_laserPoseX, -m_laserPoseY, 0.f);
    glRotatef(rad2deg(-m_laserPoseTheta), 0.f, 0.f, 1.f);

    GLdouble modelView[16];
    glGetDoublev( GL_MODELVIEW_MATRIX, modelView);
    GLdouble projection[16];
    glGetDoublev( GL_PROJECTION_MATRIX, projection);
    GLint viewPort[4];
    glGetIntegerv( GL_VIEWPORT, viewPort);
    GLdouble viewX, viewY, worldX=0., worldY=0., worldZ=0.;
    gluProject(worldX, worldY, worldZ, modelView, projection, viewPort, &viewX, &viewY, &m_GLdefaultDepth);

    for(unsigned int i = 0; i < m_renderObject.size(); i++){
	m_renderObject[i]->render();
    }
    
    return;
}

void RendererWidget::setRotation(float _x, float _y, float _z){
    m_rotationX = _x;
    m_rotationY = _y;
    m_rotationZ = _z;
}

void RendererWidget::setOffset(float _x, float _y, float _z){
    m_offsetX = _x;
    m_offsetY = _y;
    m_offsetZ = _z;    
}

void RendererWidget::setLaserPose(float x, float y, float theta){
    m_laserPoseX = x;
    m_laserPoseY = y;
    m_laserPoseTheta = theta;
}

void RendererWidget::setScale(float _scale){
    m_scale = _scale;
}

void RendererWidget::makeSnapshot(const char * _filename){

}
    
void RendererWidget::mousePressEvent(QMouseEvent *_event){
    m_lastMousePosition = _event->pos();
    m_lastMousePressPosition = _event->pos();
/*    
    makeCurrent();
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    GLint x = viewport[0] + _event->x();
    GLint y = viewport[0] + viewport[3] - _event->y();
    
    emit mousePressedGL(_event->buttons(), x, y);*/
}

void RendererWidget::mouseReleaseEvent(QMouseEvent *_event){
    if(m_lastMousePressPosition == _event->pos()){
	makeCurrent();
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	GLint x = viewport[0] + _event->x();
	GLint y = viewport[0] + viewport[3] - _event->y();
	m_lastMousePressPosition = _event->pos();
	emit mousePressedGL(!_event->buttons(), x, y);
    } else{
        m_lastMousePressPosition = _event->pos();
    }
}

void RendererWidget::mouseMoveEvent(QMouseEvent *_event){
    makeCurrent();
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    GLint x1 = viewport[0] + m_lastMousePosition.x();
    GLint y1 = viewport[1] + viewport[3] - m_lastMousePosition.y();
    GLint x2 = viewport[0] + _event->x();
    GLint y2 = viewport[1] + viewport[3] - _event->y();

    GLint dx = _event->x() - m_lastMousePosition.x();
    GLint dy = _event->y() - m_lastMousePosition.y();
    if (_event->buttons() & Qt::LeftButton) {
	    m_offsetX += ((double) dx) / width() * 8;
	    m_offsetY -= ((double) dy) / height() * 4;
    } else if (_event->buttons() & Qt::RightButton) {
	    m_rotationX += 0.4 * dy;
	    m_rotationZ += 0.4 * dx;
    }
    updateGL();
    m_lastMousePosition = _event->pos();
    emit mouseMovedGL(_event->buttons(), x1, y1, x2, y2);
}

void RendererWidget::wheelEvent(QWheelEvent *_event){
    _event->delta() > 0 ? m_scale += m_scale * 0.1f : m_scale -= m_scale * 0.1f;
    updateGL();
}

