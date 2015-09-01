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

#ifndef SENSORSTREAMWIDGET_H_
#define SENSORSTREAMWIDGET_H_

#include <sensorstream/SensorStream.h>

#include <QtGui/QGridLayout>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QFrame>
#include <QtGui/QSpinBox>

class SensorStreamWidget: public QFrame{
    Q_OBJECT
    
    public:
	SensorStreamWidget(QWidget * _parent = 0);
	virtual ~SensorStreamWidget();
	inline int getPosition() const {return m_position;}
    
    signals:
	void newReading();
	void newReading(int _position);
	
    public slots:
	void seekPosition(int _position);
	void nextPosition();
	void seekable(bool seek, unsigned int _size = 0);
	void streamReady();

    protected:
	void buildGui();
    
	bool m_isSeekable;
	unsigned int m_position;
	QGridLayout *m_layout;
	QPushButton *m_nextButton;
	QSlider *m_sensorSlider;
	QSpinBox *m_sensorBox;
};

#endif

