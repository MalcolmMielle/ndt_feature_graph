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

#ifndef DESCRIPTORCHOOSERPRESENTER_H_
#define DESCRIPTORCHOOSERPRESENTER_H_

#include <gui/DescriptorPresenter.h>
#include <gui/ParameterWidget.h>
#include <QtGui/QComboBox>
#include <QtGui/QLabel>
#include <QtGui/QGridLayout>
#include <QtCore/QVector>
#include <QtCore/QObject>
#include <QtCore/QString>

// class DescriptorChooserWidget: public QWidget{
//     Q_OBJECT
//     
//     public:
// 	DescriptorChooserWidget(QWidget * parent = 0);
//     
// 	inline int getDescriptor() const
// 	    {return m_descriptor.currentIndex();}
// 	inline void insertDescriptor(const QString& name)
// 	    {m_descriptor.addItem(name);}
// 	    
// 	void resetDescriptor();
// 	
//     signals:
// 	void descriptorChanged(int descriptor);
// 	
//     public slots:
// 	void changeDescriptor(int descriptor);
// 	
//     protected:
// 	void buildGui();
// 	
// 	QComboBox m_descriptor;
// 	QLabel m_descriptorLabel;
// 	QGridLayout m_layout;
// 
// };

class DescriptorChooserPresenter: public QObject{
    Q_OBJECT

    public:
	DescriptorChooserPresenter(ParameterWidget* chooser);
	
	void insertDescriptor(const QString& name, DescriptorPresenter* descriptor);

	void setChooser(ParameterWidget* chooser);
	
	inline const ParameterWidget* getChooserParameter() const
	    {return m_chooser;}
	
	inline const DescriptorPresenter* getCurrentDescriptorPresenter() const
	    {return m_currentDescriptorPresenter;}
	
	inline const DescriptorGenerator* getCurrentDescriptor() const
	    {return m_currentDescriptorPresenter->getDescriptor();}
	
	inline ParameterWidget* getChooserParameter() 
	    {return m_chooser;}
	
	inline DescriptorPresenter* getCurrentDescriptorPresenter() 
	    {return m_currentDescriptorPresenter;}
	
	inline DescriptorGenerator* getCurrentDescriptor() 
	    {return m_currentDescriptorPresenter->getDescriptor();}
		
    signals:
	void descriptorChanged();
	
    public slots:
	void changeDescriptor(int descriptor);	
	void changeParameter(const QString& name);	

    protected:
	void syncronize();
	void reconnect();
	ParameterWidget* m_chooser;
	DescriptorPresenter* m_currentDescriptorPresenter;
	int m_currentDescriptorPresenterIndex;
	QVector< DescriptorPresenter* > m_descriptorPresenters;
	QStringList m_descriptorPresenterNames;
};

#endif

