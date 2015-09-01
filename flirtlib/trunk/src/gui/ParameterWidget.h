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

#ifndef PARAMETERWIDGET_H_
#define PARAMETERWIDGET_H_

#include <QtGui/QWidget>
#include <QtGui/QSpinBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QComboBox>
#include <QtGui/QGridLayout>
#include <QtGui/QLabel>
#include <QtGui/QCheckBox>
#include <QtCore/QString>
#include <QtCore/QHash>
#include <QtCore/QSignalMapper>


struct ParameterPair{
    QLabel *description;
    QWidget *parameter;
};

class ParameterWidget: public QWidget {
    Q_OBJECT
    
    public:
	ParameterWidget(const QString &name, QWidget *parent = 0);
	
	virtual inline void activate()
	    {show();}
	    
	virtual inline void deactivate()
	    {hide();}
	
	void addDoubleParameter(const QString& name, const QString& description, double initialValue = 0., double minValue = -10e16, double maxValue = 10e16, int decimals = 2, double step = 0.1);
	void addIntParameter(const QString& name, const QString& description, int initialValue = 0, int minValue = -10e7, int maxValue = 10e7, int step = 1);
	void addBoolParameter(const QString& name, const QString& description, bool initialValue = false);
	void addEnumParameter(const QString& name, const QString& description, QStringList& values, int initialValue = 0);
	inline void addEnumParameter(const QString& name, const QString& description, int initialValue = 0)
	    {QStringList values; addEnumParameter(name, description, values, initialValue);}
	
	void removeParameter(QString& name);
	void clearParameterMap();
	
	bool getDoubleValue(const QString& name, double& result) const;
	bool getIntValue(const QString& name, int& result) const;
	bool getBoolValue(const QString& name, bool& result) const;
	bool getEnumValue(const QString& name, int& result) const;
	
	inline double getDoubleValue(const QString& name) const
	    {double result; getDoubleValue(name, result); return result;}
	inline int getIntValue(const QString& name) const
	    {int result; getIntValue(name, result); return result;}
	inline bool getBoolValue(const QString& name) const
	    {bool result; getBoolValue(name, result); return result;}
	inline int getEnumValue(const QString& name) const
	    {int result; getEnumValue(name, result); return result;}
	
    signals:
	void parameterChanged(const QString& name);
	
    public slots:
	void setDoubleValue(const QString& name, double result);
	void setIntValue(const QString& name, int result);
	void setBoolValue(const QString& name, bool result);
	void setEnumValue(const QString& name, int result);
	void insertEnumValue(const QString& name, const QString& value);
	void removeEnumValue(const QString& name, const QString& value);
	void clearEnumParameter(const QString& name);
	
    protected:
	QString m_name;
	QHash<QString, ParameterPair> m_parameterMap;
	QGridLayout *m_layout;
	QSignalMapper m_mapper;


};

#endif

