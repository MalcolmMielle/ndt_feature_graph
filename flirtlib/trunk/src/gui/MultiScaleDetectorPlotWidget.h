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

#ifndef MULTISCALEDETECTORPLOTWIDGET_H_
#define MULTISCALEDETECTORPLOTWIDGET_H_

#include <QtGui/QGridLayout>
#include <QtCore/QVector>
#include <QtGui/QColor>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_marker.h>
#include <qwt_symbol.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_layout.h>

class MultiScaleDetectorPlotWidget: public QWidget {
    Q_OBJECT
    public:
	MultiScaleDetectorPlotWidget(QWidget* parent = 0, unsigned int _scales = 0);
	
	virtual ~MultiScaleDetectorPlotWidget();
	
	inline void setScales(unsigned int _scales)
	    {m_scales = _scales; initCurves();}
	
	inline unsigned int getScales() const
	    {return m_scales;}
	    
	inline void setColors(const std::vector<QColor>& _colors)
	    {if(_colors.size() == m_scales) m_colors = _colors;}
	
	inline const std::vector<QColor>& getColors() const
	    {return m_colors;}
	
    public slots:
	virtual void setSmoothData(QVector<double>& _dataX, QVector<double>& _dataY, unsigned int _scale);
	virtual void setSmoothData(QVector< QVector<double> >& _dataX, QVector< QVector<double> >& _dataY);

	virtual void setDifferentialData(QVector<double>& _dataX, QVector<double>& _dataY, unsigned int _scale);
	virtual void setDifferentialData(QVector< QVector<double> >& _dataX, QVector< QVector<double> >& _dataY);
	
	virtual void setSmoothMarker(QVector<double>& _dataX, QVector<double>& _dataY, unsigned int _scale);
	virtual void setSmoothMarker(QVector< QVector<double> >& _dataX, QVector< QVector<double> >& _dataY);

	virtual void setDifferentialMarker(QVector<double>& _dataX, QVector<double>& _dataY, unsigned int _scale);
	virtual void setDifferentialMarker(QVector< QVector<double> >& _dataX, QVector< QVector<double> >& _dataY);
	
	virtual void replot();
	
    protected:
	void buildGui();
	void initCurves();
	
	void setData(QVector<QwtPlotCurve *>& _curve, QVector<double>& _dataX, QVector<double>& _dataY, unsigned int _scale);
	void setData(QVector<QwtPlotCurve *>& _curve, QVector< QVector<double> >& _dataX, QVector< QVector<double> >& _dataY);

	QGridLayout *m_plotLayout;
	QwtPlot *m_smooth;
	QwtPlot *m_derivative;
	QVector<QwtPlotCurve *> m_smoothCurve;
	QVector<QwtPlotCurve *> m_smoothMarker;
	QVector<QwtPlotCurve *> m_diffCurve;
	QVector<QwtPlotCurve *> m_diffMarker;
	QwtPlotZoomer* m_smoothZoomer;
	QwtPlotZoomer* m_diffZoomer;
	unsigned int m_scales;
	
	std::vector<QColor> m_colors;
};

#endif

