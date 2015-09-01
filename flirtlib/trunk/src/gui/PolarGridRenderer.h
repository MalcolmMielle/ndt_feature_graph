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

#ifndef POLARGRIDRENDERER_H_
#define POLARGRIDRENDERER_H_

#include <gui/AbstractRenderer.h>

#include <geometry/point.h>
#include <vector>
#include <GL/gl.h>
#include <GL/glu.h>
#include <iostream>

class PolarGridRenderer: public AbstractRenderer {
    public:
	PolarGridRenderer(const std::vector< std::vector<double> > *grid, const std::vector<double> *phiEdges, const std::vector<double> *rhoEdges); 
	
	PolarGridRenderer(const PolarGridRenderer& _renderer); 
	
	PolarGridRenderer& operator=(const PolarGridRenderer& _renderer); 
	
	virtual ~PolarGridRenderer();

	inline void setDepth(float depth)
	    {m_depth = depth;}
	
	inline void setColor(float _red, float _green, float _blue, float _alpha = 1.0f)
	    {m_color = RGB2HSL(Color(_red, _green, _blue, _alpha));}
	inline void setColor(const Color& color)
	    {m_color = RGB2HSL(color);}
	inline void setSubdivision(int _around, int _along)
	    {m_subdivision[0] = _around, m_subdivision[1] = _along;}
	
/*	inline void getColor(float& _red, float& _green, float& _blue, float& _alpha) const
	    {_red = m_color.red(); _green = m_color.green(); _blue = m_color.blue(); _alpha = m_color.alpha(); }*/
	inline const HSLColor& getHSLColor() const
	    {return m_color; }
	inline const Color getColor() const
	    {return HSL2RGB(m_color); }
	inline void getSubdivision(int& _around, int& _along) const
	    {_around = m_subdivision[0]; _along = m_subdivision[1];}
	
	void setGrid(const std::vector< std::vector<double> > *grid, const std::vector<double> *phiEdges, const std::vector<double> *rhoEdges);
	
	inline const std::vector< std::vector<double> > * getGrid() const
	    {return m_grid;}
	
	inline const std::vector<double> * getPhiEdges() const
	    {return m_phiEdges;}
	    
	inline const std::vector<double> * getRhoEdges() const
	    {return m_rhoEdges;}

	inline const OrientedPoint2D* getPosition() const
	    {return m_position;}
	    
	inline void setPosition(const OrientedPoint2D* position)
	    {m_position = position;}
	
	    
	virtual void render();
	
    protected:
	const std::vector< std::vector<double> >* m_grid;
	const std::vector<double> * m_phiEdges;
	const std::vector<double> * m_rhoEdges;
	std::vector<GLUquadricObj*> m_GLUGrids;
	std::vector<GLUquadricObj*> m_GLUSectors;
	const OrientedPoint2D * m_position;
	double m_maxValue;
	HSLColor m_color;
	float m_depth;
	int m_subdivision[2];
};

#endif

