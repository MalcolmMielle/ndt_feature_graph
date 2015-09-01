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

#ifndef INTERESTPOINTRENDERER_H_
#define INTERESTPOINTRENDERER_H_

#include <gui/AbstractRenderer.h>

#include <geometry/point.h>
#include <vector>
#include <GL/gl.h>
#include <GL/glu.h>

#include <iostream>

class InterestPointRenderer: public AbstractRenderer {
    public:
	InterestPointRenderer(const std::vector<const OrientedPoint2D *> *_points, 
			      const std::vector<double> * _scales); 
	
	InterestPointRenderer(const InterestPointRenderer& _renderer); 
	
	InterestPointRenderer& operator=(const InterestPointRenderer& _renderer); 
	
	virtual ~InterestPointRenderer();

	inline void setDepth(float depth)
	    {m_depth = depth;}
	
	inline void setColors(const std::vector<Color>& _colors)
	    {m_colors = _colors;}
	inline void setColor(unsigned int _index, float _red, float _green, float _blue, float _alpha = 1.0f)
	    {if(_index < m_colors.size()) m_colors[_index] = Color(_red, _green, _blue, _alpha);}
	inline void setScaleFactor(float _size)
	    {m_scaleFactor = _size;}
	inline void setSubdivision(int _around, int _along)
	    {m_subdivision[0] = _around, m_subdivision[1] = _along;}
	
	inline const std::vector<Color>& getColors() const
	    {return m_colors;}
	inline float getScaleFactor() const
	    {return m_scaleFactor;}
	inline void getSubdivision(int& _around, int& _along) const
	    {_around = m_subdivision[0]; _along = m_subdivision[1];}
	
	void setInterestPoints(const std::vector<const OrientedPoint2D *> *_points,
			       const std::vector<double> * _scales); 
	
	inline const std::vector<const OrientedPoint2D *> * getInterestPoints() const
	    {return m_interestPoints;}
	inline const std::vector<double> * getScales() const
	    {return m_scales;}
	
	inline const OrientedPoint2D* getLaserPose() const
	    {return m_laserPose;}
	    
	inline void setLaserPose(const OrientedPoint2D* pose)
	    {m_laserPose = pose;}
	virtual void render();
	
    protected:
	const std::vector<const OrientedPoint2D *> *m_interestPoints;
	const std::vector<double> * m_scales;
	const OrientedPoint2D* m_laserPose;
	std::vector<GLUquadricObj*> m_GLUPoints;
	std::vector<Color> m_colors;
	float m_depth;
	float m_scaleFactor;
	int m_subdivision[2];
};

#endif

