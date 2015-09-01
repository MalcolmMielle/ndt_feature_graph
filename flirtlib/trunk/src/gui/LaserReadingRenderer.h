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

#ifndef LASERREADINGRENDERER_H_
#define LASERREADINGRENDERER_H_

#include <gui/AbstractRenderer.h>

#include <geometry/point.h>
#include <vector>
#include <GL/gl.h>
#include <GL/glu.h>
#include <iostream>

class LaserReadingRenderer: public AbstractRenderer {
    public:
	LaserReadingRenderer(const std::vector<Point2D> *_points, const std::vector<double>& normals = std::vector<double>(0)); 
	
	LaserReadingRenderer(const LaserReadingRenderer& _renderer); 
	
	LaserReadingRenderer& operator=(const LaserReadingRenderer& _renderer);
	
	inline void setDepth(float depth)
	    {m_depth = depth;m_listValid=false;}
	
	virtual ~LaserReadingRenderer();

	inline void setColor(float _red, float _green, float _blue, float _alpha = 1.0f)
	    {m_color = Color(_red, _green, _blue, _alpha);m_listValid=false;}
	inline void setColor(const Color& _color)
	    {m_color = _color;m_listValid=false;}
	inline void setSize(float _size)
	    {m_pointSize = _size;m_listValid=false;}
	inline void setSubdivision(int _around, int _along)
	    {m_subdivision[0] = _around, m_subdivision[1] = _along;m_listValid=false;}
	
	inline void getColor(float& _red, float& _green, float& _blue, float& _alpha) const
	    {_red = m_color.red(); _green = m_color.green(); _blue = m_color.blue(); _alpha = m_color.alpha(); }
	inline const Color& getColor() const
	    {return m_color; }
	inline float getSize() const
	    {return m_pointSize;}
	inline void getSubdivision(int& _around, int& _along) const
	    {_around = m_subdivision[0]; _along = m_subdivision[1];}
	
	void setLaserPoints(const std::vector<Point2D> *_points, const std::vector<double>& normals = std::vector<double>(0));
	
	inline const std::vector<Point2D> * getLaserPoints() const
	    {return m_laserPoints;}
	
	inline const std::vector<double> & getLaserNormals() const
	    {return m_laserNormals;}
	    
	inline const OrientedPoint2D* getLaserPose() const
	    {return m_laserPose;}
	    
	inline void setLaserPose(const OrientedPoint2D* pose)
	    {m_laserPose = pose;m_listValid=false;}
	
	virtual void render();
	
    protected:
	void makeList();
	GLuint m_list;
	bool m_listValid;
	const std::vector<Point2D> * m_laserPoints;
	std::vector<double> m_laserNormals;
	std::vector<GLUquadricObj*> m_GLUPoints;
	const OrientedPoint2D* m_laserPose;
	Color m_color;
	float m_pointSize;
	float m_depth;
	int m_subdivision[2];
};

#endif

