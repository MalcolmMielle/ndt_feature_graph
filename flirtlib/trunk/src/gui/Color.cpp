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

#include "Color.h"

Color::Color(float r, float g, float b, float a):m_red(r),m_green(g),m_blue(b),m_alpha(a) { }

HSLColor::HSLColor(float h, float s, float l, float a):m_hue(h),m_saturation(s),m_lightness(l),m_alpha(a) { }

Color HSLColor::toColor() const{
    return HSL2RGB(*this);
}

void HSLColor::fromColor(const Color& color){
    *this = RGB2HSL(color);
}

HSLColor RGB2HSL(const Color& color){
    float min, max, hue, saturation, lightness, alpha;
    
    min = (color.red() < color.green()) ? color.red() : color.green();
    min = (min < color.blue()) ? min : color.blue();
    
    max = (color.red() > color.green()) ? color.red() : color.green();
    max = (max > color.blue()) ? max : color.blue();
    
    lightness = 0.5*(min + max);
    
    if(max == min){
	hue = 0.f;
	saturation = 0.f;
    } else {
	saturation = lightness < 0.5 ? (max - min)/(max + min) : (max - min)/(2.f - max - min);
	
	if(max == color.red()) {
	    hue = (color.green() - color.blue())/(max - min);
	} else if(max == color.green()){
	    hue = 2.f + (color.blue() - color.red())/(max - min);
	} else {
	    hue = 4.f + (color.red() - color.green())/(max - min);
	}
	hue = hue < 0 ? hue + 6.f : hue;
    }
    
    alpha = color.alpha();
    return HSLColor(hue, saturation, lightness, alpha);
}

Color HSL2RGB(const HSLColor& color) {
    float q, p, h, tc[3], col[3];
    q = color.lightness() < 0.5 ? color.lightness() * (1.f + color.saturation()) : color.lightness() + color.saturation() - color.lightness() * color.saturation();
    p = 2 * color.lightness() - q;
    h = color.hue() / 6.f;
    
    tc[0] = h  + 1.f/3.f;
    tc[1] = h;
    tc[2] = h - 1.f/3.f;
    
    for(unsigned int i = 0; i < 3; i++){
	tc[i] = tc[i] < 0 ? tc[i] + 1.f: tc[i];
	tc[i] = tc[i] > 1 ? tc[i] - 1.f: tc[i];
	if(6.f * tc[i] < 1.f){
	    col[i] = p + ((q - p) * 6.f * tc[i]);
	} else if(2.f * tc[i] < 1.f){
	    col[i] = q;
	} else if(3.f * tc[i] < 2.f){
	    col[i] = p + ((q - p) * 6.f * (2.f/3.f - tc[i]));
	} else {
	    col[i] = p;
	}
    }
    
    return Color(col[0], col[1], col[2], color.alpha());
}

std::ostream & operator<< (std::ostream& out, const Color& color){
    out << "RGB(" << color.red() << ", " << color.green() << ", " << color.blue() << ")";
    return out;
}

std::ostream & operator<< (std::ostream& out, const HSLColor& color){
    out << "HSL(" << color.hue() << ", " << color.saturation() << ", " << color.lightness() << ")";
    return out;
}
