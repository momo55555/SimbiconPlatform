/*
        Copyright 2014 Rudy Snow

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "Technique.h"
#include "math_3d.h"

using namespace Math3D;

class WireFrameTechnique : public Technique 
{
public:

    WireFrameTechnique();

    virtual ~WireFrameTechnique() {}
    
    virtual bool Init();

    void SetColor(const Vector3f& wireColor);
    void SetWVP(const Matrix4f& WVP);
    void SetWorldMatrix(const Matrix4f& WVP);

private:

    GLuint m_Color;
    GLuint m_WVPLocation;
    GLuint m_WorldMatrixLocation;
};
