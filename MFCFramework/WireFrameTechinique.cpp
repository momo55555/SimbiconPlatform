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

#include "stdafx.h"

#include <limits.h>
#include <string.h>

#include "math_3d.h"
#include "GLUtil.h"
#include "WireFrameTechnique.h"

//#define CHECK_BVH_UNIFORM_STATUS

WireFrameTechnique::WireFrameTechnique()
{   
}

bool WireFrameTechnique::Init()
{
    if (!Technique::Init()) {
        return false;
    }

    if (!AddShader(GL_VERTEX_SHADER, "../Data/GLEffects/WireFrame.vp")) 
    {
        return false;
    }

    if (!AddShader(GL_FRAGMENT_SHADER, "../Data/GLEffects/WireFrame.fp")) 
    {
        return false;
    }

    if (!Finalize()) 
    {
        return false;
    }

    m_Color = GetUniformLocation("gColor");
    m_WVPLocation = GetUniformLocation("gWVP");
    m_WorldMatrixLocation = GetUniformLocation("gWorld");

    return true;
}

void WireFrameTechnique::SetColor(const Vector3f& WireColor)
{
    glUniform3f(m_Color, WireColor.x, WireColor.y, WireColor.z);
}


void WireFrameTechnique::SetWVP(const Matrix4f& WVP)
{
    glUniformMatrix4fv(m_WVPLocation, 1, GL_TRUE, (const GLfloat*)WVP.m);    
}

void WireFrameTechnique::SetWorldMatrix(const Matrix4f& WorldInverse)
{
    glUniformMatrix4fv(m_WorldMatrixLocation, 1, GL_TRUE, (const GLfloat*)WorldInverse.m);
}