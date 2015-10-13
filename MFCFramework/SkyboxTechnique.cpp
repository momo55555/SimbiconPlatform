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

#include "SkyboxTechnique.h"
#include "GLUtil.h"


SkyboxTechnique::SkyboxTechnique()
{   
}

bool SkyboxTechnique::Init()
{
    if (!Technique::Init()) 
    {
        return false;
    }

    if (!AddShader(GL_VERTEX_SHADER, "../Data/GLEffects/Skybox.vp")) 
    {
        return false;
    }

    if (!AddShader(GL_FRAGMENT_SHADER, "../Data/GLEffects/Skybox.fp")) 
    {
        return false;
    }

    if (!Finalize()) 
    {
        return false;
    }

    m_WVPLocation = GetUniformLocation("gWVP");
    m_textureLocation = GetUniformLocation("gCubemapTexture");
 
    if (m_WVPLocation == INVALID_UNIFORM_LOCATION ||
        m_textureLocation == INVALID_UNIFORM_LOCATION) 
    {
        return false;
    }

    return true;
}


void SkyboxTechnique::SetWVP(const Matrix4f& WVP)
{
    glUniformMatrix4fv(m_WVPLocation, 1, GL_TRUE, (const GLfloat*)WVP.m);    
}


void SkyboxTechnique::SetTextureUnit(unsigned int TextureUnit)
{
    glUniform1i(m_textureLocation, TextureUnit);
}
