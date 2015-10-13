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

#include <GL/glew.h>
#include <GL/freeglut.h>

#include <list>

// Inherent from this class to manipulate shaders
class Technique
{
public:
    Technique();

    virtual ~Technique();

    virtual bool Init();

    void Enable();

    bool AddShader(GLenum ShaderType, const char* pFilename);

    bool Finalize();

    GLint GetUniformLocation(const char* pUniformName);

    GLint GetProgramParam(GLint param);

    GLuint m_shaderProg;

private:
    typedef std::list<GLuint> ShaderObjList;
    ShaderObjList m_shaderObjList;
};
