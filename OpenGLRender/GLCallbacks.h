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

#include <gl/glew.h>
#include <GL/freeglut.h>

// Inherent from this class, override the virtual methods.
class CGLCallbacks
{
public:
    // Will be called in each frame.
    virtual void Render() = 0;
    
    // Do some initialization
    virtual bool Initialize() = 0;

    // Will be called at the beginning of each frame.
    // @current: time from Start() was called.
    // @delta: time from last frame.
    // @return: false will exit the main loop.
    virtual bool Update(GLuint current, GLuint delta) = 0;

private:


};