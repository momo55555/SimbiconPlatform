/* 
 * File:   engine_common.h
 * Author: cl
 *
 * Created on June 10, 2011, 8:38 PM
 */

#include "stdafx.h"
#include "math_3d.h"

#ifndef ENGINE_COMMON_H
#define	ENGINE_COMMON_H

#define COLOR_TEXTURE_UNIT              GL_TEXTURE0
#define COLOR_TEXTURE_UNIT_INDEX        0
#define SHADOW_TEXTURE_UNIT             GL_TEXTURE1
#define SHADOW_TEXTURE_UNIT_INDEX       1
#define NORMAL_TEXTURE_UNIT             GL_TEXTURE2
#define NORMAL_TEXTURE_UNIT_INDEX       2
#define RANDOM_TEXTURE_UNIT             GL_TEXTURE3
#define RANDOM_TEXTURE_UNIT_INDEX       3
#define DISPLACEMENT_TEXTURE_UNIT       GL_TEXTURE4
#define DISPLACEMENT_TEXTURE_UNIT_INDEX 4

#define COLOR_RED                   Vector3f(1.0f,  0.267f, 0.267f)
#define COLOR_GREEN                 Vector3f(0.6f,  0.8f,   1.0f)
#define COLOR_BLUE                  Vector3f(0.2f,  0.71f,  0.9f)
#define COLOR_YELLOW                Vector3f(1.0f,  0.73f,  0.2f)
#define COLOR_PURPLE                Vector3f(0.67f, 0.4f,   0.8f)
#define COLOR_BLACK                 Vector3f(0.0f,  0.0f,   0.0f);
#define COLOR_WHITE                 Vector3f(1.0f,  1.0f,   1.0f);

#endif	/* ENGINE_COMMON_H */

