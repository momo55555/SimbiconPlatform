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

#include <stdio.h>
#ifdef WIN32
#define _USE_MATH_DEFINES 
#include <cmath>
#else
#include <math.h>
#endif

#include "GLUtil.h"
#include <assimp/vector3.h>
#include <assimp/matrix3x3.h>
#include <assimp/matrix4x4.h>

#include <iostream>
#include <limits>

#define M_PI 3.14159265358979323846

const double eps = 1e-6;

#define ToRadian(x) (float)(((x) * M_PI / 180.0f))
#define ToDegree(x) (float)(((x) * 180.0f / M_PI))

float RandomFloat();

using namespace std;

namespace Math3D
{
    struct Vector2i
    {
        int x;
        int y;
    };

    struct Vector2f
    {
        float x;
        float y;

        Vector2f()
        {
        }

        Vector2f(float _x, float _y)
        {
            x = _x;
            y = _y;
        }
    };


    struct Vector3f
    {
        float x;
        float y;
        float z;

        Vector3f()
        {
        }

        Vector3f(float _x, float _y, float _z)
        {
            x = _x;
            y = _y;
            z = _z;
        }

        Vector3f& operator+=(const Vector3f& r)
        {
            x += r.x;
            y += r.y;
            z += r.z;

            return *this;
        }

        Vector3f& operator-=(const Vector3f& r)
        {
            x -= r.x;
            y -= r.y;
            z -= r.z;

            return *this;
        }

        Vector3f& operator*=(float f)
        {
            x *= f;
            y *= f;
            z *= f;

            return *this;
        }

        Vector3f Cross(const Vector3f& v) const;

        Vector3f& Normalize();

        void Rotate(float Angle, const Vector3f& Axis);

        void Print() const
        {
            printf("(%.02f, %.02f, %.02f", x, y, z);
        }
    };


    inline Vector3f operator+(const Vector3f& l, const Vector3f& r)
    {
        Vector3f Ret(l.x + r.x,
                     l.y + r.y,
                     l.z + r.z);

        return Ret;
    }

    inline Vector3f operator-(const Vector3f& l, const Vector3f& r)
    {
        Vector3f Ret(l.x - r.x,
                     l.y - r.y,
                     l.z - r.z);

        return Ret;
    }

    inline Vector3f operator*(const Vector3f& l, float f)
    {
        Vector3f Ret(l.x * f,
                     l.y * f,
                     l.z * f);

        return Ret;
    }

    struct PersProjInfo
    {
        float FOV;
        float Width; 
        float Height;
        float zNear;
        float zFar;
    };

    class Matrix4f
    {
    public:
        float m[4][4];

        Matrix4f()
        {        
        }

        // constructor from Assimp matrix
        Matrix4f(const aiMatrix4x4& AssimpMatrix)
        {
            m[0][0] = AssimpMatrix.a1; m[0][1] = AssimpMatrix.a2; m[0][2] = AssimpMatrix.a3; m[0][3] = AssimpMatrix.a4;
            m[1][0] = AssimpMatrix.b1; m[1][1] = AssimpMatrix.b2; m[1][2] = AssimpMatrix.b3; m[1][3] = AssimpMatrix.b4;
            m[2][0] = AssimpMatrix.c1; m[2][1] = AssimpMatrix.c2; m[2][2] = AssimpMatrix.c3; m[2][3] = AssimpMatrix.c4;
            m[3][0] = AssimpMatrix.d1; m[3][1] = AssimpMatrix.d2; m[3][2] = AssimpMatrix.d3; m[3][3] = AssimpMatrix.d4;
        }

        Matrix4f(const aiMatrix3x3& AssimpMatrix)
        {
            m[0][0] = AssimpMatrix.a1; m[0][1] = AssimpMatrix.a2; m[0][2] = AssimpMatrix.a3; m[0][3] = 0.0f;
            m[1][0] = AssimpMatrix.b1; m[1][1] = AssimpMatrix.b2; m[1][2] = AssimpMatrix.b3; m[1][3] = 0.0f;
            m[2][0] = AssimpMatrix.c1; m[2][1] = AssimpMatrix.c2; m[2][2] = AssimpMatrix.c3; m[2][3] = 0.0f;
            m[3][0] = 0.0f           ; m[3][1] = 0.0f           ; m[3][2] = 0.0f           ; m[3][3] = 1.0f;
        }

        inline void InitIdentity()
        {
            m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f; m[0][3] = 0.0f;
            m[1][0] = 0.0f; m[1][1] = 1.0f; m[1][2] = 0.0f; m[1][3] = 0.0f;
            m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = 1.0f; m[2][3] = 0.0f;
            m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
        }

        void SetZero()
        {
            ZERO_MEM(m);
        }

        Matrix4f Transpose() const
        {
            Matrix4f n;

            for (unsigned int i = 0 ; i < 4 ; i++) {
                for (unsigned int j = 0 ; j < 4 ; j++) {
                    n.m[i][j] = m[j][i];
                }
            }

            return n;
        }

        float Determinant() const;

        Matrix4f& Inverse();

        inline Matrix4f operator*(const Matrix4f& Right) const
        {
            Matrix4f Ret;

            for (unsigned int i = 0 ; i < 4 ; i++) 
            {
                for (unsigned int j = 0 ; j < 4 ; j++) 
                {
                    Ret.m[i][j] = m[i][0] * Right.m[0][j] +
                                  m[i][1] * Right.m[1][j] +
                                  m[i][2] * Right.m[2][j] +
                                  m[i][3] * Right.m[3][j];
                }
            }

            return Ret;
        }
    
        void Print()
        {
            for (int i = 0 ; i < 4 ; i++) {
                printf("%f %f %f %f\n", m[i][0], m[i][1], m[i][2], m[i][3]);
            }       
        }

        void InitScaleTransform(float ScaleX, float ScaleY, float ScaleZ);
        void InitRotateTransform(float RotateX, float RotateY, float RotateZ);
        void InitTranslationTransform(float x, float y, float z);
        void InitCameraTransform(const Vector3f& Target, const Vector3f& Up);
        void InitPersProjTransform(const PersProjInfo& p);
        void InitOrthoTransform(float Left, float Right, float Bottom, float Top, float Near, float Far);
    };


    struct Quaternions
    {
        float x, y, z, w;

        Quaternions(float _x = 0.0f, float _y = 0.0f, float _z = 1.0f, float _w = 0.0f);

        void Normalize();

        Quaternions Conjugate();

        Quaternions operator*(const Quaternions& r);

        Quaternions operator*(const Vector3f& v);

        double length()
        {
            return sqrtf(x * x + y * y + z * z + w * w);
        }
    };

    inline void Quat2Matrix(Quaternions& q, Matrix4f& m)
    {
        double length2 = q.length();
        if(abs(length2) <= eps)
        {
            m.m[0][0] = 0.0; m.m[1][0] = 0.0; m.m[2][0] = 0.0;
            m.m[0][1] = 0.0; m.m[1][1] = 0.0; m.m[2][1] = 0.0;
            m.m[0][2] = 0.0; m.m[1][2] = 0.0; m.m[2][2] = 0.0;
        }
        else
        {
            double rlength2;
            // normalize quat if required.
            // We can avoid the expensive sqrt in this case since all 'coefficients' below are products of two q components.
            // That is a square of a square root, so it is possible to avoid that
            if (length2 != 1.0)
            {
                rlength2 = 2.0 / length2;
            }
            else
            {
                rlength2 = 2.0;
            }

            // Source: Gamasutra, Rotating Objects Using Quaternions
            //
            //http://www.gamasutra.com/features/19980703/quaternions_01.htm

            double wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;

            // calculate coefficients
            x2 = rlength2*q.x;
            y2 = rlength2*q.y;
            z2 = rlength2*q.z;

            xx = q.x * x2;
            xy = q.x * y2;
            xz = q.x * z2;

            yy = q.y * y2;
            yz = q.y * z2;
            zz = q.z * z2;

            wx = q.w * x2;
            wy = q.w * y2;
            wz = q.w * z2;

            // Note. Gamasutra gets the matrix assignments inverted, resulting
            // in left-handed rotations, which is contrary to OpenGL and OSG's 
            // methodology. The matrix assignment has been altered in the next
            // few lines of code to do the right thing.
            // Don Burns - Oct 13, 2001
            m.m[0][0] = 1.0 - (yy + zz);
            m.m[1][0] = xy - wz;
            m.m[2][0] = xz + wy;


            m.m[0][1] = xy + wz;
            m.m[1][1] = 1.0 - (xx + zz);
            m.m[2][1] = yz - wx;

            m.m[0][2] = xz - wy;
            m.m[1][2] = yz + wx;
            m.m[2][2] = 1.0 - (xx + yy);
        }
        m.m[3][0] = m.m[3][1] = m.m[3][2] = m.m[0][3] = m.m[1][3] = m.m[2][3] = 0;
        m.m[3][3] = 1.0;
        m.Transpose();
    }

    inline void Matrix4f2Quat(Matrix4f& m, Quaternions& q)
    {
        float s;
        float tq[4];
        int i, j;

        m.Transpose();

        // Use tq to store the largest trace
        tq[0] = 1 + m.m[0][0] + m.m[1][1] + m.m[2][2];
        tq[1] = 1 + m.m[0][0] - m.m[1][1] - m.m[2][2];
        tq[2] = 1 - m.m[0][0] + m.m[1][1] - m.m[2][2];
        tq[3] = 1 - m.m[0][0] - m.m[1][1] + m.m[2][2];

        // Find the maximum (could also use stacked if's later)
        j = 0;
        for(i = 1; i < 4; ++i) j = (tq[i] > tq[j])? i : j;

        // check the diagonal
        if (j == 0)
        {
            /* perform instant calculation */
            q.w = tq[0];
            q.x = m.m[1][2] - m.m[2][1];
            q.y = m.m[2][0] - m.m[0][2];
            q.z = m.m[0][1] - m.m[1][0];
        }
        else if(j == 1)
        {
            q.w = m.m[1][2] - m.m[2][1];
            q.x = tq[1];
            q.y = m.m[0][1] + m.m[1][0];
            q.z = m.m[2][0] + m.m[0][2];
        }
        else if(j == 2)
        {
            q.w = m.m[2][0] - m.m[0][2];
            q.x = m.m[0][1] + m.m[1][0];
            q.y = tq[2];
            q.z = m.m[1][2] + m.m[2][1];
        }
        else /* if (j==3) */
        {
            q.w = m.m[0][1] - m.m[1][0];
            q.x = m.m[2][0] + m.m[0][2];
            q.y = m.m[1][2] + m.m[2][1];
            q.z = tq[3];
        }

        s = sqrt(0.25 / tq[j]);
        q.w *= s;
        q.x *= s;
        q.y *= s;
        q.z *= s;
    }
}