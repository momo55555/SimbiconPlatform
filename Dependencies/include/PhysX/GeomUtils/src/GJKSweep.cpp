// This code contains NVIDIA Confidential Information and is disclosed to you 
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and 
// any modifications thereto. Any use, reproduction, disclosure, or 
// distribution of this software and related documentation without an express 
// license agreement from NVIDIA Corporation is strictly prohibited.
// 
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2008-2011 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


/*-------------------------------------------------------*\
|
|  The source code within this file is taken from the 
|  the Bullet Library v. 2.71 by Erwin Coumans.  
|  It is reused under license as posted below.
|
|  This version has been altered to integrate with the
|  PhysX SDK.
|
\*-------------------------------------------------------*/

/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
/*
This means Bullet can freely be used in any software, including commercial and console software. A Playstation 3 optimized version is available through Sony.
*/

#include "GJKSweep.h"
#include "PsMathUtils.h"
#include "PxBounds3.h"
#include "PsFoundation.h"
#include "GuGJKObjectSupport.h" // for GJKSphereSupport

#include <stdio.h>	//temp debug

//#define DEBUG_RENDER_CCD 1
#if DEBUG_RENDER_CCD
#include "CmRenderOutput.h"
#include "CmRenderBuffer.h"
#include "PxPhysics.h"
#include "PxScene.h"
#endif

#define MAX_GJK_SWEEP_ITERATIONS 40
#define PX_GJK_REL_ERROR2 PxReal(1.0e-6)		//must be above the machine epsilon
#define PX_VORONOI_SIMPLEX_MAX_VERTS 5

using namespace physx;

struct UsageBitfield
{
	UsageBitfield()
	{
		reset();
	}

	void reset()
	{
		usedVertexA = false;
		usedVertexB = false;
		usedVertexC = false;
		usedVertexD = false;
	}
	unsigned short usedVertexA	: 1;
	unsigned short usedVertexB	: 1;
	unsigned short usedVertexC	: 1;
	unsigned short usedVertexD	: 1;
	unsigned short unused1		: 1;
	unsigned short unused2		: 1;
	unsigned short unused3		: 1;
	unsigned short unused4		: 1;
};

struct SubSimplexClosestResult
{
	PxVec3			m_closestPointOnSimplex;

	//MASK for m_usedVertices
	//stores the simplex vertex-usage, using the MASK, 
	// if m_usedVertices & MASK then the related vertex is used
	UsageBitfield	m_usedVertices;
	PxReal			m_barycentricCoords[4];
	bool			m_degenerate;

	void	reset()
	{
		m_degenerate = false;
		setBarycentricCoordinates();
		m_usedVertices.reset();
	}
	bool	isValid()
	{
		bool valid = (m_barycentricCoords[0] >= PxReal(0.)) &&
			(m_barycentricCoords[1] >= PxReal(0.)) &&
			(m_barycentricCoords[2] >= PxReal(0.)) &&
			(m_barycentricCoords[3] >= PxReal(0.));


		return valid;
	}
	void	setBarycentricCoordinates(PxReal a=PxReal(0.),PxReal b=PxReal(0.),PxReal c=PxReal(0.),PxReal d=PxReal(0.))
	{
		m_barycentricCoords[0] = a;
		m_barycentricCoords[1] = b;
		m_barycentricCoords[2] = c;
		m_barycentricCoords[3] = d;
	}

};

// VoronoiSimplexSolver is an implementation of the closest point distance algorithm from a 1-4 points simplex to the origin.
// Can be used with GJK, as an alternative to Johnson distance algorithm.

class VoronoiSimplexSolver
{
public:

	int	m_numVertices;

	PxVec3	m_simplexVectorW[PX_VORONOI_SIMPLEX_MAX_VERTS];
	PxVec3	m_simplexPointsP[PX_VORONOI_SIMPLEX_MAX_VERTS];
	PxVec3	m_simplexPointsQ[PX_VORONOI_SIMPLEX_MAX_VERTS];

	

	PxVec3	m_cachedP1;
	PxVec3	m_cachedP2;
	PxVec3	m_cachedV;
	PxVec3	m_lastW;
	bool		m_cachedValidClosest;

	SubSimplexClosestResult m_cachedBC;

	bool	m_needsUpdate;
	
	void	removeVertex(int index);
	void	reduceVertices (const UsageBitfield& usedVerts);
	bool	updateClosestVectorAndPoints();

	bool	closestPtPointTetrahedron(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, SubSimplexClosestResult& finalResult);
	int		pointOutsideOfPlane(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d);
	bool	closestPtPointTriangle(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c,SubSimplexClosestResult& result);

public:

	 void reset();

	PX_FORCE_INLINE	void	addVertex(const PxVec3& w, const PxVec3& p, const PxVec3& q)
	{
		m_lastW = w;
		m_needsUpdate = true;

		m_simplexVectorW[m_numVertices] = w;
		m_simplexPointsP[m_numVertices] = p;
		m_simplexPointsQ[m_numVertices] = q;

		m_numVertices++;
	}

	//return/calculate the closest vertex
	PX_FORCE_INLINE bool	closest(PxVec3& v)
	{
		bool succes = updateClosestVectorAndPoints();
		v = m_cachedV;
		return succes;
	}

	PxReal maxVertex()	const;

	PX_FORCE_INLINE bool fullSimplex() const
	{
		return (m_numVertices == 4);
	}

	int getSimplex(PxVec3* pBuf, PxVec3* qBuf, PxVec3* yBuf) const;

	bool inSimplex(const PxVec3& w)	const;
	
	PX_FORCE_INLINE	void	backup_closest(PxVec3& v) 
	{
		v = m_cachedV;
	}

/*	PX_FORCE_INLINE	bool	emptySimplex() const 
	{
		return (m_numVertices == 0);
	}*/

	PX_FORCE_INLINE	void	compute_points(PxVec3& p1, PxVec3& p2) 
	{
		updateClosestVectorAndPoints();
		p1 = m_cachedP1;
		p2 = m_cachedP2;
	}

	PX_FORCE_INLINE int numVertices() const 
	{
		return m_numVertices;
	}
};

#define VERTA  0
#define VERTB  1
#define VERTC  2
#define VERTD  3
#define CATCH_DEGENERATE_TETRAHEDRON 1

void VoronoiSimplexSolver::removeVertex(int index)
{
	
	PX_ASSERT(m_numVertices>0);
	m_numVertices--;
	m_simplexVectorW[index] = m_simplexVectorW[m_numVertices];
	m_simplexPointsP[index] = m_simplexPointsP[m_numVertices];
	m_simplexPointsQ[index] = m_simplexPointsQ[m_numVertices];
}

void VoronoiSimplexSolver::reduceVertices(const UsageBitfield& usedVerts)
{
	if ((m_numVertices >= 4) && (!usedVerts.usedVertexD))
		removeVertex(3);

	if ((m_numVertices >= 3) && (!usedVerts.usedVertexC))
		removeVertex(2);

	if ((m_numVertices >= 2) && (!usedVerts.usedVertexB))
		removeVertex(1);
	
	if ((m_numVertices >= 1) && (!usedVerts.usedVertexA))
		removeVertex(0);
}

//clear the simplex, remove all the vertices
void VoronoiSimplexSolver::reset()
{
	m_cachedValidClosest = false;
	m_numVertices = 0;
	m_needsUpdate = true;
	m_lastW = PxVec3(PxReal(1e30),PxReal(1e30),PxReal(1e30));
	m_cachedBC.reset();
}

bool VoronoiSimplexSolver::updateClosestVectorAndPoints()
{
	if (m_needsUpdate)
	{
		m_cachedBC.reset();

		m_needsUpdate = false;

		switch (m_numVertices)
		{
		case 0:
				m_cachedValidClosest = false;
				break;
		case 1:
			{
				m_cachedP1 = m_simplexPointsP[0];
				m_cachedP2 = m_simplexPointsQ[0];
				m_cachedV = m_cachedP1-m_cachedP2;
				m_cachedBC.reset();
				m_cachedBC.setBarycentricCoordinates(PxReal(1.),PxReal(0.),PxReal(0.),PxReal(0.));
				m_cachedValidClosest = m_cachedBC.isValid();
				break;
			};
		case 2:
			{
			//closest point origin from line segment
					const PxVec3& from = m_simplexVectorW[0];
					const PxVec3& to = m_simplexVectorW[1];
					PxVec3 nearest;

					PxVec3 p (PxReal(0.),PxReal(0.),PxReal(0.));
					PxVec3 diff = p - from;
					PxVec3 v = to - from;
					PxReal t = v.dot(diff);
					
					if (t > 0) {
						PxReal dotVV = v.dot(v);
						if (t < dotVV) {
							t /= dotVV;
							diff -= t*v;
							m_cachedBC.m_usedVertices.usedVertexA = true;
							m_cachedBC.m_usedVertices.usedVertexB = true;
						} else {
							t = 1;
							diff -= v;
							//reduce to 1 point
							m_cachedBC.m_usedVertices.usedVertexB = true;
						}
					} else
					{
						t = 0;
						//reduce to 1 point
						m_cachedBC.m_usedVertices.usedVertexA = true;
					}
					m_cachedBC.setBarycentricCoordinates(1-t,t);
					nearest = from + t*v;

					m_cachedP1 = m_simplexPointsP[0] + t * (m_simplexPointsP[1] - m_simplexPointsP[0]);
					m_cachedP2 = m_simplexPointsQ[0] + t * (m_simplexPointsQ[1] - m_simplexPointsQ[0]);
					m_cachedV = m_cachedP1 - m_cachedP2;
					
					reduceVertices(m_cachedBC.m_usedVertices);

					m_cachedValidClosest = m_cachedBC.isValid();
					break;
			}
		case 3: 
			{ 
				//closest point origin from triangle 
				PxVec3 p (PxReal(0.),PxReal(0.),PxReal(0.)); 

				const PxVec3& a = m_simplexVectorW[0]; 
				const PxVec3& b = m_simplexVectorW[1]; 
				const PxVec3& c = m_simplexVectorW[2]; 

				closestPtPointTriangle(p,a,b,c,m_cachedBC); 


				m_cachedP1 = m_simplexPointsP[0] * m_cachedBC.m_barycentricCoords[0] + 
				m_simplexPointsP[1] * m_cachedBC.m_barycentricCoords[1] + 
				m_simplexPointsP[2] * m_cachedBC.m_barycentricCoords[2]; 

				m_cachedP2 = m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoords[0] + 
				m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoords[1] + 
				m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoords[2]; 

				m_cachedV = m_cachedP1-m_cachedP2; 

				reduceVertices (m_cachedBC.m_usedVertices); 
				m_cachedValidClosest = m_cachedBC.isValid(); 

				break; 
			}
		case 4:
			{

				
				PxVec3 p (PxReal(0.),PxReal(0.),PxReal(0.));
				
				const PxVec3& a = m_simplexVectorW[0];
				const PxVec3& b = m_simplexVectorW[1];
				const PxVec3& c = m_simplexVectorW[2];
				const PxVec3& d = m_simplexVectorW[3];

				bool hasSeperation = closestPtPointTetrahedron(p,a,b,c,d,m_cachedBC);

				if (hasSeperation)
				{

					m_cachedP1 = m_simplexPointsP[0] * m_cachedBC.m_barycentricCoords[0] +
						m_simplexPointsP[1] * m_cachedBC.m_barycentricCoords[1] +
						m_simplexPointsP[2] * m_cachedBC.m_barycentricCoords[2] +
						m_simplexPointsP[3] * m_cachedBC.m_barycentricCoords[3];

					m_cachedP2 = m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoords[0] +
						m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoords[1] +
						m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoords[2] +
						m_simplexPointsQ[3] * m_cachedBC.m_barycentricCoords[3];

					m_cachedV = m_cachedP1-m_cachedP2;
					reduceVertices (m_cachedBC.m_usedVertices);
				} else
				{
					if (m_cachedBC.m_degenerate)
					{
						m_cachedValidClosest = false;
					} else
					{
						m_cachedValidClosest = true;
						//degenerate case == false, penetration = true + zero
						m_cachedV = PxVec3(PxReal(0.),PxReal(0.),PxReal(0.));
					}
					break;
				}

				m_cachedValidClosest = m_cachedBC.isValid();

				//closest point origin from tetrahedron
				break;
			}
		default:
			{
				m_cachedValidClosest = false;
			}
		};
	}

	return m_cachedValidClosest;
}

PxReal VoronoiSimplexSolver::maxVertex() const
{
	int numverts = m_numVertices;
	PxReal maxV = PxReal(0.);
	for(int i=0;i<numverts;i++)
	{
		const PxReal curLen2 = m_simplexVectorW[i].magnitudeSquared();
		if (maxV < curLen2)
			maxV = curLen2;
	}
	return maxV;
}

	//return the current simplex
int VoronoiSimplexSolver::getSimplex(PxVec3* pBuf, PxVec3* qBuf, PxVec3* yBuf) const
{
	for(int i=0;i<m_numVertices;i++)
	{
		yBuf[i] = m_simplexVectorW[i];
		pBuf[i] = m_simplexPointsP[i];
		qBuf[i] = m_simplexPointsQ[i];
	}
	return m_numVertices;
}

bool VoronoiSimplexSolver::inSimplex(const PxVec3& w) const
{
//	bool found = false;
	int numverts = m_numVertices;
	
	//w is in the current (reduced) simplex
	for(int i=0;i<numverts;i++)
	{
		if (m_simplexVectorW[i] == w)
//			found = true;
			return true;
	}

	//check in case lastW is already removed
	if (w == m_lastW)
		return true;
    	
//	return found;
	return false;
}

bool VoronoiSimplexSolver::closestPtPointTriangle(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, SubSimplexClosestResult& result)
{
	result.m_usedVertices.reset();

    // Check if P in vertex region outside A
    PxVec3 ab = b - a;
    PxVec3 ac = c - a;
    PxVec3 ap = p - a;
    PxReal d1 = ab.dot(ap);
    PxReal d2 = ac.dot(ap);
    if (d1 <= PxReal(0.0) && d2 <= PxReal(0.0)) 
	{
		result.m_closestPointOnSimplex = a;
		result.m_usedVertices.usedVertexA = true;
		result.setBarycentricCoordinates(1,0,0);
		return true;// a; // barycentric coordinates (1,0,0)
	}

    // Check if P in vertex region outside B
    PxVec3 bp = p - b;
    PxReal d3 = ab.dot(bp);
    PxReal d4 = ac.dot(bp);


    if (d3 >= PxReal(0.0) && d4 <= d3) 
	{
		result.m_closestPointOnSimplex = b;
		result.m_usedVertices.usedVertexB = true;
		result.setBarycentricCoordinates(0,1,0);

		return true; // b; // barycentric coordinates (0,1,0)
	}
    // Check if P in edge region of AB, if so return projection of P onto AB
    PxReal vc = d1*d4 - d3*d2;
    if (vc <= PxReal(0.0) && d1 > PxReal(0.0) && d3 < PxReal(0.0))	//AM: changed >= to > to avoid divide by zero case below.
		{
        PxReal v = d1 / (d1 - d3);

		result.m_closestPointOnSimplex = a + v * ab;
		result.m_usedVertices.usedVertexA = true;
		result.m_usedVertices.usedVertexB = true;
		result.setBarycentricCoordinates(1-v,v,0);
		return true;
        //return a + v * ab; // barycentric coordinates (1-v,v,0)
    }

    // Check if P in vertex region outside C
    PxVec3 cp = p - c;
    PxReal d5 = ab.dot(cp);
    PxReal d6 = ac.dot(cp);
    if (d6 >= PxReal(0.0) && d5 <= d6) 
	{
		result.m_closestPointOnSimplex = c;
		result.m_usedVertices.usedVertexC = true;
		result.setBarycentricCoordinates(0,0,1);
		return true;//c; // barycentric coordinates (0,0,1)
	}

    // Check if P in edge region of AC, if so return projection of P onto AC
    PxReal vb = d5*d2 - d1*d6;
    if (vb <= PxReal(0.0) && d2 > PxReal(0.0) && d6 < PxReal(0.0)) {	//AM: changed >= to > to avoid divide by zero case below.
        PxReal w = d2 / (d2 - d6);

		result.m_closestPointOnSimplex = a + w * ac;
		result.m_usedVertices.usedVertexA = true;
		result.m_usedVertices.usedVertexC = true;
		result.setBarycentricCoordinates(1-w,0,w);
		return true;
        //return a + w * ac; // barycentric coordinates (1-w,0,w)
    }

    // Check if P in edge region of BC, if so return projection of P onto BC
    PxReal va = d3*d6 - d5*d4;
    if (va <= PxReal(0.0) && (d4 - d3) > PxReal(0.0) && (d5 - d6) > PxReal(0.0))	//AM: changed >= to > to avoid divide by zero case below.
		{
//		printf("%f %f %f %f\n", d4, d3, d5, d6);
        PxReal w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
	
		result.m_closestPointOnSimplex = b + w * (c - b);
		result.m_usedVertices.usedVertexB = true;
		result.m_usedVertices.usedVertexC = true;
		result.setBarycentricCoordinates(0,1-w,w);
		return true;		
       // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
    }

    // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
    PxReal denom = PxReal(1.0) / (va + vb + vc);
    PxReal v = vb * denom;
    PxReal w = vc * denom;
    
	result.m_closestPointOnSimplex = a + ab * v + ac * w;
	result.m_usedVertices.usedVertexA = true;
	result.m_usedVertices.usedVertexB = true;
	result.m_usedVertices.usedVertexC = true;
	result.setBarycentricCoordinates(1-v-w,v,w);
	
	return true;
//	return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = PxReal(1.0) - v - w
}

/// Test if point p and d lie on opposite sides of plane through abc
int VoronoiSimplexSolver::pointOutsideOfPlane(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d)
{
	PxVec3 normal = (b-a).cross(c-a);

    PxReal signp = (p - a).dot(normal); // [AP AB AC]
    PxReal signd = (d - a).dot( normal); // [AD AB AC]

#ifdef CATCH_DEGENERATE_TETRAHEDRON
#ifdef BT_USE_DOUBLE_PRECISION
if (signd * signd < (PxReal(1e-8) * PxReal(1e-8)))
	{
		return -1;
	}
#else
	if (signd * signd < (PxReal(1e-4) * PxReal(1e-4)))
	{
//		printf("affine dependent/degenerate\n");//
		return -1;
	}
#endif

#endif
	// Points on opposite sides if expression signs are opposite
    return signp * signd < PxReal(0.);
}


bool VoronoiSimplexSolver::closestPtPointTetrahedron(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, SubSimplexClosestResult& finalResult)
{
	SubSimplexClosestResult tempResult;

    // Start out assuming point inside all halfspaces, so closest to itself
	finalResult.m_closestPointOnSimplex = p;
	finalResult.m_usedVertices.reset();
    finalResult.m_usedVertices.usedVertexA = true;
	finalResult.m_usedVertices.usedVertexB = true;
	finalResult.m_usedVertices.usedVertexC = true;
	finalResult.m_usedVertices.usedVertexD = true;

    int pointOutsideABC = pointOutsideOfPlane(p, a, b, c, d);
	int pointOutsideACD = pointOutsideOfPlane(p, a, c, d, b);
  	int	pointOutsideADB = pointOutsideOfPlane(p, a, d, b, c);
	int	pointOutsideBDC = pointOutsideOfPlane(p, b, d, c, a);

   if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0)
   {
	   finalResult.m_degenerate = true;
	   return false;
   }

   if (!pointOutsideABC  && !pointOutsideACD && !pointOutsideADB && !pointOutsideBDC)
	 {
		 return false;
	 }


    PxReal bestSqDist = FLT_MAX;
    // If point outside face abc then compute closest point on abc
	if (pointOutsideABC) 
	{
        closestPtPointTriangle(p, a, b, c,tempResult);
		PxVec3 q = tempResult.m_closestPointOnSimplex;
		
        PxReal sqDist = (q - p).dot( q - p);
        // Update best closest point if (squared) distance is less than current best
        if (sqDist < bestSqDist) {
			bestSqDist = sqDist;
			finalResult.m_closestPointOnSimplex = q;
			//convert result bitmask!
			finalResult.m_usedVertices.reset();
			finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;
			finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexB;
			finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexC;
			finalResult.setBarycentricCoordinates(
					tempResult.m_barycentricCoords[VERTA],
					tempResult.m_barycentricCoords[VERTB],
					tempResult.m_barycentricCoords[VERTC],
					0
			);

		}
    }
  

	// Repeat test for face acd
	if (pointOutsideACD) 
	{
        closestPtPointTriangle(p, a, c, d,tempResult);
		PxVec3 q = tempResult.m_closestPointOnSimplex;
		//convert result bitmask!

        PxReal sqDist = (q - p).dot( q - p);
        if (sqDist < bestSqDist) 
		{
			bestSqDist = sqDist;
			finalResult.m_closestPointOnSimplex = q;
			finalResult.m_usedVertices.reset();
			finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;

			finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexB;
			finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexC;
			finalResult.setBarycentricCoordinates(
					tempResult.m_barycentricCoords[VERTA],
					0,
					tempResult.m_barycentricCoords[VERTB],
					tempResult.m_barycentricCoords[VERTC]
			);

		}
    }
    // Repeat test for face adb

	
	if (pointOutsideADB)
	{
		closestPtPointTriangle(p, a, d, b,tempResult);
		PxVec3 q = tempResult.m_closestPointOnSimplex;
		//convert result bitmask!

        PxReal sqDist = (q - p).dot( q - p);
        if (sqDist < bestSqDist) 
		{
			bestSqDist = sqDist;
			finalResult.m_closestPointOnSimplex = q;
			finalResult.m_usedVertices.reset();
			finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;
			finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexC;
			
			finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexB;
			finalResult.setBarycentricCoordinates(
					tempResult.m_barycentricCoords[VERTA],
					tempResult.m_barycentricCoords[VERTC],
					0,
					tempResult.m_barycentricCoords[VERTB]
			);

		}
    }
    // Repeat test for face bdc
    

	if (pointOutsideBDC)
	{
        closestPtPointTriangle(p, b, d, c,tempResult);
		PxVec3 q = tempResult.m_closestPointOnSimplex;
		//convert result bitmask!
        PxReal sqDist = (q - p).dot( q - p);
        if (sqDist < bestSqDist) 
		{
			bestSqDist = sqDist;
			finalResult.m_closestPointOnSimplex = q;
			finalResult.m_usedVertices.reset();
			//
			finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexA;
			finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexC;
			finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexB;

			finalResult.setBarycentricCoordinates(
					0,
					tempResult.m_barycentricCoords[VERTA],
					tempResult.m_barycentricCoords[VERTC],
					tempResult.m_barycentricCoords[VERTB]
			);

		}
    }

	//help! we ended up full !
	
	if (finalResult.m_usedVertices.usedVertexA &&
		finalResult.m_usedVertices.usedVertexB &&
		finalResult.m_usedVertices.usedVertexC &&
		finalResult.m_usedVertices.usedVertexD) 
	{
		return true;
	}

    return true;
}

#ifndef __SPU__
/*
static void SetInterpolate3(const PxVec3& a, const PxVec3& b, PxReal lambda, PxVec3& dest)
{
	const PxReal s = PxReal(1.0) - lambda;
	dest.x = s * a.x + lambda * b.x;
	dest.y = s * a.y + lambda * b.y;
	dest.z = s * a.z + lambda * b.z;
}
*/
struct VoronoiSimplexSolver2
{
	PxVec3	ws[4], as[4]; // since w = p-q we know q
	PxU32	nextToDrop;
	PxReal	bary[2];
	PxU16	baryIdx[3];
	PxVec3	lastA; // only used if we terminated due to being inside of a tetrahedron
	bool	lastAValid;

	VoronoiSimplexSolver2(const PxVec3& w0, const PxVec3& p0) : nextToDrop(0), lastA(0.0f), lastAValid(false)
	{
		ws[0] = ws[1] = ws[2] = ws[3] = w0;
		as[0] = as[1] = as[2] = as[3] = p0;
		bary[0] = bary[1] = 0.0f;
		baryIdx[0] = baryIdx[1] = baryIdx[2] = 0;
	}

	void addVertex(const PxVec3& w, const PxVec3& a)
	{
		ws[nextToDrop] = w;
		as[nextToDrop] = a; // a is on A in Minkowski sum A+(-B)
	}

	int pointOutsideOfPlane(
		const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, PxVec3& normalArg)
	{
		PxVec3 normal = (b-a).cross(c-a);
		normalArg = normal;

	    PxReal signp = (p - a).dot(normal); // [AP AB AC]
	    PxReal signd = (d - a).dot(normal); // [AD AB AC]

		if (signd * signd < (PxReal(1e-4) * PxReal(1e-4)))
			return -1;

		// Points on opposite sides if expression signs are opposite
	    return signp * signd < PxReal(0.);
	}

	PxReal closestOnSegment(const PxVec3& p, const PxVec3& a, const PxVec3& b, PxVec3& pab)
	{
		PxVec3 ab = b-a;
		PxReal ab2 = ab.dot(ab);
		PxReal invDenom = (ab2 < 1e-6f) ? 0.0f : 1.0f / ab2; // AP scaffold newccd epsilon
		PxReal tp = ab.dot(p-a) * invDenom;
		tp = PxMax(PxMin(tp, 1.0f), 0.0f); // clamp to [0,1]
		pab = a+ab*tp;
		return tp;
	}

	PxVec3 closestOnTriangle(const PxVec3& p, PxU32 i0, PxU32 i1, PxU32 i2)
	{
		const PxReal epsilon = 1e-8f; // AP scaffold newccd: magic number
		PxVec3 a = ws[i0], b = ws[i1], c = ws[i2];
		PxVec3 n = (b-a).cross(c-a);
		// n.a + d = 0
		PxReal d = -n.dot(a);
		// n.(p + t*n)+d = 0
		// n.p + tn.n+d = 0
		// t = (-n.p-d)/n.n
		PxReal invN = n.dot(n);
		bool degenerate = invN < epsilon;
		invN = degenerate ? 1.0f : 1.0f / invN;
		PxReal t = (-n.dot(p)-d) * invN;
		PxVec3 r = p + n*t;
		PxVec3 ar = r-a, ca = a-c, cb = b-c, ab = b-a, cr = r-c;
		PxVec3 crossABAC = n;
		PxVec3 crossCACR = ca.cross(cr);
		PxVec3 crossCRCB = cr.cross(cb);
		// areaABC = dot(crossABC, crossABC) / |crossABC|
		// areaABAR = dot(crossABAR, crossABAR) / |crossABR|
		// since dot(crossABAC, crossABAR) = |crossABAC|*|crossABAR| =>
		//  dot(crossABAC, crossABR)/dot(crossABAC, crossABAC) = |crossABAC|/|crossABAR|*sign = signed area ratio or bary coord
		PxReal invABC2 = invN;
		crossABAC *= invABC2;
		bary[0] = crossABAC.dot(crossCRCB);
		bary[1] = crossABAC.dot(crossCACR);
		baryIdx[0] = i0;
		baryIdx[1] = i1;
		baryIdx[2] = i2;
		PxReal bary2 = 1.0f-bary[0]-bary[1];

		if (degenerate || PxMin(PxMin(bary[0], bary[1]), bary2) < 0.0f || PxMax(PxMax(bary[0], bary[1]), bary2) > 1.0f)
		{
			PxVec3 pab, pbc, pca;
			PxReal t0 = closestOnSegment(p, a, b, pab);
			PxReal t1 = closestOnSegment(p, b, c, pbc);
			PxReal t2 = closestOnSegment(p, c, a, pca);
			PxReal mag0 = (p-pab).magnitudeSquared();
			PxReal mag1 = (p-pbc).magnitudeSquared();
			PxReal mag2 = (p-pca).magnitudeSquared();
			if (mag0 <= mag1 && mag0 <= mag2)
			{
				bary[0] = 1.0f - t0;
				bary[1] = t0;
				baryIdx[0] = i0;
				baryIdx[1] = i1;
				baryIdx[2] = i2;
				r = pab;
			}
			else if (mag1 <= mag2)
			{
				bary[0] = 1.0f - t1;
				bary[1] = t1;
				baryIdx[0] = i1;
				baryIdx[1] = i2;
				baryIdx[2] = i0;
				r = pbc;
			}
			else
			{
				bary[0] = 1.0f - t2;
				bary[1] = t2;
				baryIdx[0] = i2;
				baryIdx[1] = i0;
				baryIdx[2] = i1;
				r = pca;
			}
		}

		return r;
	}

	PX_FORCE_INLINE PxReal vec3det(const PxVec3& v1, const PxVec3& v2, const PxVec3& v3)
	{
		return v1.x * (v2.y * v3.z - v3.y * v2.z) -
			   v2.x * (v1.y * v3.z - v3.y * v1.z) +
			   v3.x * (v1.y * v2.z - v2.y * v1.z);
	}

	bool bary4(const PxVec3& A, const PxVec3& B, const PxVec3& C, const PxVec3& D, const PxVec3& Q, PxReal out[4])
	{
		PxVec3 AB = B-A, AC = C-A, AD = D-A;
		PxReal vABCD = vec3det(AB, AC, AD);
		if (PxAbs(vABCD) < 1e-6f)
			return false;

		PxVec3 QA = A-Q, QB = B-Q, QC = C-Q, QD = D-Q;

		PxReal vQBCD = vec3det(QB, QC, QD);
		PxReal vQADC = vec3det(QA, QD, QC);
		PxReal vQABD = vec3det(QA, QB, QD);
		PxReal vQACB = vec3det(QA, QC, QB);
		
		out[ 0 ] = vQBCD / vABCD;
		out[ 1 ] = vQADC / vABCD;
		out[ 2 ] = vQABD / vABCD;
		out[ 3 ] = vQACB / vABCD;

		return true;
	}

	PxVec3 getClosest(const PxVec3& w)
	{
		PxReal b4[4];
		bool validTetrahedron = bary4(ws[0], ws[1], ws[2], ws[3], w, b4);
		PxReal minBary = PxMin(PxMin(b4[0], b4[1]), PxMin(b4[2], b4[3]));
		PxReal maxBary = PxMax(PxMax(b4[0], b4[1]), PxMax(b4[2], b4[3]));
		if (validTetrahedron && minBary > 1e-5f && maxBary < 1.0f-1e-5f)
		{
			lastAValid = true;
			lastA = as[0]*b4[0] + as[1]*b4[1] + as[2]*b4[2] + as[3]*b4[3];
			return w;
		}

		// AP scaffold newccd TODO: optimize (closestOnTriangle computed twice for same params)
		PxVec3 p0 = closestOnTriangle(w, 0, 1, 2); 
		PxVec3 p1 = closestOnTriangle(w, 0, 1, 3); 
		PxVec3 p2 = closestOnTriangle(w, 0, 2, 3); 
		PxVec3 p3 = closestOnTriangle(w, 1, 2, 3); 
		PxReal mag0 = (w-p0).magnitudeSquared();
		PxReal mag1 = (w-p1).magnitudeSquared();
		PxReal mag2 = (w-p2).magnitudeSquared();
		PxReal mag3 = (w-p3).magnitudeSquared();
		if (mag0 <= mag1 && mag0 <= mag2 && mag0 <= mag3)
		{
			nextToDrop = 3;
			return closestOnTriangle(w, 0, 1, 2); 
		}
		else if (mag1 <= mag2 && mag1 <= mag3)
		{
			nextToDrop = 2;
			return closestOnTriangle(w, 0, 1, 3); 
		}
		else if (mag2 <= mag3)
		{
			nextToDrop = 1;
			return closestOnTriangle(w, 0, 3, 2);
		}
		else
		{
			nextToDrop = 0;
			return closestOnTriangle(w, 2, 3, 1);
		}
	}

	PxVec3 getClosestA()
	{
		if (lastAValid)
			return lastA;

		PxReal bary2 = 1.0f - bary[0] - bary[1];
		return as[baryIdx[0]]*bary[0] + as[baryIdx[1]]*bary[1] + as[baryIdx[2]]*bary2;
	}
};

//-------------------------------------------------------
//Note: This algo will return a zero destNormal if the shapes were already almost in contact or overlapping at the start of the frame!
bool physx::convexConvexLinearSweep(
	const GJKConvexInterface& convexA,
	const GJKConvexInterface& convexB,
	const PxTransform& shape2worldA,				//these are shape2world transforms!
	const PxVec3& worldDestPosA,
	const PxTransform& shape2worldB,
	const PxVec3& worldDestPosB,
	PxReal distTreshold, 
	PxVec3& destNormal, PxVec3& destWorldPointA,
	PxReal& toi)
{
	//printf("gjkSweep start\n");
	GJKConvexInterfaceCache cache;

	destNormal = destWorldPointA = PxVec3(0.0f);
	toi = PX_MAX_REAL;

	PxTransform xformA = shape2worldB.transformInv(shape2worldA); // coordsys_1  - change coordsys by subtracting shape2worldB from both A and B
	PxTransform xformB = PxTransform::createIdentity();
	xformA.q.normalize();

	PxVec3 linMotionA, linMotionB;
	linMotionA = shape2worldB.transformInv(worldDestPosA-shape2worldA.p); // also subtract shape2worldB
	linMotionB = shape2worldB.transformInv(worldDestPosB-shape2worldB.p); // also subtract shape2worldB

				#if DEBUG_RENDER_CCD
				PxScene *s; PxGetPhysics()->getScenes(&s, 1, 0);
				Cm::RenderOutput((Cm::RenderBuffer&)s->getRenderBuffer())
					<< PxDebugColor::eARGB_GREEN << Cm::RenderOutput::LINES << shape2worldA.p << shape2worldA.p + linMotionA;
				Cm::RenderOutput((Cm::RenderBuffer&)s->getRenderBuffer())
					<< PxDebugColor::eARGB_GREEN << Cm::RenderOutput::LINES << shape2worldB.p << shape2worldB.p + linMotionB;
				#endif

	// We just translated A and B at [t=0] so that B[t=0] is at 0,0,0 and A is translated correspondingly
	// Next, by subtracting motionB from motionA we effectively translate the problem into a problem with static B
	// (subtract motionB from both motionA and motionB yields zero motion for motionB)
	// We define r as linMotionA-linMotionB
	// Since B is now static, the Minkowski difference A-B also moves by linMotionA-linMotionB
	// We want to find the first time when Minkowski difference contains the origin (A and B overlap)
	// This is equivalent to finding the first time of intersection of the ray originating at 0 and moving in the direction r,
	// Since we treat the object as static and the origin as moving (thus a raycast) we should negate r
	PxVec3 r = -(linMotionA-linMotionB);

	// if we don't have motion, there is nothing to compute.
	if (r.magnitudeSquared() < distTreshold*distTreshold)
	{
		//printf("gjkSweep return due to no motion\n");
		return false;
	}

	// support[A-B](v) = support[A](v)-support[B](-v)
	// support query on Minkowski difference - this should initially hit on the 'front' sides of both objects
	// as far as the motion is concerned.
	// We use -r as initial guess at support query direction.
	// This is a good initial guess for objects moving towards each other
	#if 0 // AP: for debugging only.
	PxReal angleA;
	PxVec3 axisA;
	xformA.q.toRadiansAndUnitAxis(angleA, axisA);
	#endif

	PxVec3 supVertexA = xformA.transform(convexA.projectHullMax(xformA.rotateInv(-r.getNormalized()), cache));
	PxVec3 supVertexB = convexB.projectHullMax(r.getNormalized(), cache); // xformB is identity
	//printf("supva=%.4f, %.4f, %.4f supvb=%.4f %.4f %.4f\n", supVertexA.x, supVertexA.y, supVertexA.z, supVertexB.x, supVertexB.y, supVertexB.z);
	PxVec3 v = supVertexA-supVertexB;	// initial guess support vertex on Minkowski difference

	VoronoiSimplexSolver2 simplexSolver(v, supVertexA);

	PxVec3 w, p(PxVec3(PxReal(0))), n(PxReal(0)), closestA; // p is the ray origin and it is in Minkowski sum space
	PxReal lambdaLo = 0.0f, lambdaHi = 1.0f; // lower and upper bounds on unclipped ray section, these are relative to starting ray

	// for threshold of 1e-6 a triangle inflated with a sphere had a difficult time converging
	// gjk sweep legitimately wouldn't converge to 1e-6 magnitude of |v-p| because of inverse square root
	// inaccuracy in support mapping computation for capsules/spheres, normalize of a vector using
	// x*1.0f/sqrtf(x.magnitude2()) produces (-1-e6, -1, -1e-6), which turns out to be enough to
	// destabilize the algorithm). In this case a triangle parallel to y=0 is Minkowski subtracted
	// from a sphere hitting it straight on in y direction and as the raycast approaches MD
	// surface the spanning points for the tetrahedron converge along the rounded corners of the MD
	// to almost near the actual triangle surface but as they try to approach closer to the plane they fail
	// just short of necessary precision
	distTreshold = 1e-4f; // AP scaffold newccd
	int maxIter = MAX_GJK_SWEEP_ITERATIONS;
	// count how many times in a row GJK produced a degenerate result. this is to take care of a case
	// when the raycast direction is parallel to a degenerate planar GJK tetrahedron (simplex)
	int successiveDegenerate = 0;
	PxU32 lastNextToDrop = 0xFFFF;
	PxU32 sameNextToDropCount = 0;
	PxU32 numPullbacks = 0;
	while (maxIter--)
	{
		const PxReal vMag = (v-p).magnitudeSquared();
		if (vMag < distTreshold*distTreshold)
			// v is near p means support vertex is near the origin, meaning the convex intersects with the ray right away
			// => proceed to exit and return current lambdaLo
			break;

		//printf("v=%.4f %.4f %.4f p=%.4f %.4f %.4f\n", v.x, v.y, v.z, p.x, p.y, p.z);
		n = -(v-p).getNormalized();

		// we use -(v-p) as new direction to compute the support mapping in. v is the support vertex from the previous GJK iteration
		// since support[A-B](D) = support[A](D)-support[B](-D)
		supVertexA = xformA.transform(convexA.projectHullMax(xformA.rotateInv(n), cache));
		supVertexB = convexB.projectHullMax(-n, cache); // xformB is identity

		// w[k+1] in Gino's notation is a new vertex on Minkowski difference to be added to simplex W[k+1] (new simplex)
		// v[k] is the closest point on Minkowski simplex to the origin
		w = supVertexA-supVertexB; // support point on Minkowski difference

		// the equation for the intersection of the ray with a halfspace tangent to a convex (w, n) is:
		// (s+lambda*r-w).n=0
		// s.n+lambda*r.n-w.n = 0
		// lambda = (w.n-s.n)/(r.n)
		// if r.n>epsilon then lambda is a high clip
		// if r.n<-epsilon lambda is a low clip
		// S=0 since the ray originates at 0

		// add w to simplex. w was computed before simplex translation so we translate after adding w
		simplexSolver.addVertex(w, supVertexA);
		v = simplexSolver.getClosest(p);
		
		if (lastNextToDrop == simplexSolver.nextToDrop)
			sameNextToDropCount ++;
		else
			sameNextToDropCount = 0;
		lastNextToDrop = simplexSolver.nextToDrop;

		const PxReal rDotN = r.dot(n);
		// ray nearly parallel to the plane, but origin is not within threshold from the plane
		// => not useful if near 0, iterate gjk some more
		//printf("rDotN=%.7f r=%.7f %.7f %.7f n=%.7f %.7f %.7f\n", rDotN, r.x, r.y, r.z, n.x, n.y, n.z);
		if (1 && successiveDegenerate >= 4)
		{
			// stuck in a degenerate loop with no distance within threshold.. report no intersection
			// this is possible for instance when a degenerate tetrahedron is falling on a box and
			// the plane the tetrahedron is in, is perpendicular to the box face.
			toi = PX_MAX_REAL;
			//printf("gjkSweep returning due to successiveDegenerate test at maxIter = %d\n", maxIter);
			return false;
		}
		if (PxAbs(rDotN) > 1e-10f)
		{
			successiveDegenerate = 0; // reset the counter for successive degenerate raycasts
			PxReal lambda = w.dot(n)/rDotN;
			if (rDotN < 0.0f) // ray is opposite normal, empty halfspace increases the lower clip bound
			{
				if (lambda > lambdaLo)
				{
					lambdaLo = lambda;
					p = lambda*r;
				}
			} else // ray is same direction as normal, empty halfspace decreases the upper clip bound
				lambdaHi = PxMin(lambdaHi, lambda);

			if (lambdaLo > lambdaHi)
			{
				// the entire ray was clipped away, return no intersection and "infinite" toi
				toi = PX_MAX_REAL;
				//printf("gjkSweep returning false\n");
				return false;
			}
		} else
			successiveDegenerate ++; // increment the counter of successive degenerate raycasts

		// The shape in Minkowski space is a slightly rotated box + capsule centered around x=+20 with raycast from 0 to 20 or so.
		// So effectively the impact side of the Minkowski sum is a square+capsule. The situation is pretty unstable as is,
		// even for normal GJK because as it converges the spanning simplex points on the capsule approach the accurate values rather slowly.
		// In addition as the ray gets clipped, the v-p magnitude approaches zero as the ray nears the surface of the Minkowski difference
		// (within the precision threshold) adding extra instability to the algorithm
		// We attempt to fix this case by pulling back along the ray to give the algorithm more precision room to converge.
		if (sameNextToDropCount == 4 && numPullbacks < 2) // pull back the ray point
		{
			numPullbacks ++;
			sameNextToDropCount = 0;
			lambdaLo *= 0.9f;
			p = lambdaLo*r;
		}
	}

	if (maxIter == -1)
		PX_WARN_ONCE(maxIter == -1, "GJKSweep - poor convergence");

	// low ray clip is the toi
	toi = lambdaLo;

	// we can't use xformA adjusted in Minkowski space since it was using relative motion, we need just A's motion
	xformA.p = shape2worldA.p * (1.0f-toi) + worldDestPosA * toi;
	xformA.q = shape2worldA.q;
	xformB.p = shape2worldB.p * (1.0f-toi) + worldDestPosB * toi;
	xformB.q = shape2worldB.q;

	// now we have v on Minkowski sum, retrieve corresponding P
	// since we recorded it for A with subtracted shape2worldB transform, (see coordsys_1 performed above), we need to undo that effect
	// (by adding shape2worldB). Also we subtract shape2worldA.p and add world transform adjusted for toi to get the actual position
	// at time of impact (since the sweep is linear). Note that xformA.p is adjusted for toi just above
	/* added to fix a weird glitch in visual assist */
	/* added to fix a weird glitch in visual assist */
	PxVec3 worldA = shape2worldB.transform(simplexSolver.getClosestA()) - shape2worldA.p + xformA.p;

	PxVec3 localA = xformA.transformInv(worldA);
	PxVec3 localB = xformB.transformInv(worldA);

	int isFaceA = 0, isFaceB = 0;
	PxVec3 localNormalA = convexA.inverseSupportMapping(localA, isFaceA);
	PxVec3 localNormalB = convexB.inverseSupportMapping(localB, isFaceB);

	if (isFaceB > isFaceA)
		destNormal = -xformB.rotate(localNormalB);
	else
		destNormal = xformA.rotate(localNormalA);

	destWorldPointA = worldA;

	//printf("gjkSweep returning true\n");
	return true;
}
#endif

#if 0
//-------------------------------------------------------
// Prototype code for 2-way sphere sweep as an option for CCD
bool physx::convexConvexLinearSweep2WaySphere(
	const GJKConvexInterface& convexA,
	const GJKConvexInterface& convexB,
	const PxTransform& shape2worldA,				//these are shape2world transforms!
	const PxVec3& worldDestPosA,
	const PxTransform& shape2worldB,
	const PxVec3& worldDestPosB,
	PxReal distTreshold, 
	PxVec3& destNormal, PxVec3& destWorldPointA,
	PxReal& toi)
{
	PxVec3 centerA, centerB;
	PxReal radiusA, radiusB;
	convexA.getInnerSphere(centerA, radiusA);
	convexB.getInnerSphere(centerB, radiusB);
	Gu::GJKSphereSupport sA(radiusA);
	Gu::GJKSphereSupport sB(radiusB);
	PxVec3 normal0, normal1, point0, point1;
	PxReal toi0, toi1;
	bool status0 = convexConvexLinearSweep(
		sA, convexB,
		PxTransform(shape2worldA.p + centerA, shape2worldA.q), worldDestPosA + centerA,
		shape2worldB, worldDestPosB,
		distTreshold, normal0, point0, toi0);
	bool status1 = convexConvexLinearSweep(
		convexA, sB,
		shape2worldA, worldDestPosA,
		PxTransform(shape2worldB.p + centerB, shape2worldB.q), worldDestPosB + centerB,
		distTreshold, normal1, point1, toi1);

	if (toi0 < toi1)
	{
		toi = toi0;
		destNormal = normal0;
		destWorldPointA = point0;
		return status0;
	}
	else
	{
		toi = toi1;
		destNormal = -normal1;
		destWorldPointA = point1;
		return status1;
	}
}
#endif

//-------------------------------------------------------
struct DiscreteCollisionDetectorInterface
{
	struct ClosestPointInput
	{
		ClosestPointInput()	:m_maximumDistanceSquared(PxReal(1e30))
		{
		}

		PxTransform	m_transformA;
		PxTransform	m_transformB;
		PxReal		m_maximumDistanceSquared;
	};
};


struct PointCollector
{
	PxVec3	m_normalOnBInWorld;
	PxVec3	m_pointInWorldA;
	PxVec3	m_pointInWorldB;
	PxReal	m_distance;//negative means penetration

	bool	m_hasResult;

	PointCollector () 
		: m_distance(PxReal(1e30)),m_hasResult(false)
	{
	}

	void addContactPoint(const PxVec3& normalOnBInWorld, const PxVec3& pointInWorldA, const PxVec3& pointInWorldB, PxReal depth)
	{
		if (depth< m_distance)
		{
			m_hasResult			= true;
			m_normalOnBInWorld	= normalOnBInWorld;
			m_pointInWorldA		= pointInWorldA;
			m_pointInWorldB		= pointInWorldB;
			//negative means penetration
			m_distance			= depth;
		}
	}
};



class GjkPairDetector : public DiscreteCollisionDetectorInterface
{
	PxVec3	m_cachedSeparatingAxis;
	VoronoiSimplexSolver* m_simplexSolver;
	const GJKConvexInterface* m_minkowskiA;
	const GJKConvexInterface* m_minkowskiB;
	

public:

	//some debugging to fix degeneracy problems
	int			m_lastUsedMethod;
	int			m_curIter;
	int			m_degenerateSimplex;
	int			m_catchDegeneracies;


	GjkPairDetector(const GJKConvexInterface* objectA,const GJKConvexInterface* objectB,VoronoiSimplexSolver* simplexSolver, const PxVec3& worldSepAxisGuess);

	//virtual
		void	getClosestPoints(const ClosestPointInput& input,PointCollector& output, GJKConvexInterfaceCache& cache);
	const PxVec3&	getCachedAxis() const { return m_cachedSeparatingAxis; } 

};



GjkPairDetector::GjkPairDetector(const GJKConvexInterface* objectA,const GJKConvexInterface* objectB,VoronoiSimplexSolver* simplexSolver, const PxVec3& worldSepAxisGuess)
:m_cachedSeparatingAxis(worldSepAxisGuess), //PxReal(0.),PxReal(0.),PxReal(1.)),
m_simplexSolver(simplexSolver),
m_minkowskiA(objectA),
m_minkowskiB(objectB),
m_lastUsedMethod(-1),
m_catchDegeneracies(1)
{
}

//#define LOCAL_SPACE_GJK
#ifdef LOCAL_SPACE_GJK
#include "PsMatrix34.h"
#endif

void GjkPairDetector::getClosestPoints(const ClosestPointInput& input, PointCollector& output, GJKConvexInterfaceCache& cache)
{
#ifdef LOCAL_SPACE_GJK
	PxTransform inverseA = input.m_transformA.getInverse();
//	PxTransform idtCheck = input.m_transformA.transform(inverseA);
	PxTransform relTM = inverseA.transform(input.m_transformB);
	m_cachedSeparatingAxis = inverseA.rotate(m_cachedSeparatingAxis);
#endif

	PxReal distance = PxReal(0.);
	PxVec3 normalInB(PxReal(0.),PxReal(0.),PxReal(0.));
	PxVec3 pointOnA,pointOnB;
#ifdef LOCAL_SPACE_GJK
//	PxTransform localTransA;	// Idt
	PxTransform localTransB = relTM;
	const PxVec3 positionOffset = (localTransB.p) * PxReal(0.5);
//	localTransA.p -= positionOffset;
	localTransB.p -= positionOffset;

	Ps::Matrix34 relMat(relTM);
	Ps::Matrix34 localMatB(localTransB);
#else
	PxTransform localTransA = input.m_transformA;
	PxTransform localTransB = input.m_transformB;
	const PxVec3 positionOffset = (localTransA.p + localTransB.p) * PxReal(0.5);
	localTransA.p -= positionOffset;
	localTransB.p -= positionOffset;
#endif

	const PxReal marginA = 0.0f;//m_minkowskiA->getMargin();	//Could do inflation here!
	const PxReal marginB = 0.0f;//m_minkowskiB->getMargin();

	m_curIter = 0;
	int gGjkMaxIter = 1000;//this is to catch invalid input, perhaps check for #NaN?
//	m_cachedSeparatingAxis.set(0,1,0);							//NEW: warm start with whatever we used last!

	bool isValid = false;
	bool checkSimplex = false;
	bool checkPenetration = true;
	m_degenerateSimplex = 0;

	m_lastUsedMethod = -1;

	{
		PxReal squaredDistance = FLT_MAX;
		PxReal delta = PxReal(0.);
		
		PxReal margin = marginA + marginB;
		
		

		m_simplexSolver->reset();
		
		for ( ; ; )
		{
#ifdef LOCAL_SPACE_GJK
//			PxVec3 separatingAxisInA = -m_cachedSeparatingAxis;//* input.m_transformA.getBasis();
//			PxVec3 seperatingAxisInB = relTM.rotateInv(m_cachedSeparatingAxis);//* input.m_transformB.getBasis();
			PxVec3 seperatingAxisInB = relMat.rotateTranspose(m_cachedSeparatingAxis);//* input.m_transformB.getBasis();
//			PxVec3 pInA = m_minkowskiA->projectHullMax(seperatingAxisInA, cache);
			PxVec3 pInA = m_minkowskiA->projectHullMax(-m_cachedSeparatingAxis, cache);
			PxVec3 qInB = m_minkowskiB->projectHullMax(seperatingAxisInB, cache);
//			PxVec3  pWorld = localTransA.transform(pInA);
			PxVec3  pWorld = pInA - positionOffset;
//			PxVec3  qWorld = localTransB.transform(qInB);
			PxVec3  qWorld = localMatB.transform(qInB);
#else
			PxVec3 seperatingAxisInA = input.m_transformA.rotateInv(-m_cachedSeparatingAxis);//* input.m_transformA.getBasis();
			PxVec3 seperatingAxisInB = input.m_transformB.rotateInv(m_cachedSeparatingAxis);//* input.m_transformB.getBasis();

			PxReal magA = seperatingAxisInA.normalizeSafe();
			PxReal magB = seperatingAxisInB.normalizeSafe();
			if (magA == 0.0f)
				seperatingAxisInA = PxVec3(1.0f, 0.0f, 0.0f);
			if (magB == 0.0f)
				seperatingAxisInB = PxVec3(-1.0f, 0.0f, 0.0f);
			PxVec3 pInA = m_minkowskiA->projectHullMax(seperatingAxisInA, cache);
			PxVec3 qInB = m_minkowskiB->projectHullMax(seperatingAxisInB, cache);
			PxVec3  pWorld = localTransA.transform(pInA);	
			PxVec3  qWorld = localTransB.transform(qInB);
#endif
			
			PxVec3 w = pWorld - qWorld;
			delta = m_cachedSeparatingAxis.dot(w);

			// potential exit, they don't overlap
			if ((delta > PxReal(0.0)) && (delta * delta > squaredDistance * input.m_maximumDistanceSquared)) 
			{
				checkPenetration = false;
				break;
			}

			//exit 0: the new point is already in the simplex, or we didn't come any closer
			if (m_simplexSolver->inSimplex(w))
			{
				m_degenerateSimplex = 1;
				checkSimplex = true;
				break;
			}
			// are we getting any closer ?
			PxReal f0 = squaredDistance - delta;
			PxReal f1 = squaredDistance * PX_GJK_REL_ERROR2;

			if (f0 <= f1)
			{
				if (f0 <= PxReal(0.))
				{
					m_degenerateSimplex = 2;
				}
				checkSimplex = true;
				break;
			}
			//add current vertex to simplex
			m_simplexSolver->addVertex(w, pWorld, qWorld);

			//calculate the closest point to the origin (update vector v)
			if (!m_simplexSolver->closest(m_cachedSeparatingAxis))
			{
				m_degenerateSimplex = 3;
				checkSimplex = true;
				break;
			}

			if(m_cachedSeparatingAxis.magnitudeSquared()<PX_GJK_REL_ERROR2)
            {
                m_degenerateSimplex = 6;
                checkSimplex = true;
                break;
            }

			PxReal previousSquaredDistance = squaredDistance;
			squaredDistance = m_cachedSeparatingAxis.magnitudeSquared();
			
			//redundant m_simplexSolver->compute_points(pointOnA, pointOnB);

			//are we getting any closer ?
			if (previousSquaredDistance - squaredDistance <= FLT_EPSILON * previousSquaredDistance) 
			{ 
				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
				checkSimplex = true;
				break;
			}

			  //degeneracy, this is typically due to invalid/uninitialized worldtransforms for a CollisionObject   
              if (m_curIter++ > gGjkMaxIter)   
              {   
					  PX_ASSERT(0);
                      #if defined(DEBUG) || defined (_DEBUG)   

                              //printf("GjkPairDetector maxIter exceeded:%i\n",m_curIter);   
                              /*printf("sepAxis=(%f,%f,%f), squaredDistance = %f\n",   
                              m_cachedSeparatingAxis.x,   
                              m_cachedSeparatingAxis.y,   
                              m_cachedSeparatingAxis.z,   
                              squaredDistance);   
							  */

                      #endif   
                      break;   

              } 


			bool check = (!m_simplexSolver->fullSimplex());
			//bool check = (!m_simplexSolver->fullSimplex() && squaredDistance > FLT_EPSILON * m_simplexSolver->maxVertex());

			if (!check)
			{
				//do we need this backup_closest here ?
				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
				break;
			}
		}

		if (checkSimplex)
		{
			m_simplexSolver->compute_points(pointOnA, pointOnB);
			normalInB = pointOnA-pointOnB;
			PxReal lenSqr = m_cachedSeparatingAxis.magnitudeSquared();
			//valid normal
			if (lenSqr < PX_GJK_REL_ERROR2)
			{
				m_degenerateSimplex = 5;
				m_lastUsedMethod = 2;
			} 
			else //if (lenSqr > FLT_EPSILON*FLT_EPSILON)	//this recovery at all costs stuff produces false epsilon distances in 5% of the cases when I have a sphere embedded in a convex.
			{
				PxReal rlen = PxRecipSqrt(lenSqr);
				normalInB *= rlen; //normalize
				distance = lenSqr*rlen - margin;

				PX_ASSERT(squaredDistance > PxReal(0.0));
				PxReal s = PxRecipSqrt(squaredDistance);
			
				pointOnA -= m_cachedSeparatingAxis * (marginA * s);
				pointOnB += m_cachedSeparatingAxis * (marginB * s);
				isValid = true;
				
				m_lastUsedMethod = 1;
			}
		/*else
			{
				m_lastUsedMethod = 2;
			}
		*/
		}

		bool catchDegeneratePenetrationCase = (m_catchDegeneracies && m_degenerateSimplex && ((distance+margin) < PX_GJK_REL_ERROR2));

		//if (checkPenetration && !isValid)
		if (checkPenetration && (!isValid || catchDegeneratePenetrationCase ))
		{
		//penetration case
		//PX_ASSERT(0);	//DO SOMETHING!
		//printf("warning: degenerate distance or penetration!\n");	
		}
	}
	//printf("%d\n", m_curIter);

	PX_ASSERT(distance < 1000.0f);


	if (isValid)
	{
#ifdef LOCAL_SPACE_GJK
		m_cachedSeparatingAxis = input.m_transformA.rotate(m_cachedSeparatingAxis);
		output.addContactPoint(
			input.m_transformA.rotate(normalInB),
			input.m_transformA.transform(pointOnA+positionOffset),
			input.m_transformA.transform(pointOnB+positionOffset),
			distance);
#else
		output.addContactPoint(
			normalInB,
			pointOnA+positionOffset,
			pointOnB+positionOffset,
			distance);
#endif
	}
}

bool physx::convexConvexDistance(
		const GJKConvexInterface& convexA, const GJKConvexInterface& convexB,
		const PxTransform& shape2worldA, const PxTransform& shape2worldB,
		PxVec3& worldSepAxisGuessInOut,
		PxVec3& destWorldNormalOnB, PxVec3& destWorldPointA, PxVec3& destWorldPointB, PxReal& destDistance,
		GJKConvexInterfaceCache& cache,
		PxU32* nbIter)
{
	VoronoiSimplexSolver simplexSolver;
	simplexSolver.reset();
	PointCollector pointCollector;
	
	GjkPairDetector gjk(&convexA, &convexB, &simplexSolver, worldSepAxisGuessInOut);		
	GjkPairDetector::ClosestPointInput input;
	input.m_transformA = shape2worldA;
	input.m_transformB = shape2worldB;
	gjk.getClosestPoints(input, pointCollector, cache);

	worldSepAxisGuessInOut = gjk.getCachedAxis();

	destWorldNormalOnB = pointCollector.m_normalOnBInWorld;
	destWorldPointA = pointCollector.m_pointInWorldA;
	destWorldPointB = pointCollector.m_pointInWorldB;

	destDistance = pointCollector.m_distance;

	if(nbIter)
		*nbIter = gjk.m_curIter;

	return pointCollector.m_hasResult;
}

#ifndef __SPU__

static PxReal getMaxRelevantRadius(const GJKConvexInterface* convex, const PxTransform& convex2world, const PxVec3& angMotion, GJKConvexInterfaceCache& cache)
{
	const PxVec3 localAngMotion = convex2world.rotateInv(angMotion);
	PxVec3 t0, t1;	
	Ps::normalToTangents(localAngMotion, t0, t1);	

	const PxReal localminT0 = t0.dot(convex->projectHullMax(-t0, cache));//finds the vertex with maximum dot product with vector.
	const PxReal localmaxT0 = t0.dot(convex->projectHullMax( t0, cache));//TODO: these could be found out efficiently with a single call to find the box directly
	const PxReal localminT1 = t1.dot(convex->projectHullMax(-t1, cache));
	const PxReal localmaxT1 = t1.dot(convex->projectHullMax( t1, cache));

	//find vertex of rectangle furthest from origin:
	const PxReal a = PxAbs(localminT0) > PxAbs(localmaxT0) ? localminT0 : localmaxT0;
	const PxReal b = PxAbs(localminT1) > PxAbs(localmaxT1) ? localminT1 : localmaxT1;
	return PxSqrt(a*a + b*b);
}

bool physx::convexConvexFullMotionSweep(
		const GJKConvexInterface& convexA,
		const GJKConvexInterface& convexB,
		const PxTransform& shape2worldA,
		const Cm::SpatialVector& motionA,
		const PxTransform& shape2worldB,
		const Cm::SpatialVector& motionB,
		PxReal distTreshold, PxVec3& destNormal, PxVec3& destPoint, PxReal& toi, GJKConvexInterfaceCache& cache)
{
	VoronoiSimplexSolver simplexSolver;

	simplexSolver.reset();

	//TODO: the below stuff could be precomputed!
	PxBounds3 aabbA;
	convexA.getBounds(aabbA);
	PxReal  localSphereRadiusA = (aabbA.maximum - aabbA.minimum).magnitude() * 0.5f;
	PxReal  localCenterMagnitudeA = ((aabbA.maximum + aabbA.minimum) * 0.5f).magnitude();
	PxReal  maxRadiusFromCenterOfMassA = localSphereRadiusA + localCenterMagnitudeA;

	PxBounds3 aabbB;
	convexB.getBounds(aabbB);
	PxReal  localSphereRadiusB = (aabbB.maximum - aabbB.minimum).magnitude() * 0.5f;
	PxReal  localCenterMagnitudeB = ((aabbB.maximum + aabbB.minimum) * 0.5f).magnitude();
	PxReal  maxRadiusFromCenterOfMassB = localSphereRadiusB + localCenterMagnitudeB;


	//maximum linear motion contribution of rotation(s):

	//project hulls in two directions orthogonal to angular motion
//	PxReal betterMaxLinMotionFromRotation = 0.0f;
	if (!motionA.angular.isZero())
		{
		PxReal maxRelevantRadius = getMaxRelevantRadius(&convexA, shape2worldA, motionA.angular, cache);
		//we want to have a better bound.  Its possible to be not better if we took the AABB in an unfavorable random space...
		if (maxRelevantRadius < maxRadiusFromCenterOfMassA)
			maxRadiusFromCenterOfMassA = maxRelevantRadius;
		}
	if (!motionB.angular.isZero())
		{
		PxReal maxRelevantRadius = getMaxRelevantRadius(&convexB, shape2worldB, motionB.angular, cache);
		if (maxRelevantRadius < maxRadiusFromCenterOfMassB)
			maxRadiusFromCenterOfMassB = maxRelevantRadius;		
		}
 
	PxReal maxLinMotionFromRotation = motionA.angular.magnitude() * maxRadiusFromCenterOfMassA + motionB.angular.magnitude() * maxRadiusFromCenterOfMassB;



	PxVec3 relativeLinMotion = motionB.linear - motionA.linear;

	PxReal lambda = PxReal(0.);
	PxVec3 v(1,0,0);

	int maxIter = MAX_GJK_SWEEP_ITERATIONS;

	PxVec3 n(PxReal(0.),PxReal(0.),PxReal(0.));

	PxReal lastLambda = lambda;

	int numIter = 0;
	//first solution, using GJK

	PointCollector	pointCollector1;
	GjkPairDetector gjk(&convexA,&convexB,&simplexSolver, PxVec3(0.0f, 0.0f, 1.0f));			//NEW: reuse gjk object so that we warmstart with a persistent axis.
	{
		
		GjkPairDetector::ClosestPointInput input;
	
		input.m_transformA = shape2worldA;
		input.m_transformB = shape2worldB;
		gjk.getClosestPoints(input,pointCollector1, cache);

	}


	PxTransform interpolatedTransA, interpolatedTransB;

	if (pointCollector1.m_hasResult)
		{
		PxReal dist = pointCollector1.m_distance;
		destNormal = n = pointCollector1.m_normalOnBInWorld;
		destPoint = pointCollector1.m_pointInWorldB;

		//not close enough
		while (dist > distTreshold)
		{
			numIter++;
			if (numIter > maxIter)
			{
				//printf("GJK sweep infinite loop!\n");
				//PX_ASSERT(0);
				toi = -2.0f;	//signal error
				return true; //todo: report a failure
			}
			PxReal linearMotionAlongSeparationVector = (relativeLinMotion).dot(n);

			PxReal maxMotion;
			/*
			if (numIter > 1)	//have a better bound
				{

				PxReal betterRotationTermBound = 0.0f;

				if (!motionB.angular.isZero())
					{
					PxVec3 worldSupportDirB = n.cross(motionB.angular);
					PxVec3 worldSupportDirBLocal = interpolatedTransB.rotateInv(worldSupportDirB);
					//note how we just rotate out the support direction,rather than transforming it out!
					PxVec3 maximalPointBLocal = //interpolatedTransB.rotate 
							(convexB->localGetSupportingVertex(-worldSupportDirBLocal));
					betterRotationTermBound = worldSupportDirBLocal.dot(maximalPointBLocal);
					}

				if (!motionA.angular.isZero())
					{
					PxVec3 worldSupportDirA = n.cross(motionA.angular);
					PxVec3 worldSupportDirALocal = interpolatedTransA.rotateInv(worldSupportDirA);
					PxVec3 maximalPointALocal = //interpolatedTransA.rotate
						(convexA->localGetSupportingVertex(-worldSupportDirALocal));//NB: we want the greatest dot product, and this funct returns the smallest, so we negate the query vector!
					//I have a feeling this may be a minus for reasons of symmetry:
					betterRotationTermBound  -= worldSupportDirALocal.dot(maximalPointALocal);
					}

				maxMotion = linearMotionAlongSeparationVector + betterRotationTermBound;
				}
			else 
			*/
				maxMotion = linearMotionAlongSeparationVector + maxLinMotionFromRotation;

			if (maxMotion <= PX_GJK_REL_ERROR2)
				{
				//no motion -> no sweep -> no contact!
				return false;
				}

			//calculate safe moving fraction from distance / (linear+rotational velocity)
			
			PxReal dLambda = (dist - distTreshold * 0.5f)  / maxMotion;	//note: we substract some skin from the distance to prevent getting too close!  That is to prevent accidentally moving into penetration and calling a distance check on that scenario.
																			//we subtract only half of it here because thats the distance we will typically end up with, and we want to reliably pass the less than distTreshold check above!

			lambda = lambda + dLambda;

			if (lambda > PxReal(1.))	//far enough away to execute full motion
				return false;

			if (lambda < PxReal(0.))	//moving away from eachother!?
				return false;


			//todo: next check with relative epsilon
			if (lambda <= lastLambda)
			{
				//printf("GJK sweep not making forward progress!\n");	//should not happen!
				PX_ASSERT(0);
				toi = -2.0f;	//signal error
				return true;
			}
			lastLambda = lambda;

			

			//execute part of motion that we found to be safe!
			//interpolate to next lambda

			PxQuat unused;
			Ps::integrateTransform(shape2worldA,motionA.linear,motionA.angular,lambda,interpolatedTransA);
			Ps::integrateTransform(shape2worldB,motionB.linear,motionB.angular,lambda,interpolatedTransB);


			PointCollector	pointCollector;
			//GjkPairDetector gjk(convexA,convexB,&simplexSolver, PxVec3(0.0f, 0.0f, 1.0f));
			GjkPairDetector::ClosestPointInput input;
			input.m_transformA = interpolatedTransA;
			input.m_transformB = interpolatedTransB;
			gjk.getClosestPoints(input,pointCollector, cache);

			if (!pointCollector.m_hasResult || (pointCollector.m_distance < distTreshold))
			{
				//printf("degenerate or penetrating scenario!\n");	//get rid of this case by not sweeping quite so far!
				//return last valid distance as result.  Though that might get us stuck if this case persists!?
				//TODO: also signal an error in this case!??
				toi = lastLambda;
				return true;
			}

			destPoint = pointCollector.m_pointInWorldB;
			destNormal = n = pointCollector.m_normalOnBInWorld;
			dist = pointCollector.m_distance;

		}
		//it's possible to get here even without executing a single iter of the above advancement loop --
		//in that case the initial distance check is under the skin distance we want to maintain...we can't sweep forward but at least we have a distance:
		toi = lambda;
		return true;
	}
	else
	{
		//printf("sweep failed initial distance check: probably initially penetrating config!\n");	//Fall back to pierre's penetration based code here.
		toi = -1.0f;
		return true;
	}
}
#endif