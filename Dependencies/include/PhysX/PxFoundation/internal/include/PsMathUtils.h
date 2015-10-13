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


#ifndef PX_FOUNDATION_PSMATHUTILS_H
#define PX_FOUNDATION_PSMATHUTILS_H

#include "PxTransform.h"
#include "PxMat33.h"
#include "Ps.h"
#include "PsIntrinsics.h"

// General guideline is: if it's an abstract math function, it belongs here.
// If it's a math function where the inputs have specific semantics (e.g.
// separateSwingTwist) it doesn't.

namespace physx
{
namespace shdfnd3
{
	/**
	\brief sign returns the sign of its argument. The sign of zero is undefined.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF32 sign(const PxF32 a)			{	return intrinsics::sign(a); }

	/**
	\brief sign returns the sign of its argument. The sign of zero is undefined.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF64 sign(const PxF64 a)			{	return (a >= 0.0) ? 1.0 : -1.0;		}

	/**
	\brief sign returns the sign of its argument. The sign of zero is undefined.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxI32 sign(const PxI32 a)			{	return (a >= 0) ? 1 : -1;			}

	/**
	\brief Returns true if the two numbers are within eps of each other.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE bool equals(const PxF32 a, const PxF32 b, const PxF32 eps){	return (PxAbs(a - b) < eps);	}

	/**
	\brief Returns true if the two numbers are within eps of each other.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE bool equals(const PxF64 a, const PxF64 b, const PxF64 eps){	return (PxAbs(a - b) < eps);	}

	/**
	\brief The floor function returns a floating-point value representing the largest integer that is less than or equal to x.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF32 floor(const PxF32 a)			{	return floatFloor(a);			}

	/**
	\brief The floor function returns a floating-point value representing the largest integer that is less than or equal to x.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF64 floor(const PxF64 a)			{	return ::floor(a);					}

	/**
	\brief The ceil function returns a single value representing the smallest integer that is greater than or equal to x. 
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF32 ceil(const PxF32 a)			{	return ::ceilf(a);	}

	/**
	\brief The ceil function returns a double value representing the smallest integer that is greater than or equal to x. 
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF64 ceil(const PxF64 a)			{	return ::ceil(a);	}

	/** 
	\brief mod returns the floating-point remainder of x / y. 

	If the value of y is 0.0, mod returns a quiet NaN.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF32 mod(const PxF32 x, const PxF32 y)		{	return (PxF32)::fmod(x,y);			}

	/**
	\brief mod returns the floating-point remainder of x / y. 

	If the value of y is 0.0, mod returns a quiet NaN.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF64 mod(const PxF64 x, const PxF64 y)		{	return ::fmod(x,y);					}

	/**
	\brief Square.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF32 sqr(const PxF32 a)					{	return a*a;		}

	/**
	\brief Square.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF64 sqr(const PxF64 a)					{	return a*a;	}

	/**
	\brief Calculates x raised to the power of y.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF32 pow(const PxF32 x, const PxF32 y)		{	return ::powf(x,y);	}

	/**
	\brief Calculates x raised to the power of y.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF64 pow(const PxF64 x, const PxF64 y)		{	return ::pow(x,y);	}

	/**
	\brief Calculates e^n
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF32 exp(const PxF32 a)				{	return ::exp(a);	}
	/**

	\brief Calculates e^n
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF64 exp(const PxF64 a)				{	return ::exp(a);	}

	/**
	\brief Calculates logarithms.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF32 logE(const PxF32 a)				{	return ::log(a);	}

	/**
	\brief Calculates logarithms.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF64 logE(const PxF64 a)				{	return ::log(a);	}

	/**
	\brief Calculates logarithms.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF32 log2(const PxF32 a)				{   return ::log(a) / 0.693147180559945309417f;	}

	/**
	\brief Calculates logarithms.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF64 log2(const PxF64 a)				{	return ::log(a) / 0.693147180559945309417;	}

	/**
	\brief Calculates logarithms.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF32 log10(const PxF32 a)				{	return (PxF32)::log10(a);	}

	/**
	\brief Calculates logarithms.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF64 log10(const PxF64 a)				{	return ::log10(a);	}

	/**
	\brief Converts degrees to radians.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF32 degToRad(const PxF32 a)			{	return (PxF32)0.01745329251994329547 * a;	}

	/**
	\brief Converts degrees to radians.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF64 degToRad(const PxF64 a)			{	return (PxF64)0.01745329251994329547 * a;	}

	/**
	\brief Converts radians to degrees.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF32 radToDeg(const PxF32 a)			{	return (PxF32)57.29577951308232286465 * a;	}

	/**
	\brief Converts radians to degrees.
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF64 radToDeg(const PxF64 a)			{	return (PxF64)57.29577951308232286465 * a;	}

	//! \brief compute sine and cosine at the same time. There is a 'fsincos' on PC that we probably want to use here
	PX_CUDA_CALLABLE static PX_FORCE_INLINE void sincos(const PxF32 radians, PxF32& sin, PxF32& cos)
	{
		/* something like:
		_asm fld  Local
		_asm fsincos
		_asm fstp LocalCos
		_asm fstp LocalSin
		*/
		sin = PxSin(radians);
		cos = PxCos(radians);
	}

	/**
	\brief uniform random number in [a,b]
	*/
	PX_FORCE_INLINE PxI32 rand(const PxI32 a, const PxI32 b)
	{
		return a + (PxI32)(::rand()%(b-a+1));
	}

	/**
	\brief uniform random number in [a,b]
	*/
	PX_FORCE_INLINE PxF32 rand(const PxF32 a, const PxF32 b)	
{
		return a + (b-a)*::rand()/RAND_MAX;
	}

	//! \brief return angle between two vectors in radians
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxF32 angle(const PxVec3& v0, const PxVec3& v1)
	{
		const PxF32 cos = v0.dot(v1);					// |v0|*|v1|*Cos(Angle)
		const PxF32 sin = (v0.cross(v1)).magnitude();	// |v0|*|v1|*Sin(Angle)
		return PxAtan2(sin, cos);
	}

	//! If possible use instead fsel on the dot product /*fsel(d.dot(p),onething,anotherthing);*/
	//! Compares orientations (more readable, user-friendly function)
	PX_CUDA_CALLABLE static PX_FORCE_INLINE bool sameDirection(const PxVec3& d, const PxVec3& p)
	{
		return d.dot(p) >= 0.0f;
	}

	//! Checks 2 values have different signs
	PX_CUDA_CALLABLE static PX_FORCE_INLINE IntBool differentSign(PxReal f0, PxReal f1)
	{
		union { PxU32 u; PxReal f; } u1, u2;
		u1.f = f0;
		u2.f = f1;
		return (u1.u^u2.u)&PX_SIGN_BITMASK;
	}

	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxMat33 star(const PxVec3 &v)
	{
		return PxMat33(PxVec3(0,v.z,-v.y),
					   PxVec3(-v.z,0,v.x),
					   PxVec3(v.y,-v.x,0));
	}

	PX_CUDA_CALLABLE static PX_INLINE PxVec3 log(const PxQuat& q)
	{
		const PxReal s = q.getImaginaryPart().magnitude();
		if(s<1e-12)
			return PxVec3(0.0f);
		// force the half-angle to have magnitude <= pi/2
		PxReal halfAngle = q.w<0 ? PxAtan2(-s,-q.w): PxAtan2(s,q.w);
		PX_ASSERT(halfAngle >= -PxPi/2 && halfAngle <= PxPi/2);

		return q.getImaginaryPart().getNormalized() * 2 * halfAngle;
	}

	PX_CUDA_CALLABLE static PX_INLINE PxQuat exp(const PxVec3& v)
	{
		const PxReal m = v.magnitudeSquared();
		return m<1e-24 ? PxQuat::createIdentity() : 
						 PxQuat(PxSqrt(m),v*PxRecipSqrt(m));
	}

	//! Computes the maximum delta to another transform
	PX_CUDA_CALLABLE static PX_INLINE PxReal maxComponentDelta(const PxTransform& t0, const PxTransform& t1)
	{
		PxReal delta =       PxAbs(t0.p.x - t1.p.x);
		delta = PxMax(delta, PxAbs(t0.p.y - t1.p.y));
		delta = PxMax(delta, PxAbs(t0.p.z - t1.p.z));
		delta = PxMax(delta, PxAbs(t0.q.x - t1.q.x));
		delta = PxMax(delta, PxAbs(t0.q.y - t1.q.y));
		delta = PxMax(delta, PxAbs(t0.q.z - t1.q.z));
		delta = PxMax(delta, PxAbs(t0.q.w - t1.q.w));

		return delta;
	}

	/**
	\brief returns largest axis
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxU32 largestAxis(const PxVec3& v)
	{
		PxU32 m = v.y > v.x ? 1 : 0;
		return v.z > v[m] ? 2 : m;
	}

	/**
	\brief returns axis with smallest absolute value
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxU32 closestAxis(const PxVec3& v)
	{
		PxU32 m = PxAbs(v.y) > PxAbs(v.x) ? 1 : 0;
		return PxAbs(v.z) > PxAbs(v[m]) ? 2 : m;
	}

	PX_CUDA_CALLABLE static PX_INLINE PxU32 closestAxis(const PxVec3& v, PxU32& j, PxU32& k)
	{
		// find largest 2D plane projection
		const PxF32 absPx = PxAbs(v.x);
		const PxF32 absNy = PxAbs(v.y);
		const PxF32 absNz = PxAbs(v.z);

		PxU32 m = 0;	//x biggest axis
		j = 1;
		k = 2;
		if( absNy > absPx && absNy > absNz)
		{
			//y biggest
			j = 2;
			k = 0;
			m = 1;
		}
		else if(absNz > absPx)
		{
			//z biggest
			j = 0;
			k = 1;
			m = 2;
		}
		return m;
	}

	/*!
	Extend an edge along its length by a factor
	*/
	PX_CUDA_CALLABLE static PX_FORCE_INLINE void makeFatEdge(PxVec3& p0, PxVec3& p1, PxReal fatCoeff)
	{
		PxVec3 delta = p1 - p0;

		const PxReal m = delta.magnitude();
		if(m>0.0f)
		{
			delta *= fatCoeff/m;
			p0 -= delta;
			p1 += delta;
		}
	}

	//! Compute point as combination of barycentric coordinates
	PX_CUDA_CALLABLE static PX_FORCE_INLINE PxVec3 computeBarycentricPoint(const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, PxReal u, PxReal v)
	{
		// This seems to confuse the compiler...
		// return (1.0f - u - v)*p0 + u*p1 + v*p2;
		const PxF32 w = 1.0f - u - v;
		return PxVec3(	w*p0.x + u*p1.x + v*p2.x,
						w*p0.y + u*p1.y + v*p2.y,
						w*p0.z + u*p1.z + v*p2.z);
	}

	// generates a pair of quaternions (swing, twist) such that in = swing * twist, with
	// swing.x = 0
	// twist.y = twist.z = 0, and twist is a unit quat
	PX_FORCE_INLINE void separateSwingTwist(const PxQuat& q, PxQuat& swing, PxQuat& twist)
	{
		twist = q.x != 0.0f ? PxQuat(q.x,0,0,q.w).getNormalized() : PxQuat::createIdentity();
		swing = q * twist.getConjugate();
	}

	// generate two tangent vectors to a given normal
	PX_FORCE_INLINE void normalToTangents(const PxVec3& normal, PxVec3& tangent0, PxVec3& tangent1)
	{
		tangent0 = PxAbs(normal.x) < 0.70710678f ? PxVec3(0,-normal.z,normal.y) : PxVec3(-normal.y,normal.x,0);
		tangent0.normalize();
		tangent1 = normal.cross(tangent0);
	}

	// todo: what is this function doing?
	PxQuat computeQuatFromNormal(const PxVec3& n);

	/**
	\brief computes a oriented bounding box around the scaled basis.
	\param basis Input = skewed basis, Output = (normalized) orthogonal basis.
	\return Bounding box extent.
	*/
	PxVec3 optimizeBoundingBox(PxMat33& basis);

	PxQuat slerp(const PxReal t, const PxQuat& left, const PxQuat& right);

	PX_INLINE PxVec3 ellipseClamp(const PxVec3& point, const PxVec3& radii)
	{
		//This function need to be implemented in the header file because 
		//it is included in a spu shader program.

		// finds the closest point on the ellipse to a given point

		// (p.y, p.z) is the input point
		// (e.y, e.z) are the radii of the ellipse

		// lagrange multiplier method with Newton/Halley hybrid root-finder.
		// see http://www.geometrictools.com/Documentation/DistancePointToEllipse2.pdf
		// for proof of Newton step robustness and initial estimate. 
		// Halley converges much faster but sometimes overshoots - when that happens we take
		// a newton step instead

		// converges in 1-2 iterations where D&C works well, and it's good with 4 iterations 
		// with any ellipse that isn't completely crazy

		const PxU32 MAX_ITERATIONS = 20;
		const PxReal convergenceThreshold = 1e-4f; 

		// iteration requires first quadrant but we recover generality later

		PxVec3 q(0,PxAbs(point.y),PxAbs(point.z));
		const PxReal tinyEps = (PxReal)(1e-6f);		// very close to minor axis is numerically problematic but trivial
		if(radii.y>=radii.z)
		{
			if(q.z<tinyEps)
				return PxVec3(0, point.y>0 ? radii.y : -radii.y, 0);
		}
		else
		{
			if(q.y<tinyEps)
				return PxVec3(0, 0, point.z>0 ? radii.z : -radii.z);	
		}

		PxVec3 denom, e2 = radii.multiply(radii), q2 = q.multiply(q), eq = radii.multiply(q);

		// we can use any initial guess which is > maximum(-e.y^2,-e.z^2) and for which f(t) is > 0. 
		// this guess works well near the axes, but is weak along the diagonals. 

		PxReal t = PxMax(eq.y-e2.y, eq.z-e2.z);

		for (PxU32 i = 0; i < MAX_ITERATIONS; i++)
		{
			denom = PxVec3(0,1/(t + e2.y), 1/(t + e2.z));
			PxVec3 denom2 = eq.multiply(denom);

			PxVec3 fv = denom2.multiply(denom2);
			PxReal f = fv.y + fv.z - 1;

			// although in exact arithmetic we are guaranteed f>0, we can get here
			// on the first iteration via catastrophic cancellation if the point is
			// very close to the origin. In that case we just behave as if f=0

			if (f < convergenceThreshold)
				return e2.multiply(point).multiply(denom);

			PxReal df = fv.dot(denom) * -2.0f;
			t = t - f/df;
		}

		// we didn't converge, so clamp what we have
		PxVec3 r = e2.multiply(point).multiply(denom);
		return r * PxRecipSqrt(sqr(r.y/radii.y) + sqr(r.z/radii.z));
	}

	PX_INLINE PxReal tanHalf(PxReal sin, PxReal cos)
	{
		return sin / (1+cos);
	}

	PX_INLINE PxQuat quatFromTanQVector(const PxVec3 &v)
	{
		PxReal v2 = v.dot(v);
		if(v2<1e-12f)
			return PxQuat::createIdentity();
		PxReal d = 1/(1+v2);
		return PxQuat(v.x*2, v.y*2, v.z*2, 1-v2)*d;
	}

	PX_FORCE_INLINE PxVec3 cross100(const PxVec3& b)	{ return PxVec3(0.0f, -b.z, b.y);	}
	PX_FORCE_INLINE PxVec3 cross010(const PxVec3& b)	{ return PxVec3(b.z, 0.0f, -b.x);	}
	PX_FORCE_INLINE PxVec3 cross001(const PxVec3& b)	{ return PxVec3(-b.y, b.x, 0.0f);	}
		
	//! \brief Return (i+1)%3
	// Avoid variable shift for XBox:
	// PX_INLINE PxU32 Ps::getNextIndex3(PxU32 i)			{	return (1<<i) & 3;			}
	PX_INLINE PxU32 getNextIndex3(PxU32 i)			{	return (i+1+(i>>1)) & 3;	}

	PX_INLINE PxMat33 rotFrom2Vectors(const PxVec3& from, const PxVec3& to)
	{
		// See bottom of http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/index.htm

		// Early exit if to = from
		if( (from - to).magnitudeSquared() < 1e-4f )
			return PxMat33::createIdentity();

		// Early exit if to = -from
		if( (from + to).magnitudeSquared() < 1e-4f )
			return PxMat33::createDiagonal(PxVec3(1.0f, -1.0f, -1.0f));

		PxVec3 n = from.cross(to);

		PxReal C = from.dot(to),
			S = PxSqrt(1 - C * C),
			CC = 1 - C;

		PxReal xx = n.x * n.x,
			yy = n.y * n.y,
			zz = n.z * n.z,
			xy = n.x * n.y,
			yz = n.y * n.z,
			xz = n.x * n.z;

		PxMat33 R;

		R(0,0) =  1 + CC * (xx - 1);
		R(0,1) = -n.z * S + CC * xy;
		R(0,2) =  n.y * S + CC * xz;

		R(1,0) =  n.z * S + CC * xy;
		R(1,1) =  1 + CC * (yy - 1);
		R(1,2) = -n.x * S + CC * yz;

		R(2,0) = -n.y * S + CC * xz;
		R(2,1) =  n.x * S + CC * yz;
		R(2,2) =  1 + CC * (zz - 1);

		return R;
	}

	void integrateTransform(const PxTransform& curTrans, const PxVec3& linvel, const PxVec3& angvel, PxReal timeStep, PxTransform& result);

} // namespace shdfnd3
} // namespace physx

#endif
