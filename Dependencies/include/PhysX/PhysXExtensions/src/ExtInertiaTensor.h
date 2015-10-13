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


#ifndef PX_PHYSICS_EXTENSIONS_INERTIATENSOR_H
#define PX_PHYSICS_EXTENSIONS_INERTIATENSOR_H

#include "CmPhysXCommon.h"
#include "PxGeometry.h"
#include "PxBoxGeometry.h"
#include "PxSphereGeometry.h"
#include "PxCapsuleGeometry.h"
#include "PxConvexMeshGeometry.h"
#include "PxConvexMesh.h"
#include "PxMat33Legacy.h"
#include "PsMathUtils.h"
#include "PsInlineArray.h"

namespace physx
{
namespace Ext
{
	class MassProps
	{
	public:
		// mass props of geometry at density 1, mass & inertia scale linearly with density

		MassProps(const PxGeometry &geometry): 
			mCom(0)
		{
			switch(geometry.getType())
			{
			case PxGeometryType::eSPHERE:
				{
					const PxSphereGeometry &s = static_cast<const PxSphereGeometry &>(geometry);
					mMass = (4.0f/3.0f) * PxPi * s.radius * s.radius * s.radius;
					mInertia = PxMat33::createDiagonal(PxVec3(2.0f/5.0f * mMass * s.radius * s.radius));
					break;
				}

			case PxGeometryType::eBOX:
				{
					const PxBoxGeometry &b = static_cast<const PxBoxGeometry &>(geometry);
					mMass = b.halfExtents.x * b.halfExtents.y * b.halfExtents.z * 8.0f;
					PxVec3 d2 = b.halfExtents.multiply(b.halfExtents);
					mInertia = PxMat33::createDiagonal(PxVec3(d2.y+d2.z, d2.x+d2.z, d2.x+d2.y)) * (mMass * 2.0f/3.0f);
					break;
				}

			case PxGeometryType::eCAPSULE:
				{
					const PxCapsuleGeometry &c = static_cast<const PxCapsuleGeometry &>(geometry);
					PxReal r = c.radius, h = c.halfHeight;
					mMass = ((4.0f/3.0f) * r + 2 * c.halfHeight) * PxPi * r * r;

					PxReal a = r*r*r * (8.0f/15.0f) + h*r*r * (3.0f/2.0f) + h*h*r * (4.0f/3.0f) + h*h*h * (2.0f/3.0f);
					PxReal b = r*r*r * (8.0f/15.0f) + h*r*r;				
					mInertia = PxMat33::createDiagonal(PxVec3(b,a,a) * PxPi * r * r);

					break;
				}

			case PxGeometryType::eCONVEXMESH:
				{
					const PxConvexMeshGeometry &c = static_cast<const PxConvexMeshGeometry &>(geometry);
					c.convexMesh->getMassInformation(mMass, mInertia, mCom);
					
					const PxMeshScale &s = c.scale;
					mMass *= (s.scale.x * s.scale.y * s.scale.z);
					mCom = s.rotation.rotate(s.scale.multiply(s.rotation.rotateInv(mCom)));
					mInertia = scaleInertia(mInertia, s.rotation, s.scale);
					break;
				}
			}
		}

		MassProps(PxReal mass, const PxMat33& inertia, const PxVec3 com):
		  mMass(mass), mInertia(inertia), mCom(com)	{}

		MassProps operator*(PxReal scale)					const
		{
			return MassProps(mMass * scale, mInertia * scale, mCom);
		}

		static MassProps sum(MassProps *props, PxTransform *tm, PxU32 count)
		{
			PxReal	mass = 0;
			PxVec3	com(0);
			PxMat33 inertia = PxMat33::createZero();
			
			Ps::InlineArray<PxVec3, 16> comArray; 
			comArray.reserve(count);

			for(PxU32 i=0;i<count;i++)
			{
				mass += props[i].mMass;
				comArray[i] = tm[i].transform(props[i].mCom);
				com += comArray[i] * props[i].mMass;
			}
			com /= mass;

			for(PxU32 i=0;i<count;i++)
				inertia += translateInertia(rotateInertia(props[i].mInertia, tm[i].q), props[i].mMass, com - comArray[i]);

			return MassProps(mass, inertia, com);
		}

		PxReal getMass()									const
		{
			return mMass;
		}

		PxVec3 getMassSpaceInertia(PxQuat &massFrame) const
		{
			massFrame = diagonalize(mInertia);

			PxMat33 axes(massFrame);
			PxMat33 d = axes * mInertia * axes.getTranspose();
			return PxVec3(d[0][0], d[1][1], d[2][2]);
		}

	//private:

		// rotate the inertia around the center of mass
		static PxMat33 rotateInertia(const PxMat33 inertia, const PxQuat &q)
		{
			PxMat33 m(q);
			return m * inertia * m.getTranspose();
		}

		// translate from CoM frame to a relative frame using parallel axis theorem
		static PxMat33 translateInertia(const PxMat33 &inertia, const PxReal mass, const PxVec3 &v)
		{
			PxMat33 s = Ps::star(v);
			return inertia + mass * s * s.getTranspose();
		}

		static PxMat33 scaleInertia(const PxMat33 inertia, const PxQuat &scaleRotation, const PxVec3 scale)
		{
			PxMat33 localInertia = rotateInertia(inertia, scaleRotation);		// rotate inertia into scaling frame
			PxVec3 diagonal(localInertia[0][0], localInertia[1][1], localInertia[2][2]);

			PxVec3 xyz2 = PxVec3(diagonal.dot(PxVec3(0.5f))) - diagonal;						// original x^2, y^2, z^2
			PxVec3 scaledxyz2 = xyz2.multiply(scale).multiply(scale);

			PxReal xx = scaledxyz2.y + scaledxyz2.z,
				   yy = scaledxyz2.z + scaledxyz2.x,
				   zz = scaledxyz2.x + scaledxyz2.y;

			PxReal xy = localInertia[0][1] * scale.x * scale.y,
			       xz = localInertia[0][2] * scale.x * scale.z,
				   yz = localInertia[1][2] * scale.y * scale.z;

			PxMat33 scaledInertia(PxVec3(xx, xy, xz),
								  PxVec3(xy, yy, yz),
								  PxVec3(xz, yz, zz));

			return rotateInertia(scaledInertia * (scale.x * scale.y * scale.z), scaleRotation.getConjugate());
		}


		static PxQuat diagonalize(const PxMat33 &m)
		{
			// jacobi rotation using quaternions (from Numerical Recipes and an idea of Stan Melax)

			static const PxU32 MAX_ITERS = 24;
			static const PxReal COS_TINY_ROTATION = 0.99999f;

			PxQuat q = PxQuat::createIdentity();

			for(PxU32 i=0; i < MAX_ITERS;i++)
			{
				PxMat33 axes(q);
				PxMat33 d = axes * m * axes.getTranspose();
				
				PxReal d12 = PxAbs(d[1][2]), d20 = PxAbs(d[0][2]), d01 = PxAbs(d[0][1]);

				PxU32 index = d12 > d20 && d12 > d01 ? 0 : d20 > d01 ? 1 : 2;				// rotation axis index, from largest off-diagonal element
				PxU32 i1 = (index+1)%3, i2 = (index+2)%3;											

				PxReal theta = (d[i2][i2]-d[i1][i1])/(2.0f*d[i1][i2]);						// cot(2 * phi), where phi is the rotation angle
				PxReal sign = PxSign(theta);
				PxReal t = theta<1e10f ? sign / (PxAbs(theta) + PxSqrt(theta*theta+1))
									   : 1 / (theta * 2);									// tan(phi)

				PxReal c = 1/PxSqrt(t*t+1);													// cos(phi)
				if(c>=COS_TINY_ROTATION)
					break;

				PxReal s = PxSqrt((1-c)/2);													// sin(phi/2)
				PxQuat r(0,0,0,PxSqrt(1-s*s));
				r.getImaginaryPart()[index] = -sign*s;

				q = (q*r).getNormalized();  
			}

			return q;
		}
		

		PxMat33 mInertia;
		PxVec3  mCom;
		PxReal	mMass;
	};



	class InertiaTensorComputer
	{
		public:
									InertiaTensorComputer(bool initTozero = true);
									InertiaTensorComputer(const PxMat33Legacy& inertia, const PxVec3& com, PxReal mass);
									~InertiaTensorComputer();

		PX_INLINE	void			zero();																//sets to zero mass
		PX_INLINE	void			setDiagonal(PxReal mass, const PxVec3& diagonal);					//sets as a diagonal tensor
		PX_INLINE	void			rotate(const PxMat33Legacy& rot);									//rotates the mass
					void			translate(const PxVec3& t);											//translates the mass
		PX_INLINE	void			transform(const PxTransform& transform);							//transforms the mass
		PX_INLINE	void			scaleDensity(PxReal densityScale);									//scales by a density factor
		PX_INLINE	void			add(const InertiaTensorComputer& it);								//adds a mass
		PX_INLINE	void			center();															//recenters inertia around center of mass

					void			setBox(const PxVec3& halfWidths);									//sets as an axis aligned box
		PX_INLINE	void			setBox(const PxVec3& halfWidths, const PxTransform* pose);			//sets as an oriented box
					void			addBox(PxReal density, const PxVec3& halfWidths, const PxTransform* pose = 0);//adds an oriented box mass to the current state

					void			setSphere(PxReal radius);
		PX_INLINE	void			setSphere(PxReal radius, const PxTransform* pose);
					void			addSphere(PxReal density, PxReal radius, const PxTransform* pose = 0);

					void			setCylinder(int dir, PxReal r, PxReal l);
		PX_INLINE	void			setCylinder(int dir, PxReal r, PxReal l, const PxTransform* pose);
					void			addCylinder(PxReal density, int dir, PxReal r, PxReal l, const PxTransform* pose = 0);

					void			setCapsule(int dir, PxReal r, PxReal l);
		PX_INLINE	void			setCapsule(int dir, PxReal r, PxReal l, const PxTransform* pose);
					void			addCapsule(PxReal density, int dir, PxReal r, PxReal l, const PxTransform* pose = 0);

					void			setEllipsoid(PxReal rx, PxReal ry, PxReal rz);
		PX_INLINE	void			setEllipsoid(PxReal rx, PxReal ry, PxReal rz, const PxTransform* pose);
					void			addEllipsoid(PxReal density, PxReal rx, PxReal ry, PxReal rz, const PxTransform* pose = 0);

		PX_INLINE	PxVec3			getCenterOfMass()				const	{ return mG;	}
		PX_INLINE	PxReal			getMass()						const	{ return mMass;	}
		PX_INLINE	PxMat33Legacy	getInertia()					const	{ return mI;	}

		private:
					PxMat33Legacy		mI;
					PxVec3				mG;
					PxReal				mMass;
	};


	//--------------------------------------------------------------
	//
	// Helper routines
	//
	//--------------------------------------------------------------

	// Special version allowing 2D quads
	PX_INLINE PxReal volume(const PxVec3& extents)
	{
		PxReal v = 1.0f;
		if(extents.x != 0.0f)	v*=extents.x;
		if(extents.y != 0.0f)	v*=extents.y;
		if(extents.z != 0.0f)	v*=extents.z;
		return v;
	}

	// Sphere
	PX_INLINE PxReal computeSphereRatio(PxReal radius)						{ return (4.0f/3.0f) * PxPi * radius * radius * radius;	}
	PxReal computeSphereMass(PxReal radius, PxReal density)					{ return density * computeSphereRatio(radius);				}
	PxReal computeSphereDensity(PxReal radius, PxReal mass)					{ return mass / computeSphereRatio(radius);					}

	// Box
	PX_INLINE PxReal computeBoxRatio(const PxVec3& extents)					{ return volume(extents);									}
	PxReal computeBoxMass(const PxVec3& extents, PxReal density)			{ return density * computeBoxRatio(extents);				}
	PxReal computeBoxDensity(const PxVec3& extents, PxReal mass)			{ return mass / computeBoxRatio(extents);					}

	// Ellipsoid
	PX_INLINE PxReal computeEllipsoidRatio(const PxVec3& extents)			{ return (4.0f/3.0f) * PxPi * volume(extents);		}
	PxReal computeEllipsoidMass(const PxVec3& extents, PxReal density)		{ return density * computeEllipsoidRatio(extents);			}
	PxReal computeEllipsoidDensity(const PxVec3& extents, PxReal mass)		{ return mass / computeEllipsoidRatio(extents);				}

	// Cylinder
	PX_INLINE PxReal computeCylinderRatio(PxReal r, PxReal l)				{ return PxPi * r  * r * (2.0f*l);					}
	PxReal computeCylinderMass(PxReal r, PxReal l, PxReal density)			{ return density * computeCylinderRatio(r, l);				}
	PxReal computeCylinderDensity(PxReal r, PxReal l, PxReal mass)			{ return mass / computeCylinderRatio(r, l);					}

	// Capsule
	PX_INLINE PxReal computeCapsuleRatio(PxReal r, PxReal l)				{ return computeSphereRatio(r) + computeCylinderRatio(r, l);}
	PxReal computeCapsuleMass(PxReal r, PxReal l, PxReal density)			{ return density * computeCapsuleRatio(r, l);				}
	PxReal computeCapsuleDensity(PxReal r, PxReal l, PxReal mass)			{ return mass / computeCapsuleRatio(r, l);					}

	// Cone
	PX_INLINE PxReal computeConeRatio(PxReal r, PxReal l)					{ return PxPi * r * r * PxAbs(l)/3.0f;			}
	PxReal computeConeMass(PxReal r, PxReal l, PxReal density)				{ return density * computeConeRatio(r, l);					}
	PxReal computeConeDensity(PxReal r, PxReal l, PxReal mass)				{ return mass / computeConeRatio(r, l);						}

	void computeBoxInertiaTensor(PxVec3& inertia, PxReal mass, PxReal xlength, PxReal ylength, PxReal zlength);
	void computeSphereInertiaTensor(PxVec3& inertia, PxReal mass, PxReal radius, bool hollow);
	bool jacobiTransform(PxI32 n, PxF64 a[], PxF64 w[]);
	bool diagonalizeInertiaTensor(const PxMat33Legacy& denseInertia, PxVec3& diagonalInertia, PxMat33Legacy& rotation);

} // namespace Ext

void Ext::computeBoxInertiaTensor(PxVec3& inertia, PxReal mass, PxReal xlength, PxReal ylength, PxReal zlength)
{
	//to model a hollow block, one would have to multiply coeff by up to two.
	const PxReal coeff = mass/12;
	inertia.x = coeff * (ylength*ylength + zlength*zlength);
	inertia.y = coeff * (xlength*xlength + zlength*zlength);
	inertia.z = coeff * (xlength*xlength + ylength*ylength);

	PX_ASSERT(inertia.x != 0.0);
	PX_ASSERT(inertia.y != 0.0);
	PX_ASSERT(inertia.z != 0.0);
	PX_ASSERT(inertia.isFinite());
}


void Ext::computeSphereInertiaTensor(PxVec3& inertia, PxReal mass, PxReal radius, bool hollow)
{
	inertia.x = mass * radius * radius;
	if (hollow) 
		inertia.x *= PxReal(2 / 3.0);
	else
		inertia.x *= PxReal(2 / 5.0);

	inertia.z = inertia.y = inertia.x;
	PX_ASSERT(inertia.isFinite());
}


// Diagonalize a matrix
bool Ext::jacobiTransform(PxI32 n, PxF64 a[], PxF64 w[])
{
	const PxF64	TINY_		= 1E-20f;
	const PxF64	EPS			= 1E-6f;
	const PxI32 MAX_ITER	= 100;

#define	rotate(a, i, j, k, l) {		\
	PxF64 x=a[i*n+j], y=a[k*n+l];	\
	a[i*n+j] = x*c - y*s;			\
	a[k*n+l] = x*s + y*c;			\
		}

	PxF64	t, c, s, tolerance, offdiag;

	s = offdiag = 0;
	for(PxI32 j=0;j<n;j++)
	{
		PxI32 k;
		for(k=0;k<n;k++) 
			w[j*n+k] = 0;

		w[j*n+j] = 1;
		s += a[j*n+j] * a[j*n+j];

		for(k=j+1;k<n;k++)	
			offdiag += a[j*n+k] * a[j*n+k];
	}
	tolerance = EPS * EPS * (s / 2 + offdiag);

	for(PxI32 iter=0;iter<MAX_ITER;iter++)
	{
		offdiag = 0;
		PxI32 j;
		for(j=0;j<n-1;j++)
		{
			for(PxI32 k=j+1;k<n;k++)
			{
				offdiag += a[j*n+k] * a[j*n+k];
			}
		}

		if(offdiag < tolerance)	return true;

		for(j=0; j<n-1; j++)
		{
			for(PxI32 k=j+1; k<n; k++)
			{
				if(fabs(a[j*n+k]) < TINY_) continue;

				t = (a[k*n+k] - a[j*n+j]) / (2 * a[j*n+k]);
				s = PxSqrt(t * t + 1);
				t = 1 / (t + (t>=0 ? s : -s));

				c = PxRecipSqrt(t * t + 1);
				s = t * c;
				t *= a[j*n+k];
				a[j*n+j] -= t;
				a[k*n+k] += t;
				a[j*n+k] = 0;
				PxI32 i;
				for(i=0;i<j;i++)	
					rotate(a, i, j, i, k);
				for(i=j+1;i<k;i++)		
					rotate(a, j, i, i, k);
				for(i=k+1;i<n;i++)		
					rotate(a, j, i, k, i);
				for(i=0;i<n;i++)		
					rotate(w, j, i, k, i);
			}
		}
	}
#undef	rotate
	return false;
}


bool Ext::diagonalizeInertiaTensor(const PxMat33Legacy& denseInertia, PxVec3& diagonalInertia, PxMat33Legacy& rotation)
{
	// We just convert to doubles temporarily, for higher precision
	double A[3*3], R[3*3];

	A[0*3+0]=denseInertia(0,0);	A[0*3+1]=denseInertia(1,0);	A[0*3+2]=denseInertia(2,0);
	A[1*3+0]=denseInertia(0,1);	A[1*3+1]=denseInertia(1,1);	A[1*3+2]=denseInertia(2,1);
	A[2*3+0]=denseInertia(0,2);	A[2*3+1]=denseInertia(1,2);	A[2*3+2]=denseInertia(2,2);

	// Here we diagonalize the inertia tensor
	if(!jacobiTransform( 3, A, R ))
	{
		// Can't diagonalize inertia tensor!

		// Setups a default tensor in sake of robustness
		diagonalInertia = PxVec3(1,1,1);
		return false;
	}	
	else
	{
		// Save eigenvalues
		diagonalInertia = PxVec3( (float)A[0*3+0], (float)A[1*3+1], (float)A[2*3+2] );
		PX_ASSERT(diagonalInertia.isFinite());

		rotation(0,0) = (float)R[0*3+0];	rotation(0,1) = (float)R[1*3+0];	rotation(0,2) = (float)R[2*3+0];
		rotation(1,0) = (float)R[0*3+1];	rotation(1,1) = (float)R[1*3+1];	rotation(1,2) = (float)R[2*3+1];
		rotation(2,0) = (float)R[0*3+2];	rotation(2,1) = (float)R[1*3+2];	rotation(2,2) = (float)R[2*3+2];
		/*
		rotation(0,0) = (float)R[0*3+0];	rotation(1,0) = (float)R[1*3+0];	rotation(2,0) = (float)R[2*3+0];
		rotation(0,1) = (float)R[0*3+1];	rotation(1,1) = (float)R[1*3+1];	rotation(2,1) = (float)R[2*3+1];
		rotation(0,2) = (float)R[0*3+2];	rotation(1,2) = (float)R[1*3+2];	rotation(2,2) = (float)R[2*3+2];
		*/
		return true;
	}	
}


//--------------------------------------------------------------
//
// InertiaTensorComputer implementation
//
//--------------------------------------------------------------

Ext::InertiaTensorComputer::InertiaTensorComputer(bool initTozero)
{
	if (initTozero)
		zero();
}


Ext::InertiaTensorComputer::InertiaTensorComputer(const PxMat33Legacy& inertia, const PxVec3& com, PxReal mass) :
	mI(inertia),
	mG(com),
	mMass(mass)
{
}


Ext::InertiaTensorComputer::~InertiaTensorComputer()
{
	//nothing
}


PX_INLINE void Ext::InertiaTensorComputer::zero()
{
	mMass = 0.0f;
	mI.setZero();
	mG = PxVec3(0);
}


PX_INLINE void Ext::InertiaTensorComputer::setDiagonal(PxReal mass, const PxVec3& diag)
{
	mMass = mass;
	mI.setDiagonal(diag);
	mG = PxVec3(0);
	PX_ASSERT(mI.isFinite());
	PX_ASSERT(PxIsFinite(mMass));
}


void Ext::InertiaTensorComputer::setBox(const PxVec3& halfWidths)
{
	// Setup inertia tensor for a cube with unit density
	const PxReal mass = 8.0f * computeBoxRatio(halfWidths);
	const PxReal s =(1.0f/3.0f) * mass;

	const PxReal x = halfWidths.x*halfWidths.x;
	const PxReal y = halfWidths.y*halfWidths.y;
	const PxReal z = halfWidths.z*halfWidths.z;

	setDiagonal(mass, PxVec3(y+z, z+x, x+y) * s);
}


PX_INLINE void Ext::InertiaTensorComputer::rotate(const PxMat33Legacy& rot)
{
	//well known inertia tensor rotation expression is: RIR' -- this could be optimized due to symmetry, see code to do that in Body::updateGlobalInverseInertia
	mI.setMultiplyTransposeRight(rot * mI, rot);
	PX_ASSERT(mI.isFinite());
	//com also needs to be rotated
	rot.multiply(mG, mG);
	PX_ASSERT(mG.isFinite());
}


void Ext::InertiaTensorComputer::translate(const PxVec3& t)
{
	if (!t.isZero())	//its common for this to be zero
	{
		PxMat33Legacy t1, t2;

		t1.setStar(mG);

		PxVec3 sum = mG + t;
		if (sum.isZero())
		{
			mI += (t1 * t1)*mMass;
		}
		else
		{
			t2.setStar(sum);
			mI += (t1 * t1 - t2 * t2)*mMass;
		}

		//move center of mass
		mG += t;

		PX_ASSERT(mI.isFinite());
		PX_ASSERT(mG.isFinite());
	}
}


PX_INLINE void Ext::InertiaTensorComputer::transform(const PxTransform& transform)
{
	rotate(PxMat33Legacy(transform.q));
	translate(transform.p);
}


PX_INLINE void Ext::InertiaTensorComputer::setBox(const PxVec3& halfWidths, const PxTransform* pose)
{
	setBox(halfWidths);
	if (pose)
		transform(*pose);

}


PX_INLINE void Ext::InertiaTensorComputer::scaleDensity(PxReal densityScale)
{
	mI *= densityScale;
	mMass *= densityScale;
	PX_ASSERT(mI.isFinite());
	PX_ASSERT(PxIsFinite(mMass));
}


PX_INLINE void Ext::InertiaTensorComputer::add(const InertiaTensorComputer& it)
{
	const PxReal TotalMass = mMass + it.mMass;
	mG = (mG * mMass + it.mG * it.mMass) / TotalMass;

	mMass = TotalMass;
	mI += it.mI;
	PX_ASSERT(mI.isFinite());
	PX_ASSERT(mG.isFinite());
	PX_ASSERT(PxIsFinite(mMass));
}


void Ext::InertiaTensorComputer::addBox(PxReal density, const PxVec3& halfWidths, const PxTransform* pose)
{
	InertiaTensorComputer i(false);
	i.setBox(halfWidths, pose);
	if (density != 1.0f)
		i.scaleDensity(density);
	add(i);
}


PX_INLINE void Ext::InertiaTensorComputer::center()
{
	PxVec3 center = -mG;
	translate(center);
}


void Ext::InertiaTensorComputer::setSphere(PxReal radius)
{
	// Compute mass of the sphere
	const PxReal m = computeSphereRatio(radius);
	// Compute moment of inertia
	const PxReal s = m * radius * radius * (2.0f/5.0f);
	setDiagonal(m,PxVec3(s,s,s));
}


PX_INLINE void Ext::InertiaTensorComputer::setSphere(PxReal radius, const PxTransform* pose)
{
	setSphere(radius);
	if (pose)
		transform(*pose);
}


void Ext::InertiaTensorComputer::addSphere(PxReal density, PxReal radius, const PxTransform* pose)
{
	InertiaTensorComputer i(false);
	i.setSphere(radius, pose);
	if (density != 1.0f)
		i.scaleDensity(density);
	add(i);
}


void Ext::InertiaTensorComputer::setCylinder(int dir, PxReal r, PxReal l)
{
	// Compute mass of cylinder
	const PxReal m = computeCylinderRatio(r, l);

	const PxReal i1 = r*r*m/2.0f;
	const PxReal i2 = (3.0f*r*r+4.0f*l*l)*m/12.0f;

	switch(dir)
	{
	case 0:		setDiagonal(m,PxVec3(i1,i2,i2));	break;
	case 1:		setDiagonal(m,PxVec3(i2,i1,i2));	break;
	default:	setDiagonal(m,PxVec3(i2,i2,i1));	break;
	}
}


PX_INLINE void Ext::InertiaTensorComputer::setCylinder(int dir, PxReal r, PxReal l, const PxTransform* pose)
{
	setCylinder(dir, r, l);
	if (pose)
		transform(*pose);
}


void Ext::InertiaTensorComputer::addCylinder(PxReal density, int dir, PxReal r, PxReal l, const PxTransform* pose)
{
	InertiaTensorComputer i(false);
	i.setCylinder(dir,r,l,pose);
	if (density != 1.0f)
		i.scaleDensity(density);
	add(i);
}


void Ext::InertiaTensorComputer::setCapsule(int dir, PxReal r, PxReal l)
{
	// Compute mass of capsule
	const PxReal m = computeCapsuleRatio(r, l);

	const PxReal t  = PxPi * r * r;
	const PxReal i1 = t * ((r*r*r * 8.0f/15.0f) + (l*r*r));
	const PxReal i2 = t * ((r*r*r * 8.0f/15.0f) + (l*r*r * 3.0f/2.0f) + (l*l*r * 4.0f/3.0f) + (l*l*l * 2.0f/3.0f));

	switch(dir)
	{
	case 0:		setDiagonal(m,PxVec3(i1,i2,i2));	break;
	case 1:		setDiagonal(m,PxVec3(i2,i1,i2));	break;
	default:	setDiagonal(m,PxVec3(i2,i2,i1));	break;
	}
}


PX_INLINE void Ext::InertiaTensorComputer::setCapsule(int dir, PxReal r, PxReal l, const PxTransform* pose)
{
	setCapsule(dir, r, l);
	if (pose)
		transform(*pose);
}


void Ext::InertiaTensorComputer::addCapsule(PxReal density, int dir, PxReal r, PxReal l, const PxTransform* pose)
{
	InertiaTensorComputer i(false);
	i.setCapsule(dir,r,l,pose);
	if (density != 1.0f)
		i.scaleDensity(density);
	add(i);
}


void Ext::InertiaTensorComputer::setEllipsoid(PxReal rx, PxReal ry, PxReal rz)
{
	// Compute mass of ellipsoid
	const PxReal m = computeEllipsoidRatio(PxVec3(rx, ry, rz));

	// Compute moment of inertia
	const PxReal s = m * (2.0f/5.0f);

	// Setup inertia tensor for an ellipsoid centered at the origin
	setDiagonal(m,PxVec3(ry*rz,rz*rx,rx*ry)*s);
}


PX_INLINE void Ext::InertiaTensorComputer::setEllipsoid(PxReal rx, PxReal ry, PxReal rz, const PxTransform* pose)
{
	setEllipsoid(rx,ry,rz);
	if (pose)
		transform(*pose);
}


void Ext::InertiaTensorComputer::addEllipsoid(PxReal density, PxReal rx, PxReal ry, PxReal rz, const PxTransform* pose)
{
	InertiaTensorComputer i(false);
	i.setEllipsoid(rx,ry,rz,pose);
	if (density != 1.0f)
		i.scaleDensity(density);
	add(i);
}

}

#endif
