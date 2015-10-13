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

#include "PsFoundation.h"
#include "PsUtilities.h"
#include "PsMathUtils.h"

#include "PxSimpleFactory.h"
#include "PxRigidStatic.h"
#include "PxSphereGeometry.h"
#include "PxBoxGeometry.h"
#include "PxCapsuleGeometry.h"
#include "PxConvexMeshGeometry.h"
#include "PxPlaneGeometry.h"
#include "PxRigidBodyExt.h"
#include "PxRigidStatic.h"
#include "PxScene.h"
#include "PxShape.h"
#include "PxRigidDynamic.h"

using namespace physx;
using namespace physx::shdfnd3;

namespace
{
template<class A>
 A* setShape(A* actor, const PxGeometry& geometry, PxMaterial& material, const PxTransform &shapeOffset, PxShape*& shape)
{
	if(!actor)
		return NULL;

	shape = actor->createShape(geometry, material, shapeOffset);
	if(!shape)
	{ 
		actor->release();
		return NULL;
	}
	return actor;
}

bool isDynamicGeometry(const PxGeometry &geometry)
{
	return geometry.getType() == PxGeometryType::eBOX 
		|| geometry.getType() == PxGeometryType::eSPHERE
		|| geometry.getType() == PxGeometryType::eCAPSULE
		|| geometry.getType() == PxGeometryType::eCONVEXMESH;
}
}

PxRigidDynamic* PxCreateDynamic(PxPhysics& sdk, 
								const PxTransform& transform, 
								const PxGeometry& geometry,
							    PxMaterial& material, 
								PxReal density,
								const PxTransform &shapeOffset)
{
	PX_CHECK(transform.isValid());
	PX_CHECK(shapeOffset.isValid());

	if(!isDynamicGeometry(geometry) || density <= 0.0f)
	    return NULL;

	PxShape* shape;
	PxRigidDynamic* actor = setShape(sdk.createRigidDynamic(transform), geometry, material, shapeOffset, shape);
	if(actor)
		PxRigidBodyExt::updateMassAndInertia(*actor, density);
	return actor;
}


PxRigidDynamic* PxCreateKinematic(PxPhysics& sdk, 
								  const PxTransform& transform, 
								  const PxGeometry& geometry, 
								  PxMaterial& material,
								  PxReal density,
								  const PxTransform &shapeOffset)
{
	PX_CHECK(transform.isValid()); PX_CHECK(shapeOffset.isValid());

	bool isDynGeom = isDynamicGeometry(geometry);
	if(isDynGeom && density <= 0.0f)
	    return NULL;

	PxShape* shape;
	PxRigidDynamic* actor = setShape(sdk.createRigidDynamic(transform), geometry, material, shapeOffset, shape);
	
	if(actor)
	{
		actor->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);

		if(isDynGeom)
			PxRigidBodyExt::updateMassAndInertia(*actor, density);
		else		
		{
			shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
			actor->setMass(1);
			actor->setMassSpaceInertiaTensor(PxVec3(1,1,1));
		}
	}
	return actor;
}


PxRigidStatic* PxCreateStatic(PxPhysics& sdk, 
							  const PxTransform &transform, 
							  const PxGeometry &geometry, 
							  PxMaterial &material,
							  const PxTransform &shapeOffset)
{
	PX_CHECK(transform.isValid()); PX_CHECK(shapeOffset.isValid());

	PxShape* shape;
	return setShape(sdk.createRigidStatic(transform), geometry, material, shapeOffset, shape);
}

PxRigidStatic* PxCreatePlane(physx::PxPhysics& sdk,
							 const PxVec3 &outwardNormal,
							 const PxReal distance,
							 PxMaterial& material)
{
	PX_CHECK(outwardNormal.isFinite());

	if (!outwardNormal.isNormalized())
		return NULL;

	// Check for Plane normals as follows:
	// 0: PxVec3(-1.0f,  0.0f,  0.0f);
	// 1: PxVec3( 1.0f,  0.0f,  0.0f);
	// 2: PxVec3( 0.0f, -1.0f,  0.0f);
	// 3: PxVec3( 0.0f,  1.0f,  0.0f);
	// 4: PxVec3( 0.0f,  0.0f, -1.0f);
	// 5: PxVec3( 0.0f,  0.0f,  1.0f);
	PxU32 bitmap = 0;

	if(outwardNormal.z ==  0.0f) bitmap |= 0x001;
	else if(outwardNormal.z ==  1.0f) bitmap |= 0x002;
	else if(outwardNormal.z == -1.0f) bitmap |= 0x004;
	if(outwardNormal.y ==  0.0f) bitmap |= 0x010;
	else if(outwardNormal.y ==  1.0f) bitmap |= 0x020;
	else if(outwardNormal.y == -1.0f) bitmap |= 0x040;
	if(outwardNormal.x ==  0.0f) bitmap |= 0x100;
	else if(outwardNormal.x ==  1.0f) bitmap |= 0x200;
	else if(outwardNormal.x == -1.0f) bitmap |= 0x400;

	PxTransform transform;
	switch(bitmap)
	{
		case 0x411:
			transform.q = PxQuat(PxPi, PxVec3(0.0f, 0.0f, 1.0f));
			break;
		case 0x211:
			transform.q = PxQuat::createIdentity();
			break;
		case 0x141:
			transform.q = PxQuat(-PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f));
			break;
		case 0x121:
			transform.q = PxQuat( PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f));
			break;
		case 0x114:
			transform.q = PxQuat( PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f));
			break;
		case 0x112:
			transform.q = PxQuat(-PxHalfPi, PxVec3(0.0f, 1.0f, 0.0f));
			break;
		default:
			transform.q = computeQuatFromNormal(outwardNormal);
			break;
	}
	transform.p = outwardNormal * distance;

	PxShape* shape;
	return setShape(sdk.createRigidStatic(transform), PxPlaneGeometry(), material, PxTransform::createIdentity(), shape);
}
