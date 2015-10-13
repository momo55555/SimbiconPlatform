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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "PsIntrinsics.h"
#include "GuIntersectionBoxBox.h"
#include "GuIntersectionCapsuleBox.h"
#include "GuIntersectionSphereBox.h"
#include "./Ice/CTC_RayAABBOverlap.h"
#include "./Ice/CTC_PlaneAABBOverlap.h"
#include "SqFreePruner.h"
#include "SqUtilities.h"

using namespace physx;
using namespace Ice;
using namespace Sq;

FreePruner::FreePruner()
{
	mCullFunc			= (CullFunc)			&FreePruner::Cull;
	mStabFunc			= (StabFunc)			&FreePruner::Stab;
	mOverlapSphereFunc	= (OverlapSphereFunc)	&FreePruner::OverlapSphere;
	mOverlapAABBFunc	= (OverlapAABBFunc)		&FreePruner::OverlapAABB;
	mOverlapOBBFunc		= (OverlapOBBFunc)		&FreePruner::OverlapOBB;
	mOverlapCapsuleFunc	= (OverlapCapsuleFunc)	&FreePruner::OverlapCapsule;
}

FreePruner::~FreePruner()
{
}

///////////////////////////////////////////////////////////////////////////////

bool Sq::cullObjects(Pruner* owner, Prunable** objects, PxU32 nb, CulledObjects& dest, const Gu::Plane* planes, PxU32 nbPlanes, PxU32 cullingFlags)
{
	const PxU32 Mask = (1<<nbPlanes)-1;

	while(nb--)
	{
		Prunable* PRN = *objects++;
		PxU32 OutClipMask;
		if(!PlanesAABBOverlap(*owner->GetWorldAABB(*PRN), planes, OutClipMask, Mask))
			continue;
		dest.AddPrunable(PRN);
	}
	return true;
}

bool FreePruner::Cull(PruningTemps& temps, CulledObjects& objects, const Gu::Plane* planes, PxU32 nbPlanes, PxU32 cullingFlags)
{
	Prunable** Objects = GetObjects();
	PxU32 Nb = GetNbActiveObjects();
	return cullObjects(this, Objects, Nb, objects, planes, nbPlanes, cullingFlags);
}

///////////////////////////////////////////////////////////////////////////////

PxU32 Sq::stabObjects(Pruner* owner, Prunable** objects, PxU32 nb, StabCallback callback, void* userData, const PxVec3& orig, const PxVec3& dir, float& maxDist)
{
	if(maxDist==PX_MAX_F32)
	{
		float MaxDist = maxDist;
		while(nb--)
		{
			Prunable* Current = *objects++;
			if(nb)	Ps::prefetch(*objects);

			if(RayAABB(orig, dir, *owner->GetWorldAABB(*Current)))
			{
				PxU32 Status = (callback)(Current, MaxDist, userData);
				if(Status & STAB_STOP)
					return STAB_STOP;
			}
		}
	}
	else
	{
		Gu::Segment Seg(orig, orig+dir*maxDist);

		float MaxDist = maxDist;
		while(nb--)
		{
			Prunable* Current = *objects++;
			if(nb)	Ps::prefetch(*objects);

			if(SegmentAABB(Seg, *owner->GetWorldAABB(*Current)))
			{
				PxU32 Status = (callback)(Current, MaxDist, userData);
				if(Status & STAB_STOP)
					return STAB_STOP;
				if(Status & STAB_UPDATE_MAX_DIST)
				{
					Seg.p1 = orig + dir * MaxDist;
				}
			}
		}
	}
	return STAB_CONTINUE;
}

PxU32 FreePruner::Stab(StabCallback callback, void* userData, const PxVec3& orig, const PxVec3& dir, float& maxDist)
{
	Prunable** Objects = GetObjects();
	PxU32 Nb = GetNbActiveObjects();
	return stabObjects(this, Objects, Nb, callback, userData, orig, dir, maxDist);
}

///////////////////////////////////////////////////////////////////////////////

#define MAX_ACCUMULATED_OBJECTS	32	// PT: 32*4 = 128 bytes, one cache line
class PrunableAccumulator
{
	public:
	PX_FORCE_INLINE	PrunableAccumulator(ReportPrunablesCallback cb, void* userData) : mCallback(cb), mUserData(userData), mNb(0)
	{
	}
	PX_FORCE_INLINE	bool	fireCallbacks()
	{
		bool ret = (mCallback)(mObjects, mNb, mUserData);
		mNb = 0;
		return ret;
	}
	PX_FORCE_INLINE	bool	accum(Prunable* object)
	{
		mObjects[mNb++] = object;
		if(mNb==MAX_ACCUMULATED_OBJECTS)
			return fireCallbacks();
		return true;
	}
	PX_FORCE_INLINE	bool	finalize()
	{
		if(mNb)
			return fireCallbacks();
		else
			return true;
	}

	ReportPrunablesCallback	mCallback;
	void*					mUserData;
	PxU32					mNb;
	Prunable*				mObjects[MAX_ACCUMULATED_OBJECTS];
};

bool Sq::overlapObjects(Pruner* owner, Prunable** objects, PxU32 nb, ReportPrunablesCallback cb, void* userData, const Gu::Sphere& sphere, bool firstContact)
{
	PrunableAccumulator accumulator(cb, userData);
	while(nb--)
	{
		Prunable* current = *objects++;
		if(Gu::intersectSphereAABB(sphere.center, sphere.radius, *owner->GetWorldAABB(*current)))
		{
			if(!accumulator.accum(current))
				return false;
		}
	}
	return accumulator.finalize();
}

bool FreePruner::OverlapSphere(ReportPrunablesCallback cb, void* userData, const Gu::Sphere& sphere, bool firstContact)
{
	Prunable** Objects = GetObjects();
	PxU32 Nb = GetNbActiveObjects();
	return overlapObjects(this, Objects, Nb, cb, userData, sphere, firstContact);
}

///////////////////////////////////////////////////////////////////////////////

bool Sq::overlapObjects(Pruner* owner, Prunable** objects, PxU32 nb, ReportPrunablesCallback cb, void* userData, const PxBounds3& box, bool firstContact)
{
	PrunableAccumulator accumulator(cb, userData);
	while(nb--)
	{
		Prunable* current = *objects++;
		if(box.intersects(*owner->GetWorldAABB(*current)))
		{
			if(!accumulator.accum(current))
				return false;
		}
	}
	return accumulator.finalize();
}

bool FreePruner::OverlapAABB(ReportPrunablesCallback cb, void* userData, const PxBounds3& box, bool firstContact)
{
	Prunable** Objects = GetObjects();
	PxU32 Nb = GetNbActiveObjects();
	return overlapObjects(this, Objects, Nb, cb, userData, box, firstContact);
}

///////////////////////////////////////////////////////////////////////////////

bool Sq::overlapObjects(Pruner* owner, Prunable** objects, PxU32 nb, ReportPrunablesCallback cb, void* userData, const Gu::Box& box, bool firstContact)
{
	PrunableAccumulator accumulator(cb, userData);
	while(nb--)
	{
		Prunable* current = *objects++;
		if(Gu::intersectOBBAABB(box, *owner->GetWorldAABB(*current)))
		{
			if(!accumulator.accum(current))
				return false;
		}
	}
	return accumulator.finalize();
}

bool FreePruner::OverlapOBB(ReportPrunablesCallback cb, void* userData, const Gu::Box& box, bool firstContact)
{
	Prunable** Objects = GetObjects();
	PxU32 Nb = GetNbActiveObjects();
	return overlapObjects(this, Objects, Nb, cb, userData, box, firstContact);
}

///////////////////////////////////////////////////////////////////////////////

bool Sq::overlapObjects(Pruner* owner, Prunable** objects, PxU32 nb, ReportPrunablesCallback cb, void* userData, const Gu::Capsule& capsule, bool firstContact)
{
	PrunableAccumulator accumulator(cb, userData);
	while(nb--)
	{
		Prunable* current = *objects++;
		if(Gu::intersectCapsuleAABB(capsule, *owner->GetWorldAABB(*current)))
		{
			if(!accumulator.accum(current))
				return false;
		}
	}
	return accumulator.finalize();
}

bool FreePruner::OverlapCapsule(ReportPrunablesCallback cb, void* userData, const Gu::Capsule& capsule, bool firstContact)
{
	Prunable** Objects = GetObjects();
	PxU32 Nb = GetNbActiveObjects();
	return overlapObjects(this, Objects, Nb, cb, userData, capsule, firstContact);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "CmRenderOutput.h"
void FreePruner::visualize(Cm::RenderOutput& out, PxU32 color)
{
	PxU32 nb = GetNbActiveObjects();
	Prunable** Objects = GetObjects();
//	const PxBounds3* boxes = GetCurrentWorldBoxes();	// PT: SIGH. The carefully designed PruningPool is all fucked up to hell those days.

	PxTransform idt = PxTransform::createIdentity();
	out << idt;
	out << color;
	while(nb--)
	{
		Prunable* Current = *Objects++;
//		out << Cm::DebugBox(*boxes++, true);
		out << Cm::DebugBox(*GetWorldAABB(*Current), true);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PT: specialized version for FreePruner. Avoids virtual calls at least.
void FreePruner::addShapes(PxU32 nbShapes, SceneQueryShapeData*const* PX_RESTRICT shapes)
{
	SceneQueryShapeData* nextSqData = shapes[0];
	for(PxU32 i=0; i<nbShapes; i++)
	{
		SceneQueryShapeData* sqData = nextSqData;
		PX_ASSERT(sqData);

		if(i!=nbShapes-1)
		{
			nextSqData = shapes[i+1];
			Ps::prefetch128(nextSqData);
		}

		// PT: same as "AddObject", optimized
		PX_ASSERT(sqData->mHandle==INVALID_PRUNING_HANDLE);

		// PT: TODO: sigh... this stuff is so broken. In the original version doing "AddObject" immediately followed by "UpdateObject" was useless.
		// We need to figure out if it changed, and why.

		// AddObject
		{
			// Add the object to the pool
			PruningPool::AddObject(*sqData);
		}

		// Update object
		{
			// Lazy-rebuild the world box
			if(!sqData->IsSet(PRN_VALIDAABB))
			{
				sqData->mPRNFlags|=PRN_VALIDAABB;
//				sqData->GetWorldAABB(mWorldBoxes[sqData->mHandle]);
				sqData->ComputeWorldAABB_Special(mWorldBoxes[sqData->mHandle]);
			}
		}
	}

	if(nbShapes)
	{
		// Invalidate acceleration structure
		mSignature.Invalidate();
	}
}
