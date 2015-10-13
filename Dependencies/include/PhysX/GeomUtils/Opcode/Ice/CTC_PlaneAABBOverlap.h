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
// Include Guard
#ifndef CTCPLANEAABBOVERLAP_H
#define CTCPLANEAABBOVERLAP_H

#include "PxBounds3.h"
#include "GuPlane.h"

namespace physx
{
namespace Ice
{

	// Following code from Umbra/dPVS.

	//------------------------------------------------------------------------
	//
	// Function:        DPVS::intersectAABBFrustum()
	//
	// Description:     Determines whether an AABB intersects a frustum
	//
	// Parameters:      a           = reference to AABB (defined by minimum & maximum vectors)
	//                  p           = array of pre-normalized clipping planes
	//                  outClipMask = output clip mask (if function returns 'true')
	//                  inClipMask  = input clip mask (indicates which planes are active)
	//
	// Returns:         true if AABB intersects the frustum, false otherwise
	//
	//                  Intersection of AABB and a frustum. The frustum may 
	//                  contain 0-32 planes (active planes are defined by inClipMask). 
	//                  If AABB intersects the frustum, an output clip mask is returned 
	//                  as well (indicating which planes are crossed by the AABB). This 
	//                  information can be used to optimize testing of child nodes or 
	//                  objects inside the nodes (pass value as 'inClipMask' next time).
	//
	//                  This is a variant of the classic "fast" AABB/frustum 
	//                  intersection tester. AABBs that are not culled away by any single 
	//                  plane are classified as "intersecting" even though the AABB may 
	//                  actually be outside the convex volume formed by the planes.
	//------------------------------------------------------------------------

	PX_INLINE bool PlanesAABBOverlap(const PxBounds3& a, const Gu::Plane* p, PxU32& out_clip_mask, PxU32 in_clip_mask)
	{
		//------------------------------------------------------------------------
		// Convert the AABB from (minimum,maximum) form into (center,half-diagonal).
		// Note that we could get rid of these six subtractions and three
		// multiplications if the AABB was originally expressed in (center,
		// half-diagonal) form.
		//------------------------------------------------------------------------

		PxVec3 m = a.getCenter();			// get center of AABB ((minimum+maximum)*0.5f)
		PxVec3 d = a.maximum;	d-=m;		// get positive half-diagonal (maximum - center)

		//------------------------------------------------------------------------
		// Evaluate through all active frustum planes. We determine the relation 
		// between the AABB and a plane by using the concept of "near" and "far"
		// vertices originally described by Zhang (and later by Moeller). Our
		// variant here uses 3 fabs ops, 6 muls, 7 adds and two floating point
		// comparisons per plane. The routine early-exits if the AABB is found
		// to be outside any of the planes. The loop also constructs a new output
		// clip mask. Most FPUs have a native single-cycle fabsf() operation.
		//------------------------------------------------------------------------

		PxU32 Mask				= 1;			// current mask index (1,2,4,8,..)
		PxU32 TmpOutClipMask	= 0;			// initialize output clip mask into empty. 

		while(Mask<=in_clip_mask)				// keep looping while we have active planes left...
		{
			if(in_clip_mask & Mask)				// if clip plane is active, process it..
			{               
				const float NP = d.x*PxAbs(p->normal.x) + d.y*PxAbs(p->normal.y) + d.z*PxAbs(p->normal.z);
				const float MP = m.x*p->normal.x + m.y*p->normal.y + m.z*p->normal.z + p->d;

				if(NP < MP)						// near vertex behind the clip plane... 
					return false;				// .. so there is no intersection..
				if((-NP) < MP)					// near and far vertices on different sides of plane..
					TmpOutClipMask |= Mask;		// .. so update the clip mask...
			}
			Mask+=Mask;							// mk = (1<<plane)
			p++;								// advance to next plane
		}

		out_clip_mask = TmpOutClipMask;			// copy output value (temp used to resolve aliasing!)
		return true;							// indicate that AABB intersects frustum
	}

	PX_INLINE bool PlanesAABBOverlap(const PxBounds3& a, const Gu::Plane** pp, PxU32& out_clip_mask, PxU32 in_clip_mask)
	{
		//------------------------------------------------------------------------
		// Convert the AABB from (minimum,maximum) form into (center,half-diagonal).
		// Note that we could get rid of these six subtractions and three
		// multiplications if the AABB was originally expressed in (center,
		// half-diagonal) form.
		//------------------------------------------------------------------------

		PxVec3 m = a.getCenter();			// get center of AABB ((minimum+maximum)*0.5f)
		PxVec3 d = a.maximum;	d-=m;		// get positive half-diagonal (maximum - center)

		//------------------------------------------------------------------------
		// Evaluate through all active frustum planes. We determine the relation 
		// between the AABB and a plane by using the concept of "near" and "far"
		// vertices originally described by Zhang (and later by Moeller). Our
		// variant here uses 3 fabs ops, 6 muls, 7 adds and two floating point
		// comparisons per plane. The routine early-exits if the AABB is found
		// to be outside any of the planes. The loop also constructs a new output
		// clip mask. Most FPUs have a native single-cycle fabsf() operation.
		//------------------------------------------------------------------------

		PxU32 Mask				= 1;			// current mask index (1,2,4,8,..)
		PxU32 TmpOutClipMask	= 0;			// initialize output clip mask into empty. 

		while(Mask<=in_clip_mask)				// keep looping while we have active planes left...
		{
			const Gu::Plane* p = *pp++;

			if(in_clip_mask & Mask)				// if clip plane is active, process it..
			{               
				const float NP = d.x*PxAbs(p->normal.x) + d.y*PxAbs(p->normal.y) + d.z*PxAbs(p->normal.z);
				const float MP = m.x*p->normal.x + m.y*p->normal.y + m.z*p->normal.z + p->d;

				if(NP < MP)						// near vertex behind the clip plane... 
					return false;				// .. so there is no intersection..
				if((-NP) < MP)					// near and far vertices on different sides of plane..
					TmpOutClipMask |= Mask;		// .. so update the clip mask...
			}
			Mask+=Mask;							// mk = (1<<plane)
//			p++;								// advance to next plane
		}

		out_clip_mask = TmpOutClipMask;			// copy output value (temp used to resolve aliasing!)
		return true;							// indicate that AABB intersects frustum
	}
}

}

#endif // CTCPLANEAABBOVERLAP_H
