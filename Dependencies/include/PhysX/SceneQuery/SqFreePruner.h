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
#ifndef ICEFREEPRUNER_H
#define ICEFREEPRUNER_H

#include "SqPruner.h"

namespace physx
{
namespace Sq
{
	bool	cullObjects(Pruner* owner, Prunable** objects, PxU32 nb, CulledObjects& dest, const Gu::Plane* planes, PxU32 nb_planes, PxU32 culling_flags);
	PxU32	stabObjects(Pruner* owner, Prunable** objects, PxU32 nb, StabCallback callback, void* user_data, const PxVec3& orig, const PxVec3& dir, float& max_dist);
	// PT: return 'false' to abort query (i.e. we already gathered enough results)
	bool	overlapObjects(Pruner* owner, Prunable** objects, PxU32 nb, ReportPrunablesCallback cb, void* userData, const Gu::Sphere& sphere, bool first_contact);
	bool	overlapObjects(Pruner* owner, Prunable** objects, PxU32 nb, ReportPrunablesCallback cb, void* userData, const PxBounds3& box, bool first_contact);
	bool	overlapObjects(Pruner* owner, Prunable** objects, PxU32 nb, ReportPrunablesCallback cb, void* userData, const Gu::Box& box, bool first_contact);
	bool	overlapObjects(Pruner* owner, Prunable** objects, PxU32 nb, ReportPrunablesCallback cb, void* userData, const Gu::Capsule& capsule, bool first_contact);

	class FreePruner : public Pruner
	{
		public:
						FreePruner();
		virtual			~FreePruner();

		// Queries
				bool	Cull(PruningTemps& temps, CulledObjects& objects, const Gu::Plane* planes, PxU32 nb_planes, PxU32 culling_flags);
				PxU32	Stab(StabCallback callback, void* user_data, const PxVec3& orig, const PxVec3& dir, float& max_dist);
				bool	OverlapSphere(ReportPrunablesCallback cb, void* userData, const Gu::Sphere& sphere, bool first_contact);
				bool	OverlapAABB(ReportPrunablesCallback cb, void* userData, const PxBounds3& box, bool first_contact);
				bool	OverlapOBB(ReportPrunablesCallback cb, void* userData, const Gu::Box& box, bool first_contact);
				bool	OverlapCapsule(ReportPrunablesCallback cb, void* userData, const Gu::Capsule& capsule, bool first_contact);

		virtual	void	visualize(Cm::RenderOutput& out, PxU32 color);

		virtual	PX_FORCE_INLINE bool		UpdateObject(Prunable& object){ GetWorldAABB(object); return Pruner::UpdateObject(object);}
		virtual	void						addShapes(PxU32 nbShapes, SceneQueryShapeData*const* PX_RESTRICT shapes);

		// Note: free objects don't have a corresponding acceleration structure
	};

} // namespace Sq

}

#endif // ICEFREEPRUNER_H
