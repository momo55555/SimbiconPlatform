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


#ifndef PX_PHYSICS_NXPHYSICS_API
#define PX_PHYSICS_NXPHYSICS_API
/** \addtogroup physics
@{
*/

/**
This is the main include header for the Physics SDK, for users who
want to use a single #include file.

Alternatively, one can instead directly #include a subset of the below files.
*/

#pragma warning(push)

// Foundation SDK 
#include "PxErrorCallback.h"
#include "PxAllocatorCallback.h"
#include "PxFoundation.h"
#include "PxVec3.h"
#include "PxMat33.h"
#include "PxQuat.h"
#include "PxTransform.h"
#include "PxBounds3.h"
#include "PxVec4.h"
#include "PxMat44.h"


// PhysX Core SDK

#include "PxPhysics.h"

#include "PxScene.h"
#include "PxSceneDesc.h"

#include "PxActor.h"
#include "PxActorDesc.h"
#include "PxRigidStatic.h"
#include "PxRigidDynamic.h"

#include "PxMaterial.h"

#include "PxShape.h"

#include "PxContactStreamIterator.h"
#include "PxSimulationEventCallback.h"
#include "PxContactModifyCallback.h"
#include "PxObserver.h"

#include "PxBatchQuery.h"
#include "PxSweepCache.h"

#include "PxConstraint.h"

#include "PxArticulation.h"
#include "PxArticulationLink.h"
#include "PxArticulationJoint.h"

#include "geometry/PxBoxGeometry.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxPlaneGeometry.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "geometry/PxHeightFieldGeometry.h"

#if PX_USE_PARTICLE_SYSTEM_API
#include "particles/PxParticleSystem.h"
#include "particles/PxParticleSystemDesc.h"
#include "particles/PxParticleFluid.h"
#include "particles/PxParticleFluidDesc.h"
#endif

#if PX_USE_DEFORMABLE_API
#include "deformable/PxDeformable.h"
#include "deformable/PxDeformableDesc.h"
#include "deformable/PxAttachment.h"
#endif

#if PX_USE_CLOTH_API
#include "cloth/PxCloth.h"
#endif

#include "PxAggregate.h"

#include "PxSimulationStatistics.h"
#include "geometry/PxTriangleMesh.h"
#include "geometry/PxConvexMesh.h"
#include "geometry/PxHeightField.h"
#include "geometry/PxHeightFieldDesc.h"
#include "geometry/PxHeightFieldSample.h"

#include "pvd/PxVisualDebugger.h"

#include "cooking/PxCooking.h"


#pragma warning(pop)

/** @} */
#endif
