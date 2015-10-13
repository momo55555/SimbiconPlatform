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


#ifndef PX_PHYSICS_GEOMUTILS_GU_MESH_FACTORY
#define PX_PHYSICS_GEOMUTILS_GU_MESH_FACTORY

#include "PxTriangleMesh.h"
#include "PxConvexMesh.h"
#include "PxHeightField.h"
#include "PxDeformableMesh.h"

#include "CmPhysXCommon.h"
#include "PsMutex.h"
#include "PsArray.h"

#include "PsUserAllocated.h"
#include "PsNoCopy.h"

namespace physx
{

class PxHeightFieldDesc;

namespace Gu
{
	class ConvexMesh;
	class HeightField;
	class TriangleMesh;
}
class NpDeformableMesh;


class GuMeshFactory : public Ps::UserAllocated, public Ps::NoCopy
{
public:
									GuMeshFactory()	:
										mTriangleMeshArray	(PX_DEBUG_EXP("meshFactoryTriMesh")),
										mConvexMeshArray	(PX_DEBUG_EXP("meshFactoryConvexMesh")),
										mHeightFieldArray	(PX_DEBUG_EXP("meshFactoryHeightField"))
#if PX_USE_DEFORMABLE_API
										,mDeformableMeshArray(PX_DEBUG_EXP("meshFactoryDeformableMesh"))
#endif
									{}
	virtual								~GuMeshFactory();

	// Triangle mehes
	void							addTriangleMesh(Gu::TriangleMesh* np);
	PxTriangleMesh*					createTriangleMesh(const PxStream&);
	bool							removeTriangleMesh(PxTriangleMesh&);
	PxU32							getNbTriangleMeshes()	const;
	PxU32							getTriangleMeshes(PxTriangleMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

	// Convexes
	void							addConvexMesh(Gu::ConvexMesh* np);
	PxConvexMesh*					createConvexMesh(const PxStream&);
	bool							removeConvexMesh(PxConvexMesh&);
	PxU32							getNbConvexMeshes() const;
	PxU32							getConvexMeshes(PxConvexMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

	// Heightfields
	void							addHeightField(Gu::HeightField* np);
	PxHeightField*					createHeightField(const PxHeightFieldDesc&);
	bool							removeHeightField(PxHeightField&);
	PxU32							getNbHeightFields()	const;
	PxU32							getHeightFields(PxHeightField** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

#if PX_USE_DEFORMABLE_API
	// Deformables
	void							addDeformableMesh(NpDeformableMesh* np);
	PxDeformableMesh*				createDeformableMesh(const PxStream&);
	bool							removeDeformableMesh(PxDeformableMesh&);
	PxU32							getNbDeformableMeshes()	const;
	PxU32							getDeformableMeshes(PxDeformableMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;
#endif

protected:
#if PX_SUPPORT_GPU_PHYSX
	virtual void					notifyReleaseTriangleMesh(const PxTriangleMesh&) {}
	virtual void					notifyReleaseHeightField(const PxHeightField&) {}
	virtual void					notifyReleaseConvexMesh(const PxConvexMesh&) {}
#endif

private:
	Ps::Mutex						mTriangleMeshMutex;
	Ps::Array<Gu::TriangleMesh*>	mTriangleMeshArray;

	Ps::Mutex						mConvexMeshMutex;
	Ps::Array<Gu::ConvexMesh*>		mConvexMeshArray;

	Ps::Mutex						mHeightFieldMutex;
	Ps::Array<Gu::HeightField*>		mHeightFieldArray;

#if PX_USE_DEFORMABLE_API
	Ps::Mutex						mDeformableMeshMutex;
	Ps::Array<NpDeformableMesh*>	mDeformableMeshArray;
#endif

};

}

#endif
