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


#include "PxPhysXGeomUtils.h"

#if PX_USE_DEFORMABLE_API

#include "PsIntrinsics.h"
#include "NpDeformableMesh.h"

#include "GuMeshFactory.h"
#include "DeformableMesh.h"

using namespace physx;

// PX_SERIALIZATION
#include "CmSerialFramework.h"
BEGIN_FIELDS(NpDeformableMesh)
END_FIELDS(NpDeformableMesh)
//~PX_SERIALIZATION

//----------------------------------------------------------------------------//

NpDeformableMesh::NpDeformableMesh(GuMeshFactory& meshFactory) : 
	mMeshFactory(&meshFactory)
{
// PX_SERIALIZATION
	setType(PxSerialType::eDEFORMABLE_MESH);
//~PX_SERIALIZATION
}

//----------------------------------------------------------------------------//

NpDeformableMesh::~NpDeformableMesh()
{
}

//----------------------------------------------------------------------------//

void NpDeformableMesh::release()
{
	decRefCount();
}

void NpDeformableMesh::onRefCountZero()
{
	if(mMeshFactory->removeDeformableMesh(*this))
		return deleteSerializedObject(this);

	// PT: if we reach this point, we didn't find the mesh in the Physics object => don't delete!
	// This prevents deleting the object twice.
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "NpDeformableMesh::release: double deletion detected!");
}

//----------------------------------------------------------------------------//

bool NpDeformableMesh::saveToDesc(PxDeformableMeshDesc& desc) const
{
	DeformableMesh::saveToDesc(desc);
	return true;
}

//----------------------------------------------------------------------------//

PxDeformablePrimitiveType::Enum	NpDeformableMesh::getPrimitiveType() const 
{ 
	return DeformableMesh::getPrimitiveType(); 
}

//----------------------------------------------------------------------------//


PxU32 NpDeformableMesh::getReferenceCount() const
{
	return getRefCount();
}

#endif // PX_USE_DEFORMABLE_API
