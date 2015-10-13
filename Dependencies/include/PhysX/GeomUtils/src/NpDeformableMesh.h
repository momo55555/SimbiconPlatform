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


#ifndef PX_PHYSICS_GEOMUTILS_NP_DEFORMABLEMESH
#define PX_PHYSICS_GEOMUTILS_NP_DEFORMABLEMESH

#include "PxPhysXGeomUtils.h"

#if PX_USE_DEFORMABLE_API

#include "PxDeformableMesh.h"
#include "CmRefCountable.h"
#include "PsUserAllocated.h"
#include "DeformableMesh.h"

namespace physx
{

class GuMeshFactory;

// Ref counting works like this: SDK object holds a counted reference to the Np mesh
// as does every Np deformable. Sim controller object holds a counted reference to the Px Mesh,
// as does the Np mesh. That way the user can get the reference count according to the app's
// view, whereas deletion of the underlying mesh is controlled by SC's view

// PT: TODO: unify this with the other Gu "mesh classes". "Np" has to go.
class NpDeformableMesh : public PxDeformableMesh, public DeformableMesh
{
private:
	virtual										~NpDeformableMesh();

public:
// PX_SERIALIZATION
												NpDeformableMesh(PxRefResolver& v)	: PxDeformableMesh(v), DeformableMesh(v), mMeshFactory(mMeshFactory)	{}
												DECLARE_SERIAL_CLASS(NpDeformableMesh, PxDeformableMesh)
	PX_INLINE	void							setMeshFactory(GuMeshFactory* f)	{ mMeshFactory = f;	}
	virtual		PxU32							getOrder()				const	{ return PxSerialOrder::eDEFMESH;		}
//~PX_SERIALIZATION
												NpDeformableMesh(GuMeshFactory& meshFactory);
	// PxDeformableMesh
	virtual		void							release();
	virtual		bool							saveToDesc(PxDeformableMeshDesc& desc) const;
	virtual		PxU32							getReferenceCount() const;
	virtual		PxDeformablePrimitiveType::Enum	getPrimitiveType() const;
	//~PxDeformableMesh

protected:
	virtual		void							onRefCountZero();

private:
				GuMeshFactory*					mMeshFactory;	// PT: changed to pointer for serialization
};

#endif // PX_USE_DEFORMABLE_API

}

#endif
