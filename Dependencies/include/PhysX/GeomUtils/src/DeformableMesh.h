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


#ifndef PX_PHYSICS_DEFORMABLEMESH
#define PX_PHYSICS_DEFORMABLEMESH

// PX_SERIALIZATION
#include "PxSerialFramework.h"
#include "CmReflection.h"
//~PX_SERIALIZATION

#include "PsUserAllocated.h"
#include "PsArray.h"
#include "CmRefCountable.h"
#include "CmPhysXCommon.h"
#include "PxDeformableMeshDesc.h"
#include "PxStrideIterator.h"

namespace physx
{

class NpDeformableMesh;

//----------------------------------------------------------------------------//

// todo: this struct is ugly!
struct DeformableConstraint
{
	PxI32 particleId[6];
	PxU32 flags; // PxvDeformableConstraintFlag
	union
	{
		struct
		{
			// cloth parameters			
			PxReal stretchingRestLength;	
			PxReal bendingRestLength;
		};
		struct
		{
			// soft body parameters			
			PxReal restEdgeLengths[6];
			PxReal restVolume;
			PxU32  pad1;
		};
	};
};

//----------------------------------------------------------------------------//

class DeformableMesh : public Ps::UserAllocated, public Cm::RefCountable
{
public:
// PX_SERIALIZATION
											DeformableMesh(PxRefResolver& v)	: Cm::RefCountable(v), mOwnsMemory(false)	{}
		virtual	void						onRefCountZero();
//~PX_SERIALIZATION
	DeformableMesh();

	void saveToDesc(PxDeformableMeshDesc& desc) const;
	bool load(const PxStream& stream);
	PxDeformablePrimitiveType::Enum getPrimitiveType() const { return mPrimitiveType; }

	// these procedures are not visible to the user
	// the user has to use PxCookDeformableMesh
	bool loadFromDesc(const PxDeformableMeshDesc& desc);	
	bool save(PxStream& stream, bool platformMismatch) const;

	// data used by Deformable
	const Ps::Array<PxVec3>& getVertexPositions() const { return mVertexPositions; }
	const Ps::Array<PxU32>& getVertexIndices() const { return mPrimitives; }
	const Ps::Array<PxReal>& getVertexMasses() const { return mVertexMasses; }
	const Ps::Array<PxU32>& getVertexFlags() const { return mVertexFlags; }
	
	PxU32 getNumWeldedVertices() const { return mNumWeldedVertices; }
	const Ps::Array<PxU32>& getVertexToParticleMap() const { return mVertexToParticleMap; }
	const Ps::Array<DeformableConstraint>& getConstraints() const { return mConstraints; }	

	PxU32 getFlags() const { return mFlags; }
	PxU32 getMaxHierarchyLevels() const { return mMaxHierarchyLevels; }

	void calculateInvMasses(Ps::Array<PxReal>& invMasses, PxReal mass) const;

	virtual	~DeformableMesh() {}

	static bool raycast(
		PxU32& vertexId,
		PxVec3& hit,
		PxDeformablePrimitiveType::Enum primitiveType,
		const PxVec3& rayOrigin,
		const PxVec3& rayDir,
		const PxStrideIterator<const PxVec3>& positionIt,
		const PxU32* indexPermutation,
		const PxU32* indices,
		const PxU32 numIndices,
		const PxBounds3& bounds);

private:
	void clear();

	bool loadMesh(const PxDeformableMeshDesc& desc);
	void weldMesh();

	void generateConstraintsFromTriangles();
	void generateConstraintsFromTetrahedra();

	PxDeformablePrimitiveType::Enum mPrimitiveType;

	Ps::Array<PxVec3>	mVertexPositions;
	Ps::Array<PxU32>	mPrimitives;
	Ps::Array<PxReal>	mVertexMasses;
	Ps::Array<PxU32>	mVertexFlags;
	
	PxU32				mNumWeldedVertices;
	Ps::Array<PxVec3>	mWeldedVertices;	// only temporarily used during constraint generation
	Ps::Array<PxU32>	mVertexToParticleMap;	
		
	PxU32	mFlags;
	PxU32	mMaxHierarchyLevels;
	PxReal	mWeldingDistance;

	Ps::Array<DeformableConstraint> mConstraints;
// PX_SERIALIZATION
	// PT: TODO: optimize that one away
	PX_INLINE	bool	ownsMemory()	const	{ return mOwnsMemory;	}
	bool	mOwnsMemory;
//~PX_SERIALIZATION
};

//----------------------------------------------------------------------------//

}

#endif
