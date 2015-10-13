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
#include "PsUserAllocated.h"
#include "OPC_MeshInterface.h"

using namespace physx;
using namespace Ice;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MeshInterface::MeshInterface() :
	mRemapCallback	(NULL),
	mRemapUserData	(NULL),
	mTris			(NULL),
	mVerts			(NULL),
	mNbTris			(0),
	mNbVerts		(0)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MeshInterface::~MeshInterface()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Checks the mesh interface is valid, i.e. things have been setup correctly.
 *	\return		true if valid
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MeshInterface::IsValid() const
{
	if(!mNbTris || !mNbVerts)	return false;
	if(!mTris || !mVerts)		return false;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Checks the mesh itself is valid.
 *	Currently we only look for degenerate faces.
 *	\return		number of degenerate faces
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PxU32 MeshInterface::CheckTopology()	const
{
	// Check topology. If the model contains degenerate faces, collision report can be wrong in some cases.
	// e.g. it happens with the standard MAX teapot. So clean your meshes first... If you don't have a mesh cleaner
	// you can try this: www.codercorner.com/Consolidation.zip

	PxU32 NbDegenerate = 0;

	VertexPointers VP;

	// Using callbacks, we don't have access to vertex indices. Nevertheless we still can check for
	// redundant vertex pointers, which cover all possibilities (callbacks/pointers/strides).
	for(PxU32 i=0;i<mNbTris;i++)
	{
		GetTriangle(VP, i);

		if(		(VP.vertex[0]==VP.vertex[1])
			||	(VP.vertex[1]==VP.vertex[2])
			||	(VP.vertex[2]==VP.vertex[0]))	NbDegenerate++;
	}

	return NbDegenerate;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Pointers control: setups object pointers. Must provide access to faces and vertices for a given object.
 *	\param		tris	[in] pointer to triangles
 *	\param		verts	[in] pointer to vertices
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MeshInterface::SetPointers(const void* tris, bool has16BitIndices, const PxVec3* verts)
{
	if(!tris || !verts)	return SetIceError("MeshInterface::SetPointers: pointer is NULL");

	mTris	= tris;
	mVerts	= verts;
	mHas16BitIndices = has16BitIndices;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Remaps client's mesh according to a permutation.
 *	\param		nb_indices	[in] number of indices in the permutation (will be checked against number of triangles)
 *	\param		permutation	[in] list of triangle indices
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MeshInterface::RemapClient(PxU32 nb_indices, const PxU32* permutation) const
{
	// Checkings
	if(!nb_indices || !permutation)	return false;
	if(nb_indices!=mNbTris)			return false;

	if(mRemapCallback)
	{
		if(!(mRemapCallback)(nb_indices, permutation, mRemapUserData))	return true;
	}

	if (has16BitIndices())
		return remapCopy<Gu::TriangleT<PxU16> >(permutation);
	else
		return remapCopy<Gu::TriangleT<PxU32> >(permutation);
}



template<class Triangle>			//either IndexedTriangle16 or IndexedTriangle32
bool MeshInterface::remapCopy(const PxU32* permutation) const
{
	const PxU32 Stride = sizeof(Triangle);

	//!!! Pierre's version does not compile under XBox. Revisit...
	Triangle* Tmp = (Triangle*)PX_ALLOC_TEMP(sizeof(Triangle) * mNbTris);

	for(PxU32 i=0;i<mNbTris;i++)
	{
		const Triangle* T = (const Triangle*)(((PxU8*)mTris) + i * Stride);
		Tmp[i] = *T;
	}
	for(PxU32 i=0;i<mNbTris;i++)
	{
		Triangle* T = (Triangle*)(((PxU8*)mTris) + i * Stride);
		*T = Tmp[permutation[i]];
	}

	PX_FREE_AND_RESET(Tmp);

	return true;
}
