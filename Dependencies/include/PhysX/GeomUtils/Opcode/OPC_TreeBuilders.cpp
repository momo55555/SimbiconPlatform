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
#include "OPC_TreeBuilders.h"
#include "OPC_MeshInterface.h"

using namespace physx;
using namespace Ice;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the AABB of a set of primitives.
 *	\param		primitives		[in] list of indices of primitives
 *	\param		nb_prims		[in] number of indices
 *	\param		global_box		[out] global AABB enclosing the set of input primitives
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBTreeOfAABBsBuilder::ComputeGlobalBox(const PxU32* primitives, PxU32 nb_prims, PxBounds3& global_box) const
{
	// Checkings
	if(!primitives || !nb_prims)	return false;

	// Initialize global box
	global_box = mAABBArray[primitives[0]];

	// Loop through boxes
	for(PxU32 i=1;i<nb_prims;i++)
	{
		// Update global box
		global_box.include(mAABBArray[primitives[i]]);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the splitting value along a given axis for a given primitive.
 *	\param		index		[in] index of the primitive to split
 *	\param		axis		[in] axis index (0,1,2)
 *	\return		splitting value
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float AABBTreeOfAABBsBuilder::GetSplittingValue(PxU32 index, PxU32 axis) const
{
	// For an AABB, the splitting value is the middle of the given axis,
	// i.e. the corresponding component of the center point
	return mAABBArray[index].getCenter(axis);
}

void AABBTreeOfAABBsBuilder::GetSplittingValues(PxU32 index, PxVec3& values) const
{
	values = mAABBArray[index].getCenter();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the AABB of a set of primitives.
 *	\param		primitives		[in] list of indices of primitives
 *	\param		nb_prims		[in] number of indices
 *	\param		global_box		[out] global AABB enclosing the set of input primitives
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBTreeOfTrianglesBuilder::ComputeGlobalBox(const PxU32* primitives, PxU32 nb_prims, PxBounds3& global_box) const
{
	// Checkings
	if(!primitives || !nb_prims)	return false;

	// Initialize global box
	PxVec3 Min(PX_MAX_F32, PX_MAX_F32, PX_MAX_F32);
	PxVec3 Max(-PX_MAX_F32, -PX_MAX_F32, -PX_MAX_F32);

	// Loop through triangles
	VertexPointers VP;

	if(mIMesh->has16BitIndices())
	{
		while(nb_prims--)
		{
			mIMesh->GetTriangle<Gu::TriangleT<PxU16> >(VP, *primitives++);
			Min = Min.minimum(*VP.vertex[0]).minimum(*VP.vertex[1]).minimum(*VP.vertex[2]);
			Max = Max.maximum(*VP.vertex[0]).maximum(*VP.vertex[1]).maximum(*VP.vertex[2]);
		}
	}
	else
	{
		while(nb_prims--)
		{
			mIMesh->GetTriangle<Gu::TriangleT<PxU32> >(VP, *primitives++);
			Min = Min.minimum(*VP.vertex[0]).minimum(*VP.vertex[1]).minimum(*VP.vertex[2]);
			Max = Max.maximum(*VP.vertex[0]).maximum(*VP.vertex[1]).maximum(*VP.vertex[2]);
		}
	}
	global_box.minimum = Min;
	global_box.maximum = Max;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the splitting value along a given axis for a given primitive.
 *	\param		index		[in] index of the primitive to split
 *	\param		axis		[in] axis index (0,1,2)
 *	\return		splitting value
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define	INV3	0.33333333333333333333f		//!< 1/3
float AABBTreeOfTrianglesBuilder::GetSplittingValue(PxU32 index, PxU32 axis) const
{
/*	// Compute center of triangle
	Point Center;
	mTriList[index].Center(mVerts, Center);
	// Return value
	return Center[axis];*/

	// Compute correct component from center of triangle
//	return	(mVerts[mTriList[index].mVRef[0]][axis]
//			+mVerts[mTriList[index].mVRef[1]][axis]
//			+mVerts[mTriList[index].mVRef[2]][axis])*INV3;

	VertexPointers VP;
	if (mIMesh->has16BitIndices())
		mIMesh->GetTriangle<Gu::TriangleT<PxU16> >(VP, index);
	else
		mIMesh->GetTriangle<Gu::TriangleT<PxU32> >(VP, index);

	// Compute correct component from center of triangle
	return	((*VP.vertex[0])[axis]
			+(*VP.vertex[1])[axis]
			+(*VP.vertex[2])[axis])*INV3;
}

void AABBTreeOfTrianglesBuilder::GetSplittingValues(PxU32 index, PxVec3& values) const
{
	VertexPointers VP;
	if (mIMesh->has16BitIndices())
		mIMesh->GetTriangle<Gu::TriangleT<PxU16> >(VP, index);
	else
		mIMesh->GetTriangle<Gu::TriangleT<PxU32> >(VP, index);
	// Compute correct component from center of triangle
	values = (*VP.vertex[0] + *VP.vertex[1] + *VP.vertex[2])*INV3;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the splitting value along a given axis for a given node.
 *	\param		primitives		[in] list of indices of primitives
 *	\param		nb_prims		[in] number of indices
 *	\param		global_box		[in] global AABB enclosing the set of input primitives
 *	\param		axis			[in] axis index (0,1,2)
 *	\return		splitting value
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float AABBTreeOfTrianglesBuilder::GetSplittingValue(const PxU32* primitives, PxU32 nb_prims, const PxBounds3& global_box, PxU32 axis)	const
{
	if(mSettings.mRules&SPLIT_GEOM_CENTER)
	{
		// Loop through triangles
		float SplitValue = 0.0f;
		VertexPointers VP;

		if (mIMesh->has16BitIndices())
			for(PxU32 i=0;i<nb_prims;i++)
			{
				mIMesh->GetTriangle<Gu::TriangleT<PxU16> >(VP, primitives[i]);
				SplitValue += (*VP.vertex[0])[axis];
				SplitValue += (*VP.vertex[1])[axis];
				SplitValue += (*VP.vertex[2])[axis];
			}
		else
			for(PxU32 i=0;i<nb_prims;i++)
			{
				mIMesh->GetTriangle<Gu::TriangleT<PxU32> >(VP, primitives[i]);
				SplitValue += (*VP.vertex[0])[axis];
				SplitValue += (*VP.vertex[1])[axis];
				SplitValue += (*VP.vertex[2])[axis];
			}

		return SplitValue / float(nb_prims*3);
	}
	else return AABBTreeBuilder::GetSplittingValue(primitives, nb_prims, global_box, axis);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the AABB of a set of primitives.
 *	\param		primitives		[in] list of indices of primitives
 *	\param		nb_prims		[in] number of indices
 *	\param		global_box		[out] global AABB enclosing the set of input primitives
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AABBTreeOfVerticesBuilder::ComputeGlobalBox(const PxU32* primitives, PxU32 nb_prims, PxBounds3& global_box) const
{
	// Checkings
	if(!primitives || !nb_prims)	return false;

	// Initialize global box
	global_box.setEmpty();

	// Loop through vertices
	for(PxU32 i=0;i<nb_prims;i++)
	{
		// Update global box
		global_box.include(mVertexArray[primitives[i]]);
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the splitting value along a given axis for a given primitive.
 *	\param		index		[in] index of the primitive to split
 *	\param		axis		[in] axis index (0,1,2)
 *	\return		splitting value
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float AABBTreeOfVerticesBuilder::GetSplittingValue(PxU32 index, PxU32 axis) const
{
	// For a vertex, the splitting value is simply the vertex coordinate.
	return mVertexArray[index][axis];
}

void AABBTreeOfVerticesBuilder::GetSplittingValues(PxU32 index, PxVec3& values) const
{
	values = mVertexArray[index];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the splitting value along a given axis for a given node.
 *	\param		primitives		[in] list of indices of primitives
 *	\param		nb_prims		[in] number of indices
 *	\param		global_box		[in] global AABB enclosing the set of input primitives
 *	\param		axis			[in] axis index (0,1,2)
 *	\return		splitting value
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float AABBTreeOfVerticesBuilder::GetSplittingValue(const PxU32* primitives, PxU32 nb_prims, const PxBounds3& global_box, PxU32 axis)	const
{
	if(mSettings.mRules&SPLIT_GEOM_CENTER)
	{
		// Loop through vertices
		float SplitValue = 0.0f;
		for(PxU32 i=0;i<nb_prims;i++)
		{
			// Update split value
			SplitValue += mVertexArray[primitives[i]][axis];
		}
		return SplitValue / float(nb_prims);
	}
	else return AABBTreeBuilder::GetSplittingValue(primitives, nb_prims, global_box, axis);
}
