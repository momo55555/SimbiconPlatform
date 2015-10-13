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

#ifndef PX_COLLISION_RTREE
#define PX_COLLISION_RTREE

#include "PxSimpleTypes.h"
#include "PxVec4.h"
#include "PsUserAllocated.h" // for PxSerialStream
#include "CmMetaData.h"

namespace physx
{

using namespace physx::pubfnd3;
using namespace physx::shdfnd3;

namespace Gu {

	class Box;

	PX_ALIGN_PREFIX(16)
	struct RTreePage {
		// AP: It would be nice not to have 8 as a magic number but there's SIMD code
		// that wouldn't work even if all 8s were replaced with let's say 16
		// so 8 is a magic number for the current rtree implementation
		enum { SIZE = 8 };
		PxU16 minx[8];
		PxU16 miny[8];
		PxU16 minz[8];
		PxU16 maxx[8];
		PxU16 maxy[8];
		PxU16 maxz[8];
		PxU32 ptrs[8];
	} PX_ALIGN_SUFFIX(16);

	// RTree root data structure
	PX_ALIGN_PREFIX(16)
	struct RTree
	{
		// PX_SERIALIZATION
		RTree(PxRefResolver& v);
		void	exportExtraData(PxSerialStream&);
		char*	importExtraData(char* address, PxU32& totalPadding);
		static	void	getMetaData(PxSerialStream& stream);
		//~PX_SERIALIZATION

		RTree();
		~RTree() { release(); }

		void release();
		bool save(PxStream& stream) const; // always saves as big endian
		bool load(const PxStream& stream); // converts to proper endian at load time

		void buildFromBounds(const PxBounds3* allBounds, const PxU32* boundedData, PxU32 numBounds);
		void buildFromTriangles(const PxVec3* verts, PxU32 numVerts, const PxU32* tris, PxU32 numTris);
		void buildFromLeafTriangles( // scaffold - temporary for perf comparison with opcode only
			const void* leafTriangles, PxU32 numLeaves,
			const PxVec3* verts, PxU32 numVerts, const PxU16* tris16, const PxU32* tris32, PxU32 numTris);

		struct Callback
		{
			// result buffer should have room for at least 8 (rtree page size) items
			// should return true to continue traversal. If false is returned, traversal is aborted
			virtual bool processResults(PxU32 count, PxU32* buf) = 0;
		};

		// callback will be issued as soon as the buffer overflows maxResultsPerBlock-8 entries
		// use maxResults = 8 and return false from callback for "first hit" early out
		void traverseAABB(
			const PxVec3& boxMin, const PxVec3& boxMax,
			const PxU32 maxResultsPerBlock, PxU32* resultsBlockBuf, Callback* processResultsBlockCallback) const;
		void traverseOBB(
			const Gu::Box& obb,
			const PxU32 maxResultsPerBlock, PxU32* resultsBlockBuf, Callback* processResultsBlockCallback) const;
		template <int useRadius, int raySegment>
		void traverseRay(
			const PxVec3& rayOrigin, const PxVec3& rayDir, // dir doesn't have to be normalized and is B-A for raySegment
			const PxU32 maxResults, PxU32* resultsPtr, Gu::RTree::Callback* callback,
			const PxVec3& inflateAABBs) const;

		// remember to update save() and load() when adding or removing data
		PxVec4		mBoundsMin, mBoundsMax, mInvDiagonal, mDiagonalScaler;
		PxU32		mPageSize;
		PxU32		mNumRootPages;
		PxU32		mNumLevels;
		PxU32		mTotalNodes;
		PxU32		mTotalPages;
		PxU32		mBottomLevelFirstNodeIndex;
		PxU32		mFlags; enum { USER_ALLOCATED = 0x1, };
		RTreePage*	mPages;

		static PxU32 mVersion;
	} PX_ALIGN_SUFFIX(16);

	// explicit instantiations for traverseRay
	// XXX: dima: g++ 4.4 won't compile this => skipping by PX_LINUX
#if (defined(PX_X86) || defined(PX_X360)) && !(defined(PX_LINUX) || defined(PX_APPLE))
	template void RTree::traverseRay<0,0>(const PxVec3&, const PxVec3&, const PxU32, PxU32*, Gu::RTree::Callback*, const PxVec3&) const;
	template void RTree::traverseRay<0,1>(const PxVec3&, const PxVec3&, const PxU32, PxU32*, Gu::RTree::Callback*, const PxVec3&) const;
	template void RTree::traverseRay<1,0>(const PxVec3&, const PxVec3&, const PxU32, PxU32*, Gu::RTree::Callback*, const PxVec3&) const;
	template void RTree::traverseRay<1,1>(const PxVec3&, const PxVec3&, const PxU32, PxU32*, Gu::RTree::Callback*, const PxVec3&) const;
#endif
} // namespace Gu

}

#endif // #ifdef PX_COLLISION_RTREE
