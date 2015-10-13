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


#include "RTree.h"
#include "PsSort.h"
#include "CmPhysXCommon.h"
#include "PxBounds3.h"
#include "PsAlignedMalloc.h"
#include "OPC_ModelData.h" // scaffold - only included for LeafTriangles, todo: remove LeafTriangles
#include "../../GeomUtils/Opcode/Ice/IceUtils.h" // for LittleEndian
#include "../../GeomUtils/Opcode/Ice/IceSerialize.h" // for WriteDword etc
#include "CmSerialAlignment.h"
#include "../../LowLevel/common/include/utils/PxcMemFetch.h"

using namespace physx;
using Ps::Array;
using Ps::sort;
using namespace Ice;

namespace physx
{
namespace Gu {

#define RTREE_MINIMUM_BOUNDS_EPSILON 1e-4F

struct RTreeNodeNQ
{
	PxBounds3	bounds;
	PxI32		childPageFirstNodeIndex;

	RTreeNodeNQ() : bounds(PxBounds3::empty()), childPageFirstNodeIndex(-1) {}
};

// quantized RTree node - only used temporarily before conversion to pages
struct RTreeNodeQ
{
	PxU16 minx, miny, minz, maxx, maxy, maxz;
	PxU32 ptr;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//#define EXPORT_TO_MAXSCRIPT
#ifdef EXPORT_TO_MAXSCRIPT
#include "stdio.h"
static void exportRTreeToMaxscript(PxU32 pageSize, const Array<Array<RTreeNodeNQ> > & treeLevels)
{
	//FILE * f = fopen("/app_home/rtree.ms", "wt");
	FILE * f = fopen("e:\\rtree.ms", "wt");
	for (PxU32 l = treeLevels.size()-1; l != PxU32(-1); l--)
		for (PxU32 i = 0; i < treeLevels[l].size(); i++)
		{
			const RTreeNodeNQ & n = treeLevels[l][i];
			if (n.childPageFirstNodeIndex == -1)
				continue;
			PxVec3 dims = n.bounds.getDimensions();
			PxVec3 center = n.bounds.getCenter();
			// in MAX width is x, length is y and height is z
			// Also MAX Box() has a center in the middle of the x and y and at the base of the box in z
			// in PhysX height is y
			fprintf(f, "box name: \"rtree_%02d_%05d\" width: %.4f length: %.4f height: %.4f pos:[%.4f, %.4f, %.4f]",
				l, i, dims.x, dims.z, dims.y, center.x, center.z, center.y-dims.y*0.5f);
			if (l < treeLevels.size()-1)
				fprintf(f, " parent:(getNodeByName(\"rtree_%02d_%05d\"))\n", l+1, i/pageSize);
			else
				fprintf(f, "\n");
		}

	fclose(f);
}
#endif

/////////////////////////////////////////////////////////////////////////
struct RTreeNodeQuantizer
{
	static PxU16 cvtPxF32ToPxU16(PxF32 f)
	{
		PX_ASSERT(f >= 0.0f && f <= PxF32(0xFFFF));
		return PxU16(f);
	}

	static void Swap(PxU16& a, PxU16& b) { PxU16 tmp = a; a = b; b = tmp; }

	static PxVec3 computeInvDiagUpdateBounds(PxBounds3& treeBounds)
	{
		PxVec3 treeDiag = treeBounds.getDimensions();
		PxVec3 boundsEpsilon(RTREE_MINIMUM_BOUNDS_EPSILON);
		treeDiag += boundsEpsilon * 2.0f;
		treeBounds.maximum += boundsEpsilon; // adjust bounds after PX_EPS_F32 expansion for dims
		treeBounds.minimum -= boundsEpsilon;
		// in addition, inflate the bounds so that we have at least 1 quantization step of empty space on each side
		// this is done so we can clamp a quantized query to [1,65534] range without excluding any objects
		// and have the [65535,0] inverted sentinel range used for empty nodes to always return no intersection without additional checks
		boundsEpsilon = treeBounds.getDimensions() / 65536.0f;
		treeBounds.minimum -= boundsEpsilon * 1.5f;
		treeBounds.maximum += boundsEpsilon * 1.5f;
		treeDiag = treeBounds.getDimensions();
		treeDiag.x = 1.0f / treeDiag.x;
		treeDiag.y = 1.0f / treeDiag.y;
		treeDiag.z = 1.0f / treeDiag.z;
		return treeDiag;
	}

	static RTreeNodeQ quantize(
		const PxVec3& nqMin, const PxVec3& nqMax, const PxBounds3& treeBounds, const PxVec3& invDiagonal
	)
	{
		// since we quantize conservatively, we perform queryMax<=treeMin for rejection (not queryMax<treeMin)
		// (consider floor(treeMin=1.02) quantizing to 1 on min-size, and ceil(queryMax=1.01) quantizing to 2 on max-side)
		// offline we scale min & max to full 16 bit range-2 plus unused 1 on each side to leave room for out-of-bound compares
		// then at runtime we clamp "wide" so that bounds that are out of range of quantization still produce correct results
		// note: there is a simd version of this routine in RTreeQuery.cpp

		// Quantization logic derivation for Q=query, T=tree
		// NO OVERLAP IFF (exists maxQ <= minT || exists minQ>=maxT)
		// IFF (exists maxQ < minT+1 || exists minQ > maxT-1)
		PxF32 hiClamp = PxF32(0xFFFF);
		PxF32 loClamp = PxF32(0);
		RTreeNodeQ q;
		PxF32 range = PxF32(0xFFFF); // using narrow clamped quantization range for both narrow and wide clamp
		PxVec3 scaledMin = (nqMin - treeBounds.minimum).multiply(invDiagonal) * range;
		PxVec3 scaledMax = (nqMax - treeBounds.minimum).multiply(invDiagonal) * range;
		q.minx = cvtPxF32ToPxU16(PxClamp(PxFloor(scaledMin.x), loClamp, hiClamp));
		q.maxx = cvtPxF32ToPxU16(PxClamp(PxCeil(scaledMax.x), loClamp, hiClamp));
		q.miny = cvtPxF32ToPxU16(PxClamp(PxFloor(scaledMin.y), loClamp, hiClamp));
		q.maxy = cvtPxF32ToPxU16(PxClamp(PxCeil(scaledMax.y), loClamp, hiClamp));
		q.minz = cvtPxF32ToPxU16(PxClamp(PxFloor(scaledMin.z), loClamp, hiClamp));
		q.maxz = cvtPxF32ToPxU16(PxClamp(PxCeil(scaledMax.z), loClamp, hiClamp));

		// see derivation above - we shrink the tree node by 2 and quantized query by 1
		q.minx += (q.minx != 0xFFFF) ? 1 : 0;
		q.miny += (q.miny != 0xFFFF) ? 1 : 0;
		q.minz += (q.minz != 0xFFFF) ? 1 : 0;
		q.maxx -= (q.maxx != 0) ? 1 : 0;
		q.maxy -= (q.maxy != 0) ? 1 : 0;
		q.maxz -= (q.maxz != 0) ? 1 : 0;

		q.ptr = 0;

		return q;
	}
};

/////////////////////////////////////////////////////////////////////////
RTree::RTree()
{
	PX_ASSERT((PxMemFetchPtr(this) & 15) == 0);
	mFlags = 0;
	mPages = NULL;
	mTotalNodes = 0;
	mNumLevels = 0;
	mPageSize = RTreePage::SIZE;
}

/////////////////////////////////////////////////////////////////////////
void RTree::buildFromLeafTriangles(
	const void* leafTrianglesVoid, PxU32 numLeaves,
	const PxVec3* verts, PxU32 numVerts, const PxU16* tris16, const PxU32* tris32, PxU32 numTris)
{
	release();

	Ice::LeafTriangles* leafTriangles = (Ice::LeafTriangles*)leafTrianglesVoid;
	Array<PxBounds3> allBounds;
	Array<PxU32> boundedData;
	allBounds.reserve(numLeaves);
	boundedData.reserve(numLeaves);
	for (PxU32 i = 0; i < numLeaves; i ++)
	{
		const Ice::LeafTriangles& lTris = leafTriangles[i];
		PxU32 firstTriIdx = lTris.GetTriangleIndex();
		PxBounds3 b(PxBounds3::empty());
		for (PxU32 iTri = 0; iTri < lTris.GetNbTriangles(); iTri++)
		{
			PxU32 idx = firstTriIdx + iTri;
			PxVec3 vert0 = verts[tris16 ? tris16[idx*3 + 0] : tris32[idx*3 + 0]];
			PxVec3 vert1 = verts[tris16 ? tris16[idx*3 + 1] : tris32[idx*3 + 1]];
			PxVec3 vert2 = verts[tris16 ? tris16[idx*3 + 2] : tris32[idx*3 + 2]];
			b.include(vert0);
			b.include(vert1);
			b.include(vert2);
		}
		if (lTris.GetNbTriangles())
		{
			allBounds.pushBack(b);
			boundedData.pushBack(leafTriangles[i].Data);
		}
	}

	buildFromBounds(allBounds.begin(), boundedData.begin(), allBounds.size());
}

/////////////////////////////////////////////////////////////////////////
struct SortBoundsPredicate
{
	PxU32 coordIndex;
	const PxBounds3* allBounds;
	SortBoundsPredicate(PxU32 coordIndex, const PxBounds3* allBounds)
		: coordIndex(coordIndex), allBounds(allBounds)
	{}

	bool operator()(const PxU32 & idx1, const PxU32 & idx2) const
	{
		PxF32 center1 = allBounds[idx1].getCenter()[coordIndex];
		PxF32 center2 = allBounds[idx2].getCenter()[coordIndex];
		return (center1 < center2);
	}
};

static PxU32 PxVec3MaxElementIndex(const PxVec3 & v) { return (v.x >= v.y && v.x >= v.z) ? 0 : (v.y >= v.z ? 1 : 2); }

/////////////////////////////////////////////////////////////////////////
void RTree::buildFromTriangles(const PxVec3* verts, PxU32 numVerts, const PxU32* tris, PxU32 numTris)
{
	Array<PxBounds3> allBounds;
	Array<PxU32> boundedData;
	allBounds.reserve(numTris);
	boundedData.reserve(numTris);
	for (PxU32 i = 0; i < numTris; i ++)
	{
		PxVec3 vert0 = verts[tris[i*3 + 0]];
		PxVec3 vert1 = verts[tris[i*3 + 1]];
		PxVec3 vert2 = verts[tris[i*3 + 2]];
		PxBounds3 & b = allBounds.pushBack(PxBounds3::boundsOfPoints(vert0, vert1));
		b.include(vert2);
		boundedData.pushBack(i);
	}

	buildFromBounds(allBounds.begin(), boundedData.begin(), numTris);
}

/////////////////////////////////////////////////////////////////////////
void RTree::buildFromBounds(const PxBounds3* allBounds, const PxU32* boundedData, PxU32 numBounds)
{

	/////////////////////////////////////////////////////////////////////////
	struct SubSort
	{
		PxU32* permute;
		const PxBounds3* allBounds;
		SubSort(PxU32 * permute, const PxBounds3 * allBounds)
			: allBounds(allBounds) {}

		// scaffold - this is slow, optimize (probably because of quicksort pathological O(N^2))
		void sortBinary(PxU32* permute, PxU32 clusterSize)
		{
			if (clusterSize <= 1)
				return;
			PxBounds3 clusterBounds(PxBounds3::empty());
			for (PxU32 i = 0; i < clusterSize; i ++)
				clusterBounds.include(allBounds[permute[i]]);

			// split along longest axis
			PxU32 maxDiagElement = PxVec3MaxElementIndex(clusterBounds.getDimensions());
			SortBoundsPredicate sortPredicate(maxDiagElement, allBounds);
			Ps::sort<PxU32, SortBoundsPredicate>(permute, clusterSize, sortPredicate);

			PxU32 halfSize = 1;
			while (halfSize*2 < clusterSize)
				halfSize *= 2;
			// sort first half
			sortBinary(permute, halfSize);
			// sort second half
			sortBinary(permute + halfSize, clusterSize - halfSize);
		}
	};

	/////////////////////////////////////////////////////////////////////////
	struct BuildRecursive
	{
		static void build(PxU32 pageSize, Array<Array<RTreeNodeNQ> > & resultTree)
		{
			resultTree.reserve(resultTree.capacity() + 1);
			Array<RTreeNodeNQ> & curLevel = resultTree.back();
			// pad with empty nodes to align to pageSize
			while (curLevel.size() % pageSize != 0)
				curLevel.pushBack(RTreeNodeNQ());
			PxU32 numNodes = curLevel.size();
			if (numNodes <= 2 * pageSize)
				return;

			Array<RTreeNodeNQ> & parentLevel = resultTree.pushBack(Array<RTreeNodeNQ>());
			for (PxU32 offs = 0; offs < numNodes; offs += pageSize)
			{
				PxBounds3 parentBound(PxBounds3::empty());
				PxU32 offsHigh = PxMin(numNodes, offs + pageSize);
				for (PxU32 i = offs; i < offsHigh; i++)
					parentBound.include(curLevel[i].bounds);
				RTreeNodeNQ rtn;
				rtn.bounds = parentBound;
				rtn.childPageFirstNodeIndex = offs;
				PX_ASSERT(rtn.childPageFirstNodeIndex % 8 == 0);
				parentLevel.pushBack(rtn);
			}
			build(pageSize, resultTree);
		}
	};

	// build overall tree bounds
	PxBounds3 treeBounds(PxBounds3::empty());
	for (PxU32 i = 0; i < numBounds; i ++)
		treeBounds.include(allBounds[i]);

	// start off with an identity permutation
	Array<PxU32> permute;
	permute.reserve(numBounds);
	for (PxU32 j = 0; j < numBounds; j ++)
		permute.pushBack(j);

	// sort by shuffling the permutation
	SubSort ss(permute.begin(), allBounds);
	ss.sortBinary(permute.begin(), permute.size());

	// load sorted nodes into an RTreeNodeNQ tree representation
	Array<Array<RTreeNodeNQ> > resultTree;
	Array<RTreeNodeNQ>& bottomLevel = resultTree.pushBack(Array<RTreeNodeNQ>());
	bottomLevel.reserve(numBounds);
	for (PxU32 j = 0; j < numBounds; j ++)
	{
		RTreeNodeNQ rtn;
		rtn.bounds = allBounds[permute[j]];
		rtn.childPageFirstNodeIndex = boundedData[permute[j]];
		bottomLevel.pushBack(rtn);
	}

	// build the tree structure from sorted nodes
	const PxU32 pageSize = 8;
	BuildRecursive::build(pageSize, resultTree);

	#ifdef EXPORT_TO_MAXSCRIPT
	// export to maxscript for debugging
	exportRTreeToMaxscript(pageSize, resultTree);
	#endif

	// Quantize the tree
	Array<RTreeNodeQ> qtreeNodes;
	PxVec3 invDiag = RTreeNodeQuantizer::computeInvDiagUpdateBounds(treeBounds);
	PxU32 firstEmptyIndex = -1;
	PxU32 sumLevelCounts = 0;
	PxU32 bottomLevelFirstNodeIndex = 0;
	for (int l = resultTree.size()-1; l >=0; l--)
	{
		PxU32 levelCount = resultTree[l].size();
		if (l == 0)
			bottomLevelFirstNodeIndex = sumLevelCounts;
		sumLevelCounts += levelCount;
		for (PxU32 i = 0; i < levelCount; i++)
		{
			const RTreeNodeNQ & u = resultTree[l][i];
			RTreeNodeQ q;
			if (u.childPageFirstNodeIndex == -1)
			{
				if (firstEmptyIndex == -1)
					firstEmptyIndex = qtreeNodes.size();
				q.minx = q.miny = q.minz = 0xFFFF;
				q.maxx = q.maxy = q.maxz = 0;
				q.ptr = firstEmptyIndex;
			} else
			{
				q = RTreeNodeQuantizer::quantize(u.bounds.minimum, u.bounds.maximum, treeBounds, invDiag);
				if (l == 0)
					q.ptr = PxU32(u.childPageFirstNodeIndex);
				else {
					q.ptr = (sumLevelCounts + i * pageSize);
					PX_ASSERT(q.ptr % 8 == 0);
				}
			}
			qtreeNodes.pushBack(q);
		}
	}

	// build quantized rtree image
	mInvDiagonal = PxVec4(invDiag.x, invDiag.y, invDiag.z, 0.0f);
	RTreeNodeQ* mNodes = static_cast<RTreeNodeQ*>(
		Ps::AlignedAllocator<128>().allocate(sizeof(RTreeNodeQ)*qtreeNodes.size(), __FILE__, __LINE__));
	mNodes = static_cast<RTreeNodeQ*>(
		Ps::memCopy(mNodes, qtreeNodes.begin(), sizeof(RTreeNodeQ)*qtreeNodes.size()));
	mPages = (RTreePage*)mNodes;
	mBoundsMin = PxVec4(treeBounds.minimum.x, treeBounds.minimum.y, treeBounds.minimum.z, 0.0f);
	mBoundsMax = PxVec4(treeBounds.maximum.x, treeBounds.maximum.y, treeBounds.maximum.z, 0.0f);
	mDiagonalScaler = (mBoundsMax - mBoundsMin) / 65535.0f;
	mPageSize = pageSize;
	mNumLevels = resultTree.size();
	mTotalNodes = qtreeNodes.size();
	mTotalPages = mTotalNodes / pageSize;
	PX_ASSERT(mTotalNodes % pageSize == 0);
	mBottomLevelFirstNodeIndex = bottomLevelFirstNodeIndex;
	mNumRootPages = resultTree.back().size() / pageSize;

	// transpose from RTreeNodeQs to RTreePage
	union Transpose {
		RTreePage p;
		RTreeNodeQ n[8];
	};
	for (PxU32 j = 0; j < mTotalPages; j++)
	{
		Transpose& t = ((Transpose*)mPages)[j];
		Transpose s = t;
		for (int k = 0; k < 8; k ++)
		{
			t.p.maxx[k] = s.n[k].maxx;
			t.p.maxy[k] = s.n[k].maxy;
			t.p.maxz[k] = s.n[k].maxz;
			t.p.minx[k] = s.n[k].minx;
			t.p.miny[k] = s.n[k].miny;
			t.p.minz[k] = s.n[k].minz;
			t.p.ptrs[k] = s.n[k].ptr;
			if (j*8 < bottomLevelFirstNodeIndex)
				PX_ASSERT(t.p.ptrs[k] % 8 == 0 || t.p.ptrs[k] == firstEmptyIndex);
		}
	}
}

/////////////////////////////////////////////////////////////////////////
PxU32 RTree::mVersion = 1;

bool RTree::save(PxStream& stream) const
{
	/*
		PxVec4		mBoundsMin, mBoundsMax, mInvDiagonal, mDiagonalScaler;
		PxU32		mPageSize;
		PxU32		mNumRootPages;
		PxU32		mNumLevels;
		PxU32		mTotalNodes;
		PxU32		mTotalPages;
		PxU32		mBottomLevelFirstNodeIndex;
		RTreePage*	mPages;
	*/

	bool mismatch = (LittleEndian() == 1);
	WriteChunk('R', 'T', 'R', 'E', stream);
	WriteDword(mVersion, mismatch, stream);
	WriteFloatBuffer(&mBoundsMin.x, 4, mismatch, stream);
	WriteFloatBuffer(&mBoundsMax.x, 4, mismatch, stream);
	WriteFloatBuffer(&mInvDiagonal.x, 4, mismatch, stream);
	WriteFloatBuffer(&mDiagonalScaler.x, 4, mismatch, stream);
	WriteDword(mPageSize, mismatch, stream);
	WriteDword(mNumRootPages, mismatch, stream);
	WriteDword(mNumLevels, mismatch, stream);
	WriteDword(mTotalNodes, mismatch, stream);
	WriteDword(mTotalPages, mismatch, stream);
	WriteDword(mBottomLevelFirstNodeIndex, mismatch, stream);
	for (PxU32 j = 0; j < mTotalPages; j++)
	{
		WriteWordBuffer(mPages[j].minx, 8, mismatch, stream);
		WriteWordBuffer(mPages[j].miny, 8, mismatch, stream);
		WriteWordBuffer(mPages[j].minz, 8, mismatch, stream);
		WriteWordBuffer(mPages[j].maxx, 8, mismatch, stream);
		WriteWordBuffer(mPages[j].maxy, 8, mismatch, stream);
		WriteWordBuffer(mPages[j].maxz, 8, mismatch, stream);
		WriteDwordBuffer(mPages[j].ptrs, 8, mismatch, stream);
	}

	return true;
}

/////////////////////////////////////////////////////////////////////////
bool RTree::load(const PxStream& stream)
{
	release();

	PxU8 a, b, c, d;
	ReadChunk(a, b, c, d, stream);
	if(a!='R' || b!='T' || c!='R' || d!='E')
		return false;

	bool mismatch = (LittleEndian() == 1);
	if (ReadDword(mismatch, stream) != mVersion)
		return false;

	ReadFloatBuffer(&mBoundsMin.x, 4, mismatch, stream);
	ReadFloatBuffer(&mBoundsMax.x, 4, mismatch, stream);
	ReadFloatBuffer(&mInvDiagonal.x, 4, mismatch, stream);
	ReadFloatBuffer(&mDiagonalScaler.x, 4, mismatch, stream);
	mPageSize = ReadDword(mismatch, stream);
	mNumRootPages = ReadDword(mismatch, stream);
	mNumLevels = ReadDword(mismatch, stream);
	mTotalNodes = ReadDword(mismatch, stream);
	mTotalPages = ReadDword(mismatch, stream);
	mBottomLevelFirstNodeIndex = ReadDword(mismatch, stream);

	mPages = static_cast<RTreePage*>(
		Ps::AlignedAllocator<128>().allocate(sizeof(RTreePage)*mTotalPages, __FILE__, __LINE__));
	for (PxU32 j = 0; j < mTotalPages; j++)
	{
		ReadWordBuffer(mPages[j].minx, 8, mismatch, stream);
		ReadWordBuffer(mPages[j].miny, 8, mismatch, stream);
		ReadWordBuffer(mPages[j].minz, 8, mismatch, stream);
		ReadWordBuffer(mPages[j].maxx, 8, mismatch, stream);
		ReadWordBuffer(mPages[j].maxy, 8, mismatch, stream);
		ReadWordBuffer(mPages[j].maxz, 8, mismatch, stream);
		ReadDwordBuffer(mPages[j].ptrs, 8, mismatch, stream);
	}

	return true;
}

/////////////////////////////////////////////////////////////////////////
RTree::RTree(PxRefResolver& v)
{
	mFlags = USER_ALLOCATED;
	mPages = NULL;
	mTotalNodes = 0;
	mNumLevels = 0;
	mPageSize = RTreePage::SIZE;
}

/////////////////////////////////////////////////////////////////////////
void RTree::release()
{
	if ((mFlags & USER_ALLOCATED) == 0 && mPages)
	{
		Ps::AlignedAllocator<128>().deallocate(mPages);
		mPages = NULL;
	}
}

// PX_SERIALIZATION
/////////////////////////////////////////////////////////////////////////
void RTree::exportExtraData(PxSerialStream& stream)
{
	Cm::alignStream(stream, 128);
	stream.storeBuffer(mPages, mTotalPages*sizeof(RTreePage));
}

/////////////////////////////////////////////////////////////////////////
char* RTree::importExtraData(char* address, PxU32& totalPadding)
{
	address = Cm::alignStream(address, totalPadding, 128);
	mPages = (RTreePage*)address;
	address += mTotalPages*sizeof(RTreePage);

	return address;
}

//~PX_SERIALIZATION

} // namespace Gu

}
