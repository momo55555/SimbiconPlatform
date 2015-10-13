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



// PT: hack to avoid compiling the file alone. Blame it on the lazy build system
// that includes all cpp files from a folder...
#ifdef COMPILE_ME

#include "SqDynamicPruner.h"
#include "PsUserAllocated.h"
#include "GuIntersectionBoxBox.h"
#include "GuIntersectionCapsuleBox.h"
#include "GuIntersectionSphereBox.h"
#include "./Ice/CTC_RayAABBOverlap.h"
#include "./Ice/CTC_PlaneAABBOverlap.h"

#ifdef USE_QUADTREE
/*	PX_INLINE PxU32 ComputeMortonIndex(PxU32 j, PxU32 k, PxU32 level)
	{
		PxU32 Index = 0;
		for(PxU32 n=0;n<level;n++)
		{
			PxU32 Mask = 1<<n;

			PxU32 jb = (j&Mask)>>n;
			PxU32 kb = (k&Mask)>>n;

			PxU32 Bits = ((kb)<<(n*2+0)) | ((jb)<<(n*2+1));

			Index|=Bits;
		}
		return Index;
	}*/

#ifdef USE_MORTON
	static	// VC7.1 bug
	PX_FORCE_INLINE PxU32 Part1By1(PxU32 n)
	{
		n = (n^(n<<8)) & 0x00ff00ff;
		n = (n^(n<<4)) & 0x0f0f0f0f;
		n = (n^(n<<2)) & 0x33333333;
		n = (n^(n<<1)) & 0x55555555;
		return n;
	}

	static	// VC7.1 bug
	PX_FORCE_INLINE PxU32 ComputeMortonIndex(PxU32 y, PxU32 x, PxU32 level)
	{
		return (Part1By1(y)<<1) + Part1By1(x);
	}
#else
	static	// VC7.1 bug
	PX_FORCE_INLINE PxU32 ComputeMortonIndex(PxU32 y, PxU32 x, PxU32 level)
	{
		return x+(y<<2);
	}
#endif

static	// VC7.1 bug
PX_FORCE_INLINE bool IsInside(const AABB_2D& b, const AABB_2D& box)
{
	if(box.minimum[0]>b.minimum[0])	return false;
	if(box.minimum[1]>b.minimum[1])	return false;
	if(box.maximum[0]<b.maximum[0])	return false;
	if(box.maximum[1]<b.maximum[1])	return false;
	return true;
}

#else

	static PX_FORCE_INLINE PxU32 ComputeMortonIndex(PxU32 i, PxU32 j, PxU32 k, PxU32 level)
	{
		PxU32 Index = 0;
		for(PxU32 n=0;n<level;n++)
		{
			const PxU32 Mask = 1<<n;

			const PxU32 ib = (i&Mask)>>n;
			const PxU32 jb = (j&Mask)>>n;
			const PxU32 kb = (k&Mask)>>n;

			const PxU32 Bits = ((kb)<<(n*3+0)) | ((jb)<<(n*3+1)) | ((ib)<<(n*3+2));

			Index|=Bits;
		}
		return Index;
	}

static	// VC7.1 bug
PX_FORCE_INLINE bool IsInside(const PxBounds3& b, const PxBounds3& box)
{
	if(box.minimum[0]>b.minimum[0])	return false;
	if(box.minimum[1]>b.minimum[1])	return false;
	if(box.minimum[2]>b.minimum[2])	return false;
	if(box.maximum[0]<b.maximum[0])	return false;
	if(box.maximum[1]<b.maximum[1])	return false;
	if(box.maximum[2]<b.maximum[2])	return false;
	return true;
}

#endif

//#define OCTREE_CHECK

	static	// VC7.1 bug
	PX_FORCE_INLINE	PxU32 ComputeOctreeStride(PxU32 level)
	{
		return 1<<level;
	}

/*
#ifdef USE_QUADTREE
	static	// VC7.1 bug
	PX_INLINE	PxU32 ComputeNbQuadtreeCells(PxU32 level)
	{
		PxU32 Stride = ComputeOctreeStride(level);
		return Stride * Stride;
	}
#else
	static	// VC7.1 bug
	PX_INLINE	PxU32 ComputeNbOctreeCells(PxU32 level)
	{
		PxU32 Stride = ComputeOctreeStride(level);
		return Stride * Stride * Stride;
	}
#endif
*/

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	Computes the total number of cells in a linear octree, given a maximal depth.
	 *	\param		max_depth	[in] maximum depth for the linear octree
	 *	\return		total number of cells
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	static	// VC7.1 bug
	PX_FORCE_INLINE	PxU32 ComputeTotalNbCells(PxU32 max_depth)
	{
#ifdef USE_QUADTREE
		static PxU32 Nb[] = { 1, 5, 21, 85, 341, 1365, 5461, 21845, 87381 };
		PX_ASSERT(max_depth<9);
		return Nb[max_depth];
//		PxU32 Size = 2<<max_depth;
//		return (Size*Size - 1)/3;
#else
		const PxU32 Size = 2<<max_depth;
		return (Size*Size*Size - 1)/7;
#endif
	}

	static	// VC7.1 bug
	PX_FORCE_INLINE	PxU32 ComputeOctreeOffset(PxU32 level)
	{
		if(!level)	return 0;
		return ComputeTotalNbCells(level-1);
	}

	#define	INVLN2	1.44269504089f	//!< 1.0f / ln(2)
	static	// VC7.1 bug
	PX_FORCE_INLINE	PxU32 ComputeOctreeDepth(float world_size, float radius)
	{
		return PxU32(logf(world_size/(radius*2.0f)) * INVLN2);
	}

	// The FPU trick
	PX_FORCE_INLINE PxU32 CodeSize2(PxU32 value)
	{
		int n;
		*((float*)&n) = (float)value;
		n = (n>>23)-126;
		return n;
	}

	static	// VC7.1 bug
	PX_FORCE_INLINE	PxU32 ComputeOctreeDepth2(float world_size, float radius)
	{
		return CodeSize2(PxU32(world_size / (radius*4.0f)));
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Makes a cube from the AABB.
 *	\param		cube	[out] the cube AABB
 *	\return		cube edge length
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_QUADTREE
static float MakeCube(const AABB_2D& src, AABB_2D& cube)
{
	Point2D Ext;	src.getExtents(Ext);
	const float Max = Ext.Max();

	Point2D Cnt;	src.getCenter(Cnt);
	cube.setCenterExtents(Cnt, Point2D(Max, Max));
	return Max;
}
#else
static float MakeCube(const PxBounds3& src, PxBounds3& cube)
{
	PxVec3 Ext = src.getExtents();
	const float Max = Ext.maxElement();

	PxVec3 Cnt = src.getCenter();
	cube = PxBounds3::centerExtents(Cnt, PxVec3(Max, Max, Max));
	return Max;
}
#endif

#ifdef USE_QUADTREE
	#ifdef USE_MORTON
	static	// VC7.1 bug
	PX_FORCE_INLINE void UndoMortonIndex(PxU32 index, PxU32& _j, PxU32& _k)
	{
		PxU32 j,k;
		j = k = 0;
		PxU32 n=0;
		while(index)
		{
/*			PxU32 kb = index & 1;
			PxU32 jb = (index & 2)>>1;
			j |= (jb<<n);
			k |= (kb<<n);
			n++;
			index>>=2;*/
			k |= (index & 1)<<n;
			j |= (index & 2)<<n;
			n++;
			index>>=2;
		}
		_j = j>>1;
		_k = k;
	}
	#else
	static	// VC7.1 bug
	PX_FORCE_INLINE void UndoMortonIndex(PxU32 index, PxU32& j, PxU32& k)
	{
		Compute2DCoords(k, j, index, 4);
	}
	#endif
#else
	static	// VC7.1 bug
	PX_FORCE_INLINE void UndoMortonIndex(PxU32 index, PxU32& _i, PxU32& _j, PxU32& _k)
	{
		PxU32 i,j,k;
		i = j = k = 0;
		PxU32 n=0;
		while(index)
		{
			PxU32 kb = index & 1;
			PxU32 jb = (index & 2)>>1;
			PxU32 ib = (index & 4)>>2;
			i |= (ib<<n);
			j |= (jb<<n);
			k |= (kb<<n);
			n++;
			index>>=3;
		}
		_i = i;
		_j = j;
		_k = k;
	}
#endif
	static	// VC7.1 bug
	PX_FORCE_INLINE PxU32 ComputeDepth(PxU32 index)
	{
		PxU32 Depth = 0;
		while(1)
		{
			PxU32 Limit = ComputeTotalNbCells(Depth);
			if(index<Limit)	return Depth;
			Depth++;
		}
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TREE_CLASS_NAME::TREE_CLASS_NAME()
{
	mWorldOffset = OctreePoint(0);
	mWorldSize	= 0.0f;
	mMaxDepth	= 0;
	mNbCells	= 0;
	mCells		= NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TREE_CLASS_NAME::~TREE_CLASS_NAME()
{
	Release();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Releases everything.
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool TREE_CLASS_NAME::Release()
{
	PX_DELETE_ARRAY(mCells);
	mNbCells = 0;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive octree initialization.
 *	\param		index	[in] cell index. Validity interval is [0, mNbCells[
 *	\param		box		[in] cell's box
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef USE_IMPLICIT_BOX
void TREE_CLASS_NAME::_InitOctree(PxU32 index, const OctreeBox& box)
{
	// Check index
	if(index>=mNbCells)	return;

	mCells[index].mBox = box;	// ### could be implicit ? => yeah, we can compute it while recursively culling => but we need it for updates
	//### maybe can be computed with the node index only, cf oni

	// Current box: minimum, maximum
	// 8 children:
	// 0 = keep value
	// 1 = divide / 2
	// 000 : minx, m

	OctreePoint Center = box.getCenter();

	for(PxU32 CubeIndex=1;CubeIndex<=NB_OCTREE_CHILDREN;CubeIndex++)
	{
		OctreePoint Min = box.minimum;
		OctreePoint Max = box.maximum;

		switch(CubeIndex)
		{
			// 0 = +
			// 1 = -
#ifdef USE_QUADTREE
			case 1: Max.x = Center.x; Max.y = Center.y; 	break;	// 00
			case 2: Max.x = Center.x; Min.y = Center.y; 	break;	// 01
			case 3: Min.x = Center.x; Max.y = Center.y; 	break;	// 10
			case 4: Min.x = Center.x; Min.y = Center.y; 	break;	// 11
#else
			case 1: Max.x = Center.x; Max.y = Center.y; Max.z = Center.z;	break;	// 000
			case 2: Max.x = Center.x; Max.y = Center.y; Min.z = Center.z;	break;	// 001
			case 3: Max.x = Center.x; Min.y = Center.y; Max.z = Center.z;	break;	// 010
			case 4: Max.x = Center.x; Min.y = Center.y; Min.z = Center.z;	break;	// 011
			case 5: Min.x = Center.x; Max.y = Center.y; Max.z = Center.z;	break;	// 100
			case 6: Min.x = Center.x; Max.y = Center.y; Min.z = Center.z;	break;	// 101
			case 7: Min.x = Center.x; Min.y = Center.y; Max.z = Center.z;	break;	// 110
			case 8: Min.x = Center.x; Min.y = Center.y; Min.z = Center.z;	break;	// 111
#endif
		}

		OctreeBox NewBox;
		NewBox.minimum = Min;
		NewBox.maximum = Max;

		// Compute child index & recurse
		_InitOctree(MAKE_CHILD_INDEX(index, CubeIndex), NewBox);
	}
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Traverses the whole octree, calling the user back for each cell.
 *	\param		cb			[in] user-defined callback
 *	\param		user_data	[in] user-defined data
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool TREE_CLASS_NAME::Walk(CALLBACK_NAME cb, void* user_data) const
{
	// Checkings
	if(!cb)	return false;

	// Start at index 0 with no parent (NULL)
	_Walk(0, NULL, cb, user_data);

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Recursive walking.
 *	\param		index	[in] index of current cell
 *	\param		parent	[in] parent cell
 *	\param		cb		[in] user-defined callback
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TREE_CLASS_NAME::_Walk(PxU32 index, CELL_CLASS_NAME* parent, CALLBACK_NAME cb, void* user_data) const
{
	// Check index. Valid interval is [0, mNbCells[
	if(index>=mNbCells)	return;

	// Get current cell
	CELL_CLASS_NAME* Current = &mCells[index];

	// User callback (has already been checked by the caller)
	bool Recurse = (cb)(parent, Current, user_data);
	// End recursion if false
	if(!Recurse)	return;

	// Loop through children
	for(PxU32 CubeIndex=1;CubeIndex<=NB_OCTREE_CHILDREN;CubeIndex++)
	{
		// Compute child index & recurse
		_Walk(MAKE_CHILD_INDEX(index, CubeIndex), Current, cb, user_data);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Initializes the loose octree.
 *	\param		max_depth	[in] maximum depth for the linear octree
 *	\param		world_box	[in] world's maximum bounding box
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_QUADTREE
bool TREE_CLASS_NAME::Init(PxU32 max_depth, const OctreeBox& world_box, PxU32 quadtree_up)
#else
bool TREE_CLASS_NAME::Init(PxU32 max_depth, const OctreeBox& world_box)
#endif
{
	// Make sure everything has been released for re-init
	Release();

#ifdef USE_QUADTREE
	if(max_depth>7)
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Quadtree: subdivision level is too high. Max recommended level is 7.");
#else
	if(max_depth>5)
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Octree: subdivision level is too high. Max recommended level is 5.");
#endif

	// Keep track of some parameters
#ifdef USE_QUADTREE
	mQuadtreeUp	= quadtree_up;
#endif
	mMaxDepth	= max_depth;
	mNbCells	= ComputeTotalNbCells(max_depth);
	mWorldBox	= world_box;

	// Allocate all cells
	mCells		= PX_NEW(CELL_CLASS_NAME)[mNbCells];

	// Compute world size
	OctreeBox RootCube;
	float InitSize = MakeCube(world_box, RootCube);	// This is only an extent, real edge length is InitSize * 2

//	Point Ext;	world_box.GetExtents(Ext);
//	Point Cnt;	world_box.GetCenter(Cnt);

//	float InitSize = Ext.Max();
	mWorldSize = InitSize * 2.0f;

	mWorldSize2		= mWorldSize/2.0f;
/*	mWorldSize4		= mWorldSize/4.0f;
	mWorldSize8		= mWorldSize/8.0f;
	mWorldSize16	= mWorldSize/16.0f;
	mWorldSize32	= mWorldSize/32.0f;
	mWorldSize64	= mWorldSize/64.0f;
	mWorldSize128	= mWorldSize/128.0f;
*/

//	RootBox.SetCenterExtents(Cnt, Point(InitSize, InitSize, InitSize));

	mWorldOffset = -RootCube.minimum;

#ifndef USE_IMPLICIT_BOX
	_InitOctree(0, RootCube);
#endif

/*
	The original cube length is N (see below). The loose version is N*2. So, the center of any object of size <= N can
	freely move within the original N-sided cube, and still be bounded by its loose version. Thus the maximum *radius* for
	a cube whose *original* size is N, is N/2. The center position must be tested against the *original* cubes, not their
	loose counterparts.

	       N*2
	<---------------->
	__________________
	|                |

	        N
	    <------->
	    _________
	    |       |
	 N/2|   ____|__
	<-->|  /       \
	    |  |    |  |
	    |  |   *   |
        ---| - -   |
	       |       |<----an object whose center is *
	       \-------/


	Level	#cells	Original size	Loose size		Max obj size	Max radius
	0		1		mWorldSize		mWorldSize*2	mWorldSize		mWorldSize/2
	1		8		mWorldSize/2	mWorldSize		mWorldSize/2	mWorldSize/4
	2		64		mWorldSize/4	mWorldSize/2	mWorldSize/4	mWorldSize/8
	3		512		mWorldSize/8	mWorldSize/4	mWorldSize/8	mWorldSize/16
	4		4096	mWorldSize/16	mWorldSize/8	mWorldSize/16	mWorldSize/32
	5		32768	mWorldSize/32	mWorldSize/16	mWorldSize/32	mWorldSize/64

	Once the octree is loose, each level of (original) size S is now twice as big, i.e. S/2 larger in each direction.
	So it can now handle any object of size S, regardless of the object's position. If we're working with a radius, it's S/2.

		Max radius(level) = mWorldSize / (2 * 2^level)

	We want R < radius(level) <= mWorldSize / (2 * 2^level)

	<=>	2^level <= mWorldSize/(2*R)
	<=> level <= log2(mWorldSize/(2*R))

	We obviously select the best level, i.e. the deepest one, i.e.

		level = log2(mWorldSize/(2*R))
*/
#ifndef USE_IMPLICIT_BOX
	struct Local
	{
		static bool MakeLoose(CELL_CLASS_NAME* parent, CELL_CLASS_NAME* current, void* user_data)
		{
			// ### setupable coeff ?
			if(current)	current->ScaleBox(2.0f);	
			return true;
		}
	};

	Walk(Local::MakeLoose, NULL);
#endif

//TEST
#ifdef TEST_OCTREE
float Coeff=2.0f;
PxU32 Level;
float Radius;
float Epsilon = 0.1f;
// Too big => fall in 0
Radius = mWorldSize/2.0f;			Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
Radius = mWorldSize/2.0f+Epsilon;	Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
Radius = mWorldSize/2.0f-Epsilon;	Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
// => 0 limit
Radius = mWorldSize/4.0f;			Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
// => a little bigger than the 0 limit => 0
Radius = mWorldSize/4.0f+Epsilon;	Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
// => a little smaller than the 0 limit => 1
Radius = mWorldSize/4.0f-Epsilon;	Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
// => 1 limit
Radius = mWorldSize/8.0f;			Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
// => a little bigger than the 1 limit => 1
Radius = mWorldSize/8.0f+Epsilon;	Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
// => a little smaller than the 1 limit => 2
Radius = mWorldSize/8.0f-Epsilon;	Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
// => 2 limit
Radius = mWorldSize/16.0f;			Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
// => a little bigger than the 2 limit => 2
Radius = mWorldSize/16.0f+Epsilon;	Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
// => a little smaller than the 2 limit => 3
Radius = mWorldSize/16.0f-Epsilon;	Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
// => 3 limit
Radius = mWorldSize/32.0f;			Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
// => a little bigger than the 3 limit => 3
Radius = mWorldSize/32.0f+Epsilon;	Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
// => a little smaller than the 3 limit => 4
Radius = mWorldSize/32.0f-Epsilon;	Level = logf(mWorldSize/(Radius*Coeff)) * INVLN2;
#endif



#ifdef REORGANIZE_OCTREE

	CELL_CLASS_NAME* NewCells = ICE_NEW_MEM(CELL_CLASS_NAME[mNbCells],OctreeCell);

//	PxU32* SortKeys = new PxU32[mNbCells];
	PxU32 CurrentCell=0;
	PxU32 Offset = 0;
	for(PxU32 Level=0;Level<=max_depth;Level++)
	{
		PxU32 Stride = ComputeOctreeStride(Level);
//		PxU32 Nb = ComputeNbOctreeCells(Level);

		// (i,j,k)  E  [0, Stride[
		// Stride*Stride*Stride = Nb

//Container SortKeys;
		for(PxU32 k=0;k<Stride;k++)
		{
			for(PxU32 j=0;j<Stride;j++)
			{
				for(PxU32 i=0;i<Stride;i++)
				{
					// Compute Peano index for this triplet
					PxU32 Index = ComputeMortonIndex(i, j, k, Level);

					NewCells[Offset+i+j*Stride+k*Stride*Stride] = mCells[Offset+Index];
//SortKeys.Add(Index);
//					NewCells[CurrentCell++] = mCells[Offset+Index];
//					SortKeys[CurrentCell++] = Offset
				}
			}
		}
/*
RadixSort RS;
PxU32* Ranks = RS.Sort(SortKeys.GetEntries(), Stride*Stride*Stride).GetRanks();

for(PxU32 i=0;i<Stride*Stride*Stride;i++)
{
	NewCells[CurrentCell++] = mCells[Offset+Ranks[i]];
}
*/
		Offset+=Stride*Stride*Stride;
	}
	ASSERT(Offset==mNbCells);

	PX_DELETE_ARRAY(mCells);
	mCells = NewCells;

#endif

	// TEST
/*	for(PxU32 i=0;i<mNbCells;i++)
	{
		AABB_2D Box;
		ComputeBox(i, mWorldOffset, mWorldSize, Box);
		const AABB_2D& bbb = mCells[i].GetBox();
	}
*/
	return true;
}

#ifdef USE_IMPLICIT_BOX
	#ifdef USE_QUADTREE
	static void ComputeBox(PxU32 index, const Point2D& offset, float world_size, Point2D& center, float& extent)
	{
		PxU32 Depth = ComputeDepth(index);
		PxU32 Stride = ComputeOctreeStride(Depth);
		extent = world_size/float(Stride);

		PxU32 BaseOffset = ComputeOctreeOffset(Depth);
		PxU32 j,k;
		UndoMortonIndex(index-BaseOffset, j, k);

		float HalfExt = extent*0.5f;
		center.x = float(j)*extent - offset.x + HalfExt;
		center.y = float(k)*extent - offset.y + HalfExt;
	}
	#else
	static void ComputeBox(PxU32 index, const Point& offset, float world_size, Point& center, float& extent)
	{
		PxU32 Depth = ComputeDepth(index);
		PxU32 Stride = ComputeOctreeStride(Depth);
		extent = world_size/float(Stride);

		PxU32 BaseOffset = ComputeOctreeOffset(Depth);
		PxU32 i,j,k;
		UndoMortonIndex(index-BaseOffset, i, j, k);

		float HalfExt = extent*0.5f;
		center.x = float(i)*extent - offset.x + HalfExt;
		center.y = float(j)*extent - offset.y + HalfExt;
		center.z = float(k)*extent - offset.z + HalfExt;
	}
	#endif

void TREE_CLASS_NAME::ComputeBox(PxU32 index, OctreeBox& box) const
{
	OctreePoint Center;
	float Extent;
	::ComputeBox(index, mWorldOffset, mWorldSize, Center, Extent);
#ifdef USE_QUADTREE
	box.setCenterExtents(Center, Point2D(Extent, Extent));
#else
	box.setCenterExtents(Center, PxVec3(Extent, Extent, Extent));
#endif
}

void TREE_CLASS_NAME::ComputeBox(PxU32 index, OctreePoint& center, float& extent) const
{
	::ComputeBox(index, mWorldOffset, mWorldSize, center, extent);
}

PxU32 TREE_CLASS_NAME::ComputeChildrenBoxes(PxU32 parent_index, OctreePoint* centers, float& extent, PxU32* indices) const
{
	PxU32 Index = MAKE_CHILD_INDEX(parent_index, 1);
	if(Index>=mNbCells)	return 0;

	// Compute common data
	const PxU32 Depth = ComputeDepth(Index);
//	const PxU32 Stride = ComputeOctreeStride(Depth);
//	extent = mWorldSize/float(Stride);
//	const float HalfExt = extent*0.5f;
#ifdef USE_QUADTREE
//	const float OffsetX = HalfExt - mWorldOffset.x;
//	const float OffsetY = HalfExt - mWorldOffset.y;
#else
	const float OffsetX = HalfExt - mWorldOffset.x;
	const float OffsetY = HalfExt - mWorldOffset.y;
	const float OffsetZ = HalfExt - mWorldOffset.z;
#endif
	const PxU32 BaseOffset = ComputeOctreeOffset(Depth);

	// Compute boxes
/*	PxU32 Count = 0;
	const PxU32 Last = Index + NB_OCTREE_CHILDREN;
	while(Index!=Last)
	{
		const CELL_CLASS_NAME& Current = mCells[Index];
		if(Current.GetCount())
		{
#ifdef USE_QUADTREE
			PxU32 j,k;
			UndoMortonIndex(Index-BaseOffset, j, k);

			indices[Count] = Index;
			centers[Count].x = float(j)*extent + OffsetX;
			centers[Count].y = float(k)*extent + OffsetY;
#else
			PxU32 i,j,k;
			UndoMortonIndex(Index-BaseOffset, i, j, k);

			indices[Count] = Index;
			centers[Count].x = float(i)*extent + OffsetX;
			centers[Count].y = float(j)*extent + OffsetY;
			centers[Count].z = float(k)*extent + OffsetZ;
#endif
			Count++;
		}
		Index++;
	}*/

// Following version doesn't seem that better in practice

#ifdef USE_QUADTREE
	float cx, cy;
	bool valid = false;
#endif
	PxU32 Count = 0;
	const PxU32 Last = Index + NB_OCTREE_CHILDREN;
PxU32 Offset = 0;
	while(Index!=Last)
	{
		const CELL_CLASS_NAME& Current = mCells[Index];
		if(Current.GetCount())
		{
#ifdef USE_QUADTREE
			if(!valid)
			{
				valid=true;

				const PxU32 Stride = ComputeOctreeStride(Depth);
				extent = mWorldSize/float(Stride);

				PxU32 j,k;
				UndoMortonIndex(Last - NB_OCTREE_CHILDREN - BaseOffset, j, k);

				const float HalfExt = extent*0.5f;
				const float OffsetX = HalfExt - mWorldOffset.x;
				const float OffsetY = HalfExt - mWorldOffset.y;
				cx = float(j)*extent + OffsetX;
				cy = float(k)*extent + OffsetY;
			}

//			indices[Count] = Index;
//			centers[Count].x = cx + (((Index-1) & 2) ? extent : 0.0f);
//			centers[Count].y = cy + (((Index-1) & 1) ? extent : 0.0f);
indices[Offset] = Index;
centers[Offset].x = cx + (((Index-1) & 2) ? extent : 0.0f);
centers[Offset].y = cy + (((Index-1) & 1) ? extent : 0.0f);
#endif
			Count++;
		}
else indices[Offset] = PX_INVALID_U32;
		Index++;
Offset++;
	}


/*#ifdef USE_QUADTREE
#ifdef _DEBUG
	// TEST
	{
		PxU32 ci = MAKE_CHILD_INDEX(parent_index, 1);
		PxU32 j1,k1;
		UndoMortonIndex(ci-BaseOffset, j1, k1);

		ci++;
		PxU32 j2,k2;
		UndoMortonIndex(ci-BaseOffset, j2, k2);
		ASSERT(j2==j1);
		ASSERT(k2==k1+1);

		ci++;
		PxU32 j3,k3;
		UndoMortonIndex(ci-BaseOffset, j3, k3);
		ASSERT(j3==j1+1);
		ASSERT(k3==k1);

		ci++;
		PxU32 j4,k4;
		UndoMortonIndex(ci-BaseOffset, j4, k4);
		ASSERT(j4==j1+1);
		ASSERT(k4==k1+1);

		int stop=0;
	}
	//~TEST
#endif
#endif
*/
	return Count;
}

void TREE_CLASS_NAME::ComputeOrder(const PxVec3& dir, PxU32* sorted) const
{
	OctreePoint Centers[NB_OCTREE_CHILDREN];
	float Extent;
//	PxU32 Indices[NB_OCTREE_CHILDREN];
//	ComputeChildrenBoxes(0, Centers, Extent, Indices);
	// PT: TODO: use hardcoded boxes
	{
	PxU32 Index = 1;
	const PxU32 Depth = ComputeDepth(Index);
#ifdef USE_QUADTREE
//	const float OffsetX = HalfExt - mWorldOffset.x;
//	const float OffsetY = HalfExt - mWorldOffset.y;
#else
	const float OffsetX = HalfExt - mWorldOffset.x;
	const float OffsetY = HalfExt - mWorldOffset.y;
	const float OffsetZ = HalfExt - mWorldOffset.z;
#endif
	const PxU32 BaseOffset = ComputeOctreeOffset(Depth);

#ifdef USE_QUADTREE
	float cx, cy;
	bool valid = false;
#endif
	PxU32 Count = 0;
	const PxU32 Last = Index + NB_OCTREE_CHILDREN;
	while(Index!=Last)
	{
//		const CELL_CLASS_NAME& Current = mCells[Index];
//		if(Current.GetCount())
		{
#ifdef USE_QUADTREE
			if(!valid)
			{
				valid=true;

				const PxU32 Stride = ComputeOctreeStride(Depth);
				Extent = mWorldSize/float(Stride);

				PxU32 j,k;
				UndoMortonIndex(Last - NB_OCTREE_CHILDREN - BaseOffset, j, k);

				const float HalfExt = Extent*0.5f;
				const float OffsetX = HalfExt - mWorldOffset.x;
				const float OffsetY = HalfExt - mWorldOffset.y;
				cx = float(j)*Extent + OffsetX;
				cy = float(k)*Extent + OffsetY;
			}

//			indices[Count] = Index;
			Centers[Count].x = cx + (((Index-1) & 2) ? Extent : 0.0f);
			Centers[Count].y = cy + (((Index-1) & 1) ? Extent : 0.0f);
#endif
			Count++;
		}
		Index++;
	}
	}

//	PxU32 Sorted[NB_OCTREE_CHILDREN];
	float dp[NB_OCTREE_CHILDREN];
#ifdef USE_QUADTREE
	const float dx = dir.x;
	const float dy = dir[mQuadtreeUp];
#endif
	for(PxU32 i=0;i<NB_OCTREE_CHILDREN;i++)
	{
		sorted[i] = i;
#ifdef USE_QUADTREE
		dp[i] = dx*Centers[i].x + dy*Centers[i].y;
#else
		dp[i] = sqd.mRay.mDir | Centers[i];
#endif
	}
	for(PxU32 j=0;j<NB_OCTREE_CHILDREN;j++)
	{
		bool Break = true;
		for(PxU32 i=0;i<NB_OCTREE_CHILDREN-1;i++)
		{
			if(dp[i]>dp[i+1])
			{
				Ice::TSwap(dp[i], dp[i+1]);
				Ice::TSwap(sorted[i], sorted[i+1]);
				Break = false;
			}
		}
		if(Break)	break;
	}
}
#endif

/*
PxU32 TestIndex2(PxU32 i, PxU32 j, PxU32 k)
{
	PxU32 Index = 0;
	for(PxU32 n=0;n<32;n++)
	{
		Index |= ((k&1)<<(n*3+0));
		Index |= ((j&1)<<(n*3+1));
		Index |= ((i&1)<<(n*3+2));

		i>>=1;
		j>>=1;
		k>>=1;
	}
	return Index;
}
*/

/*
PxU32 TestIndex(PxU32 i, PxU32 j, PxU32 k)
{
	int	l1 = lowest_one(x | z);
	int	depth = log_size - l1 - 1;

	int	base = 0x55555555 & ((1 << depth*2) - 1);	// total node count in all levels above ours.
	int	shift = l1 + 1;

	// Effective coords within this node's level.
	int	col = x >> shift;
	int	row = z >> shift;

	return base + (row << depth) + col;
*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Inserts an object in the linear octree.
 *	\param		obj				[in] object to insert
 *	\param		bounding_cube	[in] object's bounding cube, if available (else NULL)
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool TREE_CLASS_NAME::InsertObject(Prunable& obj, const OctreeBox* bounding_cube)
{
//### test against worldbox ?

	// 1) Compute object's location in the octree = owner cell's index
/*
AABB blah;
blah.SetMinMax(Point(113.39198f, 1405.9547f, 25.684986f), Point(114.01275f, 1406.5756f, 26.305767f));
bounding_cube = &blah;
*/

	// Catch object's bounds
	OctreeBox TestCube;
	const OctreeBox* CurrentCube;
	OctreeSphere WorldSphere;
//	float Radius;
//	Point Center;
	if(bounding_cube)
	{
		// Use provided data
//		Radius = bounding_cube->GetExtents(0);
//		bounding_cube->GetCenter(Center);
		WorldSphere.radius = bounding_cube->getExtents(0);
		WorldSphere.center = bounding_cube->getCenter();
		CurrentCube = bounding_cube;
	}
	else
	{
		// Catch data
//		Radius = LocalSphere->mRadius;
//		Center = LocalSphere->mCenter;
//		if(WorldTransform)	Center *= *WorldTransform;

		// PT: the sphere callback is gone in 3.0, so we're left with a slightly worse AABB. The AABB is a tighter BV but the octree code only works with spheres,
		// so we'll need to derive a sphere from the AABB, which is not going to be as tight as the real bounding sphere we had in 2.8.
		PxBounds3 box3d;
		obj.GetWorldAABB(box3d);

		PxVec3 boxCenter = box3d.getCenter(), boxExtents = box3d.getExtents();
#ifdef USE_QUADTREE
		WorldSphere.center.x = boxCenter.x;
		WorldSphere.center.y = boxCenter[mQuadtreeUp];
		WorldSphere.radius = boxExtents.maxElement();

		TestCube.minimum.x = box3d.minimum.x;
		TestCube.minimum.y = box3d.minimum[mQuadtreeUp];
		TestCube.maximum.x = box3d.maximum.x;
		TestCube.maximum.y = box3d.maximum[mQuadtreeUp];
#else
		WorldSphere.center = boxCenter;
		WorldSphere.radius = boxExtents.maxElement();

		TestCube = box3d;
#endif
		CurrentCube = &TestCube;
	}

	// Check radius
	// PT: ### TIWAK: negative radius??
#ifdef PX_WINDOWS
	if(IR(WorldSphere.radius)>IR(mWorldSize2))
#else
	if(WorldSphere.radius>mWorldSize2)
#endif
	{
		// The object is too big, even for first level!
		PX_ASSERT(!"LinearLooseOctree::Insert: object is too big for the octree!");
		obj.Clear();
		return false;
	}

	// Check center
	if(!mWorldBox.contains(WorldSphere.center))
	{
		// The object is out of the world !
		PX_ASSERT(!"LinearLooseOctree::Insert: object is out of the world!");
		obj.Clear();
		return false;
		// ### we could wrap the center around the world, maybe
	}


//PxU32 Time;
//StartProfile(Time);
	// Compute depth with the theoretical formula
	PxU32 Depth = ComputeOctreeDepth(mWorldSize, WorldSphere.radius);
//EndProfile(Time);
//Log("OctreeDepth1: %d %d\n", Depth, Time);

#ifdef OCTREE_TEST_DEPTH

StartProfile(Time);
	// Compute depth with an alternative faster version
	PxU32 Depth2 = ComputeOctreeDepth2(mWorldSize, Radius);
	ASSERT(Depth==Depth2);
EndProfile(Time);
Log("OctreeDepth2: %d %d\n", Depth2, Time);

StartProfile(Time);
	// Actually since we're limited to mMaxDepth, we can do better than that!
	PxU32 IRadius = IR(Radius);
	PxU32 Depth3=-1;
	Depth3 += (IRadius - IR(mWorldSize2))>>31;
	Depth3 += (IRadius - IR(mWorldSize4))>>31;
	Depth3 += (IRadius - IR(mWorldSize8))>>31;
	Depth3 += (IRadius - IR(mWorldSize16))>>31;
	Depth3 += (IRadius - IR(mWorldSize32))>>31;
	Depth3 += (IRadius - IR(mWorldSize64))>>31;
	Depth3 += (IRadius - IR(mWorldSize128))>>31;
	// Should only goes until MaxDepth since we're going to clip it anyway.
	// Uh, actually clipping is useless if we only can go up to maxdeph !
	// We can also "early exit" as soon as one test fail, but then we introduce a branch. Good ? Bad ?
EndProfile(Time);
Log("OctreeDepth3: %d %d\n", Depth3, Time);

StartProfile(Time);
	// Actually since we're limited to mMaxDepth, we can do better than that!
	PxU32 Depth4;
			if(Radius>mWorldSize2)		Depth4 = -1;
	else	if(Radius>mWorldSize4)		Depth4 = 0;
	else	if(Radius>mWorldSize8)		Depth4 = 1;
	else	if(Radius>mWorldSize16)		Depth4 = 2;
	else	if(Radius>mWorldSize32)		Depth4 = 3;
	else	if(Radius>mWorldSize64)		Depth4 = 4;
	else	if(Radius>mWorldSize128)	Depth4 = 5;
EndProfile(Time);
Log("OctreeDepth4: %d %d\n", Depth4, Time);

#endif

	//##keep track of highest depth to adjust octree size for each scene?

	// The theoretical depth has been computed regardless of the actual number
	// of allocated cells, so we have to clamp it.
	if(Depth>mMaxDepth)	Depth = mMaxDepth;


	// Compute offset corresponding to current depth
	PxU32 BaseOffset = ComputeOctreeOffset(Depth);


#ifdef OCTREE_CHECK
	if(0)
	{
		PxU32 nb=0;
		PxU32 Nb = ComputeNbOctreeCells(Depth);
		for(PxU32 nn=0;nn<Nb;nn++)
		{
			if(CurrentCube->IsInside(mCells[BaseOffset+nn].mBox))
			{
				nb++;
			}
		}
	}
#endif

	// Compute index

//StartProfile(Time);
	PxU32 Stride = ComputeOctreeStride(Depth);
	float Coeff = float(Stride) / mWorldSize;	// tested against original non-loose size

//	PxU32 i = (PxU32)((mWorldSize*0.5f + center.x)*Coeff);
//	PxU32 j = (PxU32)((mWorldSize*0.5f + center.y)*Coeff);
//	PxU32 k = (PxU32)((mWorldSize*0.5f + center.z)*Coeff);

#ifdef USE_QUADTREE
	const PxU32 i = (PxU32)((mWorldOffset.x + WorldSphere.center.x)*Coeff);
	const PxU32 j = (PxU32)((mWorldOffset.y + WorldSphere.center.y)*Coeff);
	PxU32 Index = ComputeMortonIndex(i, j, Depth);
#else
	const PxU32 i = (PxU32)((mWorldOffset.x + WorldSphere.center.x)*Coeff);
	const PxU32 j = (PxU32)((mWorldOffset.y + WorldSphere.center.y)*Coeff);
	const PxU32 k = (PxU32)((mWorldOffset.z + WorldSphere.center.z)*Coeff);
	PxU32 Index = ComputeMortonIndex(i, j, k, Depth);
#endif

	Index+=BaseOffset;

//	PxU32 Index = Offset + i + j*Stride + k*Stride*Stride;
//EndProfile(Time);
//Log("OctreeIndex: %d %d\n", Index, Time);


//StartProfile(Time);
	// Look for a possible better fit :
	// "Note that this procedure is not quite ideal: it does not actually find the tightest possible containing node for all cases (see Figure 6).
	// To get the last bit of tightness, first find the candidate node using the above formulas, and then check the child node nearest to the object
	// to see if the object fits inside it."
	for(PxU32 CubeIndex=1;CubeIndex<=NB_OCTREE_CHILDREN;CubeIndex++)
	{
		// Compute child index
		PxU32 ChildID = MAKE_CHILD_INDEX(Index, CubeIndex);
		if(ChildID>=mNbCells)	continue;

#ifdef USE_IMPLICIT_BOX
		// ### could compute all boxes at once here
		OctreeBox Box;
		ComputeBox(ChildID, Box);
		if(IsInside(*CurrentCube, Box))
#else
		if(IsInside(*CurrentCube, mCells[ChildID].mBox))
#endif
		{
//check that
//PxU32 ii = (Index - BaseOffset)>>((Level-1)*3);
			// found better
			Index = ChildID;
			break;
		}
	}
//EndProfile(Time);
//Log("OctreeBetterFit: %d %d\n", Index, Time);


#ifdef _DEBUG
#ifdef OCTREE_CHECK
	{
		PxU32 CurrentIndex = Index;
		while(CurrentIndex)
		{
			bool CurStatus = CurrentCube->IsInside(mCells[CurrentIndex].mBox);
			ASSERT(CurStatus);

			// Jump to parent cell
			MAKE_PARENT_INDEX(CurrentIndex);
		}
	}
#endif
#endif



//StartProfile(Time);
	// 2) Traverse the tree from current cell to root, increasing counters.
	// This requires between 1 and mMaxDepth iterations, so it's "fast". (however it's probably full of cache misses)

	PxU32 CurrentIndex = Index;	// ### could be done after 3 to save that var
	while(CurrentIndex)
	{
		// Increase counter
		mCells[CurrentIndex].mCount++;

		// Jump to parent cell
		MAKE_PARENT_INDEX(CurrentIndex);
	}
	// Increase root counter
	mCells[0].mCount++;

//EndProfile(Time);
//Log("OctreeCount: %d\n", Time);


//StartProfile(Time);
	// 3) Update lists
/*
	- Insertion:
		- find the right OctreeCell for current object
		- if the cell is empty:
			- update OctreeCell's ObjDesc pointer with current ObjDesc
			- setup ObjDesc's owner
			- setup ObjDesc's previous to NULL
			- setup ObjDesc's next to NULL
		- if the cell already has objects:
			- update OctreeCell's ObjDesc pointer with current ObjDesc (ie insert to the top of the list)
			- setup ObjDesc's owner
			- setup ObjDesc's previous to NULL (it's the top of the list)
			- setup ObjDesc's next to the previously first ObjDesc
			- setup previously first's ObjDesc's previous (supposed NULL) with current ObjDesc
*/
	CELL_CLASS_NAME* C = &mCells[Index];
	if(!C->mList)
	{
		// We landed in an empty OctreeCell, i.e. "obj" is the first object of the linked list.
		// 1) So let's init the list
		C->mList		= &obj;
		// 2) Then initialize the object itself
		obj.mOwner		= C;
		obj.mPrevious	= NULL;
		obj.mNext		= NULL;
	}
	else
	{
		// There's already people here.
		obj.mOwner		= C;
		obj.mPrevious	= NULL;
		obj.mNext		= C->mList;

		C->mList->mPrevious = &obj;

		C->mList		= &obj;
	}
//EndProfile(Time);
//Log("OctreeList: %d\n", Time);

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Removes an object from the linear octree.
 *	\param		obj		[in] object to remove
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool TREE_CLASS_NAME::RemoveObject(Prunable& obj)
{
	// Check the object is valid (i.e. checks it belongs to the octree)
	if(!obj.mOwner)	return false;

	// 1) Compute owner cell's index
	// ### store indices instead ? It would prevent access to the OctreeCell from an OctreeObject, so not that good
	PxU32 Index = (PxU32)(size_t(obj.mOwner) - size_t(mCells)) / sizeof(CELL_CLASS_NAME);
//	ASSERT(sizeof(OctreeCell)==32);	// The above division should be compiled as a shift

	// 2) Traverse the tree from current cell to root, decreasing counters.
	// This requires between 1 and mMaxDepth iterations, so it's "fast". (however it's probably full of cache misses)
	PxU32 CurrentIndex = Index;
	while(CurrentIndex)
	{
		// Decrease counter
		mCells[CurrentIndex].mCount--;

		// Jump to parent cell
		MAKE_PARENT_INDEX(CurrentIndex);
	}
	// Decrease root counter
	mCells[0].mCount--;

	// 3) Update lists
	//	- Deletion:
	//		- update previous & next ObjDesc if they exist
	//		- update the OctreeCell's first ObjDesc if this is us

	// Before: obj.mPrevious => obj => obj.mNext
	// After: obj.mPrevious => obj.mNext
	if(obj.mPrevious)	obj.mPrevious->mNext	= obj.mNext;
	if(obj.mNext)		obj.mNext->mPrevious	= obj.mPrevious;

	CELL_CLASS_NAME* Owner = (CELL_CLASS_NAME*)obj.mOwner;	// PT: Sigh. Introduced because we want octree/quadtree at the same time
	if(Owner->mList==&obj)	Owner->mList = obj.mPrevious ? obj.mPrevious : obj.mNext;

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Updates an object.
 *	\param		obj		[in] object to update
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool TREE_CLASS_NAME::UpdateObject(Prunable& obj)
{
#ifdef OLDIES
	// Check the object is valid (i.e. checks it belongs to the octree)
	if(!obj.mOwner)	return false;

		//	- Motion:
		//		- check the owner's box against our new box: (or simply check the center position if the size is constant (movable != dynamic))
		//			- if we're still inside, we're done
		//			- else delete & re-insert

	OctreeBox TestBox;
	OctreeSphere WorldSphere;

		// Build an AABB for current object
		obj.GetWorldSphere(WorldSphere);
	//	float Radius = LocalSphere->mRadius;
	//	Point p = LocalSphere->mCenter;
	//	if(WorldTransform)	p *= *WorldTransform;
	#ifdef USE_QUADTREE
		TestBox.SetCenterExtents(WorldSphere.mCenter, Point2D(WorldSphere.mRadius, WorldSphere.mRadius));
	#else
		TestBox.SetCenterExtents(WorldSphere.mCenter, PxVec3(WorldSphere.mRadius, WorldSphere.mRadius, WorldSphere.mRadius));
	#endif

		// Nothing to do if the owner's AABB still contains the object's AABB
		if(IsInside(TestBox, obj.mOwner->mBox))	return false;

		// Else remove & re-insert. The process is reasonably fast, and since it's only done once in a while (when
		// the object goes from one cell to another, rather then each time it moves) the overall cost is cheap.
		RemoveObject(obj);	// ~O(1)

	// Re-insert using the already-computed box (so we save a transform here)
	return InsertObject(obj, &TestBox);	// ~O(1)
#endif



//#ifdef REMOVED
	OctreeBox* TB = NULL;
	OctreeBox TestBox;
	OctreeSphere WorldSphere;

	// Check the object is valid (i.e. checks it belongs to the octree)
	if(obj.mOwner)
	{
		//	- Motion:
		//		- check the owner's box against our new box: (or simply check the center position if the size is constant (movable != dynamic))
		//			- if we're still inside, we're done
		//			- else delete & re-insert

		// Build an AABB for current object

#ifdef USE_QUADTREE
		PxBounds3 box3d;
		obj.GetWorldAABB(box3d);

		TestBox.minimum.x = box3d.minimum.x;
		TestBox.minimum.y = box3d.minimum[mQuadtreeUp];
		TestBox.maximum.x = box3d.maximum.x;
		TestBox.maximum.y = box3d.maximum[mQuadtreeUp];

#else
		obj.GetWorldAABB(TestBox);
#endif

		// Nothing to do if the owner's AABB still contains the object's AABB
		CELL_CLASS_NAME* Owner = (CELL_CLASS_NAME*)obj.mOwner;	// PT: Sigh. Introduced because we want octree/quadtree at the same time
#ifdef USE_IMPLICIT_BOX
		PxU32 ImplicitIndex = PxU32((Owner - mCells)/sizeof(CELL_CLASS_NAME));
		OctreeBox Box;
		ComputeBox(ImplicitIndex, Box);
		if(IsInside(TestBox, Box))	return false;
#else
		if(IsInside(TestBox, Owner->mBox))	return false;
#endif

		// Else remove & re-insert. The process is reasonably fast, and since it's only done once in a while (when
		// the object goes from one cell to another, rather then each time it moves) the overall cost is cheap.
		RemoveObject(obj);	// ~O(1)

		TB = &TestBox;
	}

	// Re-insert using the already-computed box (so we save a transform here)
	return InsertObject(obj, TB);	// ~O(1)
//#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Dumps the list of objects contained in a cell and in all children.
 *	\param		index	[in] index of current cell
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void _FullDump(PxU32 index, PxU32 max_index, const CELL_CLASS_NAME* cells, Ice::ContainerSizeT& array)
{
	// Check index
	if(index>=max_index)	return;

	// Check current cell. If cumulative count is NULL, there's no object at all in the cell and in any branches. So we're done.
	const CELL_CLASS_NAME& Current = cells[index];
	if(!Current.GetCount())	return;

	// Dump this node
	Current.Dump(array);

	// Recurse through children
	for(PxU32 CubeIndex=1;CubeIndex<=NB_OCTREE_CHILDREN;CubeIndex++)
	{
		// Compute child index & recurse
		_FullDump(MAKE_CHILD_INDEX(index, CubeIndex), max_index, cells, array);
	}
}

	struct PlaneQueryData
	{
	#ifdef USE_IMPLICIT_BOX
		TREE_CLASS_NAME*		mOctree;
	#endif
	#ifdef USE_QUADTREE
		PxU32					mQuadtreeUp;
	#endif
		PxU32					mMaxIndex;
		const CELL_CLASS_NAME*	mCells;
		Ice::ContainerSizeT*	mClip;
		Ice::ContainerSizeT*	mNoClip;
		const Gu::Plane*		mPlanes;
	};

static void _Cull(PxU32 index, PxU32 clip_mask, const PlaneQueryData& pqd)
{
	// Check index
	if(index>=pqd.mMaxIndex)	return;

	// Check current cell. If cumulative count is NULL, there's no object at all in the cell and in any branches. So we're done.
	const CELL_CLASS_NAME& Current = pqd.mCells[index];
	if(!Current.GetCount())	return;

	// Else cull current box
	PxU32 OutClipMask;

	// Test the box against the planes. If the box is completely culled, so are its children, hence we exit.
#ifdef USE_IMPLICIT_BOX
	// PT: optimization potential here: use center/extents
	OctreeBox NodeBox;
	pqd.mOctree->ComputeBox(index, NodeBox);
#else
	const OctreeBox& NodeBox = Current.GetBox();
#endif

#ifdef USE_QUADTREE
	const PxU32 y = pqd.mQuadtreeUp;
	const PxU32 z = 3-y;
	PxVec3 TmpMin, TmpMax;
	TmpMin.x	= NodeBox.minimum.x;
	TmpMin[y]	= NodeBox.minimum.y;
	TmpMin[z]	= -1000000.0f;
	TmpMax.x	= NodeBox.maximum.x;
	TmpMax[y]	= NodeBox.maximum.y;
	TmpMax[z]	= 1000000.0f;
	PxBounds3 Tmp;
	Tmp.minimum = TmpMin;
	Tmp.maximum = TmpMax;
	if(!Ice::PlanesAABBOverlap(Tmp, pqd.mPlanes, OutClipMask, clip_mask))	return;
#else
	if(!Ice::PlanesAABBOverlap(NodeBox, pqd.mPlanes, OutClipMask, clip_mask))	return;
#endif

	// If the box is completely included, so are its children. We don't need to do extra tests, we
	// can immediately output a list of visible children. Those ones won't need to be clipped.
	if(!OutClipMask)
	{
		// Dump everything
		_FullDump(index, pqd.mMaxIndex, pqd.mCells, *pqd.mNoClip);
		return;
	}

	// The box is partially visible => objects in that cell will need further culling and maybe clipping.
	Current.Dump(*pqd.mClip);

	// Recurse through children
	for(PxU32 CubeIndex=1;CubeIndex<=NB_OCTREE_CHILDREN;CubeIndex++)
	{
		// Compute child index & recurse
		_Cull(MAKE_CHILD_INDEX(index, CubeIndex), OutClipMask, pqd);
	}
}

void TREE_CLASS_NAME::TestAgainstPlanes(const Gu::Plane* planes, PxU32 nb_planes, Ice::ContainerSizeT& box_indices_clip, Ice::ContainerSizeT& box_indices_noclip)
{
	// Use a struct with constant data, to minimize used stack ram
	PlaneQueryData PQD;
#ifdef USE_IMPLICIT_BOX
	PQD.mOctree		= this;
#endif
#ifdef USE_QUADTREE
	PQD.mQuadtreeUp	= mQuadtreeUp;
#endif
	PQD.mMaxIndex	= GetNbCells();
	PQD.mCells		= GetCells();
	PQD.mClip		= &box_indices_clip;
	PQD.mNoClip		= &box_indices_noclip;
	PQD.mPlanes		= planes;

	_Cull(0, (1<<nb_planes)-1, PQD);
}



















#ifdef USE_QUADTREE

static	// VC7.1 bug
PX_FORCE_INLINE	float SquareDistance(const PxVec3& a, const Point2D& b, PxU32 up)
{
	return ((a.x - b.x)*(a.x - b.x) + (a[up] - b.y)*(a[up] - b.y));
}

static	// VC7.1 bug
PX_FORCE_INLINE Ps::IntBool SphereContainsBox(const Gu::Sphere& sphere, const AABB_2D& aabb, PxU32 up)
{
	// I assume if all 4 box vertices are inside the sphere, so does the whole box.
	// Sounds ok but maybe there's a better way?
	const float R2 = sphere.radius * sphere.radius;
	const Point2D& Max = aabb.maximum;
	const Point2D& Min = aabb.minimum;
	Point2D p;
	p.x=Max.x; p.y=Max.y;	if(SquareDistance(sphere.center, p, up)>=R2)	return Ps::IntFalse;
	p.x=Min.x;				if(SquareDistance(sphere.center, p, up)>=R2)	return Ps::IntFalse;
	p.x=Max.x; p.y=Min.y;	if(SquareDistance(sphere.center, p, up)>=R2)	return Ps::IntFalse;
	p.x=Min.x;				if(SquareDistance(sphere.center, p, up)>=R2)	return Ps::IntFalse;
	return Ps::IntTrue;
}

static bool SphereAABB(const PxVec3& center, float radius, const Point2D& minimum, const Point2D& maximum, PxU32 up)
{ 
	float d = 0.0f;

	//find the square of the distance
	//from the sphere to the box
	if(center.x<minimum.x)
	{
		float s = center.x - minimum.x;
		d += s*s;
	}
	else if(center.x>maximum.x)
	{
		float s = center.x - maximum.x;
		d += s*s;
	}

	if(center[up]<minimum.y)
	{
		float s = center[up] - minimum.y;
		d += s*s;
	}
	else if(center[up]>maximum.y)
	{
		float s = center[up] - maximum.y;
		d += s*s;
	}

	return d <= radius*radius;
}

	static	// VC7.1 bug
	PX_FORCE_INLINE bool SphereAABB(const PxVec3& center, float radius, const AABB_2D& aabb, PxU32 up)
	{
		return SphereAABB(center, radius, aabb.minimum, aabb.maximum, up);
	}

static	// VC7.1 bug
PX_FORCE_INLINE Ps::IntBool Intersect(const PxBounds3& a, const AABB_2D& b, PxU32 up)
{
	if(		b.maximum.x < a.minimum.x
		||	a.maximum.x < b.minimum.x
		||	b.maximum.y < a.minimum[up]
		||	a.maximum[up] < b.minimum.y)	return Ps::IntFalse;
	return Ps::IntTrue;
}

static	// VC7.1 bug
PX_FORCE_INLINE bool SegmentAABBCenterExtent(const Point2D& dir, const Point2D& diff, const Point2D& fdir, const Point2D& center, const float extent)
{
	float dx = diff.x - center.x;
	if(PxAbs(dx)>extent + fdir.x)	return false;

	float dy = diff.y - center.y;
	if(PxAbs(dy)>extent + fdir.y)	return false;

	float f = dir.x * dy - dir.y * dx;
	if(PxAbs(f)>extent*(fdir.x + fdir.y))	return false;
	return true;
}

	static	// VC7.1 bug
	PX_FORCE_INLINE bool SegmentAABB(const PxVec3& dir, const PxVec3& diff, const PxVec3& fdir, const PxVec3& minimum, const PxVec3& maximum)
	{
		PxVec3 BoxExtents, Diff;

		BoxExtents.x = (maximum.x - minimum.x);
		Diff.x = (diff.x - (maximum.x + minimum.x));
		if(PxAbs(Diff.x)>BoxExtents.x + fdir.x)	return false;

		BoxExtents.y = (maximum.y - minimum.y);
		Diff.y = (diff.y - (maximum.y + minimum.y));
		if(PxAbs(Diff.y)>BoxExtents.y + fdir.y)	return false;

		BoxExtents.z = (maximum.z - minimum.z);
		Diff.z = (diff.z - (maximum.z + minimum.z));
		if(PxAbs(Diff.z)>BoxExtents.z + fdir.z)	return false;

		float f;
		f = dir.y * Diff.z - dir.z * Diff.y;
		if(PxAbs(f)>BoxExtents.y*fdir[2] + BoxExtents.z*fdir[1])	return false;

		f = dir.z * Diff.x - dir.x * Diff.z;
		if(PxAbs(f)>BoxExtents.x*fdir[2] + BoxExtents.z*fdir[0])	return false;

		f = dir.x * Diff.y - dir.y * Diff.x;
		if(PxAbs(f)>BoxExtents.x*fdir[1] + BoxExtents.y*fdir[0])	return false;

		return true;
	}

	static	// VC7.1 bug
	PX_FORCE_INLINE bool SegmentAABB(const PxVec3& dir, const PxVec3& diff, const PxVec3& fdir, const PxBounds3& aabb)
	{
		return ::SegmentAABB(dir, diff, fdir, aabb.minimum, aabb.maximum);
	}

static bool RayAABBCenterExtents(const PxVec3& orig, const PxVec3& dir, const Point2D& center, const float extent, PxU32 up)
{
	Point2D Diff;
	Diff.x = orig.x - center.x;
	if(PxAbs(Diff.x)>extent && Diff.x*dir.x>=0.0f)		return false;

	////////////

	Diff.y = orig[up] - center.y;
	if(PxAbs(Diff.y)>extent && Diff.y*dir[up]>=0.0f)	return false;

	float fAWdU[2];
	fAWdU[0] = PxAbs(dir.x);
	fAWdU[1] = PxAbs(dir[up]);

	float f = dir[up] * Diff.x - dir.x * Diff.y;	if(PxAbs(f)>extent*(fAWdU[1] + fAWdU[0]))	return false;
	return true;
}

#endif





///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PRUNER_NAME::PRUNER_NAME() :
	mOctree				(NULL),
	mUpAxis				(0),
	mSubdivisionLevel	(0)
{
	mExpectedWorldBox.setEmpty();

	mCullFunc			= (CullFunc)			&PRUNER_NAME::Cull;
	mStabFunc			= (StabFunc)			&PRUNER_NAME::Stab;
	mOverlapSphereFunc	= (OverlapSphereFunc)	&PRUNER_NAME::OverlapSphere;
	mOverlapAABBFunc	= (OverlapAABBFunc)		&PRUNER_NAME::OverlapAABB;
	mOverlapOBBFunc		= (OverlapOBBFunc)		&PRUNER_NAME::OverlapOBB;
	mOverlapCapsuleFunc	= (OverlapCapsuleFunc)	&PRUNER_NAME::OverlapCapsule;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PRUNER_NAME::~PRUNER_NAME()
{
	PX_DELETE_AND_RESET(mOctree);
}

bool PRUNER_NAME::Setup(const PRUNERCREATE& create)
{
	mExpectedWorldBox	= create.mExpectedWorldBox;
	mSubdivisionLevel	= create.mSubdivisionLevel;

	// Don't bite more than we can chew 
	if(mSubdivisionLevel>8)	mSubdivisionLevel = 8;

	// We want "2" for Y and "1" for Z
			if(create.mUpAxis==1)	mUpAxis = 2;
	else	if(create.mUpAxis==2)	mUpAxis = 1;
	else	mUpAxis = 2; // Shouldn't happen but better safe than sorry

	return Pruner::Setup(create);
}

bool PRUNER_NAME::BuildLooseOctree()
{
	PX_DELETE_AND_RESET(mOctree);

	// Don't bother building an octree if there isn't a single dynamic object
	PxU32 NbObjects = GetNbActiveObjects();
	if(!NbObjects)	return true;

	Prunable** Objects = (Prunable**)GetObjects();

	mOctree = PX_NEW(TREE_CLASS_NAME);

	//
	PxBounds3 GlobalBox;

	// ### take largest ?
	if(!mExpectedWorldBox.isEmpty())
	{
		GlobalBox = mExpectedWorldBox;
	}
	else
	{
		GlobalBox.setEmpty();
		for(PxU32 i=0;i<NbObjects;i++)
		{
	//		mPools[PRT_DYNAMIC].mObjects[i]->ComputeWorldAABB(mPools[PRT_DYNAMIC].mWorldBoxes[mPools[PRT_DYNAMIC].mObjects[i]->mHandle]);
			const PxBounds3* Box = GetWorldAABB(*Objects[i]);
	//		GlobalBox.Add(mPools[PRT_DYNAMIC].mWorldBoxes[mPools[PRT_DYNAMIC].mObjects[i]->mHandle]);
			GlobalBox.include(*Box);
		}
	}

	// Typically: level 7 for quadtree, level 5 for octree
#ifdef USE_QUADTREE
	AABB_2D GlobalBox2D;
	GlobalBox2D.minimum.x = GlobalBox.minimum.x;
	GlobalBox2D.minimum.y = GlobalBox.minimum[mUpAxis];
	GlobalBox2D.maximum.x = GlobalBox.maximum.x;
	GlobalBox2D.maximum.y = GlobalBox.maximum[mUpAxis];
	mOctree->Init(mSubdivisionLevel, GlobalBox2D, mUpAxis);
#else
	mOctree->Init(mSubdivisionLevel, GlobalBox);
#endif

	for(PxU32 i=0;i<NbObjects;i++)
	{
		Objects[i]->Clear();
//		mOctree->InsertObject(*mPools[PRT_DYNAMIC].mObjects[i]);
		mOctree->InsertObject(*Objects[i]);
	}
	return true;
}

void PRUNER_NAME::eagerUpdatePruningTrees()
{
	if(!mOctree) BuildLooseOctree();

	// TEST
/*	PxU32 Time;
	StartProfile(Time);
//	mExpectedWorldBox.SetEmpty();
	mExpectedWorldBox.SetMinMax(Point(-100.0f, -50.0f, -100.0f), Point(100.0f, 150.0f, 100.0f));
	BuildLooseOctree();
	EndProfile(Time);
	printf("Rebuild: %d\n", Time);*/
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Adds an object to the pruner.
 *	\param		object	[in] the object to register
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool PRUNER_NAME::AddObject(Prunable& object)
{
	// If a tree has already been built for some objects, update it
	if(mOctree/* && object.GetPruningSection()!=PRP_LOW*/)
		mOctree->InsertObject(object);

	return Pruner::AddObject(object);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Removes an object from the pruner.
 *	\param		object	[in] the object to remove
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool PRUNER_NAME::RemoveObject(Prunable& object)
{
	if(mOctree)
	{
		mOctree->RemoveObject(object);
		// Release the octree if it was the last object
		if(!mOctree->GetNbContainedObjects())
		{
			PX_DELETE_AND_RESET(mOctree);
		}
	}
	return Pruner::RemoveObject(object);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Updates an object, i.e. updates the pruner's spatial database.
 *	\param		object	[in] the object to update
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool PRUNER_NAME::UpdateObject(Prunable& object)
{
	// ### not sure the order is important
	Pruner::UpdateObject(object);

	// PT: this is now done here to make sure the box is cached right away, i.e.
	// we don't call the box callback from a raycast call... This is fine since
	// we only call this function once, from SwapBuffers, so we won't compile
	// the box several times in vain (hopefully)
	// ### to do lazy => mais pb avec GetWorldBoxes()
	// Compute and cache the new AABB
	GetWorldAABB(object);

	// PT: changed this at Tiwak
//	if(mOctree)
	if(mOctree/* && object.GetPruningSection()!=PRP_LOW*/)
		mOctree->UpdateObject(object);
	return true;
}

bool PRUNER_NAME::Cull(PruningTemps& temps, CulledObjects& objects, const Gu::Plane* planes, PxU32 nb_planes, PxU32 culling_flags)
{
	// Indices in mWorldBoxes
	Ice::ContainerSizeT& VisibleBoxIndicesClip		= temps.mVisibleBoxIndicesClip;
	Ice::ContainerSizeT& VisibleBoxIndicesNoClip	= temps.mVisibleBoxIndicesNoClip;

	// Lazy build
	if(!mOctree)	BuildLooseOctree();
	if(!mOctree)	return true;

	// ## TODO
VisibleBoxIndicesClip.Reset();
VisibleBoxIndicesNoClip.Reset();

	mOctree->TestAgainstPlanes(planes, nb_planes, VisibleBoxIndicesClip, VisibleBoxIndicesNoClip);

	PxU32 NbVis = VisibleBoxIndicesClip.GetNbEntries();
	for(PxU32 i=0;i<NbVis;i++)
	{
		Prunable* PRN = (Prunable*)VisibleBoxIndicesClip.GetEntry(i);

PxU32 OutClipMask;
if(!Ice::PlanesAABBOverlap(*GetWorldAABB(*PRN), planes, OutClipMask, (1<<nb_planes)-1))	continue;
objects.AddPrunable(PRN);
	}
	NbVis = VisibleBoxIndicesNoClip.GetNbEntries();
	for(PxU32 i=0;i<NbVis;i++)
	{
		Prunable* PRN = (Prunable*)VisibleBoxIndicesNoClip.GetEntry(i);
/*
PxU32 OutClipMask;
if(!PlanesAABB(*mPools[PRT_DYNAMIC].GetWorldAABB(*PRN), planes, OutClipMask, (1<<nbplanes)-1))	continue;
if(setupclip)	list.AddPrunable(PRN, OutClipMask!=0);
else			list.AddPrunable(PRN);
*/
		objects.AddPrunable(PRN);
	}

	return true;
}



	struct RayQueryData
	{
#ifdef USE_IMPLICIT_BOX
		TREE_CLASS_NAME*		mOctree;
#endif
#ifdef USE_QUADTREE
		PxU32					mQuadtreeUp;
#endif
		PxU32					mMaxIndex;
		const CELL_CLASS_NAME*	mCells;
		PxVec3					mOrig;
		PxVec3					mDir;

		StabCallback			mCallback;
		void*					mUserData;
		PRUNER_NAME*			mPruner;
		bool					mExit;
	};

// ### to customize
static void _TestAgainstRay(PxU32 index, RayQueryData& rqd)
{
	if(rqd.mExit)	return;

	// Check index
	if(index>=rqd.mMaxIndex)	return;

	{
		// Check current cell. If cumulative count is NULL, there's no object at all in the cell and in any branches. So we're done.
		const CELL_CLASS_NAME& Current = rqd.mCells[index];
		if(!Current.GetCount())	return;

		// Test the box against the segment
#ifdef USE_IMPLICIT_BOX
	#ifdef USE_QUADTREE
		OctreePoint Center;
		float Extent;
		rqd.mOctree->ComputeBox(index, Center, Extent);
		if(!RayAABBCenterExtents(rqd.mOrig, rqd.mDir, Center, Extent, rqd.mQuadtreeUp))	return;
	#else
		#pragma message("Octree: missing optim here - use center/extent version")
		OctreeBox Box;
		rqd.mOctree->ComputeBox(index, Box);
		if(!Ice::RayAABB(rqd.mOrig, rqd.mDir, Box))	return;
	#endif
#else
		if(!Ice::RayAABB(rqd.mOrig, rqd.mDir, Current.GetBox()))	return;
#endif

		// The loose box has been hit, so we just return a list of *possibly* hit objects.
		// There's no possible "full dump" here.
		const Prunable* Obj = Current.GetObjects();
		while(Obj)
		{
			const Prunable* NextObj = Obj->GetNextObject();	// PT: trying to avoid LHS
			if(NextObj)	Ps::prefetch((void *)NextObj);

			if(Ice::RayAABB(rqd.mOrig, rqd.mDir, *rqd.mPruner->GetWorldAABB(*Obj)))
			{
/*				if(rqd.mObjects)
				{
					rqd.mObjects->AddPrunable(Obj);
				}
				else*/
				{
					float MaxDist = PX_MAX_F32;
					PxU32 Status = (rqd.mCallback)(Obj, MaxDist, rqd.mUserData);
					if(Status & Ice::STAB_STOP)
					{
						rqd.mExit = true;
						return;
					}
				}
			}

			Obj = NextObj;
		}
	}


	// Recurse through children
	for(PxU32 CubeIndex=1;CubeIndex<=NB_OCTREE_CHILDREN;CubeIndex++)
	{
		// Compute child index & recurse
		_TestAgainstRay(MAKE_CHILD_INDEX(index, CubeIndex), rqd);
	}
}

/*
// Non-recursive ### missing first node tests
static void TestAgainstRay(const RayQueryData& rqd)
{
	static Container tmp;
	tmp.Reset();
	tmp.Add(PxU32(0));

	while(tmp.GetNbEntries())
	{
		PxU32 _index = tmp.GetEntries()[tmp.GetNbEntries()-1];
		tmp.DeleteLastEntry();

		for(PxU32 CubeIndex=1;CubeIndex<=NB_OCTREE_CHILDREN;CubeIndex++)
		{
			PxU32 index = MAKE_CHILD_INDEX(_index, CubeIndex);

			// Check index
			if(index>=rqd.mMaxIndex)	continue;

			// Check current cell. If cumulative count is NULL, there's no object at all in the cell and in any branches. So we're done.
			const CELL_CLASS_NAME& Current = rqd.mCells[index];
			if(!Current.GetCount())	continue;

			// Test the box against the segment
	#ifdef USE_IMPLICIT_BOX
			OctreePoint Center;
			float Extent;
			rqd.mOctree->ComputeBox(index, Center, Extent);
			if(!RayAABBCenterExtents(rqd.mRay, Center, Extent))	continue;
	#else
			if(!RayAABB(rqd.mRay, Current.GetBox()))	continue;
	#endif
			tmp.Add(index);

			// The loose box has been hit, so we just return a list of *possibly* hit objects.
			// There's no possible "full dump" here.
			const Prunable* Obj = Current.GetObjects();
			while(Obj)
			{
				const Prunable* NextObj = Obj->GetNextObject();	// PT: trying to avoid LHS
				if(NextObj)	_prefetch(NextObj);

				if(RayAABB(rqd.mRay, *rqd.mPruner->GetWorldAABB(*Obj)))
				{
					if(rqd.mObjects)
					{
						rqd.mObjects->AddPrunable(Obj);
					}
					else
					{
						bool Status = (rqd.mCallback)(Obj, NULL, NULL, rqd.mUserData);
						if(!Status)	return;
					}
				}
	//			if(first_contact)	return true;

				Obj = NextObj;
			}
		}
	}
}
*/





#ifdef USE_QUADTREE
	// ### Insane BS because VC7.1 messes up otherwise
	struct QuadtreeSegmentQueryData
#else
	struct OctreeSegmentQueryData
#endif
	{
		Ps::IntBool				mExit;
#ifdef USE_IMPLICIT_BOX
		TREE_CLASS_NAME*		mOctree;
#endif
#ifdef USE_QUADTREE
		PxU32					mQuadtreeUp;
#endif
		PxU32					mMaxIndex;
		const CELL_CLASS_NAME*	mCells;
		PxU32*					mSorted;
		PxVec3					mRayOrig;
		PxVec3					mRayDir;
		float					mMaxDist;
#ifdef USE_IMPLICIT_BOX
		OctreePoint				mDir0;
		OctreePoint				mDiff;
		OctreePoint				mFDir;
#endif
#ifdef USE_IMPLICIT_BOX
	#ifdef USE_QUADTREE
		PxVec3					m_Dir;
		PxVec3					m_Diff;
		PxVec3					m_FDir;
	#else
		Segment					mSegment;
	#endif
#else
		Gu::Segment				mSegment;
#endif
		PRUNER_NAME*			mPruner;
		StabCallback			mCallback;
		void*					mUserData;

		PX_FORCE_INLINE	void	SetupSegment()
		{
			PxVec3 Delta = mRayDir * mMaxDist;

#ifdef USE_IMPLICIT_BOX
	#ifdef USE_QUADTREE
			Gu::Segment mSegment;
			mSegment.p0 = mRayOrig;
			mSegment.p1 = mRayOrig + Delta;
	#else
			mSegment.p0 = mRayOrig;
			mSegment.p1 = mRayOrig + Delta;
	#endif
#else
			mSegment.p0 = mRayOrig;
			mSegment.p1 = mRayOrig + Delta;
#endif

#ifdef USE_IMPLICIT_BOX
			mDir0.x = Delta.x*0.5f;
			mDir0.y = Delta[mQuadtreeUp]*0.5f;

			mDiff.x = (mSegment.p1.x + mSegment.p0.x)*0.5f;
			mDiff.y = (mSegment.p1[mQuadtreeUp] + mSegment.p0[mQuadtreeUp])*0.5f;

			mFDir.x = PxAbs(mDir0.x);
			mFDir.y = PxAbs(mDir0.y);
#endif
#ifdef USE_IMPLICIT_BOX
	#ifdef USE_QUADTREE
			m_Dir = Delta;
			m_Diff = mSegment.p1 + mSegment.p0;
			m_FDir.x = PxAbs(m_Dir.x);
			m_FDir.y = PxAbs(m_Dir.y);
			m_FDir.z = PxAbs(m_Dir.z);
	#endif
#endif
		}
	};

// ### Insane BS because VC7.1 messes up otherwise
#ifdef USE_QUADTREE
	typedef QuadtreeSegmentQueryData	SegmentQueryData;
#else
	typedef OctreeSegmentQueryData	SegmentQueryData;
#endif

static int count;
static int maxcount=0;

// ### to customize
#ifndef USE_IMPLICIT_BOX
#ifndef USE_QUADTREE
static void _TestAgainstSegment(PxU32 index, SegmentQueryData& sqd)
{
	count++;
	if(sqd.mExit)	return;

	// Check index
	if(index>=sqd.mMaxIndex)	return;

	{
		// Check current cell. If cumulative count is NULL, there's no object at all in the cell and in any branches. So we're done.
		const CELL_CLASS_NAME& Current = sqd.mCells[index];
		if(!Current.GetCount())	return;

		// Test the box against the segment
#ifdef USE_IMPLICIT_BOX
	#ifdef USE_QUADTREE
		OctreePoint Center;
		float Extent;
		sqd.mOctree->ComputeBox(index, Center, Extent);
//		if(!SegmentAABBCenterExtent(sqd.mSegment, Center, Extent))	return;
		if(!SegmentAABBCenterExtent(sqd.mDir, sqd.mDiff, sqd.mFDir, Center, Extent))	return;
	#else
		#pragma message("Octree: missing optim here - use center/extent version")
		OctreeBox Box;
		sqd.mOctree->ComputeBox(index, Box);
		if(!Ice::SegmentAABB(sqd.mSegment, Box))	return;
	#endif
#else
		if(!Ice::SegmentAABB(sqd.mSegment, Current.GetBox()))	return;
#endif

		// The loose box has been hit, so we just return a list of *possibly* hit objects.
		// There's no possible "full dump" here.
		const Prunable* Obj = Current.GetObjects();
		while(Obj)
		{
			const Prunable* NextObj = Obj->GetNextObject();	// PT: trying to avoid LHS
			if(NextObj)	Ps::prefetch((void *)NextObj);

			if(Ice::SegmentAABB(sqd.mSegment, *sqd.mPruner->GetWorldAABB(*Obj)))
			{
/*				if(sqd.mObjects)
				{
					sqd.mObjects->AddPrunable(Obj);
				}
				else*/
				{
//					float MaxDist = PX_MAX_F32;
//					PxU32 Status = (sqd.mCallback)(Obj, MaxDist, sqd.mUserData);
					PxU32 Status = (sqd.mCallback)(Obj, sqd.mMaxDist, sqd.mUserData);
					if(Status & Ice::STAB_STOP)
					{
						sqd.mExit = true;
						return;
					}
					if(Status & Ice::STAB_UPDATE_MAX_DIST)
					{
						sqd.SetupSegment();
					}
				}
			}

			Obj = NextObj;
		}
	}

	// Recurse through children
	for(PxU32 CubeIndex=1;CubeIndex<=NB_OCTREE_CHILDREN;CubeIndex++)
	{
		// Compute child index & recurse
		_TestAgainstSegment(MAKE_CHILD_INDEX(index, CubeIndex), sqd);
	}
}
#endif
#endif

#ifdef USE_IMPLICIT_BOX
#ifdef USE_QUADTREE
static void _TestAgainstSegment2(PxU32 index, const OctreePoint& center, float extent, SegmentQueryData& sqd)
{
//	count++;
	if(sqd.mExit)	return;

	{
		// Test the box against the segment
		if(!SegmentAABBCenterExtent(sqd.mDir0, sqd.mDiff, sqd.mFDir, center, extent))	return;

		// The loose box has been hit, so we just return a list of *possibly* hit objects.
		// There's no possible "full dump" here.
		const CELL_CLASS_NAME& Current = sqd.mCells[index];
		const Prunable* Obj = Current.GetObjects();
		while(Obj)
		{
			const Prunable* NextObj = Obj->GetNextObject();	// PT: trying to avoid LHS
			if(NextObj)	Ps::prefetch((void *)NextObj);

			// PT: SegmentAABB can still be optimized here
			if(SegmentAABB(sqd.m_Dir, sqd.m_Diff, sqd.m_FDir, *sqd.mPruner->GetWorldAABB(*Obj)))
//			if(SegmentAABB(sqd.mSegment, *sqd.mPruner->GetWorldAABB(*Obj)))
			{
/*				if(sqd.mObjects)
				{
					sqd.mObjects->AddPrunable(Obj);
				}
				else*/
				{
					PxU32 Status = (sqd.mCallback)(Obj, sqd.mMaxDist, sqd.mUserData);
					if(Status & Ice::STAB_STOP)
					{
						sqd.mExit = true;
						return;
					}
					if(Status & Ice::STAB_UPDATE_MAX_DIST)
					{
						sqd.SetupSegment();
					}
				}
			}

			Obj = NextObj;
		}
	}

	// Compute children boxes
	OctreePoint Centers[NB_OCTREE_CHILDREN];
	float Extent;
	PxU32 Indices[NB_OCTREE_CHILDREN];
	PxU32 Nb = sqd.mOctree->ComputeChildrenBoxes(index, Centers, Extent, Indices);
	if(Nb)
	{
/*
#define SORT_CHILDREN	// Not needed for "any hit"
// TODO: precompute sorts?
#ifdef SORT_CHILDREN
		PxU32 Sorted[NB_OCTREE_CHILDREN];
for(PxU32 i=0;i<NB_OCTREE_CHILDREN;i++)	Sorted[i] = 42;
//		{
			float dp[NB_OCTREE_CHILDREN];
	#ifdef USE_QUADTREE
			const float dx = sqd.mRay.mDir.x;
			const float dy = sqd.mRay.mDir[sqd.mQuadtreeUp];
	#endif
			for(PxU32 i=0;i<Nb;i++)
			{
				Sorted[i] = i;
	#ifdef USE_QUADTREE
				dp[i] = dx*Centers[i].x + dy*Centers[i].y;
	#else
				dp[i] = sqd.mRay.mDir | Centers[i];
	#endif
			}
			for(PxU32 j=0;j<Nb;j++)
			{
				bool Break = true;
				for(PxU32 i=0;i<Nb-1;i++)
				{
					if(dp[i]>dp[i+1])
					{
						TSwap(dp[i], dp[i+1]);
						TSwap(Sorted[i], Sorted[i+1]);
						Break = false;
					}
				}
				if(Break)	break;
			}
//		}
#endif*/

		// Recurse through children
/*		for(PxU32 ii=0;ii<Nb;ii++)
		{
#ifdef SORT_CHILDREN
			PxU32 i = Sorted[ii];
#else
			PxU32 i = ii;
#endif
			_TestAgainstSegment2(Indices[i], Centers[i], Extent, sqd);
		}*/
		for(PxU32 ii=0;ii<NB_OCTREE_CHILDREN;ii++)
		{
			PxU32 i = sqd.mSorted[ii];
			if(Indices[i]!=PX_INVALID_U32)
				_TestAgainstSegment2(Indices[i], Centers[i], Extent, sqd);
		}
	}
}
#endif
#endif


PxU32 PRUNER_NAME::Stab(StabCallback callback, void* user_data, const PxVec3& orig, const PxVec3& dir, float& max_dist)
{
	if(!mOctree)	BuildLooseOctree();
	if(!mOctree)	return Ice::STAB_CONTINUE;

	if(max_dist==PX_MAX_F32)
	{
		RayQueryData RQD;
#ifdef USE_IMPLICIT_BOX
		RQD.mOctree			= mOctree;
#endif
#ifdef USE_QUADTREE
		RQD.mQuadtreeUp		= mOctree->GetQuadtreeUp();
#endif
		RQD.mMaxIndex		= mOctree->GetNbCells();
		RQD.mCells			= mOctree->GetCells();
		RQD.mOrig			= orig;
		RQD.mDir			= dir;
		RQD.mPruner			= this;
//		RQD.mObjects		= NULL;
		RQD.mCallback		= callback;
		RQD.mUserData		= user_data;
		RQD.mExit			= false;
		_TestAgainstRay(0, RQD);
	}
	else
	{
		SegmentQueryData SQD;
#ifdef USE_IMPLICIT_BOX
		SQD.mOctree			= mOctree;
#endif
#ifdef USE_QUADTREE
		SQD.mQuadtreeUp		= mOctree->GetQuadtreeUp();
#endif
		SQD.mMaxIndex		= mOctree->GetNbCells();
		SQD.mCells			= mOctree->GetCells();
		SQD.mRayOrig		= orig;
		SQD.mRayDir			= dir;
		SQD.mMaxDist		= max_dist;
		SQD.SetupSegment();
		SQD.mPruner			= this;
//		SQD.mObjects		= NULL;
		SQD.mCallback		= callback;
		SQD.mUserData		= user_data;
		SQD.mExit			= false;
		count=0;
#ifdef USE_IMPLICIT_BOX
	#ifdef USE_QUADTREE
		const CELL_CLASS_NAME& Current = SQD.mCells[0];
		if(!Current.GetCount())	return true;

		OctreePoint Center;
		float Extent;
		SQD.mOctree->ComputeBox(0, Center, Extent);

		PxU32 Sorted[NB_OCTREE_CHILDREN];
		mOctree->ComputeOrder(dir, Sorted);
		SQD.mSorted = Sorted;

		_TestAgainstSegment2(0, Center, Extent, SQD);
	#else
		#pragma message("Octree: missing optim here - implement _TestAgainstSegment2")
		_TestAgainstSegment(0, SQD);
	#endif
#else
		#pragma message("Octree: missing optim here - implement _TestAgainstSegment2")
		_TestAgainstSegment(0, SQD);
#endif
		if(count>maxcount)	maxcount=count;
	}
	return Ice::STAB_CONTINUE;
}

static void _FullDumpCB(PxU32 index, PxU32 max_index, const CELL_CLASS_NAME* cells, Sq::ReportPrunablesCallback callback, void* userData)
{
	// Check index
	if(index>=max_index)	return;

	// Check current cell. If cumulative count is NULL, there's no object at all in the cell and in any branches. So we're done.
	const CELL_CLASS_NAME& Current = cells[index];
	if(!Current.GetCount())	return;

	// Dump this node
	Prunable* Obj = Current.GetObjects();
	while(Obj)
	{
		Prunable* NextObj = Obj->GetNextObject();	// PT: trying to avoid LHS
		if(NextObj)	Ps::prefetch((void *)NextObj);

		(callback)(&Obj, 1, userData);
//		if(first_contact)	return true;

		Obj = NextObj;
	}

	// Recurse through children
	for(PxU32 CubeIndex=1;CubeIndex<=NB_OCTREE_CHILDREN;CubeIndex++)
	{
		// Compute child index & recurse
		_FullDumpCB(MAKE_CHILD_INDEX(index, CubeIndex), max_index, cells, callback, userData);
	}
}

	struct BoxQueryData
	{
#ifdef USE_IMPLICIT_BOX
		TREE_CLASS_NAME*		mOctree;
#endif
#ifdef USE_QUADTREE
		PxU32					mQuadtreeUp;
#endif
		PxU32					mMaxIndex;
		const CELL_CLASS_NAME*	mCells;
		ReportPrunablesCallback	mCallback;
		void*					mUserData;
		PxBounds3				mBox;
		PRUNER_NAME*			mPruner;
	};

#ifdef USE_QUADTREE
static	// VC7.1 bug
PX_FORCE_INLINE	bool Box2DIsInsideBox3D(const AABB_2D& box2d, const PxBounds3& box3d, PxU32 up)
{
	if(box3d.minimum.x		> box2d.minimum.x)	return false;
	if(box3d.minimum[up]	> box2d.minimum.y)	return false;
	if(box3d.maximum.x		< box2d.maximum.x)	return false;
	if(box3d.maximum[up]	< box2d.maximum.y)	return false;
	return true;
}
#endif

static void _TestAgainstAABB(PxU32 index, const BoxQueryData& bqd)
{
	// Check index
	if(index>=bqd.mMaxIndex)	return;

	// Check current cell. If cumulative count is NULL, there's no object at all in the cell and in any branches. So we're done.
	const CELL_CLASS_NAME& Current = bqd.mCells[index];
	if(!Current.GetCount())	return;

	// Test the box against the box
#ifdef USE_IMPLICIT_BOX
	// PT: optimization potential here: use center/extents
	OctreeBox NodeBox;
	bqd.mOctree->ComputeBox(index, NodeBox);
#else
	const OctreeBox& NodeBox = Current.GetBox();
#endif

#ifdef USE_QUADTREE
	if(!Intersect(bqd.mBox, NodeBox, bqd.mQuadtreeUp))	return;
#else
	if(!bqd.mBox.intersects(NodeBox))	return;
#endif

#ifdef USE_IMPLICIT_BOX
	#ifdef USE_QUADTREE
	if(Box2DIsInsideBox3D(NodeBox, bqd.mBox, bqd.mQuadtreeUp))
	#else
	if(NodeBox.IsInside(bqd.mBox))
	#endif
#else
	if(Current.GetBox().isInside(bqd.mBox))
#endif
	{
		// Full dump & no further test
//		_FullDump(index, bqd.mMaxIndex, bqd.mCells, *bqd.mObjects);
		_FullDumpCB(index, bqd.mMaxIndex, bqd.mCells, bqd.mCallback, bqd.mUserData);
		return;
	}

	Prunable* Obj = Current.GetObjects();
	while(Obj)
	{
		Prunable* NextObj = Obj->GetNextObject();	// PT: trying to avoid LHS
		if(NextObj)	Ps::prefetch((void *)NextObj);

		if(bqd.mBox.intersects(*bqd.mPruner->GetWorldAABB(*Obj)))
			(bqd.mCallback)(&Obj, 1, bqd.mUserData);
//		if(first_contact)	return true;

		Obj = NextObj;
	}

	// Recurse through children
	for(PxU32 CubeIndex=1;CubeIndex<=NB_OCTREE_CHILDREN;CubeIndex++)
	{
		// Compute child index & recurse
		_TestAgainstAABB(MAKE_CHILD_INDEX(index, CubeIndex), bqd);
	}
}

bool PRUNER_NAME::OverlapAABB(ReportPrunablesCallback cb, void* userData, const PxBounds3& box, bool first_contact)
{
	if(!mOctree)	BuildLooseOctree();
	if(!mOctree)	return true;

	BoxQueryData BQD;
#ifdef USE_IMPLICIT_BOX
	BQD.mOctree			= mOctree;
#endif
#ifdef USE_QUADTREE
	BQD.mQuadtreeUp		= mOctree->GetQuadtreeUp();
#endif
	BQD.mMaxIndex		= mOctree->GetNbCells();
	BQD.mCells			= mOctree->GetCells();
	BQD.mBox			= box;

	BQD.mCallback		= cb;
	BQD.mUserData		= userData;
	BQD.mPruner			= this;

	_TestAgainstAABB(0, BQD);

	return true;
}

	struct OBBQueryData
	{
#ifdef USE_IMPLICIT_BOX
		TREE_CLASS_NAME*		mOctree;
#endif
#ifdef USE_QUADTREE
		PxU32					mQuadtreeUp;
#endif
		PxU32					mMaxIndex;
		const CELL_CLASS_NAME*	mCells;
		ReportPrunablesCallback	mCallback;
		void*					mUserData;
		Gu::Box					mBox;
		PRUNER_NAME*			mPruner;
	};

static void _TestAgainstOBB(PxU32 index, const OBBQueryData& bqd)
{
	// Check index
	if(index>=bqd.mMaxIndex)	return;

	// Check current cell. If cumulative count is NULL, there's no object at all in the cell and in any branches. So we're done.
	const CELL_CLASS_NAME& Current = bqd.mCells[index];
	if(!Current.GetCount())	return;

	// Test the box against the box
#ifdef USE_IMPLICIT_BOX
	// PT: optimization potential here: use center/extents
	OctreeBox NodeBox;
	bqd.mOctree->ComputeBox(index, NodeBox);
#else
	const OctreeBox& NodeBox = Current.GetBox();
#endif

#ifdef USE_QUADTREE
	const PxU32 y = bqd.mQuadtreeUp;
	const PxU32 z = 3-y;
	PxVec3 TmpMin, TmpMax;
	TmpMin.x	= NodeBox.minimum.x;
	TmpMin[y]	= NodeBox.minimum.y;
	TmpMin[z]	= -1000000.0f;
	TmpMax.x	= NodeBox.maximum.x;
	TmpMax[y]	= NodeBox.maximum.y;
	TmpMax[z]	= 1000000.0f;
	PxBounds3 Tmp;
	Tmp.minimum = TmpMin;
	Tmp.maximum = TmpMax;
	if(!Gu::intersectOBBAABB(bqd.mBox, Tmp))	return;
#else
	if(!Gu::intersectOBBAABB(bqd.mBox, NodeBox))	return;
#endif

	// ### not implemented yet
/*	if(Current->GetBox().IsInside(bqd.mBox))
	{
		// Full dump & no further test
		_FullDump(index, bqd.mMaxIndex, bqd.mCells, *bqd.mObjects);
		return;
	}*/

	Prunable* Obj = Current.GetObjects();
	while(Obj)
	{
		Prunable* NextObj = Obj->GetNextObject();	// PT: trying to avoid LHS
		if(NextObj)	Ps::prefetch((void *)NextObj);

		if(Gu::intersectOBBAABB(bqd.mBox, *bqd.mPruner->GetWorldAABB(*Obj)))
			(bqd.mCallback)(&Obj, 1, bqd.mUserData);
//		if(first_contact)	return true;

		Obj = NextObj;
	}

	// Recurse through children
	for(PxU32 CubeIndex=1;CubeIndex<=NB_OCTREE_CHILDREN;CubeIndex++)
	{
		// Compute child index & recurse
		_TestAgainstOBB(MAKE_CHILD_INDEX(index, CubeIndex), bqd);
	}
}

bool PRUNER_NAME::OverlapOBB(ReportPrunablesCallback cb, void* userData, const Gu::Box& box, bool first_contact)
{
	if(!mOctree)	BuildLooseOctree();
	if(!mOctree)	return true;

	OBBQueryData OQD;
#ifdef USE_IMPLICIT_BOX
	OQD.mOctree			= mOctree;
#endif
#ifdef USE_QUADTREE
	OQD.mQuadtreeUp		= mOctree->GetQuadtreeUp();
#endif
	OQD.mMaxIndex		= mOctree->GetNbCells();
	OQD.mCells			= mOctree->GetCells();
	OQD.mBox			= box;

	OQD.mCallback		= cb;
	OQD.mUserData		= userData;
	OQD.mPruner			= this;

	_TestAgainstOBB(0, OQD);

	return true;
}

	struct SphereQueryData
	{
#ifdef USE_IMPLICIT_BOX
		TREE_CLASS_NAME*		mOctree;
#endif
#ifdef USE_QUADTREE
		PxU32					mQuadtreeUp;
#endif
		PxU32					mMaxIndex;
		const CELL_CLASS_NAME*	mCells;
		Gu::Sphere				mSphere;

		ReportPrunablesCallback	mCallback;
		void*					mUserData;

		PRUNER_NAME*			mPruner;
	};

// ### to customize
static void _TestAgainstSphere(PxU32 index, const SphereQueryData& sqd)
{
	// Check index
	if(index>=sqd.mMaxIndex)	return;

	// Check current cell. If cumulative count is NULL, there's no object at all in the cell and in any branches. So we're done.
	const CELL_CLASS_NAME& Current = sqd.mCells[index];
	if(!Current.GetCount())	return;

#ifdef USE_IMPLICIT_BOX
	// PT: optimization potential here: use center/extents
	OctreeBox NodeBox;
	sqd.mOctree->ComputeBox(index, NodeBox);
#else
	const OctreeBox& NodeBox = Current.GetBox();
#endif

		// Test the box against the sphere
#ifdef USE_QUADTREE
		if(!SphereAABB(sqd.mSphere.center, sqd.mSphere.radius, NodeBox, sqd.mQuadtreeUp))	return;
#else
		if(!Gu::intersectSphereAABB(sqd.mSphere.center, sqd.mSphere.radius, NodeBox))	return;
#endif

#ifdef USE_QUADTREE
		if(SphereContainsBox(sqd.mSphere, NodeBox, sqd.mQuadtreeUp))
#else
		if(sqd.mSphere.contains(NodeBox.minimum, NodeBox.maximum))
#endif
		{
			// Full dump & no further test
//			_FullDump(index, sqd.mMaxIndex, sqd.mCells, *sqd.mObjects);
			_FullDumpCB(index, sqd.mMaxIndex, sqd.mCells, sqd.mCallback, sqd.mUserData);
			return;
		}

	Prunable* Obj = Current.GetObjects();
	while(Obj)
	{
		Prunable* NextObj = Obj->GetNextObject();	// PT: trying to avoid LHS
		if(NextObj)	Ps::prefetch((void *)NextObj);

		if(Gu::intersectSphereAABB(sqd.mSphere.center, sqd.mSphere.radius, *sqd.mPruner->GetWorldAABB(*Obj)))
			(sqd.mCallback)(&Obj, 1, sqd.mUserData);
//		if(first_contact)	return true;

		Obj = NextObj;
	}

	// Recurse through children
	for(PxU32 CubeIndex=1;CubeIndex<=NB_OCTREE_CHILDREN;CubeIndex++)
	{
		// Compute child index & recurse
		_TestAgainstSphere(MAKE_CHILD_INDEX(index, CubeIndex), sqd);
	}
}

bool PRUNER_NAME::OverlapSphere(ReportPrunablesCallback cb, void* userData, const Gu::Sphere& sphere, bool first_contact)
{
	if(!mOctree)	BuildLooseOctree();
	if(!mOctree)	return true;

	SphereQueryData SQD;
#ifdef USE_IMPLICIT_BOX
	SQD.mOctree			= mOctree;
#endif
#ifdef USE_QUADTREE
	SQD.mQuadtreeUp		= mOctree->GetQuadtreeUp();
#endif
	SQD.mMaxIndex		= mOctree->GetNbCells();
	SQD.mCells			= mOctree->GetCells();
	SQD.mSphere			= sphere;

	SQD.mCallback		= cb;
	SQD.mUserData		= userData;
	SQD.mPruner			= this;

	_TestAgainstSphere(0, SQD);

	return true;
}


	struct CapsuleQueryData
	{
#ifdef USE_IMPLICIT_BOX
		TREE_CLASS_NAME*		mOctree;
#endif
#ifdef USE_QUADTREE
		PxU32					mQuadtreeUp;
#endif
		PxU32					mMaxIndex;
		const CELL_CLASS_NAME*	mCells;
		Gu::Capsule				mCapsule;

		ReportPrunablesCallback	mCallback;
		void*					mUserData;
		PRUNER_NAME*			mPruner;
	};

// ### to customize
static void _TestAgainstCapsule(PxU32 index, const CapsuleQueryData& cqd)
{
	// Check index
	if(index>=cqd.mMaxIndex)	return;

	// Check current cell. If cumulative count is NULL, there's no object at all in the cell and in any branches. So we're done.
	const CELL_CLASS_NAME& Current = cqd.mCells[index];
	if(!Current.GetCount())	return;

		// Test the box against the capsule
#ifdef USE_IMPLICIT_BOX
	// PT: optimization potential here: use center/extents
	OctreeBox NodeBox;
	cqd.mOctree->ComputeBox(index, NodeBox);
#else
	const OctreeBox& NodeBox = Current.GetBox();
#endif

#ifdef USE_QUADTREE
		const PxU32 y = cqd.mQuadtreeUp;
		const PxU32 z = 3-y;
		PxVec3 TmpMin, TmpMax;
		TmpMin.x	= NodeBox.minimum.x;
		TmpMin[y]	= NodeBox.minimum.y;
		TmpMin[z]	= -1000000.0f;
		TmpMax.x	= NodeBox.maximum.x;
		TmpMax[y]	= NodeBox.maximum.y;
		TmpMax[z]	= 1000000.0f;
		PxBounds3 Tmp;
		Tmp.minimum = TmpMin;
		Tmp.maximum = TmpMax;
		if(!Gu::intersectCapsuleAABB(cqd.mCapsule, Tmp))	return;
#else
		if(!Gu::intersectCapsuleAABB(cqd.mCapsule, NodeBox))	return;
#endif
		// ### not implemented yet
/*		if(cqd.mCapsule->Contains(NodeBox))
		{
			// Full dump & no further test
			_FullDump(index, cqd.mMaxIndex, cqd.mCells, *cqd.mTouched);
			return;
		}*/

	Prunable* Obj = Current.GetObjects();
	while(Obj)
	{
		Prunable* NextObj = Obj->GetNextObject();	// PT: trying to avoid LHS
		if(NextObj)	Ps::prefetch((void *)NextObj);

		if(Gu::intersectCapsuleAABB(cqd.mCapsule, *cqd.mPruner->GetWorldAABB(*Obj)))
			(cqd.mCallback)(&Obj, 1, cqd.mUserData);
//		if(first_contact)	return true;

		Obj = NextObj;
	}

	// Recurse through children
	for(PxU32 CubeIndex=1;CubeIndex<=NB_OCTREE_CHILDREN;CubeIndex++)
	{
		// Compute child index & recurse
		_TestAgainstCapsule(MAKE_CHILD_INDEX(index, CubeIndex), cqd);
	}
}


bool PRUNER_NAME::OverlapCapsule(ReportPrunablesCallback cb, void* userData, const Gu::Capsule& capsule, bool first_contact)
{
	if(!mOctree)	BuildLooseOctree();
	if(!mOctree)	return true;

	CapsuleQueryData CQD;
#ifdef USE_IMPLICIT_BOX
	CQD.mOctree			= mOctree;
#endif
#ifdef USE_QUADTREE
	CQD.mQuadtreeUp		= mOctree->GetQuadtreeUp();
#endif
	CQD.mMaxIndex		= mOctree->GetNbCells();
	CQD.mCells			= mOctree->GetCells();
	CQD.mCapsule		= capsule;

	CQD.mCallback		= cb;
	CQD.mUserData		= userData;
	CQD.mPruner			= this;

	_TestAgainstCapsule(0, CQD);

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "CmRenderOutput.h"
void PRUNER_NAME::visualize(Cm::RenderOutput& out, PxU32 color)
{
#ifdef USE_QUADTREE
	if(mOctree)
	{
		struct Local
		{
			static void convert(PxVec3& v3, const Point2D& v2, PxU32 up)
			{
				v3.x = v2.x;
				if(up==1)
				{
					v3.y = v2.y;
					v3.z = 0.0f;
				}
				else
				{
					v3.y = 0.0f;
					v3.z = v2.y;
				}
			}

			static void convert(PxBounds3& v3, const AABB_2D& v2, PxU32 up)
			{
				Local::convert(v3.minimum, v2.minimum, up);
				Local::convert(v3.maximum, v2.maximum, up);
			}

/*			static bool cb(QuadtreeCell* parent, QuadtreeCell* current, void* userData)
			{
				Cm::RenderOutput* out = (Cm::RenderOutput*)userData;

				if(current)
				{
					if(current->GetCount())
					{
						PxBounds3 worldBox3;
						Local::convert(worldBox3, current->GetBox(), mUp);
						(*out) << Cm::DebugBox(worldBox3, true);
					}
				}
				return true;
			}*/
		};
		PxTransform idt = PxTransform::createIdentity();
		out << idt;
		out << color;

//		mOctree->Walk(Local::cb, &out);

		// Render the quadtree's original world box
		out << PxDebugColor::eARGB_MAGENTA;

		PxBounds3 worldBox3;
		Local::convert(worldBox3, mOctree->GetWorldBox(), mUpAxis);

		out << Cm::DebugBox(worldBox3, true);
	}
#endif

#ifdef USE_OCTREE
	if(mOctree)
	{
		struct Local
		{
			static bool cb(OctreeCell* parent, OctreeCell* current, void* userData)
			{
				Cm::RenderOutput* out = (Cm::RenderOutput*)userData;

				if(current)
				{
					if(current->GetCount())
					{
						(*out) << Cm::DebugBox(current->GetBox(), true);
					}
				}
				return true;
			}
		};
		PxTransform idt = PxTransform::createIdentity();
		out << idt;
		out << color;

		mOctree->Walk(Local::cb, &out);

		// Render the octree's original world box
		out << PxDebugColor::eARGB_MAGENTA;
		out << Cm::DebugBox(mOctree->GetWorldBox(), true);
	}
#endif
}

#endif
