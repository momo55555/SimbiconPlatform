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


#ifndef PX_COLLISION_HEIGHTFIELD
#define PX_COLLISION_HEIGHTFIELD

#include "PsUserAllocated.h"
#include "CmRefCountable.h"
#include "PsMathUtils.h"
#include "PxBounds3.h"
#include "GuSphere.h"
#include "PxHeightFieldSample.h"
#include "PxHeightFieldDesc.h"
#include "GuHeightFieldData.h"
#include "PxHeightField.h"

// PX_SERIALIZATION
#include "PxSerialFramework.h"
#include "CmReflection.h"
//~PX_SERIALIZATION

namespace physx
{

class GuMeshFactory;
class PxHeightFieldDesc;

#ifndef __CELLOS_LV2__
	#define HF_USE_PRECOMPUTED_BITMAP
	#define HF_USE_RESERVED_BIT	// PT: store precomputed data in samples' reserved bit
#endif

}

#ifdef HF_USE_PRECOMPUTED_BITMAP
	#ifndef HF_USE_RESERVED_BIT
	#include "PsBitmap.h"
	#endif
#endif

#ifdef __SPU__
	#include "CellUtil.h"
	#include "..\..\ps3\include\spu\CellHeightfieldTileCache.h"

namespace physx
{
	extern CellHeightfieldTileCache g_sampleCache;
	extern unsigned char g_HFSampleBuffer[32];
}

#endif

namespace physx
{

//#define HF_USE_HEIGHT_CACHE
#ifdef HF_USE_HEIGHT_CACHE
	struct CacheEntry
	{
		PxU32	vertexIndex;
		PxReal	height;
	};
#endif

namespace Gu
{

class HeightField : public PxHeightField, public Ps::UserAllocated, public Cm::RefCountable
{
public:
// PX_SERIALIZATION
												HeightField(PxRefResolver& v)	: PxHeightField(v), Cm::RefCountable(v), mData(v)
#ifdef HF_USE_PRECOMPUTED_BITMAP
	#ifndef HF_USE_RESERVED_BIT
													, mBitmap(v)
	#endif
#endif
												{}
												DECLARE_SERIAL_CLASS(HeightField, PxHeightField)

		virtual		void						exportExtraData(PxSerialStream&);
		virtual		char*						importExtraData(char* address, PxU32& totalPadding);
	PX_FORCE_INLINE	void						setMeshFactory(GuMeshFactory* f)		{ mMeshFactory = f;					}
		static		void						getMetaData(PxSerialStream& stream);
//~PX_SERIALIZATION
												HeightField(GuMeshFactory& meshFactory);

		// PxHeightField
		virtual		void						release();
		virtual		PxU32						saveCells(void* destBuffer, PxU32 destBufferSize) const;
		virtual		PxU32						getNbRows()						const	{ return mData.rows;				}
		virtual		PxU32						getNbColumns()					const	{ return mData.columns;				}
		virtual		PxHeightFieldFormat::Enum	getFormat()						const	{ return mData.format;				}
		virtual		PxU32						getSampleStride()				const	{ return sizeof(PxHeightFieldSample);	}
		virtual		PxReal						getThickness()					const	{ return mData.thickness;			}
		virtual		PxReal						getConvexEdgeThreshold()		const	{ return mData.convexEdgeThreshold;	}
		virtual		PxHeightFieldFlags			getFlags()						const	{ return mData.flags;				}
		virtual		PxReal						getHeight(PxReal x, PxReal z)	const	{ return getHeightInternal(x, z);	}
		virtual		PxU32						getReferenceCount()				const;
		//~PxHeightField

		// RefCountable
		virtual		void						onRefCountZero();
		//~RefCountable
		virtual		PxMaterialTableIndex		getTriangleMaterialIndex(PxTriangleID triangleIndex)	const
												{
													return getTriangleMaterial(triangleIndex);
												}	

					bool						loadFromDesc(const PxHeightFieldDesc&);

	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getNbRowsFast()					const	{ return mData.rows;				}
	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getNbColumnsFast()				const	{ return mData.columns;				}
	PX_FORCE_INLINE	PxHeightFieldFormat::Enum	getFormatFast()					const	{ return mData.format;				}
	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxReal	getThicknessFast()				const	{ return mData.thickness;			}
	PX_FORCE_INLINE	PxU32						getFlagsFast()					const	{ return mData.flags;				}

	PX_FORCE_INLINE	bool						isDeltaHeightInsideExtent(PxReal dy, PxReal eps = 0.0f) const	
												{ 
													return (mData.thickness <= 0.0f && dy <= eps && dy >= mData.thickness) || 
															(mData.thickness > 0.0f && dy > -eps && dy < mData.thickness);
												}

	PX_FORCE_INLINE	bool						isDeltaHeightOppositeExtent(PxReal dy) const	
												{
													return (mData.thickness <= 0.0f && dy > 0.0f) || (mData.thickness > 0.0f && dy < 0.0f);
												}

	PX_CUDA_CALLABLE PX_FORCE_INLINE	bool	isZerothVertexShared(PxU32 vertexIndex) const
												{
//													return (getSample(vertexIndex).tessFlag & PxHeightFieldTessFlag::e0TH_VERTEX_SHARED);
													return getSample(vertexIndex).tessFlag() != 0;
												}

	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU16	getMaterialIndex0(PxU32 vertexIndex) const	{ return getSample(vertexIndex).materialIndex0;	}
	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU16	getMaterialIndex1(PxU32 vertexIndex) const	{ return getSample(vertexIndex).materialIndex1;	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxReal	getHeight(PxU32 vertexIndex) const;
	PX_INLINE		PxReal						getHeightInternal2(PxU32 vertexIndex, PxReal fracX, PxReal fracZ)	const;
	PX_FORCE_INLINE	PxReal						getHeightInternal(PxReal x, PxReal z) const
												{
													PxReal fracX, fracZ;
													const PxU32 vertexIndex = computeCellCoordinates(x, z, fracX, fracZ);

													return getHeightInternal2(vertexIndex, fracX, fracZ);
												}

	PX_FORCE_INLINE bool						isValidVertex(PxU32 vertexIndex) const	{ return vertexIndex < mData.rows*mData.columns;	}

	PX_INLINE		PxVec3						getVertex(PxU32 vertexIndex) const;
//	PX_INLINE		PxU32						getVertexEdgeIndices(PxU32 vertexIndex, PxU32 edgeIndices[8]) const;
	PX_INLINE		bool						isConvexVertex(PxU32 vertexIndex, PxU32 row, PxU32 column) const;

	PX_INLINE		bool						isValidEdge(PxU32 edgeIndex) const;
	PX_INLINE		PxU32						getEdgeTriangleIndices(PxU32 edgeIndex, PxU32 triangleIndices[2]) const;
	PX_INLINE		PxU32						getEdgeTriangleIndices(PxU32 edgeIndex, PxU32 triangleIndices[2], PxU32 cell, PxU32 row, PxU32 column) const;
	PX_INLINE		void						getEdgeVertexIndices(PxU32 edgeIndex, PxU32& vertexIndex0, PxU32& vertexIndex1) const;
//	PX_INLINE		bool						isConvexEdge(PxU32 edgeIndex) const;
	PX_INLINE		bool						isConvexEdge(PxU32 edgeIndex, PxU32 cell, PxU32 row, PxU32 column) const;
	PX_FORCE_INLINE	bool						isConvexEdge(PxU32 edgeIndex) const
												{
													const PxU32 cell = edgeIndex / 3;
													const PxU32 row = cell / mData.columns;
													const PxU32 column = cell % mData.columns;
													return isConvexEdge(edgeIndex, cell, row, column);
												}

//	PX_INLINE		void						computeCellCoordinates(PxReal x, PxReal z, PxU32& row, PxU32& column, PxReal& fracX, PxReal& fracZ) const;
//	PX_INLINE		PxU32						computeCellCoordinates(PxReal x, PxReal z, PxU32 nbColumns, PxReal& fracX, PxReal& fracZ) const;
					PxU32						computeCellCoordinates(PxReal x, PxReal z, PxReal& fracX, PxReal& fracZ) const;

	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getMinRow(PxReal x)		const	{ return PxClamp(PxI32(Ps::floor(x)), PxI32(0), PxI32(mData.rows-2));		}
	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getMaxRow(PxReal x)		const	{ return PxClamp(PxI32(Ps::ceil(x)), PxI32(0), PxI32(mData.rows-1));		}
	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getMinColumn(PxReal z)	const	{ return PxClamp(PxI32(Ps::floor(z)), PxI32(0), PxI32(mData.columns-2));	}
	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU32	getMaxColumn(PxReal z)	const	{ return PxClamp(PxI32(Ps::ceil(z)), PxI32(0), PxI32(mData.columns-1));		}

	PX_CUDA_CALLABLE PX_INLINE			bool	isValidTriangle(PxU32 triangleIndex) const;
	PX_CUDA_CALLABLE PX_FORCE_INLINE	bool	isFirstTriangle(PxU32 triangleIndex) const	{ return ((triangleIndex & 0x1) == 0);	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxU16	getTriangleMaterial(PxU32 triangleIndex) const
												{
													return isFirstTriangle(triangleIndex) ? getMaterialIndex0(triangleIndex >> 1) : getMaterialIndex1(triangleIndex >> 1);
												}

	PX_CUDA_CALLABLE PX_INLINE			void	getTriangleVertexIndices(PxU32 triangleIndex, PxU32& vertexIndex0, PxU32& vertexIndex1, PxU32& vertexIndex2) const;
	PX_CUDA_CALLABLE PX_INLINE			void	getTriangleEdgeIndices(PxU32 triangleIndex, PxU32& edgeIndex0, PxU32& edgeIndex1, PxU32& edgeIndex2) const;
	PX_CUDA_CALLABLE PX_INLINE			PxVec3	getTriangleNormal(PxU32 triangleIndex) const;

	PX_INLINE		PxVec3						getNormal_2(PxU32 vertexIndex, PxReal fracX, PxReal fracZ, PxReal xcoeff, PxReal ycoeff, PxReal zcoeff) const;
	PX_FORCE_INLINE PxVec3						getNormal_(PxReal x, PxReal z, PxReal xcoeff, PxReal ycoeff, PxReal zcoeff) const
												{
													PxReal fracX, fracZ;
													const PxU32 vertexIndex = computeCellCoordinates(x, z, fracX, fracZ);

													return getNormal_2(vertexIndex, fracX, fracZ, xcoeff, ycoeff, zcoeff);
												}

	PX_INLINE		PxU32						getTriangleIndex(PxReal x, PxReal z) const;
	PX_INLINE		PxU32						getTriangleIndex2(PxU32 cell, PxReal fracX, PxReal fracZ) const;
	PX_FORCE_INLINE	PxU16						getMaterial(PxReal x, PxReal z) const
												{
													return getTriangleMaterial(getTriangleIndex(x, z));
												}

	PX_FORCE_INLINE	PxReal						getMinHeight()					const	{ return minHeight; }
	PX_FORCE_INLINE	PxReal						getMaxHeight()					const	{ return maxHeight; }

	PX_FORCE_INLINE	PxU16						getCommonMaterialIndex0()		const	{ return commonMaterialIndex0; }
	PX_FORCE_INLINE	PxU16						getCommonMaterialIndex1()		const	{ return commonMaterialIndex1; }

	PX_FORCE_INLINE	const Gu::HeightFieldData&	getData()						const	{ return mData; }
	
	PX_CUDA_CALLABLE PX_FORCE_INLINE	void	getTriangleVertices(PxU32 triangleIndex, PxU32 row, PxU32 column, PxVec3& v0, PxVec3& v1, PxVec3& v2) const;

#ifdef HF_USE_PRECOMPUTED_BITMAP
					bool						isCollisionVertexPreca(PxU32 vertexIndex, PxU32 row, PxU32 column, PxU16 holeMaterialIndex) const;
	PX_FORCE_INLINE	bool						isCollisionVertex(PxU32 vertexIndex, PxU32, PxU32, PxU16) const
												{
	#ifdef HF_USE_RESERVED_BIT
													return getSample(vertexIndex).materialIndex1.isBitSet()!=0;
	#else
													return mBitmap.test(vertexIndex)!=0;
	#endif
												}
#else
					bool						isCollisionVertex(PxU32 vertexIndex, PxU32 row, PxU32 column, PxU16 holeMaterialIndex) const;
#endif
					PxReal						computeExtreme(PxU32 minRow, PxU32 maxRow, PxU32 minColumn, PxU32 maxColumn)	const;
#ifndef __SPU__
	PX_FORCE_INLINE 
#endif
	PX_CUDA_CALLABLE const PxHeightFieldSample&	getSample(PxU32 vertexIndex) const
												{
													PX_ASSERT(isValidVertex(vertexIndex));

#ifdef HF_TILED_MEMORY_LAYOUT
													PxU32 col = vertexIndex%mData.columns;
													PxU32 row = vertexIndex/mData.columns;
#endif

#ifdef __SPU__

	#ifdef HF_TILED_MEMORY_LAYOUT
													
													PxHeightFieldSample* cachedHF = g_sampleCache.getSample(col, row);

													/*
													PxU32 vertexIndexBase = ((row/HF_TILE_SIZE_V)*mData.tilesU+(col/HF_TILE_SIZE_U))*HF_TILE_SIZE_V*HF_TILE_SIZE_U;
													PxU32 vertexIndexOffset = (row&(PxU32)(HF_TILE_SIZE_V-1))*HF_TILE_SIZE_U+(col&(PxU32)(HF_TILE_SIZE_U-1));
													vertexIndex = vertexIndexBase + vertexIndexOffset;
													spu_printf("Load1: from %x %d\n", mData.samples2+vertexIndexBase, vertexIndexOffset*4);
													PxHeightFieldSample* pHF = pxMemFetchAsync<PxHeightFieldSample>(g_HFSampleBuffer, PxMemFetchPtr(mData.samples2+vertexIndex), sizeof(PxHeightFieldSample), 1);
													pxMemFetchWait(1);
													if (cachedHF->height != pHF->height || cachedHF->tessFlag != pHF->tessFlag || cachedHF->materialIndex0 != pHF->materialIndex0 || cachedHF->materialIndex1 != pHF->materialIndex1) {
														spu_printf("Error: %d != %d col %d row %d\n", cachedHF->height, pHF->height, col, row);
													}*/

													return *cachedHF;

	#else
													PxHeightFieldSample* pHF = pxMemFetchAsync<PxHeightFieldSample>(g_HFSampleBuffer, PxMemFetchPtr(mData.samples+vertexIndex), sizeof(PxHeightFieldSample), 1);
													pxMemFetchWait(1);
													return *pHF;
	#endif
#else
	#ifdef HF_TILED_MEMORY_LAYOUT
													PxU32 vertexIndexBase = ((row/HF_TILE_SIZE_V)*mData.tilesU+(col/HF_TILE_SIZE_U))*HF_TILE_SIZE_V*HF_TILE_SIZE_U;
													PxU32 vertexIndexOffset = (row&(PxU32)(HF_TILE_SIZE_V-1))*HF_TILE_SIZE_U+(col&(PxU32)(HF_TILE_SIZE_U-1));
													vertexIndex = vertexIndexBase + vertexIndexOffset;
	#endif
													return mData.samples[vertexIndex];
#endif
												}

#ifdef __CUDACC__
	PX_CUDA_CALLABLE void		setSamplePtr(PxHeightFieldSample* s) { mData.samples = s; }
#endif

protected:
#ifdef HF_USE_PRECOMPUTED_BITMAP
	#ifndef HF_USE_RESERVED_BIT
					Cm::BitMap					mBitmap;
	#endif
#endif
#ifdef HF_USE_HEIGHT_CACHE
		mutable		CacheEntry					mCache[32];
#endif
					Gu::HeightFieldData			mData;
					PxU32						mSampleStride;
					PxU32						mNbSamples;	// PT: added for platform conversion. Try to remove later.
					PxReal						minHeight;
					PxReal						maxHeight;
					PxMaterialTableIndex		commonMaterialIndex0;
					PxMaterialTableIndex		commonMaterialIndex1;
					// methods
					void						releaseMemory();

	virtual										~HeightField();

private:
					GuMeshFactory*				mMeshFactory;	// PT: changed to pointer for serialization
};

} // namespace Gu


PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal Gu::HeightField::getHeight(PxU32 vertexIndex) const
{
#ifdef HF_USE_HEIGHT_CACHE
	#ifdef _DEBUG
	static PxU32 hits = 0;
	static PxU32 misses = 0;
	#endif
	const PxU32 index = vertexIndex&31;
	if(mCache[index].vertexIndex==vertexIndex)
	{
	#ifdef _DEBUG
		hits++;
	#endif
		return mCache[index].height;
	}
	#ifdef _DEBUG
	misses++;
	#endif
	mCache[index].vertexIndex = vertexIndex;
	const PxReal height = PxReal(getSample(vertexIndex).height);
	mCache[index].height = height;
	return height;
#else
	return PxReal(getSample(vertexIndex).height);
#endif
}

PX_INLINE PxVec3 Gu::HeightField::getVertex(PxU32 vertexIndex) const
{
	const PxU32 row    = vertexIndex / mData.columns;
	const PxU32 column = vertexIndex % mData.columns;
//	return PxVec3(PxReal(row), getHeight(row * mData.columns + column), PxReal(column));
	return PxVec3(PxReal(row), getHeight(vertexIndex), PxReal(column));
}

#ifdef REMOVED
PX_INLINE PxU32 Gu::HeightField::getVertexEdgeIndices(PxU32 vertexIndex, PxU32 edgeIndices[8]) const
{
	const PxU32 row    = vertexIndex / mData.columns;
	const PxU32 column = vertexIndex % mData.columns;
	PxU32 count = 0;
	
	if (row > 0) 
	{
		/*
		//   COL -->
		//       
		// R     |
		// O     |
		// W     +  
		// |      
		// V  
		*/
		edgeIndices[count++] = 3 * (vertexIndex - mData.columns) + 2;
	}
	
	if (column < mData.columns-1)
	{
		/*
		//   COL -->
		//       
		// R       /
		// O      / 
		// W     +--
		// |      \
		// V       \ 
		*/
		if (row > 0)
		{
			/*
			//   COL -->
			//       
			// R       /
			// O      / 
			// W     +
			// |       
			// V
			*/
			if (!isZerothVertexShared(vertexIndex - mData.columns))
			{
				edgeIndices[count++] = 3 * (vertexIndex - mData.columns) + 1;
			}
		}
		/*
		//   COL -->
		//       
		// R     
		// O     
		// W     +--
		// |       
		// V         
		*/
		edgeIndices[count++] = 3 * vertexIndex;
		if (row < mData.rows - 1)
		{
			/*
			//   COL -->
			//       
			// R     
			// O     
			// W     +  
			// |      \
			// V       \ 
			*/
			if (isZerothVertexShared(vertexIndex))
			{
				edgeIndices[count++] = 3 * vertexIndex + 1;
			}
		}
	}

	/*
	//   COL -->
	//       
	// R     
	// O     
	// W     +  
	// |     |
	// V     | 
	*/
	if (row < mData.rows - 1)
	{
		edgeIndices[count++] = 3 * vertexIndex + 2;
	}

	if (column > 0)
	{
		/*
		//   COL -->
		//        
		// R   \  
		// O    \ 
		// W   --+
		// |    / 
		// V   /   
		*/
		if (row < mData.rows - 1)
		{
			if (!isZerothVertexShared(vertexIndex - 1))
			{
				edgeIndices[count++] = 3 * (vertexIndex - 1) + 1;
			}
		}
		edgeIndices[count++] = 3 * (vertexIndex - 1);
		if (row > 0)
		{
			if (isZerothVertexShared(vertexIndex - mData.columns - 1))
			{
				edgeIndices[count++] = 3 * (vertexIndex - mData.columns - 1) + 1;
			}
		}
	}
	return count;
}
#endif

// PT: only called from "isCollisionVertex", should move
PX_INLINE bool Gu::HeightField::isConvexVertex(PxU32 vertexIndex, PxU32 row, PxU32 column) const
{
#ifdef PX_HEIGHTFIELD_DEBUG
	PX_ASSERT(isValidVertex(vertexIndex));
#endif
	PX_ASSERT((vertexIndex / mData.columns)==row);
	PX_ASSERT((vertexIndex % mData.columns)==column);

//	PxReal h0 = PxReal(2) * getHeight(vertexIndex);
	PxI32 h0 = getSample(vertexIndex).height;
	h0 += h0;

	bool definedInX, definedInZ;
	PxI32 convexityX, convexityZ;

	if ((row > 0) &&  (row < mData.rows - 1))
	{
//		convexityX = h0 - getHeight(vertexIndex + mData.columns) - getHeight(vertexIndex - mData.columns);
		convexityX = h0 - getSample(vertexIndex + mData.columns).height - getSample(vertexIndex - mData.columns).height;
		definedInX = true;
	}
	else
	{
		convexityX = 0;
		definedInX = false;
	}

	if ((column > 0) &&  (column < mData.columns - 1))
	{
//		convexityZ = h0 - getHeight(vertexIndex + 1) - getHeight(vertexIndex - 1);
		convexityZ = h0 - getSample(vertexIndex + 1).height - getSample(vertexIndex - 1).height;
		definedInZ = true;
	}
	else
	{
		convexityZ = 0;
		definedInZ = false;
	}

	if(definedInX || definedInZ)
	{
		// PT: use XOR here
		// saddle points
/*		if ((convexityX > 0) && (convexityZ < 0)) 
			return false;		
		if ((convexityX < 0) && (convexityZ > 0)) 
			return false;*/
		if(((convexityX ^ convexityZ) & 0x80000000)==0)
			return false;

		// inequality depends on thickness and offset by threshold.
		const PxReal value = PxReal(convexityX + convexityZ);
		// PT: thickness is always the same for a given heightfield so the comparison shouldn't be here
		if (mData.thickness <= 0)
			return value > mData.convexEdgeThreshold;
		else 
			return value < -mData.convexEdgeThreshold;
	}

	// this has to be one of the two corner vertices
	return true;
}


PX_INLINE bool Gu::HeightField::isValidEdge(PxU32 edgeIndex) const
{
	const PxU32 cell   = (edgeIndex / 3);
	const PxU32 row    = cell / mData.columns;
	const PxU32 column = cell % mData.columns;
//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
		case 0:
			if (row > mData.rows - 1) return false;
			if (column >= mData.columns - 1) return false;
			break;
		case 1:
			if (row >= mData.rows - 1) return false;
			if (column >= mData.columns - 1) return false;
			break;
		case 2:
			if (row >= mData.rows - 1) return false;
			if (column > mData.columns - 1) return false;
			break;
	}
	return true;
}


PX_INLINE PxU32 Gu::HeightField::getEdgeTriangleIndices(PxU32 edgeIndex, PxU32 triangleIndices[2]) const
{
	const PxU32 cell   = edgeIndex / 3;
	const PxU32 row    = cell / mData.columns;
	const PxU32 column = cell % mData.columns;
	PxU32 count = 0;
//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
		case 0:
			if (column < mData.columns - 1)
			{
				if (row > 0)
				{
/*					if (isZerothVertexShared(cell - mData.columns))
						triangleIndices[count++] = ((cell - mData.columns) << 1);
					else 
						triangleIndices[count++] = ((cell - mData.columns) << 1) + 1;*/
					triangleIndices[count++] = ((cell - mData.columns) << 1) + isZerothVertexShared(cell - mData.columns);
				}
				if (row < mData.rows - 1)
				{
/*					if (isZerothVertexShared(cell))
						triangleIndices[count++] = (cell << 1) + 1;
					else 
						triangleIndices[count++] = cell << 1;*/
					triangleIndices[count++] = (cell << 1) + isZerothVertexShared(cell);
				}
			}
			break;
		case 1:
			if ((row < mData.rows - 1) && (column < mData.columns - 1))
			{
				triangleIndices[count++] = cell << 1;
				triangleIndices[count++] = (cell << 1) + 1;
			}
			break;
		case 2:
			if (row < mData.rows - 1)
			{
				if (column > 0)
					triangleIndices[count++] = ((cell - 1) << 1) + 1;
				if (column < mData.columns - 1)
					triangleIndices[count++] = cell << 1;
			}
			break;
	}
	return count;
}

PX_INLINE PxU32 Gu::HeightField::getEdgeTriangleIndices(PxU32 edgeIndex, PxU32 triangleIndices[2], PxU32 cell, PxU32 row, PxU32 column) const
{
//	const PxU32 cell   = edgeIndex / 3;
//	const PxU32 row    = cell / mData.columns;
//	const PxU32 column = cell % mData.columns;
	PxU32 count = 0;
//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
		case 0:
			if (column < mData.columns - 1)
			{
				if (row > 0)
				{
/*					if (isZerothVertexShared(cell - mData.columns))
						triangleIndices[count++] = ((cell - mData.columns) << 1);
					else 
						triangleIndices[count++] = ((cell - mData.columns) << 1) + 1;*/
					triangleIndices[count++] = ((cell - mData.columns) << 1) + isZerothVertexShared(cell - mData.columns);
				}
				if (row < mData.rows - 1)
				{
/*					if (isZerothVertexShared(cell))
						triangleIndices[count++] = (cell << 1) + 1;
					else 
						triangleIndices[count++] = cell << 1;*/
					triangleIndices[count++] = (cell << 1) + isZerothVertexShared(cell);
				}
			}
			break;
		case 1:
			if ((row < mData.rows - 1) && (column < mData.columns - 1))
			{
				triangleIndices[count++] = cell << 1;
				triangleIndices[count++] = (cell << 1) + 1;
			}
			break;
		case 2:
			if (row < mData.rows - 1)
			{
				if (column > 0)
					triangleIndices[count++] = ((cell - 1) << 1) + 1;
				if (column < mData.columns - 1)
					triangleIndices[count++] = cell << 1;
			}
			break;
	}
	return count;
}


PX_INLINE void Gu::HeightField::getEdgeVertexIndices(PxU32 edgeIndex, PxU32& vertexIndex0, PxU32& vertexIndex1) const
{
	const PxU32 cell = edgeIndex / 3;
//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
		case 0:
			vertexIndex0 = cell;
			vertexIndex1 = cell + 1;
			break;
		case 1:
			{
/*			if (isZerothVertexShared(cell))
			{
				vertexIndex0 = cell;
				vertexIndex1 = cell + mData.columns + 1;
			}
			else
			{
				vertexIndex0 = cell + 1;
				vertexIndex1 = cell + mData.columns;
			}*/
			const bool b = isZerothVertexShared(cell);
			vertexIndex0 = cell + 1 - b;
			vertexIndex1 = cell + mData.columns + b;
			}
			break;
		case 2:
			vertexIndex0 = cell;
			vertexIndex1 = cell + mData.columns;
			break;
	}
}

#ifdef REMOVED
PX_INLINE bool Gu::HeightField::isConvexEdge(PxU32 edgeIndex) const
{
	const PxU32 cell = edgeIndex / 3;

	const PxU32 row = cell / mData.columns;
	if (row > mData.rows-2) return false;

	const PxU32 column = cell % mData.columns;
	if (column > mData.columns-2) return false;

//	PxReal h0 = 0, h1 = 0, h2 = 0, h3 = 0;
//	PxReal convexity = 0;
	PxI32 h0 = 0, h1 = 0, h2 = 0, h3 = 0;
	PxI32 convexity = 0;

//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
		case 0:
			if (row < 1) return false;
/*			if(isZerothVertexShared(cell - mData.columns)) 
			{
				//      <------ COL  
				//       +----+  0  R
				//       |   /  /#  O
				//       |  /  / #  W
				//       | /  /  #  |
				//       |/  /   #  |
				//       +  +====1  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  V
				//      
//				h0 = getHeight(cell - mData.columns);
//				h1 = getHeight(cell);
				h0 = getSample(cell - mData.columns).height;
				h1 = getSample(cell).height;
			}
			else
			{
				//      <------ COL  
				//       0  +----+  R
				//       #\  \   |  O
				//       # \  \  |  W
				//       #  \  \ |  |
				//       #   \  \|  |
				//       1====+  +  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  V
				//      
//				h0 = getHeight(cell - mData.columns + 1);
//				h1 = getHeight(cell + 1);
				h0 = getSample(cell - mData.columns + 1).height;
				h1 = getSample(cell + 1).height;
			}*/
			const bool b0 = !isZerothVertexShared(cell - mData.columns);
			h0 = getSample(cell - mData.columns + b0).height;
			h1 = getSample(cell + b0).height;

/*			if(isZerothVertexShared(cell)) 
			{
				//      <------ COL  
				//                  R
				//                  O
				//                  W
				//                  |
				//                  |
				//                  |
				//       2====+  0  |
				//       #   /  /|  |
				//       #  /  / |  |
				//       # /  /  |  |
				//       #/  /   |  |
				//       3  +----+  |
				//                  V
				//      
//				h2 = getHeight(cell + 1);
//				h3 = getHeight(cell + mData.columns + 1);
				h2 = getSample(cell + 1).height;
				h3 = getSample(cell + mData.columns + 1).height;
			}
			else
			{
				//      <------ COL  
				//                  R
				//                  O
				//                  W
				//                  |
				//                  |
				//                  |
				//       +  +====2  |
				//       |\  \   #  |
				//       | \  \  #  |
				//       |  \  \ #  |
				//       |   \  \#  |
				//       +----+  3  |
				//                  V
				//      
//				h2 = getHeight(cell);
//				h3 = getHeight(cell + mData.columns);
				h2 = getSample(cell).height;
				h3 = getSample(cell + mData.columns).height;
			}*/
			const bool b1 = isZerothVertexShared(cell);
			h2 = getSample(cell + b1).height;
			h3 = getSample(cell + mData.columns + b1).height;

			//convex = (h3-h2) < (h1-h0);
			convexity = (h1-h0) - (h3-h2);
			break;
		case 1:
//			h0 = getHeight(cell);
//			h1 = getHeight(cell + 1);
//			h2 = getHeight(cell + mData.columns);
//			h3 = getHeight(cell + mData.columns + 1);
			h0 = getSample(cell).height;
			h1 = getSample(cell + 1).height;
			h2 = getSample(cell + mData.columns).height;
			h3 = getSample(cell + mData.columns + 1).height;
			if (isZerothVertexShared(cell))
				//convex = (h0 + h3) > (h1 + h2);
				convexity = (h0 + h3) - (h1 + h2);
			else 
				//convex = (h2 + h1) > (h0 + h3);
				convexity = (h2 + h1) - (h0 + h3);
			break;
		case 2:
			if (column < 1) return false;
/*			if(isZerothVertexShared(cell-1)) 
			{
				//      <-------------- COL  
				//                1====0  + R
				//                +   /  /| O
				//                +  /  / | W
				//                + /  /  | |
				//                +/  /   | |
				//                +  +----+ V
				//      
//				h0 = getHeight(cell - 1);
//				h1 = getHeight(cell);
				h0 = getSample(cell - 1).height;
				h1 = getSample(cell).height;
			}
			else
			{
				//      <-------------- COL  
				//                +  +----+ R
				//                +\  \   | O
				//                + \  \  | W
				//                +  \  \ | |
				//                +   \  \| |
				//                1====0  + V
				//      
//				h0 = getHeight(cell - 1 + mData.columns);
//				h1 = getHeight(cell + mData.columns);
				h0 = getSample(cell - 1 + mData.columns).height;
				h1 = getSample(cell + mData.columns).height;
			}*/
			const PxU32 offset0 = isZerothVertexShared(cell-1) ? 0 : mData.columns;
			h0 = getSample(cell - 1 + offset0).height;
			h1 = getSample(cell + offset0).height;

/*			if(isZerothVertexShared(cell)) 
			{
				//      <-------------- COL  
				//       +----+  +          R
				//       |   /  /+          O
				//       |  /  / +          W
				//       | /  /  +          |
				//       |/  /   +          |
				//       +  3====2          V
				//      
//				h2 = getHeight(cell + mData.columns);
//				h3 = getHeight(cell + mData.columns + 1);
				h2 = getSample(cell + mData.columns).height;
				h3 = getSample(cell + mData.columns + 1).height;
			}
			else
			{
				//      <-------------- COL  
				//       +  3====2          R
				//       |\  \   +          O
				//       | \  \  +          W
				//       |  \  \ +          |
				//       |   \  \+          |
				//       +----+  +          V
				//      
//				h2 = getHeight(cell);
//				h3 = getHeight(cell + 1);
				h2 = getSample(cell).height;
				h3 = getSample(cell + 1).height;
			}*/
			const PxU32 offset1 = isZerothVertexShared(cell) ? mData.columns : 0;
			h2 = getSample(cell + offset1).height;
			h3 = getSample(cell + offset1 + 1).height;

			//convex = (h3-h2) < (h1-h0);
			convexity = (h1-h0) - (h3-h2);
			break;
	}

	const PxI32 threshold = PxI32(mData.convexEdgeThreshold);
	if (mData.thickness <= 0)
	{
//		return convexity > mData.convexEdgeThreshold;
		return convexity > threshold;
	}
	else
	{
//		return convexity < -mData.convexEdgeThreshold;
		return convexity < -threshold;
	}
}
#endif

PX_INLINE bool Gu::HeightField::isConvexEdge(PxU32 edgeIndex, PxU32 cell, PxU32 row, PxU32 column) const
{
//	const PxU32 cell = edgeIndex / 3;
	PX_ASSERT(cell == edgeIndex / 3);

//	const PxU32 row = cell / mData.columns;
	PX_ASSERT(row == cell / mData.columns);
	if (row > mData.rows-2) return false;

//	const PxU32 column = cell % mData.columns;
	PX_ASSERT(column == cell % mData.columns);
	if (column > mData.columns-2) return false;

//	PxReal h0 = 0, h1 = 0, h2 = 0, h3 = 0;
//	PxReal convexity = 0;
	PxI32 h0 = 0, h1 = 0, h2 = 0, h3 = 0;
	PxI32 convexity = 0;

//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
		case 0:
			{
			if (row < 1) return false;
/*			if(isZerothVertexShared(cell - mData.columns)) 
			{
				//      <------ COL  
				//       +----+  0  R
				//       |   /  /#  O
				//       |  /  / #  W
				//       | /  /  #  |
				//       |/  /   #  |
				//       +  +====1  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  V
				//      
//				h0 = getHeight(cell - mData.columns);
//				h1 = getHeight(cell);
				h0 = getSample(cell - mData.columns).height;
				h1 = getSample(cell).height;
			}
			else
			{
				//      <------ COL  
				//       0  +----+  R
				//       #\  \   |  O
				//       # \  \  |  W
				//       #  \  \ |  |
				//       #   \  \|  |
				//       1====+  +  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  |
				//                  V
				//      
//				h0 = getHeight(cell - mData.columns + 1);
//				h1 = getHeight(cell + 1);
				h0 = getSample(cell - mData.columns + 1).height;
				h1 = getSample(cell + 1).height;
			}*/
			const bool b0 = !isZerothVertexShared(cell - mData.columns);
			h0 = getSample(cell - mData.columns + b0).height;
			h1 = getSample(cell + b0).height;

/*			if(isZerothVertexShared(cell)) 
			{
				//      <------ COL  
				//                  R
				//                  O
				//                  W
				//                  |
				//                  |
				//                  |
				//       2====+  0  |
				//       #   /  /|  |
				//       #  /  / |  |
				//       # /  /  |  |
				//       #/  /   |  |
				//       3  +----+  |
				//                  V
				//      
//				h2 = getHeight(cell + 1);
//				h3 = getHeight(cell + mData.columns + 1);
				h2 = getSample(cell + 1).height;
				h3 = getSample(cell + mData.columns + 1).height;
			}
			else
			{
				//      <------ COL  
				//                  R
				//                  O
				//                  W
				//                  |
				//                  |
				//                  |
				//       +  +====2  |
				//       |\  \   #  |
				//       | \  \  #  |
				//       |  \  \ #  |
				//       |   \  \#  |
				//       +----+  3  |
				//                  V
				//      
//				h2 = getHeight(cell);
//				h3 = getHeight(cell + mData.columns);
				h2 = getSample(cell).height;
				h3 = getSample(cell + mData.columns).height;
			}*/
			const bool b1 = isZerothVertexShared(cell);
			h2 = getSample(cell + b1).height;
			h3 = getSample(cell + mData.columns + b1).height;

			//convex = (h3-h2) < (h1-h0);
			convexity = (h1-h0) - (h3-h2);
			}
			break;
		case 1:
//			h0 = getHeight(cell);
//			h1 = getHeight(cell + 1);
//			h2 = getHeight(cell + mData.columns);
//			h3 = getHeight(cell + mData.columns + 1);
			h0 = getSample(cell).height;
			h1 = getSample(cell + 1).height;
			h2 = getSample(cell + mData.columns).height;
			h3 = getSample(cell + mData.columns + 1).height;
			if (isZerothVertexShared(cell))
				//convex = (h0 + h3) > (h1 + h2);
				convexity = (h0 + h3) - (h1 + h2);
			else 
				//convex = (h2 + h1) > (h0 + h3);
				convexity = (h2 + h1) - (h0 + h3);
			break;
		case 2:
			{
			if (column < 1) return false;
/*			if(isZerothVertexShared(cell-1)) 
			{
				//      <-------------- COL  
				//                1====0  + R
				//                +   /  /| O
				//                +  /  / | W
				//                + /  /  | |
				//                +/  /   | |
				//                +  +----+ V
				//      
//				h0 = getHeight(cell - 1);
//				h1 = getHeight(cell);
				h0 = getSample(cell - 1).height;
				h1 = getSample(cell).height;
			}
			else
			{
				//      <-------------- COL  
				//                +  +----+ R
				//                +\  \   | O
				//                + \  \  | W
				//                +  \  \ | |
				//                +   \  \| |
				//                1====0  + V
				//      
//				h0 = getHeight(cell - 1 + mData.columns);
//				h1 = getHeight(cell + mData.columns);
				h0 = getSample(cell - 1 + mData.columns).height;
				h1 = getSample(cell + mData.columns).height;
			}*/
			const PxU32 offset0 = isZerothVertexShared(cell-1) ? 0 : mData.columns;
			h0 = getSample(cell - 1 + offset0).height;
			h1 = getSample(cell + offset0).height;

/*			if(isZerothVertexShared(cell)) 
			{
				//      <-------------- COL  
				//       +----+  +          R
				//       |   /  /+          O
				//       |  /  / +          W
				//       | /  /  +          |
				//       |/  /   +          |
				//       +  3====2          V
				//      
//				h2 = getHeight(cell + mData.columns);
//				h3 = getHeight(cell + mData.columns + 1);
				h2 = getSample(cell + mData.columns).height;
				h3 = getSample(cell + mData.columns + 1).height;
			}
			else
			{
				//      <-------------- COL  
				//       +  3====2          R
				//       |\  \   +          O
				//       | \  \  +          W
				//       |  \  \ +          |
				//       |   \  \+          |
				//       +----+  +          V
				//      
//				h2 = getHeight(cell);
//				h3 = getHeight(cell + 1);
				h2 = getSample(cell).height;
				h3 = getSample(cell + 1).height;
			}*/
			const PxU32 offset1 = isZerothVertexShared(cell) ? mData.columns : 0;
			h2 = getSample(cell + offset1).height;
			h3 = getSample(cell + offset1 + 1).height;

			//convex = (h3-h2) < (h1-h0);
			convexity = (h1-h0) - (h3-h2);
			}
			break;
	}

	const PxI32 threshold = PxI32(mData.convexEdgeThreshold);
	if (mData.thickness <= 0)
	{
//		return convexity > mData.convexEdgeThreshold;
		return convexity > threshold;
	}
	else
	{
//		return convexity < -mData.convexEdgeThreshold;
		return convexity < -threshold;
	}
}
#ifdef REMOVED
PX_INLINE void Gu::HeightField::computeCellCoordinates(PxReal x, PxReal z, PxU32& _row, PxU32& _column, PxReal& _fracX, PxReal& _fracZ) const
{
	// ptchernev: 
	// Had to remove this assert since it may happen sometimes due to float precision (or lack thereof).
	// When HeightFieldShape::raycast() clips the ray the clipped points may end up ever so slightly on
	// the wrong side of the height field bounds. I could have fixed the problem there but I could only 
	// guarantee that the points where inside the shape space bounds. Something may still have gone wrong 
	// when transforming to height field grid space.
	// PX_ASSERT(!((x < 0) || (z < 0) || (x > mData.rows - 1) || (z > mData.columns - 1)));
	
	// ptchernev:
	// Apparently there is no standard for float to int conversion of negative values (cause that would make sense).
//	if (x < 0) x = 0;
//	if (z < 0) z = 0;
	x = physx::intrinsics::selectMax(x, 0.0f);
	z = physx::intrinsics::selectMax(z, 0.0f);

/*	const PxReal xf = floorf(x);
//	PxU32 row = (PxU32)x;
	PxU32 row = intFloor(x);
	PxReal fracX = x - xf;*/
	PxU32 row = (PxU32)x;
	PxReal fracX = x - PxReal(row);
	if (row > mData.rows - 2) 
	{
		row = mData.rows - 2;
		fracX = PxReal(1);
	}

/*	const PxReal zf = floorf(z);
//	PxU32 column = (PxU32)z;
	PxU32 column = intFloor(z);
	PxReal fracZ = z - zf;*/
	PxU32 column = (PxU32)z;
	PxReal fracZ = z - PxReal(column);
	if (column > mData.columns - 2) 
	{
		column = mData.columns - 2;
		fracZ = PxReal(1);
	}

	_row = row;
	_column = column;
	_fracX = fracX;
	_fracZ = fracZ;
}
#endif

PX_INLINE bool Gu::HeightField::isValidTriangle(PxU32 triangleIndex) const
{
	const PxU32 cell = triangleIndex >> 1;
	const PxU32 row  = cell / mData.columns;
	if (row >= (mData.rows - 1)) return false;
	const PxU32 column = cell % mData.columns;
	if (column >= (mData.columns - 1)) return false;
	return true;
}




PX_INLINE void Gu::HeightField::getTriangleVertexIndices(PxU32 triangleIndex, PxU32& vertexIndex0, PxU32& vertexIndex1, PxU32& vertexIndex2) const
{
	const PxU32 cell = triangleIndex >> 1;
	if (isZerothVertexShared(cell))
	{
		//      <---- COL  
		//      0----2  1 R
		//      | 1 /  /| O
		//      |  /  / | W
		//      | /  /  | |
		//      |/  / 0 | |
		//      1  2----0 V
		//      
		if (isFirstTriangle(triangleIndex))
		{
			vertexIndex0 = cell + mData.columns;
			vertexIndex1 = cell;
			vertexIndex2 = cell + mData.columns + 1;
		}
		else
		{
			vertexIndex0 = cell + 1;
			vertexIndex1 = cell + mData.columns + 1;
			vertexIndex2 = cell;
		}
	}
	else
	{
		//      <---- COL  
		//      2  1----0 R
		//      |\  \ 0 | O
		//      | \  \  | W
		//      |  \  \ | |
		//      | 1 \  \| |
		//      0----1  2 V
		//                   
		if (isFirstTriangle(triangleIndex))
		{
			vertexIndex0 = cell;
			vertexIndex1 = cell + 1;
			vertexIndex2 = cell + mData.columns;
		}
		else
		{
			vertexIndex0 = cell + mData.columns + 1;
			vertexIndex1 = cell + mData.columns;
			vertexIndex2 = cell + 1;
		}
	}
}


PX_INLINE void Gu::HeightField::getTriangleEdgeIndices(PxU32 triangleIndex, PxU32& edgeIndex0, PxU32& edgeIndex1, PxU32& edgeIndex2) const
{
	const PxU32 cell = triangleIndex >> 1;
	const PxU32 edge = 3 * cell;
	//      <-- COL  
	//      +--0--+ R
	//      |:   :| O
	//      | : : | W
	//      |  1  2 |
	//      | : : | |
	//      |:   :| |
	//      +-----+ V
	//      
	if (isZerothVertexShared(cell))
	{
		//      <---- COL  
		//      +-2--+  + R
		//      |   /  /| O
		//      0  1  / | W
		//      | /  1  0 |
		//      |/  /   | |
		//      +  +--2-+ V
		//      
		if (isFirstTriangle(triangleIndex))
		{
			edgeIndex0 = edge + 2;
			edgeIndex1 = edge + 1;
			edgeIndex2 = edge + 3*mData.columns;
		}
		else
		{
			edgeIndex0 = edge + 3 + 2;
			edgeIndex1 = edge + 1;
			edgeIndex2 = edge;
		}
	}
	else
	{
		//      <---- COL  
		//      +  +--0-+ R
		//      |\  \   | O
		//      | \  1  2 W
		//      2  1  \ | |
		//      |   \  \| |
		//      +-0--+  + V
		//      
		if (isFirstTriangle(triangleIndex))
		{
			edgeIndex0 = edge;
			edgeIndex1 = edge + 1;
			edgeIndex2 = edge + 2;
		}
		else
		{
			edgeIndex0 = edge + 3*mData.columns;
			edgeIndex1 = edge + 1;
			edgeIndex2 = edge + 3 + 2;
		}
	}
}


PX_INLINE PxVec3 Gu::HeightField::getTriangleNormal(PxU32 triangleIndex) const
{
	PxU32 v0, v1, v2;
	getTriangleVertexIndices(triangleIndex, v0, v1, v2); 

//	const PxReal h0 = getHeight(v0);
//	const PxReal h1 = getHeight(v1);
//	const PxReal h2 = getHeight(v2);
	const PxI32 h0 = getSample(v0).height;
	const PxI32 h1 = getSample(v1).height;
	const PxI32 h2 = getSample(v2).height;

	// Fix for NvBug 685420
//	if(mThickness>0.0f)
//		n = -n;
	const PxReal coeff = physx::intrinsics::fsel(mData.thickness, -1.0f, 1.0f);

//	PxVec3 n(0,1,0);
	const PxU32 cell = triangleIndex >> 1;
	if (isZerothVertexShared(cell))
	{
		//      <---- COL  
		//      0----2  1 R
		//      | 1 /  /| O
		//      |  /  / | W
		//      | /  /  | |
		//      |/  / 0 | |
		//      1  2----0 V
		//      
		if (isFirstTriangle(triangleIndex))
		{
//			n.x = -(h0-h1);
//			n.z = -(h2-h0);
			return PxVec3(coeff*PxReal(h1-h0), coeff, coeff*PxReal(h0-h2));
		}
		else
		{
//			n.x = -(h1-h0);
//			n.z = -(h0-h2);
			return PxVec3(coeff*PxReal(h0-h1), coeff, coeff*PxReal(h2-h0));
		}
	}
	else
	{
		//      <---- COL  
		//      2  1----0 R
		//      |\  \ 0 | O
		//      | \  \  | W
		//      |  \  \ | |
		//      | 1 \  \| |
		//      0----1  2 V
		//                   
		if (isFirstTriangle(triangleIndex))
		{
//			n.x = -(h2-h0);
//			n.z = -(h1-h0);
			return PxVec3(coeff*PxReal(h0-h2), coeff, coeff*PxReal(h0-h1));
		}
		else
		{
//			n.x = -(h0-h2);
//			n.z = -(h0-h1);
			return PxVec3(coeff*PxReal(h2-h0), coeff, coeff*PxReal(h1-h0));
		}
	}
//	return n;
}

PX_INLINE PxReal Gu::HeightField::getHeightInternal2(PxU32 vertexIndex, PxReal fracX, PxReal fracZ) const
{
	if (isZerothVertexShared(vertexIndex))
	{
		//    <----Z---+
		//      +----+ | 
		//      |   /| |
		//      |  / | X
		//      | /  | |
		//      |/   | |
		//      +----+ |
		//             V
		const PxReal h0 = getHeight(vertexIndex);
		const PxReal h2 = getHeight(vertexIndex + mData.columns + 1);
		if (fracZ > fracX)
		{
			//    <----Z---+
			//      1----0 | 
			//      |   /  |
			//      |  /   X
			//      | /    |
			//      |/     |
			//      2      |
			//             V
			const PxReal h1 = getHeight(vertexIndex + 1);
			return h0 + fracZ*(h1-h0) + fracX*(h2-h1);
		}
		else
		{
			//    <----Z---+
			//           0 | 
			//          /| |
			//         / | X
			//        /  | |
			//       /   | |
			//      2----1 |
			//             V
			const PxReal h1 = getHeight(vertexIndex + mData.columns);
			return h0 + fracX*(h1-h0) + fracZ*(h2-h1);
		}
	}
	else
	{
		//    <----Z---+
		//      +----+ | 
		//      |\   | |
		//      | \  | X
		//      |  \ | |
		//      |   \| |
		//      +----+ |
		//             V
		const PxReal h2 = getHeight(vertexIndex + mData.columns);
		const PxReal h1 = getHeight(vertexIndex + 1);
		if (fracX + fracZ < 1.0f)
		{
			//    <----Z---+
			//      1----0 | 
			//       \   | |
			//        \  | X
			//         \ | |
			//          \| |
			//           2 |
			//             V
			const PxReal h0 = getHeight(vertexIndex);
			return h0 + fracZ*(h1-h0) + fracX*(h2-h0);
		}
		else
		{
			//    <----Z---+
			//      1      | 
			//      |\     |
			//      | \    X
			//      |  \   |
			//      |   \  |
			//      0----2 |
			//             V
			//
			// Note that we need to flip fracX and fracZ since we are moving the origin
			const PxReal h0 = getHeight(vertexIndex + mData.columns + 1);
			return h0 + (1.0f - fracZ)*(h2-h0) + (1.0f - fracX)*(h1-h0);
		}
	}
}

PX_INLINE PxVec3 Gu::HeightField::getNormal_2(PxU32 vertexIndex, PxReal fracX, PxReal fracZ, PxReal xcoeff, PxReal ycoeff, PxReal zcoeff) const
{
	PxVec3 normal;
	if (isZerothVertexShared(vertexIndex))
	{
		//    <----Z---+
		//      +----+ | 
		//      |   /| |
		//      |  / | X
		//      | /  | |
		//      |/   | |
		//      +----+ |
		//             V
//		const PxReal h0 = getHeight(vertexIndex);
//		const PxReal h2 = getHeight(vertexIndex + mData.columns + 1);
		const PxI32 ih0 = getSample(vertexIndex).height;
		const PxI32 ih2 = getSample(vertexIndex + mData.columns + 1).height;
		if (fracZ >= fracX)
		{
			//    <----Z---+
			//      1----0 | 
			//      |   /  |
			//      |  /   X
			//      | /    |
			//      |/     |
			//      2      |
			//             V
//			const PxReal h0 = getHeight(vertexIndex);
//			const PxReal h1 = getHeight(vertexIndex + 1);
//			const PxReal h2 = getHeight(vertexIndex + mData.columns + 1);
//			normal.set(-(h2-h1), 1.0f, -(h1-h0));
			const PxI32 ih1 = getSample(vertexIndex + 1).height;
			normal = PxVec3(PxReal(ih1 - ih2)*xcoeff, ycoeff, PxReal(ih0 - ih1)*zcoeff);
		}
		else
		{
			//    <----Z---+
			//           0 | 
			//          /| |
			//         / | X
			//        /  | |
			//       /   | |
			//      2----1 |
			//             V
//			const PxReal h0 = getHeight(vertexIndex);
//			const PxReal h1 = getHeight(vertexIndex + mData.columns);
//			const PxReal h2 = getHeight(vertexIndex + mData.columns + 1);
//			normal.set(-(h1-h0), 1.0f, -(h2-h1));
			const PxI32 ih1 = getSample(vertexIndex + mData.columns).height;
			normal = PxVec3(PxReal(ih0 - ih1)*xcoeff, ycoeff, PxReal(ih1 - ih2)*zcoeff);
		}
	}
	else
	{
		//    <----Z---+
		//      +----+ | 
		//      |\   | |
		//      | \  | X
		//      |  \ | |
		//      |   \| |
		//      +----+ |
		//             V
		const PxI32 ih1 = getSample(vertexIndex + 1).height;
		const PxI32 ih2 = getSample(vertexIndex + mData.columns).height;
		if (fracX + fracZ <= PxReal(1))
		{
			//    <----Z---+
			//      1----0 | 
			//       \   | |
			//        \  | X
			//         \ | |
			//          \| |
			//           2 |
			//             V
//			const PxReal h0 = getHeight(vertexIndex);
//			const PxReal h1 = getHeight(vertexIndex + 1);
//			const PxReal h2 = getHeight(vertexIndex + mData.columns);
//			normal.set(-(h2-h0), 1.0f, -(h1-h0));
			const PxI32 ih0 = getSample(vertexIndex).height;
//			const PxI32 ih1 = getSample(vertexIndex + 1).height;
//			const PxI32 ih2 = getSample(vertexIndex + mData.columns).height;
			normal = PxVec3(PxReal(ih0 - ih2)*xcoeff, ycoeff, PxReal(ih0 - ih1)*zcoeff);
		}
		else
		{
			//    <----Z---+
			//      2      | 
			//      |\     |
			//      | \    X
			//      |  \   |
			//      |   \  |
			//      0----1 |
			//             V
			//
			// Note that we need to flip fracX and fracZ since we are moving the origin
//			const PxReal h2 = getHeight(vertexIndex + 1);
//			const PxReal h1 = getHeight(vertexIndex + mData.columns);
//			const PxReal h0 = getHeight(vertexIndex + mData.columns + 1);
//			normal.set(-(h0-h2), 1.0f, -(h0-h1));
//			const PxI32 ih2 = getSample(vertexIndex + 1).height;
//			const PxI32 ih1 = getSample(vertexIndex + mData.columns).height;
			const PxI32 ih0 = getSample(vertexIndex + mData.columns + 1).height;
//			normal.set(PxReal(ih2 - ih0), 1.0f, PxReal(ih1b - ih0));
			normal = PxVec3(PxReal(ih1 - ih0)*xcoeff, ycoeff, PxReal(ih2 - ih0)*zcoeff);
		}
	}
	return (mData.thickness <= 0.0f) ? normal : -normal;
}

PX_INLINE PxU32 Gu::HeightField::getTriangleIndex2(PxU32 cell, PxReal fracX, PxReal fracZ) const
{
	if (isZerothVertexShared(cell))
		return (fracZ > fracX) ? (cell << 1) + 1 : (cell << 1);
	else
		return (fracX + fracZ > 1) ? (cell << 1) + 1 : (cell << 1);
}

PX_INLINE PxU32 Gu::HeightField::getTriangleIndex(PxReal x, PxReal z) const
{
	PxReal fracX, fracZ;
	const PxU32 cell = computeCellCoordinates(x, z, fracX, fracZ);

	return getTriangleIndex2(cell, fracX, fracZ);
}

/**
Although inefficient, this is used for particles and deformables (PxcHeightFieldAabbTest.h).
*/
PX_FORCE_INLINE void Gu::HeightField::getTriangleVertices(PxU32 triangleIndex, PxU32 row, PxU32 column, PxVec3& v0, PxVec3& v1, PxVec3& v2) const
{
	PxU32 cell = triangleIndex >> 1;
	PX_ASSERT(row * getNbColumnsFast() + column == cell);

	PxReal h0 = getHeight(cell);
	PxReal h1 = getHeight(cell + 1);
	PxReal h2 = getHeight(cell + getNbColumnsFast());
	PxReal h3 = getHeight(cell + getNbColumnsFast() + 1);

	if (isFirstTriangle(triangleIndex))
	{
		if (isZerothVertexShared(cell))
		{
			//      <---- COL  
			//              1 R
			//             /| O
			//            / | W
			//           /  | |
			//          / 0 | |
			//         2----0 V
			//
			v0 = PxVec3((PxReal)(row + 1),	h2,	(PxReal)(column    ));
			v1 = PxVec3((PxReal)(row    ),	h0,	(PxReal)(column    ));
			v2 = PxVec3((PxReal)(row + 1),	h3,	(PxReal)(column + 1));
		}
		else
		{
			//      <---- COL  
			//         1----0 R
			//          \ 0 | O
			//           \  | W
			//            \ | |
			//             \| |
			//              2 V
			//
			v0 = PxVec3((PxReal)(row    ),	h0,	(PxReal)(column    ));
			v1 = PxVec3((PxReal)(row    ),	h1,	(PxReal)(column + 1));
			v2 = PxVec3((PxReal)(row + 1),	h2,	(PxReal)(column    ));
		}
	}
	else
	{
		if (isZerothVertexShared(cell))
		{
			//      <---- COL  
			//      0----2    R
			//      | 1 /     O
			//      |  /      W
			//      | /       |
			//      |/        |
			//      1         V
			//
			v0 = PxVec3((PxReal)(row    ),	h1, (PxReal)(column + 1));
			v1 = PxVec3((PxReal)(row + 1),	h3, (PxReal)(column + 1));
			v2 = PxVec3((PxReal)(row    ),	h0,	(PxReal)(column    ));
		}
		else
		{
			//      <---- COL  
			//      2         R
			//      |\        O
			//      | \       W
			//      |  \      |
			//      | 1 \     |
			//      0----1    V
			//
			v0 = PxVec3((PxReal)(row + 1),	h3,	(PxReal)(column + 1));
			v1 = PxVec3((PxReal)(row + 1),	h2,	(PxReal)(column    ));
			v2 = PxVec3((PxReal)(row    ),	h1,	(PxReal)(column + 1));
		}
	}
}

struct EdgeData
{
	PxU32	edgeIndex;
	PxU32	cell;
	PxU32	row;
	PxU32	column;
};
PxU32 getVertexEdgeIndices(const Gu::HeightField& heightfield, PxU32 vertexIndex, PxU32 row, PxU32 column, EdgeData edgeIndices[8]);
PxU32 getEdgeTriangleIndices(const Gu::HeightField& heightfield, const EdgeData& edgeData, PxU32* PX_RESTRICT triangleIndices);

}

#endif
