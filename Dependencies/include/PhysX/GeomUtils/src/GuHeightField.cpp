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


#include "PsIntrinsics.h"
#include "GuHeightField.h"
#include "PsAlloca.h"
#include "PsUtilities.h"
#include "GuMeshFactory.h"
#include "PsAlignedMalloc.h"

using namespace physx;

#ifdef __SPU__
namespace physx
{
	CellHeightfieldTileCache g_sampleCache __attribute__((aligned(16)));
	unsigned char g_HFSampleBuffer[32] __attribute__((aligned(16)));
}
#endif


#ifndef __SPU__

// PX_SERIALIZATION
#include "PxStream.h"
#include "CmSerialFramework.h"
#include "CmSerialAlignment.h"
BEGIN_FIELDS(Gu::HeightField)
END_FIELDS(Gu::HeightField)
//~PX_SERIALIZATION

static const PxU32 MAX_MATERIALS = 128; // seven bit materials


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Gu::HeightField::HeightField(GuMeshFactory& meshFactory) :
	mSampleStride			(0),
	mNbSamples				(0),
	minHeight				(0.0f),
	maxHeight				(0.0f),
	commonMaterialIndex0	(0xFFFF),
	commonMaterialIndex1	(0xFFFF),
	mMeshFactory			(&meshFactory)
{
	mData.format				= PxHeightFieldFormat::eS16_TM;
	mData.rows					= 0;
	mData.columns				= 0;
	mData.convexEdgeThreshold	= 0;
	mData.flags					= PxHeightFieldFlags();
	mData.samples				= NULL;
	mData.thickness				= 0;

// PX_SERIALIZATION
	setType(PxSerialType::eHEIGHTFIELD);
//~PX_SERIALIZATION
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Gu::HeightField::~HeightField()
{
	releaseMemory();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PX_SERIALIZATION
void Gu::HeightField::onRefCountZero()
{
	if(mMeshFactory->removeHeightField(*this))
		return deleteSerializedObject(this);
	
	// PT: if we reach this point, we didn't find the mesh in the Physics object => don't delete!
	// This prevents deleting the object twice.
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Gu::HeightField::onRefCountZero: double deletion detected!");
}

void Gu::HeightField::exportExtraData(PxSerialStream& stream)
{
	// PT: warning, order matters for the converter. Needs to export the base stuff first
#ifdef HF_USE_PRECOMPUTED_BITMAP
	#ifndef HF_USE_RESERVED_BIT
	mBitmap.exportExtraData(stream);
	#endif
#endif

#ifdef HF_TILED_MEMORY_LAYOUT
	const PxU32 size = mData.rowsPadded * mData.columnsPadded * sizeof(PxHeightFieldSample);
	Cm::alignStream(stream, 16);
#else
	const PxU32 size = mData.rows * mData.columns * sizeof(PxHeightFieldSample);
#endif
	stream.storeBuffer(mData.samples, size);
}

char* Gu::HeightField::importExtraData(char* address, PxU32& totalPadding)
{
#ifdef HF_USE_PRECOMPUTED_BITMAP
	#ifndef HF_USE_RESERVED_BIT
	address = mBitmap.importExtraData(address, totalPadding);
	#endif
#endif

#ifdef HF_TILED_MEMORY_LAYOUT
	address = Cm::alignStream(address, totalPadding, 16);
	mData.samples = (PxHeightFieldSample*)address;
	address += mData.rowsPadded * mData.columnsPadded * sizeof(PxHeightFieldSample);
#else
	mData.samples = (PxHeightFieldSample*)address;
	address += mData.rows * mData.columns * sizeof(PxHeightFieldSample);
#endif
	return address;
}

//~PX_SERIALIZATION

void Gu::HeightField::release()
{
	decRefCount();
}

PxU32 Gu::HeightField::getReferenceCount() const
{
	return getRefCount();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CA: Convert to tiled memory layout on PS3
static PxHeightFieldSample* convertToTiled(const PxHeightFieldSample* src, PxU32 nbColumns, PxU32 nbRows, PxU32 columnsPadded, PxU32 rowsPadded, PxU32 tilesU, PxU32 tilesV)
{
	// PT: WARNING: if you change this code, the equivalent code must be changed in ConvX!

	// ### serial alignment
	PxHeightFieldSample* samplesTiled = (PxHeightFieldSample*)Ps::AlignedAllocator<16>().allocate(columnsPadded*rowsPadded*sizeof(PxHeightFieldSample), __FILE__, __LINE__);

	const PxHeightFieldSample emptySample = {0,0,0};
	for (PxU32 i=0;i<tilesV;i++)
	{
		for (PxU32 j=0;j<tilesU;j++)
		{
			const PxU32 tileBase = (i*tilesU+j)*HF_TILE_SIZE_V*HF_TILE_SIZE_U;

			for (PxU32 v=0;v<HF_TILE_SIZE_V;v++)
			{
				for (PxU32 u=0;u<HF_TILE_SIZE_U;u++)
				{
					const PxU32 tileOffset = tileBase+v*HF_TILE_SIZE_U+u;

					if ((j < tilesU-1 || u < HF_TILE_SIZE_U-(columnsPadded - nbColumns)) &&
						(i < tilesV-1 || v < HF_TILE_SIZE_V-(rowsPadded - nbRows)))
					{
						const PxU32 offset2 = (i*HF_TILE_SIZE_V+v)*nbColumns+(j*HF_TILE_SIZE_U+u);
							
						PX_ASSERT(tileOffset<columnsPadded*rowsPadded);
						PX_ASSERT(offset2<nbColumns*nbRows);
						samplesTiled[tileOffset] = src[offset2];
					}
					else
					{
						PX_ASSERT(tileOffset<columnsPadded*rowsPadded);
						samplesTiled[tileOffset] = emptySample;
					}
				}
			}
		}
	}

	// CA: Check if mapping is correct
	/*for (PxU32 i=0;i<desc.nbRows;i++) {
		for (PxU32 j=0;j<desc.nbColumns;j++) {
			PxU32 offset = ((i/HF_TILE_SIZE_V)*mData.tilesU+(j/HF_TILE_SIZE_U))*HF_TILE_SIZE_V*HF_TILE_SIZE_U+(i&(PxU32)(HF_TILE_SIZE_V-1))*HF_TILE_SIZE_U+(j&(PxU32)(HF_TILE_SIZE_U-1));
			PxHeightFieldSample newS = samplesTiled[offset];
			PxHeightFieldSample oldS = mData.samples[i*desc.nbColumns+j];
			if (newS.height != oldS.height || newS.materialIndex0 != oldS.materialIndex0 || newS.materialIndex1 != oldS.materialIndex1 || newS.tessFlag != oldS.tessFlag ) {
				assert(false);
			}
		}
	}*/
	return samplesTiled;
}

bool Gu::HeightField::loadFromDesc(const PxHeightFieldDesc& desc)
{
	// verify descriptor
	PX_CHECK_AND_RETURN_NULL(desc.isValid(), "Gu::HeightField::load: desc.isValid() failed!");

	// release old memory
	releaseMemory();

	// copy trivial data
	mData.format				= desc.format;
	mData.rows					= desc.nbRows;
	mData.columns				= desc.nbColumns;
	mData.thickness				= desc.thickness;
	mData.convexEdgeThreshold	= desc.convexEdgeThreshold;
	mData.flags					= desc.flags;
	mSampleStride				= desc.samples.stride;

	// PT: precompute some data - mainly for Xbox
	mData.rowLimit				= float(mData.rows - 2);
	mData.colLimit				= float(mData.columns - 2);
	mData.nbColumns				= float(desc.nbColumns);

//#ifdef HF_TILED_MEMORY_LAYOUT
	// CA: tiled memory layout on PS3
	mData.columnsPadded	= (((PxU32)(desc.nbColumns)+(PxU32)(HF_TILE_SIZE_U-1))&((PxU32)~(HF_TILE_SIZE_U-1)));
	mData.rowsPadded	= (((PxU32)(desc.nbRows)+(PxU32)(HF_TILE_SIZE_V-1))&((PxU32)~(HF_TILE_SIZE_V-1)));
	mData.tilesU		= mData.columnsPadded/HF_TILE_SIZE_U;
	mData.tilesV		= mData.rowsPadded/HF_TILE_SIZE_V;
//#endif

	// allocate and copy height samples
	mData.samples = NULL;
	const PxU32 nbVerts = desc.nbRows * desc.nbColumns;
	PxU32 n = nbVerts * sizeof(PxHeightFieldSample);
	if (n > 0) 
	{
		mData.samples = (PxHeightFieldSample*)PX_ALLOC(n);
		if (mData.samples == NULL)
		{
			Ps::getFoundation().error(PxErrorCode::eOUT_OF_MEMORY, __FILE__, __LINE__, "Gu::HeightField::load: PX_ALLOC failed!");
			return false;
		}
		const PxU8* PX_RESTRICT src = (const PxU8* PX_RESTRICT)desc.samples.data;
		PxHeightFieldSample* PX_RESTRICT dst = mData.samples;
		for(PxU32 i=0;i<nbVerts;i++)
		{
			*dst++ = *(const PxHeightFieldSample*)src;
			src += desc.samples.stride;
		}

		// Enumerate the materials to see if we only have two.
		// Haven't benchmarked, but the "two pass" - "brute force" should be fastest.
		// 1) Mark each referenced material in one unconditional sweep.
		// 2) Then do the checks on the array of 128 (=MAX_MATERIALS) reference flags.

		PX_ALLOCA(nbMaterials, PxU8, MAX_MATERIALS);
		Ps::memSet(nbMaterials, 0, MAX_MATERIALS*sizeof(PxU8));

		int offset = 0;
		for(PxU32 row=0; row<mData.rows-1; row++)
		{
			for(PxU32 column=0; column<desc.nbColumns-1; column++)
			{
//				const PxHeightFieldSample& sample = getSample(offset++);
				const PxHeightFieldSample& sample = mData.samples[offset++];	// PT: don't use "getSample" here, we're not tiled yet
				nbMaterials[sample.materialIndex0] = 1;
				nbMaterials[sample.materialIndex1] = 1;
			}
			// skip the unused cell belonging to the last column vertex
			offset++;
		}

		int i = -1;
		while(++i < MAX_MATERIALS) if (nbMaterials[i])
		{
			// found first material
			commonMaterialIndex0 = PxMaterialTableIndex(i); 
			break;
		}
		while(++i < MAX_MATERIALS) if (nbMaterials[i])
		{ 
			// found second material
			commonMaterialIndex1 = PxMaterialTableIndex(i);
			break;
		}
		while(++i < MAX_MATERIALS) if (nbMaterials[i]) 
		{
			// oops..  found third material -> clear them both!
			commonMaterialIndex0 = 0xFFFF;
			commonMaterialIndex1 = 0xFFFF;
			break;
		}
	}

#ifdef HF_TILED_MEMORY_LAYOUT
	PxHeightFieldSample* samplesTiled = convertToTiled(mData.samples, mData.columns, mData.rows, mData.columnsPadded, mData.rowsPadded, mData.tilesU, mData.tilesV);
	PX_FREE_AND_RESET(mData.samples);
	mData.samples = samplesTiled;
#endif

	// compute extents
	minHeight = PX_MAX_REAL;
	maxHeight = -PX_MAX_REAL;
	for(PxU32 offset = 0; offset < nbVerts; offset++) 
	{
		const PxReal h = getHeight(offset);
		if (h < minHeight) minHeight = h;
		if (h > maxHeight) maxHeight = h;
	}

#ifdef HF_USE_PRECOMPUTED_BITMAP
	#ifndef HF_USE_RESERVED_BIT
	mBitmap.clear(nbVerts);
	#endif
	for(PxU32 i=0;i<nbVerts;i++)
	{
		const PxU32 row = i / getNbColumnsFast();
		const PxU32 column = i % getNbColumnsFast();

		if(isCollisionVertexPreca(i, row, column, PxHeightFieldMaterial::eHOLE))
		{
		#ifdef HF_USE_RESERVED_BIT
			mData.samples[i].materialIndex1.setBit();
		#else
			mBitmap.set(i);
		#endif
		}
		#ifdef HF_USE_RESERVED_BIT
		else
		{
			mData.samples[i].materialIndex1.clearBit();
		}
		#endif
	}
#endif

// PT: "mNbSamples" only used by binary converter
#ifdef HF_TILED_MEMORY_LAYOUT
	mNbSamples	= mData.rowsPadded * mData.columnsPadded;
#else
	mNbSamples	= mData.rows * mData.columns;
#endif

#ifdef HF_USE_HEIGHT_CACHE
	for(PxI32 i=0;i<32;i++)
	{
		mCache[i].vertexIndex = 0xffffffff;
	}
#endif

	//Compute local space aabb.
	PxBounds3 bounds;
	bounds.minimum.y = getMinHeight();
	bounds.maximum.y = getMaxHeight();
	if (getThicknessFast() < 0) 
		bounds.minimum.y += getThicknessFast();
	else if (getThicknessFast() > 0) 
		bounds.maximum.y += getThicknessFast();
	bounds.minimum.x = 0;
	bounds.maximum.x = PxReal(getNbRowsFast() - 1);
	bounds.minimum.z = 0;
	bounds.maximum.z = PxReal(getNbColumnsFast() - 1);
	mData.mAABB=bounds;

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxU32 Gu::HeightField::saveCells(void* destBuffer, PxU32 destBufferSize) const
{
	PxU32 n = mData.columns * mData.rows * sizeof(PxHeightFieldSample);

#ifdef HF_TILED_MEMORY_LAYOUT
	if (n <= destBufferSize)
	{
		PxHeightFieldSample* destSampleBuffer = (PxHeightFieldSample*)destBuffer;
		for (PxU32 i=0;i<mData.rows;i++)
		{
			for (PxU32 j=0;j<mData.columns;j++)
			{
				PxU32 offset = ((i/HF_TILE_SIZE_V)*mData.tilesU+(j/HF_TILE_SIZE_U))*HF_TILE_SIZE_V*HF_TILE_SIZE_U+(i&(PxU32)(HF_TILE_SIZE_V-1))*HF_TILE_SIZE_U+(j&(PxU32)(HF_TILE_SIZE_U-1));
				destSampleBuffer[i*mData.columns+j] = mData.samples[offset];
			}
		}
	}
	else
		return 0;
#else
	//PxU32 n = mData.columns * mData.rows * sizeof(PxHeightFieldSample);
	if (n > destBufferSize) n = destBufferSize;
	Ps::memCopy(destBuffer, mData.samples, n);
#endif

	return n;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Gu::HeightField::releaseMemory()
{
// PX_SERIALIZATION
	if(ownsMemory())
//~PX_SERIALIZATION
	{
#ifdef HF_TILED_MEMORY_LAYOUT
		Ps::AlignedAllocator<16>().deallocate(mData.samples);
		mData.samples = NULL;
#else
		PX_FREE_AND_RESET(mData.samples);
#endif
	}
}

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PT: TODO: use those faster functions everywhere
namespace physx
{

/*static PX_FORCE_INLINE*/ PxU32 getVertexEdgeIndices(const Gu::HeightField& heightfield, PxU32 vertexIndex, PxU32 row, PxU32 column, EdgeData edgeIndices[8])
{
	const PxU32 nbColumns = heightfield.getData().columns;
	const PxU32 nbRows = heightfield.getData().rows;
	PX_ASSERT((vertexIndex / nbColumns)==row);
	PX_ASSERT((vertexIndex % nbColumns)==column);

	PxU32 count = 0;
	
	if (row > 0) 
	{
//		edgeIndices[count++] = 3 * (vertexIndex - nbColumns) + 2;
		const PxU32 cell = vertexIndex - nbColumns;
		edgeIndices[count].edgeIndex	= 3 * cell + 2;
		edgeIndices[count].cell			= cell;
		edgeIndices[count].row			= row-1;
		edgeIndices[count].column		= column;
		count++;
	}
	
	if (column < nbColumns-1)
	{
		if (row > 0)
		{
			if (!heightfield.isZerothVertexShared(vertexIndex - nbColumns))
			{
//				edgeIndices[count++] = 3 * (vertexIndex - nbColumns) + 1;
				const PxU32 cell = vertexIndex - nbColumns;
				edgeIndices[count].edgeIndex	= 3 * cell + 1;
				edgeIndices[count].cell			= cell;
				edgeIndices[count].row			= row-1;
				edgeIndices[count].column		= column;
				count++;
			}
		}
//		edgeIndices[count++] = 3 * vertexIndex;
		edgeIndices[count].edgeIndex	= 3 * vertexIndex;
		edgeIndices[count].cell			= vertexIndex;
		edgeIndices[count].row			= row;
		edgeIndices[count].column		= column;
		count++;

		if (row < nbRows - 1)
		{
			if (heightfield.isZerothVertexShared(vertexIndex))
			{
//				edgeIndices[count++] = 3 * vertexIndex + 1;
				edgeIndices[count].edgeIndex	= 3 * vertexIndex + 1;
				edgeIndices[count].cell			= vertexIndex;
				edgeIndices[count].row			= row;
				edgeIndices[count].column		= column;
				count++;
			}
		}
	}

	if (row < nbRows - 1)
	{
//		edgeIndices[count++] = 3 * vertexIndex + 2;
		edgeIndices[count].edgeIndex	= 3 * vertexIndex + 2;
		edgeIndices[count].cell			= vertexIndex;
		edgeIndices[count].row			= row;
		edgeIndices[count].column		= column;
		count++;
	}

	if (column > 0)
	{
		if (row < nbRows - 1)
		{
			if (!heightfield.isZerothVertexShared(vertexIndex - 1))
			{
//				edgeIndices[count++] = 3 * (vertexIndex - 1) + 1;
				const PxU32 cell = vertexIndex - 1;
				edgeIndices[count].edgeIndex	= 3 * cell + 1;
				edgeIndices[count].cell			= cell;
				edgeIndices[count].row			= row;
				edgeIndices[count].column		= column-1;
				count++;
			}
		}
//		edgeIndices[count++] = 3 * (vertexIndex - 1);
		const PxU32 cell = vertexIndex - 1;
		edgeIndices[count].edgeIndex	= 3 * cell;
		edgeIndices[count].cell			= cell;
		edgeIndices[count].row			= row;
		edgeIndices[count].column		= column-1;
		count++;
		if (row > 0)
		{
			if (heightfield.isZerothVertexShared(vertexIndex - nbColumns - 1))
			{
//				edgeIndices[count++] = 3 * (vertexIndex - nbColumns - 1) + 1;
				const PxU32 cell = vertexIndex - nbColumns - 1;
				edgeIndices[count].edgeIndex	= 3 * cell + 1;
				edgeIndices[count].cell			= cell;
				edgeIndices[count].row			= row-1;
				edgeIndices[count].column		= column-1;
				count++;
			}
		}
	}
	return count;
}

/*static PX_FORCE_INLINE*/ PxU32 getEdgeTriangleIndices(const Gu::HeightField& heightfield, const EdgeData& edgeData, PxU32* PX_RESTRICT triangleIndices)
{
	const PxU32 nbColumns = heightfield.getData().columns;
	const PxU32 nbRows = heightfield.getData().rows;

	const PxU32 edgeIndex	= edgeData.edgeIndex;
	const PxU32 cell		= edgeData.cell;
	const PxU32 row			= edgeData.row;
	const PxU32 column		= edgeData.column;
	PX_ASSERT(cell==edgeIndex / 3);
	PX_ASSERT(row==cell / nbColumns);
	PX_ASSERT(column==cell % nbColumns);
	PxU32 count = 0;
//	PxU32 faceID0, faceID1;
//	switch (edgeIndex % 3)
	switch (edgeIndex - cell*3)
	{
		case 0:
			if (column < nbColumns - 1)
			{
				if (row > 0)
				{
					if (heightfield.isZerothVertexShared(cell - nbColumns))
						triangleIndices[count++] = ((cell - nbColumns) << 1);
//						faceID0 = ((cell - nbColumns) << 1);
					else 
						triangleIndices[count++] = ((cell - nbColumns) << 1) + 1;
//						faceID0 = ((cell - nbColumns) << 1) + 1;

//					count++;
				}
				if (row < nbRows - 1)
				{
					if (heightfield.isZerothVertexShared(cell))
						triangleIndices[count++] = (cell << 1) + 1;
//						faceID1 = (cell << 1) + 1;
					else 
						triangleIndices[count++] = cell << 1;
//						faceID1 = cell << 1;
//					count++;
				}
			}
			break;
		case 1:
			if ((row < nbRows - 1) && (column < nbColumns - 1))
			{
				triangleIndices[count++] = cell << 1;
				triangleIndices[count++] = (cell << 1) + 1;
//				faceID0 = cell << 1;
//				faceID1 = (cell << 1) + 1;
//				count = 2;
			}
			break;
		case 2:
			if (row < nbRows - 1)
			{
				if (column > 0)
				{
//					triangleIndices[count++] = ((cell - 1) << 1) + 1;
					triangleIndices[count++] = ((cell - 1) << 1) + 1;
				}
				if (column < nbColumns - 1)
				{
					triangleIndices[count++] = cell << 1;
				}
			}
			break;
	}

/*
		if(faceCounts[i] > 1) 
		{
			// ptchernev TODO: this is a bit arbitrary
			if (getTriangleMaterial(currentfaceIndices[0]) != holeMaterialIndex) 
			{
				nbSolid = true;
				if (getTriangleMaterial(currentfaceIndices[1]) == holeMaterialIndex)
					return true;
			}
			if (getTriangleMaterial(currentfaceIndices[1]) != holeMaterialIndex) 
			{
				nbSolid = true;
				if (getTriangleMaterial(currentfaceIndices[0]) == holeMaterialIndex)
					return true;
			}
		} 
		else 
		{
			if (getTriangleMaterial(currentfaceIndices[0]) != holeMaterialIndex) 
				return true;
		}*/


	return count;
}

}

#ifdef HF_USE_PRECOMPUTED_BITMAP
bool Gu::HeightField::isCollisionVertexPreca(PxU32 vertexIndex, PxU32 row, PxU32 column, PxU16 holeMaterialIndex) const
#else
bool Gu::HeightField::isCollisionVertex(PxU32 vertexIndex, PxU32 row, PxU32 column, PxU16 holeMaterialIndex) const
#endif
{
#ifdef PX_HEIGHTFIELD_DEBUG
	PX_ASSERT(isValidVertex(vertexIndex));
#endif
	PX_ASSERT((vertexIndex / getNbColumnsFast()) == row);
	PX_ASSERT((vertexIndex % getNbColumnsFast()) == column);
	// check boundary conditions
	if(mData.flags & PxHeightFieldFlag::eNO_BOUNDARY_EDGES) 
	{
		if ((row < 1) || (column < 1) || (row > mData.rows-2) || (column > mData.columns-2))
		{
			return false;
		}
	}


//	bool nbSolid;
//	if(testHoleMaterial(vertexIndex, row, column, holeMaterialIndex, nbSolid))
//		return true;


	// check if solid and boundary
//	PxU32 edgeIndices[8];
	EdgeData edgeIndices[8];
//	const PxU32 edgeCount = getVertexEdgeIndices(vertexIndex, edgeIndices);
	const PxU32 edgeCount = ::getVertexEdgeIndices(*this, vertexIndex, row, column, edgeIndices);

	PxU32 faceCounts[8];
	PxU32 faceIndices[2*8];
	PxU32* PX_RESTRICT dst = faceIndices;
	for(PxU32 i=0; i<edgeCount; i++)
	{
//		PxU32 faceIndices[2];
//		const PxU32 faceCount = getEdgeTriangleIndices(edgeIndices[i], faceIndices);
		faceCounts[i] = ::getEdgeTriangleIndices(*this, edgeIndices[i], dst);
		dst += 2;
	}

	bool nbSolid = false;
	const PxU32* PX_RESTRICT currentfaceIndices = faceIndices;
	for(PxU32 i=0; i<edgeCount; i++)
	{
		if(faceCounts[i] > 1) 
		{
			// ptchernev TODO: this is a bit arbitrary
			if (getTriangleMaterial(currentfaceIndices[0]) != holeMaterialIndex) 
			{
				nbSolid = true;
				if (getTriangleMaterial(currentfaceIndices[1]) == holeMaterialIndex)
					return true;
			}
			if (getTriangleMaterial(currentfaceIndices[1]) != holeMaterialIndex) 
			{
				nbSolid = true;
				if (getTriangleMaterial(currentfaceIndices[0]) == holeMaterialIndex)
					return true;
			}
		} 
		else 
		{
			if (getTriangleMaterial(currentfaceIndices[0]) != holeMaterialIndex) 
				return true;
		}
		currentfaceIndices += 2;
	}

	// return true if it is boundary or solid and convex
	return (nbSolid && isConvexVertex(vertexIndex, row, column));
}


/*struct int64
{
	int a,b;
};*/

#ifdef REMOVED
// PT: special version computing vertex index directly
PxU32 Gu::HeightField::computeCellCoordinates(PxReal x, PxReal z, PxU32 nbColumns, PxReal& fracX, PxReal& fracZ) const
{
	x = physx::intrinsics::selectMax(x, 0.0f);
	z = physx::intrinsics::selectMax(z, 0.0f);

	PxU32 row = (PxU32)x;
	PxU32 column = (PxU32)z;

/*int64	tmp_x, tmp_z;
_asm	lwz		r11, x
_asm	lfs		fr0, 0(r11)
_asm	fctiwz	fr13, fr0
_asm	stfd	fr13, tmp_x

_asm	lwz		r11, z
_asm	lfs		fr0, 0(r11)
_asm	fctiwz	fr13, fr0
_asm	stfd	fr13, tmp_z

PxU32 row = tmp_x.b;
PX_ASSERT(row==PxU32(x));*/
	if (row > mData.rows - 2) 
	{
		row = mData.rows - 2;
		fracX = PxReal(1);
	}
	else
	{
		fracX = x - PxReal(row);
	}

//PxU32 column = tmp_z.b;
//PX_ASSERT(column==PxU32(z));

	if (column > mData.columns - 2) 
	{
		column = mData.columns - 2;
		fracZ = PxReal(1);
	}
	else
	{
		fracZ = z - PxReal(column);
	}
	const PxU32 vertexIndex = row * nbColumns + column;

	return vertexIndex;
}
#endif

// AP scaffold: this naming is confusing and inconsistent with return value. the function appears to compute vertex coord rather than cell coords
// it would most likely be better to stay in cell coords instead, since fractional vertex coords just do not make any sense
PxU32 Gu::HeightField::computeCellCoordinates(PxReal x, PxReal z,
											  PxReal& fracX, PxReal& fracZ) const
{
	namespace i = physx::intrinsics;

	x = i::selectMax(x, 0.0f);
	z = i::selectMax(z, 0.0f);
	PxF32 x1 = i::selectMin(x, mData.rowLimit+0.999999f); // 64.999999 for 65x65
	PxF32 z1 = i::selectMin(z, mData.colLimit+0.999999f);
	x = PxFloor(x1);
	fracX = x1 - PxFloor(x1);
	z = PxFloor(z1);
	fracZ = z1 - PxFloor(z1);
	PX_ASSERT(x >= 0.0f && x < PxF32(mData.rows));
	PX_ASSERT(z >= 0.0f && z < PxF32(mData.columns));

	const PxU32 vertexIndex = PxU32(x * (mData.nbColumns) + z);
	PX_ASSERT(vertexIndex < (mData.rows)*(mData.columns));

	return vertexIndex;
}

PxReal Gu::HeightField::computeExtreme(PxU32 minRow, PxU32 maxRow, PxU32 minColumn, PxU32 maxColumn) const
{
	const bool thicknessNegOrNull = (getThicknessFast() <= 0.0f);

//	PxReal hfExtreme = thicknessNegOrNull ? -PX_MAX_REAL : PX_MAX_REAL;
	PxI32 hfExtreme = thicknessNegOrNull ? PX_MIN_I32 : PX_MAX_I32;

/*	for(PxU32 row = minRow; row <= maxRow; row++)
	{
		for(PxU32 column = minColumn; column <= maxColumn; column++)
		{
			const PxReal h = getHeight(row * getNbColumnsFast() + column);
			hfExtreme = thicknessNegOrNull ? PxMax(hfExtreme, h) : PxMin(hfExtreme, h);
		}
	}*/

	if(thicknessNegOrNull)
	{
		for(PxU32 row = minRow; row <= maxRow; row++)
		{
			for(PxU32 column = minColumn; column <= maxColumn; column++)
			{
//				const PxReal h = getHeight(row * getNbColumnsFast() + column);
				const PxI32 h = getSample(row * getNbColumnsFast() + column).height;
				hfExtreme = PxMax(hfExtreme, h);
			}
		}
	}
	else
	{
		for(PxU32 row = minRow; row <= maxRow; row++)
		{
			for(PxU32 column = minColumn; column <= maxColumn; column++)
			{
//				const PxReal h = getHeight(row * getNbColumnsFast() + column);
				const PxI32 h = getSample(row * getNbColumnsFast() + column).height;
				hfExtreme = PxMin(hfExtreme, h);
			}
		}
	}

//	return hfExtreme;
	return PxReal(hfExtreme);
}
