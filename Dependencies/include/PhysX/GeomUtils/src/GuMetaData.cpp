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

#include "GuHeightField.h"
#include "GuConvexMeshData.h"
#include "BigConvexData.h"
#include "GuConvexMesh.h"
#include "OPC_MeshInterface.h"
#include "OPC_HybridModel.h"
#include "GuTriangleMesh.h"
#include "GuGeometryUnion.h"
#include "CmSerialAlignment.h"

using namespace physx;
using namespace Ps;
using namespace Cm;
using namespace Gu;
using namespace Ice;

#if defined PX_WINDOWS || defined PX_X360
	#pragma warning(disable:4101)	// SIGH... stupid compiler flags
#endif

///////////////////////////////////////////////////////////////////////////////

static void getMetaData_Valency(PxSerialStream& stream)
{
// 4 bytes
	DEFINE_MD_CLASS(Valency)
	DEFINE_MD_ITEM(Valency, PxU16,	mCount,		0)
	DEFINE_MD_ITEM(Valency, PxU16,	mOffset,	0)
}

static void getMetaData_BigConvexRawData(PxSerialStream& stream)
{
// 24 bytes
	DEFINE_MD_CLASS(BigConvexRawData)
	DEFINE_MD_ITEM(BigConvexRawData, PxU16,			mSubdiv,			0)
	DEFINE_MD_ITEM(BigConvexRawData, PxU16,			mNbSamples,			0)
	DEFINE_MD_ITEM(BigConvexRawData, PxU8,			mSamples,			MdFlags::ePTR)
	DEFINE_MD_ITEM(BigConvexRawData, PxU32,			mNbVerts,			0)
	DEFINE_MD_ITEM(BigConvexRawData, PxU32,			mNbAdjVerts,		0)
	DEFINE_MD_ITEM(BigConvexRawData, Valency,		mValencies,			MdFlags::ePTR)
	DEFINE_MD_ITEM(BigConvexRawData, PxU8,			mAdjacentVerts,		MdFlags::ePTR)
}

void BigConvexData::getMetaData(PxSerialStream& stream)
{
	getMetaData_Valency(stream);
	getMetaData_BigConvexRawData(stream);

// 28 bytes
	DEFINE_MD_CLASS(BigConvexData)
	DEFINE_MD_ITEM(BigConvexData, BigConvexRawData,	mData,		0)
	DEFINE_MD_ITEM(BigConvexData, void,				mVBuffer,	MdFlags::ePTR)

	//------ Extra-data ------

	// PT: can't use one array of PxU16 since we don't want to flip those
//	DEFINE_MD_EXTRA_DATA_ARRAY(BigConvexData, PxU8, mData.mNbSamples, PX_SERIAL_DEFAULT_ALIGN_EXTRA_DATA, 0)
	DEFINE_MD_EXTRA_DATA_ARRAY(BigConvexData, PxU8, mData.mNbSamples, 0, 0)
	DEFINE_MD_EXTRA_DATA_ARRAY(BigConvexData, PxU8, mData.mNbSamples, 0, 0)

//	DEFINE_MD_EXTRA_DATA_ARRAY(BigConvexData, Valency, mData.mNbVerts, PX_SERIAL_DEFAULT_ALIGN_EXTRA_DATA, 0)
	DEFINE_MD_EXTRA_DATA_ARRAY(BigConvexData, Valency, mData.mNbVerts, 0, 0)
	DEFINE_MD_EXTRA_DATA_ARRAY(BigConvexData, PxU8, mData.mNbAdjVerts, 0, 0)
}

static void getMetaData_InternalObjectsData(PxSerialStream& stream)
{
// 16 bytes
	DEFINE_MD_CLASS(InternalObjectsData)
	DEFINE_MD_ITEM(InternalObjectsData,		PxReal,	mRadius,	0)
#ifdef DEFINE_MD_ITEMS2
	DEFINE_MD_ITEMS2(InternalObjectsData,	PxReal,	mExtents,	0)
#else
	DEFINE_MD_ITEMS(InternalObjectsData,	PxReal,	mExtents,	0, 3)
#endif
}

static void getMetaData_HullPolygonData(PxSerialStream& stream)
{
// 20 bytes
	DEFINE_MD_CLASS(HullPolygonData)
#ifdef DEFINE_MD_ITEMS2
	DEFINE_MD_ITEMS2(HullPolygonData,	PxReal,	mPlane,		0)
#else
	DEFINE_MD_ITEMS(HullPolygonData,	PxReal,	mPlane,		0, 4)
#endif
	DEFINE_MD_ITEM(HullPolygonData,		PxU16,	mVRef8,		0)
	DEFINE_MD_ITEM(HullPolygonData,		PxU8,	mNbVerts,	0)
	DEFINE_MD_ITEM(HullPolygonData,		PxU8,	mMinIndex,	0)
}

static void getMetaData_ConvexHullData(PxSerialStream& stream)
{
// 64 bytes
	DEFINE_MD_CLASS(ConvexHullData)
	DEFINE_MD_ITEM(ConvexHullData, PxBounds3,			mAABB,				0)
	DEFINE_MD_ITEM(ConvexHullData, PxVec3,				mCenterOfMass,		0)
	DEFINE_MD_ITEM(ConvexHullData, HullPolygonData,		mPolygons,			MdFlags::ePTR)
	DEFINE_MD_ITEM(ConvexHullData, BigConvexRawData,	mBigConvexRawData,	MdFlags::ePTR)
	DEFINE_MD_ITEM(ConvexHullData, PxU16,				mNbEdges,			0)
	DEFINE_MD_ITEM(ConvexHullData, PxU8,				mNbHullVertices,	0)
	DEFINE_MD_ITEM(ConvexHullData, PxU8,				mNbPolygons,		0)
	DEFINE_MD_ITEM(ConvexHullData, InternalObjectsData,	mInternal,			0)
}

void Gu::ConvexMesh::getMetaData(PxSerialStream& stream)
{
	getMetaData_InternalObjectsData(stream);
	getMetaData_HullPolygonData(stream);
	getMetaData_ConvexHullData(stream);
	BigConvexData::getMetaData(stream);

// 136 bytes
	DEFINE_MD_VCLASS(ConvexMesh)
	DEFINE_MD_BASE_CLASS(ConvexMesh, PxSerializable)
	DEFINE_MD_BASE_CLASS(ConvexMesh, RefCountable)

	//
	DEFINE_MD_ITEM(ConvexMesh, ConvexHullData,	chull,			0)
	DEFINE_MD_ITEM(ConvexMesh, PxU32,			mNb,			0)
	DEFINE_MD_ITEM(ConvexMesh, BigConvexData,	bigConvexData,	MdFlags::ePTR)
	DEFINE_MD_ITEM(ConvexMesh, PxReal,			mass,			0)
	DEFINE_MD_ITEM(ConvexMesh, PxMat33,			inertia,		0)
	DEFINE_MD_ITEM(ConvexMesh, GuMeshFactory,	mMeshFactory,	MdFlags::ePTR)

	//------ Extra-data ------

/*
void Gu::ConvexMesh::exportExtraData(PxSerialStream& stream)
{
	stream.storeBuffer(chull.mPolygons, getBufferSize());

	if(bigConvexData)
	{
		stream.storeBuffer(bigConvexData, sizeof(BigConvexData));

		bigConvexData->exportExtraData(stream);
	}
}
*/
	DEFINE_MD_EXTRA_DATA_ARRAY(Gu::ConvexMesh, HullPolygonData, chull.mNbPolygons, 4, 0)
//	DEFINE_MD_EXTRA_DATA_ARRAY(Gu::ConvexMesh, HullPolygonData, chull.mNbPolygons, PX_SERIAL_DEFAULT_ALIGN_EXTRA_DATA, 0)

	DEFINE_MD_EXTRA_DATA_ARRAY(Gu::ConvexMesh, PxVec3, chull.mNbHullVertices, 0, 0)
	DEFINE_MD_EXTRA_DATA_ARRAY(Gu::ConvexMesh, PxU8, chull.mNbEdges, 0, 0)
	DEFINE_MD_EXTRA_DATA_ARRAY(Gu::ConvexMesh, PxU8, chull.mNbEdges, 0, 0)
	DEFINE_MD_EXTRA_DATA_ARRAY(Gu::ConvexMesh, PxU8, mNb, 0, MdFlags::eMASK_MSB_COUNT)

	DEFINE_MD_EXTRA_DATA_ITEM(Gu::ConvexMesh, BigConvexData, bigConvexData)
}

///////////////////////////////////////////////////////////////////////////////

static void getMetaData_PxHeightFieldSample(PxSerialStream& stream)
{
	DEFINE_MD_CLASS(PxHeightFieldSample)
	DEFINE_MD_ITEM(PxHeightFieldSample, PxI16,			height,			0)
	DEFINE_MD_ITEM(PxHeightFieldSample, PxBitAndByte,	materialIndex0,	0)
	DEFINE_MD_ITEM(PxHeightFieldSample, PxBitAndByte,	materialIndex1,	0)
	DEFINE_MD_TYPEDEF(PxBitAndByte, PxU8)
}

static void getMetaData_HeightFieldData(PxSerialStream& stream)
{
	DEFINE_MD_TYPEDEF(PxHeightFieldFlags, PxU16)
	DEFINE_MD_TYPEDEF(PxHeightFieldFormat::Enum, PxU32)

	DEFINE_MD_CLASS(HeightFieldData)
	DEFINE_MD_ITEM(HeightFieldData, PxU32,						rows,					0)
	DEFINE_MD_ITEM(HeightFieldData, PxU32,						columns,				0)
	DEFINE_MD_ITEM(HeightFieldData, PxReal,						rowLimit,				0)
	DEFINE_MD_ITEM(HeightFieldData, PxReal,						colLimit,				0)
	DEFINE_MD_ITEM(HeightFieldData, PxReal,						nbColumns,				0)
	DEFINE_MD_ITEM(HeightFieldData, PxHeightFieldSample,		samples,				MdFlags::ePTR)
	DEFINE_MD_ITEM(HeightFieldData, PxReal,						thickness,				0)
	DEFINE_MD_ITEM(HeightFieldData, PxReal,						convexEdgeThreshold,	0)
	DEFINE_MD_ITEM(HeightFieldData, PxHeightFieldFlags,			flags,					0)
#ifdef EXPLICIT_PADDING_METADATA
	DEFINE_MD_ITEM(HeightFieldData, PxU16,						paddAfterFlags,			MdFlags::ePADDING)
#endif
	DEFINE_MD_ITEM(HeightFieldData, PxHeightFieldFormat::Enum,	format,					0)
	DEFINE_MD_ITEM(HeightFieldData, PxBounds3,					mAABB,					0)
//#ifdef HF_TILED_MEMORY_LAYOUT
	DEFINE_MD_ITEM(HeightFieldData, PxU32,						rowsPadded,				0)
	DEFINE_MD_ITEM(HeightFieldData, PxU32,						columnsPadded,			0)
	DEFINE_MD_ITEM(HeightFieldData, PxU32,						tilesU,					0)
	DEFINE_MD_ITEM(HeightFieldData, PxU32,						tilesV,					0)
//#endif
}

void Gu::HeightField::getMetaData(PxSerialStream& stream)
{
	getMetaData_PxHeightFieldSample(stream);
	getMetaData_HeightFieldData(stream);

	DEFINE_MD_TYPEDEF(PxMaterialTableIndex, PxU16)

	DEFINE_MD_VCLASS(HeightField)
	DEFINE_MD_BASE_CLASS(HeightField, PxSerializable)
	DEFINE_MD_BASE_CLASS(HeightField, RefCountable)

#ifdef HF_USE_PRECOMPUTED_BITMAP
	#ifndef HF_USE_RESERVED_BIT
	DEFINE_MD_ITEM(HeightField, BitMap,				mBitmap,				0)
	#endif
#endif
#ifdef HF_USE_HEIGHT_CACHE
	assert(0);
#endif
	DEFINE_MD_ITEM(HeightField, HeightFieldData,		mData,					0)
	DEFINE_MD_ITEM(HeightField, PxU32,					mSampleStride,			0)
	DEFINE_MD_ITEM(HeightField, PxU32,					mNbSamples,				0)
	DEFINE_MD_ITEM(HeightField, PxReal,					minHeight,				0)
	DEFINE_MD_ITEM(HeightField, PxReal,					maxHeight,				0)
	DEFINE_MD_ITEM(HeightField, PxMaterialTableIndex,	commonMaterialIndex0,	0)
	DEFINE_MD_ITEM(HeightField, PxMaterialTableIndex,	commonMaterialIndex1,	0)
	DEFINE_MD_ITEM(HeightField, GuMeshFactory,			mMeshFactory,			MdFlags::ePTR)

	//------ Extra-data ------

#ifdef HF_TILED_MEMORY_LAYOUT
	DEFINE_MD_EXTRA_DATA_ARRAY(HeightField, PxHeightFieldSample, mNbSamples, 16, 0)	// PT: ### try to remove mNbSamples later
#else
	DEFINE_MD_EXTRA_DATA_ARRAY(HeightField, PxHeightFieldSample, mNbSamples, 0, 0)	// PT: ### try to remove mNbSamples later
#endif
}

///////////////////////////////////////////////////////////////////////////////

static void getMetaData_LeafTriangles(PxSerialStream& stream)
{
	DEFINE_MD_CLASS(LeafTriangles)
	DEFINE_MD_ITEM(LeafTriangles, PxU32,	Data,	0)
}

void MeshInterface::getMetaData(PxSerialStream& stream)
{
	DEFINE_MD_CLASS(MeshInterface)

	DEFINE_MD_ITEM(MeshInterface, RemapCallback,	mRemapCallback,		MdFlags::ePTR)
	DEFINE_MD_ITEM(MeshInterface, void,				mRemapUserData,		MdFlags::ePTR)
	DEFINE_MD_ITEM(MeshInterface, PxU32,			mNbTris,			0)
	DEFINE_MD_ITEM(MeshInterface, PxU32,			mNbVerts,			0)
	DEFINE_MD_ITEM(MeshInterface, void,				mTris,				MdFlags::ePTR)
	DEFINE_MD_ITEM(MeshInterface, PxVec3,			mVerts,				MdFlags::ePTR)
	DEFINE_MD_ITEM(MeshInterface, PxU32,			mHas16BitIndices,	0)
}

void HybridModel::getMetaData(PxSerialStream& stream)
{
	getMetaData_LeafTriangles(stream);

// 128 bytes
	DEFINE_MD_CLASS(HybridModel)

	DEFINE_MD_ITEM(HybridModel, MeshInterface,	mIMesh,				MdFlags::ePTR)
	DEFINE_MD_ITEM(HybridModel, PxU32,			mModelCode,			0)
	DEFINE_MD_ITEM(HybridModel, AABBTree,		mSource,			MdFlags::ePTR)
	DEFINE_MD_ITEM(HybridModel, PxReal,			mGeomEpsilon,		0)
	DEFINE_MD_ITEM(HybridModel, RTree,			mRTree,				0)
	DEFINE_MD_ITEM(HybridModel, PxU32,			mNbLeaves,			0)
	DEFINE_MD_ITEM(HybridModel, LeafTriangles,	mTriangles,			MdFlags::ePTR)
	DEFINE_MD_ITEM(HybridModel, PxU32,			mNbPrimitives,		0)
	DEFINE_MD_ITEM(HybridModel, PxU32,			mIndices,			MdFlags::ePTR)

	//------ Extra-data ------

	DEFINE_MD_EXTRA_DATA_ARRAY(HybridModel, LeafTriangles, mNbLeaves, 0, MdFlags::eMASK_MSB_COUNT)
	DEFINE_MD_EXTRA_DATA_ITEMS(HybridModel, PxU32, mIndices, mNbPrimitives, 0)
}

static void getMetaData_RTreePage(PxSerialStream& stream)
{
	DEFINE_MD_CLASS(RTreePage)
	DEFINE_MD_ITEMS(RTreePage, PxU16,	minx,	0, 8)
	DEFINE_MD_ITEMS(RTreePage, PxU16,	miny,	0, 8)
	DEFINE_MD_ITEMS(RTreePage, PxU16,	minz,	0, 8)
	DEFINE_MD_ITEMS(RTreePage, PxU16,	maxx,	0, 8)
	DEFINE_MD_ITEMS(RTreePage, PxU16,	maxy,	0, 8)
	DEFINE_MD_ITEMS(RTreePage, PxU16,	maxz,	0, 8)
	DEFINE_MD_ITEMS(RTreePage, PxU32,	ptrs,	0, 8)
}

void RTree::getMetaData(PxSerialStream& stream)
{
	getMetaData_RTreePage(stream);

// 96 bytes
	DEFINE_MD_CLASS(RTree)

	DEFINE_MD_ITEM(RTree, PxVec4,		mBoundsMin,					0)
	DEFINE_MD_ITEM(RTree, PxVec4,		mBoundsMax,					0)
	DEFINE_MD_ITEM(RTree, PxVec4,		mInvDiagonal,				0)
	DEFINE_MD_ITEM(RTree, PxVec4,		mDiagonalScaler,			0)
	DEFINE_MD_ITEM(RTree, PxU32,		mPageSize,					0)
	DEFINE_MD_ITEM(RTree, PxU32,		mNumRootPages,				0)
	DEFINE_MD_ITEM(RTree, PxU32,		mNumLevels,					0)
	DEFINE_MD_ITEM(RTree, PxU32,		mTotalNodes,				0)
	DEFINE_MD_ITEM(RTree, PxU32,		mTotalPages,				0)
	DEFINE_MD_ITEM(RTree, PxU32,		mBottomLevelFirstNodeIndex,	0)
	DEFINE_MD_ITEM(RTree, PxU32,		mFlags,						0)
	DEFINE_MD_ITEM(RTree, RTreePage,	mPages,						MdFlags::ePTR)

	//------ Extra-data ------

	DEFINE_MD_EXTRA_DATA_ARRAY(RTree, RTreePage, mTotalPages, 128, 0)
}

///////////////////////////////////////////////////////////////////////////////

static void getMetaData_InternalTriangleMeshData(PxSerialStream& stream)
{
// 224 => 208 => 192 bytes
	DEFINE_MD_CLASS(InternalTriangleMeshData)

	DEFINE_MD_ITEM(InternalTriangleMeshData, PxU32,			mNumVertices,			0)
	DEFINE_MD_ITEM(InternalTriangleMeshData, PxU32,			mNumTriangles,			0)
	DEFINE_MD_ITEM(InternalTriangleMeshData, PxVec3,		mVertices,				MdFlags::ePTR)
	DEFINE_MD_ITEM(InternalTriangleMeshData, void,			mTriangles,				MdFlags::ePTR)
	DEFINE_MD_ITEM(InternalTriangleMeshData, PxU8,			mExtraTrigData,			MdFlags::ePTR)
	DEFINE_MD_ITEM(InternalTriangleMeshData, HybridModel,	mOpcodeModel,			0)
	DEFINE_MD_ITEM(InternalTriangleMeshData, PxBounds3,		mAABB,					0)
	DEFINE_MD_ITEM(InternalTriangleMeshData, bool,			m16BitIndices,			0)
	#ifdef EXPLICIT_PADDING_METADATA
	DEFINE_MD_ITEMS(InternalTriangleMeshData, bool,			mPaddingFromBool,		MdFlags::ePADDING, 3)
	#endif
	//------ Extra-data ------

//	DEFINE_MD_EXTRA_DATA_ARRAY(InternalTriangleMeshData, PxU8, mData.mNbSamples, 0, 0)
	DEFINE_MD_EXTRA_DATA_ITEMS(InternalTriangleMeshData, PxVec3, mVertices, mNumVertices, 0)

	// PT: oh dear... super tricky here
	DEFINE_MD_EXTRA_DATA_ITEMS(InternalTriangleMeshData, PxU16, m16BitIndices, mNumTriangles, 0)
	DEFINE_MD_EXTRA_DATA_ITEMS(InternalTriangleMeshData, PxU16, m16BitIndices, mNumTriangles, 0)
	DEFINE_MD_EXTRA_DATA_ITEMS(InternalTriangleMeshData, PxU16, m16BitIndices, mNumTriangles, 0)
	DEFINE_MD_EXTRA_DATA_ITEMS(InternalTriangleMeshData, PxU32, m16BitIndices, mNumTriangles, MdFlags::eFLIP_CONTROL)
	DEFINE_MD_EXTRA_DATA_ITEMS(InternalTriangleMeshData, PxU32, m16BitIndices, mNumTriangles, MdFlags::eFLIP_CONTROL)
	DEFINE_MD_EXTRA_DATA_ITEMS(InternalTriangleMeshData, PxU32, m16BitIndices, mNumTriangles, MdFlags::eFLIP_CONTROL)

	DEFINE_MD_EXTRA_DATA_ITEMS(InternalTriangleMeshData, PxU8, mExtraTrigData, mNumTriangles, 0)
}

void InternalTriangleMesh::getMetaData(PxSerialStream& stream)
{
	MeshInterface::getMetaData(stream);
	HybridModel::getMetaData(stream);
	RTree::getMetaData(stream);

	getMetaData_InternalTriangleMeshData(stream);

// 256 => 240 bytes
	DEFINE_MD_CLASS(InternalTriangleMesh)

	DEFINE_MD_ITEM(InternalTriangleMesh, InternalTriangleMeshData,	mData,					0)
	DEFINE_MD_ITEM(InternalTriangleMesh, PxU16,						mMaterialIndices,		MdFlags::ePTR)
	DEFINE_MD_ITEM(InternalTriangleMesh, PxU32,						mFaceRemap,				MdFlags::ePTR)
	DEFINE_MD_ITEM(InternalTriangleMesh, AdjacenciesBuilder,		mAdjacencies,			MdFlags::ePTR)
	DEFINE_MD_ITEM(InternalTriangleMesh, PxReal,					mConvexEdgeThreshold,	0)
	DEFINE_MD_ITEM(InternalTriangleMesh, PxU32,						mOwnsMemory,			0)
	DEFINE_MD_ITEM(InternalTriangleMesh, MeshInterface,				mMeshInterface,			0)

	//------ Extra-data ------

	DEFINE_MD_EXTRA_DATA_ITEMS(InternalTriangleMesh, PxU16, mMaterialIndices, mData.mNumTriangles, 0)
	DEFINE_MD_EXTRA_DATA_ITEMS(InternalTriangleMesh, PxU32, mFaceRemap, mData.mNumTriangles, 0)
}

void Gu::TriangleMesh::getMetaData(PxSerialStream& stream)
{
	InternalTriangleMesh::getMetaData(stream);

// 320 => 304 => 272 => 256 bytes
	DEFINE_MD_VCLASS(TriangleMesh)
	DEFINE_MD_BASE_CLASS(TriangleMesh, PxSerializable)
	DEFINE_MD_BASE_CLASS(TriangleMesh, RefCountable)

	DEFINE_MD_ITEM(TriangleMesh, InternalTriangleMesh,	mesh,						0)
	DEFINE_MD_ITEM(TriangleMesh, GuMeshFactory,			mMeshFactory,				MdFlags::ePTR)
#ifdef EXPLICIT_PADDING_METADATA
	DEFINE_MD_ITEMS2(TriangleMesh, PxU32,				mPaddingFromInternalMesh,	MdFlags::ePADDING)
#endif
}

///////////////////////////////////////////////////////////////////////////////

void Gu::GeometryUnion::getMetaData(PxSerialStream& stream)
{
	DEFINE_MD_TYPEDEF(PxGeometryType::Enum, PxU32)

	// SIGH. The various PxGeometry classes are all public, so I can't really put the meta-data function in there. And then
	// I can't access their protected members. So we use the same trick as for the ShapeContainer
	class ShadowConvexMeshGeometry : public PxConvexMeshGeometryLL
	{
	public:
		static void getMetaData(PxSerialStream& stream)
		{
			DEFINE_MD_CLASS(ShadowConvexMeshGeometry)
			DEFINE_MD_ITEM(ShadowConvexMeshGeometry, PxGeometryType::Enum,	mType,		0)
			DEFINE_MD_ITEM(ShadowConvexMeshGeometry, PxMeshScale,			scale,		0)
			DEFINE_MD_ITEM(ShadowConvexMeshGeometry, PxConvexMesh,			convexMesh,	MdFlags::ePTR)
			DEFINE_MD_ITEM(ShadowConvexMeshGeometry, ConvexHullData,		hullData,	MdFlags::ePTR)
		}
	};
	ShadowConvexMeshGeometry::getMetaData(stream);
	DEFINE_MD_TYPEDEF(PxConvexMeshGeometryLL, ShadowConvexMeshGeometry)

	/////////////////

	class ShadowTriangleMeshGeometry : public PxTriangleMeshGeometryLL
	{
	public:
		static void getMetaData(PxSerialStream& stream)
		{
			DEFINE_MD_TYPEDEF(PxMeshGeometryFlags, PxU8)

			DEFINE_MD_CLASS(ShadowTriangleMeshGeometry)
			DEFINE_MD_ITEM(ShadowTriangleMeshGeometry, PxGeometryType::Enum,		mType,				0)
			DEFINE_MD_ITEM(ShadowTriangleMeshGeometry, PxMeshScale,					scale,				0)
			DEFINE_MD_ITEM(ShadowTriangleMeshGeometry, PxMeshGeometryFlags,			meshFlags,			0)
			DEFINE_MD_ITEMS(ShadowTriangleMeshGeometry, PxU8,						paddingFromFlags,	MdFlags::ePADDING, 3)
			DEFINE_MD_ITEM(ShadowTriangleMeshGeometry, PxTriangleMesh,				triangleMesh,		MdFlags::ePTR)
			DEFINE_MD_ITEM(ShadowTriangleMeshGeometry, InternalTriangleMeshData,	meshData,			MdFlags::ePTR)
		}
	};
	ShadowTriangleMeshGeometry::getMetaData(stream);
	DEFINE_MD_TYPEDEF(PxTriangleMeshGeometryLL, ShadowTriangleMeshGeometry)

	/////////////////

	class ShadowHeightFieldGeometry : public PxHeightFieldGeometryLL
	{
	public:
		static void getMetaData(PxSerialStream& stream)
		{
			DEFINE_MD_CLASS(ShadowHeightFieldGeometry)
			DEFINE_MD_ITEM(ShadowHeightFieldGeometry, PxGeometryType::Enum,	mType,					0)
			DEFINE_MD_ITEM(ShadowHeightFieldGeometry, PxHeightField,		heightField,			MdFlags::ePTR)
			DEFINE_MD_ITEM(ShadowHeightFieldGeometry, PxReal,				heightScale,			0)
			DEFINE_MD_ITEM(ShadowHeightFieldGeometry, PxReal,				rowScale,				0)
			DEFINE_MD_ITEM(ShadowHeightFieldGeometry, PxReal,				columnScale,			0)
			DEFINE_MD_ITEM(ShadowHeightFieldGeometry, PxMeshGeometryFlags,	heightFieldFlags,		0)
#ifdef DEFINE_MD_ITEMS2
			DEFINE_MD_ITEMS2(ShadowHeightFieldGeometry, PxU8,				paddingFromFlags,		MdFlags::ePADDING)
#else
			DEFINE_MD_ITEMS(ShadowHeightFieldGeometry, PxU8,				paddingFromFlags,		MdFlags::ePADDING, 3)
#endif
			DEFINE_MD_ITEM(ShadowHeightFieldGeometry, HeightField,			heightFieldData,		MdFlags::ePTR)
		}
	};
	ShadowHeightFieldGeometry::getMetaData(stream);
	DEFINE_MD_TYPEDEF(PxHeightFieldGeometryLL, ShadowHeightFieldGeometry)

	/////////////////

	class ShadowPlaneGeometry : public PxPlaneGeometry
	{
	public:
		static void getMetaData(PxSerialStream& stream)
		{
			DEFINE_MD_CLASS(ShadowPlaneGeometry)
			DEFINE_MD_ITEM(ShadowPlaneGeometry, PxGeometryType::Enum,	mType,		0)
		}
	};
	ShadowPlaneGeometry::getMetaData(stream);
	DEFINE_MD_TYPEDEF(PxPlaneGeometry, ShadowPlaneGeometry)

	/////////////////

	class ShadowSphereGeometry : public PxSphereGeometry
	{
	public:
		static void getMetaData(PxSerialStream& stream)
		{
			DEFINE_MD_CLASS(ShadowSphereGeometry)
			DEFINE_MD_ITEM(ShadowSphereGeometry, PxGeometryType::Enum,	mType,		0)
			DEFINE_MD_ITEM(ShadowSphereGeometry, PxReal,				radius,		0)
		}
	};
	ShadowSphereGeometry::getMetaData(stream);
	DEFINE_MD_TYPEDEF(PxSphereGeometry, ShadowSphereGeometry)

	/////////////////

	class ShadowCapsuleGeometry : public PxCapsuleGeometry
	{
	public:
		static void getMetaData(PxSerialStream& stream)
		{
			DEFINE_MD_CLASS(ShadowCapsuleGeometry)
			DEFINE_MD_ITEM(ShadowCapsuleGeometry, PxGeometryType::Enum,	mType,		0)
			DEFINE_MD_ITEM(ShadowCapsuleGeometry, PxReal,				radius,		0)
			DEFINE_MD_ITEM(ShadowCapsuleGeometry, PxReal,				halfHeight,	0)
		}
	};
	ShadowCapsuleGeometry::getMetaData(stream);
	DEFINE_MD_TYPEDEF(PxCapsuleGeometry, ShadowCapsuleGeometry)

	/////////////////

	class ShadowBoxGeometry : public PxBoxGeometry
	{
	public:
		static void getMetaData(PxSerialStream& stream)
		{
			DEFINE_MD_CLASS(ShadowBoxGeometry)
			DEFINE_MD_ITEM(ShadowBoxGeometry, PxGeometryType::Enum,	mType,		0)
			DEFINE_MD_ITEM(ShadowBoxGeometry, PxVec3,				halfExtents,0)
		}
	};
	ShadowBoxGeometry::getMetaData(stream);
	DEFINE_MD_TYPEDEF(PxBoxGeometry, ShadowBoxGeometry)

	/*
	- geom union offset & size
	- control type offset & size
	- type-to-class mapping
	*/

// 44 bytes
	DEFINE_MD_CLASS(Gu::GeometryUnion)

	DEFINE_MD_UNION(Gu::GeometryUnion, geometry)
	DEFINE_MD_UNION_TYPE(Gu::GeometryUnion, PxSphereGeometry,			PxGeometryType::eSPHERE)
	DEFINE_MD_UNION_TYPE(Gu::GeometryUnion, PxPlaneGeometry,			PxGeometryType::ePLANE)
	DEFINE_MD_UNION_TYPE(Gu::GeometryUnion, PxCapsuleGeometry,			PxGeometryType::eCAPSULE)
	DEFINE_MD_UNION_TYPE(Gu::GeometryUnion, PxBoxGeometry,				PxGeometryType::eBOX)
	DEFINE_MD_UNION_TYPE(Gu::GeometryUnion, PxConvexMeshGeometryLL,		PxGeometryType::eCONVEXMESH)
	DEFINE_MD_UNION_TYPE(Gu::GeometryUnion, PxTriangleMeshGeometryLL,	PxGeometryType::eTRIANGLEMESH)
	DEFINE_MD_UNION_TYPE(Gu::GeometryUnion, PxHeightFieldGeometryLL,	PxGeometryType::eHEIGHTFIELD)
}
