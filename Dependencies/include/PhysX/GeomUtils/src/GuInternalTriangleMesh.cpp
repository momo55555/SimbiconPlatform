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
#include "GuInternalTriangleMesh.h"
#include "IceSupport.h"

#include "GuAdjacencies.h"
#include "GuEdgeList.h"

#include "GuHillClimbing.h"
#include "ConvexHull.h"
#include "PsFoundation.h"
#include "PxStream.h"
#include "CmSerialAlignment.h"

using namespace physx;
using namespace Ice;

// This is horrible. Really need to make cooking stay self contained or link
// to libraries it needs instead of this conditionally compiled chaos. -jd
#if !(defined(PX_COOKING) && defined(PX_PHYSX_STATIC_LIB))

// PX_SERIALIZATION
/*BEGIN_FIELDS(InternalTriangleMesh)
	DEFINE_DYNAMIC_ARRAY(InternalTriangleMesh, mData.mVertices, PxField::eVEC3, mData.mNumVertices, Ps::F_SERIALIZE),
	DEFINE_DYNAMIC_ARRAY(InternalTriangleMesh, mData.mConvexParts, PxField::eWORD, mData.mNumTriangles, Ps::F_SERIALIZE),
	DEFINE_DYNAMIC_ARRAY(InternalTriangleMesh, mData.mExtraTrigData, PxField::eBYTE, mData.mNumTriangles, Ps::F_SERIALIZE),
	DEFINE_DYNAMIC_ARRAY(InternalTriangleMesh, mFaceRemap, PxField::eDWORD, mData.mNumTriangles, Ps::F_SERIALIZE),
	DEFINE_DYNAMIC_ARRAY(InternalTriangleMesh, mMaterialIndices, PxField::eWORD, mData.mNumTriangles, Ps::F_SERIALIZE),
END_FIELDS(InternalTriangleMesh)*/
//~PX_SERIALIZATION

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

InternalTriangleMesh::InternalTriangleMesh() :
	mAdjacencies			(NULL),
	mConvexEdgeThreshold	(0.001f),
	mOwnsMemory				(1),
	mMaterialIndices		(NULL),
	mFaceRemap				(NULL)
{
//	Ps::memSet(&mData, 0, sizeof(Gu::InternalTriangleMeshData));	// PT: "evil", zeroes v-tables
	mData.mNumVertices		= 0;
	mData.mNumTriangles		= 0;
	mData.mVertices			= NULL;
	mData.mTriangles		= NULL;
	mData.mExtraTrigData	= NULL;
	mData.m16BitIndices		= false;
	mData.mOpcodeModel.Release();
	mData.mAABB.minimum		= PxVec3(0);
	mData.mAABB.maximum		= PxVec3(0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

InternalTriangleMesh::~InternalTriangleMesh()
{
	release();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PX_SERIALIZATION
void InternalTriangleMesh::exportExtraData(PxSerialStream& stream)
{
	mData.mOpcodeModel.exportExtraData(stream);

//DEFINE_DYNAMIC_ARRAY(InternalTriangleMesh, mData.mVertices, PxField::eVEC3, mData.mNumVertices, Ps::F_SERIALIZE),
if(mData.mVertices)
{
//	Cm::alignStream(stream, PX_SERIAL_DEFAULT_ALIGN_EXTRA_DATA);
	stream.storeBuffer(mData.mVertices, mData.mNumVertices * sizeof(PxVec3));
}

	const PxU32 triangleSize = mData.m16BitIndices ? sizeof(PxU16) : sizeof(PxU32);
//	Cm::alignStream(stream, PX_SERIAL_DEFAULT_ALIGN_EXTRA_DATA);
	stream.storeBuffer(mData.mTriangles, mData.mNumTriangles * 3 * triangleSize);

//DEFINE_DYNAMIC_ARRAY(InternalTriangleMesh, mData.mExtraTrigData, PxField::eBYTE, mData.mNumTriangles, Ps::F_SERIALIZE),
if(mData.mExtraTrigData)
{
//	Cm::alignStream(stream, PX_SERIAL_DEFAULT_ALIGN_EXTRA_DATA);
	stream.storeBuffer(mData.mExtraTrigData, mData.mNumTriangles * sizeof(PxU8));
}

//DEFINE_DYNAMIC_ARRAY(InternalTriangleMesh, mMaterialIndices, PxField::eWORD, mData.mNumTriangles, Ps::F_SERIALIZE),
if(mMaterialIndices)
{
//	Cm::alignStream(stream, PX_SERIAL_DEFAULT_ALIGN_EXTRA_DATA);
	stream.storeBuffer(mMaterialIndices, mData.mNumTriangles * sizeof(PxU16));
}
//DEFINE_DYNAMIC_ARRAY(InternalTriangleMesh, mFaceRemap, PxField::eDWORD, mData.mNumTriangles, Ps::F_SERIALIZE),
if(mFaceRemap)
{
//	Cm::alignStream(stream, PX_SERIAL_DEFAULT_ALIGN_EXTRA_DATA);
	stream.storeBuffer(mFaceRemap, mData.mNumTriangles * sizeof(PxU32));
}

//	mData.mOpcodeModel.exportExtraData(stream);
}

char* InternalTriangleMesh::importExtraData(char* address, PxU32& totalPadding)
{
	address = mData.mOpcodeModel.importExtraData(address, totalPadding);

//DEFINE_DYNAMIC_ARRAY(InternalTriangleMesh, mData.mVertices, PxField::eVEC3, mData.mNumVertices, Ps::F_SERIALIZE),
if(mData.mVertices)
{
//	address = Cm::alignStream(address, totalPadding, PX_SERIAL_DEFAULT_ALIGN_EXTRA_DATA);
	mData.mVertices = reinterpret_cast<PxVec3*>(address);
	address += mData.mNumVertices * sizeof(PxVec3);
}

//	address = Cm::alignStream(address, totalPadding, PX_SERIAL_DEFAULT_ALIGN_EXTRA_DATA);
	mData.mTriangles = address;
	const PxU32 triangleSize = mData.m16BitIndices ? sizeof(PxU16) : sizeof(PxU32);
	address += mData.mNumTriangles * 3 * triangleSize;

//DEFINE_DYNAMIC_ARRAY(InternalTriangleMesh, mData.mExtraTrigData, PxField::eBYTE, mData.mNumTriangles, Ps::F_SERIALIZE),
if(mData.mExtraTrigData)
{
//	address = Cm::alignStream(address, totalPadding, PX_SERIAL_DEFAULT_ALIGN_EXTRA_DATA);
	mData.mExtraTrigData = reinterpret_cast<PxU8*>(address);
	address += mData.mNumTriangles * sizeof(PxU8);
}

//DEFINE_DYNAMIC_ARRAY(InternalTriangleMesh, mMaterialIndices, PxField::eWORD, mData.mNumTriangles, Ps::F_SERIALIZE),
if(mMaterialIndices)
{
//	address = Cm::alignStream(address, totalPadding, PX_SERIAL_DEFAULT_ALIGN_EXTRA_DATA);
	mMaterialIndices = reinterpret_cast<PxU16*>(address);
	address += mData.mNumTriangles * sizeof(PxU16);
}

//DEFINE_DYNAMIC_ARRAY(InternalTriangleMesh, faceRemap, PxField::eDWORD, mData.mNumTriangles, Ps::F_SERIALIZE),
if(mFaceRemap)
{
//	address = Cm::alignStream(address, totalPadding, PX_SERIAL_DEFAULT_ALIGN_EXTRA_DATA);
	mFaceRemap = reinterpret_cast<PxU32*>(address);
	address += mData.mNumTriangles * sizeof(PxU32);
}

//	address = mData.mOpcodeModel.importExtraData(address, totalPadding);

	mData.mOpcodeModel.SetMeshInterface(&mMeshInterface);
	setupMeshInterface();

	return address;
}

//~PX_SERIALIZATION

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void InternalTriangleMesh::release()
{
// PX_SERIALIZATION
	if(mOwnsMemory)
//~PX_SERIALIZATION
	{
		if(size_t(mAdjacencies)!=1)
			PX_DELETE_AND_RESET(mAdjacencies);

		PX_FREE_AND_RESET(mData.mExtraTrigData);

		mData.mOpcodeModel.Release();

		PX_FREE_AND_RESET(mFaceRemap);
		PX_FREE_AND_RESET(mMaterialIndices);
		PX_FREE_AND_RESET(mData.mTriangles);
		PX_FREE_AND_RESET(mData.mVertices);
	}
	mAdjacencies = NULL;
	mData.mExtraTrigData = NULL;
	mFaceRemap = NULL;
	mMaterialIndices = NULL;
	mData.mTriangles = NULL;
	mData.mVertices = NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void InternalTriangleMesh::createAdjacencies()
{
	ADJACENCIESCREATE create;
	create.NbFaces	= getNumTriangles();

	if (has16BitIndices())
	{
		create.WFaces	= (const PxU16*)getTriangles();
		create.DFaces	= NULL;
	}
	else
	{
		create.WFaces	= NULL;
		create.DFaces	= (const PxU32*)getTriangles();
	}

	create.Verts	= getVertices();

	mAdjacencies = PX_NEW(AdjacenciesBuilder);
	if(!mAdjacencies->Init(create))
		PX_DELETE_AND_RESET(mAdjacencies);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxVec3* InternalTriangleMesh::allocateVertices(PxU32 nbVertices)
{
	mData.mNumVertices = nbVertices;
	PX_ASSERT(!mData.mVertices);
	mData.mVertices = (PxVec3*)PX_ALLOC(mData.mNumVertices * sizeof(PxVec3));
	return mData.mVertices;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void* InternalTriangleMesh::allocateTriangles(PxU32 nbTriangles, bool force32Bit)
{
	if(mData.mNumVertices == 0)		//please specify vertex count first, so we can size the index list properly
		return NULL;

	mData.mNumTriangles = nbTriangles;
	PX_ASSERT(!mData.mTriangles);

	if(mData.mNumVertices <= 0xffff && !force32Bit)
	{
		mData.mTriangles = PX_NEW(PxU16)[mData.mNumTriangles * 3];
		mData.m16BitIndices = true;
	}
	else
	{
		mData.mTriangles = PX_NEW(PxU32)[mData.mNumTriangles * 3];
		mData.m16BitIndices = false;
	}

	return mData.mTriangles;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxU16* InternalTriangleMesh::allocateMaterials()
{
	if(!mData.mNumTriangles)	
		return NULL;
	PX_ASSERT(!mMaterialIndices);
	mMaterialIndices = PX_NEW(PxU16)[mData.mNumTriangles];
	return mMaterialIndices;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxU32* InternalTriangleMesh::allocateFaceRemap()
{
	if(!mData.mNumTriangles)	return NULL;
	PX_ASSERT(!mFaceRemap);
	mFaceRemap = PX_NEW(PxU32)[mData.mNumTriangles];
	return mFaceRemap;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void InternalTriangleMesh::setupMeshInterface()
{
	mMeshInterface.SetNbVertices (getNumVertices());
	mMeshInterface.SetNbTriangles(getNumTriangles());
	mMeshInterface.SetPointers	(getTriangles(), has16BitIndices(), getVertices());
}

bool InternalTriangleMesh::loadOpcodeModel(const PxStream& modelData, const PxU32 meshVersion)
{
	mData.mOpcodeModel.Release();

	//create the meshInterface:
	setupMeshInterface();

	if (meshVersion <= 5)
	{
		Ps::getFoundation().error(
			PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__,
			"Obsolete cooked mesh found. Mesh version has been updated, please recook your meshes.");
		PX_ASSERT(0 && "Obsolete cooked mesh found. Mesh version has been updated, please recook your meshes.");
		return false;
	}

	if(!mData.mOpcodeModel.Build(mMeshInterface, modelData))
	{
		Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Opcode is not OK.");
		return false;
	}

	if(!mData.mOpcodeModel.mRTree.load(modelData))
	{
		Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "RTree binary image load error.");
		return false;
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif // #if !(defined(PX_COOKING) && defined(PX_PHYSX_STATIC_LIB))



