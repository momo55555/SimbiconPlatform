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


#include "PxVisualizationParameter.h"
#include "PsIntrinsics.h"
#include "CmPhysXCommon.h"
#include "CmRenderOutput.h"
#include "PsMathUtils.h"
#include "GuConvexMesh.h"
#include "GuTriangle32.h"
#include "BigConvexData.h"
#include "Serialize.h"
#include "./Ice/IceSerialize.h"
#include "GuMeshFactory.h"

using namespace physx;
using namespace Ice;

// PX_SERIALIZATION
#include "CmSerialFramework.h"
#include "CmSerialAlignment.h"
BEGIN_FIELDS(Gu::ConvexMesh)
END_FIELDS(Gu::ConvexMesh)
//~PX_SERIALIZATION

bool Gu::ConvexMesh::getPolygonData(PxU32 i, PxHullPolygon& data) const
{
	if(i>=chull.mNbPolygons)
		return false;

	const HullPolygonData& poly = chull.mPolygons[i];
	data.mPlane[0]		= poly.mPlane.normal.x;
	data.mPlane[1]		= poly.mPlane.normal.y;
	data.mPlane[2]		= poly.mPlane.normal.z;
	data.mPlane[3]		= poly.mPlane.d;
	data.mNbVerts	= poly.mNbVerts;
	data.mIndexBase	= poly.mVRef8;
	return true;
}

/// ======================================


Gu::ConvexMesh::ConvexMesh() :
	bigConvexData	(NULL),
	mass			(0),
	inertia			(PxMat33::createIdentity()),
	mNb				(0)
{
	// the mesh is created with a reference count of 1 and the Np class immediately takes ownership
	// so we don't need to increment the ref count here
// PX_SERIALIZATION
	setType(PxSerialType::eCONVEX_MESH);
//~PX_SERIALIZATION

	initConvexHullData(chull);
}


Gu::ConvexMesh::~ConvexMesh()
{
// PX_SERIALIZATION
	if(ownsMemory())
//~PX_SERIALIZATION
	{
		PX_DELETE_POD(chull.mPolygons);
		PX_DELETE_AND_RESET(bigConvexData);
	}
}

// PX_SERIALIZATION
void Gu::ConvexMesh::exportExtraData(PxSerialStream& stream)
{
	Cm::alignStream(stream, 4);
	const PxU32 bufferSize = computeBufferSize(chull, getNb());
	stream.storeBuffer(chull.mPolygons, bufferSize);

	if(bigConvexData)
	{
		stream.storeBuffer(bigConvexData, sizeof(BigConvexData));

		bigConvexData->exportExtraData(stream);
	}
}

char* Gu::ConvexMesh::importExtraData(char* address, PxU32& totalPadding)
{
	address = Cm::alignStream(address, totalPadding, 4);
	chull.mPolygons = (Gu::HullPolygonData*)address;
	const PxU32 bufferSize = computeBufferSize(chull, getNb());
	address += bufferSize;

	if(bigConvexData)
	{
		bigConvexData = (BigConvexData*)address;
		address += sizeof(BigConvexData);

		Cm::RefResolver resolver;
		new (bigConvexData) BigConvexData(resolver);

		address = bigConvexData->importExtraData(address, totalPadding);

		chull.mBigConvexRawData = &bigConvexData->mData;
	}

	return address;
}

namespace
{
	/* function not used, suppress warning

	void ReadNormals(PxU32 nb, PxVec3* normals, bool mismatch, const PxStream& stream)
	{
		PxU16* tmp = (PxU16*)PxAlloca(sizeof(PxU16)*nb);
		ReadWordBuffer(tmp, nb, mismatch, stream);

		static PxVec3 Table[32*32];
		static bool initDone = false;
		if(!initDone)
		{
			initDone = true;
			const int NORM_NBNORMALS = (1<<5);
			const float Coeff = PxPi / float((NORM_NBNORMALS-1) * 4);

			for(int j=0;j<32;j++)
			{
				const float b = float(j) * Coeff;
				const float CosTheta = PxCos(b);
				const float SinTheta = PxSin(b);

				for(int i=0;i<32;i++)
				{
					const float a = float(i) * Coeff;
					const float CosPhi = PxCos(a);
					const float SinPhi = PxSin(a);

					float Sorted[3];
					Sorted[0] = CosTheta * CosPhi;
					Sorted[1] = SinPhi;
					Sorted[2] = SinTheta * CosPhi;

					for(int k=0;k<2;k++)
					{
						if(Sorted[0]>Sorted[1])
						{
							float tmp = Sorted[1];
							Sorted[1] = Sorted[0];
							Sorted[0] = tmp;
						}
						if(Sorted[1]>Sorted[2])
						{
							float tmp = Sorted[2];
							Sorted[2] = Sorted[1];
							Sorted[1] = tmp;
						}
					}

					Table[i+j*32].x = Sorted[0];
					Table[i+j*32].y = Sorted[1];
					Table[i+j*32].z = Sorted[2];
				}
			}
		}

		const PxU32* IntTable = (const PxU32*)Table;
		PxU32* dest = (PxU32*)normals;
		while(nb--)
		{
			const PxU16 code = *tmp++;
			const PxU16 Index = (code>>6) & 1023;
			const PxU16 octant	= (code>>3) & 7;
			const PxU16 sextant	= (code & 7)<<1;

			const PxU32* Data = IntTable + Index*3;
			*dest++ = Data[(1176>>sextant)&3] | ((octant    )<<31);
			*dest++ = Data[(8544>>sextant)&3] | ((octant & 2)<<30);
			*dest++ = Data[(6660>>sextant)&3] | ((octant & 4)<<29);
		}
	}
	*/

	bool convexHullLoad(Gu::ConvexHullData& data, const PxStream& stream, PxBitAndDword& bufferSize)
	{
		PxU32 Version;
		bool Mismatch;
		if(!ReadHeader('C', 'L', 'H', 'L', Version, Mismatch, stream))
			return false;

		if(!ReadHeader('C', 'V', 'H', 'L', Version, Mismatch, stream))
			return false;

		PxU32 Nb;

		// Import figures
		{
			PxU32 tmp[4];
			ReadDwordBuffer(tmp, 4, Mismatch, stream);
			data.mNbHullVertices	= tmp[0];
			data.mNbEdges			= tmp[1];
			data.mNbPolygons		= tmp[2];
			Nb						= tmp[3];
		}

		//AM: In practice the old aligner approach wastes 20 bytes and there is no reason to 20 byte align this data.
		//I changed the code to just 4 align for the time being.  
		//On consoles if anything we will need to make this stuff 16 byte align vectors to have any sense, which will have to be done by padding data structures.
		PX_ASSERT(sizeof(Gu::HullPolygonData) % sizeof(PxReal) == 0);	//otherwise please pad it.
		PX_ASSERT(sizeof(PxVec3) % sizeof(PxReal) == 0);

		PxU32 bytesNeeded = computeBufferSize(data, Nb);

		PX_FREE(data.mPolygons);	// Load() can be called for an existing convex mesh. In that case we need to free
		// the memory first.

		bufferSize = (PxU32)Nb;
		void* mDataMemory = PX_ALLOC(bytesNeeded);

		PxU8* address = (PxU8*)mDataMemory;

		data.mPolygons				= (Gu::HullPolygonData*)	address;	address += sizeof(Gu::HullPolygonData) * data.mNbPolygons;
		PxVec3* mDataHullVertices	= (PxVec3*)					address;	address += sizeof(PxVec3) * data.mNbHullVertices;
		PxU8* mDataFacesByEdges8	= (PxU8*)					address;	address += sizeof(PxU8) * data.mNbEdges * 2;
		PxU8* mDataVertexData8		= (PxU8*)					address;	address += sizeof(PxU8) * Nb;	// PT: leave that one last, so that we don't need to serialize "Nb"

		PX_ASSERT(!(size_t(mDataHullVertices) % sizeof(PxReal)));
		PX_ASSERT(!(size_t(data.mPolygons) % sizeof(PxReal)));
		PX_ASSERT(size_t(address)<=size_t(mDataMemory)+bytesNeeded);

		// Import vertices
		ReadFloatBuffer(&mDataHullVertices->x, 3*data.mNbHullVertices, Mismatch, stream);

		PxU16 useUnquantizedNormals = ReadWord(Mismatch, stream);

		// Import polygons
		stream.readBuffer(data.mPolygons, data.mNbPolygons*sizeof(Gu::HullPolygonData));

		if(Mismatch)
		{
			for(PxU32 i=0;i<data.mNbPolygons;i++)
			{
				Flip(data.mPolygons[i].mVRef8);
				Flip(data.mPolygons[i].mPlane.normal.x);
				Flip(data.mPolygons[i].mPlane.normal.y);
				Flip(data.mPolygons[i].mPlane.normal.z);
				Flip(data.mPolygons[i].mPlane.d);
			}
		}

		stream.readBuffer(mDataVertexData8, Nb);
		stream.readBuffer(mDataFacesByEdges8, data.mNbEdges*2);

		return true;
	}
}

bool Gu::ConvexMesh::load(const PxStream& stream)
{
	// Import header
	PxU32 version;
	bool mismatch;
	if(!readHeader('C', 'V', 'X', 'M', version, mismatch, stream))
		return false;

	// Check if old (incompatible) mesh format is loaded
	if (version < PX_CONVEX_VERSION)
	{
		Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Loading convex mesh failed: "
			"Deprecated mesh cooking format. Please install and run the mesh converter tool to convert your mesh to the new cooking format.");
		return false;
	}

	// Import serialization flags
	PxU32 serialFlags	= readDword(mismatch, stream);

	// Import convex hull before Opcode model in this mode
	if(!convexHullLoad(chull, stream, mNb))
		return false;

	// Import local bounds
	float tmp[8];
	readFloatBuffer(tmp, 8, mismatch, stream);
//	geomEpsilon				= tmp[0];
	chull.mAABB = PxBounds3(PxVec3(tmp[1], tmp[2], tmp[3]), PxVec3(tmp[4],tmp[5],tmp[6]));

	// Import mass info
	mass = tmp[7];
	if(mass!=-1.0f)
	{
		readFloatBuffer(&inertia(0,0), 9, mismatch, stream);
		readFloatBuffer(&chull.mCenterOfMass.x, 3, mismatch, stream);
	}

	// Import gaussmaps
	PxF32 gaussMapFlag = readFloat(mismatch, stream);
	if(gaussMapFlag != -1.0f)
	{
		PX_ASSERT(gaussMapFlag == 1.0f);	//otherwise file is corrupt

		PX_DELETE_AND_RESET(bigConvexData);
		bigConvexData = PX_NEW(BigConvexData);
		if(bigConvexData)	
		{
			bigConvexData->Load(stream);
			chull.mBigConvexRawData = &bigConvexData->mData;
		}
	}
	
/*
	printf("\n\n");
	printf("COM: %f %f %f\n", massInfo.centerOfMass.x, massInfo.centerOfMass.y, massInfo.centerOfMass.z);
	printf("BND: %f %f %f\n", chull.aabb.getCenter().x, chull.aabb.getCenter().y, chull.aabb.getCenter().z);
	printf("CNT: %f %f %f\n", chull.mCenterxx.x, chull.mCenterxx.y, chull.mCenterxx.z);
	printf("COM-BND: %f BND-CNT: %f, CNT-COM: %f\n", (massInfo.centerOfMass - chull.aabb.getCenter()).magnitude(), (chull.aabb.getCenter() - chull.mCenterxx).magnitude(), (chull.mCenterxx - massInfo.centerOfMass).magnitude());
*/

// TEST_INTERNAL_OBJECTS
	chull.mInternal.mRadius		= readFloat(mismatch, stream);
	chull.mInternal.mExtents[0]	= readFloat(mismatch, stream);
	chull.mInternal.mExtents[1]	= readFloat(mismatch, stream);
	chull.mInternal.mExtents[2]	= readFloat(mismatch, stream);
//~TEST_INTERNAL_OBJECTS
	return true;
}



void Gu::ConvexMesh::release()
{
	decRefCount();
}

void Gu::ConvexMesh::onRefCountZero()
{
	if(mMeshFactory->removeConvexMesh(*this))
		return deleteSerializedObject(this);

	// PT: if we reach this point, we didn't find the mesh in the Physics object => don't delete!
	// This prevents deleting the object twice.
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Gu::ConvexMesh::release: double deletion detected!");
}

PxU32 Gu::ConvexMesh::getReferenceCount() const
{
	return getRefCount();
}


void Gu::ConvexMesh::getMassInformation(PxReal& mass, PxMat33& localInertia, PxVec3& localCenterOfMass) const
{
	mass = Gu::ConvexMesh::getMass();
	localInertia = Gu::ConvexMesh::getInertia();
	localCenterOfMass = Gu::ConvexMesh::getHull().mCenterOfMass;
}

#if PX_ENABLE_DEBUG_VISUALIZATION
#include "CmMatrix34.h"
#include "GuDebug.h"
void Gu::ConvexMesh::debugVisualize(
	Cm::RenderOutput& out, const Cm::Matrix34& absPose, const PxBounds3& cullbox,
	const PxU64 mask, const PxReal fscale)	const
{
	// Skip all polygons if at least one vertice is not within culling box.
	if (mask & ((PxU64)1 << PxVisualizationParameter::eCULL_BOX))
	{
		const PxVec3* vertices = chull.getHullVertices();
		const PxU8* indexBuffer = chull.getVertexData8();
		const PxU32 nbPolygons = getNbPolygonsFast();

		for (PxU32 i = 0; i < nbPolygons; i++)
		{
			PxU32 pnbVertices = chull.mPolygons[i].mNbVerts;

			for (PxU32 j = 1; j < pnbVertices; j++)
			{
				const PxVec3& vertex = vertices[indexBuffer[j]];
				if (!cullbox.contains(absPose.base3 + vertex))
					return;
			}

			indexBuffer += pnbVertices;
		}
	}

	if (mask & ((PxU64)1 << PxVisualizationParameter::eCOLLISION_SHAPES))
	{
		const PxU32 scolor = PxU32(PxDebugColor::eARGB_MAGENTA);

		const PxVec3* vertices = chull.getHullVertices();
		const PxU8* indexBuffer = chull.getVertexData8();
		const PxU32 nbPolygons = getNbPolygonsFast();

		const PxMat44 m44 = Gu::Debug::convertToPxMat44(absPose);

		out << m44 << scolor;	// PT: no need to output this for each segment!

		for (PxU32 i = 0; i < nbPolygons; i++)
		{
			const PxU32 pnbVertices = chull.mPolygons[i].mNbVerts;

			PxVec3 begin = m44.transform(vertices[indexBuffer[0]]);	// PT: transform it only once before the loop starts
			for (PxU32 j = 1; j < pnbVertices; j++)
			{
				PxVec3 end = m44.transform(vertices[indexBuffer[j]]);
				out.outputSegment(begin, end);
				begin = end;
			}
			out.outputSegment(begin, m44.transform(vertices[indexBuffer[0]]));

			indexBuffer += pnbVertices;
		}
	}
}

#endif
