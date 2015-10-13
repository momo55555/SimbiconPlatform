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

#include "PxExtensionsAPI.h"
#include "PsFoundation.h"
#include "CmMetaData.h"
#include "CmStringTable.h"
#include "PxGaussMapLimit.h"
#include "ExtDistanceJoint.h"
#include "ExtD6Joint.h"
#include "ExtFixedJoint.h"
#include "ExtPrismaticJoint.h"
#include "ExtRevoluteJoint.h"
#include "ExtSphericalJoint.h"

#include <stdio.h>

#if PX_SUPPORT_VISUAL_DEBUGGER
#include "PvdConnectionManager.h"
#include "ExtVisualDebugger.h"
#endif

using namespace physx;
using namespace Ps;

#if PX_SUPPORT_VISUAL_DEBUGGER
struct JointConnectionHandler : public PVD::PvdConnectionHandler
{
	virtual void onPvdConnected( PVD::PvdConnection* inFactory )
	{
		using namespace PVD;
		//register the joint classes.
		PvdDataStream* connection = inFactory->createDataStream();
		connection->addRef();
		Ext::VisualDebugger::sendClassDescriptions( *connection );
		connection->flush();
		connection->release();
	}
	virtual void onPvdDisconnected( PVD::PvdConnection*)
	{
	}
};

static JointConnectionHandler gPvdHandler;
#endif

bool PxInitExtensions(PxPhysics& physics)
{
	Ps::Foundation::setInstance(static_cast<Ps::Foundation*>(&physics.getFoundation()));

	physics.registerClass(PxSerialType::eUSER_SPHERICAL_JOINT,	Ext::SphericalJoint::createInstance);
	physics.registerClass(PxSerialType::eUSER_REVOLUTE_JOINT,	Ext::RevoluteJoint::createInstance);
	physics.registerClass(PxSerialType::eUSER_DISTANCE_JOINT,	Ext::DistanceJoint::createInstance);
	physics.registerClass(PxSerialType::eUSER_D6_JOINT,			Ext::D6Joint::createInstance);
	physics.registerClass(PxSerialType::eUSER_PRISMATIC_JOINT,	Ext::PrismaticJoint::createInstance);
	physics.registerClass(PxSerialType::eUSER_FIXED_JOINT,		Ext::FixedJoint::createInstance);
#if PX_SUPPORT_VISUAL_DEBUGGER
	if ( physics.getPvdConnectionManager() != NULL )
		physics.getPvdConnectionManager()->addHandler( &gPvdHandler );
#endif
	return true;
}

void PxCloseExtensions(void)
{
}

void PxRegisterExtJointMetaData(PxSerialStream& stream);
bool PxDumpMetaData(const char* metaFile, const PxPhysics& sdk)
{
	PX_ASSERT(metaFile);
	if(!metaFile)
		return false;

	class MetaDataStream : public PxSerialStream
	{
		public:
		virtual		void		storeBuffer(const void* buffer, PxU32 size)
		{
			PX_ASSERT(size==sizeof(Cm::MetaDataEntry));
			const Cm::MetaDataEntry* entry = (const Cm::MetaDataEntry*)buffer;
			metaData.pushBack(*entry);
		}
		virtual		PxU32		getTotalStoredSize()	{ return 0; }

		Array<Cm::MetaDataEntry> metaData;
	}s;

	sdk.getMetaData(s);
	PxRegisterExtJointMetaData(s);

	physx::shdfnd3::Array<char>	stringTable;

	PxU32 nb = s.metaData.size();
	Cm::MetaDataEntry* entries = s.metaData.begin();
	for(PxU32 i=0;i<nb;i++)
	{
		entries[i].mType = (const char*)Cm::addToStringTable(stringTable, entries[i].mType);
		entries[i].mName = (const char*)Cm::addToStringTable(stringTable, entries[i].mName);
	}

	PxU32 platformTag = 0;
#ifdef PX_X64
	platformTag = 'PC64';
	const PxU32 gaussMapLimit = PxGetGaussMapVertexLimitForPlatform(PxPlatform::ePC);
	const PxU32 tiledHeightFieldSamples = 0;
#endif
#if defined(PX_X86) || defined(__CYGWIN__)
	platformTag = 'PC32';
	const PxU32 gaussMapLimit = PxGetGaussMapVertexLimitForPlatform(PxPlatform::ePC);
	const PxU32 tiledHeightFieldSamples = 0;
#endif
#ifdef PX_X360
	platformTag = 'XBOX';
	const PxU32 gaussMapLimit = PxGetGaussMapVertexLimitForPlatform(PxPlatform::eXENON);
	const PxU32 tiledHeightFieldSamples = 0;
#endif
#ifdef PX_PS3
	platformTag = 'PS_3';
	const PxU32 gaussMapLimit = PxGetGaussMapVertexLimitForPlatform(PxPlatform::ePLAYSTATION3);
	const PxU32 tiledHeightFieldSamples = 1;
#endif
#ifdef PX_ARM
	platformTag = 'ARM ';
	const PxU32 gaussMapLimit = PxGetGaussMapVertexLimitForPlatform(PxPlatform::eARM);
	const PxU32 tiledHeightFieldSamples = 1;
#endif

	FILE* fp = fopen(metaFile, "wb");
	if(!fp)
		return false;

	const PxU32 header = 'META';
	const PxU32 version = 1;
	const PxU32 ptrSize = sizeof(void*);
	fwrite(&header, 1, 4, fp);
	fwrite(&version, 1, 4, fp);
	fwrite(&ptrSize, 1, 4, fp);
	fwrite(&platformTag, 1, 4, fp);
	fwrite(&gaussMapLimit, 1, 4, fp);
	fwrite(&tiledHeightFieldSamples, 1, 4, fp);

	fwrite(&nb, 1, 4, fp);
	fwrite(entries, 1, nb*sizeof(Cm::MetaDataEntry), fp);

	PxU32 length = stringTable.size();
	const char* table = stringTable.begin();
	fwrite(&length, 1, 4, fp);
	fwrite(table, 1, length, fp);

	fclose(fp);

	return true;
}

// PT: TODO: move those functions to a separate file, remove all allocations

#include "PxConvexMesh.h"
#include "PxTriangleMesh.h"
#include "PxHeightField.h"
#include "PxDeformableMesh.h"
#include "PxMaterial.h"
#include "cloth/PxClothFabric.h"

void PxCollectForExportSDK(const PxPhysics& sdk, PxCollection& collection)
{
	// Collect convexes
	{
		const PxU32 nbObjects = sdk.getNbConvexMeshes();
		PxConvexMesh** objects = new PxConvexMesh*[nbObjects];
		const PxU32 nb = sdk.getConvexMeshes(objects, nbObjects);
		PX_ASSERT(nb==nbObjects);
		for(PxU32 i=0;i<nbObjects;i++)
			objects[i]->collectForExport(collection);
		delete [] objects;
	}

	// Collect triangle meshes
	{
		const PxU32 nbObjects = sdk.getNbTriangleMeshes();
		PxTriangleMesh** objects = new PxTriangleMesh*[nbObjects];
		const PxU32 nb = sdk.getTriangleMeshes(objects, nbObjects);
		PX_ASSERT(nb==nbObjects);
		for(PxU32 i=0;i<nbObjects;i++)
			objects[i]->collectForExport(collection);
		delete [] objects;
	}

	// Collect heightfields
	{
		const PxU32 nbObjects = sdk.getNbHeightFields();
		PxHeightField** objects = new PxHeightField*[nbObjects];
		const PxU32 nb = sdk.getHeightFields(objects, nbObjects);
		PX_ASSERT(nb==nbObjects);
		for(PxU32 i=0;i<nbObjects;i++)
			objects[i]->collectForExport(collection);
		delete [] objects;
	}

#if PX_USE_DEFORMABLE_API
	// Collect deformables
	{
		const PxU32 nbObjects = sdk.getNbDeformableMeshes();
		PxDeformableMesh** objects = new PxDeformableMesh*[nbObjects];
		const PxU32 nb = sdk.getDeformableMeshes(objects, nbObjects);
		PX_ASSERT(nb==nbObjects);
		for(PxU32 i=0;i<nbObjects;i++)
			objects[i]->collectForExport(collection);
		delete [] objects;
	}
#endif

	// Collect materials
	{
		const PxU32 nbObjects = sdk.getNbMaterials();
		PxMaterial** objects = new PxMaterial*[nbObjects];
		const PxU32 nb = sdk.getMaterials(objects, nbObjects);
		PX_ASSERT(nb==nbObjects);
		for(PxU32 i=0;i<nbObjects;i++)
			objects[i]->collectForExport(collection);
		delete [] objects;
	}

	// Collect cloth fabrics
	{
		const PxU32 nbObjects = sdk.getNbClothFabrics();
		PxClothFabric** objects = new PxClothFabric*[nbObjects];
		const PxU32 nb = sdk.getClothFabrics(objects, nbObjects);
		PX_ASSERT(nb==nbObjects);
		for(PxU32 i=0;i<nbObjects;i++)
			objects[i]->collectForExport(collection);
		delete [] objects;
	}
}

#include "PxScene.h"
#include "PxAttachment.h"
#include "PxArticulation.h"
#include "PxAggregate.h"
void PxCollectForExportScene(const PxScene& scene, PxCollection& collection)
{
	// Collect actors
	{
		const PxActorTypeSelectionFlags selectionFlags = PxActorTypeSelectionFlag::eRIGID_STATIC
														|PxActorTypeSelectionFlag::eRIGID_DYNAMIC
														|PxActorTypeSelectionFlag::eDEFORMABLE
														|PxActorTypeSelectionFlag::ePARTICLE_SYSTEM
														|PxActorTypeSelectionFlag::ePARTICLE_FLUID
														|PxActorTypeSelectionFlag::eCLOTH;

		const PxU32 nbObjects = scene.getNbActors(selectionFlags);
		PxActor** objects = new PxActor*[nbObjects];
		const PxU32 nb = scene.getActors(selectionFlags, objects, nbObjects);
		PX_ASSERT(nb==nbObjects);
		for(PxU32 i=0;i<nbObjects;i++)
			objects[i]->collectForExport(collection);
		delete [] objects;
	}

	// Collect attachments
	{
		const PxU32 nbObjects = scene.getNbAttachments();
		PxAttachment** objects = new PxAttachment*[nbObjects];
		const PxU32 nb = scene.getAttachments(objects, nbObjects);
		PX_ASSERT(nb==nbObjects);
		for(PxU32 i=0;i<nbObjects;i++)
			objects[i]->collectForExport(collection);
		delete [] objects;
	}

	// Collect constraints
	{
		const PxU32 nbObjects = scene.getNbConstraints();
		PxConstraint** objects = new PxConstraint*[nbObjects];
		const PxU32 nb = scene.getConstraints(objects, nbObjects);
		PX_ASSERT(nb==nbObjects);
		for(PxU32 i=0;i<nbObjects;i++)
			objects[i]->collectForExport(collection);
		delete [] objects;
	}

	// Collect articulations
	{
		const PxU32 nbObjects = scene.getNbArticulations();
		PxArticulation** objects = new PxArticulation*[nbObjects];
		const PxU32 nb = scene.getArticulations(objects, nbObjects);
		PX_ASSERT(nb==nbObjects);
		for(PxU32 i=0;i<nbObjects;i++)
			objects[i]->collectForExport(collection);
		delete [] objects;
	}

	// Collect aggregates
	{
		const PxU32 nbObjects = scene.getNbAggregates();
		PxAggregate** objects = new PxAggregate*[nbObjects];
		const PxU32 nb = scene.getAggregates(objects, nbObjects);
		PX_ASSERT(nb==nbObjects);
		for(PxU32 i=0;i<nbObjects;i++)
			objects[i]->collectForExport(collection);
		delete [] objects;
	}
}