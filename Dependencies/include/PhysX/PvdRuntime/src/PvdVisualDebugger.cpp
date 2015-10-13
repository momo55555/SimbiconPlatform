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


#if PX_SUPPORT_VISUAL_DEBUGGER

#include "PxPhysXCommon.h"
#include "PxProfileBase.h"

#include "PvdVisualDebugger.h"
#include "PvdDataStream.h"
#include "PvdConnection.h"
#include "PvdClassDefinitions.h"

#include "ScPhysics.h"
#include "NpScene.h"
#include "PsFoundation.h"

#include "ScBodyCore.h"

#include "NpRigidDynamic.h"
#include "NpRigidStatic.h"
#include "PxHeightFieldSample.h"


namespace physx
{
namespace Pvd
{
	
PX_FORCE_INLINE static const PVD::Float3& toPvdType(const PxVec3& vec3) { return reinterpret_cast<const PVD::Float3&>(vec3); }
PX_FORCE_INLINE static const PVD::Quat& toPvdType(const PxQuat& quat) { return reinterpret_cast<const PVD::Quat&>(quat); }

VisualDebugger::VisualDebugger()
: mPvdConnection(NULL)
, mPvdConnectionFactory(NULL)
, mJointFrameScale(0.0f)
, mJointLimitScale(0.0f)
, mFlags(0)
{
}


VisualDebugger::~VisualDebugger()
{
	disconnect();
}


void VisualDebugger::disconnect()
{
	NpPhysics& npPhysics = NpPhysics::getInstance();
	if ( npPhysics.getPvdConnectionManager() )
		npPhysics.getPvdConnectionManager()->disconnect();
}


PVD::PvdConnection* VisualDebugger::getPvdConnectionFactory()
{
	return mPvdConnectionFactory;
}

PVD::PvdDataStream* VisualDebugger::getPvdConnection(const PxScene& scene)
{
	const NpScene& npScene = static_cast<const NpScene&>(scene);
	return npScene.getScene().getSceneVisualDebugger().getPvdConnection();
}


bool VisualDebugger::createCamera(PxU64 instanceKey, const char* name)
{
	if(!isConnected())
		return false;

	PVD::PvdCommLayerError error;
	mPvdConnection->pushNamespace();
	mPvdConnection->setNamespace( "" );
	error = mPvdConnection->createInstance(PvdClassKeys::Camera+1, instanceKey, PVD::EInstanceUIFlags::TopLevel);
	error = mPvdConnection->setPropertyValue(instanceKey, CameraProp::Name+1, PVD::createString(name));
	mPvdConnection->popNamespace();
	return (error == PVD::PvdCommLayerError::None);
}


bool VisualDebugger::updateCamera(PxU64 instanceKey, const PxVec3& origin, const PxVec3& up, const PxVec3& target)
{
	if(!isConnected())
		return false;

	PVD::PvdCommLayerError error;
	error = mPvdConnection->setPropertyValue(instanceKey, CameraProp::Origin+1, toPvdType(origin));
	error = mPvdConnection->setPropertyValue(instanceKey, CameraProp::Up+1, toPvdType(up));
	error = mPvdConnection->setPropertyValue(instanceKey, CameraProp::Target+1, toPvdType(target));
	error = mPvdConnection->flush();

	return (error == PVD::PvdCommLayerError::None);
}


void VisualDebugger::setJointVisualizationScale(PxReal frameScale, PxReal limitScale)
{
	mJointFrameScale = frameScale;
	mJointLimitScale = limitScale;
}

void VisualDebugger::setVisualDebuggerFlag(PxVisualDebuggerFlags::Enum flag, bool value)
{
	if(value)
		mFlags |= PxU32(flag);
	else
		mFlags &= ~PxU32(flag);
}

PxU32 VisualDebugger::getVisualDebuggerFlags()
{
	return mFlags;
}


bool VisualDebugger::isConnected()
{ 
	return mPvdConnectionFactory && mPvdConnectionFactory->isConnected(); 
}


void VisualDebugger::checkConnection()
{
	if ( mPvdConnectionFactory != NULL ) mPvdConnectionFactory->checkConnection();
}


void VisualDebugger::updateScenesPvdConnection()
{
	NpPhysics& npPhysics = NpPhysics::getInstance();
	PxU32 numScenes = npPhysics.getNbScenes();
	for(PxU32 i = 0; i < numScenes; i++)
	{
		NpScene* npScene = npPhysics.getScene(i);
		Scb::Scene& scbScene = npScene->getScene();
		setupSceneConnection(scbScene);
	}
}

void VisualDebugger::setupSceneConnection(Scb::Scene& s)
{
	PVD::PvdDataStream* conn = mPvdConnectionFactory ? mPvdConnectionFactory->createDataStream() : NULL;
	if(conn)
		conn->setNamespace(getPhysxNamespace());
	s.getSceneVisualDebugger().setPvdConnection(conn, mPvdConnectionFactory ? mPvdConnectionFactory->getConnectionType() : PVD::TConnectionFlagsType(0));
	s.getSceneVisualDebugger().setCreateContactReports(conn ? getTransmitContactsFlag() : false);
}


void VisualDebugger::sendClassDescriptions()
{
	if(!isConnected())
		return;

	// register all classes, see PvdClassDefinitions.h
	for(PxU32 i = 0; i < PvdClassKeys::NUM_ELEMENTS; i++)
	{
		const ClassRow& classRow = gClassKeyTable[i];
		PxU32 classKey = classRow.classKey;
		if ( !classRow.physxNamespace )
		{
			mPvdConnection->pushNamespace();
			mPvdConnection->setNamespace( "" );
		}
		PVD::PvdCommLayerError error;
		error = mPvdConnection->createClass(classRow.name, classKey);
		PX_ASSERT(error == PVD::PvdCommLayerError::None);
		if(classRow.parentId != (PxU16)-1)
		{
			error = mPvdConnection->deriveClass(classRow.parentId+1, classKey);
			PX_ASSERT(error == PVD::PvdCommLayerError::None);
		}

		if ( mMetaDataBinding.registerPropertyOverride( mPvdConnection, static_cast< PvdClassKeys::Enum >( classKey - 1 ) ) == false )
		{
			// register properties
			PxU32 numProperties = classRow.propertyTableSize;
			const PropertyRow* propertyTable = classRow.propertyTable;
			for(PxU32 p = 0; p < numProperties; p++)
			{
				const PropertyRow& prop = propertyTable[p];

				PxU32 propertyKey = classRow.propertyOffset + p+1;
				if (prop.dataType != (PxU32) -1)
					error = mPvdConnection->defineProperty(classKey, prop.name, NULL, prop.dataType, propertyKey);
				else
					error = mPvdConnection->defineArrayProperty(classKey, prop.name, prop.arrayClass, propertyKey);

				if(prop.dataType == PVD::PvdCommLayerDatatype::Bitflag)
					error = mPvdConnection->defineBitflagNames(classKey, propertyKey, prop.table, prop.size);
				else if(prop.dataType == PVD::PvdCommLayerDatatype::EnumerationValue)
					error = mPvdConnection->defineEnumerationNames(classKey, propertyKey, prop.table, prop.size);
					
				PX_ASSERT(error == PVD::PvdCommLayerError::None);
			}
		}
		if ( !classRow.physxNamespace )
			mPvdConnection->popNamespace();
	}
	mPvdConnection->flush();
}

#define SETGROUPNAME(name) mPvdConnection->setPropertyValue(physixId+1+SdkGroups::name, GroupProp::Name+1, PVD::createString(#name))
void VisualDebugger::createGroups()
{
	PVD::PvdCommLayerError error;
	NpPhysics& npPhysics = NpPhysics::getInstance();
	PxU64 physixId = PX_PROFILE_POINTER_TO_U64(&npPhysics);
	error = mPvdConnection->pushNamespace();
	error = mPvdConnection->setNamespace("");
	for(PxU32 i = 0; i < SdkGroups::NUM_ELEMENTS; i++)
	{
		error = mPvdConnection->createInstance(PvdClassKeys::Group+1, physixId+i+1, PVD::EInstanceUIFlags::None);
		error = mPvdConnection->addChild(physixId, physixId+i+1);
	}
	SETGROUPNAME(Scenes);
	SETGROUPNAME(TriangleMeshes);
	SETGROUPNAME(ConvexMeshes);
	SETGROUPNAME(HeightFields);
	SETGROUPNAME(ClothFabrics);
	mPvdConnection->popNamespace();
}

void VisualDebugger::releaseGroups()
{
	PVD::PvdCommLayerError error;
	NpPhysics& npPhysics = NpPhysics::getInstance();
	PxU64 physixId = PX_PROFILE_POINTER_TO_U64(&npPhysics);
	for(PxU32 i = 0; i < SdkGroups::NUM_ELEMENTS; i++)
	{
		error = mPvdConnection->destroyInstance(physixId+i+1);
		PX_ASSERT(error == PVD::PvdCommLayerError::None);
	}
}

void VisualDebugger::onPvdConnected( PVD::PvdConnection* inFactory )
{
	PVD::PvdConnection* cf( inFactory );
	if(!cf)
		return;

	if(mPvdConnection && mPvdConnectionFactory)
		disconnect();

	mPvdConnectionFactory = cf;
	mPvdConnection = mPvdConnectionFactory->createDataStream();
	if(!mPvdConnection)
		return;

	mPvdConnectionFactory->addRef();
	mPvdConnection->addRef();
	mPvdConnection->setNamespace( getPhysxNamespace() );
	updateScenesPvdConnection();

	sendClassDescriptions();
	sendEntireSDK();
}
void VisualDebugger::onPvdDisconnected( PVD::PvdConnection* inFactory )
{
	if(mPvdConnection)
	{
		NpPhysics& npPhysics = NpPhysics::getInstance();
		if(mPvdConnection->isConnected())
		{
			releaseGroups();
			mPvdConnection->destroyInstance(PX_PROFILE_POINTER_TO_U64(&NpPhysics::getInstance()));
			mPvdConnection->flush();
		}
		mPvdConnection->release();
		mPvdConnection = NULL;
		mPvdConnectionFactory->release();
		mPvdConnectionFactory = NULL;
		updateScenesPvdConnection();
		mRefCountMapLock.lock();
		mRefCountMap.clear();
		mRefCountMapLock.unlock();
	}
}

void VisualDebugger::sendEntireSDK()
{
	PVD::PvdCommLayerError error;
	error = mPvdConnection->beginFrame();
	
	NpPhysics& npPhysics = NpPhysics::getInstance();
	error = mPvdConnection->createInstance(PvdClassKeys::PhysicsSDK+1, PX_PROFILE_POINTER_TO_U64(&npPhysics), PVD::EInstanceUIFlags::TopLevel);
	mMetaDataBinding.sendAllProperties( mPvdConnection, &npPhysics );
	createGroups();

#define SEND_BUFFER_GROUP( type, name ) {					\
		Ps::Array<type*> buffers;							\
		PxU32 numBuffers = npPhysics.getNb##name();			\
		buffers.resize(numBuffers);							\
		npPhysics.get##name(buffers.begin(), numBuffers);	\
		for(PxU32 i = 0; i < numBuffers; i++)				\
			increaseReference(buffers[i]);					\
	}
	SEND_BUFFER_GROUP( PxTriangleMesh, TriangleMeshes );
	SEND_BUFFER_GROUP( PxConvexMesh, ConvexMeshes );
	SEND_BUFFER_GROUP( PxHeightField, HeightFields );
	SEND_BUFFER_GROUP( PxHeightField, HeightFields );


	
	//Ensure that all the instances and class descriptions created so far
	//are available to the rest of the system.
	error = mPvdConnection->flush();

	PxU32 numScenes = npPhysics.getNbScenes();
	for(PxU32 i = 0; i < numScenes; i++)
	{
		NpScene* npScene = npPhysics.getScene(i);
		Scb::Scene& scbScene = npScene->getScene();

		scbScene.getSceneVisualDebugger().sendEntireScene();
	}

	error = mPvdConnection->endFrame();
	error = mPvdConnection->flush();
}


void VisualDebugger::createPvdInstance(const PxTriangleMesh* triMesh)
{
	// create instance
	PVD::PvdCommLayerError error;
	NpPhysics* npPhysics = &NpPhysics::getInstance();

	error = mPvdConnection->createInstance(PvdClassKeys::TriangleMesh+1, PX_PROFILE_POINTER_TO_U64(triMesh), PVD::EInstanceUIFlags::None);
	error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(npPhysics)+SdkGroups::TriangleMeshes+1, PX_PROFILE_POINTER_TO_U64(triMesh));

	updatePvdProperties(triMesh);

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}

void VisualDebugger::updatePvdProperties(const PxTriangleMesh* triMesh)
{
	PVD::PvdCommLayerError error;

	bool hasMatIndex = triMesh->getTriangleMaterialIndex(0) != 0xffff;
	PxU64 theInstance(PX_PROFILE_POINTER_TO_U64(triMesh));
	
	// update arrays:
	// vertex Array:
	{ 
		const PxU8* vertexPtr = reinterpret_cast<const PxU8*>(triMesh->getVertices());
		const PxU32 vertexStride = sizeof(PxVec3);
		const PxU32 numVertices = triMesh->getNbVertices();
		error = PvdConnectionHelper::sendSingleElementArrayProperty(mPvdConnection, theInstance, TriangleMeshProp::VertexArray
																 , VectorArrayProp::Element, PVD::PvdCommLayerDatatype::Float3
																 , vertexPtr, vertexStride, numVertices);
		PX_ASSERT(error == PVD::PvdCommLayerError::None);
	}

	// index Array:
	{
		const bool has16BitIndices = triMesh->has16BitTriangleIndices();
		const PxU8* trianglePtr = reinterpret_cast<const PxU8*>(triMesh->getTriangles());
		const PxU32 triangleStride = has16BitIndices ? sizeof(PxU16)* 3 : sizeof(PxU32)* 3;
		const PxU32 numTriangles = triMesh->getNbTriangles();

		mPvdConnectionHelper.sendPrimitiveIndexArrayProperty(mPvdConnection, theInstance, TriangleMeshProp::TriangleIndexArray
															, trianglePtr, triangleStride, numTriangles, 3, has16BitIndices);
		

		PX_ASSERT(error == PVD::PvdCommLayerError::None);
	}

	// material Array:
	if(hasMatIndex)
	{
		PxU32 numMaterials = triMesh->getNbTriangles();
		PvdConnectionHelper::beginSingleElementArrayProperty(mPvdConnection, theInstance, TriangleMeshProp::TriangleMaterialArray, U16ArrayProp::Element, PVD::PvdCommLayerDatatype::U16);

		for(PxU32 m = 0; m < numMaterials; m++)
		{
			PVD::PvdCommLayerValue theValue(triMesh->getTriangleMaterialIndex(m));
			error = mPvdConnection->sendArrayObject(&theValue);
		}

		error = mPvdConnection->endArrayPropertyBlock();
		PX_ASSERT(error == PVD::PvdCommLayerError::None);
	}
}


void VisualDebugger::createPvdInstance(const PxConvexMesh* convexMesh)
{
	// create instance
	PVD::PvdCommLayerError error;
	NpPhysics* npPhysics = &NpPhysics::getInstance();

	error = mPvdConnection->createInstance(PvdClassKeys::ConvexMesh+1, PX_PROFILE_POINTER_TO_U64(convexMesh), PVD::EInstanceUIFlags::None);
	error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(npPhysics)+SdkGroups::ConvexMeshes+1, PX_PROFILE_POINTER_TO_U64(convexMesh));

	updatePvdProperties(convexMesh);

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void VisualDebugger::updatePvdProperties(const PxConvexMesh* convexMesh)
{
	PVD::PvdCommLayerError error;

	PxReal mass;
	PxMat33Legacy localInertia;
	PxVec3 localCom;
	convexMesh->getMassInformation(mass, reinterpret_cast<PxMat33 &>(localInertia), localCom);
	
	PxU64 theInstance(PX_PROFILE_POINTER_TO_U64(convexMesh));
	mPvdConnectionHelper.addPropertyGroupProperty(ConvexMeshProp::Mass,					mass);
	mPvdConnectionHelper.addPropertyGroupProperty(ConvexMeshProp::LocalInertia,			toPvdType(localInertia.toQuat()));
	mPvdConnectionHelper.addPropertyGroupProperty(ConvexMeshProp::LocalCenterOfMass,	toPvdType(localCom));

	mPvdConnectionHelper.sendSinglePropertyGroup(mPvdConnection, theInstance, PvdClassKeys::ConvexMesh);
	
	// update arrays:
	// vertex Array:
	{
		const PxU8* vertexPtr = reinterpret_cast<const PxU8*>(convexMesh->getVertices());
		const PxU32 vertexStride = sizeof(PxVec3);
		const PxU32 numVertices = convexMesh->getNbVertices();
		
		error = PvdConnectionHelper::sendSingleElementArrayProperty(mPvdConnection, theInstance, ConvexMeshProp::VertexArray
																 , VectorArrayProp::Element, PVD::PvdCommLayerDatatype::Float3
																 , vertexPtr, vertexStride, numVertices);
		PX_ASSERT(error == PVD::PvdCommLayerError::None);
	}

	// HullPolyArray:
	PxU16 maxIndices = 0;
	{

		PxU32 properties[HullPolygonArrayProp::NUM_ELEMENTS];
		PVD::PvdCommLayerDatatype dataTypes[HullPolygonArrayProp::NUM_ELEMENTS] = {	PVD::PvdCommLayerDatatype::Plane,
																					PVD::PvdCommLayerDatatype::U16,
																					PVD::PvdCommLayerDatatype::U16};
		for(PxU32 i = 0; i < HullPolygonArrayProp::NUM_ELEMENTS; i++)
			properties[i] = i+1;

		error = mPvdConnection->beginArrayPropertyBlock(theInstance, ConvexMeshProp::HullPolygonArray+1, properties, dataTypes, HullPolygonArrayProp::NUM_ELEMENTS);

		static const PxU32 NUM_STACK_ELT = 32;
		PX_ALLOCA(stack, PxHullPolygon, NUM_STACK_ELT);
		PxHullPolygon* pxHullPolygons = stack;
		PxHullPolygon* pxHullPolygonsEnd = pxHullPolygons+NUM_STACK_ELT;
		PxHullPolygon* curOut = pxHullPolygons;

		PxU32 numPolygons = convexMesh->getNbPolygons();
		for(PxU32 index = 0; index < numPolygons; index++)
		{
			convexMesh->getPolygonData(index, *curOut);
			maxIndices = PxMax(maxIndices, PxU16(curOut->mIndexBase + curOut->mNbVerts));
			curOut++;

			if(curOut == pxHullPolygonsEnd)
			{
				error = mPvdConnection->sendArrayObjects((PxU8*)(pxHullPolygons), sizeof(PxHullPolygon), NUM_STACK_ELT);
				curOut = pxHullPolygons;
			}
		}

		if(curOut != pxHullPolygons)
			error = mPvdConnection->sendArrayObjects((PxU8*)(pxHullPolygons), sizeof(PxHullPolygon), PxU32(curOut-pxHullPolygons));

		error = mPvdConnection->endArrayPropertyBlock();
	}
	

	// poly index Array:
	{
		const PxU8* indices = convexMesh->getIndexBuffer();
		PxU32 indexCount = maxIndices;

		error = PvdConnectionHelper::sendSingleElementArrayProperty(mPvdConnection, theInstance, ConvexMeshProp::IndexArray
																 , U8ArrayProp::Element, PVD::PvdCommLayerDatatype::U8
																 , indices, sizeof(PxU8), indexCount);
		PX_ASSERT(error == PVD::PvdCommLayerError::None);
	}
}


void VisualDebugger::createPvdInstance(const PxHeightField* heightField)
{
	// create instance
	PVD::PvdCommLayerError error;
	NpPhysics* npPhysics = &NpPhysics::getInstance();

	error = mPvdConnection->createInstance(PvdClassKeys::HeightField+1, PX_PROFILE_POINTER_TO_U64(heightField), PVD::EInstanceUIFlags::None);
	error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(npPhysics)+SdkGroups::HeightFields+1, PX_PROFILE_POINTER_TO_U64(heightField));

	updatePvdProperties(heightField);

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void VisualDebugger::updatePvdProperties(const PxHeightField* heightField)
{
	PVD::PvdCommLayerError error;

	mPvdConnectionHelper.addPropertyGroupProperty(HeightFieldProp::NumRows,				heightField->getNbRows());
	mPvdConnectionHelper.addPropertyGroupProperty(HeightFieldProp::NumColumns,			heightField->getNbColumns());
	mPvdConnectionHelper.addPropertyGroupProperty(HeightFieldProp::HeightFieldFormat,	PVD::createEnumerationValue(heightField->getFormat()));
	mPvdConnectionHelper.addPropertyGroupProperty(HeightFieldProp::Thickness,			heightField->getThickness());
	mPvdConnectionHelper.addPropertyGroupProperty(HeightFieldProp::ConvexEdgeThreshold,	heightField->getConvexEdgeThreshold());
	mPvdConnectionHelper.addPropertyGroupProperty(HeightFieldProp::Flags,				PVD::createBitflag(heightField->getFlags()));

	error = mPvdConnectionHelper.sendSinglePropertyGroup(mPvdConnection, PX_PROFILE_POINTER_TO_U64(heightField), PvdClassKeys::HeightField);

	// samples array
	{
		PxU32 nbRows = heightField->getNbRows();
		PxU32 nbCols = heightField->getNbColumns();	
		PxU8* samplesPtr = (PxU8*)PX_ALLOC(nbCols*nbRows*sizeof(PxHeightFieldSample));
		heightField->saveCells(samplesPtr, nbRows*nbCols*sizeof(PxHeightFieldSample));

		PxU32 sampleStride = heightField->getSampleStride();
		PxU32 numSamples = nbCols * nbRows;

		error = PvdConnectionHelper::sendSingleElementArrayProperty(mPvdConnection, PX_PROFILE_POINTER_TO_U64(heightField), HeightFieldProp::Samples
																	, HeightFieldSampleArrayProp::Element, PVD::PvdCommLayerDatatype::HeightFieldSample
																	, samplesPtr, sampleStride, numSamples);
		PX_FREE_AND_RESET(samplesPtr);
	}

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void VisualDebugger::createPvdInstance(const PxClothFabric* fabric)
{
	// create instance
	PVD::PvdCommLayerError error;
	NpPhysics* npPhysics = &NpPhysics::getInstance();

	error = mPvdConnection->createInstance(PvdClassKeys::ClothFabric+1, PX_PROFILE_POINTER_TO_U64(fabric), PVD::EInstanceUIFlags::None);
	error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(npPhysics)+SdkGroups::ClothFabrics+1, PX_PROFILE_POINTER_TO_U64(fabric));
	mMetaDataBinding.sendAllProperties( mPvdConnection, fabric );
	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}

void VisualDebugger::releasePvdInstance(const PxClothFabric* fabric)
{
	mPvdConnection->destroyInstance(PX_PROFILE_POINTER_TO_U64(fabric));
}


void VisualDebugger::releasePvdInstance(const void* ptr)
{
	PVD::PvdCommLayerError error;
	error = mPvdConnection->destroyInstance(PX_PROFILE_POINTER_TO_U64(ptr));
	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void VisualDebugger::createPvdInstance(const PxMaterial* mat)
{
	mPvdConnection->createInstance(PvdClassKeys::Material+1, PX_PROFILE_POINTER_TO_U64(mat), PVD::EInstanceUIFlags::None);
	updatePvdProperties( mat );
	mPvdConnection->flush();
}

void VisualDebugger::updatePvdProperties(const PxMaterial* mat)
{
	mMetaDataBinding.sendAllProperties( mPvdConnection, mat );
}

void VisualDebugger::releasePvdInstance(const PxMaterial* mat)
{
	mPvdConnection->destroyInstance( PX_PROFILE_POINTER_TO_U64(mat) );
}

void VisualDebugger::flush() 
{
	mPvdConnection->flush();
}


} // namespace Pvd

}

#endif  // PX_SUPPORT_VISUAL_DEBUGGER
