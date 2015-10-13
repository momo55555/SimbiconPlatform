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
#ifndef PX_REPXUTILITY_H
#define PX_REPXUTILITY_H
#include "PxPhysics.h"
#include "extensions/PxJointRepXExtensions.h"
#include "RepXCoreExtensions.h"
#include "PxScene.h"
#include "PsInlineArray.h"
#include "PxRigidStatic.h"
#include "PxRigidDynamic.h"
#include "PxSerialFramework.h"
#include "PxConvexMesh.h"
#include "PxTriangleMesh.h"
#include "PxMaterial.h"
#include "PxHeightField.h"
#include "PxArticulation.h"
#include "RepXUpgrader.h"
#include "PxClothFabric.h"
#include "PxCloth.h"

namespace physx { namespace repx {
	using namespace physx::shdfnd;
	
inline PxU32 getObjectCount( PxMaterial*, PxPhysics& inPhysics ) { return inPhysics.getNbMaterials(); }
inline PxU32 getObjectCount( PxConvexMesh*, PxPhysics& inPhysics ) { return inPhysics.getNbConvexMeshes(); }
inline PxU32 getObjectCount( PxTriangleMesh*, PxPhysics& inPhysics ) { return inPhysics.getNbTriangleMeshes(); }
inline PxU32 getObjectCount( PxHeightField*, PxPhysics& inPhysics ) { return inPhysics.getNbHeightFields(); }
inline PxU32 getObjectCount( PxRigidStatic*, PxScene& inPhysics ) { return inPhysics.getNbActors(PxActorTypeSelectionFlag::eRIGID_STATIC); }
inline PxU32 getObjectCount( PxRigidDynamic*, PxScene& inPhysics ) { return inPhysics.getNbActors(PxActorTypeSelectionFlag::eRIGID_DYNAMIC); }
inline PxU32 getObjectCount( PxArticulation*, PxScene& inPhysics ) { return inPhysics.getNbArticulations(); }

template<typename TArrayType>
inline void getObjects( PxMaterial*, PxPhysics& inPhysics, TArrayType& inArray, PxU32 inObjCount )
{
	inArray.resize( inObjCount );
	inPhysics.getMaterials( inArray.begin(), inObjCount );
}
template<typename TArrayType>
inline void getObjects( PxConvexMesh*, PxPhysics& inPhysics, TArrayType& inArray, PxU32 inObjCount )
{
	inArray.resize( inObjCount );
	inPhysics.getConvexMeshes( inArray.begin(), inObjCount );
}
template<typename TArrayType>
inline void getObjects( PxTriangleMesh*, PxPhysics& inPhysics, TArrayType& inArray, PxU32 inObjCount )
{
	inArray.resize( inObjCount );
	inPhysics.getTriangleMeshes( inArray.begin(), inObjCount );
}

template<typename TArrayType>
inline void getObjects( PxHeightField*, PxPhysics& inPhysics, TArrayType& inArray, PxU32 inObjCount )
{
	inArray.resize( inObjCount );
	inPhysics.getHeightFields( inArray.begin(), inObjCount );
}

template<typename TArrayType>
inline void getObjects( PxRigidStatic*, PxScene& inPhysics, TArrayType& inArray, PxU32 inObjCount )
{
	inArray.resize( inObjCount );
	PxRigidStatic** arrayPtr = inArray.begin();
	inPhysics.getActors( PxActorTypeSelectionFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(arrayPtr), inObjCount );
}
template<typename TArrayType>
inline void getObjects( PxRigidDynamic*, PxScene& inPhysics, TArrayType& inArray, PxU32 inObjCount )
{
	inArray.resize( inObjCount );
	PxRigidDynamic** arrayPtr = inArray.begin();
	inPhysics.getActors( PxActorTypeSelectionFlag::eRIGID_DYNAMIC, reinterpret_cast<PxActor**>(arrayPtr), inObjCount );
}
template<typename TArrayType>
inline void getObjects( PxArticulation*, PxScene& inPhysics, TArrayType& inArray, PxU32 inObjCount )
{
	inArray.resize( inObjCount );
	inPhysics.getArticulations( inArray.begin(), inObjCount );
}



template<typename TObjectType, typename TPhysicsType>
static inline void addItemsToRepX( TObjectType* crapPtr, TPhysicsType& inPhysics, RepXCollection* inCollection, RepXIdToRepXObjectMap* inIdMap )
{
	InlineArray<TObjectType*,20> theData;
	PxU32 objCount = getObjectCount( crapPtr, inPhysics );
	getObjects( crapPtr, inPhysics, theData, objCount );
	for ( PxU32 idx = 0; idx < theData.size(); ++idx )
		addToRepXCollectionNF( inCollection, inIdMap, theData[idx] );
}

/**
 *	This function does not save joint information!!!
 *	If you have PxJoints you need to save them separately!!!
 */
static inline void addItemsToRepX( PxPhysics& inPhysics, PxScene& inScene, RepXIdToRepXObjectMap* inIdMap, RepXCollection* inCollection  )
{
	//Run over all the buffers in physics, add them to collection
	//Run over all rigid statics, dynamics, and articulations in scene
	
	PxVec3 theUpVector = inScene.getGravity();
	theUpVector = -theUpVector;
	theUpVector.normalize();
	//Assume the up vector is the opposite of the gravity.
	inCollection->setUpVector( theUpVector );
	//Finally, run over all joints.
	//Not sure how to get all the joints from a scene.
	addItemsToRepX( (PxMaterial*)NULL, inPhysics, inCollection, inIdMap );
	addItemsToRepX( (PxConvexMesh*)NULL, inPhysics, inCollection, inIdMap );
	addItemsToRepX( (PxTriangleMesh*)NULL, inPhysics, inCollection, inIdMap );
	addItemsToRepX( (PxHeightField*)NULL, inPhysics, inCollection, inIdMap );
	addItemsToRepX( (PxRigidStatic*)NULL, inScene, inCollection, inIdMap );
	addItemsToRepX( (PxRigidDynamic*)NULL, inScene, inCollection, inIdMap );
	addItemsToRepX( (PxArticulation*)NULL, inScene, inCollection, inIdMap );
}

//Add repx items to a scene.
struct RepXCoreItemAdder
{
	PxScene* mScene;
	RepXCoreItemAdder( PxScene* inScene )
		: mScene( inScene )
	{
	}
	void operator()( const void* inId, PxConvexMesh* ) {}
	void operator()( const void* inId, PxTriangleMesh* ) {}
	void operator()( const void* inId, PxHeightField* ) {}
	void operator()( const void* inId, PxClothFabric* ) {}
	void operator()( const void* inId, PxMaterial* ) {}
	void operator()( const void* inId, PxRigidStatic* inActor ) { mScene->addActor( *inActor ); }
	void operator()( const void* inId, PxRigidDynamic* inActor ) { mScene->addActor( *inActor ); }
	void operator()( const void* inId, PxArticulation* inArticulation ) { mScene->addArticulation( *inArticulation ); }
	void operator()( const void* inId, PxCloth* inData ) { mScene->addActor( *inData ); }
	void operator()( const void* inId, PxJoint* inJoint ) {}
};

template<typename TFirstOpType, typename TSecondOpType>
struct ComposingOperator
{
	TFirstOpType mFirst;
	TSecondOpType mSecond;
	ComposingOperator( TFirstOpType inFirst, TSecondOpType inSecond )
		: mFirst( inFirst )
		, mSecond( inSecond )
	{
	}
	template<typename TDataType>
	inline void operator()( const void* inId, TDataType* inData ) 
	{
		mFirst( inId, inData );
		mSecond( inId, inData );
	}
};

template<typename TOperatorType>
struct JointRepXVisitor
{
	TOperatorType mOperator;
	JointRepXVisitor( TOperatorType inOp ) : mOperator( inOp ) {}
	template<typename TDataType>
	inline bool operator()( const void* inId, TDataType* inData ) { mOperator( inId, inData ); return true; }
	inline bool operator()( const void*, void*, const char*) { return false; }
};

template<typename TOperatorType>
struct CoreRepXVisitor
{
	TOperatorType mOperator;
	CoreRepXVisitor( TOperatorType inOp ) : mOperator( inOp ) {}
	template<typename TDataType>
	inline bool operator()( const void* inId, TDataType* inData ) { mOperator( inId, inData ); return true; }
	inline bool operator()( const void* inId, void* inLiveObject, const char* inRepXExtensionName) 
	{ 
		return visitJointRepXObject<bool>( inId, inLiveObject, inRepXExtensionName, JointRepXVisitor<TOperatorType>(mOperator) );
	}
};

template<typename TOperatorType>
struct GenericInstantiationHandler : public RepXInstantiationResultHandler
{
	TOperatorType mOperator;
	GenericInstantiationHandler( TOperatorType inOperator ) : mOperator( inOperator ) {}
	virtual void addInstantiationResult( RepXInstantiationResult inResult ) 
	{
		visitCoreRepXObject<bool>( inResult.mCollectionId, inResult.mLiveObject, inResult.mExtensionName, CoreRepXVisitor<TOperatorType>(mOperator) );
	}
};


template<typename TOperator>
static inline void instantiateCollection( RepXCollection* inCollection, RepXIdToRepXObjectMap* inIdMap, PxPhysics* inPhysics, PxCooking* inCooking, PxStringTable* inStringTable, bool inAddOriginalIdsToObjectMap, TOperator inInstantiationOperator )
{
	RepXInstantiationArgs theInstantiationArgs( inCooking, inPhysics, inStringTable );
	GenericInstantiationHandler<TOperator> theHandler( inInstantiationOperator );
	inCollection->instantiateCollection( theInstantiationArgs, inIdMap, &theHandler, inAddOriginalIdsToObjectMap );
}

template<typename TOperator>
static inline void instantiateCollection( RepXCollection* inCollection, PxPhysics* inPhysics, PxCooking* inCooking, PxStringTable* inStringTable, bool inAddOriginalIdsToObjectMap, TOperator inInstantiationOperator )
{
	RepXScopedIdToRepXObjectMap theMap( RepXIdToRepXObjectMap::create(getFoundation().getAllocator()) );
	RepXInstantiationArgs theInstantiationArgs( inCooking, inPhysics, inStringTable );
	GenericInstantiationHandler<TOperator> theHandler( inInstantiationOperator );
	inCollection->instantiateCollection( theInstantiationArgs, theMap, &theHandler, inAddOriginalIdsToObjectMap );
}

static inline void addObjectsToScene( RepXCollection* inCollection, PxPhysics* inPhysics, PxCooking* inCooking, PxScene* inScene, PxStringTable* inStringTable )
{
	instantiateCollection( inCollection, inPhysics, inCooking, inStringTable, true, RepXCoreItemAdder( inScene ) );
}

//Add repx items to a scene.
struct RepXPxCollectionCoreItemAdder
{
	PxCollection* mBufferCollection;
	PxCollection* mSceneCollection;
	RepXPxCollectionCoreItemAdder( PxCollection* bufCol, PxCollection* sceneCol )
		: mBufferCollection( bufCol )
		, mSceneCollection( sceneCol )
	{
	}

	void addBuffer( const void* inId, PxSerializable* item )
	{
		item->collectForExport( *mBufferCollection );
		mBufferCollection->setUserData( item, const_cast<void*>( inId ) );
		mSceneCollection->addExternalRef( item, const_cast<void*>( inId ) );
	}

	void addSceneObject( const void* inId, PxSerializable* item ) 
	{
		item->collectForExport( *mSceneCollection );
		mSceneCollection->setUserData( item, const_cast<void*>( inId ) );
	}
	void operator()( const void* inId, PxClothFabric* data ) { addBuffer( inId, data ); }
	void operator()( const void* inId, PxConvexMesh* mesh )  { addBuffer( inId, mesh ); }
	void operator()( const void* inId, PxTriangleMesh* mesh)  { addBuffer( inId, mesh ); }
	void operator()( const void* inId, PxHeightField* mesh) { addBuffer( inId, mesh ); }
	void operator()( const void* inId, PxMaterial* material ) { addBuffer( inId, material ); }
	void operator()( const void* inId, PxRigidStatic* inActor ) { addSceneObject( inId, inActor ); }
	void operator()( const void* inId, PxRigidDynamic* inActor ) { addSceneObject( inId, inActor ); }
	void operator()( const void* inId, PxArticulation* inArticulation ) { addSceneObject( inId, inArticulation ); }
	void operator()( const void* inId, PxJoint* inJoint ) { addSceneObject( inId, inJoint ); }
	void operator()( const void* inId, PxCloth* inCloth ) { addSceneObject( inId, inCloth ); }
};

//repx->pxcollection in a manner that allows you to insert the same collection into the same scene multiple times.
static inline void addObjectsToPxCollection( RepXCollection* srcRepxCollection
												, PxPhysics* inPhysics
												, PxCooking* inCooking
												, PxStringTable* inStringTable
												, PxCollection* outBuffers
												, PxCollection* outSceneObjects )
{
	instantiateCollection( srcRepxCollection, inPhysics, inCooking, inStringTable, true, RepXPxCollectionCoreItemAdder( outBuffers, outSceneObjects ) );
}

/**
 *	This function does not save joint information!!!
 *	If you have PxJoints you need to save them separately!!!
 */
static inline void saveSceneToRepX( PxPhysics& inPhysics, PxScene& inScene, RepXCollection* inCollection )
{
	RepXIdToRepXObjectMap* theIdMap( RepXIdToRepXObjectMap::create(getFoundation().getAllocator()) );
	RepXCollection* theCollection( inCollection );
	addItemsToRepX( inPhysics, inScene, theIdMap, theCollection );
	theIdMap->destroy();
}


typedef HashMap<void*, PxU32> TOutstandingAllocationMap;


//Use this allocator to avoid memory leaks when using repx.
//All char* name properties that are set will definitely leak
//without it.
class RepXCollectionAllocator : public PxAllocatorCallback, public UserAllocated
{
	TOutstandingAllocationMap	mAllocations;
	PxAllocatorCallback&		mCallback;
public:

	RepXCollectionAllocator( PxAllocatorCallback& inCallback )
		: mCallback( inCallback )
	{
	}

	~RepXCollectionAllocator()
	{
		for( TOutstandingAllocationMap::Iterator iter = mAllocations.getIterator();
			iter.done() == false;
			++iter )
		{
			mCallback.deallocate( iter->first );
		}
	}
	
	virtual void* allocate(size_t size, const char* typeName, const char* filename, int line)
	{
		void* retval = mCallback.allocate( size, typeName, filename, line );
		mAllocations.insert( retval, 0 );
		return retval;
	}
	virtual void deallocate(void* ptr)
	{
		mAllocations.erase( ptr );
		mCallback.deallocate( ptr );
	}
};

static inline PxU32 buildExtensionList( RepXExtension** inExtensionBuffer, PxU32 inBufferSize, PxAllocatorCallback& inCallback )
{
	PX_ASSERT( inBufferSize > getNumCoreExtensions() + getNumJointExtensions() );
	PxU32 totalCreated = 0;
	PxU32 numCreated = createCoreExtensions( inExtensionBuffer, inBufferSize, inCallback );
	totalCreated += numCreated;
	inBufferSize -= numCreated;
	numCreated = createJointExtensions( inExtensionBuffer + numCreated, inBufferSize, inCallback );
	totalCreated += numCreated;
	return totalCreated;
}

static inline RepXCollection* createCollection(const PxTolerancesScale& inScale, PxAllocatorCallback& inCallback)
{
	RepXExtension* theExtensions[64];
	PxU32 numExtensions = buildExtensionList( theExtensions, 64, inCallback );
	return RepXCollection::create( theExtensions, numExtensions, inScale, inCallback );
}

static inline RepXCollection* createCollection(const PxTolerancesScale& inScale)
{
	return createCollection( inScale, getFoundation().getAllocator() );
}

static inline RepXCollection* createCollection(const char* inPath, PxAllocatorCallback& inCallback)
{
	RepXExtension* theExtensions[64];
	PxU32 numExtensions = buildExtensionList( theExtensions, 64, inCallback );
	RepXCollection* retval = RepXCollection::create( inPath, theExtensions, numExtensions, inCallback );
	if ( retval )
		retval = &RepXUpgrader::upgradeCollection( *retval );
	return retval;
}

static inline RepXCollection* createCollection(const char* inPath)
{
	return createCollection( inPath, getFoundation().getAllocator() );
}

}}
#endif