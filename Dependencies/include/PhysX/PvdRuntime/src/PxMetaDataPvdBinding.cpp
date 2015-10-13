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

#include "PxSimpleTypes.h"
#include "PsArray.h"
//using namespace physx::pubfnd;
//using namespace physx::shdfnd;
#include "PxVisualDebugger.h"
#include "PVDCommLayerDatatypes.h"
#include "PxMetaDataPvdBinding.h"
#include "PxMetaDataObjects.h"
#include "PvdConnection.h"
#include "PvdDataStream.h"
#include "PxScene.h"
#include "PvdClassDefinitions.h"
#include "PvdClassDefinitionsRigidBody.h"
#include "ScShapeIterator.h"
#include "ScBodyCore.h"
#include "ScDeformableCore.h"
#include "PvdMetaDataExtensions.h"
#include "PvdMetaDataPropertyVisitor.h"
#include "PvdMetaDataDefineProperties.h"
#include "PvdMetaDataSendProperties.h"
#include "PvdMetaDataBindingData.h"
#include "PxRigidDynamic.h"
#include "PxArticulation.h"
#include "PxArticulationLink.h"

using namespace PVD;
using namespace physx;
using namespace Sc;

namespace physx
{
namespace Pvd
{

#define DEFINE_TYPE_TO_CLASS_MAP( datatype, classKey ) \
	template<> struct PvdClassForType<datatype> { PxU32 PvdClass; PvdClassForType<datatype>() : PvdClass(  PvdClassKeys::classKey + 1 ) {} };

DEFINE_TYPE_TO_CLASS_MAP( PxVec3, VectorArray );
DEFINE_TYPE_TO_CLASS_MAP( PxF32, FloatArray );
DEFINE_TYPE_TO_CLASS_MAP( PxU32, U32Array );
DEFINE_TYPE_TO_CLASS_MAP( PxU16, U16Array );
DEFINE_TYPE_TO_CLASS_MAP( PxClothFabricPhaseType::Enum, ClothFabricPhaseTypeArray );
DEFINE_TYPE_TO_CLASS_MAP( PxClothPhaseSolverConfig::SolverType, ClothPhaseSolverConfigArray );
DEFINE_TYPE_TO_CLASS_MAP( PxParticleFlags, ParticleFlagsArray );
	

DECLARE_BUFFER_PROPERTY( DeformableBulkData,		PxDeformable,	  const Array<PxVec3>&,						Positions,	positions, PxDeformableReadDataFlag::ePOSITION_BUFFER );
DECLARE_BUFFER_PROPERTY( DeformableBulkData,		PxDeformable,	  const Array<PxVec3>&,						Velocities, velocities, PxDeformableReadDataFlag::eVELOCITY_BUFFER );
DECLARE_BUFFER_PROPERTY( DeformableBulkData,		PxDeformable,	  const Array<PxVec3>&,						Normals,	normals, PxDeformableReadDataFlag::eNORMAL_BUFFER );
DECLARE_BUFFER_PROPERTY( DeformableBulkData,		PxDeformable,	  const Array<PxF32>&,						InverseMasses,	invMasses, PxDeformableReadDataFlag::eINVERSE_MASS_BUFFER );
DECLARE_BUFFER_PROPERTY( DeformableBulkData,		PxDeformable,	  const Array<PxU32>&,						ParentIndexes,	parentIndices, PxDeformableReadDataFlag::ePARENT_INDEX_BUFFER );
DECLARE_BUFFER_PROPERTY( DeformableBulkData,		PxDeformable,	  const Array<PxU32>&,						Indexes,	indices, PxDeformableReadDataFlag::eINDEX_BUFFER );
DECLARE_BUFFER_PROPERTY( DeformableBulkData,		PxDeformable,	  const Array<PxDeformablePrimitiveSplitPair>&,	 PrimitiveSplitPairs,	primitiveSplitPairs, PxDeformableReadDataFlag::ePRIMITIVE_SPLIT_PAIR_BUFFER );



template<typename TOperator>
inline void visitDeformableBufferProperties( TOperator inOperator )
{
	inOperator( PxDeformablePositionsProperty(), 0 );
	inOperator( PxDeformableVelocitiesProperty(), 1 );
	inOperator( PxDeformableInverseMassesProperty(), 2 );
	inOperator( PxDeformableParentIndexesProperty(), 3 );
	inOperator( PxDeformableIndexesProperty(), 4 );
	//Don't iterate through this one just yet.
	//theFilter( PxDeformablePrimitiveSplitPairsProperty(), 5 );
}



PvdMetaDataBinding::PvdMetaDataBinding() 
		: mBindingData( PX_NEW( PvdMetaDataBindingData )() )
{
}

PvdMetaDataBinding::~PvdMetaDataBinding()
{
	PX_DELETE( mBindingData );
	mBindingData = NULL;
}

template<typename TDataType, typename TValueType>
inline void definePropertyStruct( PVD::PvdDataStream* inStream, PxU32 inClassKey, PxU32 inStructKey )
{
	PvdPropertyDefinitionHelper& helper( inStream->getPropertyDefinitionHelper() );
	PvdClassInfoValueStructDefine definitionObj( helper );
	visitAllPvdProperties<TDataType>( definitionObj );
	helper.definePropertyStruct( inStructKey + 1, inClassKey + 1, sizeof(TValueType) );
}

bool PvdMetaDataBinding::registerPropertyOverride( PVD::PvdDataStream* inStream, PxU32 inClassKey ) 
{ 
	//This only happens once, so we are going to 
	PvdPropertyDefinitionHelper& helper( inStream->getPropertyDefinitionHelper() );
	PvdClassInfoDefine definitionObj( helper, inClassKey + 1 );
	if ( inClassKey == PvdClassKeys::Scene )
	{

		visitAllPvdProperties<PxSceneDesc>( definitionObj );
		helper.pushName( "SimulationStatistics" );
		visitAllPvdProperties<PxSimulationStatistics>( definitionObj );
		helper.popName();
		inStream->defineProperty( definitionObj.mClassKey, "Frame", "", PvdCommLayerDatatype::Section, PxPvdOnlyProperties::PxScene_Frame );
		inStream->defineArrayProperty( definitionObj.mClassKey, "Contacts", PvdClassKeys::ContactsArray+1, PxPvdOnlyProperties::PxScene_Contacts );
		
		definePropertyStruct<PxSceneDesc,PxSceneDescGeneratedValues>( inStream, inClassKey, inClassKey );
		definePropertyStruct<PxSimulationStatistics,PxSimulationStatisticsGeneratedValues>( inStream, inClassKey, PvdClassKeys::SimulationStatistics );
		return true;
	}
#define DEFINE_PROPERTY_STRUCT( pxclassname ) definePropertyStruct<pxclassname,pxclassname##GeneratedValues>( inStream, inClassKey, inClassKey );
	else if ( inClassKey == PvdClassKeys::PhysicsSDK )
	{
		helper.pushName( "TolerancesScale" );
		visitAllPvdProperties<PxTolerancesScale>( definitionObj );
		helper.popName();
		inStream->defineProperty( definitionObj.mClassKey, "Version.Major", "", PvdCommLayerDatatype::U32, 1 );
		inStream->defineProperty( definitionObj.mClassKey, "Version.Minor", "", PvdCommLayerDatatype::U32, 2 );
		inStream->defineProperty( definitionObj.mClassKey, "Version.Bugfix", "", PvdCommLayerDatatype::U32, 3 );
		//picking an unused struct id.
		definePropertyStruct<PxTolerancesScale,PxTolerancesScaleGeneratedValues>( inStream, inClassKey, PvdClassKeys::Group ); //
		return true;
	}

	else if ( inClassKey == PvdClassKeys::SimulationStatistics ) return true;


	else if ( inClassKey == PvdClassKeys::Material )
	{
		visitAllPvdProperties<PxMaterial>( definitionObj ); 
		DEFINE_PROPERTY_STRUCT( PxMaterial );
		return true;
	}
	else if ( inClassKey == PvdClassKeys::Shape )
	{
		visitAllPvdProperties<PxShape>( definitionObj );
		DEFINE_PROPERTY_STRUCT( PxShape );
		return true;
	}
	else if ( inClassKey == PvdClassKeys::Actor )
	{
		visitInstancePvdProperties<PxActor>( definitionObj ); 
		return true;
	}
	else if ( inClassKey == PvdClassKeys::RigidActor )
	{
		visitInstancePvdProperties<PxRigidActor>( definitionObj ); return true;
	}
	else if ( inClassKey == PvdClassKeys::RigidStatic )
	{
		visitInstancePvdProperties<PxRigidStatic>( definitionObj ); 
		DEFINE_PROPERTY_STRUCT( PxRigidStatic );
		return true;
	}
	else if ( inClassKey == PvdClassKeys::RigidBody )
	{
		visitInstancePvdProperties<PxRigidBody>( definitionObj ); return true;
	}
	else if ( inClassKey == PvdClassKeys::RigidDynamic )
	{
		visitInstancePvdProperties<PxRigidDynamic>( definitionObj ); 
		DEFINE_PROPERTY_STRUCT( PxRigidDynamic );
		return true;
	}
	else if ( inClassKey == PvdClassKeys::Articulation)
	{
		visitInstancePvdProperties<PxArticulation>( definitionObj ); 
		DEFINE_PROPERTY_STRUCT( PxArticulation );
		return true;
	}
	else if ( inClassKey == PvdClassKeys::ArticulationJoint )
	{
		visitInstancePvdProperties<PxArticulationJoint>( definitionObj ); 
		DEFINE_PROPERTY_STRUCT( PxArticulationJoint );
		return true;
	}
	else if ( inClassKey == PvdClassKeys::ArticulationLink )
	{
		visitInstancePvdProperties<PxArticulationLink>( definitionObj ); 
		DEFINE_PROPERTY_STRUCT( PxArticulationLink );
		return true;
	}
	else if ( inClassKey == PvdClassKeys::ParticleSystem )
	{
		visitInstancePvdProperties<PxParticleBase>( definitionObj ); 
		visitInstancePvdProperties<PxParticleSystem>( definitionObj ); 
		visitParticleSystemBufferProperties( makePvdPropertyFilter( definitionObj ) );
		DEFINE_PROPERTY_STRUCT( PxParticleSystem );
		return true;
	}
	else if ( inClassKey == PvdClassKeys::ParticleFluid )
	{
		visitInstancePvdProperties<PxParticleFluid>( definitionObj );
		visitParticleFluidBufferProperties( makePvdPropertyFilter( definitionObj ) ); 
		DEFINE_PROPERTY_STRUCT( PxParticleFluid );
		return true;
	}
	else if ( inClassKey == PvdClassKeys::Deformable )
	{
		visitInstancePvdProperties<PxDeformable>( definitionObj );
		DEFINE_PROPERTY_STRUCT( PxDeformable ); 
		return true;
	}
	else if ( inClassKey == PvdClassKeys::ClothFabric )
	{
		visitInstancePvdProperties<PxClothFabric>( definitionObj );
		DEFINE_PROPERTY_STRUCT( PxClothFabric );
		return true;
	}
	else if ( inClassKey == PvdClassKeys::ClothParticle )
	{
		visitInstancePvdProperties<PxClothParticle>( definitionObj );
		DEFINE_PROPERTY_STRUCT( PxClothParticle );
		return true;
	}
	else if ( inClassKey == PvdClassKeys::Cloth )
	{
		visitInstancePvdProperties<PxCloth>( definitionObj );
		DEFINE_PROPERTY_STRUCT( PxCloth );
		inStream->defineArrayProperty( inClassKey + 1, "ParticleBuffer", PvdClassKeys::ClothParticle + 1, PxPvdOnlyProperties::PxCloth_ParticleBuffer );
		return true;
	}
	return false; 
}

void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxScene* inScene )
{
	PxTolerancesScale theScale;
	PxSceneDesc theDesc( theScale );
	inScene->saveToDesc( theDesc );
	PxSceneDescGeneratedValues theValues( &theDesc );
	inStream->sendPropertyStruct( PX_PROFILE_POINTER_TO_U64( inScene ), PvdClassKeys::Scene + 1, reinterpret_cast<PxU8*>( &theValues ), sizeof( theValues ) );
}

template<typename TValueType, typename TDataType>
void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const TDataType* inDatatype, PxU32 inClassKey, PxU64 inInstanceId )
{
	TValueType theValues( inDatatype );
	inStream->sendPropertyStruct( inInstanceId, inClassKey + 1, reinterpret_cast<PxU8*>( &theValues ), sizeof( theValues ) );
}

template<typename TValueType, typename TDataType>
void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const TDataType* inDatatype, PxU32 inClassKey )
{
	sendAllProperties<TValueType, TDataType>( inStream, inDatatype, inClassKey, PX_PROFILE_POINTER_TO_U64( inDatatype ) );
}

void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxPhysics* inPhysics )
{
	PxTolerancesScale theScale( inPhysics->getTolerancesScale() );
	PxU64 instanceId( PX_PROFILE_POINTER_TO_U64( inPhysics ) );
	sendAllProperties<PxTolerancesScaleGeneratedValues>( inStream, &theScale, PvdClassKeys::Group, instanceId );
	inStream->setPropertyValue( instanceId, 1, (PxU32)PX_PHYSICS_VERSION_MAJOR );
	inStream->setPropertyValue( instanceId, 2, (PxU32)PX_PHYSICS_VERSION_MINOR );
	inStream->setPropertyValue( instanceId, 3, (PxU32)PX_PHYSICS_VERSION_BUGFIX );
}

void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxMaterial* inMaterial )
{
	sendAllProperties<PxMaterialGeneratedValues>( inStream, inMaterial, PvdClassKeys::Material );
}

void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxShape* inMaterial )
{
	sendAllProperties<PxShapeGeneratedValues>( inStream, inMaterial, PvdClassKeys::Shape );
}

void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxRigidStatic* inObj )
{
	sendAllProperties<PxRigidStaticGeneratedValues>( inStream, inObj, PvdClassKeys::RigidStatic );
}
void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxRigidDynamic* inObj )
{
	sendAllProperties<PxRigidDynamicGeneratedValues>( inStream, inObj, PvdClassKeys::RigidDynamic );
}
void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxArticulation* inObj )
{
	sendAllProperties<PxArticulationGeneratedValues>( inStream, inObj, PvdClassKeys::Articulation );
}
void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxArticulationLink* inObj )
{
	sendAllProperties<PxArticulationLinkGeneratedValues>( inStream, inObj, PvdClassKeys::ArticulationLink );
}
void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxArticulationJoint* inObj )
{
	sendAllProperties<PxArticulationJointGeneratedValues>( inStream, inObj, PvdClassKeys::ArticulationJoint );
}

void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxParticleSystem* inObj )
{
	sendAllProperties<PxParticleSystemGeneratedValues>( inStream, inObj, PvdClassKeys::ParticleSystem );
}
void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxParticleFluid* inObj )
{
	sendAllProperties<PxParticleFluidGeneratedValues>( inStream, inObj, PvdClassKeys::ParticleFluid );
}
void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxDeformable* inObj )
{
	sendAllProperties<PxDeformableGeneratedValues>( inStream, inObj, PvdClassKeys::Deformable );
}


template<typename TObjType>
struct CollectionOperator
{
	Array<PxU8>&	mTempArray;
	PxU64			mInstance;
	const TObjType* mObject;
	PvdDataStream*	mStream;

	CollectionOperator( Array<PxU8>& ary, PxU64 inst, const TObjType* obj, PvdDataStream* stream ) : mTempArray( ary ), mInstance( inst ), mObject( obj ), mStream( stream ) {}
	CollectionOperator( const CollectionOperator& other ) : mTempArray( other.mTempArray ), mInstance( other.mInstance ), mObject( other.mObject ), mStream( other.mStream ) {}
	void pushName( const char* ) {}
	void popName() {}
	template< typename TAccessor > void simpleProperty(PxU32 key, const TAccessor& ) {}
	template< typename TAccessor > void flagsProperty(PxU32 key, const TAccessor&, const PxU32ToName* ) {}

	template<typename TColType, typename TDataType, typename TCollectionProp >
	void handleCollection( PxU32 key, const TCollectionProp& prop, PvdCommLayerDatatype dtype, PxU32 countMultiplier = 1 )
	{
		PxU32 count = prop.size( mObject );
		mTempArray.resize( count * sizeof( TColType ) );
		TDataType* start = reinterpret_cast<TDataType*>( mTempArray.begin() );
		prop.get( mObject, start, count * countMultiplier );
		//All the array classes have a single property as their main property
		//And it's id is always one.
		PxU32 theProperties = 1;
		mStream->beginArrayPropertyBlock(mInstance, key, &theProperties, &dtype, 1);
		mStream->sendArrayObjects( mTempArray.begin(), sizeof( TColType ), count );
		mStream->endArrayPropertyBlock();
	}
	template< PxU32 TKey, typename TObject, typename TColType >
	void handleCollection( const PxReadOnlyCollectionPropertyInfo<TKey,TObject,TColType>& prop )
	{
		handleCollection<TColType, TColType>( TKey, prop, getDatatypeForType<TColType>() );
	}
	//Enumerations or bitflags.
	template< PxU32 TKey, typename TObject, typename TColType >
	void handleCollection( const PxReadOnlyCollectionPropertyInfo<TKey,TObject,TColType>& prop, const PxU32ToName* )
	{
		PX_COMPILE_TIME_ASSERT( sizeof( TColType ) == sizeof( PxU32 ) );
		handleCollection<PxU32, TColType>( TKey, prop, getDatatypeForType<EnumerationValue>() );
	}
};

struct PxClothFabricCollectionOperator : CollectionOperator<PxClothFabric>
{
	PxClothFabricCollectionOperator( Array<PxU8>& ary, PxU64 inst, const PxClothFabric* obj, PvdDataStream* stream ) 
		: CollectionOperator<PxClothFabric>( ary, inst, obj, stream ) {}
	PxClothFabricCollectionOperator( const PxClothFabricCollectionOperator& other ) : CollectionOperator<PxClothFabric>( other ) {}
	
	template< PxU32 TKey, typename TObject, typename TColType >
	void handleCollection( const PxReadOnlyCollectionPropertyInfo<TKey,TObject,TColType>& prop )
	{
		CollectionOperator<PxClothFabric>::handleCollection<TColType, TColType>( TKey, prop, getDatatypeForType<TColType>(), sizeof( TColType ) );
	}

	//Enumerations or bitflags.
	template< PxU32 TKey, typename TObject, typename TColType >
	void handleCollection( const PxReadOnlyCollectionPropertyInfo<TKey,TObject,TColType>& prop, const PxU32ToName* )
	{
		PX_COMPILE_TIME_ASSERT( sizeof( TColType ) == sizeof( PxU32 ) );
		CollectionOperator<PxClothFabric>::handleCollection<PxU32, TColType>( TKey, prop, getDatatypeForType<EnumerationValue>() );
	}
};

void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxClothFabric* fabric )
{
	sendAllProperties<PxClothFabricGeneratedValues>( inStream, fabric, PvdClassKeys::ClothFabric );
	PxClothFabricCollectionOperator op( mBindingData->mTempU8Array, PX_PROFILE_POINTER_TO_U64( fabric ), fabric, inStream );
	visitInstancePvdProperties<PxClothFabric>( op );
}

void PvdMetaDataBinding::sendAllProperties( PVD::PvdDataStream* inStream, const PxCloth* cloth )
{
	sendAllProperties<PxClothGeneratedValues>( inStream, cloth, PvdClassKeys::Cloth );
	CollectionOperator<PxCloth> op( mBindingData->mTempU8Array, PX_PROFILE_POINTER_TO_U64( cloth ), cloth, inStream );
	visitInstancePvdProperties<PxCloth>( op );
}

void PvdMetaDataBinding::sendBeginFrame( PVD::PvdDataStream* inStream, const PxScene* inScene )
{
	inStream->setPropertyValue( PX_PROFILE_POINTER_TO_U64( inScene ), PxPvdOnlyProperties::PxScene_Frame, createSection( SectionType::Begin ) );
}


static void beginContactArray( PVD::PvdDataStream* inStream, const PxScene* inScene )
{
	PxU32 properties[ContactsArrayProp::NUM_ELEMENTS];
	PVD::PvdCommLayerDatatype dataTypes[ContactsArrayProp::NUM_ELEMENTS] = {PVD::PvdCommLayerDatatype::Float3,
																				PVD::PvdCommLayerDatatype::Float3,
																				PVD::PvdCommLayerDatatype::ObjectId,
																				PVD::PvdCommLayerDatatype::ObjectId,
																				PVD::PvdCommLayerDatatype::Float,
																				PVD::PvdCommLayerDatatype::Float,
																				PVD::PvdCommLayerDatatype::U32,
																				PVD::PvdCommLayerDatatype::U32,
																				PVD::PvdCommLayerDatatype::Boolean};
	for(PxU32 i = 0; i < ContactsArrayProp::NUM_ELEMENTS; i++)
		properties[i] = i+1;

	inStream->beginArrayPropertyBlock(PX_PROFILE_POINTER_TO_U64(inScene), PxPvdOnlyProperties::PxScene_Contacts, properties, dataTypes, ContactsArrayProp::NUM_ELEMENTS);
}

void PvdMetaDataBinding::sendContacts( PVD::PvdDataStream* inStream, const PxScene* inScene )
{
	beginContactArray( inStream, inScene );
	inStream->endArrayPropertyBlock();
}

void PvdMetaDataBinding::sendContacts( PVD::PvdDataStream* inStream, const PxScene* inScene, Sc::ContactIterator& inContacts )
{
	beginContactArray( inStream, inScene );

	static const PxU32 NUM_STACK_ELT = 32;
	PvdContact stack[NUM_STACK_ELT];
	PvdContact* pvdContacts = stack;
	PvdContact* pvdContactsEnd = pvdContacts+NUM_STACK_ELT;

	
	PvdContact* curOut = pvdContacts;
	Sc::ContactIterator::Pair* pair;
	Sc::ContactIterator::Contact* contact;
	while ((pair = inContacts.getNextPair()))
	{
		while ((contact = pair->getNextContact()))
		{
			curOut->point = contact->point;
			curOut->axis = contact->normal;
			curOut->shape0 = PX_PROFILE_POINTER_TO_U64(contact->shape0);
			curOut->shape1 = PX_PROFILE_POINTER_TO_U64(contact->shape1);
			curOut->separation = contact->separation;
			curOut->normalForce = contact->normalForce;
			curOut->feature0 = contact->featureIndex0;
			curOut->feature1 = contact->featureIndex1;
			curOut->normalForceAvailable = contact->normalForceAvailable;

			curOut++;
			if(curOut == pvdContactsEnd)
			{
				inStream->sendArrayObjects((PxU8*)(pvdContacts), sizeof(PvdContact), NUM_STACK_ELT);
				curOut = pvdContacts;
			}
		}
	}

	if(curOut != pvdContacts)
		inStream->sendArrayObjects((PxU8*)(pvdContacts), sizeof(PvdContact), PxU32(curOut-pvdContacts));

	inStream->endArrayPropertyBlock();
}

void PvdMetaDataBinding::sendStats( PVD::PvdDataStream* inStream, const PxScene* inScene  )
{
	PxSimulationStatistics theStats;
	inScene->getSimulationStatistics( theStats );
	sendAllProperties<PxSimulationStatisticsGeneratedValues>( inStream, &theStats, PvdClassKeys::SimulationStatistics, PX_PROFILE_POINTER_TO_U64( inScene ) );
}

void PvdMetaDataBinding::sendEndFrame( PVD::PvdDataStream* inStream, const PxScene* inScene )
{
	inStream->setPropertyValue( PX_PROFILE_POINTER_TO_U64( inScene ), PxPvdOnlyProperties::PxScene_Frame, createSection( SectionType::End) );
}

struct DamnDangerousButFastPropertyBlock
{
	Transform	GlobalPose;
	Float3		LinearVelocity;
	Float3		AngularVelocity;
	PxU8		IsSleeping;
	PxU8		padding[3];
};

struct DynamicDataProvider
{
	const PxRigidDynamic* mActor;
	bool isSleeping() { return mActor->isSleeping(); }
	PxTransform getGlobalPose() { return mActor->getGlobalPose(); }
	PxVec3 getLinearVelocity() { return mActor->getLinearVelocity(); }
	PxVec3 getAngularVelocity() { return mActor->getAngularVelocity(); }
	PxU32 getBlockSize() { return 13 * 4 + 1; }
	PxU64 getId() { return PX_PROFILE_POINTER_TO_U64( mActor ); }
	const PxActor* getActor() { return mActor; }
};

struct LinkDataProvider
{
	const PxArticulationLink* mActor;
	bool mSleeping;
	bool isSleeping() { return mSleeping; }
	PxTransform getGlobalPose() { return mActor->getGlobalPose(); }
	PxVec3 getLinearVelocity() { return mActor->getLinearVelocity(); }
	PxVec3 getAngularVelocity() { return mActor->getAngularVelocity(); }
	PxU32 getBlockSize() { return 13 * 4; }
	PxU64 getId() { return PX_PROFILE_POINTER_TO_U64( mActor ); }
	const PxActor* getActor() { return mActor; }
};

template<typename TProvider>
static inline void UpdateActor( PVD::PvdDataStream* inStream, HashSet<PxActor*>& ioSleepingActors, DamnDangerousButFastPropertyBlock& theBlock, TProvider dataProvider )
{
	bool sleeping = dataProvider.isSleeping();
	PxActor * theActor( const_cast<PxActor*>( dataProvider.getActor() ) );
	bool wasSleeping = ioSleepingActors.contains( theActor );
	if ( sleeping == false 
		|| wasSleeping != sleeping )
	{
		theBlock.GlobalPose = createTransform( dataProvider.getGlobalPose() );
		theBlock.LinearVelocity = createFloat3( dataProvider.getLinearVelocity() );
		theBlock.AngularVelocity = createFloat3( dataProvider.getAngularVelocity() );
		theBlock.IsSleeping = sleeping ? 1 : 0;
		inStream->sendPropertyBlock( dataProvider.getId(), reinterpret_cast< const PxU8* >( &theBlock ), dataProvider.getBlockSize() );
	}
	if ( wasSleeping != sleeping )
	{
		if ( sleeping ) 
			ioSleepingActors.insert( theActor );
		else 
			ioSleepingActors.erase( theActor );
	}	
}

void PvdMetaDataBinding::updateDynamicActorsAndArticulations( PVD::PvdDataStream* inStream, const PxScene* inScene )
{

	PX_COMPILE_TIME_ASSERT( sizeof( DamnDangerousButFastPropertyBlock ) == 14 * 4 );
	DamnDangerousButFastPropertyBlock theBlock;
	{
		PxU32 actorCount = inScene->getNbActors( PxActorTypeSelectionFlag::eRIGID_DYNAMIC );
		if ( actorCount )
		{
			mBindingData->mActors.resize( actorCount );
			PxActor** theActors = mBindingData->mActors.begin();
			inScene->getActors( PxActorTypeSelectionFlag::eRIGID_DYNAMIC, theActors, actorCount );
			PxActor** lastActor = theActors + actorCount;
			DynamicDataProvider provider;
			SetupPropertyBlock( inStream, PvdClassKeys::RigidDynamic + 1, visitRigidDynamicPerFrameProperties<PropertyBlockSetup> );
			for( ; theActors < lastActor; ++theActors )
			{
				provider.mActor = static_cast<const PxRigidDynamic*>( *theActors );
				UpdateActor( inStream, mBindingData->mSleepingActors, theBlock, provider ); 
			}
			inStream->endPropertyBlock();
		}
	}
	{
		PxU32 articulationCount = inScene->getNbArticulations();
		if ( articulationCount )
		{
			mBindingData->mArticulations.resize( articulationCount );
			PxArticulation** firstArticulation = mBindingData->mArticulations.begin();
			PxArticulation** lastArticulation = firstArticulation + articulationCount;
			inScene->getArticulations( firstArticulation, articulationCount );
			SetupPropertyBlock( inStream, PvdClassKeys::ArticulationLink + 1, visitArticulationLinkPerFrameProperties<PropertyBlockSetup> );
			LinkDataProvider provider;
			for ( ; firstArticulation < lastArticulation; ++firstArticulation )
			{
				PxU32 linkCount = (*firstArticulation)->getNbLinks();
				bool sleeping = (*firstArticulation)->isSleeping();
				provider.mSleeping = sleeping;
				if ( linkCount )
				{
					mBindingData->mArticulationLinks.resize( linkCount );
					PxArticulationLink** theLink = mBindingData->mArticulationLinks.begin();
					PxArticulationLink** lastLink = theLink + linkCount;
					(*firstArticulation)->getLinks( theLink, linkCount );
					for ( ; theLink < lastLink; ++theLink )
					{
						PxArticulationLink* link( *theLink );
						provider.mActor = link;
						UpdateActor( inStream, mBindingData->mSleepingActors, theBlock, provider );
					}
				}
			}
			inStream->endPropertyBlock();
			firstArticulation = mBindingData->mArticulations.begin();
			for ( ; firstArticulation < lastArticulation; ++firstArticulation )
				inStream->setPropertyValue( PX_PROFILE_POINTER_TO_U64( (*firstArticulation) ), PxPropertyInfoName::PxArticulation_IsSleeping, (*firstArticulation)->isSleeping() );
		}
	}
}

void PvdMetaDataBinding::updateCloths( PVD::PvdDataStream* inStream, const PxScene* inScene )
{
	PxU32 actorCount = inScene->getNbActors( PxActorTypeSelectionFlag::eCLOTH );
	if ( actorCount  == 0 ) return;
	mBindingData->mActors.resize( actorCount );
	PxActor** theActors = mBindingData->mActors.begin();
	inScene->getActors( PxActorTypeSelectionFlag::eCLOTH, theActors, actorCount );
	PxU32 properties[2] = { PxPropertyInfoName::PxClothParticle_Pos, PxPropertyInfoName::PxClothParticle_InvWeight };
	PvdCommLayerDatatype datatypes[2] = { getDatatypeForType<PxVec3>(), getDatatypeForType<PxF32>() };
	PX_COMPILE_TIME_ASSERT( sizeof( PxClothParticle ) == sizeof( PxVec3 ) + sizeof( PxF32 ) );
	for ( PxU32 idx =0; idx < actorCount; ++idx )
	{
		PxCloth* theCloth = static_cast<PxCloth*>( theActors[idx] );
		PxU64 theInstance = PX_PROFILE_POINTER_TO_U64( theActors[idx] );
		bool isSleeping = theCloth->isSleeping();
		bool wasSleeping = mBindingData->mSleepingActors.contains( theCloth );
		if ( isSleeping == false || isSleeping != wasSleeping )
		{
			PxClothReadData* theData = theCloth->lockClothReadData();
			inStream->beginArrayPropertyBlock( theInstance, PxPvdOnlyProperties::PxCloth_ParticleBuffer, properties, datatypes, 2 ); 
			inStream->sendArrayObjects( reinterpret_cast<const PxU8*>( theData->particles ), sizeof( PxClothParticle ), theCloth->getNbParticles() );
			inStream->endArrayPropertyBlock();
			theData->unlock();
		}
		if ( isSleeping != wasSleeping )
		{
			inStream->setPropertyValue( theInstance, PxPropertyInfoName::PxCloth_IsSleeping, isSleeping );
			if ( isSleeping )
				mBindingData->mSleepingActors.insert( theActors[idx] );
			else
				mBindingData->mSleepingActors.erase( theActors[idx] );
		}
	}
}

template<typename TReadDataType>
struct ParticleFluidUpdater
{
	TReadDataType& mData;
	Array<PxU8>& mTempU8Array;
	PvdDataStream* mStream;
	PxU64 mInstanceId;
	PxU32 mRdFlags;
	ParticleFluidUpdater( TReadDataType& d, PvdDataStream* s, PxU64 id, PxU32 flags, Array<PxU8>& tempArray )
		: mData( d )
		, mStream( s )
		, mInstanceId( id )
		, mRdFlags( flags )
		, mTempU8Array( tempArray )
	{
	}
	ParticleFluidUpdater( const ParticleFluidUpdater<TReadDataType>& inOther )
		: mData( inOther.mData )
		, mStream( inOther.mStream )
		, mInstanceId( inOther.mInstanceId )
		, mRdFlags( inOther.mRdFlags )
		, mTempU8Array( inOther.mTempU8Array )
	{
	}

	template<PxU32 TKey, typename TObjectType, typename TPropertyType, PxU32 TEnableFlag>
	void handleBuffer( const PxBufferPropertyInfo< TKey, TObjectType, PxStrideIterator<const TPropertyType>, TEnableFlag >& inProp, PvdCommLayerDatatype inDatatype )
	{
		PxU32 numValidParticles = mData.numValidParticles;
		PvdDataStream* inConnection = mStream;
		PxU64 instanceId = mInstanceId;
		PxU32 validParticleRange = mData.validParticleRange;
		PxStrideIterator<const TPropertyType> iterator( inProp.get( &mData ) );
		const PxU32* validParticleBitmap = mData.validParticleBitmap;
		
		if( numValidParticles == 0 || iterator.ptr() == NULL || inProp.isEnabled(mRdFlags) == false )
			return;

		// setup the pvd array
		PxU32 arrayProperty = 1;
		inConnection->beginArrayPropertyBlock(instanceId, TKey, &arrayProperty, &inDatatype, 1);
		if(numValidParticles == validParticleRange)
		{
			inConnection->sendArrayObjects( reinterpret_cast<const PxU8*>(iterator.ptr()), iterator.stride(), numValidParticles);
		}
		else
		{
			mTempU8Array.clear();
			mTempU8Array.resize(numValidParticles * sizeof(TPropertyType));
			TPropertyType* tmpArray  = reinterpret_cast<TPropertyType*>(mTempU8Array.begin());
			PxU32 tIdx = 0;

			// iterate over bitmap and send all valid particles
			for (PxU32 w = 0; w <= (validParticleRange-1) >> 5; w++)
			{
				for (PxU32 b = validParticleBitmap[w]; b; b &= b-1)
				{
					tmpArray[tIdx++] = iterator[w<<5|Ps::lowestSetBit(b)];
				}
			}
			PX_ASSERT(tIdx == numValidParticles);
			inConnection->sendArrayObjects( mTempU8Array.begin(), 0, numValidParticles);
		}
		inConnection->endArrayPropertyBlock();
		mTempU8Array.clear();
	}
	template<PxU32 TKey, typename TObjectType, typename TPropertyType, PxU32 TEnableFlag>
	void handleBuffer( const PxBufferPropertyInfo< TKey, TObjectType, PxStrideIterator<const TPropertyType>, TEnableFlag >& inProp )
	{
		handleBuffer( inProp, getDatatypeForType<TPropertyType>() );
	}

	template<PxU32 TKey, typename TObjectType, typename TEnumType, typename TStorageType, PxU32 TEnableFlag>
	void handleFlagsBuffer( const PxBufferPropertyInfo< TKey, TObjectType, PxStrideIterator<const PxFlags<TEnumType, TStorageType> >, TEnableFlag >& inProp, const PxU32ToName* )
	{
		handleBuffer( inProp, getDatatypeForType<TStorageType>() );
	}
};

void PvdMetaDataBinding::sendArrays( PVD::PvdDataStream* inStream, const PxParticleSystem* inObj, PxParticleReadData& inData, PxU32 inFlags )
{
	ParticleFluidUpdater<PxParticleReadData> theUpdater( inData, inStream, PX_PROFILE_POINTER_TO_U64( (const PxActor*)inObj ), inFlags, mBindingData->mTempU8Array );
	visitParticleSystemBufferProperties( makePvdPropertyFilter( theUpdater ) );
}
void PvdMetaDataBinding::sendArrays( PVD::PvdDataStream* inStream, const PxParticleFluid* inObj, PxParticleFluidReadData& inData, PxU32 inFlags )
{
	ParticleFluidUpdater<PxParticleFluidReadData> theUpdater( inData, inStream, PX_PROFILE_POINTER_TO_U64( (const PxActor*)inObj ), inFlags, mBindingData->mTempU8Array );
	visitParticleSystemBufferProperties( makePvdPropertyFilter( theUpdater ) );
	visitParticleFluidBufferProperties( makePvdPropertyFilter( theUpdater ) );
}
void PvdMetaDataBinding::sendArrays( PVD::PvdDataStream* inStream, const PxDeformable* inObj, const Sc::DeformableBulkData& inData )
{
}

}

}

#endif
