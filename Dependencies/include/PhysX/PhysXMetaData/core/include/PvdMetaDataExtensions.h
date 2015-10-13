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

#ifndef PX_META_DATA_EXTENSIONS_H
#define PX_META_DATA_EXTENSIONS_H
#include "PxMetaDataObjects.h"

#if PX_SUPPORT_VISUAL_DEBUGGER
#include "PVDCommLayerDatatypes.h"
#include "PvdConnection.h"
#include "PvdDataStream.h"

namespace PVD
{
	template<> PX_INLINE PvdCommLayerDatatype getDatatypeForType<const PxBounds3&>() { return getDatatypeForType<Bounds3>(); }
	template<> PX_INLINE PvdCommLayerDatatype getDatatypeForType<physx::PxMetaDataPlane>() { return getDatatypeForType<Plane>(); }
	template<> PX_INLINE PvdCommLayerDatatype getDatatypeForType<physx::PxRigidActor*>() { return getDatatypeForType<InstanceId>(); }
	template<> PX_INLINE PvdCommLayerDatatype getDatatypeForType<physx::PxClothFabric*>() { return getDatatypeForType<InstanceId>(); }
}
#else
namespace PVD {}
#endif


namespace physx
{
namespace Pvd
{
using namespace PVD;


//Additional properties that exist only in pvd land.
struct PxPvdOnlyProperties
{
	enum Enum
	{
		FirstProp = PxPropertyInfoName::LastPxPropertyInfoName,
		PxScene_Frame,
		PxScene_Contacts,
		PxScene_SimulateElapsedTime,
#define DEFINE_ENUM_RANGE( stem, count ) \
	stem##Begin, \
	stem##End = stem##Begin + count

		//I can't easily add up the number of required property entries, but it is large due to the below
		//geometry count squared properties.  Thus I punt and allocate way more than I need right now.
		DEFINE_ENUM_RANGE( PxScene_SimulationStatistics, 1000 ),
		DEFINE_ENUM_RANGE( PxSceneDesc_Limits, PxPropertyInfoName::PxSceneLimits_PropertiesStop - PxPropertyInfoName::PxSceneLimits_PropertiesStart ),
		DEFINE_ENUM_RANGE( PxSimulationStatistics_NumBroadPhaseAdds, PxSimulationStatistics::eVOLUME_COUNT ),
		DEFINE_ENUM_RANGE( PxSimulationStatistics_NumBroadPhaseRemoves, PxSimulationStatistics::eVOLUME_COUNT ),
		DEFINE_ENUM_RANGE( PxSimulationStatistics_NumShapes, PxGeometryType::eGEOMETRY_COUNT ),
		DEFINE_ENUM_RANGE( PxSimulationStatistics_NumDiscreteContactPairs, PxGeometryType::eGEOMETRY_COUNT * PxGeometryType::eGEOMETRY_COUNT ),
		DEFINE_ENUM_RANGE( PxSimulationStatistics_NumModifiedContactPairs, PxGeometryType::eGEOMETRY_COUNT * PxGeometryType::eGEOMETRY_COUNT ),
		DEFINE_ENUM_RANGE( PxSimulationStatistics_NumSweptContactPairs, PxGeometryType::eGEOMETRY_COUNT * PxGeometryType::eGEOMETRY_COUNT ),
		DEFINE_ENUM_RANGE( PxSimulationStatistics_NumSweptIntegrationPairs, PxGeometryType::eGEOMETRY_COUNT * PxGeometryType::eGEOMETRY_COUNT ),
		DEFINE_ENUM_RANGE( PxSimulationStatistics_NumTriggerPairs, PxGeometryType::eGEOMETRY_COUNT * PxGeometryType::eGEOMETRY_COUNT ),
		DEFINE_ENUM_RANGE( PxRigidDynamic_SolverIterationCounts, 2 ),
		DEFINE_ENUM_RANGE( PxArticulationJoint_SwingLimit, 2 ),
		DEFINE_ENUM_RANGE( PxArticulationJoint_TwistLimit, 2 ),
		DEFINE_ENUM_RANGE( PxConvexMeshGeometry_Scale, PxPropertyInfoName::PxMeshScale_PropertiesStop - PxPropertyInfoName::PxMeshScale_PropertiesStart ),
		DEFINE_ENUM_RANGE( PxTriangleMeshGeometry_Scale, PxPropertyInfoName::PxMeshScale_PropertiesStop - PxPropertyInfoName::PxMeshScale_PropertiesStart ),
		DEFINE_ENUM_RANGE( PxCloth_MotionConstraintScaleBias, 2 ),
		DEFINE_ENUM_RANGE( PxArticulation_SolverIterationCounts, 2 ),
		PxDeformable_Positions,
  		PxDeformable_Velocities,
  		PxDeformable_Normals,
  		PxDeformable_InverseMasses,
  		PxDeformable_ParentIndexes,
  		PxDeformable_Indexes,
  		PxDeformable_PrimitiveSplitPairs,
		PxParticleSystem_Positions,
		PxParticleSystem_Velocities,
		PxParticleSystem_RestOffsets,
		PxParticleSystem_CollisionNormals,
		PxParticleSystem_Flags,
		PxParticleFluid_Densities,
		PxCloth_ParticleBuffer,
		PxCloth_MotionConstraints,
		PxCloth_CollisionSpheres,
		PxCloth_CollisionSpherePairs,
		PxCloth_VirtualParticleTriangleAndWeightIndexes,
		PxCloth_VirtualParticleWeights,
		LastPxPvdOnlyProperty,
	};
};

template<PxU32 TEnumValue>
struct PxEnumRangeMap
{
	bool crapValue;
};

#define DEFINE_ENUM_RANGE_MAP( enumName ) \
template<> \
struct PxEnumRangeMap<PxPropertyInfoName::enumName> \
{ \
	PxU32 Start;  \
	PxU32 Stop; \
	PxEnumRangeMap() : Start( PxPvdOnlyProperties::enumName##Begin ), Stop( PxPvdOnlyProperties::enumName##End ) {} \
}; 

DEFINE_ENUM_RANGE_MAP( PxScene_SimulationStatistics );
DEFINE_ENUM_RANGE_MAP( PxSimulationStatistics_NumBroadPhaseAdds );
DEFINE_ENUM_RANGE_MAP( PxSimulationStatistics_NumBroadPhaseRemoves );
DEFINE_ENUM_RANGE_MAP( PxSimulationStatistics_NumShapes );
DEFINE_ENUM_RANGE_MAP( PxSimulationStatistics_NumDiscreteContactPairs );
DEFINE_ENUM_RANGE_MAP( PxSimulationStatistics_NumModifiedContactPairs );
DEFINE_ENUM_RANGE_MAP( PxSimulationStatistics_NumSweptContactPairs );
DEFINE_ENUM_RANGE_MAP( PxSimulationStatistics_NumSweptIntegrationPairs );
DEFINE_ENUM_RANGE_MAP( PxSimulationStatistics_NumTriggerPairs );
DEFINE_ENUM_RANGE_MAP( PxRigidDynamic_SolverIterationCounts );
DEFINE_ENUM_RANGE_MAP( PxArticulationJoint_SwingLimit );
DEFINE_ENUM_RANGE_MAP( PxArticulationJoint_TwistLimit );
DEFINE_ENUM_RANGE_MAP( PxSceneDesc_Limits );
DEFINE_ENUM_RANGE_MAP( PxConvexMeshGeometry_Scale );
DEFINE_ENUM_RANGE_MAP( PxTriangleMeshGeometry_Scale );
DEFINE_ENUM_RANGE_MAP( PxCloth_MotionConstraintScaleBias );
DEFINE_ENUM_RANGE_MAP( PxArticulation_SolverIterationCounts );

struct PvdContact
{
	PxVec3 point;
	PxVec3 axis;
	PxU64 shape0;
	PxU64 shape1;
	PxReal separation;
	PxReal normalForce;
	PxU32 feature0;
	PxU32 feature1;
	bool normalForceAvailable;
};

template<PxU32 TKey, typename TObjectType, typename TPropertyType, PxU32 TEnableFlag>
struct PxBufferPropertyInfo : PxReadOnlyPropertyInfo< TKey, TObjectType, TPropertyType >
{
	typedef PxReadOnlyPropertyInfo< TKey, TObjectType, TPropertyType > TBaseType;
	typedef typename TBaseType::TGetterType TGetterType;
	PxBufferPropertyInfo( const char* inName, TGetterType inGetter )
		: TBaseType( inName, inGetter )
	{
	}
	bool isEnabled( PxU32 inFlags ) const { return (inFlags & TEnableFlag) > 0; }
};


#define DECLARE_BUFFER_PROPERTY( objectType, baseType, propType, propName, fieldName, flagName )												\
typedef PxBufferPropertyInfo< PxPvdOnlyProperties::baseType##_##propName, objectType, propType, flagName > T##objectType##propName##Base;		\
inline propType get##propName( const objectType* inData ) { return inData->fieldName; }															\
struct baseType##propName##Property : T##objectType##propName##Base																				\
{																																				\
	baseType##propName##Property()  : T##objectType##propName##Base( #propName, get##propName ){}												\
};

DECLARE_BUFFER_PROPERTY( PxParticleReadData,		PxParticleSystem, PxStrideIterator<const PxVec3>,			Positions, positionBuffer, PxParticleReadDataFlag::ePOSITION_BUFFER );
DECLARE_BUFFER_PROPERTY( PxParticleReadData,		PxParticleSystem, PxStrideIterator<const PxVec3>,			Velocities, velocityBuffer, PxParticleReadDataFlag::eVELOCITY_BUFFER );
DECLARE_BUFFER_PROPERTY( PxParticleReadData,		PxParticleSystem, PxStrideIterator<const PxF32>,			RestOffsets, restOffsetBuffer, PxParticleReadDataFlag::eREST_OFFSET_BUFFER );
DECLARE_BUFFER_PROPERTY( PxParticleReadData,		PxParticleSystem, PxStrideIterator<const PxVec3>,			CollisionNormals, collisionNormalBuffer, PxParticleReadDataFlag::eCOLLISION_NORMAL_BUFFER );
DECLARE_BUFFER_PROPERTY( PxParticleReadData,		PxParticleSystem, PxStrideIterator<const PxParticleFlags>,	Flags, flagsBuffer, PxParticleReadDataFlag::eFLAGS_BUFFER );
DECLARE_BUFFER_PROPERTY( PxParticleFluidReadData,	PxParticleFluid,  PxStrideIterator<const PxF32>,			Densities, densityBuffer, PxParticleReadDataFlag::eDENSITY_BUFFER );



template<typename TOperator>
inline void visitParticleSystemBufferProperties( TOperator inOperator )
{
	inOperator( PxParticleSystemPositionsProperty(), 0 );
	inOperator( PxParticleSystemVelocitiesProperty(), 1 );
	inOperator( PxParticleSystemRestOffsetsProperty(), 2 );
	inOperator( PxParticleSystemCollisionNormalsProperty(), 3 );
	inOperator( PxParticleSystemFlagsProperty(), 4 );
}

template<typename TOperator>
inline void visitParticleFluidBufferProperties( TOperator inOperator )
{
	inOperator( PxParticleFluidDensitiesProperty(), 0 );
}



template<PxU32 PropertyKey, typename TEnumType >
struct IndexerToNameMap
{
	PxEnumTraits<TEnumType> Converter;
};

struct ValueStructOffsetRecord
{
	mutable bool	mHasValidOffset;
	mutable PxU32	mOffset;
	ValueStructOffsetRecord() : mHasValidOffset( false ), mOffset( 0 ) {}
	void setupValueStructOffset( PxU32 inValue ) const
	{
		mHasValidOffset = true;
		mOffset = inValue;
	}
};

template<PxU32 TKey, typename TObjectType, typename TPropertyType>
struct PxPvdReadOnlyPropertyAccessor : public ValueStructOffsetRecord
{
	typedef PxReadOnlyPropertyInfo<TKey,TObjectType,TPropertyType> TPropertyInfoType;
	typedef TPropertyType prop_type;

	const TPropertyInfoType	mProperty;
	PxPvdReadOnlyPropertyAccessor( const TPropertyInfoType& inProp )
		: mProperty( inProp )
	{
	}
	prop_type get( const TObjectType* inObj ) const { return mProperty.get( inObj ); }
};

template<PxU32 TKey, typename TObjectType, typename TIndexType, typename TPropertyType>
struct PxPvdIndexedPropertyAccessor : public ValueStructOffsetRecord
{
	typedef PxIndexedPropertyInfo< TKey, TObjectType, TIndexType, TPropertyType > TPropertyInfoType;
	typedef TPropertyType prop_type;
	TIndexType mIndex;
	const TPropertyInfoType& mProperty;
	PxPvdIndexedPropertyAccessor( const TPropertyInfoType& inProp, PxU32 inIndex )
		: mIndex( static_cast<TIndexType>( inIndex ) )
		, mProperty( inProp )
	{
	}
	prop_type get( const TObjectType* inObj ) const { return mProperty.get( inObj, mIndex ); }
	void set( TObjectType* inObj, prop_type val ) const { mProperty.set( inObj, mIndex, val ); }
};

template<PxU32 TKey, typename TObjectType, typename TIdx0Type, typename TIdx1Type, typename TPropertyType>
struct PxPvdDualIndexedPropertyAccessor : public ValueStructOffsetRecord
{
	typedef PxDualIndexedPropertyInfo< TKey, TObjectType, TIdx0Type, TIdx1Type, TPropertyType > TPropertyInfoType;
	typedef TPropertyType prop_type;
	TIdx0Type mIdx0;
	TIdx1Type mIdx1;
	const TPropertyInfoType& mProperty;

	PxPvdDualIndexedPropertyAccessor( const TPropertyInfoType& inProp, PxU32 idx0, PxU32 idx1 )
		: mIdx0( static_cast<TIdx0Type>( idx0 ) )
		, mIdx1( static_cast<TIdx1Type>( idx1 ) )
		, mProperty( inProp )
	{
	}
	prop_type get( const TObjectType* inObj ) const { return mProperty.get( inObj, mIdx0, mIdx1 ); }
	void set( TObjectType* inObj, prop_type val ) const { mProperty.set( inObj, mIdx0, mIdx1, val ); }
};


template<PxU32 TKey, typename TObjType, typename TPropertyType>
struct PxPvdRangePropertyAccessor : public ValueStructOffsetRecord
{
	typedef PxRangePropertyInfo<TKey, TObjType, TPropertyType> TPropertyInfoType;
	typedef TPropertyType prop_type;
	bool mFirstValue;
	const TPropertyInfoType& mProperty;

	PxPvdRangePropertyAccessor( const TPropertyInfoType& inProp, bool inFirstValue )
		: mFirstValue( inFirstValue )
		, mProperty( inProp )
	{
	}

	prop_type get( const TObjType* inObj ) const {
		prop_type first,second;
		mProperty.get( inObj, first, second );
		return mFirstValue ? first : second;
	}
	void set( TObjType* inObj, prop_type val ) const 
	{ 
		prop_type first,second;
		mProperty.get( inObj, first, second );
		if ( mFirstValue ) mProperty.set( inObj, val, second ); 
		else mProperty.set( inObj, first, val );
	}
};


template<typename TDataType>
struct IsFlagsType
{
	bool FlagData;
};

template<typename TEnumType, typename TStorageType>
struct IsFlagsType<PxFlags<TEnumType, TStorageType> > 
{
	const PxU32ToName* FlagData;
	IsFlagsType<PxFlags<TEnumType, TStorageType> > () : FlagData( PxEnumTraits<TEnumType>().NameConversion ) {}
};



template<typename TDataType>
struct PvdClassForType
{
	bool Unknown;
};

}

}

#endif
