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

#include "PxVisualDebuggerExt.h"
#include "ExtVisualDebugger.h"
#include "PxExtensionMetaDataObjects.h"

#include "ExtD6Joint.h"
#include "ExtFixedJoint.h"
#include "ExtSphericalJoint.h"
#include "ExtDistanceJoint.h"
#include "ExtSphericalJoint.h"
#include "ExtRevoluteJoint.h"
#include "ExtPrismaticJoint.h"
#include "ExtJointMetaDataExtensions.h"
#include "PvdMetaDataPropertyVisitor.h"
#include "PvdMetaDataDefineProperties.h"
#include "PvdMetaDataSendProperties.h"

namespace physx
{
namespace Pvd
{

struct PvdClassKeys
{
	enum Enum
	{
		Joint = 1,
		D6Joint,
		DistanceJoint,
		FixedJoint,
		PrismaticJoint,
		RevoluteJoint,
		SphericalJoint,
		LAST_ELEMENT,
	};
};

const char* jointClassNames[PvdClassKeys::LAST_ELEMENT] = 
{
	"",
	"PxJoint",
	"PxD6Joint",
	"PxDistanceJoint",
	"PxFixedJoint",
	"PxPrismaticJoint",
	"PxRevoluteJoint",
	"PxSphericalJoint",
};

template<typename TObjType>
struct DatatypeTypeToClassKeyMap { bool unknown; };

#define MAP_DATATYPE_TO_CLASS_KEY(enumName)  \
	template<> struct DatatypeTypeToClassKeyMap<Px##enumName> { PxU32 mPvdClass; DatatypeTypeToClassKeyMap<Px##enumName>() : mPvdClass( PvdClassKeys::enumName ) {} }

MAP_DATATYPE_TO_CLASS_KEY( Joint );
MAP_DATATYPE_TO_CLASS_KEY( D6Joint );
MAP_DATATYPE_TO_CLASS_KEY( RevoluteJoint );
MAP_DATATYPE_TO_CLASS_KEY( PrismaticJoint );
MAP_DATATYPE_TO_CLASS_KEY( FixedJoint );
MAP_DATATYPE_TO_CLASS_KEY( DistanceJoint );
MAP_DATATYPE_TO_CLASS_KEY( SphericalJoint );

struct U32ToNameHolder
{
	const PxU32ToName* NameConversion;
	U32ToNameHolder( const PxU32ToName* inNames ) : NameConversion( inNames ) {}
};


}

#include "PvdMetaDataPropertyVisitor.h"
#include "PvdMetaDataDefineProperties.h"
#include "PvdMetaDataSendProperties.h"

namespace Ext
{
	using namespace Pvd;

	template<typename TObjType, typename TOperator>
	inline void visitPvdInstanceProperties( TOperator inOperator )
	{
		PxClassInfoTraits<TObjType>().Info.visitInstanceProperties( makePvdPropertyFilter( inOperator ), 0 );	
	}

	template<typename TObjType, typename TOperator>
	inline void visitPvdProperties( TOperator inOperator )
	{
		PvdPropertyFilter<TOperator> theFilter( makePvdPropertyFilter( inOperator ) );
		PxU32 thePropCount = PxClassInfoTraits<TObjType>().Info.visitBaseProperties( theFilter );
		PxClassInfoTraits<TObjType>().Info.visitInstanceProperties( theFilter, thePropCount );
	}

	using namespace Pvd;

	VisualDebugger::PvdNameSpace::PvdNameSpace(PVD::PvdDataStream& conn, const char* name)
		: mConnection(conn)
	{
		conn.pushNamespace();
		conn.setNamespace(name);
	}

	VisualDebugger::PvdNameSpace::~PvdNameSpace()
	{
		mConnection.popNamespace();
	}

	void VisualDebugger::releasePvdInstance(PVD::PvdDataStream& pvdConnection, const PxJoint* joint)
	{
		if(!pvdConnection.isConnected())
			return;
		pvdConnection.destroyInstance(PX_PROFILE_POINTER_TO_U64(joint));
	}

	template<typename TObjType>
	void registerProperties( PvdDataStream& inStream )
	{
		PvdPropertyDefinitionHelper& theHelper( inStream.getPropertyDefinitionHelper() );
		PvdClassInfoDefine theDefinitionObj( theHelper, DatatypeTypeToClassKeyMap<TObjType>().mPvdClass );
		visitPvdInstanceProperties<TObjType>( theDefinitionObj );
	}

	template<typename TObjType, typename TValueStructType>
	void registerPropertiesAndValueStruct( PvdDataStream& inStream )
	{
		PvdPropertyDefinitionHelper& theHelper( inStream.getPropertyDefinitionHelper() );
		PxU32 theClassKey = DatatypeTypeToClassKeyMap<TObjType>().mPvdClass;
		{
			PvdClassInfoDefine theDefinitionObj( theHelper, theClassKey );
			visitPvdInstanceProperties<TObjType>( theDefinitionObj );
		}
		{
			PvdClassInfoValueStructDefine theDefinitionObj( theHelper );
			visitPvdProperties<TObjType>( theDefinitionObj );
			theHelper.definePropertyStruct( theClassKey, theClassKey, sizeof( TValueStructType ) );
		}
	}

	void VisualDebugger::sendClassDescriptions(PVD::PvdDataStream& pvdConnection)
	{
		PvdNameSpace ns(pvdConnection, "physx3Ext");

		// register all classes, see ExtPvdClassDefinitions.h
		for(PxU32 classKey = PvdClassKeys::Joint; classKey < PvdClassKeys::LAST_ELEMENT; ++classKey)
		{
			pvdConnection.createClass(jointClassNames[classKey], classKey);
			if(classKey > PvdClassKeys::Joint )
				pvdConnection.deriveClass(PvdClassKeys::Joint, classKey);

			switch( classKey )
			{
			case PvdClassKeys::Joint: registerProperties<PxJoint>(pvdConnection); break;
#define REGISTER_JOINT_PROPS_AND_VALUE_STRUCT( jointtype ) \
			case PvdClassKeys::jointtype: registerPropertiesAndValueStruct<Px##jointtype, Px##jointtype##GeneratedValues>(pvdConnection);  break
				REGISTER_JOINT_PROPS_AND_VALUE_STRUCT( D6Joint );
				REGISTER_JOINT_PROPS_AND_VALUE_STRUCT( DistanceJoint );
				REGISTER_JOINT_PROPS_AND_VALUE_STRUCT( FixedJoint );
				REGISTER_JOINT_PROPS_AND_VALUE_STRUCT( PrismaticJoint );
				REGISTER_JOINT_PROPS_AND_VALUE_STRUCT( RevoluteJoint );
				REGISTER_JOINT_PROPS_AND_VALUE_STRUCT( SphericalJoint );
#undef REGISTER_JOINT_PROPS_AND_VALUE_STRUCT
			default:
				PX_ASSERT( false );
				break;
			}
		}
	}
	
	void VisualDebugger::setActors( PVD::PvdDataStream& inStream, const PxJoint* inJoint, const PxConstraint* c, const PxActor* newActor0, const PxActor* newActor1 )
	{
		PxRigidActor* actor0, *actor1;
		PxU64 theInstance = PX_PROFILE_POINTER_TO_U64( inJoint );
		c->getActors( actor0, actor1 );
		if ( actor0 )
			inStream.removeChild( PX_PROFILE_POINTER_TO_U64( (PxActor*)actor0 ), theInstance );
		if ( actor1 )
			inStream.removeChild( PX_PROFILE_POINTER_TO_U64( (PxActor*)actor1 ), theInstance );

		if ( newActor0 )
			inStream.addChild( PX_PROFILE_POINTER_TO_U64( (PxActor*)newActor0 ), theInstance );
		if ( newActor1 )
			inStream.addChild( PX_PROFILE_POINTER_TO_U64( (PxActor*)newActor1 ), theInstance );

		inStream.setPropertyValue( theInstance, PxEnumRangeMap<PxExtensionsPropertyInfoName::PxJoint_Actors>().Start, createInstanceId( PX_PROFILE_POINTER_TO_U64( newActor0 ) ) ); 
		inStream.setPropertyValue( theInstance, PxEnumRangeMap<PxExtensionsPropertyInfoName::PxJoint_Actors>().Start+1, createInstanceId( PX_PROFILE_POINTER_TO_U64( newActor1 ) ) );
	}

	template<typename TValueStructType, typename TObjType>
	void sendAllProperties( PvdDataStream& inStream, const TObjType* inSource )
	{
		VisualDebugger::PvdNameSpace ns(inStream, "physx3Ext");

		TValueStructType theValueStruct( inSource );
		const PxJoint* theJoint = static_cast<const PxJoint*>( inSource );
		inStream.sendPropertyStruct( PX_PROFILE_POINTER_TO_U64( theJoint ), DatatypeTypeToClassKeyMap<TObjType>().mPvdClass, (PxU8*)&theValueStruct, sizeof( TValueStructType ) );
	}

	template<typename TObjType>
	void createInstance( PvdDataStream& inStream, const PxConstraint* c, const TObjType* inSource )
	{
		VisualDebugger::PvdNameSpace ns(inStream, "physx3Ext");
		const PxJoint* theJoint = inSource;
		PxRigidActor* actor0, *actor1;
		PxU64 theInstance = PX_PROFILE_POINTER_TO_U64( theJoint );
		c->getActors( actor0, actor1 );
		inStream.createInstance( DatatypeTypeToClassKeyMap<TObjType>().mPvdClass, theInstance );
		inStream.addChild(PX_PROFILE_POINTER_TO_U64(c->getScene())+JOINT_GROUP+1, theInstance);
		if ( actor0 )
			inStream.addChild( PX_PROFILE_POINTER_TO_U64( (PxActor*)actor0 ), theInstance );
		if ( actor1 )
			inStream.addChild( PX_PROFILE_POINTER_TO_U64( (PxActor*)actor1 ), theInstance );
	}
	
#define IMPLEMENT_JOINT_PVD_OPERATIONS( jointtype ) \
	void VisualDebugger::updatePvdProperties(PVD::PvdDataStream& pvdConnection, const jointtype* joint) { sendAllProperties<jointtype##GeneratedValues>( pvdConnection, joint ); }	\
	void VisualDebugger::simUpdate(PVD::PvdDataStream&, const jointtype*) {}																										\
	void VisualDebugger::createPvdInstance(PVD::PvdDataStream& pvdConnection, const PxConstraint* c, const jointtype* joint)														\
	{																																												\
		createInstance( pvdConnection, c, joint );																																		\
	}	

	IMPLEMENT_JOINT_PVD_OPERATIONS( PxD6Joint );
	IMPLEMENT_JOINT_PVD_OPERATIONS( PxDistanceJoint );
	IMPLEMENT_JOINT_PVD_OPERATIONS( PxFixedJoint );
	IMPLEMENT_JOINT_PVD_OPERATIONS( PxPrismaticJoint );
	IMPLEMENT_JOINT_PVD_OPERATIONS( PxRevoluteJoint );
	IMPLEMENT_JOINT_PVD_OPERATIONS( PxSphericalJoint );

}

}

#else

#include "CmPhysXCommon.h"
#include "PxVisualDebuggerExt.h"

namespace PVD
{
	class PvdDataStream;
}

#endif // PX_SUPPORT_VISUAL_DEBUGGER
