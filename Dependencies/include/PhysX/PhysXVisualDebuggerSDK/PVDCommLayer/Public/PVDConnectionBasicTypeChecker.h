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

#ifndef PVD_PVDCONNECTIONBASICTYPECHECKER_H
#define PVD_PVDCONNECTIONBASICTYPECHECKER_H

#include "PvdDataStreamImplTypes.h"
#include "PvdRenderCommands.h"
#include <algorithm>

namespace PVD
{
#ifndef PVD_COMM_LAYER_TESTS
#define PVD_RETURN_ERROR_CODE( code ) { PX_ASSERT( code == PvdCommLayerError::None ); return code; }
#else
	#define PVD_RETURN_ERROR_CODE( code ) { return code; }
#endif
	/**
	 *	Typechecks the connection's API calls on the runtime side
	 *	and to store the state that the runtime side needs
	 *	for a few of the API calls.  The connection implementation will
	 *	forward calls to this class before forwarding to the stream
	 *	layer.  This class does not provide the implementation
	 *	the of add/remove child checks as this would require maintaining
	 *	hierarchy on the runtime side and should not be on
	 *	all the time; only in special cases.
	 *
	 *	This also implements a small state machine to ensure that the block calls
	 *	match up and that calls aren't made in a block context that shouldn't be
	 *	and vice versa.
	 *
	 *	Note the use of inline instead of PX_INLINE.  These function calls
	 *	can be somewhat expensive so it isn't clear the benefit for inlining
	 *	them.
	 */
	template<typename TDataSystem
			, typename TMutexType
			, typename TScopedLockType
			, typename TDeleteOperatorType >
	class PvdConnectionBasicTypeChecker
	{
		TDataSystem						mDataSystem;
		PxU32							mBlockClass;
		PxU32							mRenderStackDepth;
		PxU32							mTransformStackDepth;
		TMutexType						mMutex;
		PxU32							mRefCount;

	public:
		PvdConnectionBasicTypeChecker()  
			: mBlockClass( 0 ) 
			, mRenderStackDepth( 0 )
			, mTransformStackDepth( 0 )
			, mRefCount( 0 )
		{}

		inline PvdCommLayerError setNamespace( const char* ) { return PvdCommLayerError::None; }
		inline PvdCommLayerError createClass( PxU32 inNamespace, const char* inName, PxU32 inKey );
		inline PvdCommLayerError createClass( const char* inName, PxU32 inKey ) { return createClass( 0, inName, inKey ); }
		inline PvdCommLayerError deriveClass( PxU32 inNamespace, PxU32 inParentKey, PxU32 inChildKey );
		inline PvdCommLayerError deriveClass( PxU32 inParentKey, PxU32 inChildKey ) { return deriveClass( 0, inParentKey, inChildKey ); }
		inline PvdCommLayerError defineProperty( PxU32 inNamespace, PxU32 inClass
													, const char* inName
													, const char* inSemantic
													, PvdCommLayerDatatype inDatatype
													, PxU32 inKey );

		inline PvdCommLayerError defineProperty( PxU32 inClass
													, const char* inName
													, const char* inSemantic
													, PvdCommLayerDatatype inDatatype
													, PxU32 inKey )
		{ 
			return defineProperty( 0, inClass, inName, inSemantic, inDatatype, inKey ); 
		}

		inline PvdCommLayerError definePropertyOnInstance( PxU64 inInstanceId
															, const char* inName
															, const char* inSemantic
															, PvdCommLayerDatatype inDatatype
															, PxU32 inKey );
		inline PvdCommLayerError definePropertyStruct( PxU32 inStructKey
															, PxU32 inNamespace
															, PxU32 inClass
															, PxU32 inStructByteSize
															, const PropertyStructEntry* inEntries
															, PxU32 inEntryCount );
		inline PvdCommLayerError defineArrayProperty( PxU32 inNamespace, PxU32 inClass
														, const char* inName
														, PxU32 inArrayClass
														, PxU32 inKey );
		inline PvdCommLayerError defineArrayProperty( PxU32 inClass
														, const char* inName
														, PxU32 inArrayClass
														, PxU32 inKey )
		{
			return defineArrayProperty( 0, inClass, inName, inArrayClass, inKey );
		}
		inline PvdCommLayerError defineBitflagNames( PxU32 inNamespace, PxU32 inClass
														, PxU32 inPropertyKey
														, const NamedValueDefinition* inDefinitions
														, PxU32 inDefinitionLength );
		inline PvdCommLayerError defineBitflagNames( PxU32 inClass
														, PxU32 inPropertyKey
														, const NamedValueDefinition* inDefinitions
														, PxU32 inDefinitionLength )
		{
			return defineBitflagNames( 0, inClass, inPropertyKey, inDefinitions, inDefinitionLength );
		}
		inline PvdCommLayerError defineEnumerationNames( PxU32 inNamespace, PxU32 inClass
															, PxU32 inPropertyKey
															, const NamedValueDefinition* inDefinitions
															, PxU32 inDefinitionLength );
		inline PvdCommLayerError defineEnumerationNames( PxU32 inClass
															, PxU32 inPropertyKey
															, const NamedValueDefinition* inDefinitions
															, PxU32 inDefinitionLength )
		{
			return defineEnumerationNames( 0, inClass, inPropertyKey, inDefinitions, inDefinitionLength );
		}
		inline PvdCommLayerError createInstance( PxU32 inNamespace, PxU32 inClass, PxU64 inInstanceId );
		inline PvdCommLayerError createInstance( PxU32 inClass, PxU64 inInstanceId )
		{
			return createInstance( 0, inClass, inInstanceId );
		}
		inline PxU32			 getInstanceClass( PxU64 inInstanceId );
		inline PvdCommLayerError setPropertyValue( PxU64 inInstance, PxU32 inProperty, PvdCommLayerDatatype inDatatype );
		inline PvdCommLayerError beginPropertyBlock( PxU32 inNamespace, PxU32 inClass, const PxU32* inProperties, const PvdCommLayerDatatype* inDatatypes, PxU32 inPropertyCount );
		inline PvdCommLayerError beginPropertyBlock( PxU32 inClass, const PxU32* inProperties, const PvdCommLayerDatatype* inDatatypes, PxU32 inPropertyCount )
		{
			return beginPropertyBlock( 0, inClass, inProperties, inDatatypes, inPropertyCount );
		}
		inline PvdCommLayerError sendPropertyBlock( PxU64 inInstance, const PvdCommLayerValue* inValues );
		inline PvdCommLayerError endPropertyBlock();
		inline PvdCommLayerError sendPropertyStruct( PxU64 inInstance, PxU32 inStructKey, PxU32 inNamespace, const PxU8* inData, PxU32 inDataLen );
		inline PvdCommLayerError addChild( PxU64 inParent, PxU64 inChild );
		inline PvdCommLayerError removeChild( PxU64 inParent, PxU64 inChild );
		inline PvdCommLayerError removeAllChildren( PxU64 inInstanceId, PxU32 inChildClass );
		inline PvdCommLayerError destroyInstance( PxU64 inInstance );
		inline PvdCommLayerError beginArrayBlock( PxU32 inNamespace, PxU32 inClass, PxU64 inInstance, const PxU32* inProperties, const PvdCommLayerDatatype* inDatatypes, PxU32 inNumProps );
		inline PvdCommLayerError beginArrayBlock( PxU32 inClass, PxU64 inInstance, const PxU32* inProperties, const PvdCommLayerDatatype* inDatatypes, PxU32 inNumProps )
		{
			return beginArrayBlock( 0, inClass, inInstance, inProperties, inDatatypes, inNumProps );
		}
		inline PvdCommLayerError beginArrayPropertyBlock( PxU64 inInstance, PxU32 inProperty, const PxU32* inProperties, const PvdCommLayerDatatype* inDatatypes, PxU32 inPropertyCount );
		inline PvdCommLayerError sendArrayObject( const PvdCommLayerValue* inValues );
		inline PvdCommLayerError sendArrayObjects( const PxU8* inData, PxU32 inStride, PxU32 inCount );
		inline PvdCommLayerError endArrayBlock();
		inline PvdCommLayerError endArrayPropertyBlock();
		inline PvdCommLayerError beginSection( const char* inName );
		inline PvdCommLayerError endSection( const char* inName );
		inline PvdCommLayerError namedEventWithInstance(PxU64 inInstanceId, const char* inName);

		inline PvdCommLayerError checkRenderCommand()
		{
			TScopedLockType theLocker( mMutex );
			PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
		}

		template<typename TRenderCommandType>
		inline PvdCommLayerError handleRenderCommand( const TRenderCommandType& ) {	return checkRenderCommand(); }
		inline PvdCommLayerError handleRenderCommand( const PushRenderState& );
		inline PvdCommLayerError handleRenderCommand( const PopRenderState& );
		inline PvdCommLayerError handleRenderCommand( const PushTransform& );
		inline PvdCommLayerError handleRenderCommand( const PopTransform& );
		inline void addRef();
		inline void release();
	};
	

	//Implementation.
	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::createClass( PxU32 inNamespace, const char* inName, PxU32 inKey )
	{
		TScopedLockType theLocker( mMutex );
		if ( !inName ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidName );
		if ( !inKey ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );

		ClassDescription* existing = mDataSystem.findClassDescription( inNamespace, inKey );
		if ( existing != NULL )
		{
			if ( SafeStrEqual( existing->mName, inName ) )
			{
				PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
			}
			else
				PVD_RETURN_ERROR_CODE( PvdCommLayerError::NameBoundToDifferentKey );
		}
		mDataSystem.addClassDescription( inNamespace, inName, inKey );
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	
	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::deriveClass( PxU32 inNamespace, PxU32 inParentKey, PxU32 inChildKey )
	{
		TScopedLockType theLocker( mMutex );
		if ( !inParentKey ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
		if ( !inChildKey ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
		ClassDescription* parent = mDataSystem.findClassDescription( inNamespace, inParentKey );
		ClassDescription* child = mDataSystem.findClassDescription( inNamespace, inChildKey );
		if ( !parent || !child ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidClass );
		if ( !parent || !child ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidArguments );
		if ( child->mParentKey == parent->mKey )
		{
			PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
		}
		if ( parent->mParentKey == inChildKey ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::ParentDerivedFromChild );
		if ( child->mParentKey != InvalidKey() && child->mParentKey != inParentKey ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::ChildDerivedFromDifferentParent );
		if ( child->mClassLocked ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::ClassLocked );
		//Check that the child key doesn't exist on the parent.
		if ( mDataSystem.checkForPropertyCollision( child, parent ) )
			PVD_RETURN_ERROR_CODE( PvdCommLayerError::PropertyKeyCollision );


		//Run through and check the properties on the child.
		child->mParentKey = parent->mKey;
		parent->mClassLocked = true;
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::defineProperty( PxU32 inNamespace
																							, PxU32 inClass
																							, const char* inName
																							, const char* inSemantic
																							, PvdCommLayerDatatype inDatatype
																							, PxU32 inKey)
	{
		TScopedLockType theLocker( mMutex );
		if ( !inName ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidName );
		if ( !inKey ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
		if ( inDatatype == PvdCommLayerDatatype::Unknown
			|| inDatatype.mDatatype >= PvdCommLayerDatatype::Last ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidDatatype );
		ClassDescription* existing = mDataSystem.findClassDescription( inNamespace, inClass );
		if ( !existing ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidClass );
		if ( existing->mParentKey != InvalidKey() 
			&& mDataSystem.findParentPropertyDescription( existing, inKey ) ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::PropertyKeyCollision );
		PropertyDescription* theProperty = mDataSystem.findSpecificPropertyDescription( existing, inKey );
		if ( theProperty )
		{
			if ( !SafeStrEqual( inName, theProperty->mName )
				|| !SafeStrEqual( inSemantic, theProperty->mSemantic )
				|| inDatatype != theProperty->mDatatype )
				PVD_RETURN_ERROR_CODE( PvdCommLayerError::PropertyDefinitionError );
			PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
		}
		if ( existing->mClassLocked ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::ClassLocked );
		mDataSystem.addProperty( existing, inName, inSemantic, inKey, inDatatype );
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}
	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::definePropertyOnInstance( PxU64 inInstanceId
																									, const char* inName
																									, const char* inSemantic
																									, PvdCommLayerDatatype inDatatype
																									, PxU32 inKey)
	{
		TScopedLockType theLocker( mMutex );
		InstanceDescription* theDescription = mDataSystem.findInstanceDescription( inInstanceId );
		if ( !inInstanceId || theDescription == NULL ) { PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidInstance ); }
		return defineProperty( theDescription->mClassKey, inName, inSemantic, inDatatype, inKey );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::definePropertyStruct( PxU32 
																									, PxU32 inNamespace
																									, PxU32 inClass
																									, PxU32 inStructByteSize
																									, const PropertyStructEntry* inEntries
																									, PxU32 inEntryCount )
	{
		TScopedLockType theLocker( mMutex );
		ClassDescription* existing = mDataSystem.findClassDescription( inNamespace, inClass );
		if ( !existing ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidClass );
		//Ensure all of these properties actually do exist
		for ( PxU32 idx = 0; idx < inEntryCount; ++idx ) 
		{
			PropertyDescription* theProperty = mDataSystem.findPropertyDescription( existing, inEntries[idx].mProperty );
			if ( !theProperty ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidProperty );
			if ( inStructByteSize <= inEntries[idx].mOffset ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidArguments );
		}
		return PvdCommLayerError::None;
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::defineArrayProperty( PxU32 inNamespace
																								, PxU32 inClass
																								, const char* inName
																								, PxU32 inArrayClass
																								, PxU32 inKey )
	{
		TScopedLockType theLocker( mMutex );
		if ( !inName ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidName );
		if ( !inKey ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
		if ( !inArrayClass ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
		ClassDescription* existing = mDataSystem.findClassDescription( inNamespace, inClass );
		if ( !existing ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidClass );

		if ( existing->mParentKey != InvalidKey() 
			&& mDataSystem.findParentPropertyDescription( existing, inKey ) ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::PropertyKeyCollision );

		PropertyDescription* theProperty = mDataSystem.findSpecificPropertyDescription( existing, inKey );
		if ( theProperty )
		{
			if ( !SafeStrEqual( inName, theProperty->mName )
				|| inArrayClass != theProperty->mArrayClass )
				PVD_RETURN_ERROR_CODE( PvdCommLayerError::PropertyDefinitionError );
			PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
		}
		
		ClassDescription* arrayClass = mDataSystem.findClassDescription( inNamespace, inArrayClass );
		if ( !arrayClass ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidClass );

		if ( existing->mClassLocked ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::ClassLocked );
		
		mDataSystem.addArrayProperty( existing, inName, inArrayClass, inKey );
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );

	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::defineBitflagNames( PxU32 inNamespace
																				, PxU32 inClass
																				, PxU32 inPropertyName
																				, const NamedValueDefinition* /*inDefinitions*/
																				, PxU32 /*inDefinitionLength*/ )
	{
		TScopedLockType theLocker( mMutex );
		ClassDescription* existing = mDataSystem.findClassDescription( inNamespace, inClass );
		if ( !existing ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidClass );
		PropertyDescription* theProperty = mDataSystem.findSpecificPropertyDescription( existing, inPropertyName );
		if ( !theProperty ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidProperty );
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::defineEnumerationNames( PxU32 inNamespace
																				, PxU32 inClass
																				, PxU32 inPropertyName
																				, const NamedValueDefinition* inDefinitions
																				, PxU32 inDefinitionLength )
	{
		return defineBitflagNames( inNamespace, inClass, inPropertyName, inDefinitions, inDefinitionLength );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::createInstance( PxU32 inNamespace, PxU32 inClass, PxU64 inInstanceId )
	{
		TScopedLockType theLocker( mMutex );
		if ( !inInstanceId ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
		if ( !inClass ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
		InstanceDescription* theInstance = mDataSystem.findInstanceDescription( inInstanceId );
		ClassDescription* theDescription = mDataSystem.findClassDescription( inNamespace, inClass );
		if ( theInstance ) 
			PVD_RETURN_ERROR_CODE( PvdCommLayerError::InstanceExists );
		if ( !theDescription ) 
			PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidClass );
		mDataSystem.createInstanceDescription( inNamespace, inClass, inInstanceId, false );
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PxU32 PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::getInstanceClass( PxU64 inInstanceId )
	{
		TScopedLockType theLocker( mMutex );
		if ( !inInstanceId ) return 0;
		InstanceDescription* theInstance = mDataSystem.findInstanceDescription( inInstanceId );
		if ( !theInstance ) return 0;
		return theInstance->mClassKey;
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::setPropertyValue( PxU64 inInstance, PxU32 inProperty, PvdCommLayerDatatype inDatatype )
	{
		TScopedLockType theLocker( mMutex );
		if ( !inInstance || !inProperty ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );

		InstanceDescription* theInstance = mDataSystem.findInstanceDescription( inInstance );
		if ( !theInstance ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidInstance );
		ClassDescription* theDescription = mDataSystem.findClassDescription( theInstance->mNamespace, theInstance->mClassKey );
		if ( !theDescription ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidClass );
		PropertyDescription* theProperty = mDataSystem.findPropertyDescription( theDescription, inProperty );
		if ( !theProperty ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidProperty );
		if ( theProperty->mDatatype != inDatatype ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::DatatypeMismatch );
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}


	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::beginPropertyBlock( PxU32 inNamespace
																					, PxU32 inClass
																					, const PxU32* inProperties
																					, const PvdCommLayerDatatype* inDatatypes
																					, PxU32 inPropertyCount )
	{
		TScopedLockType theLocker( mMutex );
		if ( inClass == 0 ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
		ClassDescription* theDescription = mDataSystem.findClassDescription( inNamespace, inClass );
		if ( !theDescription ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidClass );
		mDataSystem.clearDatatypes();
		for ( PxU32 idx = 0; idx < inPropertyCount; ++idx )
		{
			PxU32 thePropertyKey = inProperties[idx];
			if ( !thePropertyKey ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
			PropertyDescription* theProperty = mDataSystem.findPropertyDescription( theDescription, thePropertyKey );
			if ( !theProperty ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidProperty );
			if ( theProperty->mDatatype != inDatatypes[idx] ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::DatatypeMismatch );
			mDataSystem.addDatatype( theProperty->mDatatype );

		}
		mBlockClass = inClass;
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::sendPropertyBlock( PxU64 inInstance, const PvdCommLayerValue*  )
	{
		TScopedLockType theLocker( mMutex );
		if ( !inInstance ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
		InstanceDescription* theInstance = mDataSystem.findInstanceDescription( inInstance );
		if ( !theInstance ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidInstance );
		if ( !mDataSystem.isOrDerivesFrom( mBlockClass, theInstance ) ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InstanceClassMismatch );
		if ( theInstance->mIsArray ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InstanceTypeMismatch );
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::endPropertyBlock()
	{
		TScopedLockType theLocker( mMutex );
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}
	
	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::sendPropertyStruct( PxU64 inInstance, PxU32 /*structKey*/, PxU32 /*namespace*/, const PxU8* /*inData*/, PxU32 /*inDataLen*/ )
	{
		TScopedLockType theLocker( mMutex );
		InstanceDescription* theInstance = mDataSystem.findInstanceDescription( inInstance );
		if ( !theInstance ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidInstance );
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::addChild( PxU64 inParent, PxU64 inChild )
	{
		TScopedLockType theLocker( mMutex );
		if ( !inParent || !inChild ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
		InstanceDescription* theParent = mDataSystem.findInstanceDescription( inParent );
		if ( !theParent ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidInstance );
		if ( theParent->mIsArray ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InstanceTypeMismatch );

		InstanceDescription* theChild = mDataSystem.findInstanceDescription( inChild );
		if ( !theChild ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidInstance );
		if ( theChild->mIsArray ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InstanceTypeMismatch );
		if ( mDataSystem.parentContainsChild( theParent, theChild ) ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::ChildError );
		mDataSystem.addChild( theParent, theChild );

		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::removeChild( PxU64 inParent, PxU64 inChild )
	{
		TScopedLockType theLocker( mMutex );
		if ( !inParent || !inChild ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
		InstanceDescription* theParent = mDataSystem.findInstanceDescription( inParent );
		if ( !theParent ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidInstance );
		if ( theParent->mIsArray ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InstanceTypeMismatch );

		InstanceDescription* theChild = mDataSystem.findInstanceDescription( inChild );
		if ( !theChild ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidInstance );
		if ( theChild->mIsArray ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InstanceTypeMismatch );
		if ( mDataSystem.parentDoesNotContainChild( theParent, theChild ) ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::ChildError );
		mDataSystem.removeChild( theParent, theChild );

		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}
	

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::removeAllChildren( PxU64 inInstanceId, PxU32 inChildClass )
	{
		TScopedLockType theLocker( mMutex );
		if ( !inInstanceId ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
		InstanceDescription* theParent = mDataSystem.findInstanceDescription( inInstanceId );
		if ( !theParent ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidInstance );
		if ( theParent->mIsArray ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InstanceTypeMismatch );
		if ( inChildClass )
		{
			ClassDescription* theFilter = mDataSystem.findClassDescription( theParent->mNamespace, inChildClass );
			if ( !theFilter ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidClass );
		}
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::destroyInstance( PxU64 inInstance )
	{
		TScopedLockType theLocker( mMutex );
		if ( !inInstance ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );
		InstanceDescription* theInstance = mDataSystem.findInstanceDescription( inInstance );
		if ( !theInstance ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidInstance );
		mDataSystem.destroyInstance( theInstance );
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::beginArrayBlock( PxU32 inNamespace
																								, PxU32 inClass
																								, PxU64 inInstance
																								, const PxU32* inProperties
																								, const PvdCommLayerDatatype* 
																								, PxU32 inNumProps )
	{
		TScopedLockType theLocker( mMutex );
		if ( !inClass || !inInstance ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );

		InstanceDescription* theInstance = mDataSystem.findInstanceDescription( inInstance );
		ClassDescription* theClass = mDataSystem.findClassDescription( inNamespace, inClass );
		if ( !theClass ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidClass );
		if ( theInstance ) //If the instance does exist, make sure its meta data matches
		{
			if ( !theInstance->mIsArray ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InstanceTypeMismatch );
			if ( theInstance->mClassKey != inClass ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InstanceClassMismatch );
		}
		else //Else create the instance
			mDataSystem.createInstanceDescription( inNamespace, inClass, inInstance, true );

		//Clear the datatypes.  These are used to check each array block's data
		//so we know we are getting the advertized data types.
		mDataSystem.clearDatatypes();
		for ( PxU32 idx = 0; idx < inNumProps; ++idx )
		{
			PxU32 thePropertyKey = inProperties[idx];
			if ( !thePropertyKey ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidKey );

			PropertyDescription* theProperty = mDataSystem.findPropertyDescription( theClass, inProperties[idx] );
			if ( !theProperty ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidProperty );
		}
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}
	
	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::beginArrayPropertyBlock( PxU64 , PxU32 , const PxU32* , const PvdCommLayerDatatype*, PxU32 )
	{
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::sendArrayObject( const PvdCommLayerValue* )
	{
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::sendArrayObjects( const PxU8*, PxU32 /*inStride*/, PxU32  )
	{
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::endArrayBlock()
	{
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::endArrayPropertyBlock()
	{
		return endArrayBlock();
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::beginSection( const char* inName )
	{
		TScopedLockType theLocker( mMutex );
		if ( inName == NULL ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidName );
		mDataSystem.beginSection( inName );
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::endSection( const char* inName )
	{
		TScopedLockType theLocker( mMutex );
		const char* theTop = mDataSystem.getTopSectionName();
		if ( inName == NULL ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidName );
		if ( theTop == NULL ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::NoOpenSection );
		if ( !SafeStrEqual( theTop, inName ) ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::SectionNameMismatch );

		mDataSystem.popTopSection();
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}
	
	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::namedEventWithInstance( PxU64 inInstance, const char* /*inName*/ )
	{
		TScopedLockType theLocker( mMutex );
		InstanceDescription* theInstance = mDataSystem.findInstanceDescription( inInstance );
		if ( !theInstance ) PVD_RETURN_ERROR_CODE( PvdCommLayerError::InvalidInstance );
		PVD_RETURN_ERROR_CODE( PvdCommLayerError::None );
	}
	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::handleRenderCommand( const PushRenderState& )
	{
		TScopedLockType theLocker( mMutex );
		++mRenderStackDepth;
		if ( mRenderStackDepth > 64 )
			PVD_RETURN_ERROR_CODE( PvdCommLayerError::StackOverflow );
		return checkRenderCommand();
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::handleRenderCommand( const PopRenderState& )
	{
		TScopedLockType theLocker( mMutex );
		if ( !mRenderStackDepth )
			PVD_RETURN_ERROR_CODE( PvdCommLayerError::StackUnderflow );
		--mRenderStackDepth;
		return checkRenderCommand();
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::handleRenderCommand( const PushTransform& )
	{
		TScopedLockType theLocker( mMutex );
		++mTransformStackDepth;
		if ( mTransformStackDepth > 64 )
			PVD_RETURN_ERROR_CODE( PvdCommLayerError::StackOverflow );
		return checkRenderCommand();
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	inline PvdCommLayerError PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::handleRenderCommand( const PopTransform& )
	{
		TScopedLockType theLocker( mMutex );
		if ( !mTransformStackDepth )
			PVD_RETURN_ERROR_CODE( PvdCommLayerError::StackUnderflow );
		--mTransformStackDepth;
		return checkRenderCommand();
	}
	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	void PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::addRef() 
	{ 
		TScopedLockType theLocker( mMutex );
		++mRefCount; 
	}

	template<typename TDataSystem, typename TMutexType, typename TScopedLockType, typename TDeleteOperatorType>
	void PvdConnectionBasicTypeChecker<TDataSystem,TMutexType,TScopedLockType,TDeleteOperatorType>::release()
	{
		PxU32 theRefCount;
		{
			TScopedLockType theLocker( mMutex );
			if ( mRefCount )
				--mRefCount;
			theRefCount = mRefCount;
		}
		if ( !theRefCount )
			TDeleteOperatorType()( this );
	}
}

#endif