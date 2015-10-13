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

#ifndef PVD_STLTYPECHECKERDATASYSTEM_H
#define PVD_STLTYPECHECKERDATASYSTEM_H

#include "PvdDataStreamImplTypes.h"
#include "PVDCommLayerTypes.h"
#include "PsArray.h"
#include "PsHashMap.h"
#include "ClientAllocator.h"

namespace PVD
{
	
	struct Uint32HashFunc : physx::shdfnd::Hash<PxU32>
	{
	};
	
	struct Uint64HashFunc : physx::shdfnd::Hash<PxU64>
	{
	};



	template<typename TAllocator>
	struct FullClassDescription
	{
		typedef physx::shdfnd::Array<PropertyDescription,TAllocator>	TPropertyDescriptionContainer;

		ClassDescription				mDescription;
		TPropertyDescriptionContainer	mProperties;

		PX_INLINE FullClassDescription( PxU32 inNamespace, const char* inName, PxU32 inKey )
			: mDescription( inNamespace, inName, inKey )
		{
		}
		PX_INLINE FullClassDescription(){}

		PX_INLINE FullClassDescription& operator=( const FullClassDescription& inOther )
		{
			mDescription = inOther.mDescription;
			mProperties = inOther.mProperties;
		}
	};

	/**
	 *	Implements the data storage mechanism for the type checker.
	 */
	template<typename TPropertyAllocator = ClientAllocator<PxU8> >
	class STDTypeCheckerDataSystem
	{
		typedef FullClassDescription<TPropertyAllocator>												TClassDescription;
		typedef physx::shdfnd::HashMap<PxU32,TClassDescription*,Uint32HashFunc,TPropertyAllocator>		TU32ClassDescriptionHash;
		typedef physx::shdfnd::HashMap<PxU64,InstanceDescription*,Uint64HashFunc,TPropertyAllocator>	TU32InstanceDescriptionHash;
		typedef physx::shdfnd::Array<PvdCommLayerDatatype,TPropertyAllocator>							TConnectionDatatypeContainer;
		typedef physx::shdfnd::Array<const char*,TPropertyAllocator>									TCharPtrContainer;


		TU32ClassDescriptionHash		mClassDescriptions;
		TU32InstanceDescriptionHash		mInstanceDescriptions;
		TConnectionDatatypeContainer	mDatatypes;
		TCharPtrContainer				mSections;
		TPropertyAllocator				mClassDescAllocator;
		TPropertyAllocator				mInstanceDescriptionAllocator;

	public:
		inline ~STDTypeCheckerDataSystem();

		inline void addClassDescription( PxU32 inNamespace, const char* inName, PxU32 inKey );
		inline ClassDescription* findClassDescription( PxU32 inNamespace, PxU32 inKey );
		inline bool checkForPropertyCollision( ClassDescription* inChild, ClassDescription* inParent );
		//Recursive, check this class first then parents.
		inline PropertyDescription* findPropertyDescription( ClassDescription* inDescription, PxU32 inPropertyKey );
		//Check only the parents, if they exist, of this class
		inline PropertyDescription* findParentPropertyDescription( ClassDescription* inDescription, PxU32 inPropertyKey );
		//Check only the properties on this class.
		inline PropertyDescription* findSpecificPropertyDescription( ClassDescription* inDescription, PxU32 inPropertyKey );
		inline void addProperty( ClassDescription* inDescription, const char* inName, const char* inSemantic, PxU32 inPropertyKey, PvdCommLayerDatatype inDatatype );
		inline void addArrayProperty( ClassDescription* inDescription, const char* inName, PxU32 inArrayClass, PxU32 inPropertyKey );
		inline InstanceDescription* findInstanceDescription( PxU64 inInstanceId );
		inline void createInstanceDescription( PxU32 inNamespace, PxU32 inClass, PxU64 inKey, bool inIsArray );
		inline void clearDatatypes();
		inline void addDatatype( PvdCommLayerDatatype inDatatype );
		inline const PvdCommLayerDatatype* getDatatypePointer();
		inline PxU32 getDatatypeCount();
		inline PvdCommLayerDatatype getDatatype( PxU32 inIdx );
		inline bool parentContainsChild( InstanceDescription* /*inParent*/, InstanceDescription* /*inChild*/ ) { return false; }
		inline bool parentDoesNotContainChild( InstanceDescription* /*inParent*/, InstanceDescription* /*inChild*/ ) { return false; }
		inline void addChild( InstanceDescription*, InstanceDescription*) {}
		inline void removeChild( InstanceDescription*, InstanceDescription*) {}
		inline void destroyInstance( InstanceDescription* inInstance );
		inline bool isOrDerivesFrom( PxU32 inClassKey, InstanceDescription* inInstance );
		inline void beginSection( const char* inName );
		inline const char* getTopSectionName();
		inline void popTopSection();

	protected:
		inline void doDeleteInstance( InstanceDescription* inInstance );
	};

	template<typename TPropertyAllocator>
	inline STDTypeCheckerDataSystem<TPropertyAllocator>::~STDTypeCheckerDataSystem()
	{
		//Run through the class description pointers and delete them.
		for ( typename TU32ClassDescriptionHash::Iterator theIter = mClassDescriptions.getIterator();
			theIter.done() == false;
			++theIter )
		{
			theIter->second->~TClassDescription();
			mClassDescAllocator.deallocate(theIter->second);
		}
		for ( typename TU32InstanceDescriptionHash::Iterator theIter = mInstanceDescriptions.getIterator();
			theIter.done() == false;
			++theIter )
		{
			doDeleteInstance( theIter->second );
		}
	}
	
	template<typename TPropertyAllocator>
	inline void STDTypeCheckerDataSystem<TPropertyAllocator>::addClassDescription( PxU32 inNamespace, const char* inName, PxU32 inKey )
	{

		TClassDescription* theDescription( reinterpret_cast<TClassDescription*>( mClassDescAllocator.allocate( sizeof( TClassDescription ), __FILE__, __LINE__ ) ) );
		new (theDescription) TClassDescription( inNamespace, inName, inKey );
		inKey = inKey ^ inNamespace;
		mClassDescriptions.insert( inKey, theDescription );
	}
	
	template<typename TPropertyAllocator>
	inline ClassDescription* STDTypeCheckerDataSystem<TPropertyAllocator>::findClassDescription( PxU32 inNamespace, PxU32 inKey )
	{
		inKey = inNamespace ^ inKey;
		const typename TU32ClassDescriptionHash::Entry* theDesc = mClassDescriptions.find( inKey );
		if ( theDesc )
			return const_cast<ClassDescription*>( &theDesc->second->mDescription );
		return NULL;
	}
	
	template<typename TPropertyAllocator>
	inline bool STDTypeCheckerDataSystem<TPropertyAllocator>::checkForPropertyCollision( ClassDescription* inChild, ClassDescription* inParent )
	{
		FullClassDescription<TPropertyAllocator>* theFullChild( reinterpret_cast<FullClassDescription<TPropertyAllocator>*>( inChild ) );
		PxU32 theCount = static_cast< PxU32>( theFullChild->mProperties.size() );
		for ( PxU32 idx = 0; idx < theCount; ++idx )
			if ( findPropertyDescription( inParent, theFullChild->mProperties[idx].mKey ) )
				return true;
		return false;
	}
	
	template<typename TPropertyAllocator>
	inline PropertyDescription* STDTypeCheckerDataSystem<TPropertyAllocator>::findPropertyDescription( ClassDescription* inDescription, PxU32 inPropertyKey )
	{
		PropertyDescription* theSpecific = findSpecificPropertyDescription( inDescription, inPropertyKey );
		if ( theSpecific != NULL )
			return theSpecific;
		return findParentPropertyDescription( inDescription, inPropertyKey );
	}
	
	template<typename TPropertyAllocator>
	inline PropertyDescription* STDTypeCheckerDataSystem<TPropertyAllocator>::findParentPropertyDescription( ClassDescription* inDescription, PxU32 inPropertyKey )
	{
		if ( inDescription->mParentKey != InvalidKey() )
		{
			ClassDescription* theParent = findClassDescription( inDescription->mNamespace, inDescription->mParentKey );
			return findPropertyDescription( theParent, inPropertyKey );
		}
		return NULL;
	}
	
	template<typename TPropertyAllocator>
	inline PropertyDescription* STDTypeCheckerDataSystem<TPropertyAllocator>::findSpecificPropertyDescription( ClassDescription* inDescription, PxU32 inPropertyKey )
	{
		FullClassDescription<TPropertyAllocator>* theFullDescription( reinterpret_cast<FullClassDescription<TPropertyAllocator>*>( inDescription ) );
		PxU32 theCount = static_cast< PxU32>( theFullDescription->mProperties.size() );
		for ( PxU32 idx = 0; idx < theCount; ++idx )
		{
			if ( theFullDescription->mProperties[idx].mKey == inPropertyKey )
				return &theFullDescription->mProperties[idx];
		}
		return NULL;
	}
	template<typename TPropertyAllocator>
	inline void STDTypeCheckerDataSystem<TPropertyAllocator>::addProperty( ClassDescription* inDescription
														, const char* inName
														, const char* inSemantic
														, PxU32 inPropertyKey
														, PvdCommLayerDatatype inDatatype )
	{
		FullClassDescription<TPropertyAllocator>* theFullDescription( reinterpret_cast<FullClassDescription<TPropertyAllocator>*>( inDescription ) );
		theFullDescription->mProperties.pushBack( PropertyDescription( inName
																		, inSemantic
																		, static_cast<PxU32>(theFullDescription->mProperties.size())
																		, inPropertyKey
																		, inDatatype ) );
	}
	template<typename TPropertyAllocator>
	inline void STDTypeCheckerDataSystem<TPropertyAllocator>::addArrayProperty( ClassDescription* inDescription, const char* inName, PxU32 inArrayClass, PxU32 inPropertyKey )
	{
		FullClassDescription<TPropertyAllocator>* theFullDescription( reinterpret_cast<FullClassDescription<TPropertyAllocator>*>( inDescription ) );
		theFullDescription->mProperties.pushBack( PropertyDescription( inName
																		, static_cast<PxU32>(theFullDescription->mProperties.size())
																		, inArrayClass
																		, inPropertyKey ) );
	}
	template<typename TPropertyAllocator>
	inline InstanceDescription* STDTypeCheckerDataSystem<TPropertyAllocator>::findInstanceDescription( PxU64 inInstanceId )
	{
		const typename TU32InstanceDescriptionHash::Entry* theEntry( mInstanceDescriptions.find( inInstanceId ) );
		if ( theEntry )
			return const_cast< InstanceDescription* >( theEntry->second );
		return NULL;
	}
	template<typename TPropertyAllocator>
	inline void STDTypeCheckerDataSystem<TPropertyAllocator>::createInstanceDescription( PxU32 inNamespace, PxU32 inClass, PxU64 inKey, bool inIsArray )
	{

		InstanceDescription* theDescription = reinterpret_cast<InstanceDescription*>( mInstanceDescriptionAllocator.allocate( sizeof(InstanceDescription), __FILE__, __LINE__) );
		new (theDescription) InstanceDescription( inNamespace, inClass, inKey, inIsArray );
		mInstanceDescriptions.insert( inKey, theDescription );
	}
	template<typename TPropertyAllocator>
	inline void STDTypeCheckerDataSystem<TPropertyAllocator>::clearDatatypes() { mDatatypes.clear(); }
	
	template<typename TPropertyAllocator>
	inline void STDTypeCheckerDataSystem<TPropertyAllocator>::addDatatype( PvdCommLayerDatatype inDatatype )
	{
		mDatatypes.pushBack( inDatatype );
	}
	template<typename TPropertyAllocator>
	inline const PvdCommLayerDatatype* STDTypeCheckerDataSystem<TPropertyAllocator>::getDatatypePointer()
	{
		return mDatatypes.begin();
	}
	template<typename TPropertyAllocator>
	inline PxU32 STDTypeCheckerDataSystem<TPropertyAllocator>::getDatatypeCount() { return static_cast< PxU32 >( mDatatypes.size() ); }
	
	template<typename TPropertyAllocator>
	inline PvdCommLayerDatatype STDTypeCheckerDataSystem<TPropertyAllocator>::getDatatype( PxU32 inIdx )
	{
		return mDatatypes[inIdx];
	}
	template<typename TPropertyAllocator>
	inline void STDTypeCheckerDataSystem<TPropertyAllocator>::destroyInstance( InstanceDescription* inInstance )
	{
		mInstanceDescriptions.erase( inInstance->mKey );
		doDeleteInstance( inInstance );
	}
	template<typename TPropertyAllocator>
	inline bool STDTypeCheckerDataSystem<TPropertyAllocator>::isOrDerivesFrom( PxU32 inClassKey, InstanceDescription* inInstance )
	{
		ClassDescription* theClass = findClassDescription(inInstance->mNamespace, inInstance->mClassKey);
		while((theClass->mKey != inClassKey) && (theClass->mParentKey != InvalidKey()))
		{
			theClass = findClassDescription(inInstance->mNamespace, theClass->mParentKey);
		}
		return theClass->mKey == inClassKey;
	}
	
	template<typename TPropertyAllocator>
	inline void STDTypeCheckerDataSystem<TPropertyAllocator>::beginSection( const char* inName )
	{
		mSections.pushBack( inName );
	}

	template<typename TPropertyAllocator>
	inline const char* STDTypeCheckerDataSystem<TPropertyAllocator>::getTopSectionName()
	{
		if ( !mSections.empty() )
			return mSections.back();
		return NULL;
	}

	template<typename TPropertyAllocator>
	inline void STDTypeCheckerDataSystem<TPropertyAllocator>::popTopSection()
	{
		mSections.popBack();
	}
	template<typename TPropertyAllocator>
	inline void STDTypeCheckerDataSystem<TPropertyAllocator>::doDeleteInstance( InstanceDescription* inInstance )
	{
		inInstance->~InstanceDescription();
		mInstanceDescriptionAllocator.deallocate( inInstance );
	}
}

#endif