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

#ifndef PVD_PVDCONNECTIONIMPL_H
#define PVD_PVDCONNECTIONIMPL_H

#include "PvdDataStreamImplTypes.h"
#include "PvdDataStream.h"
#include "PVDCommLayerTypes.h"
#include "PvdDataStreamWriterBase.h"
#include "PvdRender.h"
#include "PvdRenderInterfaceToCommands.h"
#include "PvdDataStreamHelpersImpl.h"

#pragma warning(push)
#pragma warning(disable:4355)	// 'this' : used in base member initializer list

namespace PVD
{
	template< bool TCheckErrors >
	struct PvdErrorChecker
	{
		PvdCommLayerError mError;
		PvdErrorChecker(){}
		PvdErrorChecker( const PvdCommLayerError& inError )
		{
			*this = inError;
		}
		//Ugghh, useless ugly warning work-around
		template<typename T>
		static inline T val(T v) {return v;}

		inline PvdErrorChecker& operator=( const PvdCommLayerError& inError )
		{
			if ( val(TCheckErrors) )
				PX_ASSERT( inError.mError == PvdCommLayerError::None || 
					inError.mError == PvdCommLayerError::NetworkError );
			mError = inError;
			return *this;
		}
		inline bool operator==( const PvdCommLayerError& inError ) const { return mError == inError; }
		inline bool operator!=( const PvdCommLayerError& inError ) const { return mError != inError; }
		inline operator PvdCommLayerError () const { return mError; }
	};
#ifdef PX_VC
#pragma warning( disable:4355 )
#endif

	class PvdDataStreamOwner
	{
	protected:
		virtual ~PvdDataStreamOwner(){}
	public:
		virtual void onPropertyStructDefinition( PxU32 inStructKey
												, PxU32 inNamespace
												, PxU32 inClass
												, PxU32 inStructByteSize
												, const PropertyStructEntry* inEntries
												, PxU32 inEntryCount ) = 0;

		//Returns a struct with no name and no properties when it fails.
		virtual RegisterPropertyStruct getStructDefinition( PxU32 inStructKey, PxU32 inNamespace ) = 0;
	};

	/**
	 *	The primary purpose of the connection implementation is to
	 *	check the function calls against a live metadata system
	 *	and to package them up into discrete events to be dealt
	 *	with by another entity.
	 */
	template<typename TTypeChecker,
			typename TWriter,
			typename TAllocatorType,
			typename TDeleteOperator=SDeleteOperator,
			bool TCheckErrors = true>
	class PvdDataStreamImpl : public PvdDataStream
	{
	public:
		typedef physx::shdfnd::Array<PxU32,TAllocatorType >						TPxU32Container;
		typedef physx::shdfnd::Array<PvdCommLayerDatatype,TAllocatorType >		TDatatypeContainer;
		typedef physx::shdfnd::Array<PvdCommLayerData,TAllocatorType >			TValueContainer;
		typedef TWriter															TLocalWriterType;
		typedef TAllocatorType													TLocalAllocatorType;
		typedef physx::shdfnd::HashMap<RegisterPropertyStructKey
										, RegisterPropertyStructEntry<TAllocatorType>
										, RegisterPropertyStructKeyHasher
										, TAllocatorType> TPropertyDefinitionStructHash;
	private:

		TTypeChecker*					mTypeChecker;
		TWriter*						mSerializer;
		TDatatypeContainer				mDatatypes;
		TValueContainer					mValues;
		PxU32							mPropertyCount;
		PxU32							mRefCount;
		PxU32							mNamespace;
		TPxU32Container					mNamespaceStack;
		PvdDataStreamWriterBase<TWriter, TDatatypeContainer> mWriter;
		PvdPropertyDefinitionHelperImpl mPropertyDefinitionHelper;
		PvdBeginPropertyBlockHelperImpl mBeginPropertyBlockHelper;
		PvdSendPropertyBlockHelperImpl	mSendPropertyBlockHelper;
		PvdDataStreamOwner*				mStreamOwner;
		TPropertyDefinitionStructHash	mLocalPropertyDefinitions;

		PvdDataStreamImpl( const PvdDataStreamImpl& inOther );
		PvdDataStreamImpl& operator=( const PvdDataStreamImpl& inOther );

	public:

		typedef	TTypeChecker	TTypeCheckerType;
		typedef TWriter			TWriterType;
		
		virtual ~PvdDataStreamImpl()
		{
			if ( mSerializer )
				mSerializer->destroy();
			mTypeChecker->release();
			mTypeChecker = NULL;
			mSerializer = NULL;
		}

		PvdDataStreamImpl(TWriter* inWriter, TTypeChecker* inTypeChecker, PvdDataStreamOwner* owner )
			: mTypeChecker( inTypeChecker )
			, mSerializer( inWriter )
			, mDatatypes( NULL )
			, mPropertyCount( 0 )
			, mRefCount( 0 )
			, mNamespace( NULL )
			, mWriter( inWriter )
			, mPropertyDefinitionHelper( *this )
			, mBeginPropertyBlockHelper( *this )
			, mSendPropertyBlockHelper( *this )
			, mStreamOwner( owner )
		{
			mTypeChecker->addRef();
		}

		inline const PvdCommLayerDatatype* getDatatypePointer() const 
		{
			if ( mDatatypes.size() )
				return mDatatypes.begin();
			return NULL;
		}

		inline void toDatatypeArray( const PvdCommLayerDatatype* inDatatypes, PxU32 inDTypeCount )
		{
			mDatatypes.clear();
			for ( PxU32 idx = 0; idx < inDTypeCount; ++idx )
				mDatatypes.pushBack( inDatatypes[idx] );
		}

		template<typename TStreamType>
		static PvdCommLayerError sendStreamInitialization( TStreamType* inOutStream ) 
		{ 
			TWriterType theWriter( inOutStream );
			PvdDataStreamWriterBase<TWriter, TDatatypeContainer> theBase( &theWriter );
			theBase.sendStreamInitialization();
			return PvdErrorChecker<TCheckErrors> ( theWriter.flush() );
		}

		PvdCommLayerError sendStreamInitialization() { return PvdErrorChecker<TCheckErrors>( mWriter.sendStreamInitialization() ); }
		
		virtual PvdCommLayerError setNamespace( const char* inNamespace ) 
		{ 
			mNamespace = safeStrHash( inNamespace );
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->setNamespace( inNamespace ) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.setNamespace( inNamespace );
			return retval;
		}

		//I have to do my own error checking here because the namespace
		//stack is connection specific
		virtual PvdCommLayerError pushNamespace()
		{
			PvdErrorChecker<TCheckErrors> retval;
			if ( mNamespaceStack.size() > 254 )
				retval = PvdCommLayerError::StackOverflow;

			if ( retval == PvdCommLayerError::None )
			{
				mNamespaceStack.pushBack( mNamespace );
				retval = mWriter.pushNamespace();
			}
			return retval;
		}

		virtual PvdCommLayerError popNamespace()
		{
			PvdErrorChecker<TCheckErrors> retval;
			if ( mNamespaceStack.size() == 0 )
				retval = PvdCommLayerError::StackUnderflow;

			if ( retval == PvdCommLayerError::None )
			{
				mNamespace = mNamespaceStack.back();
				mNamespaceStack.popBack();
				retval = mWriter.popNamespace();
			}
			return retval;
		}

		virtual PvdCommLayerError createClass( const char* inName, PxU32 inKey )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->createClass( mNamespace, inName, inKey ) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.createClass( inName, inKey );
			return retval;
		}

		virtual PvdCommLayerError deriveClass( PxU32 inParentKey, PxU32 inChildKey )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->deriveClass( mNamespace, inParentKey, inChildKey ) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.deriveClass( inParentKey, inChildKey );
			return retval;
		}

		virtual PvdCommLayerError defineProperty( PxU32 inClass
													, const char* inName
													, const char* inSemantic
													, PvdCommLayerDatatype inDatatype
													, PxU32 inKey)
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->defineProperty( mNamespace, inClass, inName, inSemantic, inDatatype, inKey ) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.defineProperty( inClass, inName, inSemantic, inDatatype, inKey );
			return retval;
		}
		
		virtual PvdCommLayerError definePropertyOnInstance( PxU64 inInstanceId
															, const char* inName
															, const char* inSemantic
															, PvdCommLayerDatatype inDatatype
															, PxU32 inKey)
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->definePropertyOnInstance( inInstanceId, inName, inSemantic, inDatatype, inKey ) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.definePropertyOnInstance( inInstanceId, inName, inSemantic, inDatatype, inKey );
			return retval;
		}

		virtual PvdCommLayerError definePropertyStruct( PxU32 inStructKey
															, PxU32 inClass
															, PxU32 inStructByteSize
															, const PropertyStructEntry* inEntries
															, PxU32 inEntryCount )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->definePropertyStruct( inStructKey, mNamespace, inClass, inStructByteSize, inEntries, inEntryCount ) );
			if ( retval == PvdCommLayerError::None )
			{
				if ( mStreamOwner != NULL )
					mStreamOwner->onPropertyStructDefinition( inStructKey, mNamespace, inClass, inStructByteSize, inEntries, inEntryCount );
				retval = mWriter.definePropertyStruct( inStructKey, inClass, inStructByteSize, inEntries, inEntryCount );
			}
			return retval;
		}
		
		virtual PvdCommLayerError defineArrayProperty( PxU32 inClass
														, const char* inName
														, PxU32 inArrayClass
														, PxU32 inKey )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->defineArrayProperty( mNamespace, inClass, inName, inArrayClass, inKey ) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.defineArrayProperty( inClass, inName, inArrayClass, inKey );
			return retval;
		}

		virtual PvdCommLayerError defineBitflagNames( PxU32 inClass
														, PxU32 inPropertyKey
														, const NamedValueDefinition* inDefinitions
														, PxU32 inDefinitionLength )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->defineBitflagNames( mNamespace, inClass, inPropertyKey, inDefinitions, inDefinitionLength ) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.defineBitflagNames( inClass, inPropertyKey, inDefinitions, inDefinitionLength );
			return retval;
		}

		virtual PvdCommLayerError defineEnumerationNames( PxU32 inClass
															, PxU32 inPropertyKey
															, const NamedValueDefinition* inDefinitions
															, PxU32 inDefinitionLength )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->defineEnumerationNames( mNamespace, inClass, inPropertyKey, inDefinitions, inDefinitionLength ) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.defineEnumerationNames( inClass, inPropertyKey, inDefinitions, inDefinitionLength );
			return retval;
		}

		virtual PvdCommLayerError createInstance( PxU32 inClass, PxU64 inInstanceId, PVD::EInstanceUIFlags inFlags )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->createInstance( mNamespace, inClass, inInstanceId ) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.createInstance( inClass, inInstanceId, inFlags.mFlags );
			return retval;
		}
		
		virtual PvdCommLayerError createInstance( PxU32 inClass, PxU64 inInstanceId )
		{
			return createInstance( inClass, inInstanceId, 0 );
		}

		virtual PvdCommLayerError setPropertyValue( PxU64 inInstance, PxU32 inProperty, const PvdCommLayerValue& inValue )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->setPropertyValue( inInstance, inProperty, inValue.getDatatype() ) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.setPropertyValue( inInstance, inProperty, inValue );
			return retval;
		}

		virtual PvdCommLayerError beginPropertyBlock( PxU32 inClass, const PxU32* inProperties, const PvdCommLayerDatatype* inDatatypes, PxU32 inPropertyCount )
		{
			mPropertyCount = 0;
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->beginPropertyBlock( mNamespace, inClass, inProperties, inDatatypes, inPropertyCount ) );
			if ( retval == PvdCommLayerError::None )
			{
				toDatatypeArray( inDatatypes, inPropertyCount );
				mPropertyCount = inPropertyCount;
				retval = mWriter.beginPropertyBlock(inClass, inProperties, getDatatypePointer(), inPropertyCount);
			}
			return retval;
		}

		virtual PvdCommLayerError sendPropertyBlock( PxU64 inInstance, const PvdCommLayerValue* inValues )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->sendPropertyBlock(inInstance, inValues) );
			if ( retval == PvdCommLayerError::None )
			{
				mValues.clear();
				for ( PxU32 idx = 0; idx < mPropertyCount; ++idx )
					mValues.pushBack( inValues[idx].getData() );
				retval = mWriter.PropertyBlock(inInstance, mValues.begin());
			}
			return retval;
		}
		
		virtual PvdCommLayerError sendPropertyBlock( PxU64 inInstance, const PxU8* inData, PxU32 inDataLen, const PvdCommLayerValue* inValues )
		{
			PvdCommLayerData* theData = NULL;
			if ( inValues )
			{
				mValues.clear();
				for ( PxU32 idx = 0; idx < mPropertyCount; ++idx )
					mValues.pushBack( inValues[idx].getData() );
				theData = mValues.begin();
			}
			return mWriter.PropertyBlock( inInstance, inData, inDataLen, theData );
		}

		virtual PvdCommLayerError endPropertyBlock()
		{
			mPropertyCount = 0;
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->endPropertyBlock() );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.endPropertyBlock();
			return retval;
		}
		virtual PvdCommLayerError sendPropertyStruct( PxU64 inInstance, PxU32 inStructKey, const PxU8* inData, PxU32 inDataLen )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->sendPropertyStruct( inInstance, inStructKey, mNamespace, inData, inDataLen ) );
			if ( retval == PvdCommLayerError::None )
			{
				RegisterPropertyStructKey theKey( inStructKey, mNamespace );
				const typename TPropertyDefinitionStructHash::Entry* localEntry = mLocalPropertyDefinitions.find( theKey );
				if ( localEntry == NULL && mStreamOwner )
				{
					RegisterPropertyStruct theStruct = mStreamOwner->getStructDefinition( inStructKey, mNamespace );
					if ( theStruct.mStructByteSize && theStruct.mPropertyCount )
					{
						mLocalPropertyDefinitions.insert( theKey, RegisterPropertyStructEntry<TAllocatorType>() );
						localEntry = mLocalPropertyDefinitions.find( theKey );
						RegisterPropertyStructEntry<TAllocatorType>& theEntryStruct( const_cast<RegisterPropertyStructEntry<TAllocatorType> &>( localEntry->second ) );
						theEntryStruct.setup( theStruct.mClass, theStruct.mStructByteSize, theStruct.mEntries, theStruct.mPropertyCount );
					}
				}
				if ( localEntry != NULL )
				{
					RegisterPropertyStructEntry<TAllocatorType>& theStruct( const_cast<RegisterPropertyStructEntry<TAllocatorType> &>( localEntry->second ) );
					if ( inDataLen != theStruct.mByteSize )
						retval = PvdCommLayerError::InvalidArguments;
					else
					{
						retval = mWriter.sendPropertyStruct( inInstance
															, inStructKey
															, theStruct.mClass
															, theStruct.mEntries.begin()
															, static_cast<PxU32>( theStruct.mEntries.size() )
															, theStruct.mStringOffsets.begin()
															, theStruct.mStringOffsets.size()
															, inData
															, inDataLen );
					}
				}
			}
			return retval;
		}
		virtual PvdCommLayerError addChild( PxU64 inParent, PxU64 inChild )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->addChild(inParent, inChild) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.addChild(inParent, inChild);
			return retval;
		}
		virtual PvdCommLayerError removeChild( PxU64 inParent, PxU64 inChild )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->removeChild(inParent, inChild) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.removeChild(inParent, inChild);
			return retval;
		}
		virtual PvdCommLayerError removeAllChildren( PxU64 inInstanceId, PxU32 inChildClass )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->removeAllChildren(inInstanceId, inChildClass) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.removeAllChildren(inInstanceId, inChildClass);
			return retval;
		}
		virtual PvdCommLayerError destroyInstance( PxU64 inInstance )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->destroyInstance(inInstance) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.destroyInstance(inInstance);
			return retval;
		}
		virtual PvdCommLayerError beginArrayBlock( PxU32 inClass, PxU64 inInstance, const PxU32* inProperties, const PvdCommLayerDatatype* inDatatypes, PxU32 inNumProps )
		{
			mPropertyCount = 0;
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->beginArrayBlock(mNamespace, inClass, inInstance, inProperties, inDatatypes, inNumProps ) );
			if ( retval == PvdCommLayerError::None )
			{
				toDatatypeArray( inDatatypes, inNumProps );
				mPropertyCount = inNumProps;
				retval = mWriter.beginArrayBlock(inClass, inInstance, inProperties, getDatatypePointer(), inNumProps);
			}
			return retval;
		}

		virtual PvdCommLayerError beginArrayPropertyBlock( PxU64 inInstance
														, PxU32 inProperty
														, const PxU32* inProperties
														, const PvdCommLayerDatatype* inDatatypes
														, PxU32 inPropertyCount )
		{
			mPropertyCount = 0;
			
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->beginArrayPropertyBlock(inInstance, inProperty, inProperties, inDatatypes, inPropertyCount ) );
			if ( retval == PvdCommLayerError::None )
			{
				toDatatypeArray( inDatatypes, inPropertyCount );
				mPropertyCount = inPropertyCount;
				retval = mWriter.beginArrayPropertyBlock( inInstance, inProperty, inProperties, inDatatypes, inPropertyCount );
			}
			return retval;
		}

		virtual PvdCommLayerError sendArrayObject( const PvdCommLayerValue* inValues )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->sendArrayObject(inValues) );
			if ( retval == PvdCommLayerError::None )
			{
				if ( mPropertyCount )
				{
					mValues.clear();
					for ( PxU32 idx = 0; idx < mPropertyCount; ++idx )
						mValues.pushBack( inValues[idx].getData() );
					retval = mWriter.ArrayObject(&mValues[0]);
				}
			}
			return retval;
		}
		
		virtual PvdCommLayerError sendArrayObjects( const PxU8* inData, PxU32 inStride, PxU32 inCount )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->sendArrayObjects(inData, inStride, inCount) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.ArrayObjects( inData, inStride, inCount );
			return retval;
		}

		virtual PvdCommLayerError endArrayBlock()
		{
			mPropertyCount = 0;
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->endArrayBlock() );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.endArrayBlock();
			return retval;
		}

		virtual PvdCommLayerError endArrayPropertyBlock()
		{
			mPropertyCount = 0;
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->endArrayPropertyBlock() );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.endArrayPropertyBlock();
			return retval;
		}

		virtual PvdCommLayerError beginSection( const char* inName )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->beginSection(inName) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.beginSection(inName, physx::shdfnd::Time::getCurrentCounterValue() );
			return retval;
		}
		virtual PvdCommLayerError endSection(const char* inName)
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->endSection(inName) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.endSection(inName, physx::shdfnd::Time::getCurrentCounterValue());
			return retval;
		}
		virtual PvdCommLayerError beginFrame()
		{
			return beginSection( "frame" );
		}
		virtual PvdCommLayerError endFrame()
		{
			return endSection( "frame" );
		}
		virtual PvdCommLayerError sendError( const char* inType, const char* inMessage, const char* inFile, PxU32 inLine )
		{
			return PvdErrorChecker<TCheckErrors>( mWriter.Error(inType, inMessage, inFile, inLine) );
		}
		virtual PvdCommLayerError sendWarning( const char* inType, const char* inMessage, const char* inFile, PxU32 inLine )
		{
			return PvdErrorChecker<TCheckErrors>( mWriter.Warning(inType, inMessage, inFile, inLine) );
		}

		virtual PvdCommLayerError namedEventWithInstance( PxU64 inInstanceId, const char* inName )
		{
			PvdErrorChecker<TCheckErrors> retval( mTypeChecker->namedEventWithInstance(inInstanceId, inName) );
			if ( retval == PvdCommLayerError::None )
				retval = mWriter.namedEventWithInstance( inInstanceId, inName );
			return retval;
		}

		virtual PvdCommLayerError sendFrameMarkerWithInstance( PxU64 inInstanceId )
		{
			return namedEventWithInstance( inInstanceId, "frame" );
		}
		
		virtual PvdCommLayerError disconnect()
		{
			return PvdErrorChecker<TCheckErrors>( mWriter.disconnect() );
		}

		virtual PvdCommLayerError flush()
		{
			return PvdErrorChecker<TCheckErrors>( mSerializer->flush() );
		}
		
		virtual PvdCommLayerError localFlush()
		{
			return PvdErrorChecker<TCheckErrors>( mSerializer->localFlush() );
		}

		void disableCaching()
		{
			mSerializer->disableCaching();
		}

		//================
		// PvdRender
		//================
		virtual PvdCommLayerError pushRenderState() { return handleRenderCommand( createPushRenderState() ); }
		virtual PvdCommLayerError setCurrentInstance( PxU64 inInstanceId ) { return handleRenderCommand( createSetCurrentInstance( inInstanceId ) ); }
		virtual PvdCommLayerError setCurrentColor( Color inColor ) { return handleRenderCommand( createSetCurrentColor( inColor ) ); }
		virtual PvdCommLayerError setCurrentTextScale( PxF32 inScale ) { return handleRenderCommand( createSetCurrentTextScale( inScale ) ); }
		virtual PvdCommLayerError setCurrentRenderFlags( const RenderFlags& inFlags ) { return handleRenderCommand( createSetCurrentRenderFlags( inFlags.mFlags ) ); }
		virtual PvdCommLayerError addCurrentRenderFlags( const RenderFlags& inFlags ) { return handleRenderCommand( createAddCurrentRenderFlags( inFlags.mFlags ) ); }
		virtual PvdCommLayerError removeCurrentRenderFlags( const RenderFlags& inFlags ) { return handleRenderCommand( createRemoveCurrentRenderFlags( inFlags.mFlags ) ); }
		virtual PvdCommLayerError popRenderState() { return handleRenderCommand( createPopRenderState() ); }

		virtual PvdCommLayerError pushTransform() { return handleRenderCommand( createPushTransform() ); }
		virtual PvdCommLayerError setTransform(const RenderTransform& inTransform) { return handleRenderCommand( createSetTransform( inTransform.getType(), inTransform.getData() ) ); }
		virtual PvdCommLayerError multiplyTransform( const RenderTransform& inTransform ) { return handleRenderCommand( createMultiplyTransform( inTransform.getType(), inTransform.getData() ) ); }
		virtual PvdCommLayerError popTransform() { return handleRenderCommand( createPopTransform() ); }
		
		virtual PvdCommLayerError drawPrimitive(const RenderPrimitive& inPrimitive)
		{
			return drawPrimitive( inPrimitive, createIdentityTransform() );
		}
		virtual PvdCommLayerError drawPrimitive(const RenderPrimitive& inPrimitive, const RenderTransform& inTransform) 
		{ 
			return handleRenderCommand( createDrawPrimitive( inTransform.getType(), inTransform.getData(), inPrimitive.getType(), inPrimitive.getData() ) );
		}
		virtual PvdCommLayerError drawPrimitive( const RenderPrimitive& inPrimitive, const RenderTransform& inTransform, const RenderState& inState )
		{
			return handleRenderCommand( createDrawRenderStatePrimitive( createDrawPrimitive( inTransform.getType(), inTransform.getData()
																				, inPrimitive.getType(), inPrimitive.getData() ), inState ) );
		}
		template<typename TRenderCommandType>
		inline PvdCommLayerError handleRenderCommand( const TRenderCommandType& inType )
		{
			PvdErrorChecker<TCheckErrors> theChecker( mTypeChecker->handleRenderCommand( inType ) );
			if ( theChecker == PvdCommLayerError::None )
				theChecker = mWriter.mSerializer->sendEvent( inType );
			return theChecker;
		}
		
		/////////////////////////////////////////////////////////////
		// Retrieving information from the debugger
		/////////////////////////////////////////////////////////////
		virtual PvdCommLayerError isDebuggerMessageAvailable( bool& outAvailable )
		{
			outAvailable = false;
			return PvdCommLayerError::None;
		}

		virtual PvdCommLayerError getNextDebuggerMessage( PvdDebugMessage& /*outMessage*/ )
		{
			return PvdCommLayerError::None;
		}

		/////////////////////////////////////////////////////////////
		// Lifetime management and diagnostics.
		/////////////////////////////////////////////////////////////
		virtual bool isConnected() const 
		{
			bool connected = mSerializer != NULL;
			if ( connected && mSerializer )
				connected = mSerializer->isConnected();
			return connected;
		}
		
		virtual PvdPropertyDefinitionHelper& getPropertyDefinitionHelper() { mPropertyDefinitionHelper.reset(); return mPropertyDefinitionHelper; }
		virtual PvdBeginPropertyBlockHelper& getBeginPropertyBlockHelper() { mBeginPropertyBlockHelper.reset(); return mBeginPropertyBlockHelper; }
		virtual PvdSendPropertyBlockHelper&	 getSendPropertyBlockHelper() { mSendPropertyBlockHelper.reset(); return mSendPropertyBlockHelper; }

		virtual PvdUnknown* queryInterface( const char* /*inExtensionInterface*/ )
		{
			return NULL;
		}
		
		virtual void addRef()
		{
			++mRefCount;
		}
		virtual void release()
		{
			if ( mRefCount ) --mRefCount;
			if ( !mRefCount ) TDeleteOperator()( this );
		}
	};
}

#pragma warning(pop)

#endif
