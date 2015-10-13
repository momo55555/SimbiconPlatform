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

#ifndef PVD_PVDCONNECTIONTRAITS_H
#define PVD_PVDCONNECTIONTRAITS_H
#include "PvdDataStreamWriter.h"
#include "PVDContainerCommOutStream.h"
#include "PvdDataStreamReader.h"
#include "PvdDataStreamEventInStream.h"
#include "PvdDataStreamEventOutStream.h"
#include "STLTypeCheckerDataSystem.h"
#include "PVDConnectionBasicTypeChecker.h"
#include "PvdConnectionNullTypeChecker.h"
#include "PvdDataStreamImpl.h"
#include "PvdConnectionImpl.h"


namespace PVD
{
	class PvdConnectionManager;
	template<typename TTraitsType>
	struct STraitsDeleteOperator
	{
		template<typename TDataType>
		inline void operator()( TDataType* inType )
		{
			inType->~TDataType();
			TTraitsType::DeallocateMemory( inType );
		}
	};
	
	template<typename TTraitsType>
	struct STraitsUntrackedDeleteOperator
	{
		template<typename TDataType>
		inline void operator()( TDataType* inType )
		{
			inType->~TDataType();
			TTraitsType::UntrackedDeallocateMemory( (char*)inType );
		}
	};

	template<typename TMutex
			, typename TScopedLock
			, typename TAllocator
			, typename TUntrackedAllocator
			, PxU32 TMaxThreadCacheSize = 0x4000>
	struct PvdConnectionTraits
	{
		typedef PvdConnectionTraits<TMutex,TScopedLock,TAllocator,TUntrackedAllocator,TMaxThreadCacheSize> TThisType;
		typedef TMutex																													TMutexType;
		typedef TScopedLock																												TScopedLockType;
		typedef TAllocator																												TAllocatorType;
		typedef TUntrackedAllocator																										TUntrackedAllocatorType;
		typedef STraitsDeleteOperator< TThisType>																						TDeleteOperatorType;
		typedef STraitsUntrackedDeleteOperator<TThisType>																				TUntrackedDeleteOperatorType;
		typedef STDTypeCheckerDataSystem<TUntrackedAllocatorType>																		TSTDTypeCheckerDataSystem;
		typedef PvdConnectionBasicTypeChecker<TSTDTypeCheckerDataSystem, TMutexType, TScopedLockType, TUntrackedDeleteOperatorType>		TBasicTypeChecker;
		typedef PvdConnectionNullTypeChecker<TMutexType, TScopedLockType, TUntrackedDeleteOperatorType>									TNullTypeChecker;
		typedef physx::shdfnd::Array<PxU8,TAllocatorType >																				TPXU8Container;
		typedef physx::shdfnd::Array<PxU32,TAllocatorType >																				TPXU32Container;
		typedef physx::shdfnd::Array<PvdCommLayerDatatype,TAllocatorType >																TDatatypeContainer;
		typedef physx::shdfnd::Array<PvdCommLayerData,TAllocatorType >																	TDataContainer;
		typedef PvdConnectionStreamOwner<TMutexType, TScopedLockType, TDeleteOperatorType>												TStreamOwnerType;
		typedef PvdDataStreamWriter<TPXU8Container, TAllocatorType, 0x4000, TDeleteOperatorType,  TStreamOwnerType>						TStreamWriter;
		typedef PvdDataStreamImpl<TBasicTypeChecker, TStreamWriter, TAllocatorType, TDeleteOperatorType>								TConnectionImpl;
		typedef PvdDataStreamImpl<TNullTypeChecker, TStreamWriter, TAllocatorType, TDeleteOperatorType>									TNoCheckConnectionImpl;
		
		typedef physx::shdfnd::Array<PxU8,TUntrackedAllocatorType >																		TUntrackedPXU8Container;
		typedef physx::shdfnd::Array<PxU32,TUntrackedAllocatorType >																	TUntrackedPXU32Container;
		typedef physx::shdfnd::Array<PvdCommLayerDatatype,TUntrackedAllocatorType >														TUntrackedDatatypeContainer;
		typedef physx::shdfnd::Array<PvdCommLayerData,TUntrackedAllocatorType >															TUntrackedDataContainer;
		typedef PvdConnectionStreamOwner<TMutexType, TScopedLockType, TUntrackedDeleteOperatorType>										TUntrackedStreamOwnerType;
		typedef PvdDataStreamWriter<TPXU8Container, TUntrackedAllocatorType, 0x4000, TUntrackedDeleteOperatorType,  TStreamOwnerType>	TUntrackedStreamWriter;
		typedef PvdDataStreamImpl<TBasicTypeChecker, TStreamWriter, TUntrackedAllocatorType, TUntrackedDeleteOperatorType>				TUntrackedConnectionImpl;
		typedef PvdDataStreamImpl<TNullTypeChecker, TStreamWriter, TUntrackedAllocatorType, TUntrackedDeleteOperatorType>				TUntrackedNoCheckConnectionImpl;

		typedef PvdContainerCommOutStream<TPXU8Container,TDeleteOperatorType>															TOutStream;
		typedef PvdConnectionImpl<TMutexType, TScopedLockType, TConnectionImpl, TAllocatorType, TDeleteOperatorType, TUntrackedConnectionImpl, TUntrackedDeleteOperatorType>					TFactoryType;
		typedef PvdConnectionImpl<TMutexType, TScopedLockType, TNoCheckConnectionImpl,TAllocatorType, TDeleteOperatorType, TUntrackedNoCheckConnectionImpl, TUntrackedDeleteOperatorType>				TNoCheckFactoryType;

		
		static inline char* AllocateMemory( size_t inSize, const char* inFile = NULL, int inAmount = 0)
		{
			return reinterpret_cast< char* >( TAllocator().allocate( inSize, inFile, inAmount ) );
		}

		static inline void DeallocateMemory( char* inMem )
		{
			return TAllocator().deallocate( inMem );
		}

		static inline void UntrackedDeallocateMemory( char* inMem )
		{
			return TUntrackedAllocator().deallocate( inMem );
		}

		template<typename TDataType>
		static inline void DeallocateMemory( TDataType* inMem ) { DeallocateMemory( reinterpret_cast< char* >( inMem ) ); }

		template<typename TObjType>
		static inline TObjType* AllocateObject()
		{
			TObjType* theObj(reinterpret_cast<TObjType*>(AllocateMemory(sizeof(TObjType))));
			new (theObj)TObjType();
			return theObj;
		}

		template<typename TObjType, typename TArgType >
		static inline TObjType* AllocateObject(TArgType inArg1 )
		{
			TObjType* theObj(reinterpret_cast<TObjType*>(AllocateMemory(sizeof(TObjType))));
			new (theObj)TObjType(inArg1);
			return theObj;
		}
		
		template<typename TObjType, typename TArgType, typename TArg2Type >
		static inline TObjType* AllocateObject(TArgType inArg1, TArg2Type inArg2 )
		{
			TObjType* theObj(reinterpret_cast<TObjType*>(AllocateMemory(sizeof(TObjType))));
			new (theObj)TObjType(inArg1,inArg2);
			return theObj;
		}
		template<typename TObjType, typename TArgType, typename TArg2Type, typename TArg3Type >
		static inline TObjType* AllocateObject(TArgType inArg1, TArg2Type inArg2, TArg3Type inArg3 )
		{
			TObjType* theObj(reinterpret_cast<TObjType*>(AllocateMemory(sizeof(TObjType))));
			new (theObj)TObjType(inArg1,inArg2,inArg3);
			return theObj;
		}
		
		template<typename TObjType, typename TArgType, typename TArg2Type, typename TArg3Type, typename TArg4Type >
		static inline TObjType* AllocateObject(TArgType inArg1, TArg2Type inArg2, TArg3Type inArg3, TArg4Type inArg4 )
		{
			TObjType* theObj(reinterpret_cast<TObjType*>(AllocateMemory(sizeof(TObjType))));
			new (theObj)TObjType(inArg1,inArg2,inArg3,inArg4);
			return theObj;
		}

		template<typename TObjType>
		static inline void DeallocateObject(TObjType* inObject)
		{
			inObject->~TObjType();
			DeallocateMemory( inObject );
		}

		template<typename TFactoryType>
		static TFactoryType* initializeFactory( TFactoryType* theFactory, PvdConnectionHandler* inHandler, PvdConnectionDataProvider* inDataProvider )
		{
			theFactory->sendStreamInitialization();
			if ( inDataProvider )
				theFactory->setDataProvider( inDataProvider );
			if ( inHandler )
			{
				theFactory->setHandler( inHandler );
				inHandler->onPvdConnected( theFactory );
			}
			return theFactory;
		}

		static PvdConnection* createDebuggerConnectionFactory( PvdCommInStream* inInStream
																		, PvdCommOutStream* inOutStream
																		, bool inCheckAPI
																		, TConnectionFlagsType inConnectionType
																		, PvdConnectionHandler* inHandler = NULL
																		, PvdConnectionDataProvider* inDataProvider = NULL
																		, bool inUseDoubleBufferedOutput = true )
		{
			PvdConnection* retval = NULL;
			if ( inCheckAPI )
			{
				TFactoryType* theFactory = AllocateObject<TFactoryType>( inOutStream, inInStream, inConnectionType, inUseDoubleBufferedOutput );
				retval = initializeFactory( theFactory, inHandler, inDataProvider );
			}
			else
			{
				TNoCheckFactoryType* theFactory = AllocateObject<TNoCheckFactoryType>( inOutStream, inInStream, inConnectionType, inUseDoubleBufferedOutput);
				retval = initializeFactory( theFactory, inHandler, inDataProvider );
			}
			return retval;
		}
	};
}

#endif