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


#ifndef PX_PHYSX_PROFILE_EVENT_CONCRETE_EVENT_FILTER_H
#define PX_PHYSX_PROFILE_EVENT_CONCRETE_EVENT_FILTER_H
#include "PxProfileBase.h"
#include "PsFoundation.h"
#include "PxProfileEventId.h"
#include "PxProfileFoundationWrapper.h"

namespace physx { namespace profile {

	//Implements event filtering for a particular subsystem.  Unset event values
	//default to a given value (that had better be true).
	class ConcreteEventFilterEntry
	{
		PxU32*	mData;
		PxU32	mSize;
		FoundationWrapper* mWrapper;
		
		//unimplemented copy constructor
		ConcreteEventFilterEntry( const ConcreteEventFilterEntry& inOther );
	public:
		ConcreteEventFilterEntry( FoundationWrapper* inWrapper )
			: mData( NULL )
			, mSize( 0 )
			, mWrapper( inWrapper )
		{
		}
		~ConcreteEventFilterEntry()
		{
			PxAllocatorCallback& allocator( mWrapper->getAllocator() );
			if ( mSize && mData )
				allocator.deallocate( mData );
			mSize = 0;
			mData = NULL;
		}
		void setEventEnabled(PxU16 eventId, bool enabled, bool inDefaultValue )
		{
			PxU16 entryIndex = eventId / 32;
			PxU8 highBit = PxU8(eventId % 32);
			if ( entryIndex >= mSize )
			{
				PxAllocatorCallback& allocator( mWrapper->getAllocator() );
				const char* handle = "physx::profile::ConcreteEventFilterEntry";
				PxU32 amountToAllocate = PxMin( (entryIndex + 1U) * 2, ( PX_MAX_U16 / 32 ) + 1 );
				PxU32* newData = reinterpret_cast<PxU32*>(allocator.allocate( amountToAllocate * sizeof( PxU32 ), handle, __FILE__, __LINE__ ));
				PxI32 memsetArg = inDefaultValue ? -1 : 0;
				memset( newData, memsetArg, amountToAllocate * sizeof( PxU32 ) );
				if ( mData && mSize )
				{
					memcpy( newData, mData, mSize * sizeof( PxU32 ) );
					allocator.deallocate( mData );
				}
				mData = newData;
				mSize = amountToAllocate;
			}
			PxU32& filterValue = mData[entryIndex];
			PxU32 highValue = 1 << highBit;
			if ( enabled )
				filterValue = filterValue | highValue;
			else
			{
				highValue = ~highValue;
				filterValue = filterValue & highValue;
			}
		}

		bool isEnabled( PxU16 eventId, bool inDefaultValue ) const
		{
			PxU16 entryIndex = eventId / 32;
			PxU8 highBit = PxU8(eventId % 32);
			if ( entryIndex < mSize )
			{
				PxU32 highValue = 1 << highBit;
				PxU32& filterValue = mData[entryIndex];
				PxU32 filterResult = highValue & filterValue;
				return filterResult != 0;
			}
			else
				return inDefaultValue;
		}
		ConcreteEventFilterEntry& operator=( const ConcreteEventFilterEntry& inOther )
		{
			if ( mSize < inOther.mSize )
			{
				mSize = inOther.mSize;
				PxAllocatorCallback& allocator( mWrapper->getAllocator() );
				const char* handle = "physx::profile::ConcreteEventFilterEntry";
				if ( mData )
					allocator.deallocate( mData );
				mData = reinterpret_cast<PxU32*>( allocator.allocate( mSize * sizeof( PxU32 ), handle, __FILE__, __LINE__ ) );
				memcpy( mData, inOther.mData, mSize * sizeof( PxU32 ) );
			}
			else if ( mSize ) //Non of our operations shrink this object, so this case shouldn't happen
			{
				memcpy( mData, inOther.mData, inOther.mSize );
			}
			return *this;
		}
	};

	/**
	 *	Concrete implementation of the event filter interface.  Provides the ability to efficiently
	 *	filter events based on subsystem and event id.
	 *	This object is meant to be chained; i.e. on filter can act as the master and there can be
	 *	various sub-filters that pull from the master whenever it is safe for them.  For instance:
	 *	The physics SDK can have the master, and each scene can have a slave event filter.  Setting
	 *	the filter settings on the SDK object will eventually affect all the scenes *however*
	 *	the scene will only update its filter at frame boundaries.  Thus the scene filter can be used
	 *	by all of the tasks and such without requiring any mutex grab.
	 */
	class ConcreteEventFilter
	{
		PxU32						mRevision;
		bool						mDefaultEnabledState;
		FoundationWrapper			mWrapper;
		ConcreteEventFilterEntry	mBitArray;
	public:
		ConcreteEventFilter( PxAllocatorCallback* inAllocator = NULL, bool inDefaultEnabledState = true )
			: mRevision( 0 )
			, mDefaultEnabledState( inDefaultEnabledState )
			, mWrapper( inAllocator )
			, mBitArray( &mWrapper )
		{
		}
		bool isEventEnabled(PxU16 eventId) const
		{
			const ConcreteEventFilterEntry& entry( mBitArray );
			return entry.isEnabled( eventId, mDefaultEnabledState );
		}

		bool isEventEnabled(const PxProfileEventId& inId) const
		{
			return isEventEnabled( inId.mEventId );
		}

		void setEventEnabled( PxU16 eventId, bool enabled )
		{
			bool currentValue = isEventEnabled( eventId );
			if ( currentValue != enabled )
			{
				++mRevision;
				ConcreteEventFilterEntry& entry( mBitArray );
				entry.setEventEnabled( eventId, enabled, mDefaultEnabledState );
			}
		}
		
		void setEventEnabled( const PxProfileEventId& inId, bool enabled )
		{
			setEventEnabled( inId.mEventId, enabled );
		}

		void updateFrom( const ConcreteEventFilter& inFilter )
		{
			if ( mRevision != inFilter.mRevision )
			{
				mDefaultEnabledState = inFilter.mDefaultEnabledState;
				mRevision = inFilter.mRevision;
				mBitArray = inFilter.mBitArray;
			}
		}
	};
}}

#endif