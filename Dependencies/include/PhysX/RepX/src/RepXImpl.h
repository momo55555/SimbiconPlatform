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
#ifndef PX_REPXIMPL_H
#define PX_REPXIMPL_H
#include "MemoryPool.h"
#include "PsString.h"

namespace physx { namespace repx {

typedef CMemoryPoolManager TMemoryPoolManager;

	struct MemoryPoolAllocator
	{
		TMemoryPoolManager* mManager;
		MemoryPoolAllocator( TMemoryPoolManager* inManager )
			: mManager( inManager )
		{
		}
		void* allocate( size_t inSize, const char*, unsigned int )
		{
			inSize += sizeof( size_t );
			size_t* theData = reinterpret_cast< size_t* >( mManager->allocate( static_cast<PxU32>(inSize) ) );
			theData[0] = inSize;
			return theData + 1;
		}
		void deallocate( void* inPtr )
		{
			if ( inPtr )
			{
				size_t* theData = reinterpret_cast< size_t* >( inPtr ) - 1;
				mManager->deallocate( reinterpret_cast< PxU8* >( theData ) );
			}
		}
	};

	inline PxU32 strLen( const char* inStr )
	{
		PxU32 len = 0;
		if ( inStr )
		{
			while ( *inStr )
			{
				++len;
				++inStr;
			}
		}
		return len;
	}

	inline const char* copyStr( PxAllocatorCallback& inAllocator, const char* inStr )
	{
		if ( inStr && *inStr )
		{
			PxU32 theLen = strLen( inStr );
			//The memory will never be released by repx.  If you want it released, you need to pass in a custom allocator
			//that tracks all allocations and releases unreleased allocations yourself.
			char* dest = reinterpret_cast<char* >( inAllocator.allocate( theLen + 1, "Repx::const char*", __FILE__, __LINE__ ) );
			memcpy( dest, inStr, theLen );
			dest[theLen] = 0;
			return dest;
		}
		return "";
	}

	template<typename TManagerType>
	inline const char* copyStr( TManagerType* inMgr, const char* inStr )
	{
		if ( inStr && *inStr )
		{
			PxU32 theLen = strLen( inStr );
			char* dest = reinterpret_cast<char* >( inMgr->allocate( theLen + 1 ) );
			memcpy( dest, inStr, theLen );
			dest[theLen] = 0;
			return dest;
		}
		return "";
	}

	inline void releaseStr( TMemoryPoolManager* inMgr, const char* inStr, PxU32 )
	{
		if ( inStr && *inStr )
		{
			inMgr->deallocate( reinterpret_cast< PxU8* >( const_cast<char*>( inStr ) ) ); 
		}
	}

	inline void releaseStr( TMemoryPoolManager* inMgr, const char* inStr )
	{
		if ( inStr && *inStr )
		{
			PxU32 theLen = strLen( inStr );
			releaseStr( inMgr, inStr, theLen );
		}
	}

	struct EditableString
	{
	private:
		EditableString( const EditableString& inOther );
		EditableString& operator=( const EditableString& inOther );

	public:
		char* mData;
		PxU32 mLen;
		TMemoryPoolManager* mManager;

		inline operator char* () { return mData; }
		EditableString( const char* inSrc, TMemoryPoolManager* inManager )
		{
			mLen = strLen( inSrc );
			if ( inSrc && *inSrc )
				mData = const_cast<char*>( copyStr( inManager, inSrc ) );
			else
				mData = NULL;
			mManager = inManager;
		}
		~EditableString() { mManager->deallocate( reinterpret_cast<PxU8*>( mData ) ); }
	};

	struct RepXNode
	{
		const char* mName; //Never released until all collections are released
		const char* mData; //Never released until all collections are released
		
		RepXNode* mNextSibling;
		RepXNode* mPreviousSibling;
		RepXNode* mFirstChild;
		RepXNode* mParent;
		RepXNode( const RepXNode& );
		RepXNode& operator=( const RepXNode& );

		PX_INLINE void initPtrs()
		{
			mNextSibling = NULL;
			mPreviousSibling = NULL;
			mFirstChild = NULL;
			mParent = NULL;
		}

		PX_INLINE RepXNode( const char* inName = "", const char* inData = "" ) 
			: mName( inName )
			, mData( inData ) 
		{ initPtrs(); }

		void addChild( RepXNode* inItem )
		{
			inItem->mParent = this;
			if ( mFirstChild == NULL )
				mFirstChild = inItem;
			else
			{
				RepXNode* theNode = mFirstChild;
				//Follow the chain till the end.
				while( theNode->mNextSibling != NULL ) theNode = theNode->mNextSibling;
				theNode->mNextSibling = inItem;
				inItem->mPreviousSibling = theNode;
			}
		}

		PX_INLINE RepXNode* findChildByName( const char* inName )
		{
			for ( RepXNode* theNode = mFirstChild; theNode; theNode = theNode->mNextSibling )
			{
				RepXNode* theRepXNode = theNode;
				if ( physx::string::stricmp( theRepXNode->mName, inName ) == 0 )
					return theNode;
			}
			return NULL;
		}
		
		PX_INLINE void orphan()
		{
			if ( mParent )
			{
				if ( mParent->mFirstChild == this )
					mParent->mFirstChild = mNextSibling;
			}
			if ( mPreviousSibling )
				mPreviousSibling->mNextSibling = mNextSibling;
			if ( mNextSibling )
				mNextSibling->mPreviousSibling = mPreviousSibling;
			if ( mFirstChild )
				mFirstChild->mParent = NULL;
			initPtrs();
		}
	};

	inline RepXNode* allocateRepXNode( TMemoryPoolManager* inManager, const char* inName, const char* inData )
	{
		RepXNode* retval = inManager->allocate<RepXNode>();
		retval->mName = copyStr( inManager, inName );
		retval->mData = copyStr( inManager, inData );
		return retval;
	}

	inline void release( TMemoryPoolManager* inManager, RepXNode* inNode )
	{
		//We *don't* release the strings associated with the node
		//because they could be shared.  Instead, we just let them 'leak'
		//in some sense, at least until the memory manager itself is deleted.
		//DO NOT UNCOMMENT THE LINES BELOW!!
		//releaseStr( inManager, inNode->mName );
		//releaseStr( inManager, inNode->mData );
		inManager->deallocate( inNode );
	}

	static void releaseNodeAndChildren( TMemoryPoolManager* inManager, RepXNode* inNode )
	{
		if ( inNode->mFirstChild )
		{
			RepXNode* childNode( inNode->mFirstChild );
			while( childNode )
			{
				RepXNode* _node( childNode );
				childNode = _node->mNextSibling;
				releaseNodeAndChildren( inManager, _node );
			}
		}
		inNode->orphan();
		release( inManager, inNode );
	}

	static RepXNode* copyRepXNodeAndSiblings( TMemoryPoolManager* inManager, const RepXNode* inNode, RepXNode* inParent );

	static RepXNode* copyRepXNode( TMemoryPoolManager* inManager, const RepXNode* inNode, RepXNode* inParent = NULL )
	{
		RepXNode* newNode( allocateRepXNode( inManager, NULL, NULL ) );
		newNode->mName = inNode->mName; //Some light structural sharing
		newNode->mData = inNode->mData; //Some light structural sharing
		newNode->mParent = inParent;
		if ( inNode->mFirstChild )
			newNode->mFirstChild = copyRepXNodeAndSiblings( inManager, inNode->mFirstChild, newNode );
		return newNode;
	}
	
	static RepXNode* copyRepXNodeAndSiblings( TMemoryPoolManager* inManager, const RepXNode* inNode, RepXNode* inParent )
	{
		RepXNode* sibling = inNode->mNextSibling;
		if ( sibling ) sibling = copyRepXNodeAndSiblings( inManager, sibling, inParent );
		RepXNode* newNode = copyRepXNode( inManager, inNode, inParent );
		newNode->mNextSibling = sibling;
		if ( sibling ) sibling->mPreviousSibling = newNode;
		return newNode;
	}

	inline bool isBigEndian() { int i = 1; return *((char*)&i)==0; }
	

	struct NameStackEntry
	{
		const char* mName;
		bool		mOpen;
		NameStackEntry( const char* nm ) : mName( nm ), mOpen( false ) {}
	};

	typedef ProfileArray<NameStackEntry> TNameStack;
} }

#endif