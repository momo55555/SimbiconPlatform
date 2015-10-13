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
#include "RepX.h"
#include "RepXImpl.h"
#include "PsHash.h"
#include "PsHashMap.h"
#include "SimpleXmlWriter.h"
#include "PxProfileBase.h"
#include "../src/PxProfileFoundationWrapper.h"
#include "PxBroadcastingAllocator.h"
#include "RepXVisitorWriter.h"
#include "RepXVisitorReader.h"

namespace physx
{
	namespace pubfnd3{}
	namespace shdfnd3{}
	namespace apex{}
	using namespace pubfnd3;
	using namespace shdfnd3;
	using namespace apex;
};

#include "FastXml.h"
#include "MemoryPool.h"
#include "PxStream.h"
#include "PxFileBuffer.h"
#include "PsFile.h"
#include "PsString.h"
#include "PxFilebufToPxStream.h"
#include "RepXCoreExtensions.h"
#include "RepXMemoryAllocator.h"
#include "RepXStringToType.h"
#include "PxProfileBase.h"
#include "PsString.h"

using namespace physx;
using namespace FAST_XML;

using namespace physx::profile; //for the foundation wrapper systme.

namespace physx { namespace repx {

	struct VoidPtrHashFn
	{
		PxU32 operator()( const void* inData )
		{
			PxU64 ptrVal( PX_PROFILE_POINTER_TO_U64( inData ) );
			return Hash<PxU64>()( ptrVal );
		}
		bool operator()(const void* k0, const void* k1) const { return k0 == k1; }
	};
	
	typedef ProfileHashMap<const void*, RepXObject, VoidPtrHashFn> TVoidLiveObjectHashMap;
	typedef ProfileHashMap<const void*, const void*, VoidPtrHashFn> TVoidVoidHashMap;

	struct RepXIdToLiveObjectMapImpl : public RepXIdToRepXObjectMap
	{
		FoundationWrapper			mWrapper;
		TVoidLiveObjectHashMap		mIdToLiveObject;
		TVoidVoidHashMap			mLiveObjectToId;

		RepXIdToLiveObjectMapImpl( PxAllocatorCallback& inCallback )
			: mWrapper ( inCallback )
			, mIdToLiveObject( mWrapper )
			, mLiveObjectToId( mWrapper )
		{
		}
		
		virtual void destroy() { PX_PROFILE_DELETE( mWrapper, this ); }
		virtual void addLiveObject( RepXObject inLiveObject )
		{
			if ( inLiveObject.mLiveObject && inLiveObject.mId )
			{
				bool inserted = mIdToLiveObject.insert( inLiveObject.mId, inLiveObject );
				PX_ASSERT( inserted );
				inserted = mLiveObjectToId.insert( inLiveObject.mLiveObject, inLiveObject.mId );
				PX_ASSERT( inserted );
			}
		}
		virtual RepXObject getLiveObjectFromId( const void* inId ) 
		{
			const TVoidLiveObjectHashMap::Entry* theEntry = mIdToLiveObject.find( inId );
			if ( theEntry )
				return theEntry->second;
			return RepXObject();
		}
		virtual const void* getIdForLiveObject( const void* inLiveObject )
		{
			const TVoidVoidHashMap::Entry* theEntry = mLiveObjectToId.find( inLiveObject );
			if ( theEntry )
				return theEntry->second;
			return NULL;
		}
	};

	RepXIdToRepXObjectMap* RepXIdToRepXObjectMap::create(PxAllocatorCallback& inAllocator) { return PX_PROFILE_NEW( inAllocator, RepXIdToLiveObjectMapImpl )(inAllocator); }

	class RepXNodeXmlWriter : public XmlWriter
	{
		RepXMemoryAllocatorImpl&	mParseAllocator;
		RepXNode*					mCurrentNode;
		RepXNode*					mTopNode;
		PxU32						mTabCount;

	public:
		RepXNodeXmlWriter( RepXMemoryAllocatorImpl& inAllocator, PxU32 inTabCount = 0 ) 
			: mParseAllocator( inAllocator ) 
			, mCurrentNode( NULL )
			, mTopNode( NULL )
			, mTabCount( inTabCount )
		{}
		virtual ~RepXNodeXmlWriter(){}
		void onNewNode( RepXNode* newNode )
		{
			if ( mCurrentNode != NULL )
				mCurrentNode->addChild( newNode );
			if ( mTopNode == NULL )
				mTopNode = newNode;
			mCurrentNode = newNode;
			++mTabCount;
		}

		RepXNode* getTopNode() { return mTopNode; }

		virtual void beginTag( const char* inTagname )
		{
			onNewNode( allocateRepXNode( &mParseAllocator.mManager, inTagname, NULL ) );
		}
		virtual void endTag()
		{
			if ( mCurrentNode )
				mCurrentNode = mCurrentNode->mParent;
			if ( mTabCount )
				--mTabCount;
		}
		virtual void addAttribute( const char*, const char* )
		{
			PX_ASSERT( false );
		}
		virtual void writeContentTag( const char* inTag, const char* inContent )
		{
			onNewNode( allocateRepXNode( &mParseAllocator.mManager, inTag, inContent ) );
			endTag();
		}
		virtual void addContent( const char* inContent )
		{
			if ( mCurrentNode->mData )
				releaseStr( &mParseAllocator.mManager, mCurrentNode->mData );
			mCurrentNode->mData = copyStr( &mParseAllocator.mManager, inContent );
		}
		virtual PxU32 tabCount() { return mTabCount; }
	};

	struct RepXWriterImpl : public RepXWriter
	{
		PxU32			mTagDepth;
		XmlWriter*		mWriter;
		MemoryBuffer*	mMemBuffer;

		RepXWriterImpl( XmlWriter* inWriter, MemoryBuffer* inMemBuffer )
			: mTagDepth( 0 )
			, mWriter( inWriter )
			, mMemBuffer( inMemBuffer )
		{
		}
		~RepXWriterImpl()
		{
			while( mTagDepth )
			{
				--mTagDepth;
				mWriter->endTag();
			}
		}
		virtual void write( const char* inName, const char* inData )
		{
			mWriter->writeContentTag( inName, inData );
		}
		virtual void write( const char* inName, const RepXObject& inLiveObject )
		{
			PxU64 theValue = PX_PROFILE_POINTER_TO_U64( inLiveObject.mId );
			(*mMemBuffer) << theValue;
			writeProperty( *mWriter, *mMemBuffer, inName );
		}
		virtual void addAndGotoChild( const char* inName )
		{
			mWriter->beginTag( inName );
			mTagDepth++;
		}
		virtual void leaveChild()
		{
			if ( mTagDepth )
			{
				mWriter->endTag();
				--mTagDepth;
			}
		}
	};

	struct RepXParseArgs
	{
		RepXMemoryAllocatorImpl*				mAllocator;
		ProfileArray<RepXCollectionItem>*		mCollection;
		ProfileArray<RepXExtension*>*			mExtensions;	

		RepXParseArgs( RepXMemoryAllocatorImpl* inAllocator
						, ProfileArray<RepXCollectionItem>* inCollection
						, ProfileArray<RepXExtension*>* inExtensions )
						: mAllocator( inAllocator )
						, mCollection( inCollection )
						, mExtensions( inExtensions )
		{
		}
	};
	

	PX_INLINE RepXExtension* getExtension( const char* inName, const ProfileArray<RepXExtension*>& inExtensions )
	{
		for( PxU32 idx = 0; idx < inExtensions.size(); ++idx )
		{
			if ( physx::string::stricmp( inExtensions[idx]->getTypeName(), inName ) == 0 )
				return inExtensions[idx];
		}
		return NULL;
	}

	struct RepXNodeReader : public RepXReaderWriter
	{
		FoundationWrapper mWrapper;
		CMemoryPoolManager& mManager;
		RepXNode* mCurrentNode;
		RepXNode* mTopNode;
		ProfileArray<RepXNode*> mContext;
		RepXNodeReader( RepXNode* inCurrentNode, PxAllocatorCallback& inAllocator, CMemoryPoolManager& nodePoolManager )
			: mWrapper( inAllocator )
			, mCurrentNode( inCurrentNode )
			, mTopNode( inCurrentNode )
			, mContext( mWrapper )
			, mManager( nodePoolManager )
		{
		}
		
		//Does this node exist as data in the format.
		virtual bool read( const char* inName, const char*& outData )
		{
			RepXNode* theChild( mCurrentNode->findChildByName( inName ) );
			if ( theChild )
			{
				outData = theChild->mData;
				return outData && *outData;
			}
			return false;
		}

		virtual bool read( const char* inName, void*& outId )
		{
			RepXNode* theChild( mCurrentNode->findChildByName( inName ) );
			if ( theChild )
			{
				PxU64 theData = 0;
				char* theValue( const_cast<char*>( theChild->mData ) );
				strto( theData, theValue );
				outId = reinterpret_cast<void*>(static_cast<PxU64>(theData));
				return true;
			}
			return false;
		}

		virtual bool gotoChild( const char* inName )
		{
			RepXNode* theChild( mCurrentNode->findChildByName( inName ) );
			if ( theChild )
			{
				mCurrentNode =theChild;
				return true;
			}
			return false;
		}
		virtual bool gotoFirstChild()
		{
			if ( mCurrentNode->mFirstChild )
			{
				mCurrentNode = mCurrentNode->mFirstChild;
				return true;
			}
			return false;
		}
		virtual bool gotoNextSibling()
		{
			if ( mCurrentNode->mNextSibling )
			{
				mCurrentNode = mCurrentNode->mNextSibling;
				return true;
			}
			return false;
		}
		virtual PxU32 countChildren()
		{
			PxU32 retval=  0;
			for ( RepXNode* theChild = mCurrentNode->mFirstChild; theChild != NULL; theChild = theChild->mNextSibling )
				++retval;
			return retval;
		}
		virtual const char* getCurrentItemName()
		{
			return mCurrentNode->mName;
		}
		virtual const char* getCurrentItemValue()
		{
			return mCurrentNode->mData;
		}

		virtual bool leaveChild()
		{
			if ( mCurrentNode != mTopNode && mCurrentNode->mParent )
			{
				mCurrentNode = mCurrentNode->mParent;
				return true;
			}
			return false;
		}
		
		virtual void pushCurrentContext()
		{
			mContext.pushBack( mCurrentNode );
		}
		virtual void popCurrentContext()
		{
			if ( mContext.size() )
			{
				mCurrentNode = mContext.back();
				mContext.popBack();
			}
		}

		virtual void setNode( RepXNode& inNode )
		{
			mContext.clear();
			mCurrentNode = &inNode;
			mTopNode = mCurrentNode;
		}
		
		virtual void addOrGotoChild( const char* inName )
		{
			if ( gotoChild( inName )== false )
			{
				RepXNode* newNode = allocateRepXNode( &mManager, inName, NULL );
				mCurrentNode->addChild( newNode );
				mCurrentNode = newNode;
			}
		}
		virtual void setCurrentItemValue( const char* inValue )
		{
			mCurrentNode->mData = copyStr( &mManager, inValue ); 
		}
		virtual bool removeChild( const char* name )
		{
			RepXNode* theChild( mCurrentNode->findChildByName( name ) );
			if ( theChild )
			{
				releaseNodeAndChildren( &mManager, theChild );
				return true;
			}
			return false;
		}
		virtual void release() { PX_PROFILE_DELETE( mWrapper.getAllocator(), this ); }
	};

	PX_INLINE void  freeNodeAndChildren( RepXNode* tempNode, TMemoryPoolManager& inManager )
	{
		for( RepXNode* theNode = tempNode->mFirstChild; theNode != NULL; theNode = theNode->mNextSibling )
			freeNodeAndChildren( theNode, inManager );
		tempNode->orphan();
		release( &inManager, tempNode );
	}


	class RepXParser : public FastXml::Callback
	{
		RepXParseArgs			mParseArgs;
		//For parse time only allocations
		RepXMemoryAllocatorImpl& mParseAllocator;
		RepXNode* mCurrentNode;
		RepXNode* mTopNode;

	public:
		RepXParser( RepXParseArgs inArgs, RepXMemoryAllocatorImpl& inParseAllocator )
			: mCurrentNode( NULL )
			, mTopNode( NULL )
			, mParseArgs( inArgs )
			, mParseAllocator( inParseAllocator )
		{
		}

		virtual ~RepXParser(){}

		virtual bool processComment(const char *comment) { return true; }
		// 'element' is the name of the element that is being closed.
		// depth is the recursion depth of this element.
		// Return true to continue processing the XML file.
		// Return false to stop processing the XML file; leaves the read pointer of the stream right after this close tag.
		// The bool 'isError' indicates whether processing was stopped due to an error, or intentionally canceled early.
		virtual bool processClose(const char *element,physx::PxU32 depth,bool &isError)
		{
			mCurrentNode = mCurrentNode->mParent;
			return true;
		}

		// return true to continue processing the XML document, false to skip.
		virtual bool processElement(
			const char *elementName,   // name of the element
			physx::PxI32 argc,         // number of attributes pairs
			const char **argv,         // list of attributes.
			const char  *elementData,  // element data, null if none
			physx::PxI32 lineno)
		{
			RepXNode* newNode = allocateRepXNode( &mParseAllocator.mManager, elementName, elementData );
			if ( mCurrentNode )
				mCurrentNode->addChild( newNode );
			mCurrentNode = newNode;
			//Add the elements as children.
			for( PxI32 item = 0; item < argc; item += 2, argv += 2 )
			{
				RepXNode* newNode = allocateRepXNode( &mParseAllocator.mManager, argv[0], argv[1] );
				mCurrentNode->addChild( newNode );
			}
			if ( mTopNode == NULL ) mTopNode = newNode;
			return true;
		}

		RepXNode* getTopNode() { return mTopNode; }

		virtual void *  fastxml_malloc(physx::PxU32 size) { if ( size ) return mParseAllocator.allocate(size); return NULL; }
		virtual void	fastxml_free(void *mem) { if ( mem ) mParseAllocator.deallocate(reinterpret_cast<PxU8*>(mem)); }
	};

	template<typename TCoreType>
	struct RepXReferenceCountedItem
	{
		FoundationWrapper& mWrapper;
		PxU32 mRefCount;
		TCoreType mType;
		RepXReferenceCountedItem( FoundationWrapper& inWrapper, const TCoreType& inSrc )
			: mWrapper( inWrapper )
			, mRefCount( 0 )
			, mType( inSrc )
		{
		}
		void addRef() { ++mRefCount; }
		void release() 
		{ 
			if ( mRefCount ) --mRefCount;
			if ( !mRefCount ) 
			{
				PX_PROFILE_DELETE( mWrapper, this );
			}
		}
		TCoreType* operator->() { return mType; }
	};

	struct RepXCollectionSharedData
	{
		FoundationWrapper				mWrapper;
		ProfileArray<RepXExtension*>	mExtensions;
		RepXMemoryAllocatorImpl			mAllocator;
		PxU32							mRefCount;

		RepXCollectionSharedData( PxAllocatorCallback& inAllocator )
			: mWrapper( inAllocator )
			, mExtensions( mWrapper )
			, mAllocator( inAllocator )
			, mRefCount( 0 )
		{
		}
		~RepXCollectionSharedData()
		{
			for ( PxU32 idx = 0; idx < mExtensions.size(); ++idx ) mExtensions[idx]->destroy();
			mExtensions.clear();
		}
		void addRef() { ++mRefCount;}
		void release()
		{
			if ( mRefCount ) --mRefCount;
			if ( !mRefCount ) PX_PROFILE_DELETE( mWrapper.getAllocator(), this );
		}
	};

	struct SharedDataPtr
	{
		RepXCollectionSharedData* mData;
		SharedDataPtr( RepXCollectionSharedData* inData )
			: mData( inData )
		{
			mData->addRef();
		}
		SharedDataPtr( const SharedDataPtr& inOther )
			: mData( inOther.mData )
		{
			mData->addRef();
		}
		SharedDataPtr& operator=( const SharedDataPtr& inOther );
		~SharedDataPtr()
		{
			mData->release();
			mData = NULL;
		}
		RepXCollectionSharedData* operator->() { return mData; }
		const RepXCollectionSharedData* operator->() const { return mData; }
	};

	class RepXCollectionImpl : public RepXCollection, public UserAllocated
	{
		SharedDataPtr							mSharedData;

		RepXMemoryAllocatorImpl&				mAllocator;
		ProfileArray<RepXExtension*>&			mExtensions;
		ProfileArray<RepXCollectionItem>		mCollection;
		TMemoryPoolManager						mSerializationManager;
		MemoryBuffer							mPropertyBuffer;
		PxTolerancesScale						mScale;
		PxVec3									mUpVector;
		const char*								mVersionStr;

	public:
		RepXCollectionImpl( RepXExtension** inExtensions, PxU32 inNumExtensions, PxTolerancesScale inScale, PxAllocatorCallback& inAllocator )
			: mSharedData( PX_PROFILE_NEW( inAllocator, RepXCollectionSharedData )( inAllocator ) )
			, mAllocator( mSharedData->mAllocator )
			, mExtensions( mSharedData->mExtensions )
			, mCollection( mSharedData->mWrapper )
			, mSerializationManager( inAllocator )
			, mPropertyBuffer( &mSerializationManager )
			, mScale( inScale )
			, mUpVector( 0, 0, 0 )
			, mVersionStr( getLatestVersion() )
		{
			if ( inNumExtensions )
			{
				mExtensions.reserve( inNumExtensions );
				for ( PxU32 idx = 0; idx < inNumExtensions; ++idx ) mExtensions.pushBack( inExtensions[idx] );
			}
		}

		RepXCollectionImpl( const RepXCollectionImpl& inSrc, const char* inNewVersion )
			: mSharedData( inSrc.mSharedData )
			, mAllocator( mSharedData->mAllocator )
			, mExtensions( mSharedData->mExtensions )
			, mCollection( mSharedData->mWrapper )
			, mSerializationManager( mSharedData->mWrapper.getAllocator() )
			, mPropertyBuffer( &mSerializationManager )
			, mScale( inSrc.mScale )
			, mUpVector( inSrc.mUpVector )
			, mVersionStr( inNewVersion )
		{
		}

		virtual ~RepXCollectionImpl()
		{
			PxU32 numItems = mCollection.size();
			for ( PxU32 idx = 0; idx < numItems; ++idx )
			{
				RepXNode* theNode = mCollection[idx].mDescriptor;
				releaseNodeAndChildren( &mAllocator.mManager, theNode );
			}
		}

		virtual void destroy() { PX_PROFILE_DELETE( mSharedData->mWrapper, this ); }

		
		virtual PxTolerancesScale getTolerancesScale() const { return mScale; }
		virtual void setUpVector( const PxVec3& inUpVector ) { mUpVector = inUpVector; }
		virtual PxVec3 getUpVector() const { return mUpVector; }

		PX_INLINE RepXExtension* getExtension( const char* inName ) const
		{
			return physx::repx::getExtension( inName, mExtensions );
		}

		PX_INLINE RepXCollectionItem findItemBySceneItem( const RepXObject& inObject ) const
		{
			//See if the object is in the collection
			for ( PxU32 idx =0; idx < mCollection.size(); ++idx )
				if ( mCollection[idx].mLiveObject.mLiveObject == inObject.mLiveObject )
					return mCollection[idx];
			return RepXCollectionItem();
		}

		virtual RepXAddToCollectionResult addRepXObjectToCollection( const RepXObject& inObject, RepXIdToRepXObjectMap* inLiveObjectIdMap )
		{
			PX_ASSERT( inObject.mLiveObject );
			PX_ASSERT( inObject.mId );
			if ( inObject.mLiveObject == NULL || inObject.mId == 0 )
				return RepXAddToCollectionResult( RepXAddToCollectionResult::InvalidParameters );
			RepXExtension* theExtension = getExtension( inObject.mTypeName);
			if ( theExtension == NULL )
				return RepXAddToCollectionResult( RepXAddToCollectionResult::ExtensionNotFound );

			RepXCollectionItem existing = findItemBySceneItem( inObject );
			if ( existing.mLiveObject.mLiveObject )
				return RepXAddToCollectionResult( RepXAddToCollectionResult::AlreadyInCollection, existing.mLiveObject.mId );
			
			inLiveObjectIdMap->addLiveObject( inObject );

			RepXNodeXmlWriter theXmlWriter( mAllocator, 1 );
			RepXWriterImpl theRepXWriter( &theXmlWriter, &mPropertyBuffer );
			{
				XmlWriter::STagWatcher theWatcher( theXmlWriter, inObject.mTypeName );
				writeProperty( theXmlWriter, mPropertyBuffer, "Id", PX_PROFILE_POINTER_TO_U64( inObject.mId ) );
				theExtension->objectToFile( inObject, inLiveObjectIdMap, theRepXWriter, mPropertyBuffer );
			}
			mCollection.pushBack( RepXCollectionItem( inObject, theXmlWriter.getTopNode() ) );
			return RepXAddToCollectionResult( RepXAddToCollectionResult::Success, inObject.mId );
		}

		virtual void instantiateCollection( RepXInstantiationArgs inArgs, RepXIdToRepXObjectMap* inLiveObjectIdMap
											, RepXInstantiationResultHandler* inResultHandler, bool inAddOriginalIdsToObjectMap )
		{
			for ( PxU32 idx =0; idx < mCollection.size(); ++idx )
			{
				RepXCollectionItem theItem( mCollection[idx] );
				RepXExtension* theExtension = getExtension( theItem.mLiveObject.mTypeName );
				if ( theExtension )
				{
					RepXNodeReader theReader( theItem.mDescriptor, mAllocator.getAllocator(), mAllocator.mManager );
					RepXMemoryAllocatorImpl instantiationAllocator( mAllocator.getAllocator() );
					RepXObject theLiveObject = theExtension->fileToObject( theReader, instantiationAllocator, inArgs, inLiveObjectIdMap );
					if ( theLiveObject.isValid() )
					{
						if ( inResultHandler )
							inResultHandler->addInstantiationResult( RepXInstantiationResult( theItem.mLiveObject.mId, const_cast<void*>( theLiveObject.mLiveObject ), theLiveObject.mTypeName ) );
						if ( inAddOriginalIdsToObjectMap )
							inLiveObjectIdMap->addLiveObject( RepXObject( theLiveObject.mTypeName, theLiveObject.mLiveObject, theItem.mLiveObject.mId ) );
						else
							inLiveObjectIdMap->addLiveObject( theLiveObject );
					}
				}
			}
		}

		void saveRepXNode( RepXNode* inNode, XmlWriter& inWriter )
		{
			RepXNode* theNode( inNode );
			if ( theNode->mData && *theNode->mData && theNode->mFirstChild == NULL )
				inWriter.writeContentTag( theNode->mName, theNode->mData );
			else
			{
				inWriter.beginTag( theNode->mName );
				if ( theNode->mData && *theNode->mData )
					inWriter.addContent( theNode->mData );
				for ( RepXNode* theChild = theNode->mFirstChild; 
						theChild != NULL;
						theChild = theChild->mNextSibling )
					saveRepXNode( theChild, inWriter );
				inWriter.endTag();
			}
		}

		virtual void save( PxStream* inFilestream )
		{
			XmlWriterImpl<PxStream> theWriter( *inFilestream, mAllocator.getAllocator() );
			theWriter.beginTag( "PhysX30Collection" );
			theWriter.addAttribute( "version", mVersionStr );
			{
				RepXWriterImpl theRepXWriter( &theWriter, &mPropertyBuffer );
				writeProperty( theWriter, mPropertyBuffer, "UpVector", mUpVector );
				theRepXWriter.addAndGotoChild( "Scale" );
				RepXIdToRepXObjectMap* theMap( NULL );
				writeAllProperties( &mScale, theRepXWriter, mPropertyBuffer, *theMap );
				theRepXWriter.leaveChild();
			}
			for ( PxU32 idx =0; idx < mCollection.size(); ++idx )
			{
				RepXCollectionItem theItem( mCollection[idx] );
				RepXNode* theNode( theItem.mDescriptor );
				saveRepXNode( theNode, theWriter );
			}
		}

		void load( PxFileBuf& inFileBuf )
		{
			RepXParser theParser( RepXParseArgs( &mAllocator, &mCollection, &mExtensions ), mAllocator );
			FastXml* theFastXml = createFastXml( &theParser );
			theFastXml->processXml( inFileBuf );
			RepXNode* theTopNode = theParser.getTopNode();
			if ( theTopNode != NULL )
			{
				{
					
					RepXMemoryAllocatorImpl instantiationAllocator( mAllocator.getAllocator() );
					RepXNodeReader theReader( theTopNode, mAllocator.getAllocator(), mAllocator.mManager );
					readProperty( theReader, "UpVector", mUpVector );
					RepXIdToRepXObjectMap* theMap( NULL );
					if ( theReader.gotoChild( "Scale" ) )
					{
						readAllProperties( RepXInstantiationArgs( NULL, NULL, NULL ), theReader, &mScale, instantiationAllocator, *theMap );
						theReader.leaveChild();
					}
					const char* verStr = NULL;
					if ( theReader.read( "version", verStr ) )
						mVersionStr = verStr;
				}
				for ( RepXNode* theChild = theTopNode->mFirstChild; 
						theChild != NULL;
						theChild = theChild->mNextSibling )
				{
					if ( physx::string::stricmp( theChild->mName, "scale" ) == 0 
						|| physx::string::stricmp( theChild->mName, "version" ) == 0 
						|| physx::string::stricmp( theChild->mName, "upvector" ) == 0 )
						continue;
					RepXNodeReader theReader( theChild, mAllocator.getAllocator(), mAllocator.mManager );
					RepXObject theObject;
					theObject.mTypeName = theChild->mName;
					theObject.mLiveObject = NULL;
					void* theId = NULL;
					theReader.read( "Id", theId );
					theObject.mId = theId;
					mCollection.pushBack( RepXCollectionItem( theObject, theChild ) );
				}
			}
		}
		
		virtual const char* getVersion() { return mVersionStr; }
		
		virtual const RepXCollectionItem* begin() const
		{
			return mCollection.begin();
		}
		virtual const RepXCollectionItem* end() const
		{
			return mCollection.end();
		}
		
		virtual RepXCollection& createCollection( const char* inVersionStr )
		{
			RepXCollectionImpl* retval = PX_PROFILE_NEW( mSharedData->mWrapper.getAllocator(), RepXCollectionImpl )( *this, inVersionStr );
			return *retval;
		}
		
		//Performs a deep copy of the repx node.
		virtual RepXNode* copyRepXNode( const RepXNode* srcNode ) 
		{
			return physx::repx::copyRepXNode( &mAllocator.mManager, srcNode );
		}

		virtual void addCollectionItem( RepXCollectionItem inItem ) 
		{
			mCollection.pushBack( inItem );
		}
		
		virtual PxAllocatorCallback& getAllocator() { return mSharedData->mAllocator.getAllocator(); }
		//Create a new repx node with this name.  Its value is unset.
		virtual RepXNode& createRepXNode( const char* name )
		{
			RepXNode* newNode = allocateRepXNode( &mSharedData->mAllocator.mManager, name, NULL );
			return *newNode;
		}

		//Release this when finished.
		virtual RepXReaderWriter& createNodeEditor()
		{
			RepXReaderWriter* retval = PX_PROFILE_NEW( mSharedData->mWrapper.getAllocator(), RepXNodeReader )( NULL, mSharedData->mWrapper.getAllocator(), mAllocator.mManager );
			return *retval;
		}
	};
	
	const char* RepXCollection::getLatestVersion() { return "3.1.1"; }

	RepXCollection* RepXCollection::create( RepXExtension** inExtensions, PxU32 inNumExtensions, const PxTolerancesScale& inScale, PxAllocatorCallback& inAllocator )
	{
		return PX_PROFILE_NEW( inAllocator, RepXCollectionImpl )( inExtensions, inNumExtensions, inScale, inAllocator );
	}

	RepXCollection* RepXCollection::create( const char* inFilename, RepXExtension** inExtensions, PxU32 inNumExtensions, PxAllocatorCallback& inAllocator )
	{
		PxFileBuffer theFileBuf( inFilename, PxFileBufferBase::OPEN_READ_ONLY );
		if (!theFileBuf.isOpen())
		{
			for ( PxU32 idx = 0; idx < inNumExtensions; ++idx ) 
				inExtensions[idx]->destroy();

			return NULL;
		}
		return create( &theFileBuf, inExtensions, inNumExtensions, inAllocator );
	}
	
	RepXCollection* RepXCollection::create( PxFileBuf* inCollectionData, RepXExtension** inExtensions, PxU32 inNumExtensions, PxAllocatorCallback& inAllocator )
	{
		PxTolerancesScale invalidScale;
		memset( &invalidScale, 0, sizeof( invalidScale ) );
		PX_ASSERT( invalidScale.isValid() == false );
		RepXCollectionImpl* theCollection = static_cast<RepXCollectionImpl*>( create( inExtensions, inNumExtensions, invalidScale, inAllocator ) );
		theCollection->load( *inCollectionData );
		return theCollection;
	}


	void RepXCollection::save( RepXCollection* inCollection, const char* inFilename )
	{
		//Create a px stream from the filename
		PxFileBuffer theFileBuf( inFilename, PxFileBufferBase::OPEN_WRITE_ONLY );
		PxFileBufPxStream theStream( theFileBuf );
		inCollection->save( &theStream );
	}
} }
