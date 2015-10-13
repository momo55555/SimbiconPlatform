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

#include "RepXUpgrader.h"
#include "PxProfileMemoryEvents.h" //safeStrEq
#include "RepXImpl.h" //RepXNode and some helper functions.
#include "RepXReader.h"
#include "RepXMemoryAllocator.h"

using namespace physx::profile;

namespace physx { namespace repx {

	#define DEFINE_REPX_DEFAULT_PROPERTY( name, val ) RepXDefaultEntry( name, val ),

	static RepXDefaultEntry gRepX1_0Defaults[] = {
	#include "RepX1_0Defaults.h"
	};

	static PxU32 gNumRepX1_0Default = sizeof( gRepX1_0Defaults ) / sizeof ( *gRepX1_0Defaults );
	
	inline const char* nextPeriod( const char* str )
	{
		for( ++str; str && *str && *str != '.'; ++str ); //empty loop intentional
		return str;
	}
	
	typedef ProfileHashMap<const char*, PxU32> TNameOffsetMap;
	
	void setMissingPropertiesToDefault( RepXNode* topNode, RepXReaderWriter& editor, const RepXDefaultEntry* defaults, PxU32 numDefaults, TNameOffsetMap& map )
	{
		for ( RepXNode* child = topNode->mFirstChild; child != NULL; child = child->mNextSibling )
			setMissingPropertiesToDefault( child, editor, defaults, numDefaults, map );

		const TNameOffsetMap::Entry* entry( map.find( topNode->mName ) );
		if ( entry )
		{
			RepXReaderWriter& theReader( editor );
			theReader.setNode( *topNode );
			char nameBuffer[512] = {0};
			size_t nameLen = strlen( topNode->mName );
			//For each default property entry for this node type.
			for ( const RepXDefaultEntry* item = defaults + entry->second; strncmp( item->name, topNode->mName, nameLen ) == 0; ++item )
			{
				bool childAdded = false;
				const char* nameStart = item->name + nameLen;
				++nameStart;
				theReader.pushCurrentContext();
				const char* str = nameStart;
				while( *str )
				{
					 const char *period = nextPeriod( str );
					 size_t len = PxMin( period - str, ptrdiff_t(1023) ); //can't be too careful these days.
					 memcpy( nameBuffer, str, len );
					 nameBuffer[len] = 0;
					 if ( theReader.gotoChild( nameBuffer ) == false )
					 {
						 childAdded = true;
						 theReader.addOrGotoChild( nameBuffer );
					 }
					 if (*period )
						 str = period + 1;
					 else
						 str = period;
				}
				if ( childAdded )
					theReader.setCurrentItemValue( item->value );
				theReader.popCurrentContext();
			}
		}
	}

	
	static void setMissingPropertiesToDefault( RepXCollection& collection, RepXReaderWriter& editor, const RepXDefaultEntry* defaults, PxU32 numDefaults )
	{
		FoundationWrapper wrapper( collection.getAllocator() );
		//Release all strings at once, instead of piece by piece
		RepXMemoryAllocatorImpl alloc( collection.getAllocator() );
		//build a hashtable of the initial default value strings.
		TNameOffsetMap nameOffsets( wrapper );
		for ( PxU32 idx = 0; idx < numDefaults; ++idx )
		{
			const RepXDefaultEntry& item( defaults[idx] );
			size_t nameLen = 0;
			const char* periodPtr = nextPeriod (item.name);
			for ( ; periodPtr && *periodPtr; ++periodPtr ) if( *periodPtr == '.' )	break;
			if ( periodPtr == NULL || *periodPtr != '.' ) continue;
			nameLen = periodPtr - item.name;
			char* newMem = (char*)alloc.allocate( PxU32(nameLen + 1) );
			memcpy( newMem, item.name, nameLen );
			newMem[nameLen] = 0;
		
			if ( nameOffsets.find( newMem ) )
				alloc.deallocate( (PxU8*)newMem );
			else
				nameOffsets.insert( newMem, idx );
		}
		//Run through each collection item, and recursively find it and its children
		//If an object's name is in the hash map, check and add any properties that don't exist.
		//else return.
		for ( const RepXCollectionItem* item = collection.begin(), *end = collection.end(); item != end; ++ item )
		{
			RepXCollectionItem theItem( *item );
			setMissingPropertiesToDefault( theItem.mDescriptor, editor, defaults, numDefaults, nameOffsets );
		}
	}

	RepXCollection& RepXUpgrader::upgrade10CollectionTo3_1Collection(RepXCollection& src)
	{
		RepXReaderWriter& editor( src.createNodeEditor() );
		setMissingPropertiesToDefault(src, editor, gRepX1_0Defaults, gNumRepX1_0Default );
		RepXCollection* dest = &src.createCollection("3.1.1");
		for ( const RepXCollectionItem* item = src.begin(), *end = src.end(); item != end; ++ item )
		{
			//either src or dest could do the copy operation, it doesn't matter who does it.
			RepXCollectionItem newItem( item->mLiveObject, src.copyRepXNode( item->mDescriptor ) );
			editor.setNode( *const_cast<RepXNode*>( newItem.mDescriptor ) );
			//Some old files have this name in their system.
			editor.renameProperty( "MassSpaceInertia", "MassSpaceInertiaTensor" );
			editor.renameProperty( "SleepEnergyThreshold", "SleepThreshold" );

			if ( strstr( newItem.mLiveObject.mTypeName, "Joint" ) || strstr( newItem.mLiveObject.mTypeName, "joint" ) )
			{
				//Joints changed format a bit.  old joints looked like:
				/*
				<Actor0 >1627536</Actor0>
				<Actor1 >1628368</Actor1>
				<LocalPose0 >0 0 0 1 0.5 0.5 0.5</LocalPose0>
				<LocalPose1 >0 0 0 1 0.3 0.3 0.3</LocalPose1>*/
				//New joints look like:
				/*
				<Actors >
					<actor0 >58320336</actor0>
					<actor1 >56353568</actor1>
				</Actors>
				<LocalPose >
					<eACTOR0 >0 0 0 1 0.5 0.5 0.5</eACTOR0>
					<eACTOR1 >0 0 0 1 0.3 0.3 0.3</eACTOR1>
				</LocalPose>
				*/
				const char* actor0, *actor1, *lp0, *lp1;
				editor.readAndRemoveProperty( "Actor0", actor0 );
				editor.readAndRemoveProperty( "Actor1", actor1 ); 
				editor.readAndRemoveProperty( "LocalPose0", lp0 );
				editor.readAndRemoveProperty( "LocalPose1", lp1 );

				editor.addOrGotoChild( "Actors" );
				editor.writePropertyIfNotEmpty( "actor0", actor0 );
				editor.writePropertyIfNotEmpty( "actor1", actor1 );
				editor.leaveChild();

				editor.addOrGotoChild( "LocalPose" );
				editor.writePropertyIfNotEmpty( "eACTOR0", lp0 );
				editor.writePropertyIfNotEmpty( "eACTOR1", lp1 );
				editor.leaveChild();
			}



			//now desc owns the new node.  Collections share a single allocation pool, however,
			//which will get destroyed when all the collections referencing it are destroyed themselves.
			//Data on nodes is shared between nodes, but the node structure itself is allocated.
			dest->addCollectionItem( newItem );
		}
		editor.release();
		src.destroy();
		return *dest;
	}


	RepXCollection& RepXUpgrader::upgradeCollection(RepXCollection& src)
	{
		RepXCollection* dest = NULL;
		const char* srcVersion = src.getVersion();
		if ( safeStrEq( src.getVersion(), RepXCollection::getLatestVersion() ) )
			return src;
		else
		{
			if ( safeStrEq( srcVersion, "1.0" ) || safeStrEq( srcVersion, "3.1" ) )
				dest = &upgrade10CollectionTo3_1Collection( src );
			else
				PX_ASSERT( false );
		}
		return *dest;
	}
}}