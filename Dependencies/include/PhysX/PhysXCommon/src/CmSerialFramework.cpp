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


/*
	TODO:
	- if things are not "automatic" through macros, how can we resolve all pointers/etc *before* creating the class? We can't.
	- streaming sample
	- articulations
	- add an empty ctor to PxBounds3, remove copy-ctor-to-itself (same for PxTransform)
	- fix insane design issues in HL/GeomUtils (GeomUtils class using HL class, etc)
- BigConvexData*				supportVertexMap;		//!< optional, only for large meshes! PT: REDUNDANT with ptr in chull data
- optimize BigConvexData space... 
	- Also PxHullGaussMapData, Gu::Valency and PxValenciesData could all use shorter types.
	- Ultimately Gu::ConvexHullData::mBigConvexData is not even needed, it would be yet another location in the big "mPolygons" buffer
	- Try to extract address from PxSerializable => PxResolvable : public PxSerializable
	- Try to teach the framework about inline arrays, etc
*/

//#ifdef REMOVED

// PX_SERIALIZATION

//#define DEBUG_STREAM_SIZES

#pragma warning( disable : 4512 ) //  assignment operator could not be generated
#pragma warning( disable : 4127 ) //  conditional expression is constant

#include "PxErrorCallback.h"
#include "CmSerialFramework.h"
#include "CmSerialAlignment.h"
#include "CmStringTable.h"


#define SERIAL_STATS
#ifdef SERIAL_STATS
#include <stdio.h>

using namespace physx;

static const bool gIncludeSentinels = false;

struct SerialStats
{
	PX_INLINE SerialStats() : count(0), totalSize(0)	{}

	PxU32	count;
	PxU32	totalSize;
};
#endif

static const char* SerialName(PxSerialType::Enum i)
{
	switch(i)
	{
	case PxSerialType::eUNDEFINED:				return "Undefined";

	case PxSerialType::eHEIGHTFIELD:			return "Gu::HeightField";
	case PxSerialType::eCONVEX_MESH:			return "Gu::ConvexMesh";
	case PxSerialType::eTRIANGLE_MESH:			return "Gu::TriangleMesh";
	case PxSerialType::eCLOTH_FABRIC:			return "NpClothFabric";

	case PxSerialType::eRIGID_DYNAMIC:			return "NpRigidDynamic";
	case PxSerialType::eRIGID_STATIC:			return "NpRigidStatic";
	case PxSerialType::eSHAPE:					return "NpShape";
	case PxSerialType::eMATERIAL:				return "NpMaterial";
	case PxSerialType::eCONSTRAINT:				return "NpConstraint";
	case PxSerialType::eCLOTH:					return "NpCloth";
	case PxSerialType::ePARTICLE_SYSTEM:		return "NpParticleSystem";
	case PxSerialType::ePARTICLE_FLUID:			return "NpParticleFluid";
	case PxSerialType::eAGGREGATE:				return "NpAggregate";
	case PxSerialType::eARTICULATION:			return "NpArticulation";
	case PxSerialType::eARTICULATION_LINK:		return "NpArticulationLink";
	case PxSerialType::eARTICULATION_JOINT:		return "NpArticulationJoint";

	case PxSerialType::eUSER_SPHERICAL_JOINT:	return "Ext::SphericalJoint";
	case PxSerialType::eUSER_REVOLUTE_JOINT:	return "Ext::RevoluteJoint";
	case PxSerialType::eUSER_PRISMATIC_JOINT:	return "Ext::PrismaticJoint";
	case PxSerialType::eUSER_FIXED_JOINT:		return "Ext::FixedJoint";
	case PxSerialType::eUSER_DISTANCE_JOINT:	return "Ext::DistanceJoint";
	case PxSerialType::eUSER_D6_JOINT:			return "Ext::D6Joint";

	case PxSerialType::eUSER_OBSERVER:			return "Observer";
	};
	PX_ASSERT(0);
	return NULL;
}

class FieldCallback : public PxSerialStream
{
	public:
	virtual		void		storeBuffer(const void* buffer, PxU32 /*size*/)
	{
		const PxFieldDescriptor* fd = reinterpret_cast<const PxFieldDescriptor*>(buffer);
		mArray.pushBack(fd);
	}
	virtual		PxU32		getTotalStoredSize()	{ return 0;	}
	Ps::Array<const PxFieldDescriptor*>	mArray;
};

typedef bool (*ProcessSerializableCallback)	(PxSerializable* s, void* userData);
static bool processCollection(Cm::InternalCollection& c, ProcessSerializableCallback cb, void* userData, bool processEmbedded)
{
	const PxU32 nb = c.internalGetNbObjects();
	for(PxU32 i=0;i<nb;i++)
	{
		PxSerializable* s = c.internalGetObject(i);
//		if(s->isSerializationDisabled())
//			continue;
		if(!(cb)(s, userData))
			return false;

		if(processEmbedded)
		{
			FieldCallback fields;
			s->getFields(fields, PxField::eSERIAL_EMBEDDED);

			const PxU32 nbFields = (PxU32)fields.mArray.size();
			for(PxU32 j=0;j<nbFields;j++)
			{
				const PxFieldDescriptor* fd = fields.mArray[j];
				PxSerializable* embedded = reinterpret_cast<PxSerializable*>(fd->Address(s));
				if(!(cb)(embedded, userData))
					return false;
			}
		}
	}
	return true;
}

#define MAX_REGISTERED_NAMES	64	// PT: "64 names should be enough for everybody"... or something.
struct RegisteredName
{
	const char*		mName;
	const char**	mNameAddress;
};
class InternalNameManager : public PxNameManager
{
	public:
		InternalNameManager() : mNbRegisteredNames(0)
	{
	}
	virtual	void registerName(const char** name)
	{
		if(*name==NULL)
			return;

		PX_ASSERT(mNbRegisteredNames<MAX_REGISTERED_NAMES);
		mRegisteredNames[mNbRegisteredNames].mNameAddress = name;
		mRegisteredNames[mNbRegisteredNames].mName = *name;
		mNbRegisteredNames++;
	}
	PxU32			mNbRegisteredNames;
	RegisteredName	mRegisteredNames[MAX_REGISTERED_NAMES];
};

struct ExportParams
{
	PxSerialStream*					mStream;
	physx::shdfnd3::Array<char>*	mStringTable;
};

void Cm::serializeCollection(Cm::InternalCollection& collection, PxSerialStream& stream, bool exportNames)
{
	// PT: sort collection by "order" value
	{
		struct SortCallback
		{
			static int compare(const void* e0, const void* e1)
			{
				PxSerializable** s0 = (PxSerializable**)e0;
				PxSerializable** s1 = (PxSerializable**)e1;
				const PxU32 order1 = (*s1)->getOrder();
				const PxU32 order0 = (*s0)->getOrder();
				return (int)order0 - (int)order1;
			}
		};

		const PxU32 nbObjects = collection.internalGetNbObjects();
		PxSerializable** objects = collection.internalGetObjects();
		qsort(objects, nbObjects, sizeof(PxSerializable*), SortCallback::compare);

		stream.storeBuffer(&nbObjects, sizeof(PxU32));
	}

	struct Local
	{
		static bool CountNbProcessed(PxSerializable* s, void* userData)
		{
			PxU32* count = reinterpret_cast<PxU32*>(userData);
			*count = *count + 1;
			return true;
		}

		static bool ExportOldAddressesToStream(PxSerializable* s, void* userData)
		{
			PxSerialStream* stream = reinterpret_cast<PxSerialStream*>(userData);
			stream->storeBuffer(&s, sizeof(PxSerializable*));
			return true;
		}

		static bool ExportSerializableToStream(PxSerializable* s, void* userData)
		{
			PX_ASSERT(s->getSerialType());
//			PxSerialStream* stream = reinterpret_cast<PxSerialStream*>(userData);

			ExportParams* params = (ExportParams*)userData;
			PxSerialStream* stream = params->mStream;
			physx::shdfnd3::Array<char>* stringTable = params->mStringTable;

			Cm::alignStream(*stream);

#ifdef DEBUG_STREAM_SIZES
			printf("%d\n", stream->getTotalStoredSize());
#endif
			// Test: optimization: automatically disable "auto resolve" if not needed
			// ### does that work for embedded objects???
/*			if(0)
			{
				s->disableAutoResolve();
//				FieldDescriptors fields(PX_DEBUG_EXP("FieldDescriptors"));	// Doesn't compile in Release!!
				FieldDescriptors fields;
				s->getFields(fields, PxField::eSERIAL_PTR);

				const PxU32 nbFields = (PxU32)fields.size();
				for(PxU32 j=0;j<nbFields;j++)
				{
					const FieldDescriptor* fd = fields[j];
					PxSerializable** tmp = reinterpret_cast<PxSerializable**>(fd->Address(s));
					if(*tmp)
					{
						s->enableAutoResolve();
						break;
					}
				}
			}

			if(0)
			{
				s->disableFields();
//				FieldDescriptors fields(PX_DEBUG_EXP("FieldDescriptors"));	// Doesn't compile in Release!!
				FieldDescriptors fields;
				s->getFields(fields, F_SERIALIZE);

				const PxU32 nbFields = (PxU32)fields.size();
				for(PxU32 j=0;j<nbFields;j++)
				{
					const FieldDescriptor* fd = fields[j];
					if(fd->IsDynamicArray())
					{
						if(fd->GetArrayAddress(s))
						{
							s->enableFields();
							break;
						}
					}
					else if(fd->mType==FIELD_STRING)
					{
						s->enableFields();
						break;
					}
					else if(fd->mType==FIELD_PX_ARRAY)
					{
						s->enableFields();
						break;
					}
				}
			}*/

			if(gIncludeSentinels)
			{
				int dead = 0xdead;
				stream->storeBuffer(&dead, 4);
			}

			// PT: gather names to export
			InternalNameManager nameManager;
			s->registerNameForExport(nameManager);

			// PT: convert pointer to indices for exported names
			for(PxU32 i=0;i<nameManager.mNbRegisteredNames;i++)
			{
				const RegisteredName& name = nameManager.mRegisteredNames[i];
				size_t* target = reinterpret_cast<size_t*>(name.mNameAddress);
				if(stringTable)
					*target = 1 + addToStringTable(*stringTable, name.mName);	// PT: +1 because we reserve 0 for NULL pointer (no name)
				else
					*target = 0;
			}

			// PT: export object to file with converted names
			stream->storeBuffer(s, s->getObjectSize());

			// PT: restore original name pointers (undo pointer-to-indice conversion)
			for(PxU32 i=0;i<nameManager.mNbRegisteredNames;i++)
			{
				const RegisteredName& name = nameManager.mRegisteredNames[i];
				*name.mNameAddress = name.mName;
			}

			if(gIncludeSentinels)
			{
				int dead = 0xdead;
				stream->storeBuffer(&dead, 4);
			}

			return true;
		}

		static bool ExportExtraData(PxSerializable* s, void* userData)
		{
			PxSerialStream* stream = reinterpret_cast<PxSerialStream*>(userData);
			Cm::alignStream(*stream);

			if(gIncludeSentinels)
			{
				int dead = 0xdead;
				stream->storeBuffer(&dead, 4);
			}

			s->exportExtraData(*stream);

			if(gIncludeSentinels)
			{
				int dead = 0xdead;
				stream->storeBuffer(&dead, 4);
			}

			return true;
		}

		static bool ExportFields(PxSerializable* s, void* userData)
		{
			PxSerialStream* stream = reinterpret_cast<PxSerialStream*>(userData);

			FieldCallback fields;
			s->getFields(fields, F_SERIALIZE);

			const PxU32 nbFields = (PxU32)fields.mArray.size();
			for(PxU32 j=0;j<nbFields;j++)
			{
				const PxFieldDescriptor* fd = fields.mArray[j];
				if(fd->IsDynamicArray())
				{
					void* arrayBase = fd->GetArrayAddress(s);
					if(arrayBase)
					{
						const PxU32 arraySize = fd->GetDynamicArraySize(s);
						const PxU32 fieldSize = fd->FieldSize();
						if(fd->mFlags & F_ALIGN)
							Cm::alignStream(*stream);
						stream->storeBuffer(arrayBase, arraySize * fieldSize);
					}
				}
				else if(fd->IsStaticArray())
				{
					void* arrayBase = fd->GetArrayAddress(s);
					if(arrayBase)
					{
						const PxU32 arraySize = fd->GetStaticArraySize();
						const PxU32 fieldSize = fd->FieldSize();
						if(fd->mFlags & F_ALIGN)
							Cm::alignStream(*stream);
						stream->storeBuffer(arrayBase, arraySize * fieldSize);
					}
				}
				else if(fd->mType==PxField::eSTRING)
				{
					char* stringBase = reinterpret_cast<char*>(fd->GetArrayAddress(s));
					const PxU32 stringSize = (PxU32)strlen(stringBase)+1;
					if(fd->mFlags & F_ALIGN)
						Cm::alignStream(*stream);
					stream->storeBuffer(stringBase, stringSize * sizeof(char));
				}
				else if(fd->mType==PxField::ePX_ARRAY)
				{
					Ps::Array<int>* pxArray = reinterpret_cast<Ps::Array<int>*>(fd->Address(s));
					if(fd->mFlags & F_ALIGN)
						Cm::alignStream(*stream);
					pxArray->exportArray(*stream, false);
				}
				else
				{
					PX_ASSERT(!"Found unsupported F_SERIALIZE field!");
				}
			}
//			return true;
			return ExportExtraData(s, userData);
		}
	};


	physx::shdfnd3::Array<char>	stringTable;

	ExportParams params;
	params.mStream		= &stream;
	params.mStringTable	= exportNames ? &stringTable : NULL;
	processCollection(collection, Local::ExportSerializableToStream, &params, false);

	PxU32 nbProcessed = 0;
	processCollection(collection, Local::CountNbProcessed, &nbProcessed, true);
	stream.storeBuffer(&nbProcessed, sizeof(PxU32));	// PT: only useful for ConvX

	processCollection(collection, Local::ExportOldAddressesToStream, &stream, true);
//	Cm::alignStream(stream);
	processCollection(collection, Local::ExportFields, &stream, true);
//	processCollection(collection, Local::ExportExtraData, &stream, true);

	// PT: export string table, if there is one
	const PxU32 length = stringTable.size();
	const char* table = stringTable.begin();
	stream.storeBuffer(&length, sizeof(PxU32));
	if(length)
		stream.storeBuffer(table, length);
}

static char* read32(char* address, PxU32& data)
{
	PxU32* p = reinterpret_cast<PxU32*>(address);
	data = *p;
	address += sizeof(PxU32);
	return address;
}

bool Cm::deserializeCollection(InternalCollection& collection, RefResolver& Ref, void* buffer)
{
#ifdef SERIAL_STATS
	SerialStats stats[PxSerialType::eLAST];
#endif
	PxU32 totalPadding = 0;

	char* Address = reinterpret_cast<char*>(buffer);

	PxU32 nbObjects;
	Address = read32(Address, nbObjects);

#ifdef _DEBUG
	PxU32 nbClasses = 0;
#endif
	while(nbObjects--)
	{
		Address = Cm::alignStream(Address, totalPadding);

#ifdef SERIAL_STATS
		char* BaseAddress = Address;
#endif

		PxSerializable* H = reinterpret_cast<PxSerializable*>(Address);
		const PxType classType = H->getSerialType();
		PxSerializable* Serial = createClass(classType, Address, Ref);
		PX_ASSERT(Serial);

#ifdef SERIAL_STATS
		if(Serial)
		{
			stats[classType].count++;
			stats[classType].totalSize += PxU32(Address - BaseAddress);
#ifdef DEBUG_STREAM_SIZES
			static PxU32 tsize = 8;
			tsize += PxU32(Address - BaseAddress);
			printf("%d\n", tsize);
//			printf("%d\n", PxU32(Address - BaseAddress));
#endif
		}
#endif

		if(Serial)
		{
			collection.internalAdd(Serial);
#ifdef _DEBUG
			nbClasses++;
#endif
		}
	}

	// Import old addresses
	{
		struct Local
		{
			struct ImportParams
			{
				char**			mAddress;
				RefResolver*	mResolver;
			};

			static bool ImportOldAddress(PxSerializable* s, void* userData)
			{
				ImportParams* params = reinterpret_cast<ImportParams*>(userData);

				char* Address = *params->mAddress;
				PxSerializable** oldAddress = reinterpret_cast<PxSerializable**>(Address);
				Address += sizeof(PxSerializable*);

				params->mResolver->setNewAddress(*oldAddress, s);

				*params->mAddress = Address;
				return true;
			}
		};

		PxU32 nbProcessed;
		Address = read32(Address, nbProcessed);

		Local::ImportParams	importParams;
		importParams.mAddress	= &Address;
		importParams.mResolver	= &Ref;

		if(!processCollection(collection, Local::ImportOldAddress, &importParams, true))
			return false;
	}

	{
		//sschirm: extracted from Local because of osx internal compile error
		struct LocalImportParams
		{
			char**			mAddress;
			PxU32			mTotalPadding;
#ifdef SERIAL_STATS
			SerialStats*	mStats;
#endif
		};
		
		struct Local
		{
			static bool ImportExtraData(PxSerializable* s, void* userData)
			{
				LocalImportParams* params = reinterpret_cast<LocalImportParams*>(userData);
				char* Address = *params->mAddress;

				Address = Cm::alignStream(Address, params->mTotalPadding);
#ifdef SERIAL_STATS
				char* previousAddress = Address;
#endif
				Address = s->importExtraData(Address, params->mTotalPadding);
#ifdef SERIAL_STATS
				params->mStats[s->getSerialType()].totalSize += PxU32(Address - previousAddress);
#endif
				*params->mAddress = Address;
				return true;
			}

			static bool ImportFields(PxSerializable* s, void* userData)
			{
				if(s->areFieldsDisabled())
//					return true;
					return ImportExtraData(s, userData);

				LocalImportParams* params = reinterpret_cast<LocalImportParams*>(userData);

				FieldCallback fields;
				s->getFields(fields, F_SERIALIZE);

				const PxU32 nbFields = (PxU32)fields.mArray.size();
				for(PxU32 j=0;j<nbFields;j++)
				{
					const PxFieldDescriptor* fd = fields.mArray[j];
					if(fd->IsDynamicArray())
					{
						if(fd->GetArrayAddress(s))
						{
							char* Address = *params->mAddress;
							if(fd->mFlags & F_ALIGN)
								Address = Cm::alignStream(Address, params->mTotalPadding);

							char** arrayAddress = reinterpret_cast<char**>(fd->Address(s));
							*arrayAddress = Address;

							const PxU32 arraySize = fd->GetDynamicArraySize(s);
							const PxU32 fieldSize = fd->FieldSize();
							Address += arraySize * fieldSize;
#ifdef SERIAL_STATS
							params->mStats[s->getSerialType()].totalSize += arraySize * fieldSize;
#endif
							*params->mAddress = Address;
						}
					}
					else if(fd->IsStaticArray())
					{
						if(fd->GetArrayAddress(s))
						{
							char* Address = *params->mAddress;
							if(fd->mFlags & F_ALIGN)
								Address = Cm::alignStream(Address, params->mTotalPadding);

							char** arrayAddress = reinterpret_cast<char**>(fd->Address(s));
							*arrayAddress = Address;

							const PxU32 arraySize = fd->GetStaticArraySize();
							const PxU32 fieldSize = fd->FieldSize();
							Address += arraySize * fieldSize;
#ifdef SERIAL_STATS
							params->mStats[s->getSerialType()].totalSize += arraySize * fieldSize;
#endif
							*params->mAddress = Address;
						}
					}


					else if(fd->mType==PxField::eSTRING)
					{
						char* Address = *params->mAddress;
						if(fd->mFlags & F_ALIGN)
							Address = Cm::alignStream(Address, params->mTotalPadding);

						char** arrayAddress = reinterpret_cast<char**>(fd->Address(s));
						*arrayAddress = Address;

						const PxU32 stringSize = (PxU32)strlen(*arrayAddress)+1;
						Address += stringSize * sizeof(char);
#ifdef SERIAL_STATS
						params->mStats[s->getSerialType()].totalSize += stringSize * sizeof(char);
#endif
						*params->mAddress = Address;
					}
					else if(fd->mType==PxField::ePX_ARRAY)
					{
						char* Address = *params->mAddress;
						if(fd->mFlags & F_ALIGN)
							Address = Cm::alignStream(Address, params->mTotalPadding);

						Ps::Array<int>* pxArray = reinterpret_cast<Ps::Array<int>*>(fd->Address(s));
#ifdef SERIAL_STATS
						char* previousAddress = Address;
#endif
						Address = pxArray->importArray(Address);
#ifdef SERIAL_STATS
						params->mStats[s->getSerialType()].totalSize += PxU32(Address - previousAddress);
#endif
						*params->mAddress = Address;
					}
					else
					{
						PX_ASSERT(!"Found unsupported F_SERIALIZE field!");
					}
				}
//				return true;
				return ImportExtraData(s, userData);
			}
		};

		LocalImportParams	importParams;
		importParams.mAddress		= &Address;
		importParams.mTotalPadding	= 0;
#ifdef SERIAL_STATS
		importParams.mStats			= stats;
#endif
		processCollection(collection, Local::ImportFields, &importParams, true);
//		processCollection(collection, Local::ImportExtraData, &importParams, true);
		totalPadding += importParams.mTotalPadding;
	}

	// PT: import string table
	PxU32 stringTableSize;
	Address = read32(Address, stringTableSize);

	const char* stringTable = NULL;
	if(stringTableSize)
	{
		stringTable = Address;
		Address += stringTableSize;
	}
	Ref.setStringTable(stringTable);

//	if(1)	// Automatic resolve
	{
		struct Local
		{
			static bool AutoResolve(PxSerializable* s, void* userData)
			{
				if(s->isAutoResolveDisabled())
					return true;

				RefResolver& resolver = *reinterpret_cast<RefResolver*>(userData);

				FieldCallback fields;
				s->getFields(fields, PxField::eSERIAL_PTR);

				const PxU32 nbFields = (PxU32)fields.mArray.size();
				for(PxU32 j=0;j<nbFields;j++)
				{
					const PxFieldDescriptor* fd = fields.mArray[j];

					PxSerializable** tmp = reinterpret_cast<PxSerializable**>(fd->Address(s));
					if(*tmp)
					{
						void* relocated = resolver.newAddress(*tmp);
						if(relocated)
							*tmp = reinterpret_cast<PxSerializable*>(relocated);
						else
						{
							char buffer[2048];
							const char* className = SerialName(PxSerialType::Enum(s->getSerialType()));
							sprintf(buffer, "auto-resolve failed for %s::%s", className ? className : "(null)", fd->mName);
							Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, buffer);
							return false;
						}
					}
				}
				return true;
			}
		};
		if(!processCollection(collection, Local::AutoResolve, &Ref, true))
			return false;
	}
/*	else	// Manual resolve
	{
		PxU32 nb = getNbObjects();
		for(PxU32 i=0;i<nb;i++)
			getObject(i)->manualResolve(Ref);
	}*/

#ifdef SERIAL_STATS
	for(PxU32 i=0;i<PxSerialType::eLAST;i++)
	{
		if(stats[i].count)
			printf("%s | %d | %d\n", SerialName(PxSerialType::Enum(i)), stats[i].count, stats[i].totalSize);
	}
	printf("Padding: %d bytes\n", totalPadding);
#endif
	return true;
}

PxSerializable* Cm::RefResolver::newAddress(PxSerializable* oldAddress) const
{
	const HashMapResolver::Entry* e = mResolver.find(oldAddress);
	return e ? static_cast<PxSerializable*>(e->second) : NULL;
}

void Cm::RefResolver::setNewAddress(PxSerializable* oldAddress, PxSerializable* newAddress)
{
	if(!mResolver.insert(oldAddress, newAddress))
		mResolver[oldAddress] = newAddress;
}

void Cm::RefResolver::setStringTable(const char* stringTable)
{
	mStringTable = stringTable;
}

const char* Cm::RefResolver::resolveName(const char* name)
{
	if(!mStringTable || !name)
		return NULL;

	const size_t offset = reinterpret_cast<size_t>(name) - 1;	// PT: -1 to undo the +1 we did when serializing
	return mStringTable + offset;
}

PxSerializable*	Cm::UserReferences::getObjectFromID(void* userData) const
{
	const UserHashMapResolver::Entry* e = mResolver.find(userData);
	return e ? static_cast<PxSerializable*>(e->second) : NULL;
}

void Cm::UserReferences::setUserData(PxSerializable* object, void* userData)
{
	if(!mResolver.insert(userData, object))
		mResolver[userData] = object;
}


//~PX_SERIALIZATION
//#endif
