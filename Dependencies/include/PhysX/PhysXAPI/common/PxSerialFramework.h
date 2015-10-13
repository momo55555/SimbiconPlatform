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


//#ifdef REMOVED

#ifndef PX_PHYSICS_COMMON_NX_SERIAL_FRAMEWORK
#define PX_PHYSICS_COMMON_NX_SERIAL_FRAMEWORK

/** \addtogroup common
@{
*/

#include "common/PxPhysXCommon.h"

// PX_SERIALIZATION

#include "common/PxFields.h"
#include "common/PxFieldDescriptor.h"
#include "PxFlags.h"
#include "common/PxStream.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

typedef PxU16 PxType;
class PxSerializable;

//! Objects are written in a fixed order within a serialized file.
struct PxSerialOrder
{
	enum Enum
	{
		eCONVEX			= 20,
		eTRIMESH		= 21,
		eHEIGHTFIELD	= 22,
		eDEFMESH		= 23,
		eCLOTHMESH		= 24,
		eMATERIAL		= 50,
		eSHAPE			= 80,
		eSTATIC			= 81,
		eDYNAMIC		= 82,
		eDEFAULT		= 100,
		eARTICULATION	= 120,
		eJOINT			= 150,
		eCONSTRAINT		= 200,
	};
};

/**
\brief Class used to "resolve pointers" during deserialization.

The ref-resolver remaps pointers to PxSerializable objects within a deserialized memory block.
This class is mainly used by the serialization framework. Users should not have to worry about it.

@see PxSerializable
*/
class PxRefResolver
{
	public:
	virtual					~PxRefResolver()														{}

	/**
	\brief Retrieves new address of deserialized object

	This is called by the serialization framework.

	\param[in]	oldAddress	Old address of PxSerializable object. See #PxSerializable
	\return		New address of PxSerializable object

	@see PxSerializable
	*/
	virtual	PxSerializable*	newAddress(PxSerializable* oldAddress) const	= 0;

	/**
	\brief Sets new address of deserialized object.

	This is called by the serialization framework.

	\param[in]	oldAddress	Old address of PxSerializable object. See #PxSerializable
	\param[in]	newAddress	New address of PxSerializable object. See #PxSerializable

	@see PxSerializable
	*/
	virtual	void			setNewAddress(PxSerializable* oldAddress, PxSerializable* newAddress)	= 0;

	/**
	\brief Sets current string table.

	This is called by the serialization framework.

	\param[in]	stringTable	Current string table address
	*/
	virtual	void			setStringTable(const char* stringTable)	= 0;

	/**
	\brief Resolves exported name.

	This is called by the serialization framework.

	\param[in]	name	Name to be resolved
	\return		Resolved name
	*/
	virtual	const char*		resolveName(const char* name)	= 0;
};

/**
\brief Container for user-defined names/references

This is mainly a "link" object accessed by the framework when serializing and
deserializing subsets.

@see PxSerializable
*/
class PxUserReferences
{
	public:
	virtual					~PxUserReferences()														{}

	/**
	\brief Gets PxSerializable object from its user-data/name

	This is called by the framework during deserialization, when import names are
	passed to the deserialize function.

	\param[in]	userData	user-defined name for this object
	\return		Corresponding object, or NULL if not found

	@see PxSerializable
	*/
	virtual	PxSerializable*	getObjectFromID(void* userData) const									= 0;

	/**
	\brief Sets user-data/name for a PxSerializable

	This is called by the framework during deserialization, when export names are
	retrieved from a deserialized collection.

	This can also be called by users to link a subset to a completely different
	subset than the one it was originally linked to.

	\param[in]	object		Serializable object. See #PxSerializable
	\param[in]	userData	user-defined name for this object

	@see PxSerializable
	*/
	virtual	void			setUserData(PxSerializable* object, void* userData)						= 0;
};

// PT: if you change this enum, make sure you replicate the change in ConvX
struct PxSerialType
{
	enum Enum
	{
		eUNDEFINED,

		eHEIGHTFIELD,
		eCONVEX_MESH,
		eTRIANGLE_MESH,
		eDEFORMABLE_MESH,
		eCLOTH_FABRIC,

		eRIGID_DYNAMIC,
		eRIGID_STATIC,
		eSHAPE,
		eMATERIAL,
		eCONSTRAINT,
		eDEFORMABLE,
		eCLOTH,
		ePARTICLE_SYSTEM,
		ePARTICLE_FLUID,
		eATTACHMENT,
		eAGGREGATE,
		eARTICULATION,
		eARTICULATION_LINK,
		eARTICULATION_JOINT,

		eDEFORMABLE_CORE,

		// Bad design here: the following types are "user types" and shouldn't be here in the end
		eUSER_SPHERICAL_JOINT,
		eUSER_REVOLUTE_JOINT,
		eUSER_PRISMATIC_JOINT,
		eUSER_FIXED_JOINT,
		eUSER_DISTANCE_JOINT,
		eUSER_D6_JOINT,

		eUSER_OBSERVER,

		eLAST
	};
};

struct PxSerialFlag
{
	enum Enum
	{
		eOWNS_MEMORY			= (1<<0),
//		eDISABLED				= (1<<1),
		eDISABLE_AUTO_RESOLVE	= (1<<1),
		eDISABLE_FIELDS			= (1<<2),
		eIN_SCENE				= (1<<3),
	};
};

typedef PxFlags<PxSerialFlag::Enum, PxU16> PxSerialFlags;
PX_FLAGS_OPERATORS(PxSerialFlag::Enum, PxU16);


typedef PxSerializable*	(*PxClassCreationCallback)(char*& address, PxRefResolver& v);


/**
\brief Collection class for serialization.

A collection is a container for serializable SDK objects. All serializable SDK objects inherit from PxSerializable.
Serialization and deserialization only work through collections.

A scene is typically serialized using the following steps:

1) create a collection
2) collect objects to serialize
3) serialize collection
4) release collection

For example the code may look like this:

	PxPhysics* physics;	// The physics SDK object
	PxScene* scene;		// The physics scene
	SerialStream s;		// The user-defined stream doing the actual write to disk

	PxCollection* collection = physics->createCollection();	// step 1)
	PxCollectForExportSDK(*physics, *collection);			// step 2)
	PxCollectForExportScene(*scene, *collection);			// step 2)
	collection->serialize(s);								// step 3)
	physics->releaseCollection(collection);					// step 4)

A scene is typically deserialized using the following steps:

1) load a serialized block somewhere in memory
2) create a collection object
3) deserialize objects (populate collection with objects from the memory block)
4) add collected objects to scene
5) release collection

For example the code may look like this:

	PxPhysics* physics;	// The physics SDK object
	PxScene* scene;		// The physics scene
	void* memory128;	// a 128-byte aligned buffer previously loaded from disk by the user	- step 1)

	PxCollection* collection = physics->createCollection();	// step 2)
	collection->deserialize(memory128, NULL, NULL);			// step 3)
	physics->addCollection(*collection, scene);				// step 4)
	physics->releaseCollection(collection);					// step 5)

@see PxSerializable
*/
class PxCollection
{
	friend class PxSerializable;
	virtual	void						addUnique(PxSerializable*)	= 0;

public:
										PxCollection()	{}
	virtual								~PxCollection()	{}

	/**
	\brief Serializes a collection.

	Writes out collected objects to a binary stream. Objects are output in the order
	defined by PxSerialOrder, according to their type.

	"Export names" and "import names", as defined by the setUserData and
	addExternalRef functions, are also serialized.

	\param[in]	stream		User-defined serialization stream. See #PxSerialStream
	\param[in]	exportNames	If true, objects' names are serialized along with the objects. Not serializing names produce smaller files.

	@see PxSerialStream PxSerialOrder setUserData addExternalRef
	*/
	virtual	void						serialize(PxSerialStream& stream, bool exportNames=false)	= 0;

	/**
	\brief Deserializes a collection.

	Initializes/creates objects within the given input buffer, which must have
	been deserialized from disk already by the user. The input buffer must be
	16-bytes aligned.
	
	Deserialized objects are added to the collection.

	Export names for the collection can be retrieved, if necessary.
	Import names from another collection can be passed, if necesary.

	\param[in]	buffer16		Deserialized input buffer, 16-bytes aligned
	\param[out]	exportNames		Possible export names, or NULL. See #PxUserReferences
	\param[in]	importNames		Possible import names, or NULL. See #PxUserReferences
	\return		True if success

	@see PxUserReferences
	*/
	virtual	bool						deserialize(void* buffer16, PxUserReferences* exportNames, const PxUserReferences* importNames)	= 0;

	/**
	\brief Sets user-data/name for a PxSerializable

	This is used to assign a user-defined data to a PxSerializable. This data is then used
	as the object's name, its reference within other collections. For example when serializing
	subsets, one collection (containing one subset) might have a reference to an object in
	another collection (another subset). The system needs to know about those external references.

	This user-data is also known as an "export name".

	\param[in]	object		Serializable object. See #PxSerializable
	\param[in]	userData	user-defined name for this object

	@see PxSerializable addExternalRef
	*/
	virtual	void						setUserData(PxSerializable* object, void* userData)	= 0;

	/**
	\brief Declares an external reference to the collection

	Some objects in the collection might have pointers/references to objects that are not within
	the same collection. The system needs to know about those external references to properly
	recreate objects during deserialization.

	This user-data is also known as an "import name".

	\param[in]	object		Serializable object. See #PxSerializable
	\param[in]	userData	user-defined name for this object

	@see PxSerializable setUserData
	*/
	virtual	void						addExternalRef(PxSerializable* object, void* userData)	= 0;

	/**
	\brief Gets number of objects in the collection
	\return	Number of objects in the collection
	*/
	virtual	PxU32						getNbObjects()	= 0;

	/**
	\brief Gets object from the collection

	\param[in]	index	Object index, between 0 (incl) and getNbObjects() (excl).
	\return		Desired object from the collection

	@see PxSerializable
	*/
	virtual	PxSerializable*				getObject(PxU32 index)	= 0;
};

class PxNameManager
{
	public:
	virtual ~PxNameManager() {}
	virtual	void						registerName(const char**)	= 0;
};

/**
\brief Base class for serializable objects

@see PxRefResolver PxCollection PxSerialStream
*/
class PxSerializable
{
	public:
											PxSerializable(PxRefResolver& v)
											{
												mSerialFlags &= ~PxSerialFlag::eOWNS_MEMORY;
											    PX_FORCE_PARAMETER_REFERENCE(v);
											}
											PxSerializable() : mType(PxSerialType::eUNDEFINED), mSerialFlags(PxSerialFlag::eOWNS_MEMORY)
											{
											}
	virtual									~PxSerializable()											{}

	virtual		PxU32						getOrder()											const	{ return PxSerialOrder::eDEFAULT;						}

	/**
	\brief Adds an object to the collection.

	\param[in]	c				collection to add the object to

	@see PxCollection
	*/
	virtual		void						collectForExport(PxCollection& c)							{ c.addUnique(this);									}

	virtual		bool						getFields(PxSerialStream&, PxU32)					const	{ return true;											}
	virtual		bool						getFields(PxSerialStream&, PxField::Enum)			const	{ return true;											}
	virtual		bool						getFields(PxSerialStream&)							const	{ return true;											}
	virtual		const PxFieldDescriptor*	getFieldDescriptor(const char*)						const	{ return NULL;											}

	virtual		PxU32						getObjectSize()										const	= 0;

	virtual		void						exportExtraData(PxSerialStream&)							{														}
	virtual		char*						importExtraData(char* address, PxU32&)						{ return address;										}
	virtual		bool						resolvePointers(PxRefResolver&, void*)						{ return true;											}

	/**
	\brief	Returns string name of serialized class.
	\return	Name of serialized class
	*/
	virtual		const char*					getClassName()										const	{ return NULL;											}

	/**
	\brief	Built-in string-based RTTI system for serialized files.

	\param[in]	superClass	name of superClass to test against

	\return	true if object is a sub-class of superClass
	*/
	virtual		bool						isKindOf(const char* superClass)					const	{ PX_UNUSED(superClass);	return false;				}

	virtual		void						registerNameForExport(PxNameManager&)	{}

//		PX_INLINE	void					disableSerialization()										{ mSerialFlags |= PX_SERIAL_DISABLED;					}
//		PX_INLINE	void					enableSerialization()										{ mSerialFlags &= ~PX_SERIAL_DISABLED;					}
//		PX_INLINE	PxU16					isSerializationDisabled()							const	{ return PxU16(mSerialFlags & PX_SERIAL_DISABLED);		}

	PX_INLINE	void						enableInScene()												{ mSerialFlags |= PxSerialFlag::eIN_SCENE;							}
	PX_INLINE	void						disableInScene()											{ mSerialFlags &= ~PxSerialFlag::eIN_SCENE;							}
	PX_INLINE	PxU16						isInScene()											const	{ return PxU16(mSerialFlags & PxSerialFlag::eIN_SCENE);				}

	PX_INLINE	void						disableAutoResolve()										{ mSerialFlags |= PxSerialFlag::eDISABLE_AUTO_RESOLVE;				}
	PX_INLINE	void						enableAutoResolve()											{ mSerialFlags &= ~PxSerialFlag::eDISABLE_AUTO_RESOLVE;				}
	PX_INLINE	PxU16						isAutoResolveDisabled()								const	{ return PxU16(mSerialFlags & PxSerialFlag::eDISABLE_AUTO_RESOLVE);	}

	PX_INLINE	void						disableFields()												{ mSerialFlags |= PxSerialFlag::eDISABLE_FIELDS;					}
	PX_INLINE	void						enableFields()												{ mSerialFlags &= ~PxSerialFlag::eDISABLE_FIELDS;					}
	PX_INLINE	PxU16						areFieldsDisabled()									const	{ return PxU16(mSerialFlags & PxSerialFlag::eDISABLE_FIELDS);		}

	PX_INLINE	void						setOwnsMemory()												{ mSerialFlags |= PxSerialFlag::eOWNS_MEMORY;						}
	PX_INLINE	void						clearOwnsMemory()											{ mSerialFlags &= ~PxSerialFlag::eOWNS_MEMORY;						}
	PX_INLINE	PxU16						ownsMemory()										const	{ return PxU16(mSerialFlags & PxSerialFlag::eOWNS_MEMORY);			}

	/**
	\brief	Returns type of serialized object. Returned type is an PxSerialType::Enum value.
	\return	PxSerialType::Enum of serialized object

	@see PxSerialType
	*/
	PX_INLINE	PxType						getSerialType()										const	{ return mType;														}

	static		void						getMetaData(PxSerialStream& stream);

	protected:
	PX_INLINE	void						setType(PxType t)											{ mType = t;														}
	private:
				PxType						mType;			// Some kind of class identifier. Could use a string = class name
				PxSerialFlags				mSerialFlags;	// Serialization flags
};

#define PX_DECLARE_SERIAL_RTTI(current_class, base_class)									\
virtual	const char*	getClassName()	const	{ return #current_class;	}					\
virtual	bool		isKindOf(const char* name)	const										\
					{																		\
						if(strcmp(current_class::getClassName(), name)==0) return true;		\
						else return base_class::isKindOf(name);								\
					}

#define PX_SERIAL_DYNAMIC_CAST(current_class, class_name)	(!current_class || !current_class->isKindOf(#class_name)) ? NULL : static_cast<##class_name*>(current_class)

//~PX_SERIALIZATION

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
