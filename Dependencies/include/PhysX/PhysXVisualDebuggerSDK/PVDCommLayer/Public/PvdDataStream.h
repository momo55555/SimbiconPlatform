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

#ifndef PVD_PVD_DATA_STREAM_H
#define PVD_PVD_DATA_STREAM_H

#include "PVDCommLayerTypes.h"
#include "PVDDebuggerMessages.h"
#include "PVDQueryInterface.h"
#include "PvdRender.h"


namespace PVD //PhysX Visual Debugger
{

	class PvdPropertyDefinitionHelper;
	class PvdBeginPropertyBlockHelper;
	class PvdSendPropertyBlockHelper;

	/**
	 *	Pvd relies on a typed communication stream to transmit data.
	 *	This means that you create a bunch of types (createClass, defineProperty)
	 *	and then send updates to objects of those types.
	 *
	 *	A data stream represents a buffered stream of data to pvd.  Multiple streams
	 *	can be sent from one connection.  Each stream is buffered but not
	 *	threadsafe.  PhysX uses a single data stream per scene and thus the per-scene
	 *	update can happen wherever fetchResults is called from.
	 *	
	 *	Represents a stream to the debugger.  The debugger has limited
	 *	two way communication abilities.  All functions could return
	 *	network error; this is not stated in the documentation as it would
	 *	be redundant.  Similarly, a BlockOpen error can be generated
	 *	for all calls other than the block data sending functions
	 *	in between Begin* and End* calls for the appropriate block.
	 *
	 *	Unless stated otherwise, strings only need to last the duration
	 *	of the function call.  Strings used in the connection's meta-data
	 *	system must stay around for the duration of the connection.
	 *	
	 *	Clients identify the keys they would like to use for class types.
	 *	The system stores properties based in index.
	 */
	class PvdDataStream : public PvdQueryInterface, public PvdRender, public PvdRefCounted
	{
	protected:
		virtual ~PvdDataStream(){}

	public:
		
		/**
		 *	Set the namespace that classes created into.
		 */
		virtual PvdCommLayerError setNamespace( const char* inNamespace ) = 0;

		/**
		 *	Push the current namespace onto the stack.  Does not change 
		 *	current namespace.  The namespace stack is connection
		 *	dependent.
		 *	
		 *	Errors:
		 *	StackOverflow - Namespace depth exceeded stack depth (255).
		 */
		virtual PvdCommLayerError pushNamespace() = 0;

		/**
		 *	Set the current namespace to the top of the stack and pop
		 *	the top of the stack.
		 *	
		 *	Errors: 
		 *	StackUnderflow - There were no namespaces on the stack.
		 */
		virtual PvdCommLayerError popNamespace() = 0;
		/////////////////////////////////////////////////////////////
		// Sending information to the debugger
		// All of these may pass back errors coming from the underlying
		// stream, so you may always get a network error.
		/////////////////////////////////////////////////////////////

		/**
		 *	create a class given a name and a key.  If a new class is created
		 *	return true.  If this class name is already bound to this key 
		 *	return false.  inName *must* be persistent; i.e. it must
		 *	stay around for the duration of the connection.
		 *
		 *	inName		- Name of the class you are creating
		 *	inKey		- The key you want to refer to this class by.
		 *					Must not be zero.
		 *	outCreated	- True if a new class was created, false otherwise.
		 *
		 *	Errors:
		 *  NameBoundToDifferentKey	- inName is already bound and inKey != boundKey. 
		 *	InvalidKeyValue			- inKey was zero.
		 *	InvalidName				- inName was NULL;
		 */
		virtual PvdCommLayerError createClass( const char* inName, PxU32 inKey ) = 0;
		/**
		 *	Derive a class from another class.  The derived class will have all of the properties
		 *	of the parent followed by any properties that it has itself.  If the child is already
		 *	derived from the parent this class is ignored.  You should be finished
		 *	adding properties to the parent class before calling derive class.  Deriving
		 *	a class from another class locks the parent class and any further calls to
		 *	define property (or derive class) will return ClassLocked.
		 *
		 *	outDerived is true if the derivation already took place.
		 *	
		 *	Errors:
		 *	ParentDerivedFromChild			- the parent is already derived from the child.
		 *	ChildDerivedFromDifferentParent - child.baseClass.exists() &&  child.baseClass != inParentKey.
		 *	InvalidClass					- either inParentKey or inChildKey don't refer to a defined class.
		 *	InvalidKey						- either inParentKey or inChildKey are zero.
		 *	PropertyKeyCollision			- Property keys in the derived class collide with property keys
		 *										in the base class.
		 *	ClassLocked						- The child class has already been locked and thus cannot
		 *									- derive from anything.
		 */
		virtual PvdCommLayerError deriveClass( PxU32 inParentKey, PxU32 inChildKey) = 0;
		/**
		 *	Define a property.  If the property is already defined exactly as stated
		 *	(datatype and all metadata matches) this call is ignored.  InKey is used
		 *	to refer to the property in later calls; it does not have to be globally
		 *	unique but must be unique to inClass.
		 *	
		 *	inSemantic may be null if unused.
		 *	inKey is the key used to refer to this property later.  Must be unique
		 *	outCreated is true when the property has been created already.
		 *
		 *	inName, inSemantic must be persistent meaning they must stay around for
		 *		duration of the connection.
		 *
		 *	
		 *	Errors:
		 *	PropertyDefinitionError		- This property exists but with different meta data.
		 *	PropertyKeyCollision		- This property exists on a parent or grandparent.
		 *	InvalidClass				- inClass doesn't point to a valid class.
		 *	InvalidKey					- inKey is zero.
		 *	InvalidName					- inName is NULL.
		 *	ClassLocked					- the parent is locked, perhaps due to derivation.
		 *	InvalidDatatype				- If the datatype >= PvdCommLayerDatatype::last 
											or datatype == PvdCommLayerDatatype::Unknown
		 */
		virtual PvdCommLayerError defineProperty( PxU32 inClass
													, const char* inName
													, const char* inSemantic
													, PvdCommLayerDatatype inDatatype
													, PxU32 inKey ) = 0;


		/** 
		 *	Similar to defineProperty, this version takes an instance handle instead
		 *	of a class handle.  The receiving side simply looks up the class of the
		 *	instance and calls define property.
		 *
		 *	See defineProperty.
		 *	
		 *	Additional Errors:
		 *	InvalidInstance
		 */
		virtual PvdCommLayerError definePropertyOnInstance( PxU64 inInstanceId
															, const char* inName
															, const char* inSemantic
															, PvdCommLayerDatatype inDatatype
															, PxU32 inKey) = 0;

		
		/**
		 *	Defines a property struct, which is a collection of properties at various offsets in memory.
		 *	This is the most efficient way to send a set of properties across the wire, the sender
		 *	controls the layout of the data.  Strings are assumed to be const char* values and are sent
		 *	directly after the data struct.  The entries must contain raw datatypes, i.e u32 u8, etc.  They
		 *	can't contain a datatype that can have a variable sized native representation like bitflags
		 *	or enumerations, they must contain a datatype that represents the actual byte size of the data
		 *	as it exists in the struct.  Pointers other than strings need to be marked the datatype
		 *	PvdCommLayerDatatype::Pointer and will be assumed to be instance ids.
		 *	
		 *	PropertyKeyCollision		- This struct key is already taken in the current connection namespace.
		 *	InvalidKey					- One of these properties doesn't exist.
		 *	InvalidClass				- The class doesn't exist.
		 *	InvalidDatatype				- A datatype doesn't roughly match a property.
		 *	InvalidArguments			- The struct byte size can't fit the furthest property entry + datatype size.
		 */
		virtual PvdCommLayerError definePropertyStruct( PxU32 inStructKey
															, PxU32 inClass
															, PxU32 inStructByteSize
															, const PropertyStructEntry* inEntries
															, PxU32 inEntryCount ) = 0;

		
		/**
		 *	Define an array property.  There are another class of properties
		 *	where the data in the array is defined by a user-created class.
		 *	
		 *	Users must use beginArrayPropertyBlock,sendArrayObject(s),endArrayPropertyBlock
		 *	to change the data in the array.  setProperty doesn't work.
		 */
		virtual PvdCommLayerError defineArrayProperty( PxU32 inClass
														, const char* inName
														, PxU32 inArrayClass
														, PxU32 inKey ) = 0;
		/**
		 *	Define names for Bitflag values.  This is optional but is incredibly useful
		 *	when debugging values as it allows the UI to display in a meaningful way
		 *	exactly which bits are high.  Later calls with same class and property
		 *	are ignored.  The property does not have to exist on the class and
		 *	no typechecking is performed.
		 *	
		 *	Errors:
		 *	InvalidClass			- inClass doesn't point to a valid class.
		 */
		virtual PvdCommLayerError defineBitflagNames( PxU32 inClass
														, PxU32 inPropertyKey
														, const NamedValueDefinition* inDefinitions
														, PxU32 inDefinitionLength ) = 0;

		/**
		 *	Define names for enumeration values.  This is optional but is incredibly useful
		 *	when debugging values as it allows the UI to display in a meaningful way
		 *	the enumeration name.  Later calls with same class and property
		 *	are ignored.  The property does not have to exist on the class and
		 *	no typechecking is performed.
		 *	
		 *	Errors:
		 *	InvalidClass			- inClass doesn't point to a valid class.
		 */
		virtual PvdCommLayerError defineEnumerationNames( PxU32 inClass
															, PxU32 inPropertyKey
															, const NamedValueDefinition* inDefinitions
															, PxU32 inDefinitionLength ) = 0;

		/**
		 *	create an instance of a given class.  
		 *	
		 *	Errors: 
		 *	InvalidKey			- inClass or inInstanceId is zero.
		 *	InvalidClass		- inClass doesn't point to a valid class.
		 *	InstanceExists		- Instance already exists and hasn't been destroyed.
		 */
		virtual PvdCommLayerError createInstance( PxU32 inClass, PxU64 inInstanceId, EInstanceUIFlags inFlags = EInstanceUIFlags::None ) = 0;

		/**
		 *	Set a property value on an instance.
		 *
		 *	Errors:
		 *	InvalidKey			- inInstance or inProperty are zero.
		 *	InvalidInstance		- inInstance doesn't point to a live instance
		 *	InvalidProperty		- inProperty doesn't point to a valid property on the instance's class
		 *	DatatypeMismatch	- inValue.getDatatype isn't the same as the property's datatype 
		 */
		virtual PvdCommLayerError setPropertyValue( PxU64 inInstance, PxU32 inProperty, const PvdCommLayerValue& inValue ) = 0;

		/** 
		 *	Begin sending a block of property values.  Properties must exist on all 
		 *	the instances indicated below.
		 *	
		 *	Errors:
		 *	InvalidKey			- inClass or one of inProperties is zero.
		 *	InvalidClass		- The class wasn't found
		 *	InvalidProperty		- One of the properties didn't exist on the class.
		 */
		virtual PvdCommLayerError beginPropertyBlock( PxU32 inClass, const PxU32* inProperties, const PvdCommLayerDatatype* inDatatypes, PxU32 inPropertyCount ) = 0;

		/**
		 *	Send an set of properties for a given instance.  The values are typechecked against
		 *	the properties setup in beginPropertyBlock and inInstance is checked to contain
		 *	every property setup in beginPropertyBlock.
		 *	
		 *	Errors:
		 *	InvalidInstance			- inInstance isn't a valid instance.
		 *	InstanceTypeMismatch	- inInstance isn't (or derived from) type inClass.
		 *	DatatypeMismatch		- One of the layer values doesn't match the type of one of the properties
		 *	InvalidContext			- A property block isn't open (call beginPropertyBlock).
		 *	InvalidBlockType		- This isn't a property block.
		 */
		virtual PvdCommLayerError sendPropertyBlock( PxU64 inInstance, const PvdCommLayerValue* inValues ) = 0;
		
		/**
		 *	Send the block of properties as opaque data.  Much faster but you better have sent exactly
		 *	what you said you would in beginPropertyBlock.  Each property is expected to be adjacent
		 *	to the last one, no padding or anything.
		 *  inValues may be null, but if it passed in it is used as a size checker to ensure the binary
		 *  data is exactly the same size as the larger comm value based data.
		 */
		virtual PvdCommLayerError sendPropertyBlock( PxU64 inInstance, const PxU8* inData, PxU32 inDataLen, const PvdCommLayerValue* inValues = NULL ) = 0;
		
		/**
		 *	End the property block.  Further calls to sendPropertyBlock will generate
		 *	an InvalidContext error.
		 *
		 *	Errors:
		 *	InvalidContext		- A property block isn't open.	
		 *	InvalidBlockType	- This isn't a property block.
		 */
		virtual PvdCommLayerError endPropertyBlock() = 0;
		
		/**
		 *	Send a property struct.  The data len *must* match the byte length sent when the struct was registered.
		 *	const char* ptrs in the struct are sent immediately after the struct.  When this messages
		 *  is received, the struct pointers contain an offset from the beginning of the struct where the string
		 *  begins.  So, going into the function we just write out the strings after.  On the receiving side
		 *	we store an offset in the structure that indicates where the string sits past the beginning of the struct.
		 *	The reason is that we can't store a global 64 bit address in a pointer sent from a 32 bit system.
		 *
		 *	Errors:
		 *	InvalidInstance			- inInstance isn't a valid instance.
		 *	InstanceTypeMismatch	- inInstance isn't (or derived from) type the class registered for inStructKey.
		 *	InvalidArguments		- inDataLen != inStructKey.byteLength || inData == NULL;
		 */
		virtual PvdCommLayerError sendPropertyStruct( PxU64 inInstance, PxU32 inStructKey, const PxU8* inData, PxU32 inDataLen ) = 0;

		/**
		 *	Add a child to an instance.
		 *
		 *	Errors:
		 *	InvalidInstance			- inParent or inChild isn't a valid instance.
		 *	InstanceTypeMismatch	- inParent or inChild is a Array instance.
		 *	ChildError				- parent already has this child.
		 */
		virtual PvdCommLayerError addChild( PxU64 inParent, PxU64 inChild ) = 0;
		
		/**
		 *	Remove a child from an instance.
		 *
		 *	Errors:
		 *	InvalidInstance			- inParent or inChild isn't a valid instance.
		 *	InstanceTypeMismatch	- inParent or inChild is a Array instance.
		 *	ChildError				- parent does not have this child.
		 */
		virtual PvdCommLayerError removeChild( PxU64 inParent, PxU64 inChild ) = 0;
		
		/**
		 *	Remove all children from a given instance.
		 *	If inChildClass is nonzero, this will remove all children
		 *	that match a given class.
		 */
		virtual PvdCommLayerError removeAllChildren( PxU64 inInstanceId, PxU32 inChildClass ) = 0;
		
		/**
		 *	destroy an instance.  This can destroy both a normal instance
		 *	and a Array or array instance.
		 *	Errors:
		 *	InvalidInstance			- inInstance isn't a valid instance.
		 */
		virtual PvdCommLayerError destroyInstance( PxU64 inInstance ) = 0;


		/**
		 *	Begin a block of Array data.  The instance ID indicates
		 *	a single id to assign to the entire block.  It is fine to call this
		 *	with an instance that doesn't exist.  If the instance does exist,
		 *	it must be of type inClass and it must be a Array instance.
		 *
		 *	inProperties specifies which properties to send along with
		 *	the order they are sent in.  
		 *	
		 *	Errors:
		 *	InvalidKey				- inClass, inInstance, or one of inProperties is zero.
		 *	InvalidClass			- inClass isn't a valid class.
		 *	InvalidProperty			- One of inProperties doesn't exist on the object.
		 *	InstanceClassMismatch	- Instance exists but is the wrong class.
		 *	InstanceTypeMismatch	- Instance exists but is not a Array instance.
		 *	BlockOpen				- A Array block is already open. 
		 */
		virtual PvdCommLayerError beginArrayBlock( PxU32 inClass, PxU64 inInstance, const PxU32* inProperties, const PvdCommLayerDatatype* inDatatypes, PxU32 inNumProps ) = 0;

		/** 
		 *	Begin an array property block.  Used to update the data in an array property.
		 */
		virtual PvdCommLayerError beginArrayPropertyBlock( PxU64 inInstance
														, PxU32 inProperty
														, const PxU32* inProperties
														, const PvdCommLayerDatatype* inDatatypes
														, PxU32 inPropertyCount ) = 0;
		/**
		 *	Send a Array object.  The object is assumed to be the datatype
		 *	used in inClass for beginArrayBlock and inValues are the values
		 *	of the properties to set.
		 *	
		 *	Errors:
		 *	DatatypeMismatch	- One of inValues isn't the same as the previously 
		 *							defined property's datatype.
		 *	InvalidContext		- A Array block isn't open.
		 */
		virtual PvdCommLayerError sendArrayObject( const PvdCommLayerValue* inValues ) = 0;

		/**
		 *	Send a set of array objects.  The data is read from directly, it is expected to be
		 *	the data itself and not the CommLayerValue larger datatypes.
		 *
		 *	This interface method doesn't work for variable size (string, buffer) datatypes.
		 *
		 *	inStride must be 0, meaning use the native size of the datatypes or greater
		 *	than or equal to the native size of the datatypes specified in BeginArrayBlock.
		 *	The stride will be added to the data pointer every iteration if it is specified.
		 *	
		 *	For errors, see sendArrayObject.
		 *	Extended Errors:
		 *	InvalidData : inCount != 0 but inData == NULL;
		 */
		virtual PvdCommLayerError sendArrayObjects( const PxU8* inData, PxU32 inStride, PxU32 inCount ) = 0;

		/**
		 *	End block of Array data.
		 *	
		 *	Errors:
		 *	InvalidContext		- A Array block wasn't open.
		 *	InvalidBlockType	- A block was open but it wasn't a Array block.
		 */
		virtual PvdCommLayerError endArrayBlock() = 0;

		/** 
		 *	End an array property block.
		 */
		virtual PvdCommLayerError endArrayPropertyBlock() = 0;

		/**
		 *	Begin a named section.  Please don't use the reserved
		 *	name "frame" as it is reserved for frame boundaries.
		 *	
		 *	inName must be a valid string for the lifetime of the section.
		 *
		 *	Errors:
		 *	InvalidName			- inName was null
		 */
		virtual PvdCommLayerError beginSection( const char* inName ) = 0;

		/**
		 *	End a named section.  Must correspond to the top-most
		 *	open section.
		 *
		 *	inName needs only to be valid until the return of the function
		 *	call.
		 *
		 *	Errors:
		 *	InvalidName			- inName was null
		 *	NoOpenSection		- There are currently no open sections.
		 *	SectionNameMismatch	- The section this end section corresponds with
		 *							has a different name.
		 */
		virtual PvdCommLayerError endSection(const char* inName) = 0;

		/**
		 *	Wraps the begin/end section calls using the reserved keyword
		 *	"frame".
		 */	
		virtual PvdCommLayerError beginFrame() = 0;

		/**
		 *	Wraps the begin/end section calls using the reserved keyword
		 *	"frame".  See the errors corresponding with endSection.
		 */
		virtual PvdCommLayerError endFrame() = 0;

		/**
		 *	Send a named event and associate it with an instance.  This is useful internally to PhysX
		 *	and APEX.  It is unlikely to be useful to an outside developer (but maybe...).
		 *	
		 *	Errors - InvalidInstance - Instance isn't a recognized instance.
		 */
		virtual PvdCommLayerError namedEventWithInstance( PxU64 inInstanceId, const char* inName ) = 0;
		
		/**
		 *	Associate a frame marker with a given instance.  Useful when different objects may be updated at different rates
		 *	per frame (i.e. different PhysX scenes and APEX scenes used in the same game and sharing the same connection).
		 *	Uses the reserved keyword "frame" for the named event.
		 *	
		 *	Errors - See namedEventWithInstance
		 */
		virtual PvdCommLayerError sendFrameMarkerWithInstance( PxU64 inInstanceId ) = 0;

		

		/**
		 *	Send an error message to be stored in the database.
		 */
		virtual PvdCommLayerError sendError( const char* inType, const char* inMessage, const char* inFile, PxU32 inLine ) = 0;
		
		/**
		 *	Send a warning message to be stored in the database.
		 */
		virtual PvdCommLayerError sendWarning( const char* inType, const char* inMessage, const char* inFile, PxU32 inLine ) = 0;

		/**
		 *	flush the output stream sending any open blocks.
		 */
		virtual PvdCommLayerError flush() = 0;

		/**
		 *	Flush the buffer, but don't force a socket flush.  
		 *	When multiple connections are sitting on a socket buffer
		 *	it is important sometimes to ensure one connection doesn't
		 *	buffer certain data.  This way you ensure the buffer is
		 *	empty but the overall socket connection is still free to
		 *	buffer information.
		 */
		virtual PvdCommLayerError localFlush() = 0;
		
		/////////////////////////////////////////////////////////////
		// Lifetime management and diagnostics.
		/////////////////////////////////////////////////////////////
		/**
		 *	Returns true if this interface is connected.
		 */
		virtual bool isConnected() const = 0;

		virtual PvdPropertyDefinitionHelper& getPropertyDefinitionHelper() = 0;
		virtual PvdBeginPropertyBlockHelper& getBeginPropertyBlockHelper() = 0;
		virtual PvdSendPropertyBlockHelper& getSendPropertyBlockHelper() = 0;
	};
}

#endif