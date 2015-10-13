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


#ifndef PX_REFLECTION_H
#define PX_REFLECTION_H

#include "PsArray.h"

// PX_SERIALIZATION

// A stripped-down version of ICE reflection stuff

#ifdef __CELLOS_LV2__
#ifdef __SNC__
      #define OFFSET_OF(Class, Member)    (size_t)&(((Class*)0)->Member)
#else // __SNC__
      #define OFFSET_OF(Class, Member)    __builtin_offsetof(Class, Member)
#endif // __SNC__
#else
#if defined LINUX || defined __APPLE__
      #define OFFSET_OF(Class, Member)    __builtin_offsetof(Class, Member)
#else // LINUX
      #define OFFSET_OF(Class, Member)    (size_t)&(((Class*)0)->Member)
#endif // LINIX
#endif
	#define ICE_ARRAYSIZE(p)				(sizeof(p)/sizeof(p[0]))
	#define SIZE_OF(Class, Member)			sizeof(((Class*)0)->Member)
	#define ICE_COMPILE_TIME_ASSERT(exp)	extern char ICE_Dummy[ (exp) ? 1 : -1 ]

#include "PxFields.h"

namespace physx
{
namespace shdfnd3
{

	enum FieldFlag
	{
		F_SERIALIZE		= (1<<0),	//!< Serialize this field
		F_ALIGN			= (1<<1),	//!< Align this serialized field on 16-bytes boundary
	};

	//! Defines a generic field
	#define _FIELD(type, name, fieldtype, count, offset_size, flags)	{ fieldtype, #name, (PxU32)OFFSET_OF(type, name), SIZE_OF(type, name), count, offset_size, flags }

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//! Defines a single base field
	#define DEFINE_FIELD(type, name, fieldtype, flags)						_FIELD(type, name, fieldtype, 1, 0, flags)
	//! Defines a static array of base fields
	#define DEFINE_STATIC_ARRAY(type, name, fieldtype, count, flags)		_FIELD(type, name, fieldtype, count, 0, flags)
	//! Defines a dynamic array of base fields
	#define DEFINE_DYNAMIC_ARRAY(type, name, fieldtype, name_size, flags)	_FIELD(type, name, fieldtype, 0, OFFSET_OF(type, name_size), flags)

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//! Starts field declarations
	#define BEGIN_FIELDS(current_class)		physx::shdfnd3::FieldDescriptor current_class::mClassDescriptor[] = {	\
											{ PxField::eFORCE_DWORD, #current_class, 0, 0, 0, 0, 0 },
	//! Ends field declarations
	#define END_FIELDS(current_class)		};	PxU32 current_class::getDescriptorSize()	const { return ICE_ARRAYSIZE(mClassDescriptor);	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//! Declares a static class descriptor for the class. (a.k.a. meta-data)
	//! You can use this on all classes, not only Cell-based ones. This macro must be used in the header files,
	//! then fields must be declared in the cpp file using BEGIN_FIELDS / DEFINE_FIELD / END_FIELDS macros.
	#define DECLARE_FIELDS																			\
																									\
	static physx::shdfnd3::FieldDescriptor		mClassDescriptor[];									\
																									\
	/* Gets the number of defined fields. There's always at least one for the class itself	*/		\
	/* This is ONLY for current class, not base classes. So NO VIRTUAL there.				*/		\
	/*virtual*/ PxU32 getDescriptorSize()	const;													\
	/* Gets field descriptors */																	\
	const physx::shdfnd3::FieldDescriptor* getDescriptor()	const	{ return mClassDescriptor;	}	\
	/* Finds a field given its name */																\
	const physx::shdfnd3::FieldDescriptor* findDescriptor(const char* name)	const					\
	{																								\
		if(!name)	return NULL;																	\
		const PxU32 Size = getDescriptorSize();														\
		for(PxU32 i=0;i<Size;i++)																	\
		{																							\
			const physx::shdfnd3::FieldDescriptor* FD = &mClassDescriptor[i];						\
			if(strcmp(FD->mName, name)==0)	return FD;												\
		}																							\
		return NULL;																				\
		}

	#define DECLARE_SERIAL_CLASS(current_class, base_class)											\
	DECLARE_FIELDS																					\
	public:																							\
	virtual	PxU32		getObjectSize()	const	{ return sizeof(*this);		}						\
																									\
	virtual bool	getFields(physx::shdfnd3::FieldDescriptors& edit, PxU32 flags)	const			\
	{																								\
		base_class::getFields(edit, flags);															\
		const physx::shdfnd3::FieldDescriptor* Fields = current_class::getDescriptor();				\
		for(PxU32 i=0;i<current_class::getDescriptorSize();i++)										\
		{																							\
			if(Fields[i].mFlags&flags)	edit.pushBack(&Fields[i]);									\
		}																							\
		return true;																				\
	}																								\
																									\
	virtual bool getFields(physx::shdfnd3::FieldDescriptors& edit, PxField::Enum type)	const		\
	{																								\
		base_class::getFields(edit, type);															\
		const physx::shdfnd3::FieldDescriptor* Fields = current_class::getDescriptor();				\
		for(PxU32 i=0;i<current_class::getDescriptorSize();i++)										\
		{																							\
			if(Fields[i].mType==type)	edit.pushBack(&Fields[i]);									\
		}																							\
		return true;																				\
	}																								\
																									\
	virtual bool getFields(physx::shdfnd3::FieldDescriptors& edit)	const							\
	{																								\
		base_class::getFields(edit);																\
		const physx::shdfnd3::FieldDescriptor* Fields = current_class::getDescriptor();				\
		for(PxU32 i=0;i<current_class::getDescriptorSize();i++)	edit.pushBack(&Fields[i]);			\
		return true;																				\
	}																								\
																									\
	virtual const physx::shdfnd3::FieldDescriptor* getFieldDescriptor(const char* name)	const		\
	{																								\
		if(!name)	return NULL;																	\
		const physx::shdfnd3::FieldDescriptor* FD = base_class::getFieldDescriptor(name);			\
		if(FD)	return FD;																			\
																									\
		return findDescriptor(name);																\
	}																								\
	static PxSerializable* createInstance(char*& address, PxRefResolver& v)							\
	{																								\
		current_class* NewObject = new (address) current_class(v);									\
		address += sizeof(*NewObject);																\
		return NewObject;																			\
	}

	//! A field descriptor
	struct FieldDescriptor
	{
		// Compulsory values
						PxField::Enum	mType;			//!< Field type (bool, byte, quaternion, etc)
						const char*		mName;			//!< Field name (appears exactly as in the source file)
						PxU32			mOffset;		//!< Offset from the start of the class (ie from "this", field is located at "this"+Offset)
						PxU32			mSize;			//!< sizeof(Type)
						PxU32			mCount;			//!< Number of items of type Type (0 for dynamic sizes)
						PxU32			mOffsetSize;	//!< Offset of dynamic size param, for dynamic arrays
						PxU32			mFlags;			//!< Field parameters
		// Generic methods
						PxU32			FieldSize()								const;
		PX_FORCE_INLINE	void*			Address(void* class_ptr)				const	{ return (void*)(size_t(class_ptr) + mOffset);			}

		PX_FORCE_INLINE	void*			GetArrayAddress(void* class_ptr)		const	{ return *(void**)Address(class_ptr);					}
		PX_FORCE_INLINE	PxU32			IsStaticArray()							const	{ return mCount;										}
		PX_FORCE_INLINE	PxU32			GetStaticArraySize()					const	{ return mCount;										}
		PX_FORCE_INLINE	PxU32			IsDynamicArray()						const	{ return mOffsetSize;									}
		PX_FORCE_INLINE	PxU32			GetDynamicArraySize(void* class_ptr)	const	{ return *(PxU32*)(size_t(class_ptr) + mOffsetSize);	}
	};

	// would like to use a typedef, but need to forward declare in PxSerialFramework
	class FieldDescriptors : public Array<const FieldDescriptor*>
	{
		typedef AllocatorTraits<const FieldDescriptor*>::Type Alloc;
	public:
		FieldDescriptors(PxRefResolver& v) : Array<const FieldDescriptor*>(v) {} 
		explicit FieldDescriptors(const Alloc& alloc = Alloc()) : Array<const FieldDescriptor*>(alloc) {} 
	};

	//~PX_SERIALIZATION

} // namespace shdfnd3
} // namespace physx

#endif