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

#ifndef PX_PHYSICS_COMMON_SERIAL_FRAMEWORK
#define PX_PHYSICS_COMMON_SERIAL_FRAMEWORK

#include "Px.h"

// PX_SERIALIZATION

#include "PxSerialFramework.h"
#include "CmPhysXCommon.h"
#include "CmReflection.h"
#include "PsUserAllocated.h"
#include "PsHashMap.h"

namespace physx
{
namespace Cm
{
	static const PxEmpty& PX_EMPTY = *reinterpret_cast<const PxEmpty*>(size_t(0xDEADD00D));

	typedef Ps::HashMap<PxSerializable*, PxSerializable*> HashMapResolver;
	class RefResolver : public PxRefResolver, public Ps::UserAllocated
	{
		public:
								RefResolver() : mStringTable(NULL)	{}
		virtual	PxSerializable* newAddress(PxSerializable* oldAddress) const;
		virtual	void			setNewAddress(PxSerializable* oldAddress, PxSerializable* newAddress);
		virtual	void			setStringTable(const char*);
		virtual	const char*		resolveName(const char*);

				HashMapResolver	mResolver;
				const char*		mStringTable;
	};

	typedef Ps::HashMap<void*, PxSerializable*> UserHashMapResolver;
	class UserReferences : public PxUserReferences, public Ps::UserAllocated
	{
		public:
		virtual	PxSerializable*	getObjectFromID(void* userData) const;
		virtual	void			setUserData(PxSerializable* object, void* userData);

		UserHashMapResolver	mResolver;
	};

	class InternalCollection : public PxCollection
	{
		public:

		// Only for internal use. Bypasses virtual calls, specialized behaviour.
		PX_INLINE	void				internalAdd(PxSerializable* s)		{ mArray.pushBack(s);								}
		PX_INLINE	PxU32				internalGetNbObjects()		const	{ return mArray.size();								}
		PX_INLINE	PxSerializable*		internalGetObject(PxU32 i)	const	{ PX_ASSERT(i<mArray.size());	return mArray[i];	}
		PX_INLINE	PxSerializable**	internalGetObjects()				{ return &mArray[0];								}

		Ps::Array<PxSerializable*>		mArray;
	};

	void	serializeCollection(InternalCollection& collection, PxSerialStream& stream, bool exportNames);
	bool	deserializeCollection(InternalCollection& collection, RefResolver& Ref, void* buffer);

	bool					registerClass(PxType type, PxClassCreationCallback callback);
	PxSerializable*			createClass(PxType type, char*& address, PxRefResolver& v);

	/**
	Any object deriving from PxSerializable needs to call this function instead of 'delete object;'. 

	We don't want implement 'operator delete' in PxSerializable because that would impose how
	memory of derived classes is allocated. Even though most or all of the time derived classes will 
	be user allocated, we don't want to put UserAllocatable into the API and derive from that.
	*/
	PX_INLINE void deleteSerializedObject(PxSerializable* object)
	{
		if(object->ownsMemory())
			PX_DELETE(object);
		else
			object->~PxSerializable();
	}

	/*void	exportArray(PxSerialStream& stream, const void* data, PxU32 size, PxU32 sizeOfElement, PxU32 capacity);
	char*	importArray(char* address, void** data, PxU32 size, PxU32 sizeOfElement, PxU32 capacity);
	void	notifyResizeDeserializedArray();*/


} // namespace Cm

}

//~PX_SERIALIZATION

#endif
//#endif