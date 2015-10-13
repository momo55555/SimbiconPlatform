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

#include <new>
#include "CctCharacterControllerManager.h"
#include "PxController.h"
#include "CctBoxController.h"
#include "CctCapsuleController.h"
#include "PsIntrinsics.h"

using namespace physx;
using namespace Cct;

namespace
{
	// "Next Largest Power of 2
	// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
	// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
	// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
	// largest power of 2. For a 32-bit value:"
	PX_INLINE PxU32	NextPowerOfTwo(PxU32 x)
	{
		--x;
		x |= (x >> 1);
		x |= (x >> 2);
		x |= (x >> 4);
		x |= (x >> 8);
		x |= (x >> 16);
		return ++x;
	}

} // namespace

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// can't use Ps::Array becase we can only specify the mAllocator type, but not the instance
// todo: This is no longer true, Ps::Array's (stateful) allocator is now passed in by the ctor.
// But this is external API, so we can't use Ps::Arrays or Ps::Allocator. Shame.
class Cct::ControllerArray {
public:
	ControllerArray(PxAllocatorCallback* userAlloc) : mAllocator(userAlloc), count(0), capacity(0), data(NULL) {}
	~ControllerArray() { if (data) mAllocator->deallocate(data); }
	PxU32 size() const { return count; }
	Controller** begin() const { return data; }
	void pushBack(Controller* controller) 
	{
		reserve();
		data[count] = controller;
		count++;
	}
	void replaceWithLast(PxU32 index) 
	{
		PX_ASSERT(index < count && count);
		data[index] = data[count-1];
		count--;
	}
	Controller* operator[](PxU32 index) const { return data[index]; }
protected:
	PxAllocatorCallback* mAllocator;
	Controller** data;
	PxU32 count;
	PxU32 capacity;
	void reserve() 
	{
		if (count+1 > capacity) 
		{
			PxU32 oldCapacity = capacity;
			capacity = NextPowerOfTwo(capacity+1);
			Controller **newData = (Controller**)mAllocator->allocate(capacity*sizeof(Controller*), 0, __FILE__, __LINE__);
			Ps::memCopy(newData,data,oldCapacity*sizeof(Controller*));
			if(data)
				mAllocator->deallocate(data);
			data = newData;
		}
	}
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CharacterControllerManager::CharacterControllerManager(PxAllocatorCallback* userAlloc) 
: mRenderBuffer(0), mAllocator(userAlloc)
{
	mControllers = (ControllerArray*)mAllocator->allocate(sizeof(ControllerArray), 0, __FILE__, __LINE__);
	new(mControllers)ControllerArray(mAllocator);
}

CharacterControllerManager::~CharacterControllerManager()
{
	mControllers->~ControllerArray();
	mAllocator->deallocate(mControllers);
	mControllers = NULL;

	if(mRenderBuffer)
	{
		delete mRenderBuffer;
		mRenderBuffer = 0;
	}
}

void CharacterControllerManager::release() 
{
	while (getNbControllers()!= 0) releaseController(*getController(0));
	PxAllocatorCallback* a = mAllocator;
	this->~CharacterControllerManager();
	a->deallocate(this);
}

PxRenderBuffer& CharacterControllerManager::getRenderBuffer()
{
	if(!mRenderBuffer)
		mRenderBuffer = PX_NEW(Cm::RenderBuffer); 

	return *mRenderBuffer;
}

PxU32 CharacterControllerManager::getNbControllers() const { return mControllers->size(); }

Controller** CharacterControllerManager::getControllers() 
{
	return mControllers->begin();
}

PxController* CharacterControllerManager::getController(PxU32 index) 
{
	PX_ASSERT(index < mControllers->size());
	PX_ASSERT((*mControllers)[index]);
	return (*mControllers)[index]->getNxController();
}

PxController* CharacterControllerManager::createController(PxPhysics& sdk, PxScene* scene, const PxControllerDesc& desc)
{
	PX_ASSERT(desc.isValid());
	PX_ASSERT(mAllocator);
	if (!desc.isValid()) return NULL;
	if (!mAllocator) return NULL;
	Controller* newController = NULL;

	PxController* N = NULL;
	if(desc.getType()==PxControllerShapeType::eBOX)
	{
		BoxController* BC = (BoxController*)mAllocator->allocate(sizeof(BoxController), 0, __FILE__, __LINE__);
		new(BC)BoxController(desc, sdk, scene);
		newController = BC;
		N = BC;
	}
	else if(desc.getType()==PxControllerShapeType::eCAPSULE)
	{
		CapsuleController* CC = (CapsuleController*)mAllocator->allocate(sizeof(CapsuleController), 0, __FILE__, __LINE__);
		new(CC)CapsuleController(desc, sdk, scene);
		newController = CC;
		N = CC;
	}
	else PX_ASSERT(0);

	if(newController)
	{
		mControllers->pushBack(newController);
		newController->mManager = this;
	}

	return N;
}

void CharacterControllerManager::releaseController(PxController& controller)
{
	for (PxU32 i = 0; i<mControllers->size(); i++)
		if ((*mControllers)[i]->getNxController() == &controller)
		{
			mControllers->replaceWithLast(i);
			break;
		}
		if (controller.getType() == PxControllerShapeType::eCAPSULE) 
		{
			CapsuleController* cc = (CapsuleController*)&controller;
			cc->~CapsuleController();
			mAllocator->deallocate(cc);
		} 
		else if (controller.getType() == PxControllerShapeType::eBOX) 
		{
			BoxController* bc = (BoxController*)&controller;
			bc->~BoxController();
			mAllocator->deallocate(bc);
		} 
		else PX_ASSERT(0);
}

void CharacterControllerManager::purgeControllers()
{
	while(mControllers->size())
		releaseController(*(*mControllers)[0]->getNxController());
}

void CharacterControllerManager::updateControllers()
{
	for (PxU32 i = 0; i<mControllers->size(); i++)
	{
		(*mControllers)[i]->mExposedPosition = (*mControllers)[i]->mFilteredPosition;
	}
	//	printStats();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public factory methods

PX_C_EXPORT PX_PHYSX_CHARACTER_API PxControllerManager* PX_CALL_CONV PxCreateControllerManager(PxFoundation& foundation)
{
	Ps::Foundation::setInstance(static_cast<Ps::Foundation*>(&foundation));

	CharacterControllerManager* cm = (CharacterControllerManager*)foundation.getAllocator().allocate(
		sizeof(CharacterControllerManager), 0, __FILE__, __LINE__);

	if (cm)
	{
		new (cm) CharacterControllerManager(&foundation.getAllocator());
		if (cm->isValid())
			return cm;
		else
			cm->release();
	}

	return NULL;
}
