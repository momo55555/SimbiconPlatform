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


#include "PsIntrinsics.h"
#include "GuMeshFactory.h"

#include "PxStream.h"
#include "PxHeightFieldDesc.h"

#include "NpDeformableMesh.h"

#include "GuTriangleMesh.h"
#include "GuConvexMesh.h"
#include "GuHeightField.h"
#include "DeformableMesh.h"

#if PX_SUPPORT_GPU_PHYSX
	#define	GU_MESH_FACTORY_GPU_NOTIFICATION(notificationMethod, argument) notificationMethod(argument);
#else
	#define GU_MESH_FACTORY_GPU_NOTIFICATION(notificationMethod, argument)
#endif

using namespace physx;

// PT: TODO: refactor all this with a dedicated container

GuMeshFactory::~GuMeshFactory()
{
	// Release all objects in case the user didn't do it

	while(mTriangleMeshArray.size() > 0)
	{
		// force destruction
		PX_ASSERT(mTriangleMeshArray[0]->getRefCount()==1);
		GU_MESH_FACTORY_GPU_NOTIFICATION(notifyReleaseTriangleMesh, *mTriangleMeshArray[0])
		mTriangleMeshArray[0]->release();
	}

	while(mConvexMeshArray.size() > 0)
	{
		// force destruction
		PX_ASSERT(mConvexMeshArray[0]->getRefCount()==1);
		GU_MESH_FACTORY_GPU_NOTIFICATION(notifyReleaseConvexMesh, *mConvexMeshArray[0])
		mConvexMeshArray[0]->release();
	}

	while(mHeightFieldArray.size() > 0)
	{
		// force destruction
		PX_ASSERT(mHeightFieldArray[0]->getRefCount()==1);
		GU_MESH_FACTORY_GPU_NOTIFICATION(notifyReleaseHeightField, *mHeightFieldArray[0])
		mHeightFieldArray[0]->release();
	}

#if PX_USE_DEFORMABLE_API
	while(mDeformableMeshArray.size() > 0)
	{
		// force destruction
		PX_ASSERT(mDeformableMeshArray[0]->getRefCount()==1);
		mDeformableMeshArray[0]->release();
	}	
#endif
}

///////////////////////////////////////////////////////////////////////////////

void GuMeshFactory::addTriangleMesh(Gu::TriangleMesh* np)
{
	Ps::Mutex::ScopedLock lock(mTriangleMeshMutex);

	// Keep track of triangle mesh
	if(!mTriangleMeshArray.size())
		mTriangleMeshArray.reserve(64);

	mTriangleMeshArray.pushBack(np);
}

PxTriangleMesh* GuMeshFactory::createTriangleMesh(const PxStream& desc)
{
	Gu::TriangleMesh* np = PX_NEW(Gu::TriangleMesh);
	np->setMeshFactory(this);
	if(!np)
		return NULL;

	if(!np->load(desc))
	{
		np->decRefCount();
		return NULL;
	}

	addTriangleMesh(np);
	return np;
}

bool GuMeshFactory::removeTriangleMesh(PxTriangleMesh& m)
{
	Ps::Mutex::ScopedLock lock(mTriangleMeshMutex);
	Gu::TriangleMesh* np = static_cast<Gu::TriangleMesh*>(&m);

	for(PxU32 i=0; i<mTriangleMeshArray.size(); i++)
	{
		if(mTriangleMeshArray[i]==np)
		{
			GU_MESH_FACTORY_GPU_NOTIFICATION(notifyReleaseTriangleMesh, m)
			mTriangleMeshArray.replaceWithLast(i);
			return true;
		}
	}
	return false;
}

PxU32 GuMeshFactory::getNbTriangleMeshes() const
{
	return mTriangleMeshArray.size();
}

PxU32 GuMeshFactory::getTriangleMeshes(PxTriangleMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	const PxU32 size = mTriangleMeshArray.size();

	const PxU32 writeCount = PxMin(size, bufferSize);
	for(PxU32 i=0; i<writeCount; i++)
		userBuffer[i] = mTriangleMeshArray[i+startIndex];

	return writeCount;
//	return Ps::dumpPointerArray((const void**)mTriangleMeshArray.begin(), mTriangleMeshArray.size(), (void**)userBuffer, bufferSize);
}

///////////////////////////////////////////////////////////////////////////////

void GuMeshFactory::addConvexMesh(Gu::ConvexMesh* np)
{
	Ps::Mutex::ScopedLock lock(mConvexMeshMutex);

	// Keep track of convex mesh
	if(!mConvexMeshArray.size())
		mConvexMeshArray.reserve(64);

	mConvexMeshArray.pushBack(np);
}

PxConvexMesh* GuMeshFactory::createConvexMesh(const PxStream& desc)
{
	Gu::ConvexMesh* np = PX_NEW(Gu::ConvexMesh);
	np->setMeshFactory(this);

	if(!np)
		return NULL;

	if(!np->load(desc))
	{
		np->decRefCount();
		return NULL;
	}

	addConvexMesh(np);
	return np;
}

bool GuMeshFactory::removeConvexMesh(PxConvexMesh& m)
{
	Gu::ConvexMesh* np = static_cast<Gu::ConvexMesh*>(&m);

	Ps::Mutex::ScopedLock lock(mConvexMeshMutex);
	for(PxU32 i=0; i<mConvexMeshArray.size(); i++)
	{
		if(mConvexMeshArray[i]==np)
		{
			GU_MESH_FACTORY_GPU_NOTIFICATION(notifyReleaseConvexMesh, m)
			mConvexMeshArray.replaceWithLast(i);
			return true;
		}
	}
	return false;
}

PxU32 GuMeshFactory::getNbConvexMeshes() const
{
	return mConvexMeshArray.size();
}

PxU32 GuMeshFactory::getConvexMeshes(PxConvexMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	const PxU32 size = mConvexMeshArray.size();

	const PxU32 writeCount = PxMin(size, bufferSize);
	for(PxU32 i=0; i<writeCount; i++)
		userBuffer[i] = mConvexMeshArray[i+startIndex];

	return writeCount;
//	return Ps::dumpPointerArray((const void**)mConvexMeshArray.begin(), mConvexMeshArray.size(), (void**)userBuffer, bufferSize);
}

///////////////////////////////////////////////////////////////////////////////

void GuMeshFactory::addHeightField(Gu::HeightField* np)
{
	Ps::Mutex::ScopedLock lock(mHeightFieldMutex);
	if(!mHeightFieldArray.size())
		mHeightFieldArray.reserve(64);

	mHeightFieldArray.pushBack(np);
}

PxHeightField* GuMeshFactory::createHeightField(const PxHeightFieldDesc& desc)
{
	Gu::HeightField* np = PX_NEW(Gu::HeightField)(*this);
	if(!np)
		return NULL;

	if(!np->loadFromDesc(desc))
	{
		np->decRefCount();
		return NULL;
	}

	addHeightField(np);
	return np;
}

bool GuMeshFactory::removeHeightField(PxHeightField& hf)
{
	Gu::HeightField* np = static_cast<Gu::HeightField*>(&hf);

	Ps::Mutex::ScopedLock lock(mHeightFieldMutex);

	for(PxU32 i=0; i<mHeightFieldArray.size(); i++)
	{
		if(mHeightFieldArray[i]==np)
		{
			GU_MESH_FACTORY_GPU_NOTIFICATION(notifyReleaseHeightField, hf)
			mHeightFieldArray.replaceWithLast(i);
			return true;
		}
	}
	return false;
}

PxU32 GuMeshFactory::getNbHeightFields() const
{
	return mHeightFieldArray.size();
}

PxU32 GuMeshFactory::getHeightFields(PxHeightField** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	const PxU32 size = mHeightFieldArray.size();

	const PxU32 writeCount = PxMin(size, bufferSize);
	for(PxU32 i=0; i<writeCount; i++)
		userBuffer[i] = mHeightFieldArray[i+startIndex];

	return writeCount;
//	return Ps::dumpPointerArray((const void**)mHeightFieldArray.begin(), mHeightFieldArray.size(), (void**)userBuffer, bufferSize);
}

///////////////////////////////////////////////////////////////////////////////

#if PX_USE_DEFORMABLE_API

void GuMeshFactory::addDeformableMesh(NpDeformableMesh* np)
{
	Ps::Mutex::ScopedLock lock(mDeformableMeshMutex);
	if(!mDeformableMeshArray.size())
		mDeformableMeshArray.reserve(64);

	mDeformableMeshArray.pushBack(np);
}

PxDeformableMesh* GuMeshFactory::createDeformableMesh(const PxStream& stream)
{
	NpDeformableMesh* np = PX_NEW(NpDeformableMesh)(*this);
	if(!np)
		return NULL;

	if(!np->load(stream))
	{
		np->decRefCount();
		return NULL;
	}

	addDeformableMesh(np);
	return np;
}

bool GuMeshFactory::removeDeformableMesh(PxDeformableMesh& deformableMesh)
{
	NpDeformableMesh* npDeformableMesh = &static_cast<NpDeformableMesh&>(deformableMesh);

	Ps::Mutex::ScopedLock lock(mDeformableMeshMutex);

	// remove the deformable mesh from the deformable mesh array
	for(PxU32 i=0; i<mDeformableMeshArray.size(); i++)
	{
		if(mDeformableMeshArray[i]==npDeformableMesh)
		{
			mDeformableMeshArray.replaceWithLast(i);
			return true;
		}
	}
	return false;
}

PxU32 GuMeshFactory::getNbDeformableMeshes() const
{
	return mDeformableMeshArray.size();
}

PxU32 GuMeshFactory::getDeformableMeshes(PxDeformableMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	const PxU32 size = mDeformableMeshArray.size();

	const PxU32 writeCount = PxMin(size, bufferSize);
	for(PxU32 i=0; i<writeCount; i++)
		userBuffer[i] = mDeformableMeshArray[i+startIndex];

	return writeCount;
//	return Ps::dumpPointerArray((const void**)mDeformableMeshArray.begin(), mDeformableMeshArray.size(), (void**)userBuffer, bufferSize);
}

#endif  // PX_USE_DEFORMABLE_API

///////////////////////////////////////////////////////////////////////////////
