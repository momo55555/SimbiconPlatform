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


#ifndef PX_FOUNDATION_PSPOOL_H
#define PX_FOUNDATION_PSPOOL_H

#include "PsArray.h"
#include "PsSort.h"
#include "PsBasicTemplates.h"
#include "PsNoCopy.h"

namespace physx
{
namespace shdfnd3
{

	/*!
	Simple allocation pool
	*/
	template<class T, class Alloc = typename AllocatorTraits<T>::Type>
	class PoolBase : public NoCopy, public UserAllocated, private Alloc
	{
	protected:
		PoolBase (const Alloc& alloc, PxU32 elementsPerSlab, PxU32 slabSize) 
		: Alloc(alloc)
		, mSlabs(alloc)
		, mElementsPerSlab(elementsPerSlab)
		, mUsed(0)
		, mSlabSize(slabSize)
		, mFreeElement(0)
		{
			// Unallocated elements are used to store free list nodes: 
			// djs (3/26/2010): the following compile-time assert fails on PS3, hence using a runtime assert
			// for now
			// PX_COMPILE_TIME_ASSERT(sizeof(T) >= sizeof(size_t));
			PX_ASSERT(sizeof(T) >= sizeof(FreeList));
		}

	public:

		~PoolBase ()
		{
			if(mUsed)
				disposeElements();

			for(void** slabIt = mSlabs.begin(), *slabEnd = mSlabs.end(); slabIt!=slabEnd; ++slabIt)
				Alloc::deallocate(*slabIt);
		}

		// Allocate space for single object
		PX_INLINE T* allocate()
		{
			if(mFreeElement == 0)
				allocateSlab();
			T* p = reinterpret_cast<T*>(mFreeElement);
			mFreeElement = mFreeElement->mNext;
			mUsed++;
			return p;
		}

		// Put space for a single element back in the lists
		PX_INLINE void deallocate(T* p)
		{
			if(p)
			{
				mUsed--;
				push(reinterpret_cast<FreeList*>(p));
			}
		}


		PX_INLINE T* construct()
		{
			T* t = allocate();
			return t ? new (t) T() : 0;
		}

		template<class A1>
		PX_INLINE T* construct(A1& a)
		{
			T* t = allocate();
			return t ? new (t) T(a) : 0;
		}

		template<class A1, class A2>
		PX_INLINE T* construct(A1& a, A2& b)
		{
			T* t = allocate();
			return t ? new (t) T(a,b) : 0;
		}

		template<class A1, class A2, class A3>
		PX_INLINE T* construct(A1& a, A2& b, A3& c)
		{
			T* t = allocate();
			return t ? new (t) T(a,b,c) : 0;
		}

		template<class A1, class A2, class A3, class A4>
		PX_INLINE T* construct(A1& a, A2& b, A3& c, A4& d)
		{
			T* t = allocate();
			return t ? new (t) T(a,b,c,d) : 0;
		}

		template<class A1, class A2, class A3, class A4, class A5>
		PX_INLINE T* construct(A1& a, A2& b, A3& c, A4& d, A5& e)
		{
			T* t = allocate();
			return t ? new (t) T(a,b,c,d,e) : 0;
		}

		PX_INLINE void destroy(T* const p)
		{
			if(p)
			{
				p->~T();
				deallocate(p);
			}
		}

	protected:
		struct FreeList
		{
			FreeList* mNext;
		};

		// All the allocated slabs, sorted by pointer
		InlineArray<void*, 64, Alloc> mSlabs;

		PxU32 mElementsPerSlab;
		PxU32 mUsed;
		PxU32 mSlabSize;

		FreeList* mFreeElement; // Head of free-list

		// Helper function to get bitmap of allocated elements

		void push(FreeList* p)
		{
			p->mNext = mFreeElement;
			mFreeElement = p;
		}

		// Allocate a slab and segregate it into the freelist
		void allocateSlab ()
		{
			T* slab = reinterpret_cast<T*>(Alloc::allocate(mSlabSize, __FILE__, __LINE__));

			mSlabs.pushBack(slab);

			// Build a chain of nodes for the freelist
			T* it = slab + mElementsPerSlab;
			while(--it >= slab)
				push(reinterpret_cast<FreeList*>(it));
		}

		/*
		Cleanup method. Go through all active slabs and call destructor for live objects,
		then free their memory
		*/
		void disposeElements ()
		{
			Array<void*, Alloc> freeNodes;
			while(mFreeElement)
			{
				freeNodes.pushBack(mFreeElement);
				mFreeElement = mFreeElement->mNext;
			}
			sort(freeNodes.begin(), freeNodes.size(), Less<void *>());
			sort(mSlabs.begin(), mSlabs.size(), Less<void *>());

 			typename Array<void*, Alloc>::Iterator slabIt = mSlabs.begin(), slabEnd = mSlabs.end();
			for(typename Array<void*, Alloc>::Iterator freeIt = freeNodes.begin(); slabIt != slabEnd; ++slabIt)
			{
				for(T *tIt = reinterpret_cast<T*>(*slabIt), *tEnd = tIt + mElementsPerSlab; tIt != tEnd; ++tIt)
				{
					if(freeIt != freeNodes.end() && *freeIt == tIt)
						++freeIt;
					else
						tIt->~T();
				}
			}
		}
	};

	// original pool implementation
	template<class T, class Alloc = typename AllocatorTraits<T>::Type>
	class Pool : public PoolBase<T, Alloc>
	{
	public:
		Pool (const Alloc& alloc = Alloc(), PxU32 elementsPerSlab = 32):
		  PoolBase<T, Alloc>(alloc, elementsPerSlab, elementsPerSlab * sizeof(T)) 
		  {
		  }

	};


	// allows specification of the slab size instead of the occupancy
	template<class T, PxU32 slabSize, class Alloc = typename AllocatorTraits<T>::Type>
	class Pool2 : public PoolBase<T, Alloc>
	{
	public:
		Pool2 (const Alloc& alloc = Alloc()):
		  PoolBase<T, Alloc>(alloc, slabSize/sizeof(T), slabSize) 
		  {
		  }
	};


} // namespace shdfnd3
} // namespace physx


#endif
