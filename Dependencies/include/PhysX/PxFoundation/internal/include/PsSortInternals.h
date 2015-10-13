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


#ifndef PX_FOUNDATION_PSSORTINTERNALS_H
#define PX_FOUNDATION_PSSORTINTERNALS_H

/** \addtogroup foundation
@{
*/

#include "PxAssert.h"
#include "PsUtilities.h"
#include "PsUserAllocated.h"
#include "PsIntrinsics.h"

namespace physx
{
namespace shdfnd3
{
	namespace internal
	{
		template<class T, class Predicate>
		PX_INLINE void median3(T *elements, PxI32 first, PxI32 last, Predicate &compare)
		{
			/*
			This creates sentinels because we know there is an element at the start minimum(or equal) 
			than the pivot and an element at the end greater(or equal) than the pivot. Plus the 
			median of 3 reduces the chance of degenerate behavour.
			*/

			PxI32 mid = (first + last)/2;

			if(compare(elements[mid], elements[first]))
				swap(elements[first], elements[mid]);

			if(compare(elements[last], elements[first]))
				swap(elements[first], elements[last]);

			if(compare(elements[last], elements[mid]))
				swap(elements[mid], elements[last]);

			//keep the pivot at last-1
			swap(elements[mid], elements[last-1]);
		}

		template<class T, class Predicate>
		PX_INLINE PxI32 partition(T *elements, PxI32 first, PxI32 last, Predicate &compare)
		{
			median3(elements, first, last, compare);

			/*
			WARNING: using the line:

			T partValue = elements[last-1];

			and changing the scan loops to:

			while(comparator.greater(partValue, elements[++i]));
			while(comparator.greater(elements[--j], partValue);

			triggers a compiler optimizer bug on xenon where it stores a double to the stack for partValue
			then loads it as a single...:-(
			*/

			PxI32 i = first;		//we know first is less than pivot(but i gets pre incremented) 
			PxI32 j = last - 1;		//pivot is in last-1 (but j gets pre decremented)

			while(true)
			{
				while(compare(elements[++i], elements[last-1]));
				while(compare(elements[last-1], elements[--j]));

				if(i>=j) break;

				PX_ASSERT(i<=last && j>=first);
				swap(elements[i], elements[j]);
			}
			//put the pivot in place

			PX_ASSERT(i<=last && first<=(last-1));
			swap(elements[i], elements[last-1]);

			return i;
		}

		template<class T, class Predicate>
		PX_INLINE void smallSort(T *elements, PxI32 first, PxI32 last, Predicate &compare)
		{
			//selection sort - could reduce to fsel on 360 with floats. 

			for(PxI32 i=first; i<last; i++)
			{
				PxI32 m = i;
				for(PxI32 j=i+1; j<=last; j++)
					if(compare(elements[j], elements[m])) m = j;

				swap(elements[m], elements[i]);
			}
		}


		class Stack
		{
			PxU32 mSize, mCapacity;
			PxI32 *mMemory;
			bool mRealloc;
		public:
			Stack(PxI32 *memory, PxU32 capacity): mSize(0), mCapacity(capacity), mMemory(memory), mRealloc(false) {}
			~Stack()
			{
				if(mRealloc) 
					PX_FREE(mMemory);
			}

			void grow()
			{
				mCapacity *=2;
				PxI32 *newMem = (PxI32*) PX_ALLOC(sizeof(PxI32)*mCapacity);
				memCopy(newMem,mMemory,mSize*sizeof(PxI32));
				if(mRealloc) 
					PX_FREE(mMemory);
				mRealloc = true;
				mMemory = newMem;
			}

			PX_INLINE void push(PxI32 start, PxI32 end) 
			{ 
				if(mSize >= mCapacity-1)
					grow();
				mMemory[mSize++] = start;
				mMemory[mSize++] = end;
			}

			PX_INLINE void pop(PxI32 &start, PxI32 &end)
			{
				PX_ASSERT(!empty());
				end = mMemory[--mSize];
				start = mMemory[--mSize];
			}

			PX_INLINE bool empty()
			{
				return mSize == 0;
			}
		};
	} // namespace internal

} // namespace shdfnd3
} // namespace physx

#endif
