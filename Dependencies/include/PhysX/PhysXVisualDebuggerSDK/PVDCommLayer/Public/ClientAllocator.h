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

#ifndef PVD_CLIENTALLOCATOR_H
#define PVD_CLIENTALLOCATOR_H
#include <limits>

namespace PVD
{
	//Taken from:http://www.codeproject.com/KB/cpp/allocator.aspx
	template<typename T>
	class ClientAllocator {
	public : 
		//    typedefs

		typedef T value_type;
		typedef value_type* pointer;
		typedef const value_type* const_pointer;
		typedef value_type& reference;
		typedef const value_type& const_reference;
		typedef std::size_t size_type;
		typedef std::ptrdiff_t difference_type;

	public : 
	static const char* getName()
	{
#if defined PX_GNUC
		return __PRETTY_FUNCTION__;
#else
		return typeid(T).name();
#endif
	}
		//    convert an allocator<T> to allocator<U>

		template<typename U>
		struct rebind {
			typedef ClientAllocator<U> other;
		};

	public : 
		inline explicit ClientAllocator() {}
		inline explicit ClientAllocator(const char*) {}
		inline ~ClientAllocator() {}
		inline ClientAllocator(ClientAllocator<T> const&) {}
		template<typename U>
		inline ClientAllocator(ClientAllocator<U> const&) {}

		//    address

		inline pointer address(reference r) { return &r; }
		inline const_pointer address(const_reference r) { return &r; }

		//    memory allocation

		/*
		inline pointer allocate(size_type cnt, 
		   typename std::allocator<void>::const_pointer = 0) { 
		  return reinterpret_cast<pointer>(malloc(cnt*sizeof(value_type))); 
		}*/
		inline pointer allocate( size_type byte_size, const char* /*file*/, int /*line*/ ) {
			return reinterpret_cast<pointer>(malloc(byte_size));
		}
		inline void deallocate(pointer p, size_type) { 
			free(p);
		}
		inline void deallocate(void* p) { 
			free(p);
		}

		//    size

		inline size_type max_size() const { 
			return std::numeric_limits<size_type>::max() / sizeof(T);
	 }

		//    construction/destruction

		inline void construct(pointer p, const T& t) { new(p) T(t); }
		inline void destroy(pointer p) { p->~T(); }

		inline bool operator==(ClientAllocator const&) { return true; }
		inline bool operator!=(ClientAllocator const& a) { return !operator==(a); }
	};    //    end of class Allocator 
}

#endif