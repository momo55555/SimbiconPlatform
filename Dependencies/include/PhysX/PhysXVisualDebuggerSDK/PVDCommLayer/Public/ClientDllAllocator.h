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

#ifndef PVD_CLIENTDLLALLOCATOR_H
#define PVD_CLIENTDLLALLOCATOR_H

template<typename T>
class ClientDllAllocator : public PVD::ClientAllocator<T>
{
public:
	ClientDllAllocator(){}
	ClientDllAllocator( const ClientDllAllocator<T>& inOther )
		: PVD::ClientAllocator<T>( inOther ) {}
	ClientDllAllocator( const char* inName ) : PVD::ClientAllocator<T>( inName ) {}

	inline typename PVD::ClientAllocator<T>::pointer allocate( typename PVD::ClientAllocator<T>::size_type byte_size, const char* file, int line) {
		return reinterpret_cast<typename PVD::ClientAllocator<T>::pointer>(AllocateMemory(byte_size, PVD::ClientAllocator<T>::getName(), file, line));
	}
	inline void deallocate(typename PVD::ClientAllocator<T>::pointer p, typename PVD::ClientAllocator<T>::size_type) { 
		DeallocateMemory(p);
	}
	inline void deallocate(void* p) { 
		DeallocateMemory(p);
	}
};


template<typename T>
class ClientDllUntrackedAllocator : public PVD::ClientAllocator<T>
{
public:
	ClientDllUntrackedAllocator(){}
	ClientDllUntrackedAllocator( const ClientDllUntrackedAllocator<T>& inOther )
		: PVD::ClientAllocator<T>( inOther ) {}
	ClientDllUntrackedAllocator( const char* inName ) : PVD::ClientAllocator<T>( inName ) {}

	inline typename PVD::ClientDllUntrackedAllocator<T>::pointer allocate( typename PVD::ClientDllUntrackedAllocator<T>::size_type byte_size, const char* file, int line) {
		return reinterpret_cast<typename PVD::ClientDllUntrackedAllocator<T>::pointer>(UntrackedAllocateMemory(byte_size, PVD::ClientAllocator<T>::getName(), file, line));
	}
	inline void deallocate(typename PVD::ClientDllUntrackedAllocator<T>::pointer p, typename PVD::ClientDllUntrackedAllocator<T>::size_type) { 
		UntrackedDeallocateMemory(p);
	}
	inline void deallocate(void* p) { 
		UntrackedDeallocateMemory((char*)p);
	}
};

#endif
