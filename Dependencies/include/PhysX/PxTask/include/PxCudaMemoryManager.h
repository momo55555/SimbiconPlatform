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

#ifndef PX_CUDA_MEMORY_MANAGER_H
#define PX_CUDA_MEMORY_MANAGER_H

#include "PxSimpleTypes.h"

// some macros to keep the source code more readable
#define NV_ALLOC_INFO(name, ID) __FILE__, __LINE__, name, physx::pxtask::AllocId::ID
#define NV_ALLOC_INFO_PARAMS_DECL(p0, p1, p2, p3)  const char* file = p0, PxU32 line = p1, const char* allocName = p2, physx::pxtask::AllocId::Enum allocId = physx::pxtask::AllocId::p3
#define NV_ALLOC_INFO_PARAMS_DEF()  const char* file, PxU32 line, const char* allocName, physx::pxtask::AllocId::Enum allocId
#define NV_ALLOC_INFO_PARAMS_INPUT()  file, line, allocName, allocId
#define NV_ALLOC_INFO_PARAMS_INPUT_INFO(info) info.getFileName(), info.getLine(), info.getAllocName(), info.getAllocId()

#ifndef NULL // don't want to include <string.h>
#define NULL 0
#endif

namespace physx
{
namespace pxtask
{
using namespace pubfnd;

PX_PUSH_PACK_DEFAULT

/** \brief ID of the Feature which owns/allocated memory from the heap
 *
 * Maximum of 64k IDs allowed.
 */
struct AllocId
{
	enum Enum
	{
		UNASSIGNED,		//!< default
		APEX,			//!< APEX stuff not further classified
		PARTICLES,		//!< all particle related
		DEFORMABLE,		//!< all deformable related (cloth & softbody)
		GPU_UTIL,		//!< e.g. RadixSort (used in SPH and deformable self collision)
		NUM_IDS			//!< number of IDs, be aware that ApexHeapStats contains AllocIdStats[NUM_IDS]
	};
};

/// \brief memory type managed by a heap
struct CudaBufferMemorySpace
{
	enum Enum
	{
		T_GPU,
		T_PINNED_HOST,
		T_WRITE_COMBINED,
		T_HOST,
		COUNT
	};
};

/// \brief class to track allocation statistics, see PxgMirrored
class AllocInfo
{
public:
	AllocInfo() {}

	AllocInfo(const char* file, int line, const char* allocName, AllocId::Enum allocId)
		: mFileName(file)
		, mLine(line)
		, mAllocName(allocName)
		, mAllocId(allocId)
	{}

	inline	const char*			getFileName() const
	{
		return mFileName;
	}
	inline	int					getLine() const
	{
		return mLine;
	}
	inline	const char*			getAllocName() const
	{
		return mAllocName;
	}
	inline	AllocId::Enum		getAllocId() const
	{
		return mAllocId;
	}

private:
	const char*			mFileName;
	int					mLine;
	const char*			mAllocName;
	AllocId::Enum		mAllocId;
};

/// \brief statistics collected per AllocationId by HeapManager.
struct AllocIdStats
{
	size_t size;		//!< currently allocated memory by this ID
	size_t maxSize;		//!< max allocated memory by this ID
	size_t elements;	//!< number of current allocations by this ID
	size_t maxElements;	//!< max number of allocations by this ID
};

class CudaMemoryManager;
typedef size_t CudaBufferPtr;

/// \brief Hint flag to tell how the buffer will be used
struct CudaBufferFlags
{
	enum Enum
	{
		F_READ			= (1 << 0),
		F_WRITE			= (1 << 1),
		F_READ_WRITE	= F_READ | F_WRITE
	};
};


/// \brief Memory statistics struct returned by CudaMemMgr::getStats()
struct CudaMemoryManagerStats
{

	size_t			heapSize;		//!< Size of all pages allocated for this memory type (allocated + free).
	size_t			totalAllocated; //!< Size occupied by the current allocations.
	size_t			maxAllocated;	//!< High water mark of allocations since the SDK was created.
	AllocIdStats	allocIdStats[AllocId::NUM_IDS]; //!< Stats for each allocation ID, see AllocIdStats
};


/// \brief Buffer type: made of hint flags and the memory space (Device Memory, Pinned Host Memory, ...)
struct CudaBufferType
{
	PX_INLINE CudaBufferType(const CudaBufferType& t)
		: memorySpace(t.memorySpace)
		, flags(t.flags)
	{}
	PX_INLINE CudaBufferType(CudaBufferMemorySpace::Enum _memSpace, CudaBufferFlags::Enum _flags)
		: memorySpace(_memSpace)
		, flags(_flags)
	{}

	CudaBufferMemorySpace::Enum		memorySpace;
	CudaBufferFlags::Enum			flags;
};


/// \brief Buffer which keeps informations about allocated piece of memory.
class NvCudaBuffer
{
public:
	/// Retrieves the manager over which the buffer was allocated.
	virtual	CudaMemoryManager*			getCudaMemoryManager() const = 0;

	/// Releases the buffer and the memory it used, returns true if successful.
	virtual	bool						free() = 0;

	/// Realloc memory. Use to shrink or resize the allocated chunk of memory of this buffer.
	/// Returns true if successful. Fails if the operation would change the address and need a memcopy.
	/// In that case the user has to allocate, copy and free the memory with separate steps.
	/// Realloc to size 0 always returns false and doesn't change the state.
	virtual	bool						realloc(size_t size, NV_ALLOC_INFO_PARAMS_DECL(NULL, 0, NULL, UNASSIGNED)) = 0;

	/// Returns the type of the allocated memory.
	virtual	const CudaBufferType&		getType() const = 0;

	/// Returns the pointer to the allocated memory.
	virtual	CudaBufferPtr				getPtr() const = 0;

	/// Returns the size of the allocated memory.
	virtual	size_t						getSize() const = 0;
};


/// \brief Allocator class for different kinds of CUDA related memory.
class CudaMemoryManager
{
public:
	/// Allocate memory of given type and size. Returns a CudaBuffer if successful. Returns NULL if failed.
	virtual NvCudaBuffer*				alloc(const CudaBufferType& type, size_t size, NV_ALLOC_INFO_PARAMS_DECL(NULL, 0, NULL, UNASSIGNED)) = 0;

	/// Basic heap allocator without NvCudaBuffer
	virtual CudaBufferPtr				alloc(CudaBufferMemorySpace::Enum memorySpace, size_t size, NV_ALLOC_INFO_PARAMS_DECL(NULL, 0, NULL, UNASSIGNED)) = 0;

	/// Basic heap deallocator without NvCudaBuffer
	virtual bool						free(CudaBufferMemorySpace::Enum memorySpace, CudaBufferPtr addr) = 0;

	/// Basic heap realloc without NvCudaBuffer
	virtual bool						realloc(CudaBufferMemorySpace::Enum memorySpace, CudaBufferPtr addr, size_t size, NV_ALLOC_INFO_PARAMS_DECL(NULL, 0, NULL, UNASSIGNED)) = 0;

	/// Retrieve stats for the memory of given type. See CudaMemoryManagerStats.
	virtual void						getStats(const CudaBufferType& type, CudaMemoryManagerStats& outStats) = 0;

	/// Ensure that a given amount of free memory is available. Triggers CUDA allocations in size of (2^n * pageSize) if necessary.
	/// Returns false if page allocations failed.
	virtual bool						reserve(const CudaBufferType& type, size_t size) = 0;

	/// Set the page size. The managed memory grows by blocks 2^n * pageSize. Page allocations trigger CUDA driver allocations,
	/// so the page size should be reasonably big. Returns false if input size was invalid, i.e. not power of two.
	/// Default is 2 MB.
	virtual bool						setPageSize(const CudaBufferType& type, size_t size) = 0;

	/// Set the upper limit until which pages of a given memory type can be allocated.
	/// Reducing the max when it is already hit does not shrink the memory until it is deallocated by releasing the buffers which own the memory.
	virtual bool						setMaxMemorySize(const CudaBufferType& type, size_t size) = 0;

	/// Returns the base size. The base memory block stays persistently allocated over the SDKs life time.
	virtual size_t						getBaseSize(const CudaBufferType& type) = 0;

	/// Returns the currently set page size. The memory grows and shrinks in blocks of size (2^n pageSize)
	virtual size_t						getPageSize(const CudaBufferType& type) = 0;

	/// Returns the upper limit until which the manager is allowed to allocate additional pages from the CUDA driver.
	virtual size_t						getMaxMemorySize(const CudaBufferType& type) = 0;
};

PX_POP_PACK

} // end pxtask namespace
} // end physx namespace

#endif
