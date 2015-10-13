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
#include "PsMemoryMappedFile.h"
#include "windows/PsWindowsInclude.h"

namespace physx
{
	namespace shdfnd3
	{

class MemoryMappedFileImpl
{
public:
	HANDLE	mMapFile;
	void *  mHeader;
};

MemoryMappedFile::MemoryMappedFile(const char *mappingObject,unsigned int mapSize)
{
	mImpl = (MemoryMappedFileImpl *)PX_ALLOC(sizeof(MemoryMappedFileImpl));
	mImpl->mHeader = NULL;
   	mImpl->mMapFile = OpenFileMappingA(FILE_MAP_ALL_ACCESS,FALSE,mappingObject);
	if ( mImpl->mMapFile == NULL )
	{
		mImpl->mMapFile = CreateFileMappingA(
      				INVALID_HANDLE_VALUE,    // use paging file
       				NULL,                    // default security
       				PAGE_READWRITE,          // read/write access
       				0,                       // maximum object size (high-order DWORD)
       				mapSize,                // maximum object size (low-order DWORD)
       				mappingObject);
	}
	if ( mImpl->mMapFile )
	{
		mImpl->mHeader = MapViewOfFile(mImpl->mMapFile,FILE_MAP_ALL_ACCESS,0,0,mapSize);
	}

}

MemoryMappedFile::~MemoryMappedFile(void)
{

	if ( mImpl->mHeader )
   	{
   		UnmapViewOfFile(mImpl->mHeader);
   		if ( mImpl->mMapFile )
   		{
   			CloseHandle(mImpl->mMapFile);
   		}
	}

	PX_FREE(mImpl);
}

void * MemoryMappedFile::getBaseAddress(void)
{
	return mImpl->mHeader;
}

}; // end of namespace
}; // end of namespace
