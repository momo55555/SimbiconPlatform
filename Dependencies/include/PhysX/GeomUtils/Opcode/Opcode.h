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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __OPCODE_H__
#define __OPCODE_H__


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preprocessor
#ifdef PX_WINDOWS
	#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#endif

	#include "PxBounds3.h"
	#include "CmPhysXCommon.h"
	#include "PsIntrinsics.h"

#if defined(_XBOX)
	#include "xbox360/OPC_IceHook_XBOX.h"
#elif defined(_WIN32)
	#include "windows/OPC_IceHook_WIN.h"
#elif defined(LINUX) || defined(__APPLE__)
	#include "linux/OPC_IceHook_LINUX.h"
#elif defined(PX_WII)
	#include "wii/OPC_IceHook_WII.h"
#elif defined(__CELLOS_LV2__)
	#include "ps3/OPC_IceHook_PS3.h"
#endif

	//by Adam:
	extern bool PxOpcodeError(const char* message, const char*, unsigned );
	#define	SetIceError(a)		PxOpcodeError(a,__FILE__, __LINE__)

	#include "CmSimd.h"

	#include "OPC_Settings.h"

#endif // __OPCODE_H__
