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

#ifndef PVD_PVDCOMMLAYERERRORS_H
#define PVD_PVDCOMMLAYERERRORS_H

#include "PxSimpleTypes.h"

namespace PVD
{
	/**
	 *	All errors in the library are of this type.  
	 *	Since we can't rely on exceptions,
	 *	all functions return one of these.
	 */
	struct PvdCommLayerError
	{
		enum
		{
			None = 0,
			NameBoundToDifferentKey,
			ParentDerivedFromChild,
			ChildDerivedFromDifferentParent,
			InstanceExists,
			InvalidArguments,
			DatatypeMismatch,
			ChildError,
			ArrayBlockError,
			NetworkError,
			PropertyKeyCollision,
			BlockOpen,
			InvalidName,
			InvalidKey,
			InvalidInstance,
			InvalidProperty,
			ClassLocked,
			InvalidDatatype,
			InvalidClass,
			PropertyDefinitionError,
			InstanceClassMismatch,
			InstanceTypeMismatch,
			InvalidContext,
			InvalidBlockType,
			NoOpenSection,
			SectionNameMismatch,
			InvalidData,
			StackOverflow,
			StackUnderflow,
			Last,
		};
		physx::pubfnd::PxU8 mError;
		PX_INLINE PvdCommLayerError( physx::pubfnd::PxU8 inError = None ) : mError( inError ) {}
		PX_INLINE bool operator==( const PvdCommLayerError& inOther ) const { return mError == inOther.mError; }
		PX_INLINE bool operator!=( const PvdCommLayerError& inOther ) const { return !(*this == inOther); }
		inline const char* toString() const
		{
			switch( mError )
			{
			case None: return "None";
			case NameBoundToDifferentKey: return "NameBoundToDifferentKey";
			case ParentDerivedFromChild: return "ParentDerivedFromChild";
			case ChildDerivedFromDifferentParent: return "ChildDerivedFromDifferentParent";
			case InstanceExists: return "InstanceExists";
			case InvalidArguments: return "InvalidArguments";
			case DatatypeMismatch: return "DatatypeMismatch";
			case ChildError: return "ChildError";
			case ArrayBlockError: return "ArrayBlockError";
			case NetworkError: return "NetworkError";
			case PropertyKeyCollision: return "PropertyKeyCollision";
			case BlockOpen: return "BlockOpen";
			case InvalidName: return "InvalidName";
			case InvalidKey: return "InvalidKey";
			case InvalidInstance: return "InvalidInstance";
			case InvalidProperty: return "InvalidProperty";
			case ClassLocked: return "ClassLocked";
			case InvalidDatatype: return "InvalidDatatype";
			case InvalidClass: return "InvalidClass";
			case PropertyDefinitionError: return "PropertyDefinitionError";
			case InstanceClassMismatch: return "InstanceClassMismatch";
			case InstanceTypeMismatch: return "InstanceTypeMismatch";
			case InvalidContext: return "InvalidContext";
			case InvalidBlockType: return "InvalidBlockType";
			case NoOpenSection: return "NoOpenSection";
			case SectionNameMismatch: return "SectionNameMismatch";
			case InvalidData: return "InvalidData";
			}
			return "";
		}
	};
}

#endif