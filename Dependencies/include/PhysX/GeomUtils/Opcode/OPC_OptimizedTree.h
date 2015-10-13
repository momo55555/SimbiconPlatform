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
#ifndef OPC_OPTIMIZEDTREE_H
#define OPC_OPTIMIZEDTREE_H

#include "./Ice/IceFPU.h"
#include "PsUserAllocated.h"
#include "OPC_Common.h"
#include "OPC_ModelData.h"

namespace physx
{
namespace Ice
{
	class AABBTree;
	class MeshInterface;

	typedef		bool				(*GenericWalkingCallback)	(const void* current, void* user_data);


    class AABBStacklessQuantizedNoLeafTree : public Ps::UserAllocated
	{
		public:
// PX_SERIALIZATION
																	AABBStacklessQuantizedNoLeafTree(PxRefResolver& v)
																	{
																		mNbNodes |= PX_SIGN_BITMASK;
																	}
//~PX_SERIALIZATION
																	AABBStacklessQuantizedNoLeafTree();
																	~AABBStacklessQuantizedNoLeafTree();

						bool										Build(AABBTree* tree);
						bool										Save(bool mismatch, PxStream& stream)	const;
						bool										Load(bool mismatch, const PxStream& stream);

		PX_FORCE_INLINE	const AABBStacklessQuantizedNoLeafNode*		GetNodes()			const	{ return mNodes;		}

						void										Release();
// PX_SERIALIZATION
						void										exportExtraData(PxSerialStream&);
						char*										importExtraData(char* address, PxU32& totalPadding);
		PX_FORCE_INLINE	PxU32										GetNbNodes()		const	{ return mNbNodes & ~PX_SIGN_BITMASK;	}
						void										SetNbNodes(PxU32 n)
																	{
																		assert(!isInUserMemory());	// This should really not happen with user data
																		mNbNodes = n;
																	}
		PX_FORCE_INLINE	PxU32										isInUserMemory()		const
																	{
																		return mNbNodes & PX_SIGN_BITMASK;
																	}
//~PX_SERIALIZATION
		private:
						PxU32										mNbNodes;
		public:
						AABBStacklessQuantizedNoLeafNode*			mNodes;
						PxVec3										mCenterCoeff;
						PxVec3										mExtentsCoeff;
    };

} // namespace Ice

}

#endif // OPC_OPTIMIZEDTREE_H
