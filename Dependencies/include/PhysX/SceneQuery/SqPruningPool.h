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
#ifndef ICEPRUNINGPOOL_H
#define ICEPRUNINGPOOL_H

#include "SqPrunable.h"

namespace physx
{
namespace Sq
{
	typedef void (*SwapCallback)(PxU32 old_index, PxU32 new_index, void* user_data);

	class PruningPool : public Ps::UserAllocated
	{
		public:
										PruningPool();
										~PruningPool();

		// Management
					bool				Init(PxU32 nb_objects);
					bool				AddObject(Prunable& object, SwapCallback callback=NULL, void* user_data=NULL);
					void				RemoveObject(Prunable& object, SwapCallback callback=NULL, void* user_data=NULL);

		// Data access
		PX_FORCE_INLINE	PxU16				GetNbObjects()			const	{ return mNbObjects;						}
		PX_FORCE_INLINE	PxU16				GetMaxNbObjects()		const	{ return mMaxNbObjects;						}
		PX_FORCE_INLINE	Prunable**			GetObjects()			const	{ return mObjects;							}

		PX_FORCE_INLINE	PxU32				GetNbActiveObjects()	const	{ return mNbObjects;						}
		PX_FORCE_INLINE	const PxBounds3*	GetCurrentWorldBoxes()	const	{ return mWorldBoxes;						}

		PX_FORCE_INLINE	const PxBounds3*	GetWorldAABB(const Prunable& object)
										{
											// Checkings
											if(object.mHandle==INVALID_PRUNING_HANDLE)
												return NULL;	// the object has not been added to the engine

//											PX_ASSERT(object.mEngine==this);

											// Lazy-rebuild the world box
											if(!object.IsSet(PRN_VALIDAABB))
											{
												// Direct validation bypasses callbacks
												const_cast<Prunable*>(&object)->mPRNFlags|=PRN_VALIDAABB;

												object.GetWorldAABB(mWorldBoxes[object.mHandle]);
											}

											// return cached box
											return &mWorldBoxes[object.mHandle];
										}
		protected:
				PxU16					mNbObjects;		//!< Current number of objects
				PxU16					mMaxNbObjects;	//!< Max. number of objects
		// Note: using 2 different dynamic arrays instead of a collection of (AABB, Prunable*)
		// allows one to access a linear list of AABBs. At some point it was handy.
				PxBounds3*				mWorldBoxes;	//!< List of world boxes
				Prunable**				mObjects;		//!< List of objects
		// Internal methods
				bool					Resize();
	};

} // namespace Sq

}

#endif // ICEPRUNINGPOOL_H
