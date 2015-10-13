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
#ifndef ICEPAIRS_H
#define ICEPAIRS_H

#include "./Ice/IceContainer.h"

namespace physx
{
namespace Ice
{

	//! A generic couple structure
	class Pair : public Ps::UserAllocated
	{
		public:
		PX_FORCE_INLINE	Pair()										{}
		PX_FORCE_INLINE	Pair(PxU32 i0, PxU32 i1) : id0(i0), id1(i1)	{}
		PX_FORCE_INLINE	~Pair()										{}

		//! Operator for "if(Pair==Pair)"
		PX_FORCE_INLINE	bool			operator==(const Pair& p)	const	{ return (id0==p.id0) && (id1==p.id1);	}
		//! Operator for "if(Pair!=Pair)"
		PX_FORCE_INLINE	bool			operator!=(const Pair& p)	const	{ return (id0!=p.id0) || (id1!=p.id1);	}

						PxU32	id0;	//!< First index of the pair
						PxU32	id1;	//!< Second index of the pair
	};
	PX_COMPILE_TIME_ASSERT(sizeof(Pair)==8);

	class Pairs : private Container
	{
		public:
									Pairs()							{}
									~Pairs()						{}

		PX_FORCE_INLINE	PxU32		GetNbPairs()		const		{ return GetNbEntries()>>1;					}
		PX_FORCE_INLINE	const Pair*	GetPairs()			const		{ return (const Pair*)GetEntries();			}
		PX_FORCE_INLINE	const Pair*	GetPair(PxU32 i)	const		{ return (const Pair*)&GetEntries()[i+i];	}

		PX_FORCE_INLINE	Ps::IntBool	HasPairs()			const		{ return IsNotEmpty();						}

		PX_FORCE_INLINE	void		ResetPairs()					{ Reset();									}
		PX_FORCE_INLINE	void		DeleteLastPair()				{ DeleteLastEntry();	DeleteLastEntry();	}

		PX_FORCE_INLINE	void		AddPair(const Pair& p)			{ Add(p.id0).Add(p.id1);					}
		PX_FORCE_INLINE	void		AddPair(PxU32 id0, PxU32 id1)	{ Add(id0).Add(id1);						}

		// HANDLE WITH CARE - I hope you know what you're doing
		PX_FORCE_INLINE	void		ForceNbPairs(PxU32 nb_pairs)	{ ForceSize(nb_pairs<<1);					}
	};
}

}

#endif // ICEPAIRS_H
