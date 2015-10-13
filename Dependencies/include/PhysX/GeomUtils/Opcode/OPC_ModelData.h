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
#ifndef OPC_MODELDATA_H
#define OPC_MODELDATA_H

#include "./Ice/IceFPU.h"
#include "OPC_Common.h"
#include "RTree.h"

namespace physx
{
namespace Ice
{
	class MeshInterface;

	PX_FORCE_INLINE	PxU32		StoreAsLeaf(PxU32 data)			{ return data | SIGN_BITMASK;	}
	PX_FORCE_INLINE	PxU32		MarkAsLeafN(PxU32 data)			{ return data | 0x40000000;		}
	PX_FORCE_INLINE	PxU32		StoreAsNonLeaf(PxU32 data)		{ return data;					}
	PX_FORCE_INLINE	Ps::IntBool	_IsLeaf(PxU32 data)				{ return data & SIGN_BITMASK;	}
	PX_FORCE_INLINE	Ps::IntBool	_IsLeafP(PxU32 data)			{ return data & SIGN_BITMASK;	}
	PX_FORCE_INLINE	Ps::IntBool	_IsLeafN(PxU32 data)			{ return data & 0x40000000;		}
	PX_FORCE_INLINE	Ps::IntBool	GetLeafData(PxU32 data)			{ return data & ~(SIGN_BITMASK|0x40000000);	}
	PX_FORCE_INLINE	Ps::IntBool	GetNonLeafData(PxU32 data)		{ return data;					}

	template<class volume>
	class AABBStacklessNode
	{
    public:
	PX_FORCE_INLINE						AABBStacklessNode()				{}
	PX_FORCE_INLINE						~AABBStacklessNode()			{}
	// Leaf test
	PX_FORCE_INLINE		Ps::IntBool		HasPosLeaf()			const	{ return _IsLeafP(mPosIndexData);				}
	PX_FORCE_INLINE		Ps::IntBool		HasNegLeaf()			const	{ return _IsLeafN(mPosIndexData);				}
//	PX_FORCE_INLINE		Ps::IntBool		HasPosAndNotNegLeaf()	const	{ return _IsLeafPAndNonLeafN(mPosIndexData);	}
//	PX_FORCE_INLINE		Ps::IntBool		HasPosAndNegLeaf()		const	{ return _IsLeafPAndLeafN(mPosIndexData);		}
	// Data access
	PX_FORCE_INLINE		PxU32			GetPosPrimitive()		const	{ return GetLeafData(mPosIndexData);			}
	PX_FORCE_INLINE		PxU32			GetNegPrimitive()		const	{ return 1+GetLeafData(mPosIndexData);			}	// REMAP_XP
	// Stats
	PX_FORCE_INLINE		PxU32			GetNodeSize()			const	{ return sizeof(*this);							}

						volume			mAABB;
						PxU32			mPosIndexData;
	//					PxU32			mNegIndexData;	// REMAP_XP
						PxU32			mEscapeIndex;	// SEPT25BUG
	};

	typedef AABBStacklessNode<CollisionAABB>	AABBStacklessNoLeafNode;
	typedef AABBStacklessNode<QuantizedAABB>	AABBStacklessQuantizedNoLeafNode;


	enum ModelFlag
	{
		OPC_SINGLE_NODE		= (1<<2),	//!< Special case for 1-node models
	};

	// PT: this class contains the source/input data for a model. It's used to decouple the colliders from the model classes, which contain the building/non-runtime code.
	struct BaseModelData
	{
			// Model data
			const MeshInterface*					mIMesh;
			PxU32									mModelCode;
			// Tree data
			PxU32									mNbNodes;
			const AABBStacklessQuantizedNoLeafNode*	mNodes;

			PX_FORCE_INLINE			Ps::IntBool			HasSingleNode()		const	{ return mModelCode & OPC_SINGLE_NODE;		}
	};

	//! Leaf descriptor
	class LeafTriangles
	{
		public:
		PX_FORCE_INLINE			LeafTriangles()		{}
		PX_FORCE_INLINE			~LeafTriangles()	{}

		PxU32			Data;		//!< Packed data

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Gets number of triangles in the leaf.
		 *	\return		number of triangles N, with 0 < N <= 16
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		PX_FORCE_INLINE	PxU32	GetNbTriangles()				const	{ return (Data & 15)+1;											}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Gets triangle index for this leaf. Indexed model's array of indices retrieved with HybridModel::GetIndices()
		 *	\return		triangle index
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		PX_FORCE_INLINE	PxU32	GetTriangleIndex()				const	{ return Data>>4;													}
		PX_FORCE_INLINE	void	SetData(PxU32 nb, PxU32 index)			{ PX_ASSERT(nb>0 && nb<=16);	nb--;	Data = (index<<4)|(nb&15);	}
	};
	PX_COMPILE_TIME_ASSERT(sizeof(LeafTriangles)==4);

	// PT: this class contains the source/input data for a hybrid model. It's used to decouple the colliders from the model classes, which contain the building/non-runtime code.
	struct HybridModelData : BaseModelData
	{
		PxU32					mNbLeaves;
		const LeafTriangles*	mTriangles;
		PxU32					mNbPrimitives;
		const PxU32*			mIndices;
		const Gu::RTree*		mRTree;

		PX_FORCE_INLINE	const LeafTriangles*	GetLeafTriangles()	const	{ return mTriangles;	}
		PX_FORCE_INLINE	const PxU32*			GetIndices()		const	{ return mIndices;		}
	};


} // namespace Ice

}

#endif // OPC_MODELDATA_H
