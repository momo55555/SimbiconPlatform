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
#ifndef OPC_AABBTREE_H
#define OPC_AABBTREE_H

#include "PsIntrinsics.h"
#include "PsUserAllocated.h"
#include "Opcode.h"

#define KEEP_PARENT_PTR
#define SUPPORT_PROGRESSIVE_BUILDING
#define SUPPORT_REFIT_BITMASK
#define DWORD_REFIT_BITMASK

#ifdef DWORD_REFIT_BITMASK

namespace physx
{
namespace Ice
{
	class Container;
	class AABBTreeBuilder;

	class BitArray
	{
		public:
		//! Constructor
									BitArray();
									BitArray(PxU32 nb_bits);
		//! Destructor
									~BitArray();

					bool			Init(PxU32 nb_bits);

		// Data management
		PX_INLINE	void			SetBit(PxU32 bit_number)
									{
//										mBits[bit_number>>5] |= BitMasks[bit_number&31];
										mBits[bit_number>>5] |= 1<<(bit_number&31);
									}
		PX_INLINE	void			ClearBit(PxU32 bit_number)
									{
//										mBits[bit_number>>5] &= NegBitMasks[bit_number&31];
										mBits[bit_number>>5] &= ~(1<<(bit_number&31));
									}
		PX_INLINE	void			ToggleBit(PxU32 bit_number)
									{
//										mBits[bit_number>>5] ^= BitMasks[bit_number&31];
										mBits[bit_number>>5] ^= 1<<(bit_number&31);
									}

		PX_INLINE	void			ClearAll()			{ Ps::memZero(mBits, mSize*4);			}
		PX_INLINE	void			SetAll()			{ Ps::memSet(mBits, 0xff, mSize*4);		}

		// Data access
		PX_INLINE	Ps::IntBool		IsSet(PxU32 bit_number)	const
									{
//										return mBits[bit_number>>5] & BitMasks[bit_number&31];
										return mBits[bit_number>>5] & (1<<(bit_number&31));
									}

		PX_INLINE	const PxU32*	GetBits()	const	{ return mBits;	}
		PX_INLINE	PxU32			GetSize()	const	{ return mSize;	}

		protected:
					PxU32*			mBits;		//!< Array of bits
					PxU32			mSize;		//!< Size of the array in dwords
	};
#endif

#ifdef SUPPORT_PROGRESSIVE_BUILDING
	class FIFOStack2;
#endif

	typedef		void				(*CullingCallback)		(PxU32 nb_primitives, PxU32* node_primitives, Ps::IntBool need_clipping, void* user_data);

	class AABBTreeNode : public Ps::UserAllocated
	{
		public:	
										AABBTreeNode();
										~AABBTreeNode();
		// Data access
		PX_INLINE	const PxBounds3&	GetAABB()	const	{ return mBV;	}
		PX_INLINE	PxBounds3&			GetAABB()			{ return mBV;	}
		// Clear the last bit
		PX_INLINE	const AABBTreeNode*	GetPos()	const	{ return (const AABBTreeNode*)(mPos&~1);					}
		PX_INLINE	const AABBTreeNode*	GetNeg()	const	{ const AABBTreeNode* P = GetPos(); return P ? P+1 : NULL;	}

		// We don't need to test both nodes since we can't have one without the other
		PX_INLINE	bool				IsLeaf()		const	{ return !GetPos();		}

		// Stats
		PX_INLINE	PxU32				GetNodeSize()	const	{ return sizeof(*this);	}
		protected:
		// Tree-independent data
		// Following data always belong to the BV-tree, regardless of what the tree actually contains.
		// Whatever happens we need the two children and the enclosing volume.
					PxBounds3			mBV;		// Global bounding-volume enclosing all the node-related primitives
					size_t				mPos;		// "Positive" & "Negative" children
		public:
		// Data access
		PX_INLINE	const PxU32*		GetPrimitives()		const	{ return mNodePrimitives;	}
		PX_INLINE	PxU32				GetNbPrimitives()	const	{ return mNbPrimitives;		}

					void				_TestAgainstPlanes(const Gu::Plane* planes, PxU32 clip_mask, CullingCallback cb, void* user_data)	const;
					void				_TestAgainstPlanes(const Gu::Plane* planes, PxU32 clip_mask, Container& box_indices_clip, Container& box_indices_noclip)	const;

#ifdef KEEP_PARENT_PTR
					AABBTreeNode*		mParent;
#endif
//		protected:
		// Tree-dependent data
					PxU32*				mNodePrimitives;	//!< Node-related primitives (shortcut to a position in mIndices below)
					PxU32				mNbPrimitives;		//!< Number of primitives for this node
		// Internal methods
					PxU32				Split(PxU32 axis, AABBTreeBuilder* builder);
					bool				Subdivide(AABBTreeBuilder* builder);
					void				_BuildHierarchy(AABBTreeBuilder* builder);
					void				_Refit(AABBTreeBuilder* builder);
	};

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	User-callback, called for each node by the walking code.
	 *	\param		current		[in] current node
	 *	\param		depth		[in] current node's depth
	 *	\param		user_data	[in] user-defined data
	 *	\return		true to recurse through children, else false to bypass them
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	typedef		bool				(*WalkingCallback)	(const AABBTreeNode* current, PxU32 depth, void* user_data);

	class AABBTree : public Ps::UserAllocated
	{
		public:
										AABBTree();
										~AABBTree();
		// Build
					bool				Build(AABBTreeBuilder* builder);
#ifdef SUPPORT_PROGRESSIVE_BUILDING
					PxU32				Build(AABBTreeBuilder* builder, PxU32 progress, PxU32 limit);
#endif
					void				Release();

		// Data access
		PX_INLINE	const PxU32*		GetIndices()		const	{ return mIndices;		}	//!< Catch the indices
		PX_INLINE	PxU32				GetNbNodes()		const	{ return mTotalNbNodes;	}	//!< Catch the number of nodes
		PX_INLINE	PxU32				GetTotalPrims()		const	{ return mTotalPrims;	}
		PX_INLINE	const AABBTreeNode*	GetNodes()			const	{ return mPool;			}

		// Infos
					bool				IsComplete()		const;
		// Stats
					PxU32				ComputeDepth()		const;
					PxU32				Walk(WalkingCallback callback, void* user_data) const;
					void				Walk2(WalkingCallback callback, void* user_data) const;

					bool				Refit(AABBTreeBuilder* builder);
					bool				Refit2(AABBTreeBuilder* builder);
					bool				Refit3(PxU32 nb_objects, const PxBounds3* boxes, const Container& indices);
#ifdef SUPPORT_REFIT_BITMASK
					void				MarkForRefit(PxU32 index);
					void				RefitMarked(PxU32 nb_objects, const PxBounds3* boxes);
#endif

		PX_INLINE	void				TestAgainstPlanes(const Gu::Plane* planes, PxU32 clip_mask, CullingCallback cb, void* user_data)							const
										{
											mPool->_TestAgainstPlanes(planes, clip_mask, cb, user_data);
										}
		PX_INLINE	void				TestAgainstPlanes(const Gu::Plane* planes, PxU32 clip_mask, Container& box_indices_clip, Container& box_indices_noclip)	const
										{
											mPool->_TestAgainstPlanes(planes, clip_mask, box_indices_clip, box_indices_noclip);
										}

		private:
					PxU32*				mIndices;			//!< Indices in the app list. Indices are reorganized during build (permutation).
					AABBTreeNode*		mPool;				//!< Linear pool of nodes for complete trees. NULL otherwise. [Opcode 1.3]
#ifdef SUPPORT_REFIT_BITMASK
	#ifdef DWORD_REFIT_BITMASK
					BitArray			mRefitBitmask;
	#else
					bool*				mRefitBitmask;
	#endif
#endif
		// Stats
					PxU32				mTotalNbNodes;		//!< Number of nodes in the tree.
					PxU32				mTotalPrims;
#ifdef SUPPORT_PROGRESSIVE_BUILDING
					FIFOStack2*			mStack;
#endif
	};

} // namespace Ice

}

#endif // OPC_AABBTREE_H
