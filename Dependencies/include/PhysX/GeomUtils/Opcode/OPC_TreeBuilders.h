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
#ifndef OPC_TREEBUILDERS_H
#define OPC_TREEBUILDERS_H

#include "PsUserAllocated.h"
#include "Opcode.h"

namespace physx
{
namespace Ice
{
	class MeshInterface;

	//! Tree splitting rules
	enum SplittingRules
	{
		// Primitive split
		SPLIT_LARGEST_AXIS		= (1<<0),		//!< Split along the largest axis
		SPLIT_SPLATTER_POINTS	= (1<<1),		//!< Splatter primitive centers (QuickCD-style)
		SPLIT_BEST_AXIS			= (1<<2),		//!< Try largest axis, then second, then last
		SPLIT_BALANCED			= (1<<3),		//!< Try to keep a well-balanced tree
		SPLIT_FIFTY				= (1<<4),		//!< Arbitrary 50-50 split
		// Node split
		SPLIT_GEOM_CENTER		= (1<<5),		//!< Split at geometric center (else split in the middle)
		//
		SPLIT_FORCE_DWORD		= 0x7fffffff
	};

	//! Simple wrapper around build-related settings [Opcode 1.3]
	struct BuildSettings
	{
		PX_FORCE_INLINE	BuildSettings() : mLimit(1), mRules(SPLIT_FORCE_DWORD), mExtensionValue(0.0f), mAxis(PX_INVALID_U32), mSkinSize(0.0f)	{}

		PxU32		mLimit;				//!< Limit number of primitives / node. If limit is 1, build a complete tree (2*N-1 nodes)
		PxU32		mRules;				//!< Building/Splitting rules (a combination of SplittingRules flags)
		float		mExtensionValue;	//!< [Opcode 1.4]
		PxU32		mAxis;				//!< [Opcode 1.4]
		float		mSkinSize;			//!< [Opcode 1.4]
	};

	class AABBTreeBuilder : public Ps::UserAllocated
	{
		public:
		//! Constructor
									AABBTreeBuilder()
									{
										Reset();
									}
		//! Destructor
		virtual						~AABBTreeBuilder()			{}

		PX_FORCE_INLINE	void			Reset()
									{
										mTotalPrims			= 0;
										mNbPrimitives		= 0;
										mNodeBase			= NULL;
										mCount				= 0;
										mNbInvalidSplits	= 0;
									}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Computes the AABB of a set of primitives.
		 *	\param		primitives		[in] list of indices of primitives
		 *	\param		nb_prims		[in] number of indices
		 *	\param		global_box		[out] global AABB enclosing the set of input primitives
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual		bool			ComputeGlobalBox(const PxU32* primitives, PxU32 nb_prims, PxBounds3& global_box)	const	= 0;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Computes the splitting value along a given axis for a given primitive.
		 *	\param		index			[in] index of the primitive to split
		 *	\param		axis			[in] axis index (0,1,2)
		 *	\return		splitting value
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual		float			GetSplittingValue(PxU32 index, PxU32 axis)	const	= 0;
		virtual		void			GetSplittingValues(PxU32 index, PxVec3& values)	const	= 0;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Computes the splitting value along a given axis for a given node.
		 *	\param		primitives		[in] list of indices of primitives
		 *	\param		nb_prims		[in] number of indices
		 *	\param		global_box		[in] global AABB enclosing the set of input primitives
		 *	\param		axis			[in] axis index (0,1,2)
		 *	\return		splitting value
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual		float			GetSplittingValue(const PxU32* primitives, PxU32 nb_prims, const PxBounds3& global_box, PxU32 axis)	const
									{
										// Default split value = middle of the axis (using only the box)
										return global_box.getCenter(axis);
									}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Validates node subdivision. This is called each time a node is considered for subdivision, during tree building.
		 *	\param		primitives		[in] list of indices of primitives
		 *	\param		nb_prims		[in] number of indices
		 *	\param		global_box		[in] global AABB enclosing the set of input primitives
		 *	\return		Ps::IntTrue if the node should be subdivised
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual		Ps::IntBool		ValidateSubdivision(const PxU32* primitives, PxU32 nb_prims, const PxBounds3& global_box)
									{
										// Check the user-defined limit
										if(nb_prims<=mSettings.mLimit)	return Ps::IntFalse;

										return Ps::IntTrue;
									}

					BuildSettings	mSettings;			//!< Splitting rules & split limit [Opcode 1.3]
					PxU32			mNbPrimitives;		//!< Total number of primitives.
					void*			mNodeBase;			//!< Address of node pool [Opcode 1.3]
		//
					PxBounds3		mInitBox;			//!< [Opcode 1.4]
					bool			mInitNode;			//!< [Opcode 1.4]
		// Stats
		PX_FORCE_INLINE	void			SetCount(PxU32 nb)				{ mCount=nb;				}
		PX_FORCE_INLINE	void			IncreaseCount(PxU32 nb)			{ mCount+=nb;				}
		PX_FORCE_INLINE	PxU32			GetCount()				const	{ return mCount;			}
		PX_FORCE_INLINE	void			SetNbInvalidSplits(PxU32 nb)	{ mNbInvalidSplits=nb;		}
		PX_FORCE_INLINE	void			IncreaseNbInvalidSplits()		{ mNbInvalidSplits++;		}
		PX_FORCE_INLINE	PxU32			GetNbInvalidSplits()	const	{ return mNbInvalidSplits;	}

					PxU32			mTotalPrims;
		private:
					PxU32			mCount;				//!< Stats: number of nodes created
					PxU32			mNbInvalidSplits;	//!< Stats: number of invalid splits
	};

	class AABBTreeOfVerticesBuilder : public AABBTreeBuilder
	{
		public:
		//! Constructor
								AABBTreeOfVerticesBuilder() : mVertexArray(NULL)	{}
		//! Destructor
		virtual					~AABBTreeOfVerticesBuilder()						{}

		// AABBTreeBuilder
		virtual	bool			ComputeGlobalBox(const PxU32* primitives, PxU32 nb_prims, PxBounds3& global_box)	const;
		virtual	float			GetSplittingValue(PxU32 index, PxU32 axis)											const;
		virtual	void			GetSplittingValues(PxU32 index, PxVec3& values)										const;
		virtual	float			GetSplittingValue(const PxU32* primitives, PxU32 nb_prims, const PxBounds3& global_box, PxU32 axis)	const;
		//~AABBTreeBuilder

		const	PxVec3*			mVertexArray;		//!< Shortcut to an app-controlled array of vertices.
	};

	class AABBTreeOfAABBsBuilder : public AABBTreeBuilder
	{
		public:
		//! Constructor
								AABBTreeOfAABBsBuilder() : mAABBArray(NULL)	{}
		//! Destructor
		virtual					~AABBTreeOfAABBsBuilder()					{}

		// AABBTreeBuilder
		virtual	bool			ComputeGlobalBox(const PxU32* primitives, PxU32 nb_prims, PxBounds3& global_box)	const;
		virtual	float			GetSplittingValue(PxU32 index, PxU32 axis)											const;
		virtual	void			GetSplittingValues(PxU32 index, PxVec3& values)										const;
		//~AABBTreeBuilder

		const	PxBounds3*		mAABBArray;			//!< Shortcut to an app-controlled array of AABBs.
	};

	class AABBTreeOfTrianglesBuilder : public AABBTreeBuilder
	{
		public:
		//! Constructor
								AABBTreeOfTrianglesBuilder() : mIMesh(NULL)										{}
		//! Destructor
		virtual					~AABBTreeOfTrianglesBuilder()													{}

		// AABBTreeBuilder
		virtual	bool			ComputeGlobalBox(const PxU32* primitives, PxU32 nb_prims, PxBounds3& global_box)	const;
		virtual	float			GetSplittingValue(PxU32 index, PxU32 axis)											const;
		virtual	void			GetSplittingValues(PxU32 index, PxVec3& values)										const;
		virtual	float			GetSplittingValue(const PxU32* primitives, PxU32 nb_prims, const PxBounds3& global_box, PxU32 axis)	const;
		//~AABBTreeBuilder

		const	MeshInterface*	mIMesh;			//!< Shortcut to an app-controlled mesh interface
	};

} // namespace Ice

}

#endif // OPC_TREEBUILDERS_H
