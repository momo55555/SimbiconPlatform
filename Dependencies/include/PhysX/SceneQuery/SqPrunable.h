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
#ifndef ICEPRUNABLE_H
#define ICEPRUNABLE_H

#include "../GeomUtils/Opcode/Ice/IceContainer.h"
#include "../GeomUtils/Opcode/Opcode.h"

namespace physx
{

namespace Gu
{
	class Sphere;
}

namespace Sq
{

	#define INVALID_PRUNING_HANDLE	0xffff	// WARNING: don't use -1 as if fails on some comparisons

	// Forward declarations
	class PruningPool;
	class Prunable;

	// PT: WARNING: this is now stored on 8bits
	enum PruningFlag
	{
		PRN_DYNAMIC			= (1<<0),		//!< PT: KEEP THIS THE FIRST BIT!
		PRN_VALIDAABB		= (1<<1),		//!< Object's cached AABB is valid
		PRN_SQ_DIRTY		= (1<<2),		//!< Dirty flag from SceneQueryShapeData, moved here. Tells if shape is inside the dirty shapes array, i.e. marked for "UpdateObject"

		PRN_FORCE_DWORD		= 0x7fffffff
	};

	enum PruningQueryFlag
	{
		PQF_FIRST_CONTACT	= (1<<2),	// PT: TODO: use this one!!
	};

	class Prunable : public Ps::UserAllocated
	{
		public:
											Prunable();
											~Prunable();

						void				GetWorldAABB(PxBounds3& box)	const;
						void				ComputeWorldAABB_Special(PxBounds3& box) const;

		PX_FORCE_INLINE	Ps::IntBool			IsSet(PruningFlag flag)	const	{ return mPRNFlags & flag;					}
		PX_FORCE_INLINE	void				SetSQDirtyFlag()				{ mPRNFlags |= PRN_SQ_DIRTY;				}
		PX_FORCE_INLINE	void				ClearSQDirtyFlag()				{ mPRNFlags &= ~PRN_SQ_DIRTY;				}
		PX_FORCE_INLINE	void				SetDynamicFlag()				{ mPRNFlags |= PRN_DYNAMIC;					}
		PX_FORCE_INLINE	void				ClearDynamicFlag()				{ mPRNFlags &= ~PRN_DYNAMIC;				}
		PX_FORCE_INLINE	Ps::IntBool			PrunerIndex()			const	{ return mPRNFlags & PRN_DYNAMIC;			}
		// PT: special function to avoid some LHS & branches
		PX_FORCE_INLINE	void				SetDynamicClearDirty(bool flag)
											{
												// PT: PRN_DYNAMIC must be the first flag, so we don't need any CMP or anything here:
												mPRNFlags = flag;
											}

		PX_FORCE_INLINE	bool				IsValid()				const	{ return mHandle!=INVALID_PRUNING_HANDLE;	}
		PX_FORCE_INLINE	PxU16				GetHandle()				const	{ return mHandle;							}

// Octree/quadtree stuff
		PX_FORCE_INLINE	Prunable*			GetPreviousObject()		const	{ return mPrevious;	}
		PX_FORCE_INLINE	Prunable*			GetNextObject()			const	{ return mNext;		}
		PX_FORCE_INLINE	void*				GetOwnerCell()			const	{ return mOwner;	}

		PX_FORCE_INLINE	void				Clear()
											{
												mPrevious	= NULL;
												mNext		= NULL;
												mOwner		= NULL;
											}
		public:
			// Linked list
				Prunable*					mPrevious;	//!< Previous object, or NULL for top of the list
				Prunable*					mNext;		//!< Next object, or NULL for bottom of the list

		// Owner cell
				void*						mOwner;		//!< Octree or quadtree cell containing those listed objects
//~Octree/quadtree stuff

//		private:
				PxU16						mHandle;		//!< Index in the pruning engine's arrays
				PxU16						mPRNFlags;		//!< Combination of PruningFlag

		// Internal methods

		friend	class						PruningPool;
	};

	typedef PxU32	(*StabCallback)				(const Prunable* prunable, float& max_dist, void* user_data);
	typedef bool	(*ReportPrunablesCallback)	(Prunable** prunables, PxU32 nb, void* userData);

	class PrunedObjects
	{
		public:
											PrunedObjects(Ice::ContainerSizeT&c) : container(c)						{}
											~PrunedObjects()					{}

		PX_FORCE_INLINE	PxU32				GetNbPrunables()			const	{ return container.GetNbEntries();					}
		PX_FORCE_INLINE	Prunable*			GetPrunable(PxU32 i)				{ return (Prunable*)container.GetEntry(i);			}
		PX_FORCE_INLINE	Prunable**			GetPrunables()						{ return (Prunable**)container.GetEntries();		}
		PX_FORCE_INLINE	const Prunable*		GetPrunable(PxU32 i)		const	{ return (const Prunable*)container.GetEntry(i);	}
		PX_FORCE_INLINE	const Prunable**	GetPrunables()				const	{ return (const Prunable**)container.GetEntries();	}

		PX_FORCE_INLINE	void				ResetObjects()						{ container.Reset();								}

		PX_FORCE_INLINE	void				AddPrunable(const Prunable* object)	{ container.Add((size_t)object);					};

	private:
						Ice::ContainerSizeT&	container;
	};

	class CulledObjects
	{
		public:
											CulledObjects(Ice::ContainerSizeT&c) : container(c)	{}
											~CulledObjects()								{}

		PX_FORCE_INLINE	PxU32				GetNbPrunables()			const	{ return container.GetNbEntries();							}
		// BEWARE of clip flags inside this pointers at bit position 0
		PX_FORCE_INLINE	const size_t*		GetData()					const	{ return container.GetEntries();							}
		PX_FORCE_INLINE	Prunable*			GetCulled(PxU32 i)			const	{ return (Prunable*)(container.GetEntry(i)&~size_t(1));		}
		PX_FORCE_INLINE	Ps::IntBool			IsClipped(PxU32 i)			const	{ return (Ps::IntBool)(container.GetEntry(i)&1);			}
		PX_FORCE_INLINE	void				DeleteCulled(PxU32 index)			{ container.DeleteIndex(index);								}
		PX_FORCE_INLINE	void				ResetObjects()						{ container.Reset();										}
		PX_FORCE_INLINE	void				AddPrunable(const Prunable* object)	{ container.Add((size_t)object);							}
		PX_FORCE_INLINE	void				AddPrunable(const Prunable* object, bool clipped)
											{
												PX_ASSERT(!(size_t(object)&1));
												container.Add(size_t(object)|size_t(clipped));
											}

	private:
						Ice::ContainerSizeT&		container;
	};

} // namespace Ice

}

#endif // ICEPRUNABLE_H
