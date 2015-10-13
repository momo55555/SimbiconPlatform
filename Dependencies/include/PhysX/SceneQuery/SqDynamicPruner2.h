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
#ifndef ICEDYNAMICPRUNER2_H
#define ICEDYNAMICPRUNER2_H

#include "SqStaticPruner.h"
#include "OPC_TreeBuilders.h"

namespace physx
{
namespace Sq
{
	enum BuildStatus
	{
		BUILD_NOT_STARTED,
		BUILD_INIT,
		BUILD_IN_PROGRESS,
		BUILD_FINISHED,

		BUILD_FORCE_DWORD	= 0xffffffff
	};

	class DynamicPruner2 : public StaticPruner
	{
		public:
												DynamicPruner2();
		virtual									~DynamicPruner2();

		virtual			bool					AddObject(Prunable& object);
		virtual			bool					RemoveObject(Prunable& object);
		virtual			bool					UpdateObject(Prunable& object);
		virtual			void					addShapes(PxU32 nbShapes, SceneQueryShapeData*const* PX_RESTRICT shapes);

		virtual			void					eagerUpdatePruningTrees();

		// Queries
						bool					Cull(PruningTemps& temps, CulledObjects& objects, const Gu::Plane* planes, PxU32 nb_planes, PxU32 culling_flags);
						PxU32					Stab(StabCallback callback, void* user_data, const PxVec3& orig, const PxVec3& dir, float& max_dist);
						bool					OverlapSphere(ReportPrunablesCallback cb, void* userData, const Gu::Sphere& sphere, bool first_contact);
						bool					OverlapAABB(ReportPrunablesCallback cb, void* userData, const PxBounds3& box, bool first_contact);
						bool					OverlapOBB(ReportPrunablesCallback cb, void* userData, const Gu::Box& box, bool first_contact);
						bool					OverlapCapsule(ReportPrunablesCallback cb, void* userData, const Gu::Capsule& capsule, bool first_contact);

		virtual			void					visualize(Cm::RenderOutput& out, PxU32 color);

		PX_FORCE_INLINE	void					SetRebuildRateHint(PxU32 nbStepsForRebuild) { PX_ASSERT(nbStepsForRebuild > 3); mRebuildRateHint = (nbStepsForRebuild-3); mAdaptiveRebuildTerm = 0; }
																								// Besides the actual rebuild steps, 3 additional steps are needed.

		PX_FORCE_INLINE	PxU32					GetNbMappingEntries()	const	{ return mNbMappingEntries;	}
		PX_FORCE_INLINE	const PxU32*			GetMapping()			const	{ return mMapping;			}
		PX_FORCE_INLINE	PxU32*					GetMapping()					{ return mMapping;			}
		PX_FORCE_INLINE	bool					AllowRecording()		const	{ return mAllowRecord;		}
		PX_FORCE_INLINE	const Ice::Container&	GetRecords()			const	{ return mRecorded;			}
		PX_FORCE_INLINE	Ice::Container&			GetRecords()					{ return mRecorded;			}

		//CA: SPU raycasts
		PX_FORCE_INLINE	Prunable**				GetAddedPrunables()		const	{ return (Prunable**)mAdded.GetPrunables(); }
		PX_FORCE_INLINE	PxU32					GetNbAddedPrunables()	const	{ return mAdded.GetNbPrunables(); }

						PxU32*					ResizeMapping(PxU32 new_index);

						bool					CheckTree()		const;
						bool					CheckMapping()	const;
		protected:
					Ice::AABBTreeOfAABBsBuilder	mBuilder;
						Ice::AABBTree*			mNewTree;
						PxBounds3*				mCachedBoxes;
						PxU32					mNbCachedBoxes;
						BuildStatus				mProgress;
						PxU32					mNbMappingEntries;
						PxU32*					mMapping;
						PxU32					mNbCalls;
#ifndef SUPPORT_REFIT_BITMASK
						Ice::Container			mModifiedIndices;
#endif
						Ice::Container			mRecorded;		// Records position changes of pruning objects which result from adding/removing objects.
																// These changes have to be recorded and applied to the new tree when it is ready.
						PrunedObjects			mAdded;			// New objects are not directly added to the tree. They are kept in a list until a new tree is built.
						Ice::ContainerSizeT		mAddedContainer;//Contains objects of mAdded.
						PxU32					mAddedSize;		// When the build of a new tree is initiated, this member records the current size of the added objects list.
																// All the objects in the list will be part of the new tree, objects which are added later will have to wait for the next tree build.
						PxU32					mRebuildRateHint;		// Fraction of the total number of primitives that should be updated per step.
						PxU32					mTotalWorkUnits;		// Estimate for how much work has to be done to rebuild the tree.
						PxI32					mAdaptiveRebuildTerm;	// Term to correct the work unit estimate if the rebuild rate is not matched.
						bool					mAllowRecord;
						bool					mAllowBuild;	// A new AABB tree is built only if an object was added, removed or updated
		// Internal methods
						void					Release();
						void					Refit();
						void					ComputeMapping();
		virtual			void					PostBuildCallback() { ComputeMapping();	}
	};

} // namespace Sq

}

#endif // ICEDYNAMICPRUNER2_H
