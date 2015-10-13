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
#ifndef ICEPRUNER_H
#define ICEPRUNER_H

#include "../GeomUtils/Opcode/Ice/IceHandleManager.h"
#include "SqPruningPool.h"
#include "../GeomUtils/Opcode/StabCodes.h"
#include "CmRenderOutput.h"
#include "PxSceneDesc.h"

namespace physx
{

namespace Gu
{
	class Capsule;
	class Box;
	class Plane;
}

namespace Sq
{
	struct SceneQueryShapeData;

	/*
	This struct is passed to opcode queries, it contains tempory buffers used by the pruners. This is to allow
	different buffers to be used per thread. In particular for the static pruner.
	*/
	struct PruningTemps
	{
		// These are used by DynamicPruner::Cull, previously static.
		// We do not share memory and allow them to be dynamically sized as before.
		Ice::ContainerSizeT		mVisibleBoxIndicesClip;
		Ice::ContainerSizeT		mVisibleBoxIndicesNoClip;
	};

	struct PRUNERCREATE
	{
									PRUNERCREATE();

		PxBounds3					mExpectedWorldBox;
		PxU32						mUpAxis;
		PxU32						mSubdivisionLevel;

		PxU32						mNbStaticObjects;		//!< expected number of static objects
		PxU32						mNbDynamicObjects;		//!< expected number of dynamic objects
		PxPruningStructure::Enum	mStaticType;			//!< expected type of static objects
		PxPruningStructure::Enum	mDynamicType;			//!< expected type of dynamic objects
	};

	class Signature
	{
		public:
		// Constructor/Destructor
								Signature();
								~Signature();

		PX_FORCE_INLINE	void	Invalidate()		{ mTimestamp++;	}

								PREVENT_COPY(Signature)
		private:
					Ice::Handle	mStructureHandle;	//!< Object's structural validity
						PxU32	mTimestamp;			//!< Object's state validity

		friend	class	SignatureState;
	};

	class SignatureState
	{
		public:
		// Constructor/Destructor
		PX_FORCE_INLINE				SignatureState() : mStructureHandle(PX_INVALID_U32), mTimestamp(PX_INVALID_U32)
									{
									}
		PX_FORCE_INLINE				SignatureState(const Signature& signature) : mStructureHandle(signature.mStructureHandle), mTimestamp(signature.mTimestamp)
									{
									}
		PX_FORCE_INLINE				~SignatureState()		{}

		PX_FORCE_INLINE	void		operator = (const Signature& signature)
									{
										mStructureHandle = signature.mStructureHandle;
										mTimestamp = signature.mTimestamp;
									}

		PX_FORCE_INLINE	Ps::IntBool	operator == (const Signature& signature)	const
									{
										if(signature.mTimestamp!=mTimestamp)				return Ps::IntFalse;
										if(signature.mStructureHandle!=mStructureHandle)	return Ps::IntFalse;
										return Ps::IntTrue;
									}

		PX_FORCE_INLINE	Ps::IntBool	operator != (const Signature& signature)	const
									{
										if(signature.mTimestamp!=mTimestamp)				return Ps::IntTrue;
										if(signature.mStructureHandle!=mStructureHandle)	return Ps::IntTrue;
										return Ps::IntFalse;
									}
		private:
						Ice::Handle	mStructureHandle;	//!< Object's structural validity
						PxU32		mTimestamp;			//!< Object's state validity
	};

	class Pruner;
	typedef bool	(Pruner::*CullFunc)				(PruningTemps& temps, CulledObjects& objects, const Gu::Plane* planes, PxU32 nb_planes, PxU32 culling_flags);
	typedef PxU32	(Pruner::*StabFunc)				(StabCallback callback, void* user_data, const PxVec3& orig, const PxVec3& dir, float& max_dist);
	typedef bool	(Pruner::*OverlapSphereFunc)	(ReportPrunablesCallback cb, void* userData, const Gu::Sphere& sphere, bool first_contact);
	typedef bool	(Pruner::*OverlapAABBFunc)		(ReportPrunablesCallback cb, void* userData, const PxBounds3& box, bool first_contact);
	typedef bool	(Pruner::*OverlapOBBFunc)		(ReportPrunablesCallback cb, void* userData, const Gu::Box& box, bool first_contact);
	typedef bool	(Pruner::*OverlapCapsuleFunc)	(ReportPrunablesCallback cb, void* userData, const Gu::Capsule& capsule, bool first_contact);

	class Pruner : public PruningPool
	{
		public:
											Pruner();
		virtual								~Pruner()								{}

		virtual	bool						Setup(const PRUNERCREATE& create)		{ return true;			}

		// Data access
		PX_FORCE_INLINE	const Signature&	GetSignature()					const	{ return mSignature;	}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Adds an object to the pruner.
		 *	\param		object	[in] the object to register
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual	PX_FORCE_INLINE bool		AddObject(Prunable& object)
											{
												// Invalidate acceleration structure
												mSignature.Invalidate();
												// Add the object to the pool
												return PruningPool::AddObject(object);
											}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Removes an object from the pruner.
		 *	\param		object	[in] the object to remove
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual	PX_FORCE_INLINE bool		RemoveObject(Prunable& object)
											{
												// Invalidate acceleration structure
												mSignature.Invalidate();
												// Remove the object from the pool
												PruningPool::RemoveObject(object);
												return true;
											}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Updates an object, i.e. updates the pruner's spatial database.
		 *	\param		object	[in] the object to update
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual	PX_FORCE_INLINE bool		UpdateObject(Prunable& object)
											{
												// Invalidate acceleration structure
												mSignature.Invalidate();
												// ### to do lazy => mais pb avec GetWorldBoxes()
												// Compute and cache the new AABB
												return true;
											}

		// PT: batching call
		virtual	void						addShapes(PxU32 nbShapes, SceneQueryShapeData*const* PX_RESTRICT shapes);


		virtual void						eagerUpdatePruningTrees()								{}

		virtual	void						visualize(Cm::RenderOutput& out, PxU32 color)	 {}

//		virtual	bool						updateRecomputesPose()	const	{ return true;	}

						CullFunc			mCullFunc;
						StabFunc			mStabFunc;
						OverlapSphereFunc	mOverlapSphereFunc;
						OverlapAABBFunc		mOverlapAABBFunc;
						OverlapOBBFunc		mOverlapOBBFunc;
						OverlapCapsuleFunc	mOverlapCapsuleFunc;

		PX_FORCE_INLINE	bool				cull(PruningTemps& temps, CulledObjects& objects, const Gu::Plane* planes, PxU32 nb_planes, PxU32 culling_flags)	{ return (this->*mCullFunc)(temps, objects, planes, nb_planes, culling_flags);	}
		PX_FORCE_INLINE	PxU32				stab(StabCallback callback, void* user_data, const PxVec3& orig, const PxVec3& dir, float& max_dist)				{ return (this->*mStabFunc)(callback, user_data, orig, dir, max_dist);			}
		PX_FORCE_INLINE	bool				overlapSphere(ReportPrunablesCallback cb, void* userData, const Gu::Sphere& sphere, bool first_contact)				{ return (this->*mOverlapSphereFunc)(cb, userData, sphere, first_contact);		}
		PX_FORCE_INLINE	bool				overlapAABB(ReportPrunablesCallback cb, void* userData, const PxBounds3& box, bool first_contact)					{ return (this->*mOverlapAABBFunc)(cb, userData, box, first_contact);			}
		PX_FORCE_INLINE	bool				overlapOBB(ReportPrunablesCallback cb, void* userData, const Gu::Box& box, bool first_contact)						{ return (this->*mOverlapOBBFunc)(cb, userData, box, first_contact);			}
		PX_FORCE_INLINE	bool				overlapCapsule(ReportPrunablesCallback cb, void* userData, const Gu::Capsule& capsule, bool first_contact)			{ return (this->*mOverlapCapsuleFunc)(cb, userData, capsule, first_contact);	}

		protected:
				Signature					mSignature;		//!< Pruner's signature
	};

} // namespace Sq

}

#endif // ICEPRUNER_H
