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


#ifndef PX_PHYSICS_SCENEQUERYSTREAM
#define PX_PHYSICS_SCENEQUERYSTREAM
/** \addtogroup physics
@{
*/

#include "CmPhysXCommon.h"
#include "GuCapsule.h"
#include "GuPlane.h"
#include "PxBounds3.h"
#include "GuGeometryUnion.h"
#include "IceSupport.h"

namespace physx
{
namespace Sq
{
	struct SceneQueryShapeData;

	struct SceneQueryID
	{
		enum Enum
		{
			QUERY_RAYCAST_ANY_OBJECT,
			QUERY_RAYCAST_CLOSEST_OBJECT,
			QUERY_RAYCAST_ALL_OBJECTS,

			QUERY_OVERLAP_SPHERE_ALL_OBJECTS,
			QUERY_OVERLAP_AABB_ALL_OBJECTS,
			QUERY_OVERLAP_OBB_ALL_OBJECTS,
			QUERY_OVERLAP_CAPSULE_ALL_OBJECTS,

			QUERY_LINEAR_OBB_SWEEP_CLOSEST_OBJECT,
			QUERY_LINEAR_CAPSULE_SWEEP_CLOSEST_OBJECT,
			QUERY_LINEAR_COMPOUND_GEOMETRY_SWEEP_CLOSEST_OBJECT,
			QUERY_LINEAR_OBB_SWEEP_ALL_OBJECTS,
			QUERY_LINEAR_CAPSULE_SWEEP_ALL_OBJECTS,
			QUERY_LINEAR_COMPOUND_GEOMETRY_SWEEP_ALL_OBJECTS,

			QUERY_CULL_OBJECTS,
		};
	};


	struct QueryData
	{
		PxFilterData			mFilterData;		//16bytes
		PxU32					mType;				//20bytes
		PxU32					mSize;				//24bytes
		void*					mUserData;			//28bytes for 32bit os
		PxU32					mFilterFlags;		//32bytes
		bool					mFilterDataValid;	//36bytes
	};

	struct RaycastQueryData : QueryData
	{
		SceneQueryShapeData*	mCache;				//40bytes for 32bit os
		//PxShape**				mUserCache;			//44bytes for 32bit os
		PxReal					mMaxDist;			//44bytes
		PxVec3					mRayOrigin;			//56bytes
		PxSceneQueryFlags		mHintFlags;			//60bytes
		PxVec3					mRayDir;			//72bytes
		PxU32					mPad[2];			//80Bytes		
	};

	//struct RaycastAnyObjectQueryData : RaycastQueryData
	//{
	//	SceneQueryShapeData*			mCache;
	//	PxShape**						mUserCache;
	//};

	//struct RaycastClosestObjectQueryData : RaycastAnyObjectQueryData
	//{
	//	PxU32				mHintFlags;
	//};

	//struct RaycastAllObjectsQueryData : RaycastQueryData
	//{
	//	PxU32				mHintFlags;
	//};

	struct SphereQueryData : QueryData
	{
		Gu::Sphere			mWorldSphere;
	//	PxU32				mMaxShapes;
	};

	struct AABBQueryData : QueryData
	{
		PxBounds3			mWorldBounds;
	//	PxU32				mMaxShapes;
	};

	struct OBBQueryData : QueryData
	{
		// PT: TODO: eventually replace with Gu::Box, unify names ("world")
		PxVec3				mBoxCenter;
		PxVec3				mBoxExtents;
		PxQuat				mBoxRot;
	//	PxU32				mMaxShapes;
	};

	struct CapsuleQueryData : QueryData
	{
		Gu::Capsule			mWorldCapsule;
	//	PxU32				mMaxShapes;
	};

	struct CompoundGeometryQueryData : QueryData
	{
//		const Gu::GeometryUnion*	mGeometryList;
//		const PxTransform*			mPoseList;
//		const PxFilterData*			mFilterDataList;
		PxU32						mGeometryListOffset;
		PxU32						mPoseListOffset;
		PxU32						mFilterDataListOffset;
		PxU32						mGeometryCount;
		const PxSweepCache*			mSweepCache;
	};

	struct SweepQueryData
	{
		PxVec3				mUnitDir;
		PxReal				mDistance;
		PxSceneQueryFlags	mHintFlags;
	};

	struct LinearOBBSweepQueryData : OBBQueryData, SweepQueryData
	{
	};

	struct LinearCapsuleSweepQueryData : CapsuleQueryData, SweepQueryData
	{
	};

	struct LinearCompoundGeometrySweepQueryData : CompoundGeometryQueryData, SweepQueryData
	{
	};

	// PT: the number of planes is variable so this header will be followed by an arbitrary amount of Gu::Plane objects
	struct CullQueryData : QueryData
	{
		PxU32				mNbPlanes;
		Gu::Plane*			mPlanes;
	};

	union UnifiedQueryData
	{
		PxU8	querydata[sizeof(QueryData)];

		PxU8	rayquerydata[sizeof(RaycastQueryData)];
		PxU8	rayanyquerydata[sizeof(RaycastQueryData)];
		PxU8	rayclosestquerydata[sizeof(RaycastQueryData)];
		PxU8	rayallquerydata[sizeof(RaycastQueryData)];

		PxU8	spherequerydata[sizeof(SphereQueryData)];
		PxU8	aabbquerydata[sizeof(AABBQueryData)];
		PxU8	obbquerydata[sizeof(OBBQueryData)];
		PxU8	capsulequerydata[sizeof(CapsuleQueryData)];

		PxU8	compoundquerydata[sizeof(CompoundGeometryQueryData)];
		PxU8	sweepquerydata[sizeof(SweepQueryData)];
		PxU8	linobbsweepquerydata[sizeof(LinearOBBSweepQueryData)];
		PxU8	lincapsweepquerydata[sizeof(LinearCapsuleSweepQueryData)];
		PxU8	lincpdsweepquerydata[sizeof(LinearCompoundGeometrySweepQueryData)];
		PxU8	cullquerydata[sizeof(CullQueryData)];
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////

	class BatchQueryStream
	{
	public:
		PX_INLINE	BatchQueryStream()							{ reset();	}
		PX_INLINE	void					reset()				
		{ 
			mData.Reset();
			mBatchedQueries.Reset();	
			mCurrentWord = 0; 
			mBatchedRaycastQueries.Reset(); 
			mCurrentRaycastWord = 0;
			mBatchedSweepQueries.Reset();
			mCurrentSweepWord = 0;
			mNbSweepQuery = 0;
			mBatchedOverlapQueries.Reset();
			mCurrentOverlapWord = 0;
		}

		PX_INLINE	const QueryData*		getQueryData();
		PX_INLINE	const QueryData*		getNextQueryData();
		PX_INLINE	const QueryData*		getRaycastQueryData();
		PX_INLINE	const QueryData*		getNextRaycastQueryData();
		PX_INLINE	const QueryData*		getNextRaycastQueryDataBatch(PxU32 nb);
		PX_INLINE	PxU32					getNbRaycastQuery()	{ return mBatchedRaycastQueries.GetNbEntries()/(sizeof(RaycastQueryData)>>2);}
		PX_INLINE	const QueryData*		getSweepQueryData();
		PX_INLINE	const QueryData*		getNextSweepQueryData();
		PX_INLINE	const QueryData*		getNextSweepQueryDataBatch(PxU32 nb);
		PX_INLINE	PxU32					getNbSweepQuery()	{ return mNbSweepQuery;}
		PX_INLINE	const QueryData*		getOverlapQueryData();
		PX_INLINE	const QueryData*		getNextOverlapQueryData();
		PX_INLINE	const QueryData*		getNextOverlapQueryDataBatch(PxU32 nb);
		PX_INLINE	PxU32					getNbOverlapQuery()	{ return mBatchedOverlapQueries.GetNbEntries()/(sizeof(UnifiedQueryData)>>2);}

		PX_INLINE	void*		offsetToPointer(PxU32 offset) const { return PxU32(~0) == offset ? NULL : mData.GetEntries() + offset; }
		PX_INLINE	PxU32		pointerToOffset(void* p) const { return NULL == p ? ~0 : PxU32(reinterpret_cast<PxU32*>(p) - mData.GetEntries()); }

		// queries
		PX_INLINE	void		raycastAny		(const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist,									const PxSceneQueryFilterData& filterData, Sq::SceneQueryShapeData* cache,	void* userData) const;
		PX_INLINE 	void		raycastSingle	(const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist, PxSceneQueryFlags hintFlags,	const PxSceneQueryFilterData& filterData, Sq::SceneQueryShapeData* cache,	void* userData) const;
		PX_INLINE 	void		raycastMultiple	(const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist, PxSceneQueryFlags hintFlags,	const PxSceneQueryFilterData& filterData,									void* userData) const;

		PX_INLINE 	void		overlapSphereMultiple	(const Gu::Sphere& worldSphere,												const PxSceneQueryFilterData& filterData, void* userData, PxU32 maxShapes) const;
		PX_INLINE 	void		overlapAABBMultiple		(const PxBounds3& worldBounds,												const PxSceneQueryFilterData& filterData, void* userData, PxU32 maxShapes) const;
		PX_INLINE 	void		overlapOBBMultiple		(const PxVec3& boxCenter, const PxVec3& boxExtents, const PxQuat& boxRot,	const PxSceneQueryFilterData& filterData, void* userData, PxU32 maxShapes) const;
		PX_INLINE 	void		overlapCapsuleMultiple	(const Gu::Capsule& worldCapsule,											const PxSceneQueryFilterData& filterData, void* userData, PxU32 maxShapes) const;

		PX_INLINE 	void		linearOBBSweepSingle				(const PxVec3& boxCenter, const PxVec3& boxExtents, const PxQuat& boxRot, const PxVec3& unitDir, const PxReal distance,	const PxSceneQueryFilterData& filterData, void* userData, PxSceneQueryFlags hintFlags) const;
		PX_INLINE 	void		linearCapsuleSweepSingle			(const Gu::Capsule& worldCapsule, const PxVec3& unitDir, const PxReal distance,											const PxSceneQueryFilterData& filterData, void* userData, PxSceneQueryFlags hintFlags) const;
		PX_INLINE 	void		linearOBBSweepMultiple				(const PxVec3& boxCenter, const PxVec3& boxExtents, const PxQuat& boxRot, const PxVec3& unitDir, const PxReal distance,	const PxSceneQueryFilterData& filterData, void* userData, PxSceneQueryFlags hintFlags) const;
		PX_INLINE 	void		linearCapsuleSweepMultiple			(const Gu::Capsule& worldCapsule, const PxVec3& unitDir, const PxReal distance,											const PxSceneQueryFilterData& filterData, void* userData, PxSceneQueryFlags hintFlags) const;
		PX_INLINE	void		linearCompoundGeometrySweepSingle	(const PxGeometry** geometryList, const PxTransform* poseList,															const PxFilterData* filterDataList, PxU32 geometryCount, const PxVec3& unitDir, const PxReal distance, PxSceneQueryFilterFlags filterFlags, void* userData, const PxSweepCache* sweepCache, PxSceneQueryFlags hintFlags) const;
		PX_INLINE	void		linearCompoundGeometrySweepMultiple	(const PxGeometry** geometryList, const PxTransform* poseList,															const PxFilterData* filterDataList, PxU32 geometryCount, const PxVec3& unitDir, const PxReal distance, PxSceneQueryFilterFlags filterFlags, void* userData, const PxSweepCache* sweepCache, PxSceneQueryFlags hintFlags) const;

		PX_INLINE 	void		cullObjects(PxU32 nbPlanes, const Gu::Plane* worldPlanes, const PxSceneQueryFilterData& filterData, void* userData) const;

	private:
		PX_INLINE	void		writeLinearCompoundGeometryData(SceneQueryID::Enum queryType, const PxGeometry** geometryList, const PxTransform* poseList, const PxFilterData* filterDataList, PxU32 geometryCount, const PxVec3& unitDir, const PxReal distance, PxSceneQueryFilterFlags filterFlags, void* userData, const PxSweepCache* sweepCache, PxSceneQueryFlags hintFlags) const;

		mutable Container	mBatchedQueries;
		mutable Container	mBatchedRaycastQueries;
		mutable Container	mBatchedSweepQueries;
		mutable Container	mBatchedOverlapQueries;
		mutable Container	mData;
		PxU32				mCurrentWord;
		PxU32				mCurrentRaycastWord;
		PxU32				mCurrentSweepWord;
		PxU32				mCurrentOverlapWord;
		mutable PxU32		mNbSweepQuery;
	};

} // namespace Sq


PX_INLINE PxU32 getNbContainerEntries(PxU32 byteSize)
{
	PxU32 nbEntries = byteSize / sizeof(PxU32);
	if ((byteSize & 0x00000003) > 0)
		nbEntries++;  // Not a multiple of 4, so add one
	return nbEntries;
}


//PX_INLINE const Sq::QueryData* Sq::BatchQueryStream::getQueryData()
//{
//	PxU32 bufferedDataSize = mBatchedQueries.GetNbEntries();
//	if(!bufferedDataSize)
//		return NULL;
//
//	return reinterpret_cast<QueryData*>(mBatchedQueries.GetEntries() + mCurrentWord);
//}

PX_INLINE const Sq::QueryData* Sq::BatchQueryStream::getNextQueryData()
{
	PxU32 bufferedDataSize = mBatchedQueries.GetNbEntries();
	if(mCurrentWord == bufferedDataSize)
		return NULL;

	const QueryData* ret = reinterpret_cast<QueryData*>(mBatchedQueries.GetEntries() + mCurrentWord);

	PxU32 size = ret->mSize;
	mCurrentWord += size;

	return ret;
}

PX_INLINE const Sq::QueryData* Sq::BatchQueryStream::getNextRaycastQueryData()
{
	PxU32 bufferedDataSize = mBatchedRaycastQueries.GetNbEntries();
	if(mCurrentRaycastWord == bufferedDataSize)
		return NULL;

	const QueryData* ret = reinterpret_cast<QueryData*>(mBatchedRaycastQueries.GetEntries() + mCurrentRaycastWord);

	PxU32 size = ret->mSize;
	mCurrentRaycastWord += size;

	return ret;
}

PX_INLINE const Sq::QueryData* Sq::BatchQueryStream::getNextRaycastQueryDataBatch(PxU32 nb)
{
	PxU32 bufferedDataSize = mBatchedRaycastQueries.GetNbEntries();
	if(mCurrentRaycastWord == bufferedDataSize)
		return NULL;

	const QueryData* ret = reinterpret_cast<QueryData*>(mBatchedRaycastQueries.GetEntries() + mCurrentRaycastWord);

	PxU32 size = ret->mSize * nb;
	mCurrentRaycastWord += size;

	return ret;
}


PX_INLINE const Sq::QueryData* Sq::BatchQueryStream::getNextSweepQueryData()
{
	PxU32 bufferedDataSize = mBatchedSweepQueries.GetNbEntries();
	if(mCurrentSweepWord == bufferedDataSize)
		return NULL;

	const QueryData* ret = reinterpret_cast<QueryData*>(mBatchedSweepQueries.GetEntries() + mCurrentSweepWord);

	PxU32 size = ret->mSize;
	mCurrentSweepWord += size;

	return ret;
}
PX_INLINE const Sq::QueryData* Sq::BatchQueryStream::getNextSweepQueryDataBatch(PxU32 nb)
{
	PxU32 bufferedDataSize = mBatchedSweepQueries.GetNbEntries();
	if(mCurrentSweepWord == bufferedDataSize)
		return NULL;

	const QueryData* ret = reinterpret_cast<QueryData*>(mBatchedSweepQueries.GetEntries() + mCurrentSweepWord);

	PxU32 size = ret->mSize * nb;
	mCurrentSweepWord += size;

	return ret;
}

PX_INLINE const Sq::QueryData* Sq::BatchQueryStream::getNextOverlapQueryData()
{
	PxU32 bufferedDataSize = mBatchedOverlapQueries.GetNbEntries();
	if(mCurrentOverlapWord == bufferedDataSize)
		return NULL;

	const QueryData* ret = reinterpret_cast<QueryData*>(mBatchedOverlapQueries.GetEntries() + mCurrentOverlapWord);

	PxU32 size = ret->mSize;
	mCurrentOverlapWord += size;

	return ret;
}
PX_INLINE const Sq::QueryData* Sq::BatchQueryStream::getNextOverlapQueryDataBatch(PxU32 nb)
{
	PxU32 bufferedDataSize = mBatchedOverlapQueries.GetNbEntries();
	if(mCurrentSweepWord == bufferedDataSize)
		return NULL;

	const QueryData* ret = reinterpret_cast<QueryData*>(mBatchedOverlapQueries.GetEntries() + mCurrentOverlapWord);

	PxU32 size = ret->mSize * nb;
	mCurrentOverlapWord += size;

	return ret;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

PX_INLINE void Sq::BatchQueryStream::raycastAny(const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist, const PxSceneQueryFilterData& filterData, Sq::SceneQueryShapeData* cache, void* userData) const
{
	const PxU32 dataSize = getNbContainerEntries(sizeof(RaycastQueryData));

	//RaycastAnyObjectQueryData* data = reinterpret_cast<RaycastAnyObjectQueryData*>(mBatchedQueries.Reserve(dataSize));
	RaycastQueryData* data = reinterpret_cast<RaycastQueryData*>(mBatchedRaycastQueries.Reserve(dataSize));
	data->mType			= SceneQueryID::QUERY_RAYCAST_ANY_OBJECT;
	data->mSize			= dataSize;
	data->mUserData		= userData;
	data->mFilterFlags	= filterData.flags;
	data->mFilterData	= filterData.data;
	data->mFilterDataValid = true;	//TODO: Always TRUE here
	//
	data->mRayOrigin	= rayOrigin;
	data->mRayDir		= rayDir;
	data->mMaxDist		= maxDist;
	//
	data->mCache		= cache;
	//data->mUserCache	= userCache;
	data->mHintFlags	= PxSceneQueryFlags();
}

PX_INLINE void Sq::BatchQueryStream::raycastSingle(const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist, PxSceneQueryFlags hintFlags, const PxSceneQueryFilterData& filterData, Sq::SceneQueryShapeData* cache, void* userData) const
{
	const PxU32 dataSize = getNbContainerEntries(sizeof(RaycastQueryData));

	//RaycastSingleQueryData* data = reinterpret_cast<RaycastSingleQueryData*>(mBatchedQueries.Reserve(dataSize));
	RaycastQueryData* data = reinterpret_cast<RaycastQueryData*>(mBatchedRaycastQueries.Reserve(dataSize));
	data->mType			= SceneQueryID::QUERY_RAYCAST_CLOSEST_OBJECT;
	data->mSize			= dataSize;
	data->mUserData		= userData;
	data->mFilterFlags	= filterData.flags;
	data->mFilterData	= filterData.data;
	data->mFilterDataValid = true;
	//
	data->mRayOrigin	= rayOrigin;
	data->mRayDir		= rayDir;
	data->mMaxDist		= maxDist;
	//
	data->mCache		= cache;
	//data->mUserCache	= userCache;
	data->mHintFlags	= hintFlags;
}

PX_INLINE void Sq::BatchQueryStream::raycastMultiple(const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist, PxSceneQueryFlags hintFlags, const PxSceneQueryFilterData& filterData, void* userData) const
{
	const PxU32 dataSize = getNbContainerEntries(sizeof(RaycastQueryData));

	//RaycastMultipleQueryData* data = reinterpret_cast<RaycastMultipleQueryData*>(mBatchedQueries.Reserve(dataSize));
	RaycastQueryData* data = reinterpret_cast<RaycastQueryData*>(mBatchedRaycastQueries.Reserve(dataSize));
	data->mType			= SceneQueryID::QUERY_RAYCAST_ALL_OBJECTS;
	data->mSize			= dataSize;
	data->mUserData		= userData;
	data->mFilterFlags	= filterData.flags;	
	data->mFilterData	= filterData.data;
	data->mFilterDataValid = true;
	//
	data->mRayOrigin	= rayOrigin;
	data->mRayDir		= rayDir;
	data->mMaxDist		= maxDist;
	//
	data->mHintFlags	= hintFlags;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

PX_INLINE void Sq::BatchQueryStream::overlapSphereMultiple(const Gu::Sphere& worldSphere, const PxSceneQueryFilterData& filterData, void* userData, PxU32 maxShapes) const
{
	const PxU32 dataSize = getNbContainerEntries(sizeof(UnifiedQueryData));

	SphereQueryData* data = reinterpret_cast<SphereQueryData*>(mBatchedOverlapQueries.Reserve(dataSize));
	data->mType			= SceneQueryID::QUERY_OVERLAP_SPHERE_ALL_OBJECTS;
	data->mSize			= dataSize;
	data->mUserData		= userData;
	data->mFilterFlags	= filterData.flags;	
	data->mFilterData	= filterData.data;
	data->mFilterDataValid = true;
	//
	data->mWorldSphere	= worldSphere;
	//data->mMaxShapes	= maxShapes;
}

PX_INLINE void Sq::BatchQueryStream::overlapAABBMultiple(const PxBounds3& worldBounds, const PxSceneQueryFilterData& filterData, void* userData, PxU32 maxShapes) const
{
	const PxU32 dataSize = getNbContainerEntries(sizeof(UnifiedQueryData));

	AABBQueryData* data = reinterpret_cast<AABBQueryData*>(mBatchedOverlapQueries.Reserve(dataSize));
	data->mType			= SceneQueryID::QUERY_OVERLAP_AABB_ALL_OBJECTS;
	data->mSize			= dataSize;
	data->mUserData		= userData;
	data->mFilterFlags	= filterData.flags;	
	data->mFilterData	= filterData.data;
	data->mFilterDataValid = true;
	//
	data->mWorldBounds	= worldBounds;
	//data->mMaxShapes	= maxShapes;
}

PX_INLINE void Sq::BatchQueryStream::overlapOBBMultiple(
	const PxVec3& boxCenter, const PxVec3& boxExtents, const PxQuat& boxRot,
	const PxSceneQueryFilterData& filterData, void* userData, PxU32 maxShapes) const
{
	PX_CHECK(boxRot.isValid());

 	const PxU32 dataSize = getNbContainerEntries(sizeof(UnifiedQueryData));
   
  	OBBQueryData* data = reinterpret_cast<OBBQueryData*>(mBatchedOverlapQueries.Reserve(dataSize));

	data->mType			= SceneQueryID::QUERY_OVERLAP_OBB_ALL_OBJECTS;
  	data->mSize			= dataSize;
	data->mUserData		= userData;
	data->mFilterFlags	= filterData.flags;	
	data->mFilterData	= filterData.data;
	data->mFilterDataValid = true;
	//
	data->mBoxCenter	= boxCenter;
	data->mBoxExtents	= boxExtents;
	data->mBoxRot		= boxRot;

	//data->mMaxShapes	= maxShapes;
}

PX_INLINE void Sq::BatchQueryStream::overlapCapsuleMultiple(const Gu::Capsule& worldCapsule, const PxSceneQueryFilterData& filterData, void* userData, PxU32 maxShapes) const
{
	const PxU32 dataSize = getNbContainerEntries(sizeof(UnifiedQueryData));

	CapsuleQueryData* data = reinterpret_cast<CapsuleQueryData*>(mBatchedOverlapQueries.Reserve(dataSize));
	data->mType			= SceneQueryID::QUERY_OVERLAP_CAPSULE_ALL_OBJECTS;
	data->mSize			= dataSize;
	data->mUserData		= userData;
	data->mFilterFlags	= filterData.flags;	
	data->mFilterData	= filterData.data;
	data->mFilterDataValid = true;
	//
	data->mWorldCapsule	= worldCapsule;
	//data->mMaxShapes	= maxShapes;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

PX_INLINE void Sq::BatchQueryStream::cullObjects(PxU32 nbPlanes, const Gu::Plane* worldPlanes, const PxSceneQueryFilterData& filterData, void* userData) const
{
	const PxU32 neededBytes = sizeof(UnifiedQueryData);
	const PxU32 dataSize = getNbContainerEntries(neededBytes);

	CullQueryData* data = reinterpret_cast<CullQueryData*>(mBatchedOverlapQueries.Reserve(dataSize));
	data->mType			= SceneQueryID::QUERY_CULL_OBJECTS;
	data->mSize			= dataSize;
	data->mSize			= dataSize;
	data->mUserData		= userData;
	data->mFilterFlags	= filterData.flags;	
	data->mFilterData	= filterData.data;
	data->mFilterDataValid = true;
	//
	data->mNbPlanes		= nbPlanes;

	Gu::Plane* batchedPlanes =  reinterpret_cast<Gu::Plane*>(mData.Reserve(nbPlanes * sizeof(Gu::Plane)));
	data->mPlanes = batchedPlanes;

	for(PxU32 i=0;i<nbPlanes;i++)
		batchedPlanes[i] = worldPlanes[i];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

PX_INLINE void Sq::BatchQueryStream::linearOBBSweepSingle(
	const PxVec3& boxCenter, const PxVec3& boxExtents, const PxQuat& boxRot, const PxVec3& unitDir, const PxReal distance,
	const PxSceneQueryFilterData& filterData, void* userData, PxSceneQueryFlags hintFlags) const
{
	PX_CHECK_VALID(boxCenter); PX_CHECK_VALID(boxExtents); PX_CHECK_VALID(boxRot); PX_CHECK_VALID(unitDir);
	const PxU32 dataSize = getNbContainerEntries(sizeof(UnifiedQueryData));

	LinearOBBSweepQueryData* data = reinterpret_cast<LinearOBBSweepQueryData*>(mBatchedSweepQueries.Reserve(dataSize));
	data->mType			= SceneQueryID::QUERY_LINEAR_OBB_SWEEP_CLOSEST_OBJECT;
	data->mSize			= dataSize;
	data->mUserData		= userData;
	data->mHintFlags	= hintFlags;	
	data->mFilterData	= filterData.data;
	data->mFilterDataValid = true;
	//
	data->mBoxCenter	= boxCenter;
	data->mBoxExtents	= boxExtents;
	data->mBoxRot		= boxRot;

	data->mUnitDir		= unitDir;
	data->mDistance		= distance;
	data->mFilterFlags	= filterData.flags;

	mNbSweepQuery++;
}

PX_INLINE void Sq::BatchQueryStream::linearCapsuleSweepSingle(
	const Gu::Capsule& worldCapsule, const PxVec3& unitDir, const PxReal distance,
	const PxSceneQueryFilterData& filterData, void* userData, PxSceneQueryFlags hintFlags) const
{
	PX_CHECK_VALID(unitDir);
	const PxU32 dataSize = getNbContainerEntries(sizeof(UnifiedQueryData));

	LinearCapsuleSweepQueryData* data = reinterpret_cast<LinearCapsuleSweepQueryData*>(mBatchedSweepQueries.Reserve(dataSize));
	data->mType			= SceneQueryID::QUERY_LINEAR_CAPSULE_SWEEP_CLOSEST_OBJECT;
	data->mSize			= dataSize;
	data->mUserData		= userData;
	data->mHintFlags	= hintFlags;	
	data->mFilterData	= filterData.data;
	data->mFilterDataValid = true;
	//
	data->mWorldCapsule	= worldCapsule;
	data->mUnitDir		= unitDir;
	data->mDistance		= distance;
	data->mFilterFlags	= filterData.flags;

	mNbSweepQuery++;
}

PX_INLINE void Sq::BatchQueryStream::linearCompoundGeometrySweepSingle(
	const PxGeometry** geometryList, const PxTransform* poseList, const PxFilterData* filterDataList, PxU32 geometryCount,
	const PxVec3& unitDir, const PxReal distance, PxSceneQueryFilterFlags filterFlags, void* userData, const PxSweepCache* sweepCache,
	PxSceneQueryFlags hintFlags) const
{
	PX_CHECK_VALID(*poseList); PX_CHECK_VALID(unitDir);
	writeLinearCompoundGeometryData(SceneQueryID::QUERY_LINEAR_COMPOUND_GEOMETRY_SWEEP_CLOSEST_OBJECT, geometryList, poseList, filterDataList, geometryCount, unitDir, distance, filterFlags, userData, sweepCache, hintFlags);
	mNbSweepQuery++;
}

PX_INLINE void Sq::BatchQueryStream::linearOBBSweepMultiple(
	const PxVec3& boxCenter, const PxVec3& boxExtents, const PxQuat& boxRot, const PxVec3& unitDir, const PxReal distance,
	const PxSceneQueryFilterData& filterData, void* userData, PxSceneQueryFlags hintFlags) const
{
	PX_CHECK_VALID(boxCenter); PX_CHECK_VALID(boxExtents); PX_CHECK_VALID(boxRot); PX_CHECK_VALID(unitDir);

	const PxU32 dataSize = getNbContainerEntries(sizeof(UnifiedQueryData));

	LinearOBBSweepQueryData* data = reinterpret_cast<LinearOBBSweepQueryData*>(mBatchedSweepQueries.Reserve(dataSize));
	data->mType			= SceneQueryID::QUERY_LINEAR_OBB_SWEEP_ALL_OBJECTS;
	data->mSize			= dataSize;
	data->mUserData		= userData;
	data->mHintFlags	= hintFlags;
	data->mFilterData	= filterData.data;
	data->mFilterDataValid = true;
	//
	data->mBoxCenter	= boxCenter;
	data->mBoxExtents	= boxExtents;
	data->mBoxRot		= boxRot;

	data->mUnitDir		= unitDir;
	data->mDistance		= distance;
	data->mFilterFlags	= filterData.flags;

	mNbSweepQuery++;
}

PX_INLINE void Sq::BatchQueryStream::linearCapsuleSweepMultiple(
	const Gu::Capsule& worldCapsule, const PxVec3& unitDir, const PxReal distance,
	const PxSceneQueryFilterData& filterData, void* userData, PxSceneQueryFlags hintFlags) const
{
	PX_CHECK_VALID(unitDir);
	const PxU32 dataSize = getNbContainerEntries(sizeof(UnifiedQueryData));

	LinearCapsuleSweepQueryData* data = reinterpret_cast<LinearCapsuleSweepQueryData*>(mBatchedSweepQueries.Reserve(dataSize));
	data->mType			= SceneQueryID::QUERY_LINEAR_CAPSULE_SWEEP_ALL_OBJECTS;
	data->mSize			= dataSize;
	data->mUserData		= userData;
	data->mHintFlags	= hintFlags;	
	data->mFilterData	= filterData.data;
	data->mFilterDataValid = true;
	//
	data->mWorldCapsule	= worldCapsule;
	data->mUnitDir		= unitDir;
	data->mDistance		= distance;
	data->mFilterFlags	= filterData.flags;

	mNbSweepQuery++;
}

PX_INLINE void Sq::BatchQueryStream::linearCompoundGeometrySweepMultiple(
	const PxGeometry** geometryList, const PxTransform* poseList, const PxFilterData* filterDataList, PxU32 geometryCount,
	const PxVec3& unitDir, const PxReal distance, PxSceneQueryFilterFlags filterFlags, void* userData, const PxSweepCache* sweepCache, PxSceneQueryFlags hintFlags) const
{
	// clow: no need to test inputs here, it's done in the writeLinearCompoundGeometryData function already
	writeLinearCompoundGeometryData(SceneQueryID::QUERY_LINEAR_COMPOUND_GEOMETRY_SWEEP_ALL_OBJECTS, geometryList, poseList,
		filterDataList, geometryCount, unitDir, distance, filterFlags, userData, sweepCache, hintFlags);
	mNbSweepQuery++;
}

PX_INLINE void Sq::BatchQueryStream::writeLinearCompoundGeometryData(
	SceneQueryID::Enum queryType, const PxGeometry** geometryList, const PxTransform* poseList,
	const PxFilterData* filterDataList, PxU32 geometryCount, const PxVec3& unitDir, const PxReal distance,
	PxSceneQueryFilterFlags filterFlags, void* userData, const PxSweepCache* sweepCache, PxSceneQueryFlags hintFlags) const
{
	PX_CHECK_VALID(unitDir);

	PxU32 dataByteSize = sizeof(Gu::GeometryUnion)*geometryCount + sizeof(PxTransform)*geometryCount;
	if (filterDataList)
		dataByteSize += sizeof(PxFilterData)*geometryCount;

	PxU32 byteSize = sizeof(UnifiedQueryData);
	const PxU32 queryDataSize = getNbContainerEntries(byteSize);
	const PxU32 compoundDataSize = getNbContainerEntries(dataByteSize);

	PxU8* data				= reinterpret_cast<PxU8*>(mBatchedSweepQueries.Reserve(queryDataSize));

	LinearCompoundGeometrySweepQueryData* queryData = reinterpret_cast<LinearCompoundGeometrySweepQueryData*>(data);

	data					=  reinterpret_cast<PxU8*>(mData.Reserve(compoundDataSize));
	Gu::GeometryUnion* geoms = reinterpret_cast<Gu::GeometryUnion*>(data);

	data					+= sizeof(Gu::GeometryUnion)*geometryCount;

	PxTransform* poses		= reinterpret_cast<PxTransform*>(data);
	data					+= sizeof(PxTransform)*geometryCount;

	for(PxU32 i=0; i < geometryCount; i++)
	{
		PX_CHECK_VALID(poseList[i]);
		geoms[i].set(*geometryList[i]);
		poses[i] = poseList[i];
	}

	PxFilterData* fds = NULL;
	if (filterDataList)
	{
		fds = reinterpret_cast<PxFilterData*>(data);
		for(PxU32 i=0; i < geometryCount; i++)
		{
			fds[i] = filterDataList[i];
		}
	}

	queryData->mType				= queryType;
	queryData->mSize				= queryDataSize;
	queryData->mUserData			= userData;
	queryData->mHintFlags			= hintFlags;
	queryData->mFilterDataValid		= (filterDataList != NULL);
	//
	queryData->mGeometryListOffset	= pointerToOffset(geoms);
	queryData->mPoseListOffset		= pointerToOffset(poses);
	queryData->mFilterDataListOffset= pointerToOffset(fds);
	queryData->mGeometryCount		= geometryCount;

	queryData->mSweepCache	= sweepCache;
	queryData->mUnitDir		= unitDir;
	queryData->mDistance	= distance;
	queryData->mFilterFlags	= filterFlags;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

}

/** @} */
#endif
