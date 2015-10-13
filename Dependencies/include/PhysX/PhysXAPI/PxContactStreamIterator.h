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


#ifndef PX_PHYSICS_NXCONTACTSTREAMITERATOR
#define PX_PHYSICS_NXCONTACTSTREAMITERATOR
/** \addtogroup physics
@{
*/

#include "PxPhysX.h"
#include "PxAssert.h"
#include "PxFlags.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxShape;

typedef char* PxContactStream;
typedef const char* PxConstContactStream;

/**
\brief Flags which describe a contact

@see PxContactStreamIterator
*/
struct PxShapePairStreamFlag
{
	enum Enum
	{
		eHAS_FEATURES_PER_POINT		= (1<<0),	//!< the stream includes per-point feature data
		eHAS_32BIT_FEATURES			= (1<<1),	//!< features are stored as 32 bit values (else 16 bit)
		eHAS_FORCES_PER_POINT		= (1<<2),	//!< the stream includes per-point forces
		eDELETED_SHAPE_0			= (1<<3),	//!< shape with index 0 has been deleted
		eDELETED_SHAPE_1			= (1<<4),	//!< shape with index 1 has been deleted
	};
};

/**
\brief collection of set bits defined in PxShapePairStreamFlag.

@see PxShapePairStreamFlag
*/
typedef PxFlags<PxShapePairStreamFlag::Enum,PxU16> PxShapePairStreamFlags;
PX_FLAGS_OPERATORS(PxShapePairStreamFlag::Enum,PxU16);


/**
\brief PxContactStreamIterator is for iterating through packed contact streams.

<p>The user code to use this iterator looks like this:
\code
void MyUserContactInfo::onContact(PxContactPair & pair, PxU32 events)
{
PxContactStreamIterator i(pair.stream);

while(i.goNextPair()) // user can call getNumPairs() here 
{
while(i.goNextPatch()) // user can also call getShape(), isDeletedShape() and getNumPatches() here
{
while(i.goNextPoint()) //user can also call getPatchNormal() and getNumPoints() here
{
//user can also call getPoint() and getSeparation() here
}
}
}
}
\endcode
</p>

\note It is NOT OK to skip any of the iteration calls. For example, you may NOT put a break or a continue
statement in any of the above blocks, short of completely aborting the iteration and deleting the 
PxContactStreamIterator object.

\note The user should not rely on the exact geometry or positioning of contact points. The SDK is free
to re-organise, merge or move contact points as long as the overall physical simulation is not affected.</p>

<h3>Visualizations:</h3>
\li #PxVisualizationParameter::eCONTACT_POINT
\li #PxVisualizationParameter::eCONTACT_NORMAL
\li #PxVisualizationParameter::eCONTACT_ERROR
\li #PxVisualizationParameter::eCONTACT_FORCE

<b>Platform:</b>
\li PC SW: Yes
\li PS3  : Yes
\li XB360: Yes
\li WII	 : Yes

@see PxConstContactStream PxSimulationEventCallback
*/

class PxContactStreamIterator
{
public:
	/**
	\brief Starts the iteration, and returns the number of pairs.

	\param[in] streamIt

	@see PxConstContactStream
	*/
	PX_INLINE PxContactStreamIterator(PxConstContactStream streamIt);

	//iteration:


	/**
	\brief Goes on to the next pair, silently skipping invalid pairs.

	Returns false if there are no more pairs. Note that getNumPairs() also includes invalid pairs in the count.

	Once goNextPoint() returns false, the user should not call it again.

	\return True if there are more pairs.

	@see getNumPairs() getShape()
	*/
	PX_INLINE bool goNextPair();	

	/**
	\brief Goes on to the next patch (contacts with the same normal).

	Returns false if there are no more. Once goNextPatch() returns false, the user should
	not call it again until they move to the next pair.

	\return True if there are more patches.

	@see getPatchNormal()
	*/
	PX_INLINE bool goNextPatch();

	/**
	\brief Goes on to the next contact point.

	Returns false if there are no more. Once goNextPoint() returns false, the user should
	not call it again unil they move to the next patch.

	\return True if there are more contact points.

	@see getPoint()
	*/
	PX_INLINE bool goNextPoint();

	//accessors:

	/**
	\brief Returns the number of pairs in the structure. 

	May be called at any time.

	\return The number of pairs in the struct (including invalid pairs).

	@see goNextPair()
	*/
	PX_INLINE PxU32 getNumPairs();

	/**
	\brief Retrieves the shapes for the current pair.

	May be called after goNextPair() returned true. ShapeIndex is 0 or 1.

	\note The shape pointers might reference deleted shapes. Check through #isDeletedShape() to see
	whether that is the case. Do not dereference a pointer to a deleted shape. The pointer to a
	deleted shape is only provided such that user data structures which might depend on the pointer
	value can be updated.

	\param[in] shapeIndex Used to choose which of the pair of shapes to retrieve(set to 0 or 1).
	\return The shape specified by shapeIndex.

	@see goNextPair() PxShape
	*/
	PX_INLINE PxShape* getShape(PxU32 shapeIndex);

	/**
	\brief Specifies for each shape of the pair if it has been deleted.

	May be called after goNextPair() returned true. ShapeIndex is 0 or 1.

	Before dereferencing the shape pointers of the contact pair you might want to use this function
	to check if the pointers reference deleted shapes. This will be the case if a shape gets deleted
	whose actor requested PxPairFlag::eNOTIFY_TOUCH_LOST or PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST events.

	\param[in] shapeIndex Used to choose which of the shapes to check for deletion(set to 0 or 1).
	\return True if the shape has been deleted, else false

	@see goNextPair() PxShape
	*/
	PX_INLINE bool isDeletedShape(PxU32 shapeIndex);

	/**
	\brief Retrieves the shape flags for the current pair.

	May be called after goNextPair() returned true

	\return The shape flags for the current pair. See #PxShapePairStreamFlag.

	@see PxShapePairStreamFlag goNextPair()
	*/
	PX_INLINE PxShapePairStreamFlags getShapeFlags(); 

	/**
	\brief Retrieves the number of patches for the current pair.

	May be called after goNextPair() returned true

	\return The number of patches in this pair.

	@see goNextPatch()
	*/
	PX_INLINE PxU32 getNumPatches();

	/**
	\brief Retrieves the number of remaining patches.

	May be called after goNextPair() returned true

	\return The number of patches remaining in this pair.

	@see goNextPatch() getNumPatches()
	*/
	PX_INLINE PxU32 getNumPatchesRemaining();

	/**
	\brief Retrieves the patch normal.

	May be called after goNextPatch() returned true

	\return The patch normal.

	@see goNextPatch()
	*/
	PX_INLINE const PxVec3& getPatchNormal();

	/**
	\brief Retrieves the number of points in the current patch.

	May be called after goNextPatch() returned true

	\return The number of points in the current patch.

	@see goNextPoint() getNumPointsRemaining()
	*/
	PX_INLINE PxU32 getNumPoints();

	/**
	\brief Retrieves the number of points remaining in the current patch.

	May be called after goNextPatch() returned true

	\return The number of points remaining in the current patch.

	@see goNextPoint() getNumPoints()
	*/
	PX_INLINE PxU32 getNumPointsRemaining();

	/**
	\brief Returns the contact point position.

	May be called after goNextPoint() returned true

	\return the current contact point

	@see getShapeFlags() goNextPoint() getNumPoints() getSeparation() getFeatureIndex0()
	*/
	PX_INLINE const PxVec3& getPoint();

	/**
	\brief Return the separation for the contact point.

	May be called after goNextPoint() returned true

	\return the seperation distance for the current point.

	@see goNextPoint() getPoint()
	*/
	PX_INLINE PxReal getSeparation();

	/**
	\brief Retrieves the feature index.

	Feature indices are only defined for triangle mesh and heightfield shapes. 

	A feature index for a triangle mesh shape is the pre cooked triangle index. For a
	heightfield shape a feature index is a triangle index as specified on creation, including
	holes in the index.

	May be called after goNextPoint() returned true
	If getShapeFlags()&PxShapePairStreamFlag::eHAS_FEATURES_PER_POINT is specified, this method returns a feature belonging to shape 0,

	\return The feature index on shape 0 for the current point.

	@see PxPairFlag::eNOTIFY_CONTACT_FEATURE_INDICES_PER_POINT goNextPoint() getPoint() getSeparation() getFeatureIndex1()
	*/
	PX_INLINE PxU32 getFeatureIndex0();

	/**
	\brief Retrieves the feature index.

	may be called after goNextPoint() returned true
	If getShapeFlags()&PxShapePairStreamFlag::eHAS_FEATURES_PER_POINT is specified, this method returns a feature belonging to shape 1,

	\return The feature index on shape1 for the current point.

	@see PxPairFlag::eNOTIFY_CONTACT_FEATURE_INDICES_PER_POINT goNextPoint() getPoint() getSeparation() getFeatureIndex0()
	*/
	PX_INLINE PxU32 getFeatureIndex1();


	/**
	\brief Retrieves the point normal force.

	May be called after goNextPoint() returned true

	If getShapeFlags()&PxShapePairStreamFlag::eHAS_FORCES_PER_POINT is true (this is the case if PxPairFlag::eNOTIFY_CONTACT_FORCE_PER_POINT is raised for the pair), 
	this method returns the contact force at this contact point.
	Returns 0 otherwise.

	\return The contact force for the current point.

	@see PxPairFlag::eNOTIFY_CONTACT_FORCE_PER_POINT getShapeFlags goNextPoint() getPoint()
	*/
	PX_INLINE PxReal getPointNormalForce();

private:

	template <typename T, size_t S> PX_INLINE const T* streamRef()
	{
		const T* result = reinterpret_cast<const T*>(streamIt);
		streamIt += S;
		return result;
	}

	template <typename T> PX_INLINE const T* streamRef()
	{
		return streamRef<T, sizeof(T)>();
	}

	template <typename T, size_t S> PX_INLINE const T& streamRead()
	{
		return *streamRef<T,S>();
	}

	template <typename T> PX_INLINE const T& streamRead()
	{
		return *streamRef<T>();
	}



	// iterator variables -- are only initialized by streamIt iterator calls:
	// Note: structs are used so that we can leave the iterators vars on the stack as they were
	// and the user's iteration code doesn't have to change when we add members.

	PxU32					numPairs;				// number of pairs in the structure
	PxShape*				shapes[2];				// shapes for the current pair
	PxShapePairStreamFlags	shapeFlags;				// shape flags for the current pair.	
	PxU16					numPatches;				// number of patches for the current pair.
	
	const PxVec3*			patchNormal;			// patch normal.
	PxU32					numPoints;				// number of points in the current patch.

	const PxVec3*			point;					// contact point position.
	PxReal					separation;				// separation for the contact point.
	PxU32					featureIndex0;			// feature index on shape 0.
	PxU32					featureIndex1;			// feature index on shape 1.
	
	PxU32					numPairsRemaining;		// number of pairs remaining in the streamIt
	PxU32					numPatchesRemaining;	// number of contact patches remaining for the current pair	
	PxU32					numPointsRemaining;		// number of contact points remaining in the current patch

protected:
	/**
	\brief Normal force for the current point

	Only exists if (shapeFlags & eHAS_FORCES_PER_POINT)
	*/
	const PxReal* pointNormalForce;

	/**
	\brief The associated streamIt
	*/
	PxConstContactStream streamIt;
};

PX_INLINE PxContactStreamIterator::PxContactStreamIterator(PxConstContactStream it)
: streamIt(it)
{
	numPairsRemaining = numPairs = streamIt ? streamRead<PxU32>() : 0;
}

PX_INLINE PxU32 PxContactStreamIterator::getNumPairs()
{
	return numPairs;
}

PX_INLINE PxShape* PxContactStreamIterator::getShape(PxU32 shapeIndex)
{
	PX_ASSERT(shapeIndex<=1);
	return shapes[shapeIndex];
}

PX_INLINE bool PxContactStreamIterator::isDeletedShape(PxU32 shapeIndex)
{
	PX_ASSERT(shapeIndex<=1);
	if (shapeIndex == 0)
		return (shapeFlags & PxShapePairStreamFlag::eDELETED_SHAPE_0);
	else
		return (shapeFlags & PxShapePairStreamFlag::eDELETED_SHAPE_1);
}

PX_INLINE PxShapePairStreamFlags PxContactStreamIterator::getShapeFlags()
{
	return shapeFlags;
}

PX_INLINE PxU32 PxContactStreamIterator::getNumPatches()
{
	return numPatches;
}

PX_INLINE PxU32 PxContactStreamIterator::getNumPatchesRemaining()
{
	return numPatchesRemaining;
}

PX_INLINE const PxVec3& PxContactStreamIterator::getPatchNormal()
{
	return *patchNormal;
}

PX_INLINE PxU32 PxContactStreamIterator::getNumPoints()
{
	return numPoints;
}

PX_INLINE PxU32 PxContactStreamIterator::getNumPointsRemaining()
{
	return numPointsRemaining;
}

PX_INLINE const PxVec3& PxContactStreamIterator::getPoint()
{
	return *point;
}

PX_INLINE PxReal PxContactStreamIterator::getSeparation()
{
	return separation;
}

PX_INLINE PxU32 PxContactStreamIterator::getFeatureIndex0()
{
	return featureIndex0;
}
PX_INLINE PxU32 PxContactStreamIterator::getFeatureIndex1()
{
	return featureIndex1;
}

PX_INLINE PxReal PxContactStreamIterator::getPointNormalForce()
{
	return pointNormalForce ? *pointNormalForce : 0;
}

PX_INLINE bool PxContactStreamIterator::goNextPair()
{
	while (numPairsRemaining--)
	{
		// Skip internal shape IDs
		streamRead<PxU32>();
		streamRead<PxU32>();

		shapes[0] = streamRead<PxShape*>();
		shapes[1] = streamRead<PxShape*>();

		PxU32 t = streamRead<PxU32>();

		numPatchesRemaining = numPatches = PxU16(t & 0xffff);
		shapeFlags = PxShapePairStreamFlags(PxU16(t >> 16));
		return true;
	}
	return false;
}

PX_INLINE bool PxContactStreamIterator::goNextPatch()
{
	if (numPatchesRemaining--)
	{
		patchNormal = streamRef<PxVec3, 3 * sizeof(PxReal)>();
		numPointsRemaining = numPoints = streamRead<PxU32>();
		return true;
	}
	else
		return false;
}

PX_INLINE bool PxContactStreamIterator::goNextPoint()
{
	if (numPointsRemaining--)
	{
		// Get contact point
		point = streamRef<PxVec3, 3*sizeof(PxReal)>();
		separation = streamRead<PxReal>();

		if (shapeFlags & PxShapePairStreamFlag::eHAS_FORCES_PER_POINT)
			 pointNormalForce = streamRef<PxReal>();
		else
			pointNormalForce = 0;	//there is no contact force.


		if (shapeFlags & PxShapePairStreamFlag::eHAS_FEATURES_PER_POINT)
		{
			if(shapeFlags & PxShapePairStreamFlag::eHAS_32BIT_FEATURES)
			{
				featureIndex0 = streamRead<PxU32>();
				featureIndex1 = streamRead<PxU32>();
			}
			else
			{
				featureIndex0 = streamRead<PxU32>();
				featureIndex1 = featureIndex0>>16;
				featureIndex0 &= 0xffff;

				if (featureIndex0 == 0xffff)
					featureIndex0 = 0xffffffff;
				if (featureIndex1 == 0xffff)
					featureIndex1 = 0xffffffff;
			}
		}
		else
		{
			featureIndex0 = 0xffffffff;
			featureIndex1 = 0xffffffff;
		}

		return true;
	}
	else
		return false;
}

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
