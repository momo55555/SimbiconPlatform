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


#ifndef PX_PHYSICS_NXBOXCONTROLLER
#define PX_PHYSICS_NXBOXCONTROLLER
/** \addtogroup character
  @{
*/

#include "characterkinematic/PxCharacter.h"
#include "characterkinematic/PxController.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Descriptor for a box character controller.

@see PxBoxController PxControllerDesc
*/
class PxBoxControllerDesc : public PxControllerDesc
{
public:
	/**
	\brief constructor sets to default.
	*/
	PX_INLINE								PxBoxControllerDesc();
	PX_INLINE virtual						~PxBoxControllerDesc();

	/**
	\brief (re)sets the structure to the default.
	*/
	PX_INLINE virtual	void				setToDefault();
	/**
	\brief returns true if the current settings are valid

	\return True if the descriptor is valid.
	*/
	PX_INLINE virtual	bool				isValid()		const;

	/**
	\brief The extents of the box controller.

	The extents of the controller specify the half width/height/depth for each axis.

	<b>Default:</b> [0.5,1.0,0.5]
	*/
	PxVec3				extents;
};

PX_INLINE PxBoxControllerDesc::PxBoxControllerDesc() : PxControllerDesc(PxControllerShapeType::eBOX)
{
	extents.x = 0.5f;
	extents.y = 1.0f;
	extents.z = 0.5f;
}

PX_INLINE PxBoxControllerDesc::~PxBoxControllerDesc()
{
}

PX_INLINE void PxBoxControllerDesc::setToDefault()
{
	*this = PxBoxControllerDesc();
}

PX_INLINE bool PxBoxControllerDesc::isValid() const
{
	if(!PxControllerDesc::isValid())	return false;
	if(extents.x<=0.0f)					return false;
	if(extents.y<=0.0f)					return false;
	if(extents.z<=0.0f)					return false;
	return true;
}

/**
\brief Box character controller.

@see PxBoxControllerDesc PxController
*/
class PxBoxController : public PxController
{
protected:
	PX_INLINE					PxBoxController()	{}
	virtual						~PxBoxController()	{}

public:

	/**
	\brief Gets controller's extents.

	\return The extents of the controller.

	@see PxBoxControllerDesc.extents setExtents()
	*/
	virtual		const PxVec3&	getExtents() const = 0;

	/**
	\brief Resets controller's extents.

	\warning this doesn't check for collisions.

	\param[in] extents The new extents for the controller.
	\return Currently always true.

	@see PxBoxControllerDesc.extents getExtents()
	*/
	virtual		bool			setExtents(const PxVec3& extents) = 0;
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
