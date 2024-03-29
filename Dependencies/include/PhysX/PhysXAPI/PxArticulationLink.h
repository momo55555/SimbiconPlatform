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


#ifndef PX_PHYSICS_NX_ARTICULATION_LINK
#define PX_PHYSICS_NX_ARTICULATION_LINK
/** \addtogroup physics 
@{ */

#include "PxPhysX.h"
#include "PxArticulationJoint.h"
#include "PxRigidBody.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxArticulationLink : public PxRigidBody
{
	protected:
// PX_SERIALIZATION
								PxArticulationLink(PxRefResolver& v) : PxRigidBody(v)		{}
								PX_DECLARE_SERIAL_RTTI(PxArticulationLink, PxRigidBody)
//~PX_SERIALIZATION
	PX_INLINE					PxArticulationLink() : PxRigidBody() {}
	virtual						~PxArticulationLink()	{}

	public:
	/**
	\brief Deletes the articulation link.
	
	\note Only a leaf articulation link can be released
	
	Do not keep a reference to the deleted instance.

	@see PxArticulation::createLink()
	*/
	virtual		void			release() = 0;


	/**
	\brief get the articulation to which this articulation link belongs

	\return the articulation to which this link belongs
	*/
	virtual		PxArticulation&	getArticulation() const = 0;

	/**
	\brief Get the joint which connects this link to its parent.
	
	\return The joint connecting the link to the parent. NULL for the root link.

	@see PxArticulationJoint
	*/
	virtual		PxArticulationJoint*	getInboundJoint() const = 0;

	/**
	\brief Get number of child links.

	\return the number of child links

	@see getChildren()
	*/
	virtual		PxU32			getNbChildren() const = 0;

	/**
	\brief Retrieve all the child links.

	\param[out] userBuffer The buffer to receive articulation link pointers.
	\param[in] bufferSize Size of provided user buffer.
	\return Number of articulation links written to the buffer.

	@see getNbChildren()
	*/
	virtual		PxU32			getChildren(PxArticulationLink** userBuffer, PxU32 bufferSize) const = 0;
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
