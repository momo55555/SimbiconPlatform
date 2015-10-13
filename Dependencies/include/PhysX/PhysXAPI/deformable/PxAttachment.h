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


#ifndef PX_PHYSICS_NX_ATTACHMENT
#define PX_PHYSICS_NX_ATTACHMENT

#include "PxPhysX.h"
// PX_SERIALIZATION
	#include "common/PxSerialFramework.h"
//~PX_SERIALIZATION

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxDeformable;
class PxShape;
class PxScene;

// this struct should eventually replace PxDeformableVertexAttachmentFlag
struct PxAttachmentFlag
{
	enum Enum
	{
		/**
		\brief The attachment interacts in both ways.
		Todo: This needs to go away and be replaced with dominance.
		*/
		eTWOWAY			= (1<<0),

		/**
		\brief The attachment is tearable.
		*/
		eTEARABLE		= (1<<1),

		/**
		\brief The attachment has been torn.
		*/
		eTORN			= (1<<2),
	};
};

PX_DEPRECATED class PxAttachment
// PX_SERIALIZATION
	: public PxSerializable
//~PX_SERIALIZATION
{
public:
// PX_SERIALIZATION
							PxAttachment()											{}
							PxAttachment(PxRefResolver& v) :	PxSerializable(v)	{}
							PX_DECLARE_SERIAL_RTTI(PxAttachment, PxSerializable)
//~PX_SERIALIZATION
	virtual void			release() = 0;

	/**
	\brief Retrieves the scene which this attachment belongs to.

	\return Owner Scene. NULL if not part of a scene.

	@see PxScene
	*/
	virtual PxScene*		getScene() const = 0;

	/**
	\brief Returns the deformable whose vertices are attached.

	\return The deformable. NULL if the deformable has been released.

	@see PxDeformable
	*/
	virtual PxDeformable*	getDeformable() = 0;

	/**
	\brief Returns the shape the deformable vertices are attached to.

	\return Shape the deformable vertices are attached to or NULL if attached to global space. Also NULL if the shape has been released.

	@see PxShape
	*/
	virtual PxShape*		getShape() = 0;

	/**
	\brief Returns the number of deformable vertices attached to the shape.

	\return Number of deformable vertices attached to the shape.

	@see PxDeformable getVertexIds()
	*/
	virtual PxU32			getNbVertices() const = 0;

	/**
	\brief Retrieve attachment positions for all deformable vertices attached to the shape.

	You can retrieve the number of attached deformable vertices by calling #getNbVertices()

	\param[out] userBuffer The buffer to store the attachment positions (in shape local space).
	\param[in] bufferSize Size of provided user buffer.
	\return Number of attachment positions written to the buffer.

	@see PxDeformable getNbVertices()
	*/
	virtual PxU32			getPositions(PxVec3* userBuffer, PxU32 bufferSize) const = 0;

	/**
	\brief Set attachment positions for all deformable vertices attached to the shape.

	You can retrieve the number of attached deformable vertices by calling #getNbVertices()

	\param[in] positions List of attachment positions (in shape local space for shape attachments and global space for world attachments).
	\param[in] bufferSize Size of provided buffer.

	@see PxDeformable getNbVertices()
	*/
	virtual void			setPositions(const PxVec3* positions, PxU32 bufferSize) = 0;

	/**
	\brief Retrieve vertex indices for all deformable vertices attached to the shape.

	You can retrieve the number of attached deformable vertices by calling #getNbVertices()

	\param[out] userBuffer The buffer to store the indices of the attached vertices.
	\param[in] bufferSize Size of provided user buffer.
	\return Number of vertex indices written to the buffer.

	@see PxDeformable getNbVertices()
	*/
	virtual PxU32			getVertexIndices(PxU32* userBuffer, PxU32 bufferSize) const = 0;

	/**
	\brief Retrieve attachment flags for all deformable vertices attached to the shape.

	You can retrieve the number of attached deformable vertices by calling #getNbVertices()

	\param[out] userBuffer The buffer to store the flags of the attachments.
	\param[in] bufferSize Size of provided user buffer.
	\return Number of flags written to the buffer.

	@see PxDeformable getNbVertices()
	*/
	virtual PxU32			getFlags(PxU32* userBuffer, PxU32 bufferSize) const = 0;


protected:
	virtual ~PxAttachment() {}
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

#endif
