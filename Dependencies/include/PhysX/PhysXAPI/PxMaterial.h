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


#ifndef PX_PHYSICS_NXMATERIAL
#define PX_PHYSICS_NXMATERIAL
/** \addtogroup physics
@{
*/

#include "PxPhysX.h"
#include "common/PxSerialFramework.h"

#ifndef PX_DOXYGEN
namespace physx
{
#endif

class PxScene;

/**
\brief Flags which control the behavior of a material.

@see PxMaterial 
*/
struct PxMaterialFlag
{
	enum Enum
	{
		/**
		\brief Flag to enable anisotropic friction computation. 

		For a pair of actors, anisotropic friction is used only if at least one of the two actors' materials are anisotropic.
		The anisotropic friction parameters for the pair are taken from the material which is more anisotropic (i.e. the difference
		between its two dynamic friction coefficients is greater).

		The anisotropy direction of the chosen material is transformed to world space:

		dirOfAnisotropyWS = shape2world * dirOfAnisotropy

		Next, the directions of anisotropy in one or more contact planes (i.e. orthogonal to the contact normal) have to be determined. 
		The two directions are:

		uAxis = (dirOfAnisotropyWS ^ contactNormal).normalize()
		vAxis = contactNormal ^ uAxis

		This way [uAxis, contactNormal, vAxis] forms a basis.

		It may happen, however, that (dirOfAnisotropyWS | contactNormal).magnitude() == 1 
		and then (dirOfAnisotropyWS ^ contactNormal) has zero length. This happens when 
		the contactNormal is coincident to the direction of anisotropy. In this case we perform isotropic friction. 

		@see PxMaterial.setDirOfAnisotropy() PxMaterial.getDirOfAnisotropy() 
		*/
		eANISOTROPIC = 1 << 0,

		/**
		If this flag is set, friction computations are always skipped between shapes with this material and any other shape.
		*/
		eDISABLE_FRICTION = 1 << 1,

		/**
		The difference between "normal" and "strong" friction is that the strong friction feature
		remembers the "friction error" between simulation steps. The friction is a force trying to
		hold objects in place (or slow them down) and this is handled in the solver. But since the
		solver is only an approximation, the result of the friction calculation can include a small
		"error" - e.g. a box resting on a slope should not move at all if the static friction is in
		action, but could slowly glide down the slope because of a small friction error in each 
		simulation step. The strong friction counter-acts this by remembering the small error and
		taking it to account during the next simulation step.

		However, in some cases the strong friction could cause problems, and this is why it is
		possible to disable the strong friction feature by setting this flag. One example is
		raycast vehicles, that are sliding fast across the surface, but still need a precise
		steering behavior. It may be a good idea to reenable the strong friction when objects
		are coming to a rest, to prevent them from slowly creeping down inclines.

		Note: This flag only has an effect if the PxMaterialFlag::eDISABLE_FRICTION bit is 0.
		*/
		eDISABLE_STRONG_FRICTION = 1 << 2,
	};
};

/**
\brief collection of set bits defined in PxMaterialFlag.

@see PxMaterialFlag
*/
typedef PxFlags<PxMaterialFlag::Enum,PxU16> PxMaterialFlags;
PX_FLAGS_OPERATORS(PxMaterialFlag::Enum,PxU16);


/**
Flag that determines the combine mode. When two actors come in contact with each other, they each have
materials with various coefficients, but we only need a single set of coefficients for the pair.

Physics doesn't have any inherent combinations because the coefficients are determined empirically on a case by case
basis. However, simulating this with a pairwise lookup table is often impractical.

For this reason the following combine behaviors are available:

eAVERAGE
eMIN
eMULTIPLY
eMAX

The effective combine mode for the pair is maximum(material0.combineMode, material1.combineMode).

@see PxMaterial.setFrictionCombineMode() PxMaterial.getFrictionCombineMode() PxMaterial.setRestitutionCombineMode() PxMaterial.getFrictionCombineMode()
*/
struct PxCombineMode
{
	enum Enum
	{
		eAVERAGE	= 0,		//!< Average: (a + b)/2
		eMIN		= 1,		//!< Minimum: minimum(a,b)
		eMULTIPLY	= 2,		//!< Multiply: a*b
		eMAX		= 3,		//!< Maximum: maximum(a,b)
		eN_VALUES	= 4,		//!< This is not a valid combine mode, it is a sentinel to denote the number of possible values. We assert that the variable's value is smaller than this.
		ePAD_32		= 0xffffffff //!< This is not a valid combine mode, it is to assure that the size of the enum type is big enough.
	};
};



/**
\brief Class for describing a shape's surface properties.

<h3>Creation</h3>

Example material creation:
\include PxMaterial_Create_Example.cpp

You can create a material which has different friction coefficients depending on the direction that
a body in contact is trying to move in. This is called anisotropic friction.

<h3>Anisotropic Friction</h3>

Anisotropic friction is useful for modeling things like sledges, skis etc

When you create an anisotropic material you specify the default friction parameters and also friction parameters for the V axis.
The friction parameters for the V axis are applied to motion along the direction of anisotropy (dirOfAnisotropy).

Anisotropic Material Example:
\include PxMaterial_Aniso_Example.cpp

<h3>Visualizations:</h3>
\li #PxVisualizationParameter::eCONTACT_POINT
\li #PxVisualizationParameter::eCONTACT_NORMAL
\li #PxVisualizationParameter::eCONTACT_ERROR
\li #PxVisualizationParameter::eCONTACT_FORCE

@see PxPhysics.createMaterial()
*/
class PxMaterial
// PX_SERIALIZATION
	: public PxSerializable
//~PX_SERIALIZATION
{
	protected:
// PX_SERIALIZATION
								PxMaterial(PxRefResolver& v) :	PxSerializable(v)	{}
								PX_DECLARE_SERIAL_RTTI(PxMaterial, PxSerializable)
//~PX_SERIALIZATION

	PX_INLINE					PxMaterial() : userData(NULL)		{}
	virtual						~PxMaterial()						{}

	public:

	/**
	\brief Deletes the material.
	
	\note This will decrease the reference count by one.

	Releases the application's reference to the material.
	The material is destroyed when the application's reference is released and all shapes referencing the material are destroyed.

	@see PxScene::createMaterial()
	*/
	virtual		void			release() = 0;

	/**
	\brief Returns the reference count of the material.

	At creation, the reference count of the material is 1. Every shape referencing this material increments the
	count by 1.	When the reference count reaches 0, and only then, the material gets destroyed automatically.

	\return the current reference count.
	*/
	virtual PxU32				getReferenceCount() const = 0;

	/**
	\brief Sets the coefficient of dynamic friction.
	
	The coefficient of dynamic friction should be in [0, +inf]. If set to greater than staticFriction, the effective value of staticFriction will be increased to match.
	If the flag PxMaterialFlag::eANISOTROPIC is set, then this value is used for the primary direction of anisotropy (U axis)

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] coef Coefficient of dynamic friction. <b>Range:</b> [0, +inf]

	@see getDynamicFriction()
	*/
	virtual		void			setDynamicFriction(PxReal coef) = 0;

	/**
	\brief Retrieves the DynamicFriction value.

	\return The coefficient of dynamic friction.

	@see setDynamicFriction
	*/
	virtual		PxReal			getDynamicFriction() const = 0;

	/**
	\brief Sets the coefficient of static friction
	
	The coefficient of static friction should be in the range [0, +inf]
	if flags & PxMaterialFlag::eANISOTROPIC is set, then this value is used for the primary direction of anisotropy (U axis)

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] coef Coefficient of static friction. <b>Range:</b> [0,inf]

	@see getStaticFriction() 
	*/
	virtual		void			setStaticFriction(PxReal coef) = 0;

	/**
	\brief Retrieves the coefficient of static friction.
	\return The coefficient of static friction.

	@see setStaticFriction 
	*/
	virtual		PxReal			getStaticFriction() const = 0;

	/**
	\brief Sets the coefficient of restitution 
	
	A coefficient of 0 makes the object bounce as little as possible, higher values up to 1.0 result in more bounce.

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] rest Coefficient of restitution. <b>Range:</b> [0,1]

	@see getRestitution() 
	*/
	virtual		void			setRestitution(PxReal rest) = 0;

	/**
	\brief Retrieves the coefficient of restitution. 

	See #setRestitution.

	\return The coefficient of restitution.

	@see setRestitution() 
	*/
	virtual		PxReal			getRestitution() const = 0;

	/**
	\brief Sets the dynamic friction coefficient along the secondary (V) axis. 

	This is used when anisotropic friction is being applied. I.e. the PxMaterialFlag::eANISOTROPIC flag is set.

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] coef Coefficient of dynamic friction in the V axis. <b>Range:</b> [0, +inf]

	@see getDynamicFrictionV() setFlag()
	*/
	virtual		void			setDynamicFrictionV(PxReal coef) = 0;

	/**
	\brief Retrieves the dynamic friction coefficient for the V direction.
	
	See #setDynamicFrictionV.

	\return The coefficient if dynamic friction in the V direction.

	@see setDynamicFrictionV() 
	*/
	virtual		PxReal			getDynamicFrictionV() const = 0;

	/**
	\brief Sets the static friction coefficient along the secondary (V) axis. 

	This is used when anisotropic friction is being applied. I.e. the PxMaterialFlag::eANISOTROPIC flag is set.

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] coef Coefficient of static friction in the V axis. <b>Range:</b> [0,inf]

	@see getStaticFrictionV()  setFlag()
	*/
	virtual		void			setStaticFrictionV(PxReal coef) = 0;

	/**
	\brief Retrieves the static friction coefficient for the V direction.

	\return The coefficient of static friction in the V direction.

	@see setStaticFrictionV() 
	*/
	virtual		PxReal			getStaticFrictionV() const = 0;

	/**
	\brief Sets the shape space direction (unit vector) of anisotropy.

	This is used when anisotropic friction is being applied. I.e. the PxMaterialFlag::eANISOTROPIC flag is set.

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] vec Shape space direction of anisotropy. <b>Range:</b> direction vector

	@see getDirOfAnisotropy() setFlag()
	*/
	virtual		void			setDirOfAnisotropy(const PxVec3& vec) = 0;

	/**
	\brief Retrieves the direction of anisotropy value.

	\return The direction of anisotropy.

	@see setDirOfAnisotropy() setFlag()
	*/
	virtual		PxVec3			getDirOfAnisotropy() const = 0;

	/**
	\brief Raises or clears a particular material flag.
	
	See the list of flags #PxMaterialFlag

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] flag The PxMaterial flag to raise(set) or clear.

	@see getFlags() PxMaterialFlag
	*/
	virtual		void			setFlag(PxMaterialFlag::Enum flag, bool) = 0;


	/**
	\brief sets all the material flags.
	
	See the list of flags #PxMaterialFlag

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	*/
	virtual		void 			setFlags( PxMaterialFlags inFlags ) = 0;

	/**
	\brief Retrieves the flags. See #PxMaterialFlag.

	\return The material flags.

	@see PxMaterialFlag setFlags()
	*/
	virtual		PxMaterialFlags	getFlags() const = 0;

	/**
	\brief Sets the friction combine mode.
	
	See the enum ::PxCombineMode .

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] combMode Friction combine mode to set for this material. See #PxCombineMode.

	@see PxCombineMode getFrictionCombineMode setStaticFriction() setDynamicFriction()
	*/
	virtual		void			setFrictionCombineMode(PxCombineMode::Enum combMode) = 0;

	/**
	\brief Retrieves the friction combine mode.
	
	See #setFrictionCombineMode.

	\return The friction combine mode for this material.

	@see PxCombineMode setFrictionCombineMode() 
	*/
	virtual		PxCombineMode::Enum	getFrictionCombineMode() const = 0;

	/**
	\brief Sets the restitution combine mode.
	
	See the enum ::PxCombineMode .

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] combMode Restitution combine mode for this material. See #PxCombineMode.

	@see PxCombineMode getRestitutionCombineMode() setRestitution()
	*/
	virtual		void			setRestitutionCombineMode(PxCombineMode::Enum combMode) = 0;

	/**
	\brief Retrieves the restitution combine mode.
	
	See #setRestitutionCombineMode.

	\return The coefficient of restitution combine mode for this material.

	@see PxCombineMode setRestitutionCombineMode getRestitution()
	*/
	virtual		PxCombineMode::Enum	getRestitutionCombineMode() const = 0;

	//public variables:
				void*			userData;	//!< user can assign this to whatever, usually to create a 1:1 relationship with a user object.
};

#ifndef PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
