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

#ifndef PX_META_DATA_PVD_BINDING_H
#define PX_META_DATA_PVD_BINDING_H

#if PX_SUPPORT_VISUAL_DEBUGGER

#include "PxSimpleTypes.h"

namespace physx
{

class PxScene;
class PxMaterial;
class PxShape;
class PxRigidStatic;
class PxRigidDynamic;
class PxArticulation;
class PxArticulationLink;
class PxArticulationJoint;
class PxActor;
class PxParticleSystem;
class PxParticleFluid;
class PxDeformable;
class PxCloth;
class PxParticleReadData;
class PxParticleFluidReadData;
class PxPhysics;
class PxClothFabric;
class PxCloth;

namespace Sc
{
	class ContactIterator;
	class BodyIterator;
	class BodyCore;
	struct DeformableBulkData;
	struct ClothBulkData;
}

namespace Pvd
{

	struct PvdMetaDataBindingData;

	class PvdMetaDataBinding
	{
		PvdMetaDataBindingData*				mBindingData;

	public:
		PvdMetaDataBinding();
		~PvdMetaDataBinding();
		bool registerPropertyOverride( PVD::PvdDataStream* inStream, PxU32 inClassKey );

		
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxPhysics* inPhysics );
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxMaterial* inMaterial );
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxShape* inMaterial );
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxRigidStatic* inObj );
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxRigidDynamic* inObj );
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxArticulation* inObj );
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxArticulationLink* inObj );
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxArticulationJoint* inObj );
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxParticleSystem* inObj );
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxParticleFluid* inObj );
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxDeformable* inObj );
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxScene* inScene );
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxClothFabric* fabric );
		void sendAllProperties( PVD::PvdDataStream* inStream, const PxCloth* cloth );
		/** per-frame updates done roughly in this order*/
		void sendBeginFrame( PVD::PvdDataStream* inStream, const PxScene* inScene );
		void sendContacts( PVD::PvdDataStream* inStream, const PxScene* inScene, Sc::ContactIterator& inContacts );
		void sendContacts( PVD::PvdDataStream* inStream, const PxScene* inScene );
		void sendStats( PVD::PvdDataStream* inStream, const PxScene* inScene );
		void sendEndFrame( PVD::PvdDataStream* inStream, const PxScene* inScene );
		void updateDynamicActorsAndArticulations( PVD::PvdDataStream* inStream, const PxScene* inScene );
		void updateCloths( PVD::PvdDataStream* inStream, const PxScene* inScene );

		
		void sendArrays( PVD::PvdDataStream* inStream, const PxParticleSystem* inObj, PxParticleReadData& inData, PxU32 inFlags );
		void sendArrays( PVD::PvdDataStream* inStream, const PxParticleFluid* inObj, PxParticleFluidReadData& inData, PxU32 inFlags );
		void sendArrays( PVD::PvdDataStream* inStream, const PxDeformable* inObj, const Sc::DeformableBulkData& inData );
		void sendArrays( PVD::PvdDataStream* inStream, const PxCloth* inObj, const Sc::ClothBulkData& inData );

	private:
		template<typename TValueType, typename TDataType>
		void sendAllProperties( PVD::PvdDataStream* inStream, const TDataType* inDatatype, PxU32 inClassKey );
		template<typename TValueType, typename TDataType>
		void sendAllProperties( PVD::PvdDataStream* inStream, const TDataType* inDatatype, PxU32 inClassKey, PxU64 inInstanceId );
	};
};

}

#endif
#endif