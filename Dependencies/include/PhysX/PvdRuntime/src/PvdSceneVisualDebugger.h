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


#ifndef PVD_SCENEVISUALDEBUGGER_H
#define PVD_SCENEVISUALDEBUGGER_H

#if PX_SUPPORT_VISUAL_DEBUGGER

#include "PsUserAllocated.h"
#include "PsArray.h"
#include "PvdConnectionHelper.h"
#include "PvdRenderAdapter.h"
#include "CmPhysXCommon.h"
#include "PxMetaDataPvdBinding.h"

namespace PVD
{
	class PvdDataStream;
	class PvdCommLayerValue;
}

namespace physx
{

class PxSimulationStatistics;
class PxGeometry;

namespace profile
{	
	class PxProfileZone;
}

namespace Scb
{
	class Scene;
	class Actor;
	class Body;
	class RigidStatic;
	class RigidObject;
	class Shape;
	class Material;
	class ParticleSystem;
	class Deformable;
	class Attachment;
	class Constraint;
	class Articulation;
	class ArticulationJoint;
	class Cloth;
}

namespace Sc
{
	class RigidCore;
	class ConstraintCore;
}

namespace Pvd
{

struct SceneGroups
{
	enum Enum
	{
		RigidDynamics = 1, // 0 is reserved for SimulationStatistics
		RigidStatics,
		Joints,
		Articulations,
		ParticleSystems,
		Deformables,
		Materials,
		ProfileZones,
		Cloths,
		NUM_ELEMENTS,
	};
};

//////////////////////////////////////////////////////////////////////////
/*!
RemoteDebug supplies functionality for writing debug info to a stream
to be read by the remote debugger application.
*/
//////////////////////////////////////////////////////////////////////////
class SceneVisualDebugger: public Ps::UserAllocated
{
public:
	SceneVisualDebugger(Scb::Scene&);
	virtual ~SceneVisualDebugger();
	
	void setPvdConnection(PVD::PvdDataStream* c, PxU32 inConnectionType);
	PVD::PvdDataStream* getPvdConnection() const;

	// internal methods
	void sendClassDescriptions();
	bool isConnected();
	bool isConnectedAndSendingDebugInformation();
	void createGroups();
	void releaseGroups();

	void sendEntireScene();
	void frameStart();
	void frameEnd();
	void createPvdInstance();
	void updatePvdProperties();
	void releasePvdInstance();

	void createPvdInstance(Scb::Actor* scbActor); // temporary for deformables and particle systems
	void updatePvdProperties(Scb::Actor* scbActor);
	void releasePvdInstance(Scb::Actor* scbActor); // temporary for deformables and particle systems

	void createPvdInstance(Scb::Body* scbBody);
	void updatePvdProperties(Scb::Body* scbBody);

	
	void createPvdInstance(Scb::Cloth* scbCloth);
	void updatePvdProperties(Scb::Cloth* scbCloth);
	void releasePvdInstance(Scb::Cloth* scbCloth);

	void createPvdInstance(Scb::RigidStatic* scbRigidStatic);
	void updatePvdProperties(Scb::RigidStatic* scbRigidStatic);

	void releasePvdInstance(Scb::RigidObject* scbRigidObject);

	void createAndUpdateShapes(Scb::RigidObject* scbRigid);
	void createPvdInstance(Scb::Shape* scbShape);
	void updatePvdProperties(Scb::Shape* scbShape);
	void releasePvdInstance(Scb::Shape* scbShape);
	void createPvdInstance(Scb::Shape* scbShape, const PxGeometry* geometry);
	void updatePvdProperties(const PxGeometry* geometry);
	void releasePvdInstance(const PxGeometry* geometry);

	void updateMaterials(Scb::Shape* scbShape);


	void createPvdInstance(Scb::ParticleSystem* scbParticleSys);
	void updatePvdProperties(Scb::ParticleSystem* scbParticleSys);
	void releasePvdInstance(Scb::ParticleSystem* scbParticleSys);
	void sendArrays(Scb::ParticleSystem* scbParticleSys);

	void createPvdInstance(Scb::Attachment* scbAttachment);
	void updatePvdProperties(Scb::Attachment* scbAttachment);
	void releasePvdInstance(Scb::Attachment* scbAttachment);
	void sendArrays(Scb::Attachment* scbAttachment);

	void createPvdInstance(Scb::Constraint* constraint);
	void updatePvdProperties(Scb::Constraint* constraint);
	void releasePvdInstance(Scb::Constraint* constraint);

	void createPvdInstance(Scb::Articulation* articulation);
	void updatePvdProperties(Scb::Articulation* articulation);
	void releasePvdInstance(Scb::Articulation* articulation);

	void createPvdInstance(Scb::ArticulationJoint* articulationJoint);
	void updatePvdProperties(Scb::ArticulationJoint* articulationJoint);
	void releasePvdInstance(Scb::ArticulationJoint* articulationJoint);

	void createPvdInstance(const Scb::Material* scbMat);
	void updatePvdProperties( const Scb::Material* material );
	void releasePvdInstance(const Scb::Material* scbMat);

	void updateContacts();
	void setCreateContactReports(bool);
	
private:
	bool updateConstraint(const Sc::ConstraintCore& scConstraint, PxU32 updateType);

	PVD::PvdDataStream*					mPvdConnection;
	PvdConnectionHelper					mPvdConnectionHelper;
	Scb::Scene&							mScbScene;
	Ps::Array<PxU64>					mProfileZoneIdList;
	PvdRenderAdapter					mRenderAdapter;
	PxU32								mConnectionType;
	PvdMetaDataBinding					mMetaDataBinding;
};

} // namespace Pvd

}

#endif // PX_SUPPORT_VISUAL_DEBUGGER

#endif // VISUALDEBUGGER_H

