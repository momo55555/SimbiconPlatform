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


#if PX_SUPPORT_VISUAL_DEBUGGER

#include "PvdSceneVisualDebugger.h"
#include "PvdDataStream.h"
#include "PvdClassDefinitions.h"

#include "ScPhysics.h"
#include "NpScene.h"
#include "PsFoundation.h"

#include "ScBodyCore.h"
#include "ScBodySim.h"
#include "ScConstraintSim.h"

#include "NpRigidDynamic.h"
#include "NpRigidStatic.h"

#include "NpArticulation.h"
#include "NpArticulationLink.h"
#include "NpArticulationJoint.h"

#include "NpParticleFluid.h"
#include "NpDeformable.h"
#include "NpAttachment.h"
#include "CmEventProfiler.h"

#include "ScbCloth.h"
#include "NpCloth.h"

namespace physx
{
namespace Pvd
{

#define UPDATE_PVD_PROPERTIES_CHECK() { if ( !isConnectedAndSendingDebugInformation() ) return; }

PX_FORCE_INLINE static const PVD::Float3& toPvdType(const PxVec3& vec3) { return reinterpret_cast<const PVD::Float3&>(vec3); }
PX_FORCE_INLINE static const PVD::Quat& toPvdType(const PxQuat& quat) { return reinterpret_cast<const PVD::Quat&>(quat); }

PX_FORCE_INLINE static NpScene* getNpScene(Scb::Scene* scbScene) 
{ 
	return static_cast<NpScene*>(scbScene->getPxScene());
}

PX_FORCE_INLINE static const NpRigidDynamic* getNpRigidDynamic(const Scb::Body* scbBody) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpRigidDynamic*>(0)->getScbActorFast()));
	return reinterpret_cast<const NpRigidDynamic*>(reinterpret_cast<const char*>(scbBody)-offset);
}

PX_FORCE_INLINE static NpRigidDynamic* getNpRigidDynamic(Scb::Body* scbBody) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpRigidDynamic*>(0)->getScbActorFast()));
	return reinterpret_cast<NpRigidDynamic*>(reinterpret_cast<char*>(scbBody)-offset);
}

PX_FORCE_INLINE static const NpRigidStatic* getNpRigidStatic(const Scb::RigidStatic* scbRigidStatic) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpRigidStatic*>(0)->getScbActorFast()));
	return reinterpret_cast<const NpRigidStatic*>(reinterpret_cast<const char*>(scbRigidStatic)-offset);
}

PX_FORCE_INLINE static NpRigidStatic* getNpRigidStatic(Scb::RigidStatic* scbRigidStatic) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpRigidStatic*>(0)->getScbActorFast()));
	return reinterpret_cast<NpRigidStatic*>(reinterpret_cast<char*>(scbRigidStatic)-offset);
}

PX_FORCE_INLINE static NpShape* getNpShape(Scb::Shape* scbShape) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpShape*>(0)->getScbShape()));
	return reinterpret_cast<NpShape*>(reinterpret_cast<char*>(scbShape)-offset);
}

PX_FORCE_INLINE static const NpDeformable* getNpDeformable(const Scb::Deformable* scbDeformable) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpDeformable*>(0)->getScbActor()));
	return reinterpret_cast<const NpDeformable*>(reinterpret_cast<const char*>(scbDeformable)-offset);
}

PX_FORCE_INLINE static NpDeformable* getNpDeformable(Scb::Deformable* scbDeformable) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpDeformable*>(0)->getScbActor()));
	return reinterpret_cast<NpDeformable*>(reinterpret_cast<char*>(scbDeformable)-offset);
}

PX_FORCE_INLINE static NpAttachment* getNpAttachment(Scb::Attachment* scbAttachment) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpAttachment*>(0)->getScbAttachment()));
	return reinterpret_cast<NpAttachment*>(reinterpret_cast<char*>(scbAttachment)-offset);
}

PX_FORCE_INLINE static const NpParticleSystem* getNpParticleSystem(const Scb::ParticleSystem* scbParticleSystem) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpParticleSystem*>(0)->getScbActor()));
	return reinterpret_cast<const NpParticleSystem*>(reinterpret_cast<const char*>(scbParticleSystem)-offset);
}

PX_FORCE_INLINE static const NpParticleFluid* getNpParticleFluid(const Scb::ParticleSystem* scbParticleSystem) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpParticleFluid*>(0)->getScbActor()));
	return reinterpret_cast<const NpParticleFluid*>(reinterpret_cast<const char*>(scbParticleSystem)-offset);
}

PX_FORCE_INLINE static const NpArticulationLink* getNpArticulationLink(const Scb::Body* scbArticulationLink) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpArticulationLink*>(0)->getScbActorFast()));
	return reinterpret_cast<const NpArticulationLink*>(reinterpret_cast<const char*>(scbArticulationLink)-offset);
}

PX_FORCE_INLINE static NpArticulationLink* getNpArticulationLink(Scb::Body* scbArticulationLink) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpArticulationLink*>(0)->getScbActorFast()));
	return reinterpret_cast<NpArticulationLink*>(reinterpret_cast<char*>(scbArticulationLink)-offset);
}

PX_FORCE_INLINE static NpArticulation* getNpArticulation(Scb::Articulation* scbArticulation) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpArticulation*>(0)->getArticulation()));
	return reinterpret_cast<NpArticulation*>(reinterpret_cast<char*>(scbArticulation)-offset);
}

PX_FORCE_INLINE static NpArticulationJoint* getNpArticulationJoint(Scb::ArticulationJoint* scbArticulationJoint) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpArticulationJoint*>(0)->getScbArticulationJoint()));
	return reinterpret_cast<NpArticulationJoint*>(reinterpret_cast<char*>(scbArticulationJoint)-offset);
}

PX_FORCE_INLINE static NpConstraint* getNpConstraint(Sc::ConstraintCore* scConstraint) 
{ 
	size_t scOffset = reinterpret_cast<size_t>(&(reinterpret_cast<Scb::Constraint*>(0)->getScConstraint()));
	size_t scbOffset = reinterpret_cast<size_t>(&(reinterpret_cast<NpConstraint*>(0)->getScbConstraint()));
	return reinterpret_cast<NpConstraint*>(reinterpret_cast<char*>(scConstraint)-scOffset-scbOffset);
}

PX_FORCE_INLINE static Scb::ParticleSystem* getScbParticleSystem(Sc::ParticleSystemCore* scParticleSystem) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<Scb::ParticleSystem*>(0)->getScParticleSystem()));
	return reinterpret_cast<Scb::ParticleSystem*>(reinterpret_cast<char*>(scParticleSystem)-offset);
}

PX_FORCE_INLINE static Scb::Attachment* getScbAttachment(Sc::AttachmentCore* scAttachment) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<Scb::Attachment*>(0)->getScAttachment()));
	return reinterpret_cast<Scb::Attachment*>(reinterpret_cast<char*>(scAttachment)-offset);
}

PX_FORCE_INLINE static NpCloth* backptr(Scb::Cloth* cloth) 
{ 
	size_t offset = reinterpret_cast<size_t>(&(reinterpret_cast<NpCloth*>(0)->getScbCloth()));
	return reinterpret_cast<NpCloth*>(reinterpret_cast<char*>(cloth)-offset);
}

PX_FORCE_INLINE static const PxActor* getPxActor(const Scb::Actor* scbActor)
{
	PxActorType::Enum type = scbActor->getActorCoreSLOW().getActorCoreType();
	if(type == PxActorType::eRIGID_DYNAMIC)
	{
		return getNpRigidDynamic(static_cast<const Scb::Body*>(scbActor));
	}
	else if(type == PxActorType::eRIGID_STATIC)
	{
		return getNpRigidStatic(static_cast<const Scb::RigidStatic*>(scbActor));
	}
	else if (type == PxActorType::eDEFORMABLE)
	{
		return getNpDeformable(static_cast<const Scb::Deformable*>(scbActor));
	}
	else if (type == PxActorType::ePARTICLE_SYSTEM)
	{
		return getNpParticleSystem(static_cast<const Scb::ParticleSystem*>(scbActor));
	}
	else if (type == PxActorType::ePARTICLE_FLUID)
	{
		return getNpParticleFluid(static_cast<const Scb::ParticleSystem*>(scbActor));
	}
	else if (type == PxActorType::eARTICULATION_LINK)
	{
		return getNpArticulationLink(static_cast<const Scb::Body*>(scbActor));
	}
	
	return NULL;
}


SceneVisualDebugger::SceneVisualDebugger(Scb::Scene& s)
: mPvdConnection(NULL)
, mScbScene(s)
, mConnectionType( 0 )
{
}


SceneVisualDebugger::~SceneVisualDebugger()
{
	if(isConnected())
	{
		releasePvdInstance();
		releaseGroups();
		setCreateContactReports(false);
	}
	if(mPvdConnection)
		mPvdConnection->release();
}


PVD::PvdDataStream* SceneVisualDebugger::getPvdConnection() const
{
	return mPvdConnection;
}


void SceneVisualDebugger::setPvdConnection(PVD::PvdDataStream* c, PxU32 inConnectionType)
{
	if(mPvdConnection)
		mPvdConnection->release();
	mConnectionType = inConnectionType;

	mPvdConnection = c;

	if(mPvdConnection)
		c->addRef();
	else
		mProfileZoneIdList.clear();		
}

void SceneVisualDebugger::setCreateContactReports(bool s)
{
	mScbScene.getScScene().setCreateContactReports(s);
}


bool SceneVisualDebugger::isConnected()
{ 
	return mPvdConnection && mPvdConnection->isConnected(); 
}

bool SceneVisualDebugger::isConnectedAndSendingDebugInformation()
{
	return isConnected()
			&& ( mConnectionType & PVD::PvdConnectionType::Debug );
}

#define SETGROUPNAME(name) mPvdConnection->setPropertyValue(sceneId+1+SceneGroups::name, GroupProp::Name+1, PVD::createString(#name))
void SceneVisualDebugger::createGroups()
{
	PVD::PvdCommLayerError error;
	PxU64 sceneId = PX_PROFILE_POINTER_TO_U64(mScbScene.getPxScene());
	error = mPvdConnection->pushNamespace();
	error = mPvdConnection->setNamespace("");
	for(PxU32 i = 1; i < SceneGroups::NUM_ELEMENTS; i++)
	{
		error = mPvdConnection->createInstance(PvdClassKeys::Group+1, sceneId+i+1, PVD::EInstanceUIFlags::None);
		error = mPvdConnection->addChild(sceneId, sceneId+i+1);
	}
	error = SETGROUPNAME(RigidDynamics);
	error = SETGROUPNAME(RigidStatics);
	error = SETGROUPNAME(Joints);
	error = SETGROUPNAME(Articulations);
	error = SETGROUPNAME(ParticleSystems);
	error = SETGROUPNAME(Deformables);
	error = SETGROUPNAME(Materials);
	error = SETGROUPNAME(ProfileZones);
	error = SETGROUPNAME(Cloths);

	error = mPvdConnection->popNamespace();
	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}

void SceneVisualDebugger::releaseGroups()
{
	PVD::PvdCommLayerError error;
	PxU64 sceneId = PX_PROFILE_POINTER_TO_U64(mScbScene.getPxScene());
	for(PxU32 i = 1; i < SceneGroups::NUM_ELEMENTS; i++)
	{
		error = mPvdConnection->destroyInstance(sceneId+i+1);
		PX_ASSERT(error == PVD::PvdCommLayerError::None);
	}
}

void SceneVisualDebugger::sendEntireScene()
{
	if(!isConnected())
		return;

	PVD::PvdCommLayerError error;

	createPvdInstance();
	createGroups();

	// materials:
	{
		NpScene* npScene = getNpScene(&mScbScene);
		PxU32 numMaterials = mScbScene.getSceneMaterialTable().size();
		for(PxU32 i = 0; i < numMaterials; i++)
		{
			createPvdInstance(mScbScene.getSceneMaterialTable()[i]);
		}
	}

	if ( isConnectedAndSendingDebugInformation() )
	{
		// RBs
		// static:
		{
			NpScene* npScene = getNpScene(&mScbScene);
			Ps::Array<PxActor*> actorArray;
			PxU32 numActors = npScene->getNbActors(PxActorTypeSelectionFlag::eRIGID_STATIC);
			actorArray.resize(numActors);
			npScene->getActors(PxActorTypeSelectionFlag::eRIGID_STATIC, actorArray.begin(), actorArray.size());

			for(PxU32 i = 0; i < numActors; i++)
			{
				PxActor* pxActor = actorArray[i];
				Scb::RigidStatic* scbRigidStatic = &static_cast<NpRigidStatic*>(pxActor)->getScbRigidStaticFast();
				createPvdInstance(scbRigidStatic);
				// shapes
				createAndUpdateShapes(scbRigidStatic);
			}
		}

		// dynamic:
		{
			NpScene* npScene = getNpScene(&mScbScene);
			Ps::Array<PxActor*> actorArray;
			PxU32 numActors = npScene->getNbActors(PxActorTypeSelectionFlag::eRIGID_DYNAMIC);
			actorArray.resize(numActors);
			npScene->getActors(PxActorTypeSelectionFlag::eRIGID_DYNAMIC, actorArray.begin(), actorArray.size());

			for(PxU32 i = 0; i < numActors; i++)
			{
				PxActor* pxActor = actorArray[i];
				Scb::Body* scbRigidDynamic = &static_cast<NpRigidDynamic*>(pxActor)->getScbBodyFast();
				createPvdInstance(scbRigidDynamic);
				// shapes
				createAndUpdateShapes(scbRigidDynamic);
			}
		}

		// particle systems & fluids:
		{
			PxU32 nbParticleSystems = mScbScene.getScScene().getNbParticleSystems();
			Sc::ParticleSystemCore** particleSystems = mScbScene.getScScene().getParticleSystems();
			for(PxU32 i = 0; i < nbParticleSystems; i++)
			{
				Scb::ParticleSystem* scbParticleSystem = getScbParticleSystem(particleSystems[i]);
				createPvdInstance(scbParticleSystem);
			}
		}

		//cloth 
		{
			NpScene* npScene = getNpScene(&mScbScene);
			Ps::Array<PxActor*> actorArray;
			PxU32 numActors = npScene->getNbActors(PxActorTypeSelectionFlag::eCLOTH);
			actorArray.resize(numActors);
			npScene->getActors(PxActorTypeSelectionFlag::eCLOTH, actorArray.begin(), actorArray.size());
			for(PxU32 i = 0; i < numActors; i++)
			{
				Scb::Cloth* scbCloth = &static_cast<NpCloth*>(actorArray[i])->getScbCloth();
				createPvdInstance(scbCloth);
			}
		}

		// attachments
		{
			Sc::AttachmentCore** scAttachments = mScbScene.getScScene().getAttachments();
			PxU32 numAttachments = mScbScene.getScScene().getNbAttachments();

			for(PxU32 i = 0; i < numAttachments; i++)
			{
				createPvdInstance(getScbAttachment(scAttachments[i]));
			}
		}

		// articulations & links
		{
			NpScene* npScene = getNpScene(&mScbScene);
			Ps::Array<PxArticulation*> articulations;
			PxU32 numArticulations = npScene->getNbArticulations();
			articulations.resize(numArticulations);
			npScene->getArticulations(articulations.begin(), articulations.size());

			for(PxU32 i = 0; i < numArticulations; i++)
			{
				PxArticulation* pxArticulation = articulations[i];
				NpArticulation* npArticulation = static_cast<NpArticulation*>(pxArticulation);
				Scb::Articulation* scbArticulation = &npArticulation->getArticulation();

				PxU32 numLinks = npArticulation->getNbLinks();
				NpArticulationLink* const* links = npArticulation->getLinks();

				// create root
				{
					Scb::Body* scbLink = &links[0]->getScbBodyFast();
					createPvdInstance(scbLink);
					createAndUpdateShapes(scbLink);
				}

				// and children
				for(PxU32 l = 1; l < numLinks; l++)
				{
					Scb::Body* scbLink = &links[l]->getScbBodyFast();
					Scb::ArticulationJoint* scbJoint = &static_cast<NpArticulationJoint*>(links[l]->getInboundJoint())->getScbArticulationJoint();
					createPvdInstance(scbLink);
					createPvdInstance(scbJoint);
					createAndUpdateShapes(scbLink);
				}
				createPvdInstance(scbArticulation);
			}
		}

		// joints
		{
			Sc::ConstraintIterator iterator;
			mScbScene.getScScene().initConstraintsIterator(iterator);
			Sc::ConstraintCore* constraint;
			bool success = true;
			while(success && (constraint = iterator.getNext()))
			{
				success = updateConstraint(*constraint, PxPvdUpdateType::CREATE_INSTANCE);
				updateConstraint(*constraint, PxPvdUpdateType::UPDATE_ALL_PROPERTIES);
			}
		}
	}

	error = mPvdConnection->flush();
}


void SceneVisualDebugger::frameStart()
{
	if(!isConnected())
		return;

	PVD::PvdCommLayerError error;

	mMetaDataBinding.sendBeginFrame( mPvdConnection, mScbScene.getPxScene() );

	error = mPvdConnection->flush();

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::frameEnd()
{

	if(!isConnected())
		return;
	//Flush the outstanding memory events.  PVD in some situations tracks memory events
	//and can display graphs of memory usage at certain points.  They have to get flushed
	//at some point...
	NpPhysics::getInstance().getPvdConnectionManager()->flushMemoryEvents();
	//Also note that PVD is a consumer of the profiling system events.  This ensures
	//that PVD gets a view of the profiling events that pertained to the last frame.
	NpPhysics::getInstance().getProfileZoneManager().flushProfileEvents();
	//End the frame *before* we send the dynamic object current data.
	//This ensures that contacts end up synced with the rest of the system.
	//Note that contacts were sent much earler in NpScene::fetchResults.
	mMetaDataBinding.sendEndFrame(mPvdConnection, mScbScene.getPxScene() );
	//flush our data to the main connection
	//and flush the main connection.
	//This could be an expensive call.
	mPvdConnection->flush(); 

	const PxScene* theScene = mScbScene.getPxScene();

	if(isConnectedAndSendingDebugInformation())
	{
		{
			CM_PROFILE_ZONE_WITH_SUBSYSTEM( mScbScene,PVD,sceneUpdate );
			mMetaDataBinding.updateDynamicActorsAndArticulations( mPvdConnection, theScene );
		}
		{
			CM_PROFILE_ZONE_WITH_SUBSYSTEM( mScbScene,PVD,updateCloths);
			mMetaDataBinding.updateCloths( mPvdConnection, theScene );
		}
		// joints
		{
			CM_PROFILE_ZONE_WITH_SUBSYSTEM( mScbScene,PVD,updateJoints );
			Sc::ConstraintIterator iterator;
			mScbScene.getScScene().initConstraintsIterator(iterator);
			Sc::ConstraintCore* constraint;
			bool success = true;
			PxI64 constraintCount = 0;
			while(success && (constraint = iterator.getNext()))
			{
				PxPvdUpdateType::Enum updateType = getNpConstraint(constraint)->isDirty() ? PxPvdUpdateType::UPDATE_ALL_PROPERTIES : PxPvdUpdateType::UPDATE_SIM_PROPERTIES;
				success = updateConstraint(*constraint, updateType);
				++constraintCount;
			}
			CM_PROFILE_VALUE( mScbScene,PVD,updateJoints, constraintCount );
		}

		// particle systems & fluids:
		{
			CM_PROFILE_ZONE_WITH_SUBSYSTEM( mScbScene,PVD,updatePariclesAndFluids );
			PxU32 nbParticleSystems = mScbScene.getScScene().getNbParticleSystems();
			Sc::ParticleSystemCore** particleSystems = mScbScene.getScScene().getParticleSystems();
			for(PxU32 i = 0; i < nbParticleSystems; i++)
			{
				Scb::ParticleSystem* scbParticleSystem = getScbParticleSystem(particleSystems[i]);
				if(scbParticleSystem->getFlags() & PxParticleBaseFlag::eENABLED)
					sendArrays(scbParticleSystem);
			}
		}
		// frame end moved to update contacts to have them in the previous frame.
	}
	mMetaDataBinding.sendStats( mPvdConnection, theScene  );
}
	

void SceneVisualDebugger::createPvdInstance()
{
	PVD::PvdCommLayerError error;
	NpPhysics* npPhysics = &NpPhysics::getInstance();

	// create instance
	error = mPvdConnection->createInstance(PvdClassKeys::Scene+1, PX_PROFILE_POINTER_TO_U64(mScbScene.getPxScene()), PVD::EInstanceUIFlags::None);
	error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(npPhysics)+1, PX_PROFILE_POINTER_TO_U64(mScbScene.getPxScene()));

	updatePvdProperties();

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::updatePvdProperties()
{
	PVD::PvdCommLayerError error;
	PxScene* theScene = mScbScene.getPxScene();
	mMetaDataBinding.sendAllProperties( mPvdConnection, theScene  );
	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}

void SceneVisualDebugger::releasePvdInstance(Scb::RigidObject* scbRigidObject)
{
	PVD::PvdCommLayerError error;

	error = mPvdConnection->destroyInstance(PX_PROFILE_POINTER_TO_U64(getPxActor(scbRigidObject)));

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::createPvdInstance(Scb::Body* scbBody)
{
	// create instance
	PVD::PvdCommLayerError error;
	PxActor* pxActor = getNpRigidDynamic(scbBody);

	bool isArticulationLink = scbBody->getActorTypeSLOW() == PxActorType::eARTICULATION_LINK;
	PvdClassKeys::Enum classKey = isArticulationLink ? PvdClassKeys::ArticulationLink : PvdClassKeys::RigidDynamic;
	error = mPvdConnection->createInstance(classKey+1, PX_PROFILE_POINTER_TO_U64(pxActor), PVD::EInstanceUIFlags::None);
	updatePvdProperties(scbBody);

	if(isArticulationLink)
	{
		NpArticulationLink* parent = getNpArticulationLink(scbBody)->getParent();
		if(parent)
			error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(parent), PX_PROFILE_POINTER_TO_U64(pxActor));
	}
	else
	{
		PxU64 parent = PX_PROFILE_POINTER_TO_U64(mScbScene.getPxScene())+SceneGroups::RigidDynamics+1;
		error = mPvdConnection->addChild(parent, PX_PROFILE_POINTER_TO_U64(pxActor));
	}

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}

void SceneVisualDebugger::createPvdInstance(Scb::Cloth* scbCloth)
{
	NpCloth* realCloth( backptr( scbCloth ) );
	VisualDebugger& sdkPvd = NpPhysics::getInstance().getPhysics()->getVisualDebugger();
	sdkPvd.increaseReference( realCloth->getFabric() );
	PxActor* pxActor = &realCloth->getPxActorSLOW();
	PxU32 classKey = PvdClassKeys::Cloth;
	mPvdConnection->createInstance(classKey+1, PX_PROFILE_POINTER_TO_U64(pxActor), PVD::EInstanceUIFlags::None);
	PxU64 parent = PX_PROFILE_POINTER_TO_U64(mScbScene.getPxScene())+SceneGroups::Cloths+1;
	mPvdConnection->addChild(parent, PX_PROFILE_POINTER_TO_U64(pxActor));
	updatePvdProperties( scbCloth );
}

void SceneVisualDebugger::updatePvdProperties(Scb::Cloth* scbCloth)
{
	NpCloth* realCloth( backptr( scbCloth ) );
	PxActor* pxActor = &realCloth->getPxActorSLOW();
	PxCloth* pxCloth = static_cast<PxCloth*>( pxActor );
	mMetaDataBinding.sendAllProperties( mPvdConnection, pxCloth );
}

void SceneVisualDebugger::releasePvdInstance(Scb::Cloth* scbCloth)
{
	NpCloth* realCloth( backptr( scbCloth ) );
	VisualDebugger& sdkPvd = NpPhysics::getInstance().getPhysics()->getVisualDebugger();
	sdkPvd.decreaseReference( realCloth->getFabric() );
	PxActor* actor = &realCloth->getPxActorSLOW();
	mPvdConnection->destroyInstance( PX_PROFILE_POINTER_TO_U64(actor) );
}

void SceneVisualDebugger::createPvdInstance(Scb::Actor* scbActor)
{
	// create instance
	PVD::PvdCommLayerError error;
	const PxActor* pxActor = getPxActor(scbActor);
	error = mPvdConnection->createInstance(PvdClassKeys::Actor+1, PX_PROFILE_POINTER_TO_U64(pxActor), PVD::EInstanceUIFlags::None);
	error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(mScbScene.getPxScene()), PX_PROFILE_POINTER_TO_U64(pxActor));

	updatePvdProperties(scbActor);

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}

void SceneVisualDebugger::releasePvdInstance(Scb::Actor* scbActor)
{
	// create instance
	PVD::PvdCommLayerError error;
	const PxActor* pxActor = getPxActor(scbActor);
	error = mPvdConnection->destroyInstance(PX_PROFILE_POINTER_TO_U64(pxActor));
	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::updatePvdProperties(Scb::Actor* scbActor)
{
	UPDATE_PVD_PROPERTIES_CHECK();
	//MAKE THIS WORK CORRECTLY!!
	//mMetaDataBinding.sendAllProperties( mPvdConnection, getPxActor(scbActor) );
}


void SceneVisualDebugger::updatePvdProperties(Scb::Body* scbBody)
{	
	UPDATE_PVD_PROPERTIES_CHECK();

	bool isArticulationLink = scbBody->getActorTypeSLOW() == PxActorType::eARTICULATION_LINK;
	if(!isArticulationLink)
	{
		NpRigidDynamic* npRigidDynamic = getNpRigidDynamic(scbBody);
		mMetaDataBinding.sendAllProperties( mPvdConnection, static_cast< const PxRigidDynamic* >( npRigidDynamic ) );
	}
	else
	{
		PX_ASSERT(scbBody->getActorTypeSLOW() == PxActorType::eARTICULATION_LINK);
		
		NpArticulationLink* link( getNpArticulationLink( scbBody ) );
		mMetaDataBinding.sendAllProperties( mPvdConnection, link );
	}
}


void SceneVisualDebugger::createPvdInstance(Scb::RigidStatic* scbRigidStatic)
{
	// create instance
	PVD::PvdCommLayerError error;
	NpRigidStatic* npRigidStatic = getNpRigidStatic(scbRigidStatic);
	error = mPvdConnection->createInstance(PvdClassKeys::RigidStatic+1, PX_PROFILE_POINTER_TO_U64(npRigidStatic), PVD::EInstanceUIFlags::None);
	error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(mScbScene.getPxScene())+SceneGroups::RigidStatics+1, PX_PROFILE_POINTER_TO_U64(npRigidStatic));

	updatePvdProperties(scbRigidStatic);

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::updatePvdProperties(Scb::RigidStatic* scbRigidStatic)
{	
	UPDATE_PVD_PROPERTIES_CHECK();
	mMetaDataBinding.sendAllProperties( mPvdConnection, getNpRigidStatic( scbRigidStatic ) );
}


void SceneVisualDebugger::createAndUpdateShapes(Scb::RigidObject* scbRigid)
{
	// shapes
	Sc::ShapeIterator shapeIterator;
	mScbScene.getScScene().initActiveShapesIterator(scbRigid->getScRigidCoreSLOW(), shapeIterator);
	Sc::ShapeCore* s = NULL;
	while ((s = shapeIterator.getNext()))
	{
		Scb::Shape* scbShape = &static_cast<NpShape*>(s->getPxShape())->getScbShape();
		createPvdInstance(scbShape);
	}
}


void SceneVisualDebugger::createPvdInstance(Scb::Shape* scbShape)
{
	// create instance
	PVD::PvdCommLayerError error;
	PxShape* npShape = getNpShape(scbShape);
	PxActor& pxActor = getNpShape( scbShape)->getActorFast();
	
	error = mPvdConnection->createInstance(PvdClassKeys::Shape+1, PX_PROFILE_POINTER_TO_U64(npShape), PVD::EInstanceUIFlags::None);
	error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(&pxActor), PX_PROFILE_POINTER_TO_U64(npShape));

	// need to ensure, that the geometry pointer is always the one from sc shape and not the buffer
	// else we get not clear IDs for the geometry
	createPvdInstance(scbShape, &scbShape->getScShape().getGeometry());

	updateMaterials(scbShape);

	updatePvdProperties(scbShape);

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::updatePvdProperties(Scb::Shape* scbShape)
{
	UPDATE_PVD_PROPERTIES_CHECK();
	mMetaDataBinding.sendAllProperties( mPvdConnection, getNpShape(scbShape) );
}


static const PxU8 gGeometryToPvdKey[7] = 
{
	PvdClassKeys::SphereGeometry,
	PvdClassKeys::PlaneGeometry,
	PvdClassKeys::CapsuleGeometry,
	PvdClassKeys::BoxGeometry,
	PvdClassKeys::ConvexMeshGeometry,
	PvdClassKeys::TriangleMeshGeometry,
	PvdClassKeys::HeightFieldGeometry,
};


void SceneVisualDebugger::createPvdInstance(Scb::Shape* scbShape, const PxGeometry* geometry)
{	
	// create instance
	PVD::PvdCommLayerError error;
	NpShape* npShape = getNpShape(scbShape);
	error = mPvdConnection->createInstance(gGeometryToPvdKey[PxU32(geometry->getType())]+1, PX_PROFILE_POINTER_TO_U64(geometry), PVD::EInstanceUIFlags::None);
	error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(npShape), PX_PROFILE_POINTER_TO_U64(geometry));

	updatePvdProperties(geometry);

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::updatePvdProperties(const PxGeometry* geometry)
{
	PVD::PvdCommLayerError error;
	
	UPDATE_PVD_PROPERTIES_CHECK();

	VisualDebugger& sdkPvd = NpPhysics::getInstance().getPhysics()->getVisualDebugger();
	
	PxGeometryType::Enum geomType = geometry->getType();
	if(geomType == PxGeometryType::eBOX)
	{
		const PxBoxGeometry* boxGeom = static_cast<const PxBoxGeometry*>(geometry);
		mPvdConnectionHelper.addPropertyGroupProperty(BoxGeometryProp::HalfExtents, toPvdType(boxGeom->halfExtents));
	}
	else if(geomType == PxGeometryType::eSPHERE)
	{
		const PxSphereGeometry* sphereGeom = static_cast<const PxSphereGeometry*>(geometry);

		mPvdConnectionHelper.addPropertyGroupProperty(SphereGeometryProp::Radius,					sphereGeom->radius);
	}
	else if(geomType == PxGeometryType::eCAPSULE)
	{
		const PxCapsuleGeometry* capsuleGeom = static_cast<const PxCapsuleGeometry*>(geometry);

		mPvdConnectionHelper.addPropertyGroupProperty(CapsuleGeometryProp::Radius,					capsuleGeom->radius);
		mPvdConnectionHelper.addPropertyGroupProperty(CapsuleGeometryProp::HalfHeight,				capsuleGeom->halfHeight);
	}
	else if(geomType == PxGeometryType::eTRIANGLEMESH)
	{
		const PxTriangleMeshGeometry* triangleGeom = static_cast<const PxTriangleMeshGeometry*>(geometry);

		sdkPvd.increaseReference(triangleGeom->triangleMesh);
		mPvdConnectionHelper.addPropertyGroupProperty(TriangleMeshGeometryProp::Scale_Scale,		toPvdType(triangleGeom->scale.scale));
		mPvdConnectionHelper.addPropertyGroupProperty(TriangleMeshGeometryProp::Scale_Rot,			toPvdType(triangleGeom->scale.rotation));
		mPvdConnectionHelper.addPropertyGroupProperty(TriangleMeshGeometryProp::Flags,				PVD::createBitflag(triangleGeom->meshFlags));
		mPvdConnectionHelper.addPropertyGroupProperty(TriangleMeshGeometryProp::TriangleMesh,		PVD::createInstanceId(PX_PROFILE_POINTER_TO_U64(triangleGeom->triangleMesh)));
	}
	else if(geomType == PxGeometryType::eCONVEXMESH)
	{
		const PxConvexMeshGeometry* convexGeom = static_cast<const PxConvexMeshGeometry*>(geometry);
		
		sdkPvd.increaseReference(convexGeom->convexMesh);
		mPvdConnectionHelper.addPropertyGroupProperty(ConvexMeshGeometryProp::Scale_Scale,			toPvdType(convexGeom->scale.scale));
		mPvdConnectionHelper.addPropertyGroupProperty(ConvexMeshGeometryProp::Scale_Rot,			toPvdType(convexGeom->scale.rotation));
		mPvdConnectionHelper.addPropertyGroupProperty(ConvexMeshGeometryProp::ConvexMesh,			PVD::createInstanceId(PX_PROFILE_POINTER_TO_U64(convexGeom->convexMesh)));
	}
	else if(geomType == PxGeometryType::eHEIGHTFIELD)
	{
		const PxHeightFieldGeometry* hfGeom = static_cast<const PxHeightFieldGeometry*>(geometry);
		
		sdkPvd.increaseReference(hfGeom->heightField);
		mPvdConnectionHelper.addPropertyGroupProperty(HeightFieldGeometryProp::HeightScale,			hfGeom->heightScale);
		mPvdConnectionHelper.addPropertyGroupProperty(HeightFieldGeometryProp::RowScale,			hfGeom->rowScale);
		mPvdConnectionHelper.addPropertyGroupProperty(HeightFieldGeometryProp::ColumnScale,			hfGeom->columnScale);
		mPvdConnectionHelper.addPropertyGroupProperty(HeightFieldGeometryProp::Flags,				PVD::createBitflag(hfGeom->heightFieldFlags));
		mPvdConnectionHelper.addPropertyGroupProperty(HeightFieldGeometryProp::HeightField,			PVD::createInstanceId(PX_PROFILE_POINTER_TO_U64(hfGeom->heightField)));
	}
	else
	{
		return;
	}

	error = mPvdConnectionHelper.sendSinglePropertyGroup(mPvdConnection, PX_PROFILE_POINTER_TO_U64(geometry), gGeometryToPvdKey[PxU32(geomType)]);
	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}

void SceneVisualDebugger::releasePvdInstance(const PxGeometry* geometry)
{	
	VisualDebugger& sdkPvd = NpPhysics::getInstance().getPhysics()->getVisualDebugger();
	PxGeometryType::Enum geomType = geometry->getType();
	if(geomType == PxGeometryType::eTRIANGLEMESH)
	{
		const PxTriangleMeshGeometry* triangleGeom = static_cast<const PxTriangleMeshGeometry*>(geometry);
		sdkPvd.decreaseReference(triangleGeom->triangleMesh);
	}
	else if(geomType == PxGeometryType::eCONVEXMESH)
	{
		const PxConvexMeshGeometry* convexGeom = static_cast<const PxConvexMeshGeometry*>(geometry);
		sdkPvd.decreaseReference(convexGeom->convexMesh);
	}
	else if(geomType == PxGeometryType::eHEIGHTFIELD)
	{
		const PxHeightFieldGeometry* hfGeom = static_cast<const PxHeightFieldGeometry*>(geometry);
		sdkPvd.decreaseReference(hfGeom->heightField);
	}

	PVD::PvdCommLayerError error;
	error = mPvdConnection->destroyInstance(PX_PROFILE_POINTER_TO_U64(geometry));
	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}

void SceneVisualDebugger::releasePvdInstance(Scb::Shape* scbShape)
{
	PVD::PvdCommLayerError error;

	NpShape* npShape = getNpShape(scbShape);
	error = mPvdConnection->destroyInstance(PX_PROFILE_POINTER_TO_U64(npShape));
	releasePvdInstance(&scbShape->getGeometry());
	
	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::releasePvdInstance()
{
	PVD::PvdCommLayerError error;

	error = mPvdConnection->destroyInstance(PX_PROFILE_POINTER_TO_U64(mScbScene.getPxScene()));

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void  SceneVisualDebugger::createPvdInstance(const Scb::Material* scbMat)
{
	UPDATE_PVD_PROPERTIES_CHECK();
	
	VisualDebugger& sdkPvd = NpPhysics::getInstance().getPhysics()->getVisualDebugger();
	const PxMaterial* theMaterial( scbMat->getNxMaterial() );
	sdkPvd.increaseReference( theMaterial );
	mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(mScbScene.getPxScene())+SceneGroups::Materials+1, PX_PROFILE_POINTER_TO_U64(scbMat->getNxMaterial()));
}

void SceneVisualDebugger::updatePvdProperties( const Scb::Material* material )
{
	UPDATE_PVD_PROPERTIES_CHECK();
	VisualDebugger& sdkPvd = NpPhysics::getInstance().getPhysics()->getVisualDebugger();
	sdkPvd.updatePvdProperties( material->getNxMaterial() );
}


void SceneVisualDebugger::releasePvdInstance(const Scb::Material* scbMat)
{
	UPDATE_PVD_PROPERTIES_CHECK();
	VisualDebugger& sdkPvd = NpPhysics::getInstance().getPhysics()->getVisualDebugger();
	mPvdConnection->removeChild(PX_PROFILE_POINTER_TO_U64(mScbScene.getPxScene())+SceneGroups::Materials+1, PX_PROFILE_POINTER_TO_U64(scbMat->getNxMaterial()));
	sdkPvd.decreaseReference( scbMat->getNxMaterial() );
}


void SceneVisualDebugger::updateMaterials(Scb::Shape* scbShape)
{

	UPDATE_PVD_PROPERTIES_CHECK();
	PVD::PvdCommLayerError error;

	// remove all materials first and then add all again.
	NpShape* npShape = getNpShape(scbShape);
	error = mPvdConnection->removeAllChildren(PX_PROFILE_POINTER_TO_U64(npShape), PvdClassKeys::Material+1);

	PxU32 numMaterials = npShape->getNbMaterials();;
	PX_ALLOCA(scMaterials, PxMaterial*, numMaterials);

	if(!scMaterials)
		return;

	npShape->getMaterials( scMaterials, numMaterials );

	for(PxU32 m = 0; m < numMaterials; m++)
	{
		error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(npShape), PX_PROFILE_POINTER_TO_U64(scMaterials[m]));
	}

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


class PxVec3ToPvd
{
public:
	PX_FORCE_INLINE static PVD::PvdCommLayerDatatype			getPvdDataType()			{ return PVD::PvdCommLayerDatatype::Float3; }
	PX_FORCE_INLINE static PvdClassKeys::Enum					getPvdClassKey()			{ return PvdClassKeys::VectorArray; }
	PX_FORCE_INLINE static PVD::Float3							convert(const PxVec3& v)	{ return toPvdType(v); }
};

class FloatToPvd
{
public:
	PX_FORCE_INLINE static PVD::PvdCommLayerDatatype			getPvdDataType()			{ return PVD::PvdCommLayerDatatype::Float; }
	PX_FORCE_INLINE static PvdClassKeys::Enum					getPvdClassKey()			{ return PvdClassKeys::FloatArray; }
	PX_FORCE_INLINE static PVD::PxF32							convert(const PxReal& r)	{ return r; }
};

class BitFlagToPvd
{
public:
	PX_FORCE_INLINE static PVD::PvdCommLayerDatatype			getPvdDataType()			{ return PVD::PvdCommLayerDatatype::Bitflag; }
	PX_FORCE_INLINE static PvdClassKeys::Enum					getPvdClassKey()			{ return PvdClassKeys::ParticleFlagsArray; }
	PX_FORCE_INLINE static PVD::Bitflag							convert(const PxU32& b)		{ return PVD::createBitflag(b); }
};


void SceneVisualDebugger::createPvdInstance(Scb::ParticleSystem* scbParticleSys)
{
	UPDATE_PVD_PROPERTIES_CHECK();
	// create instance
	PVD::PvdCommLayerError error;

	PxActorType::Enum actorType = scbParticleSys->getActorCoreSLOW().getActorCoreType();
	PX_ASSERT((actorType == PxActorType::ePARTICLE_FLUID) || (actorType == PxActorType::ePARTICLE_SYSTEM));
	const PxActor* pxActor = getPxActor(scbParticleSys);

	PvdClassKeys::Enum classKey = (actorType == PxActorType::ePARTICLE_SYSTEM) ? PvdClassKeys::ParticleSystem : PvdClassKeys::ParticleFluid;
	error = mPvdConnection->createInstance(classKey+1, PX_PROFILE_POINTER_TO_U64(pxActor), PVD::EInstanceUIFlags::None);
	error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(mScbScene.getPxScene())+SceneGroups::ParticleSystems+1, PX_PROFILE_POINTER_TO_U64(pxActor));

	sendArrays(scbParticleSys);

	updatePvdProperties(scbParticleSys);

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::sendArrays(Scb::ParticleSystem* scbParticleSys)
{
	UPDATE_PVD_PROPERTIES_CHECK();

	NpParticleFluidReadData readData;
	PxU32 rdFlags = scbParticleSys->getParticleReadDataFlags();
	scbParticleSys->getScParticleSystem().getParticleReadData(readData);
	const PxActor* pxActor = getPxActor(scbParticleSys);
	PxActorType::Enum type = pxActor->getType();
	if ( type == PxActorType::ePARTICLE_SYSTEM )
		mMetaDataBinding.sendArrays( mPvdConnection, static_cast<const PxParticleSystem*>( pxActor ), readData, rdFlags );
	else if ( type == PxActorType::ePARTICLE_FLUID )
		mMetaDataBinding.sendArrays( mPvdConnection, static_cast<const PxParticleFluid*>( pxActor ), readData, rdFlags );

}

void SceneVisualDebugger::updatePvdProperties(Scb::ParticleSystem* scbParticleSys)
{
	UPDATE_PVD_PROPERTIES_CHECK();
	PxActorType::Enum actorType = scbParticleSys->getActorCoreSLOW().getActorCoreType();
	PX_ASSERT((actorType == PxActorType::ePARTICLE_FLUID) || (actorType == PxActorType::ePARTICLE_SYSTEM));
	bool isFluid = (actorType == PxActorType::ePARTICLE_FLUID);
	const PxActor* pxActor = getPxActor(scbParticleSys);
	if ( isFluid )
	{
		const PxParticleFluid* theFluid = static_cast<PxParticleFluid*>(scbParticleSys->getPxParticleSystem());
		mMetaDataBinding.sendAllProperties( mPvdConnection, theFluid );
	}
	else
	{
		const PxParticleSystem* theSystem = static_cast<PxParticleSystem*>(scbParticleSys->getPxParticleSystem());
		mMetaDataBinding.sendAllProperties( mPvdConnection, theSystem );
	}
}


void SceneVisualDebugger::releasePvdInstance(Scb::ParticleSystem* scbParticleSys)
{
	PVD::PvdCommLayerError error;
	const PxActor* pxActor = getPxActor(scbParticleSys);
	error = mPvdConnection->destroyInstance(PX_PROFILE_POINTER_TO_U64(pxActor));
	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::createPvdInstance(Scb::Attachment* scbAttachment)
{
	// create instance
	PVD::PvdCommLayerError error;
	NpAttachment* npAttachment = getNpAttachment(scbAttachment);
	NpDeformable* npDefo = getNpDeformable(scbAttachment->getDeformable());

	error = mPvdConnection->createInstance(PvdClassKeys::Attachment+1, PX_PROFILE_POINTER_TO_U64(npAttachment), PVD::EInstanceUIFlags::None);
	error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(npDefo), PX_PROFILE_POINTER_TO_U64(npAttachment));
	if(scbAttachment->getShape())
		error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(npAttachment), PX_PROFILE_POINTER_TO_U64(getNpShape(scbAttachment->getShape())));

	sendArrays(scbAttachment);

	updatePvdProperties(scbAttachment);

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::updatePvdProperties(Scb::Attachment* scbAttachment)
{
	UPDATE_PVD_PROPERTIES_CHECK();
	PVD::PvdCommLayerError error;
	NpAttachment* npAttachment = getNpAttachment(scbAttachment);

	error = mPvdConnectionHelper.sendSinglePropertyGroup(mPvdConnection, PX_PROFILE_POINTER_TO_U64(npAttachment), PvdClassKeys::Attachment);
	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::sendArrays(Scb::Attachment* scbAttachment)
{
	const PxAttachment* pxAttachment = getNpAttachment(scbAttachment);
	PxU64 theInstance(PX_PROFILE_POINTER_TO_U64(pxAttachment));

	PxU32 nbAttachedVertices = scbAttachment->getNbVertices();

	PVD::PvdCommLayerDatatype propTypes[1];

	// positionArray Array:
	PvdConnectionHelper::beginSingleElementArrayProperty(mPvdConnection, theInstance, AttachmentProp::PositionArray, 0, PxVec3ToPvd::getPvdDataType() );
	mPvdConnection->sendArrayObjects(reinterpret_cast<const PxU8*>(scbAttachment->getPositions()), sizeof(PxVec3), nbAttachedVertices);
	mPvdConnection->endArrayPropertyBlock();
	
	// vertexIndexArray Array:
	PvdConnectionHelper::beginSingleElementArrayProperty(mPvdConnection, theInstance, AttachmentProp::VertexIndexArray, 0, PVD::PvdCommLayerDatatype::U32 );
	mPvdConnection->sendArrayObjects(reinterpret_cast<const PxU8*>(scbAttachment->getVertexIndices()), sizeof(PxU32), nbAttachedVertices);
	mPvdConnection->endArrayPropertyBlock();

	// flagsArray Array:
	PvdConnectionHelper::beginSingleElementArrayProperty(mPvdConnection, theInstance, AttachmentProp::FlagsArray, 0, BitFlagToPvd::getPvdDataType() );
	mPvdConnection->sendArrayObjects(reinterpret_cast<const PxU8*>(scbAttachment->getFlags()), sizeof(PxU32), nbAttachedVertices);
	mPvdConnection->endArrayPropertyBlock();
}


void SceneVisualDebugger::releasePvdInstance(Scb::Attachment* scbAttachment)
{
	PVD::PvdCommLayerError error;
	NpAttachment* npAttachment = getNpAttachment(scbAttachment);
	error = mPvdConnection->destroyInstance(PX_PROFILE_POINTER_TO_U64(npAttachment));
	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


bool SceneVisualDebugger::updateConstraint(const Sc::ConstraintCore& scConstraint, PxU32 updateType)
{
	PxConstraintConnector* conn;
	bool success = false;
	if( (conn = scConstraint.getPxConnector()) != NULL
		&& isConnectedAndSendingDebugInformation() )
	{
		success = conn->updatePvdProperties(*mPvdConnection, scConstraint.getPxConstraint(), PxPvdUpdateType::Enum(updateType));

		// visualize:
		VisualDebugger* pvd = static_cast<VisualDebugger*>(NpPhysics::getInstance().getVisualDebugger());
		PxReal frameScale = pvd->getJointFrameScale();
		PxReal limitScale = pvd->getJointLimitScale();
		const bool visualizeJoints = ((frameScale != 0) || (limitScale != 0)) && updateType == PxPvdUpdateType::UPDATE_SIM_PROPERTIES;
		Sc::ConstraintSim* sim = scConstraint.getSim();
		if(success && visualizeJoints && sim && sim->getConstantsLL())
		{
			Sc::BodySim* b0 = scConstraint.getSim()->getBody(0);
			Sc::BodySim* b1 = scConstraint.getSim()->getBody(1);
			PxTransform t0 = b0 ? b0->getBody2World() : PxTransform::createIdentity();
			PxTransform t1 = b1 ? b1->getBody2World() : PxTransform::createIdentity();
			(*scConstraint.getVisualize())(mRenderAdapter.getRenderBuffer(), sim->getConstantsLL(), t0, t1, frameScale, limitScale, 0xffffFFFF);
			mRenderAdapter.visualize(*mPvdConnection, PX_PROFILE_POINTER_TO_U64(conn));
		}
	}
	return success;
}


void SceneVisualDebugger::createPvdInstance(Scb::Constraint* constraint)
{
	updateConstraint(constraint->getScConstraint(), PxPvdUpdateType::CREATE_INSTANCE);
}


void SceneVisualDebugger::updatePvdProperties(Scb::Constraint* constraint)
{
	UPDATE_PVD_PROPERTIES_CHECK();
	updateConstraint(constraint->getScConstraint(), PxPvdUpdateType::UPDATE_ALL_PROPERTIES);
}


void SceneVisualDebugger::releasePvdInstance(Scb::Constraint* constraint)
{
	Sc::ConstraintCore& scConstraint = constraint->getScConstraint();
	PxConstraintConnector* conn = scConstraint.getPxConnector();
	if (conn)
	{
		conn->updatePvdProperties(*mPvdConnection, scConstraint.getPxConstraint(), PxPvdUpdateType::RELEASE_INSTANCE);
	}
}


void SceneVisualDebugger::createPvdInstance(Scb::Articulation* articulation)
{
	// create instance
	PVD::PvdCommLayerError error;
	NpArticulation* npArticulation = getNpArticulation(articulation);
	
	error = mPvdConnection->createInstance(PvdClassKeys::Articulation+1, PX_PROFILE_POINTER_TO_U64(npArticulation), PVD::EInstanceUIFlags::None);
	error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(mScbScene.getPxScene())+SceneGroups::Articulations+1, PX_PROFILE_POINTER_TO_U64(npArticulation));
	// add root as child
	if(npArticulation->getNbLinks())
		error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(npArticulation), PX_PROFILE_POINTER_TO_U64(npArticulation->getLinks()[0]));
	
	updatePvdProperties(articulation);

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::updatePvdProperties(Scb::Articulation* articulation)
{
	UPDATE_PVD_PROPERTIES_CHECK();
	NpArticulation* npArticulation = getNpArticulation(articulation);
	mMetaDataBinding.sendAllProperties( mPvdConnection, npArticulation );
}


void SceneVisualDebugger::releasePvdInstance(Scb::Articulation* articulation)
{
	PVD::PvdCommLayerError error;
	NpArticulation* npArticulation = getNpArticulation(articulation);
	error = mPvdConnection->destroyInstance(PX_PROFILE_POINTER_TO_U64(npArticulation));
	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::createPvdInstance(Scb::ArticulationJoint* articulationJoint)
{
	// create instance
	PVD::PvdCommLayerError error;
	NpArticulationJoint* npArticulationJoint = getNpArticulationJoint(articulationJoint);
	
	error = mPvdConnection->createInstance(PvdClassKeys::ArticulationJoint+1, PX_PROFILE_POINTER_TO_U64(npArticulationJoint), PVD::EInstanceUIFlags::None);
	error = mPvdConnection->addChild(PX_PROFILE_POINTER_TO_U64(&npArticulationJoint->getOwner()), PX_PROFILE_POINTER_TO_U64(npArticulationJoint));
	
	updatePvdProperties(articulationJoint);

	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}


void SceneVisualDebugger::updatePvdProperties(Scb::ArticulationJoint* articulationJoint)
{
	UPDATE_PVD_PROPERTIES_CHECK();
	PVD::PvdCommLayerError error;
	NpArticulationJoint* npArticulationJoint = getNpArticulationJoint(articulationJoint);
	mMetaDataBinding.sendAllProperties( mPvdConnection, static_cast< const PxArticulationJoint* >( npArticulationJoint ) );
}


void SceneVisualDebugger::releasePvdInstance(Scb::ArticulationJoint* articulationJoint)
{
	PVD::PvdCommLayerError error;
	NpArticulationJoint* npArticulationJoint = getNpArticulationJoint(articulationJoint);
	error = mPvdConnection->destroyInstance(PX_PROFILE_POINTER_TO_U64(npArticulationJoint));
	PX_ASSERT(error == PVD::PvdCommLayerError::None);
}

void SceneVisualDebugger::updateContacts()
{
	if(!isConnectedAndSendingDebugInformation())
		return;

	// if contacts are disabled, send empty array and return
	VisualDebugger& sdkPvd = NpPhysics::getInstance().getPhysics()->getVisualDebugger();
	const PxScene* theScene( mScbScene.getPxScene() );
	if(!sdkPvd.getTransmitContactsFlag())
	{
		mMetaDataBinding.sendContacts( mPvdConnection, theScene );
		return;
	}

	CM_PROFILE_ZONE_WITH_SUBSYSTEM( mScbScene,PVD,updateContacts );
	Sc::ContactIterator contactIter;
	mScbScene.getScScene().initContactsIterator(contactIter);
	mMetaDataBinding.sendContacts( mPvdConnection, theScene, contactIter );
}

} // namespace Pvd

}

#endif  // PX_SUPPORT_VISUAL_DEBUGGER
