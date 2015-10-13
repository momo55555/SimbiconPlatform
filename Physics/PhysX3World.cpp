#include "stdafx.h"

#include "PhysX3World.h"
#include <PUtils.h>
#include "Joint.h"
#include "StiffJoint.h"
#include "HingeJoint.h"
#include "UniversalJoint.h"
#include "BallInSocketJoint.h"
#include "PhysicsGlobals.h"
#include <cmath>

using namespace std;

static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;

/**
	Default constructor
*/
PhysX3World::PhysX3World() : World(){
	setupWorld();
}

/**
	destructor
*/
PhysX3World::~PhysX3World(void){
}

void PhysX3World::destroyWorld() {
	//delete[] cps;
	delete pcQuery;
    pcQuery = NULL;
//     for(size_t i = 0; i < physxToRbs.size(); ++i)
//     {
//          if(physxToRbs[i].id != NULL) physxToRbs[i].id->release();
//          physxToRbs[i].id = NULL;
//          delete physxToRbs[i].rb;
//          physxToRbs[i].rb = NULL;
//     }
	physxToRbs.clear();
	//destroy the PhysX physical world, simulation space and joint group
	if(gPhysicsSDK != NULL)
	{
		if(gScene != NULL) gScene->release();
		gScene = NULL;
        if(gCooking != NULL) gCooking->release();
        gCooking = NULL;
        if(defaultMaterial != NULL)defaultMaterial->release();
        defaultMaterial = NULL;
        if(gCudaContextManager != NULL)gCudaContextManager->release();
        gCudaContextManager = NULL;
        spaceID = NULL;
        gPhysicsSDK->release();
        gPhysicsSDK = NULL;
	}
	World::destroyWorld();
}

//#define min(x, y) (((x)>(y))?(y):(x))

/**
	this method is used to process the collision between the two objects passed in as parameters. More generally,
	it is used to determine if the collision should take place, and if so, it calls the method that generates the
	contact points.
*/
void PhysX3World::collisionsPostProcessing(PxContactPair & pair, PxU32 events){
	// Iterate through contact points
	PxContactStreamIterator i(pair.stream);
	//user can call getNumPairs() here
	while(i.goNextPair())
	{
		PxShape *o1,*o2;
		o1 = i.getShape(0);
		o2 = i.getShape(1);

		PxActor *b1, *b2;
		RigidBody *rb1, *rb2;
		b1 = &(o1->getActor());
		b2 = &(o2->getActor());
		rb1 = (RigidBody*) o1->userData;
		rb2 = (RigidBody*) o2->userData;

		//we'll use the minimum of the two coefficients of friction of the two bodies.
		double mu1 = rb1->getFrictionCoefficient();
		double mu2 = rb2->getFrictionCoefficient();
		double mu_to_use = min(mu1, mu2);
		double eps1 = rb1->getRestitutionCoefficient();
		double eps2 = rb2->getRestitutionCoefficient();
		double eps_to_use = min(eps1, eps2);

		double groundSoftness = 0, groundPenalty = 0;
		if (rb1){
			groundSoftness = rb1->props.groundSoftness;
			groundPenalty = rb1->props.groundPenalty;
		}else{
			groundSoftness = rb2->props.groundSoftness;
			groundPenalty = rb2->props.groundPenalty;
		}
		
		//int j = 0;
		//user can also call getShape() and getNumPatches() here
		while(i.goNextPatch())
		{
			//user can also call getPatchNormal() and getNumPoints() here
			const PxVec3& contactNormal = i.getPatchNormal();
			while(i.goNextPoint())
			{
				//user can also call getPoint() and getSeparation() here
				const PxVec3& contactPoint = i.getPoint();
				if (jointFeedbackCount >= MAX_CONTACT_FEEDBACK){}
					//tprintf("Warning: too many contacts are established. Some of them will not be reported.\n");
				else{
					if (contactPoints.size() != jointFeedbackCount){
						//tprintf("Warning: Contact forces need to be cleared after each simulation, otherwise the results are not predictable.\n");
					}
					contactPoints.push_back(ContactPoint());
					//now we'll set up the feedback for this contact joint
					contactPoints[jointFeedbackCount].rb1 = rb1;
					contactPoints[jointFeedbackCount].rb2 = rb2;
					contactPoints[jointFeedbackCount].cp = Point3d(i.getPoint().x, i.getPoint().y, i.getPoint().z);
					PxReal pointnormalforce = i.getPointNormalForce();
					PxVec3 force = contactNormal * i.getPointNormalForce(); 
					Vector3d contactforce(force.x,force.y,force.z); 
					contactPoints[jointFeedbackCount].f = contactforce;
					//make sure that the force always points away from the static objects
					if (contactPoints[jointFeedbackCount].rb1->isLocked() && !contactPoints[jointFeedbackCount].rb2->isLocked()){
						contactPoints[jointFeedbackCount].f = contactPoints[jointFeedbackCount].f * (-1);
						RigidBody* tmpBdy = contactPoints[jointFeedbackCount].rb1;
						contactPoints[jointFeedbackCount].rb1 = contactPoints[jointFeedbackCount].rb2;
						contactPoints[jointFeedbackCount].rb2 = tmpBdy;
					}

					jointFeedbackCount++;
				}
			}
		}
	}
}

class MyCallback : public PxSimulationEventCallback
{
	public:
	PhysX3World *theworld;

	virtual void onContact(PxContactPair& pair, PxU32 events)
	{
		if(!theworld->testmode)
			theworld->collisionsPostProcessing(pair,events);
	}

}gMyCallback;

PxFilterFlags myFilterShader(
        PxFilterObjectAttributes attributes0, PxFilterData filterData0,
        PxFilterObjectAttributes attributes1, PxFilterData filterData1,
        PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	// let triggers through
	if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
	{
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT | PxPairFlag::eNOTIFY_TOUCH_PERSISTS;
		return PxFilterFlags();
	}

	if (filterData0.word0 & filterData1.word0) {
		//printf("filterData0: %d; filterData1: %d\n", filterData0.word0, filterData1.word0);
		return PxFilterFlag::eSUPPRESS;
	}
    // generate contacts for all that were not filtered above
	pairFlags = PxPairFlag::eCONTACT_DEFAULT;

	if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1)) {
		//printf("filterData0: %d, %d; filterData1: %d, %d\n", filterData0.word0, filterData0.word1, filterData1.word0, filterData1.word1);
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_PERSISTS|PxPairFlag::eNOTIFY_CONTACT_POINTS|
			PxPairFlag::eNOTIFY_THRESHOLD_FORCE_FOUND | PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST;
	}
	return PxFilterFlags();
}

void PhysX3World::setupFiltering(PxRigidActor* actor, PxU32 filterGroup, PxU32 filterMask)
{
	PxFilterData filterData;
	filterData.word0 = filterGroup; // word0 = own ID
	filterData.word1 = filterMask;	// word1 = ID mask to filter pairs that trigger a contact callback;
	const PxU32 numShapes = actor->getNbShapes();
	PxShape** shapes = new PxShape*[numShapes];
	actor->getShapes(shapes, numShapes);
	for(PxU32 i = 0; i < numShapes; i++)
	{
		PxShape* shape = shapes[i];
		shape->setSimulationFilterData(filterData);
	}
	//delete [] shapes;
}


void PhysX3World::setupWorld() 
{
	int maxCont = 4;

	//desc.flags &= ~PX_SDKF_NO_HARDWARE;
	bool recordMemoryAllocations = true;
	gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback, PxTolerancesScale(), recordMemoryAllocations);

	if(!PxInitExtensions(*gPhysicsSDK))
		printf("PxInitExtensions failed!");

	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, &(gPhysicsSDK->getFoundation()), PxCookingParams());
	if(!gCooking)
		printf("PxCreateCooking failed!");

	if(gPhysicsSDK->getPvdConnectionManager())
	{
		//printf("gPhysicsSDK->getPvdConnectionManager()\n");
		PVD::TConnectionFlagsType theConnectionFlags( PVD::PvdConnectionType::Debug | PVD::PvdConnectionType::Profile | PVD::PvdConnectionType::Memory );
		PxExtensionVisualDebugger::connect(gPhysicsSDK->getPvdConnectionManager(), "127.0.0.1", 5425, 10, true, PxDebuggerConnectionFlags( (PxU32)theConnectionFlags) );
	}

	// Create a scene
	PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());

	//set gravity
	Vector3d gravity = PhysicsGlobals::up * PhysicsGlobals::gravity;
	sceneDesc.gravity = PxVec3(gravity.x, gravity.y, gravity.z);
	//sceneDesc.gravity = PxVec3(0, 0, 0);

	sceneDesc.simulationEventCallback = &gMyCallback;
	if(!sceneDesc.cpuDispatcher)
	{
		PxU32 gNbThreads;
#if defined(PX_PS3)
		gNbThreads = 1;
#elif defined(PX_X360)
		gNbThreads = 2;
#else
		gNbThreads = 3;
#endif
		physx::pxtask::CpuDispatcher* gCpuDispatcher = PxDefaultCpuDispatcherCreate(gNbThreads);
        if(!gCpuDispatcher)
			printf("PxDefaultCpuDispatcherCreate failed!\n");
        sceneDesc.cpuDispatcher = gCpuDispatcher;
	}
	if(!sceneDesc.filterShader)
        sceneDesc.filterShader  = myFilterShader;
	//sceneDesc.flags |= PxSceneFlag::eENABLE_KINEMATIC_STATIC_PAIRS;

#ifdef PX_WINDOWS
	pxtask::CudaContextManagerDesc cudaContextManagerDesc;
	gCudaContextManager = pxtask::createCudaContextManager(cudaContextManagerDesc, &(gPhysicsSDK->getProfileZoneManager()));
#endif
#ifdef PX_WINDOWS
	if(!sceneDesc.gpuDispatcher && gCudaContextManager)
	{
		sceneDesc.gpuDispatcher = gCudaContextManager->getGpuDispatcher();
	}
#endif


	gScene = gPhysicsSDK->createScene(sceneDesc);

	gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE,	0.25f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eBODY_AXES, 1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eBODY_MASS_AXES, 1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eWORLD_AXES, 1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eACTOR_AXES, 1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_PAIRS, 1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eCONTACT_FORCE, 1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eCONTACT_POINT, 1.0f);
	//gScene->setVisualizationParameter(PxVisualizationParameter::eCONTACT_NORMAL, 1.0f);
	gMyCallback.theworld = this;
	gScene->setSimulationEventCallback(&gMyCallback);
	
	// Set default material
	defaultMaterial = gPhysicsSDK->createMaterial(1.0f, 1.0f, 0.0f);
	if(!defaultMaterial)
        printf("createMaterial failed!");

	maxContactCount = maxCont;

	pcQuery = NULL;

	pcQuery = new PreCollisionQuery();
	//printf("PhysX3World::setupWorld()\n");
}

void PhysX3World::destroyAllObjects() {
	destroyWorld();
	//setupWorld();
	if(testmode){
		//this->drawBoxes();
	}
}



/**
	this method is used to copy the state of the ith rigid body to its PhysX counterpart.
*/
void PhysX3World::setPhysXStateFromRB(int i){
	if (i<0 || (uint)i>=physxToRbs.size())
		return;

	//if it is a locked object, we update its CDPS
	if (physxToRbs[i].rb->isLocked() == true) {

		for (uint j=0;j<physxToRbs[i].collisionVolumes.size();j++){
			Point3d pos = physxToRbs[i].rb->getLocalCoordinates(physxToRbs[i].rb->state.position);
			PxQuat orient(physxToRbs[i].rb->state.orientation.v.x, physxToRbs[i].rb->state.orientation.v.y,
				physxToRbs[i].rb->state.orientation.v.z, physxToRbs[i].rb->state.orientation.s);
			PxTransform pose;
			pose.q = orient;
			pose.p = PxVec3(pos.x, pos.y, pos.z);
			PxShape * t = physxToRbs[i].collisionVolumes[j];
			t->setLocalPose(pose);
		}
		return;
	}
	
	PxTransform pose;
	pose.q = PxQuat(physxToRbs[i].rb->state.orientation.v.x,physxToRbs[i].rb->state.orientation.v.y,physxToRbs[i].rb->state.orientation.v.z,physxToRbs[i].rb->state.orientation.s);
	pose.p = PxVec3(physxToRbs[i].rb->state.position.x, physxToRbs[i].rb->state.position.y, physxToRbs[i].rb->state.position.z);
	physxToRbs[i].id->setGlobalPose(pose);
	physxToRbs[i].id->setLinearVelocity(PxVec3(physxToRbs[i].rb->state.velocity.x, physxToRbs[i].rb->state.velocity.y, physxToRbs[i].rb->state.velocity.z));
	physxToRbs[i].id->setAngularVelocity(PxVec3(physxToRbs[i].rb->state.angularVelocity.x, physxToRbs[i].rb->state.angularVelocity.y, physxToRbs[i].rb->state.angularVelocity.z));
}

/**
	this method is used to copy the state of the ith rigid body, from the PhysX object to its rigid body counterpart 
*/
void PhysX3World::setRBStateFromPhysX(int i){
	PxVec3 tempData;

	//if it is a locked object, we won't do anything about it
	if (physxToRbs[i].rb->isLocked() == true){
		//printf("rb locked\n");
		return;
	}

	//if the objects is supposed to be planar, make sure we don't let drift accumulate
	if (physxToRbs[i].rb->props.isPlanar){
		//printf("rb planar\n");
		PxVec3 rot = physxToRbs[i].id->getAngularVelocity();
		PxQuat quat_ptr;
		PxQuat quat;
		PxReal quat_len;
		quat_ptr = physxToRbs[i].id->getGlobalPose().q;
		PxVec3 p0 = physxToRbs[i].id->getGlobalPose().p;
		quat.w = quat_ptr.w;
		quat.x = quat_ptr.x;
		quat.y = 0; 
		quat.z = 0; 
		quat_len = sqrt( quat.w * quat.w + quat.x * quat.x );
		quat.w /= quat_len;
		quat.x /= quat_len;
		physxToRbs[i].id->setGlobalPose(PxTransform(p0, quat));
		rot.y = 0;
		rot.z = 0;
		physxToRbs[i].id->setAngularVelocity(rot);
	}
	//printf("before x : %f\n",physxToRbs[i].rb->state.position.x);
	//printf("before y : %f\n",physxToRbs[i].rb->state.position.y);
	//printf("before z : %f\n",physxToRbs[i].rb->state.position.z);

	tempData = physxToRbs[i].id->getGlobalPose().p;
	physxToRbs[i].rb->state.position.x = tempData.x;
	physxToRbs[i].rb->state.position.y = tempData.y;
    physxToRbs[i].rb->state.position.z = tempData.z;

	//printf("after x : %f\n",physxToRbs[i].rb->state.position.x);
	//printf("after y : %f\n",physxToRbs[i].rb->state.position.y);
	//printf("after z : %f\n",physxToRbs[i].rb->state.position.z);

	PxQuat orientation;
	orientation = physxToRbs[i].id->getGlobalPose().q;
	physxToRbs[i].rb->state.orientation.s = orientation.w;
	physxToRbs[i].rb->state.orientation.v.x = orientation.x;
	physxToRbs[i].rb->state.orientation.v.y = orientation.y;
	physxToRbs[i].rb->state.orientation.v.z = orientation.z;

	tempData = physxToRbs[i].id->getLinearVelocity();
	physxToRbs[i].rb->state.velocity.x = tempData.x;
	physxToRbs[i].rb->state.velocity.y = tempData.y;
	physxToRbs[i].rb->state.velocity.z = tempData.z;

	tempData = physxToRbs[i].id->getAngularVelocity();
	physxToRbs[i].rb->state.angularVelocity.x = tempData.x;
	physxToRbs[i].rb->state.angularVelocity.y = tempData.y;
	physxToRbs[i].rb->state.angularVelocity.z = tempData.z;
}

/**
	This method is used to set up a PhysX fixed joint, based on the information in the fixed joint passed in as a parameter
*/
void PhysX3World::setupPhysXFixedJoint(StiffJoint* hj){
	//printf("setupPhysXFixedJoint\n");

	PxRigidBody* actor1 = physxToRbs[(int)(hj->parent->id)].id;
	PxRigidBody* actor2 = physxToRbs[(int)(hj->child->id)].id;
	Point3d p = hj->pJPos;
	Point3d c = hj->cJPos;
	PxFixedJoint* fixedJoint=PxFixedJointCreate(*gPhysicsSDK, actor1, PxTransform(PxVec3(p.x, p.y, p.z)),
		actor2, PxTransform(PxVec3(c.x, c.y, c.z)));
	if(fixedJoint == NULL){
		printf("Failed to create fixed joint\n");
	}
}

/**
	This method is used to set up a PhysX hinge joint, based on the information in the hinge joint passed in as a parameter
*/
void PhysX3World::setupPhysXHingeJoint(HingeJoint* hj){
	//printf("setupPhysXHingeJoint\n");
	
	PxRevoluteJoint* revoluteJoint = PxRevoluteJointCreate(*gPhysicsSDK, physxToRbs[(int)(hj->parent->id)].id, PxTransform(PxVec3(hj->pJPos.x, hj->pJPos.y, hj->pJPos.z)),
		physxToRbs[(int)(hj->child->id)].id, PxTransform(PxVec3(hj->cJPos.x, hj->cJPos.y, hj->cJPos.z)));
	Point3d p = hj->child->getWorldCoordinates(hj->cJPos);
	Vector3d a = hj->parent->getWorldCoordinates(hj->a);
	PxSetJointGlobalFrame(*revoluteJoint, &PxVec3(p.x, p.y, p.z), &PxVec3(a.x, a.y, a.z));
	//revoluteJoint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);

	if(hj->useJointLimits == true && !testmode){
		revoluteJoint->setLimit(PxJointLimitPair(hj->minAngle, hj->maxAngle, 0.01f));
		//revoluteJoint->setLimit(PxJointLimitPair(-hj->maxAngle, -hj->minAngle, 0.01f));
		revoluteJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
	}
}


/**
	This method is used to set up a PhysX universal joint, based on the information in the universal joint passed in as a parameter
*/
void PhysX3World::setupPhysXUniversalJoint(UniversalJoint* uj){
	//printf("setupPhysXUniversalJoint\n");

	Point3d p = uj->pJPos;
	Point3d c = uj->cJPos;
	PxTransform pose1(PxVec3(p.x, p.y, p.z));
	PxTransform pose2(PxVec3(c.x, c.y, c.z));
	if(uj->useJointLimits == true && !testmode){
		// set rotation for the normal 
		Vector3d defA(0, 1, 0);
		Vector3d q = defA.crossProductWith(uj->a);
		q.toUnit();
		double rotAngle = defA.angleWith(uj->a);
		pose1.q = PxQuat(rotAngle, PxVec3(q.x, q.y, q.z));
		pose2.q = PxQuat(rotAngle, PxVec3(q.x, q.y, q.z));
	}

	PxD6Joint* d6Joint = PxD6JointCreate(*gPhysicsSDK, physxToRbs[(int)(uj->parent->id)].id, pose1,
		physxToRbs[(int)(uj->child->id)].id, pose2);
	//Point3d a = uj->parent->getWorldCoordinates(uj->pJPos);
	//Vector3d b = uj->child->getWorldCoordinates(uj->b);
	//PxSetJointGlobalFrame(*d6Joint, &PxVec3(a.x, a.y, a.z), &PxVec3(b.x, b.y, b.z));
	d6Joint->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
	d6Joint->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
	d6Joint->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
	d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);

	if(uj->useJointLimits == true && !testmode){
		d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
		d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
		d6Joint->setTwistLimit(PxJointLimitPair(uj->minAngleB, uj->maxAngleB, 0.01f));
		d6Joint->setSwingLimit(PxJointLimitCone(uj->maxAngleA, uj->maxAngleA, 0.01f));
	}

}

/**
	This method is used to set up a PhysX ball-and-socket joint, based on the information in the ball in socket joint passed in as a parameter
*/
void PhysX3World::setupPhysXBallAndSocketJoint(BallInSocketJoint* basj){
	//printf("setupPhysXBallAndSocketJoint\n");
	Point3d p = basj->pJPos;
	Point3d c = basj->cJPos;
	PxTransform pose1(PxVec3(p.x, p.y, p.z));
	PxTransform pose2(PxVec3(c.x, c.y, c.z));
	if(basj->useJointLimits == true && !testmode){
		// set rotation for the normal
		Vector3d defA(0, 1, 0);
		Vector3d q = defA.crossProductWith(basj->swingAxis1);
		q.toUnit();
		double rotAngle = defA.angleWith(basj->swingAxis1);
		pose1.q = PxQuat(rotAngle, PxVec3(q.x, q.y, q.z));
		pose2.q = PxQuat(rotAngle, PxVec3(q.x, q.y, q.z));
	}
	PxD6Joint* d6Joint = PxD6JointCreate(*gPhysicsSDK, physxToRbs[(int)(basj->parent->id)].id, pose1,
		physxToRbs[(int)(basj->child->id)].id, pose2);

	d6Joint->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
	d6Joint->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
	d6Joint->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
	d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

	if(basj->useJointLimits == true && !testmode){
		Point3d a = basj->child->getWorldCoordinates(basj->cJPos);
		Vector3d b = basj->child->getWorldCoordinates(basj->twistAxis);
		if(basj->useJointLimits == true && !testmode){
			PxSetJointGlobalFrame(*d6Joint, &PxVec3(a.x, a.y, a.z), &PxVec3(b.x, b.y, b.z));
		}
		d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
		if(basj->maxTwistAngle > 2 * 22 / 7){
			d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		}
		d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
		d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
		d6Joint->setTwistLimit(PxJointLimitPair(basj->minTwistAngle, basj->maxTwistAngle, 0.01f));
		d6Joint->setSwingLimit(PxJointLimitCone(basj->maxSwingAngle1, basj->maxSwingAngle2, 0.01f));
	}

}

/**
	this method is used to create PhysX shapes for all the collision primitives of the rigid body that is passed in as a paramter
*/
void PhysX3World::createPhysXCollisionPrimitives(RigidBody* body, int index){
	//printf("createPhysXCollisionPrimitives\n");
	//now we'll set up the body's collision detection primitives
	PxRigidDynamic* actor = gPhysicsSDK->createRigidDynamic(PxTransform(PxVec3(0,0,0)));
	PxRigidStatic* staticActor = gPhysicsSDK->createRigidStatic(PxTransform(PxVec3(0,0,0)));
	for (uint j=0;j<body->cdps.size();j++){
		int cdpType = body->cdps[j]->getType();

		if(cdpType == SPHERE_CDP){
			//printf("sphere\n");
			SphereCDP* s = (SphereCDP*)body->cdps[j];
			Point3d c = s->getCenter();
			PxReal radius = s->getRadius();
			PxShape* aSphereShape = actor->createShape(PxSphereGeometry(radius), *defaultMaterial, PxTransform(PxVec3(c.x,c.y,c.z)));
			aSphereShape->userData = body;
		} else if(cdpType == CAPSULE_CDP){
			//printf("capsule\n");
			CapsuleCDP* c = (CapsuleCDP*)body->cdps[j];
			Point3d a = c->getPoint1();
			Point3d b = c->getPoint2();
			Vector3d ab(a, b);
			Point3d cen = a + ab/2.0;
			PxTransform pose;
			pose.p = PxVec3(cen.x, cen.y, cen.z);
			Vector3d defA(1, 0, 0);
			Vector3d axis = defA.crossProductWith(ab);
			axis.toUnit();
			double rotAngle = defA.angleWith(ab);
			Quaternion relOrientation = Quaternion::getRotationQuaternion(rotAngle, axis);
			pose.q = PxQuat(relOrientation.v.x,relOrientation.v.y,relOrientation.v.z,relOrientation.s);
			PxShape* aCapsuleShape = actor->createShape(PxCapsuleGeometry(c->getRadius(), ab.length()/2), *defaultMaterial, pose);
			aCapsuleShape->userData = body;
		} else if(cdpType == BOX_CDP){
			//printf("box\n");
			BoxCDP* b = (BoxCDP*)body->cdps[j];
			Point3d c = b->getCenter();
			PxShape* aBoxShape = actor->createShape(PxBoxGeometry(b->getXLen() / 2, b->getYLen() / 2, b->getZLen() / 2), *defaultMaterial, PxTransform(PxVec3(c.x, c.y, c.z)));
			aBoxShape->userData = body;
		} else if(cdpType == PLANE_CDP){
			//printf("plane\n");
			PlaneCDP* p = (PlaneCDP*)body->cdps[j];
			PxTransform pose;
			Vector3d n = body->getWorldCoordinates(p->getNormal());
			Vector3d o = Vector3d(body->getWorldCoordinates(p->getOrigin()));
			Vector3d defA(1,0,0);
			Vector3d q = defA.crossProductWith(n);
			double rotAngle = defA.angleWith(n);
			pose.p = PxVec3(o.x, o.y, o.z);
			pose.q = PxQuat(rotAngle, PxVec3(q.x, q.y, q.z));
			staticActor = gPhysicsSDK->createRigidStatic(pose);
			PxShape* aShape = staticActor->createShape(PxPlaneGeometry(), *defaultMaterial);
			aShape->userData = body;
			physxToRbs[index].collisionVolumes.push_back(aShape);
		} else {
			//printf("error\n");
			return;
		}

		PxRigidBodyExt::updateMassAndInertia(*actor, 1);

	}

	if (body->isLocked() == true)
    {
		setupFiltering(staticActor, 1, 2);
		gScene->addActor(*staticActor);
	} else {
		actor->setMass(physxToRbs[index].rb->getMass());
		Vector3d principalMoments = physxToRbs[index].rb->getPMI();
		actor->setMassSpaceInertiaTensor(PxVec3(principalMoments.x,principalMoments.y,principalMoments.z));
		setupFiltering(actor, 2, 1);
		gScene->addActor(*actor);
		actor->setContactReportThreshold(100);
		actor->setContactReportThreshold(0);
		physxToRbs[body->id].id = actor;
		if(physxToRbs[body->id].id == NULL){
			printf("create actor failed\n");
		}
	}
}

/**
	This method reads a list of rigid bodies from the specified file.
*/
void PhysX3World::loadRBsFromFile(char* fName){
	//printf("code reached p\n");
	//make sure we don't go over the old articulated figures in case this method gets called multiple times.
	int index = objects.size();
	int index_afs = AFs.size();

	World::loadRBsFromFile(fName);

	// Add all non-articulated rigid bodies in ODE
	for (uint i=index;i<objects.size();i++){
		RigidBody* rigidBody = objects[i];
		// Add a placeholder in the odeToRbs mapping
		physxToRbs.push_back(PhysX3_RB_Map(0, rigidBody));
		if( !rigidBody->isArticulated() )
			linkRigidBodyToPhysX(objects.size()-1);
	}

	DynamicArray<Joint*> joints;

	// Check every newly added articulated figures
	for (uint i=index_afs;i<AFs.size();i++){

		// For each, add the articulated bodies they contain to PhysX	
		for (uint j=0;j<objects.size();j++){
			if( !objects[j]->isArticulated() )
				continue;
			ArticulatedRigidBody* arb = (ArticulatedRigidBody*)objects[j];
			if( arb->getAFParent() == AFs[i] )
				linkRigidBodyToPhysX(j);
		}

		//now we will go through all the new joints, and create and link their PhysX equivalents
		joints.clear();
		AFs[i]->addJointsToList(&joints);
		for (uint j=0;j<joints.size();j++){
			//connect the joint to the two bodies
			int jointType = joints[j]->getJointType();
			switch (jointType){
				case STIFF_JOINT:
					setupPhysXFixedJoint((StiffJoint*)joints[j]);
					break;
				case BALL_IN_SOCKET_JOINT:
					setupPhysXBallAndSocketJoint((BallInSocketJoint*)joints[j]);
					break;
				case HINGE_JOINT:
					setupPhysXHingeJoint((HingeJoint*)joints[j]);
					break;
				case UNIVERSAL_JOINT:
					setupPhysXUniversalJoint((UniversalJoint*)joints[j]);
					break;
				default:
					return;
			}
		}
	}
}

/**
	This method adds one rigid body (not articulated).
*/
void PhysX3World::addRigidBody( RigidBody* rigidBody ) {
	//printf("code reached q\n");
	World::addRigidBody(rigidBody);

	// Add a placeholder in the physxToRbs mapping
	physxToRbs.push_back(PhysX3_RB_Map(0, rigidBody));

	// Non-articulated rigid body are already well-positioned, link them to PhysX
	if( !rigidBody->isArticulated() )
		linkRigidBodyToPhysX(objects.size()-1);
	// For articulated rigid bodies, we will only add them when (and if) they appear in an ArticulatedFigure
}

/**
	This method adds one rigid body (not articulated).
*/
void PhysX3World::addArticulatedFigure(ArticulatedFigure* articulatedFigure){
	World::addArticulatedFigure( articulatedFigure );

	// Add the articulated bodies contained into that figure to PhysX		
	for (uint j=0;j<objects.size();j++){
		if( !objects[j]->isArticulated() )
			continue;
		ArticulatedRigidBody* arb = (ArticulatedRigidBody*)objects[j];
		if( arb->getAFParent() == articulatedFigure )
			linkRigidBodyToPhysX(j);
	}

	DynamicArray<Joint*> joints;

	//now we will go through all the new joints, and create and link their PhysX equivalents
	articulatedFigure->addJointsToList(&joints);
	for (uint j=0;j<joints.size();j++){
		//connect the joint to the two bodies
		int jointType = joints[j]->getJointType();
		switch (jointType){
			case STIFF_JOINT:
				setupPhysXFixedJoint((StiffJoint*)joints[j]);
				break;
			case BALL_IN_SOCKET_JOINT:
				setupPhysXBallAndSocketJoint((BallInSocketJoint*)joints[j]);
				break;
			case HINGE_JOINT:
				setupPhysXHingeJoint((HingeJoint*)joints[j]);
				//setupPhysXHingeJoint((HingeJoint*)joints[j]);
				break;
			case UNIVERSAL_JOINT:
				setupPhysXUniversalJoint((UniversalJoint*)joints[j]);
				//setupPhysXUniversalJoint((UniversalJoint*)joints[j]);
				break;
			default:
				return;
		}
	}

	for (uint j=0;j<objects.size();j++){
		if( !objects[j]->isArticulated() )
			continue;
		for(uint k = 0; k<j;k++){
			if( !objects[k]->isArticulated() )
				continue;
			//gScene->setActorPairFlags(*(physxToRbs[objects[j]->id].id),*(physxToRbs[objects[k]->id].id),PX_IGNORE_PAIR);
		}
	}
	updateCharacterHeight();
}


/**
	This methods creates an PhysX object and links it to the passed RigidBody
*/
void PhysX3World::linkRigidBodyToPhysX( int index ) {
	//printf("code reached s\n");
	RigidBody* rigidBody = physxToRbs[index].rb; 

	if(!rigidBody->isLocked()){
		rigidBody->setBodyID( index );
	}
	//CREATE AND LINK THE PhysX BODY WITH OUR RIGID BODY
	//PROCESS THE COLLISION PRIMITIVES OF THE BODY
	createPhysXCollisionPrimitives(rigidBody, index);


	//if the body is fixed, we'll only create the colission detection primitives
	if (!rigidBody->isLocked()){
		rigidBody->setBodyID( index );
		//set the data
		physxToRbs[index].id->userData = (void*) index;
	}

	//if this is a planar object, make sure we constrain it to always stay planar
	if (rigidBody->props.isPlanar){
		PxD6Joint* pip = PxD6JointCreate(*gPhysicsSDK, NULL, PxTransform(), physxToRbs[index].id, PxTransform());
		pip->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
		pip->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);
		pip->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		pip->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		pip->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	}

	//SET THE INERTIAL PARAMETERS

	if (rigidBody->isLocked() == false){
		//set the mass and principal moments of inertia for this object
		//physxToRbs[index].id->setMass(physxToRbs[index].rb->getMass());
		//Vector3d principalMoments = physxToRbs[index].rb->getPMI();
		//physxToRbs[index].id->setMassSpaceInertiaTensor(PxVec3(principalMoments.x,principalMoments.y,principalMoments.z));
		setPhysXStateFromRB(index);
	}
}


/**
	This method is used to set the state of all the rigid body in this collection.
*/
void PhysX3World::setState(DynamicArray<double>* state, int start){
	//printf("code reached u\n");
	World::setState(state, start);
}


/**
	This method is a simple call back function that passes the message to the world whose objects are being acted upon. 
*/
void collisionCallBack(void* PhysX3World, PxShape * o1, PxShape * o2){
	//printf("code reached v\n");
	//((PhysX3World*)PhysX3World)->processCollisions(o1,o2);
}

/**
	This method is used to integrate the forward simulation in time.
*/
void PhysX3World::advanceInTime(double deltaT){
	//printf("advanceInTime\n");
	/*countto1000++;
	LARGE_INTEGER ticksPerSecond;
	QueryPerformanceFrequency(&ticksPerSecond);
	LARGE_INTEGER tickbegin;
	LARGE_INTEGER tickend;
	LARGE_INTEGER time;
	QueryPerformanceFrequency(&ticksPerSecond);
	QueryPerformanceCounter(&tickbegin);*/

	//printf("code reached x\n");
	if( deltaT <= 0 )
		return;

	//make sure that the state of the RB's is synchronized with the engine...
	setEngineStateFromRB();

	//restart the counter for the joint feedback terms
	jointFeedbackCount = 0;

	//go through all the rigid bodies in the world, and apply their external force
	for (uint j=0;j<objects.size();j++){
		if( objects[j]->isLocked() ) 
			continue;
		const Vector3d& f = objects[j]->externalForce;
		if( !f.isZeroVector() )
			physxToRbs[objects[j]->id].id->addForce(PxVec3(f.x, f.y, f.z));
		const Vector3d& t = /*Vector3d(0,0,1);//*/objects[j]->externalTorque;
		if( !t.isZeroVector() )
			//if(objects[j]->id == 1)
			physxToRbs[objects[j]->id].id->addTorque(PxVec3(t.x, t.y, t.z));
	}

	//go through all the joints in the world, and apply their torques to the parent and child rb's
	for (uint j=0;j<jts.size();j++){
		Vector3d t = jts[j]->torque;
		physxToRbs[jts[j]->parent->id].id->addTorque(PxVec3((float)t.x,(float)t.y, (float)t.z));
		physxToRbs[jts[j]->child->id].id->addTorque(PxVec3((float)-t.x,(float)-t.y, (float)-t.z));
	}

	//clear the previous list of contact forces
	contactPoints.clear();
	//gScene->setTiming(deltaT, 1, PX_TIMESTEP_FIXED);
	gScene->simulate(deltaT);
	gScene->fetchResults(true);

	//copy over the state of the PhysX bodies to the rigid bodies...
	setRBStateFromEngine();

	//printf("%d\n",countto1000);
	/*QueryPerformanceCounter(&tickend);
	after1000 += ((float)tickend.QuadPart - tickbegin.QuadPart)/ticksPerSecond.QuadPart;
	if(countto1000 >= 1000){
		printf("time of simulation : %f s\n",after1000/countto1000);
		after1000 = 0.0f;
		countto1000 = 0;
	}*/
}

/**
		This method is for performance analysis
	*/
void PhysX3World::printAllCOMPosition(){
	//printf("printing com position PhysX\n");
}

/**
	this method is used to transfer the state of the rigid bodies, from PhysX to the rigid body wrapper
*/
void PhysX3World::setRBStateFromEngine(){
	//printf("code reached y\n");
	//now update all the rigid bodies...
	for (uint i=0;i<objects.size();i++){
		setRBStateFromPhysX(i);
//		objects[i]->updateToWorldTransformation();
	}
}

/**
	this method is used to transfer the state of the rigid bodies, from the rigid body wrapper to PhysX's rigid bodies
*/
void PhysX3World::setEngineStateFromRB(){
	//printf("code reached z\n");
	//now update all the rigid bodies...
	for (uint i=0;i<objects.size();i++){
		setPhysXStateFromRB(i);
	}
}

/**
	this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
	and the force is also specified in local coordinates.
*/
void PhysX3World::applyRelForceTo(RigidBody* b, const Vector3d& f, const Point3d& p){
	//printf("code reached a1\n");
	if (!b)
		return;
	PxRigidBodyExt::addLocalForceAtLocalPos(*(physxToRbs[b->id].id), PxVec3(f.x, f.y, f.z), PxVec3(p.x, p.y, p.z));
}

/**
	this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
	and the force is specified in world coordinates.
*/
void PhysX3World::applyForceTo(RigidBody* b, const Vector3d& f, const Point3d& p){
	//printf("code reached a2\n");
	if (!b)
		return;
	PxRigidBodyExt::addForceAtLocalPos(*(physxToRbs[b->id].id), PxVec3(f.x, f.y, f.z), PxVec3(p.x, p.y, p.z));
}

/**
	this method applies a torque to a rigid body. The torque is specified in world coordinates.
*/
void PhysX3World::applyTorqueTo(RigidBody* b, const Vector3d& t){
	//printf("code reached a3\n");
	if (!b)
		return;
	physxToRbs[b->id].id->addTorque(PxVec3(t.x, t.y, t.z));
	//dBodyAddTorque(odeToRbs[b->id].id, t.x, t.y, t.z);
}