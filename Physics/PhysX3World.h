#pragma once

#include <PxPhysicsAPI.h>
#include <PxExtensionsAPI.h>
#include <PxCudaContextManager.h>
#include <PxProfileZonemanager.h>
#include <PvdConnectionManager.h>

#define NOMINMAX
#include "World.h"
#include "CollisionDetectionPrimitive.h"
#include "SphereCDP.h"  
#include "CapsuleCDP.h"
#include "BoxCDP.h"
#include "PlaneCDP.h"
#include "PreCollisionQuery.h"

#define MAX_CONTACT_FEEDBACK 200

using namespace physx;

//this structure is used to map a rigid body to the id of its PhysX counterpart
typedef struct PhysX3_RB_Map_struct{
	PxRigidBody* id;
	RigidBody* rb;
	DynamicArray<PxShape*> collisionVolumes; // Only used for rigid bodies
	PhysX3_RB_Map_struct() : id(NULL), rb(NULL) {}
	PhysX3_RB_Map_struct(PxRigidBody* newId, RigidBody* newRb){ this->id = newId; this->rb = newRb;}
} PhysX3_RB_Map;

class PhysX3World : public World
{
	friend class MyCallback;
private :
	// PhysX's id for the simulation scene	
	PxPhysics* gPhysicsSDK;
	PxScene* gScene;
	PxCooking* gCooking;
	PxMaterial* defaultMaterial;
	physx::pxtask::CudaContextManager* gCudaContextManager;

	//PhysX's static actor for collision space
	PxActor * spaceID;

	//keep track of the mapping between the rigid bodies and their PhysX counterparts with this
	DynamicArray<PhysX3_RB_Map> physxToRbs;

	//this is the max number of contacts that are going to be processed between any two objects
	int maxContactCount;

	PxContactPair jointFeedback[MAX_CONTACT_FEEDBACK];
	//this is the current number of contact joints, for the current step of the simulation
	int jointFeedbackCount;

	//this is a pointer to a physical interface object that is used as an abstract way of communicating between the simulator and the application
	PreCollisionQuery* pcQuery;

	/**
		this method is used to specify pair of objects that generate a report when touched
	*/
	void setupFiltering(PxRigidActor* actor, PxU32 filterGroup, PxU32 filterMask);

	/**
		this method is used to copy the state of the ith rigid body to its PhysX counterpart.
	*/
	void setPhysXStateFromRB(int i);

	/**
		this method is used to copy the state of the ith rigid body, from the PhysX object to its rigid body counterpart 
	*/
	void setRBStateFromPhysX(int i);

	/**
		This method is used to set up a PhysX fixed joint, based on the information in the fixed joint passed in as a parameter
	*/
	void setupPhysXFixedJoint(StiffJoint* hj);

	/**
		This method is used to set up a PhysX hinge joint, based on the information in the hinge joint passed in as a parameter
	*/
	void setupPhysXHingeJoint(HingeJoint* hj);

	/**
		This method is used to set up a PhysX universal joint, based on the information in the universal joint passed in as a parameter
	*/
	void setupPhysXUniversalJoint(UniversalJoint* uj);

	/**
		This method is used to set up a PhysX ball-and-socket joint, based on the information in the ball in socket joint passed in as a parameter
	*/
	void setupPhysXBallAndSocketJoint(BallInSocketJoint* basj);

	/**
		this method is used to create shapes for all the collision primitives of the rigid body that is passed in as a paramter
	*/
	void createPhysXCollisionPrimitives(RigidBody* body, int index);

	/**
		This method reads a list of rigid bodies from the specified file.
	*/
	virtual void loadRBsFromFile(char* fName);

	/**
		This methods creates a PhysX object and links it to the RigidBody corresponding to objects[index]
	*/
	void linkRigidBodyToPhysX( int index );

	/**
		this method is used to process the collision between the two objects passed in as parameters. More generally,
		it is used to determine if the collision should take place, and if so, it calls the method that generates the
		contact points.
	*/
	void collisionsPostProcessing(PxContactPair & pair, PxU32 events);

	/**
		this method is used to transfer the state of the rigid bodies, from the simulator to the rigid body wrapper
	*/
	virtual void setRBStateFromEngine();

	/**
		this method is used to transfer the state of the rigid bodies, from the rigid body wrapper to the simulator's rigid bodies
	*/
	virtual void setEngineStateFromRB();

	// Destroy the world, it becomes unusable, but everything is clean
	virtual void destroyWorld();

	// Setup the world as it should
	void setupWorld();

public :
	/**
		default constructor
	*/
	PhysX3World();

	/**
		destructor
	*/
	virtual ~PhysX3World(void);

	// Destroy all the objects, but the world is still usable
	virtual void destroyAllObjects();

	/**
		This method adds one rigid body (articulated or not).
	*/
	virtual void addRigidBody( RigidBody* rigidBody_disown );

	/**
		This method adds one articulated figure.
	*/
	virtual void addArticulatedFigure( ArticulatedFigure* articulatedFigure_disown );

	/**
		This method is used to set the state of all the rigid body in the physical world.
	*/
	void setState(DynamicArray<double>* state, int start = 0);

	/**
		This method is used to integrate the forward simulation in time.
	*/
	virtual void advanceInTime(double deltaT);

/**
		This method is for performance analysis
	*/
	virtual void printAllCOMPosition();

	/**
		this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
		and the force is also specified in local coordinates.
	*/
	virtual void applyRelForceTo(RigidBody* b, const Vector3d& f, const Point3d& p);

	/**
		this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
		and the force is specified in world coordinates.
	*/
	virtual void applyForceTo(RigidBody* b, const Vector3d& f, const Point3d& p);

	/**
		this method applies a torque to a rigid body. The torque is specified in world coordinates.
	*/
	virtual void applyTorqueTo(RigidBody* b, const Vector3d& t);

	/**
		draw additional information
	*/

	friend class MyCallback;

};