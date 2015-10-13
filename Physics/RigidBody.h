/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <PUtils.h>


#include <TransformationMatrix.h>

#include "RBState.h"
#include "RBProperties.h"
#include "CollisionDetectionPrimitive.h"
#include "RBForceAccumulator.h"

class Force;
class ArticulatedFigure;


//define some drawing flags:
#define SHOW_MESH				0x0001
#define SHOW_BODY_FRAME			0x0002
#define SHOW_CD_PRIMITIVES		0x0008
#define SHOW_MIN_BDG_SPHERE		0x0010
#define SHOW_JOINTS				0x0020
#define SHOW_COLOURS			0x0040
#define SHOW_FRICTION_PARTICLES 0x0080
#define SHOW_ABSTRACT_VIEW		0x0100
#define SHOW_ABSTRACT_VIEW_SKELETON 0x0200

/*=========================================================================================================================================================================*
 | This is the implementation of a Rigid Body class. It holds all the attributes that characterize the rigid body (state information, collision detection primitives, etc).|
 | This class is used as the basis for an Articulated Rigid Body. Together with the PhysicalWorld class, this class is used to implement the dynamics of rigid bodies.     |
 | NOTE: It is assumed that the location of the center of mass of the object in local coordinates is (0,0,0) and that the principal moments of inertia are aligned to the  |
 | local coordinates x, y, and z axes!                                                                                                                                     |
 *=========================================================================================================================================================================*/

class RigidBody 
{
friend class World;
friend class HingeJoint;
friend class UniversalJoint;
friend class BallInSocketJoint;
friend class ODEWorld;
friend class PhysXWorld;
friend class PhysX3World;
friend class Character;
friend class SimBiController;
friend class Joint;
friend class PoseController;
friend class BipV3BalanceController;
friend class TestApp;
friend class TestApp2;
friend class VirtualModelController;
friend class BehaviourController;
friend class PhysXWorld;
friend class PhysX3World;
friend class MyContactReport;
friend class BulletWorld;
friend class VortexWorld;
friend class AbstractRBEngine;

protected:
	//--> the state of the rigid body: made up of the object's position in the world, its orientation and linear/angular velocities (stored in world coordinates)
	RBState state;
	//--> the physical properties of the rigid bodies: mass and inertia, stored in a convenient to use form
	RBProperties props;

	//An external force that can be applied to the rigid body CM
	Vector3d externalForce;
	Vector3d externalTorque;

	//--> an array with all the collision detection primitives that are relevant for this rigid body
	DynamicArray<CollisionDetectionPrimitive*> cdps;
	//--> the name of the rigid body - it might be used to reference the object for articulated bodies
	char name[100];
	//--> the id of the rigid body
	int id;

	//--> this transformation matrix is used to transform points/vectors from local coordinates to global coordinates. It will be updated using the state
	//information, and is therefore redundant, but it will be used to draw the object quickly. Everytime the state is updated, this matrix must also be updated!
//	TransformationMatrix toWorld;

public:
	/**
		Default constructor
	*/
	RigidBody(void);

	/**
		Default destructor
	*/
	virtual ~RigidBody(void);

	const char* typeName(){
		return typeid(*this).name();
	}

	void setExternalForce( const Vector3d& ef ) { externalForce = ef; }

	void setExternalTorque( const Vector3d& et ) { externalTorque = et; }

	const Vector3d& getExternalForce() const { return externalForce; }

	const Vector3d& getExternalTorque() const { return externalTorque; }

	/**
		This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	*/
	Point3d getWorldCoordinates(const Point3d& localPoint);

	/**
		This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
	*/
	Point3d getLocalCoordinates(const Point3d& globalPoint);

	/**
		This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
	*/
	Vector3d getLocalCoordinates(const Vector3d& globalVector);

	/**
		This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	*/
	Vector3d getWorldCoordinates(const Vector3d& localVector);

	/**
		This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
		resulting velocity will be expressed in world coordinates.
	*/
	Vector3d getAbsoluteVelocityForLocalPoint(const Point3d& localPoint);

	/**
		This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in global coordinates, and the
		resulting velocity will be expressed in world coordinates.
	*/
	Vector3d getAbsoluteVelocityForGlobalPoint(const Point3d& globalPoint);


	/**
		This method returns the world coordinates of the position of the center of mass of the object
	*/
	inline Point3d getCMPosition(){
		return state.position;
	}

	/**
		This method sets the world coordinate of the posision of the center of mass of the object
	*/
	inline void setCMPosition(const Point3d& newCMPos){
		state.position = newCMPos;		
	}

	/**
		This method returns the body's center of mass velocity
	*/
	inline Vector3d getCMVelocity(){
		return state.velocity;
	}

	inline void setOrientation(double angle, Vector3d axis) {
		state.orientation = Quaternion::getRotationQuaternion(angle, axis.toUnit()) * state.orientation;
	}

	/**
		This method sets the velocity of the center of mass of the object
	*/
	inline void setCMVelocity(const Vector3d& newCMVel){
		state.velocity = newCMVel;
	}

	/**
		this method sets the angular velocity of the body
	*/
	inline void setAngularVelocity(const Vector3d& newAVel){
		state.angularVelocity = newAVel;
	}

	/**
		This method returns the rigid body's coefficient of restitution
	*/
	inline double getRestitutionCoefficient(){
		return props.epsilon;
	}

	/**
		This method returns the rigid body's coefficient of restitution
	*/
	inline double getFrictionCoefficient(){
		return props.mu;
	}

	/**
		This method draws the current rigid body.
	*/

	/**
		This method renders the rigid body in its current state as a set of vertices 
		and faces that will be appended to the passed OBJ file.

		vertexIdxOffset indicates the index of the first vertex for this object, this makes it possible to render
		multiple different meshes to the same OBJ file
		 
		Returns the number of vertices written to the file
	*/
	uint renderToObjFile(FILE* fp, uint vertexIdxOffset);


	/**
		This method loads all the pertinent information regarding the rigid body from a file.
	*/
	void loadFromFile(FILE* fp);

	/**
		This method sets the rigid body name
	*/
	void setName( char* name ) {
		strncpy_s( this->name, name, 99 );
	}

	const char* getName() const {
		return name;
	}

	/**
		This method loads an OBJ mesh and associates it with the rigid body
	*/
	void addMeshObj( char* objFilename, const Vector3d& offset = Vector3d(0,0,0), const Vector3d& scale = Vector3d(1,1,1) );

	/**
		This method sets the colour of the last mesh loaded
	*/
	void setColour( double r, double g, double b, double a );

	int getCDPCount() const {
		return cdps.size();
	}

	CollisionDetectionPrimitive* getCDP(unsigned int index) const {
		if( index > cdps.size() )
			return NULL;
		return cdps[index];
	}


	/**
		This method sets the rigid body mass
	*/
	void setMass( double mass ) {
		props.setMass( mass );
	}

	/**
		This method sets the rigid body principal moments of intertia
	*/
	void setMOI( Vector3d moi ) {
		props.setMOI( moi.x, moi.y, moi.z );
	}

	const Vector3d& getMOI() const { return props.getMOI(); }

	/**
		This method adds a collision detection primitive
	*/
	void addCollisionDetectionPrimitive( CollisionDetectionPrimitive* cdp_disown ) {
		cdp_disown->attachBody( this );
		cdps.push_back( cdp_disown );
	}
	
	CollisionDetectionPrimitive* getCollisionDetectionPrimitive( int index ) {
		return cdps[index];
	}

	int getCollisionDetectionPrimitiveCount() const {
		return cdps.size();
	}
	
	/**
		This method makes it possible to lock the body
	*/
	void lockBody(bool isLocked = true) {
		props.lockBody(isLocked);
	}

	/**
		This method lets the user set the body friction coefficient
	*/
	void setFrictionCoefficient( double frictionCoefficient ) {
		if (frictionCoefficient<0)
		props.mu = frictionCoefficient;
	}

	/**
		This method lets the user set the body friction coefficient
	*/
	void setRestitutionCoefficient( double restitutionCoefficient ) {
		if (restitutionCoefficient<0 || restitutionCoefficient>1)
		props.epsilon = restitutionCoefficient;
	}

	double getGroundSoftness() const { return props.groundSoftness; }
	double getGroundPenalty() const { return props.groundPenalty; }

	/**
		This method lets the user set the body friction coefficient
	*/
	void setODEGroundCoefficients( double softness, double penalty ) {
		props.groundSoftness = softness;
		props.groundPenalty = penalty;
	}
	
	double getODEGroundSoftness() const { return props.groundSoftness; }
	double getODEGroundPenalty() const { return props.groundPenalty; }

	/**
		This method indicates that the rigid body is planar
	*/
	void setPlanar(bool planar) {
		props.isPlanar = planar;
	}

	bool isPlanar() const { return props.isPlanar; }

	/**
		Returns the mass of the rigid body
	*/
	inline double getMass(){
		return props.mass;
	}

	/**
		This method is used to compute the correct toWorld coordinates matrix based on the state of the rigid body
	*/
//	void updateToWorldTransformation();

	/**
		this method sets the id of the current rigid body.
	*/
	inline void setBodyID(int newID){
		this->id = newID;
	}

	/**
		this method returns the body's principal moments of inertia, about the principal axes (which correspond to the bodie's local coordinate
		frame axes)
	*/
	inline Vector3d getPMI(){
		return props.MOI_local;
	}

	/**
		this method returns the body's orientation
	*/
	inline Quaternion getOrientation(){
		return state.orientation;
	}

	/**
		this method sets the body's orientation
	*/
	inline void setOrientation(Quaternion q){
		state.orientation = q;
	}

	/**
		this method returns the body's angular velocity
	*/
	inline Vector3d getAngularVelocity(){
		return state.angularVelocity;
	}

	/**
		this method returns true if the current body is locked, false otherwised
	*/
	inline bool isLocked(){
		return props.isLocked;
	}

	/**
		this method returns false if this body is a simple rigid bory, false if it is an articulated figure
	*/
	virtual bool isArticulated(){
		return false;
	}

	/**
		this method is used to update the world positions of the collision detection primitives
	*/
	void updateWorldCDPs();

	virtual ArticulatedFigure* getAFParent(){
		return NULL;
	}

};
