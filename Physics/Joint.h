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

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <typeinfo>

#include <Vector3d.h>
#include <Quaternion.h>

#define STIFF_JOINT 1
#define HINGE_JOINT 2
#define BALL_IN_SOCKET_JOINT 3
#define UNIVERSAL_JOINT 4

/*=======================================================================================================================================================================*
 * This class is responsible with the implementation of the methods that are neccessary to implement joints in an articulated system. The joints impose constraints on   *
 * the articulated rigid bodies that they connect. Each joint will be used to link a parent body to a child body. The joints that will be considered, for now at least,  *
 * are all rotationals joints with 1, 2 or 3 degrees of freedom. The default type of joint is a Ball in Socket joint with no joint limits.                               *
 *=======================================================================================================================================================================*/
class World;
class ArticulatedRigidBody;
class Joint {
friend class Controller;
friend class ArticulatedFigure;
friend class ArticulatedRigidBody;
friend class BallInSocketJoint;
friend class StiffJoint;
friend class HingeJoint;
friend class UniversalJoint;
friend class PhysX3World;
friend class Character;
friend class SimBiController;
friend class IKVMCController;
friend class PoseController;
friend class VirtualModelController;
friend class BehaviourController;
friend class TestApp;
friend class TestApp2;
protected:
	//this is the parent link
	ArticulatedRigidBody* parent;
	//this is the location of the joint on the parent body - expressed in the parent's local coordinates
	Point3d pJPos;
	
	//this is the child link
	ArticulatedRigidBody* child;
	//this is the location of the joint on the child body - expressed in the child's local coordinates 
	//NOTE: the locations of the parent and child joint locations must overlap in world coordinates
	Point3d cJPos;
	//this variable is used to indicate if this joint has joint limits or not (the details regarding the limits are specified on a per joint type basis)
	bool useJointLimits;
	//the torque applied to this joint. It should be set/reset by a controller acting on this joint.
	Vector3d torque;
	//this is the name of the joint
	char name[100];

	//this is the id of the joint...
	int id;

	//when a joint limit is reached, this variable controls the bounciness of the "collision"
	double epsBounce;


	//these are temporary vectors - allocated only once for efficiency.
	Vector3d tmpV1, tmpV2, tmpV3, tmpV4, tmpV5;

	//and some tmp quaternions
	Quaternion tmpQ1, tmpQ2, tmpQ3, tmpQ4;



	/**
		This method is used to fix the joint angular constraint to correct for drift. This is done by changing
		the orientation of the child relative to the parent
	*/
	virtual void fixAngularConstraint(const Quaternion& qRel) = 0;

	/**
		This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
		been read from an input file.
	*/
	virtual void readAxes(char* axes);

	/**
		This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
		have been read from an input file.
	*/
	virtual void readJointLimits(char* limits);

public:
	/**
		Default constructor
	*/
	Joint(void);

    	/**
		This method is used to compute the relative orientation between the parent and the child rigid bodies, expressed in 
		the frame coordinate of the parent.
	*/
	void computeRelativeOrientation(Quaternion& qRel);

	/**
		Default destructor
	*/
	virtual ~Joint(void);

	const char* typeName(){
		return typeid(*this).name();
	}

	/**
		This method is used to automatically fix the errors in the joints (i.e. drift errors caused by numercial integration). At some future
		point it can be changed into a proper stabilization technique.
	*/
	void fixJointConstraints(bool fixOrientations, bool fixVelocities, bool recursive);

	/**
		This method is used to load the details of a joint from file. The PhysicalWorld parameter points to the world in which the objects
		that need to be linked live in.
	*/
	void loadFromFile(FILE* fp, World* world);

	/**
		Returns the type of the current joint
	*/
	virtual int getJointType() = 0;

	/**
		sets the torque
	*/
	inline void setTorque(const Vector3d& t){torque = t;}

	/**
		retrieves the reference to the body's parent
	*/
	inline ArticulatedRigidBody* getParent(){return parent;}

	/**
		set the parent
	*/
	void setParent( ArticulatedRigidBody* parent );
	/**
		retrieves the reference to the child's parent
	*/
	inline ArticulatedRigidBody* getChild(){return child;}

	/**
		set the chil
	*/
	void setChild( ArticulatedRigidBody* child );

	/**
		returns the position of the child joint, expressed in child's coordinates
	*/
	inline const Point3d& getChildJointPosition(){return cJPos;}

	/**
		sets the position of the child joint, expressed in child's coordinates
	*/
	inline void setChildJointPosition( const Point3d& cJPos ){
		this->cJPos = cJPos;
	}

	/**
		returns the position of the parent joint, expressed in parent's coordinates
	*/
	inline const Point3d& getParentJointPosition(){return pJPos;}

	/**
		sets the position of the parent joint, expressed in parent's coordinates
	*/
	inline void setParentJointPosition( const Point3d& pJPos ){
		this->pJPos = pJPos;
	}

	/**
		returns the name of this joint
	*/
	inline const char* getName() { return name; }

	/**
		sets the name
	*/
	inline void setName( const char* name ) {
		strncpy_s( this->name, name, 99 );
	}


};