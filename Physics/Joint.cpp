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

#include "stdafx.h"

#include "Joint.h"
#include "RBUtils.h"
#include "World.h"
#include "BallInSocketJoint.h"
#include "StiffJoint.h"
#include "HingeJoint.h"
#include "UniversalJoint.h"
#include "ArticulatedFigure.h"
#include <PUtils.h>

Joint::Joint(void){
	this->parent = NULL;
	this->child = NULL;
	useJointLimits = false;
	torque = Vector3d(0,0,0);
	this->id = -1;
	epsBounce = 0.0;
}

Joint::~Joint(void){

}

/**
	This method is used to compute the relative orientation between the parent and the child rigid bodies, expressed in 
	the frame coordinate of the parent.
*/
void Joint::computeRelativeOrientation(Quaternion& qRel){
	//if qp is the quaternion that gives the orientation of the parent, and qc gives the orientation of the child, then  qp^-1 * qc gives the relative
	//orientation between the child and the parent, expressed in the parent's coordinates (child to parent)
	qRel = parent->state.orientation.getComplexConjugate() * child->state.orientation;
}



/**
	This method is used to automatically fix the errors in the joints (i.e. drift errors caused by numercial integration). At some future
	point it can be changed into a proper stabilization technique.
*/
void Joint::fixJointConstraints(bool fixOrientations, bool fixVelocities, bool recursive){
	if (!child)
		return;

	//if it has a parent, we will assume that the parent's position is correct, and move the children to satisfy the joint constraint
	if (parent){
		//fix the orientation problems here... hopefully no rigid body is locked (except for the root, which is ok)
		
		//first fix the relative orientation, if desired
		if (fixOrientations){
			Quaternion qRel;
			computeRelativeOrientation(qRel);
			fixAngularConstraint(qRel);
		}

		//now worry about the joint positions

		//compute the vector rc from the child's joint position to the child's center of mass (in world coordinates)
		Vector3d rc = child->getWorldCoordinates(Vector3d(cJPos, Point3d(0,0,0)));
		//and the vector rp that represents the same quanity but for the parent
		Vector3d rp = parent->getWorldCoordinates(Vector3d(pJPos, Point3d(0,0,0)));

		//the location of the child's CM is now: pCM - rp + rc
		child->state.position = parent->state.position + (rc - rp);

		//fix the velocities, if need be
		if (fixVelocities){
			//to get the relative velocity, we note that the child rotates with wRel about the joint (axis given by wRel
			//d = vector from joint position to CM of the child),
			//but it also rotates together with the parent with the parent's angular velocity, 
			//so we need to combine these (all velocities are expressed in world coordinates already) (r is the vector
			//from the CM of the parent, to that of the child).
			Vector3d wRel = child->state.angularVelocity - parent->state.angularVelocity;
			Vector3d r = Vector3d(parent->state.position, child->state.position);
			Vector3d d = Vector3d(child->getWorldCoordinates(cJPos), child->state.position);
			Vector3d vRel = parent->state.angularVelocity.crossProductWith(r) + wRel.crossProductWith(d);
			child->state.velocity = parent->state.velocity + vRel;
		}
	}

	//make sure that we recursivley fix all the other joint constraints in the articulated figure
	if (recursive)
		for (uint i=0;i<child->cJoints.size();i++)
			child->cJoints[i]->fixJointConstraints(fixOrientations, fixVelocities, recursive);
}

/**
	This method is used to load the details of a joint from file. The PhysicalWorld parameter points to the world in which the objects
	that need to be linked live in.
*/
void Joint::loadFromFile(FILE* f, World* world){
	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	char tempName[100];

	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		fgets(buffer, 200, f);
		char *line = lTrim(buffer);
		int lineType = getRBLineType(line);
		switch (lineType) {
			case RB_NAME:
				sscanf(line, "%s", this->name);
				break;
			case RB_PARENT:
				sscanf(line, "%s", tempName);
				parent = world->getARBByName(tempName);
				break;
			case RB_CHILD:
				sscanf(line, "%s", tempName);
				child = world->getARBByName(tempName);
				break;
			case RB_CPOS:
				sscanf(line, "%lf %lf %lf",&cJPos.x, &cJPos.y, &cJPos.z);
				break;
			case RB_PPOS:
				sscanf(line, "%lf %lf %lf",&pJPos.x, &pJPos.y, &pJPos.z);
				break;
			case RB_END_JOINT:
				//we now have to link together the child and parent bodies
				parent->cJoints.push_back(this);
				child->pJoint = this;
				return;//and... done
				break;
			case RB_JOINT_LIMITS:
				readJointLimits(line);
				break;
			case RB_NOT_IMPORTANT:
				break;
			default:
				break;
		}
	}
}

/**
	This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
	been read from an input file.
*/
void Joint::readAxes(char* axes){
}

/**
	This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
	have been read from an input file.
*/
void Joint::readJointLimits(char* limits){
}


void Joint::setParent( ArticulatedRigidBody* parent ){

	this->parent = parent;
	parent->cJoints.push_back(this);
}

/**
	set the chil
*/
void Joint::setChild( ArticulatedRigidBody* child ){
	this->child = child;
	child->pJoint = this;
}


