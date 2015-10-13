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

#include "RigidBody.h"
#include "RBUtils.h"
#include "CapsuleCDP.h"
#include "PlaneCDP.h"
#include "BoxCDP.h"
#include "SphereCDP.h"

#include <PUtils.h>

/**
	Default constructor - give sensible values to the class members
*/
RigidBody::RigidBody(void){
	name[0] = '\0';
//	toWorld.loadIdentity();
}

/**
	Default destructor - free up all the memory that we've used up
*/
RigidBody::~RigidBody(void){
	for (uint i=0;i<cdps.size();i++)
		delete cdps[i];

}

/**
	This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
*/
Point3d RigidBody::getWorldCoordinates(const Point3d& localPoint){
	return this->state.position + getWorldCoordinates(Vector3d(localPoint));
}

/**
	This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
*/
Vector3d RigidBody::getWorldCoordinates(const Vector3d& localVector){
	//the rigid body's orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
	return this->state.orientation.rotate(localVector);
}

/**
	This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
*/
Point3d RigidBody::getLocalCoordinates(const Point3d& globalPoint){
	Vector3d v = getLocalCoordinates(Vector3d(globalPoint)) - getLocalCoordinates(Vector3d(this->state.position));
	return Point3d(0,0,0) + v;
}

/**
	This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
*/
Vector3d RigidBody::getLocalCoordinates(const Vector3d& globalVector){
	//the rigid body's orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
	return this->state.orientation.getComplexConjugate().rotate(globalVector);
}

/**
	This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
	resulting velocity will be expressed in world coordinates.
*/
Vector3d RigidBody::getAbsoluteVelocityForLocalPoint(const Point3d& localPoint){
	//we need to compute the vector r, from the origin of the body to the point of interest
	Vector3d r(Point3d(), localPoint);
	//the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
	return state.angularVelocity.crossProductWith(getWorldCoordinates(r)) + state.velocity;
}

/**
	This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in global coordinates, and the
	resulting velocity will be expressed in world coordinates.
*/
Vector3d RigidBody::getAbsoluteVelocityForGlobalPoint(const Point3d& globalPoint){
	//we need to compute the vector r, from the origin of the body to the point of interest
	Vector3d r(state.position, globalPoint);
	//the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
	return state.angularVelocity.crossProductWith(r) + state.velocity;
}
/**
	this method is used to update the world positions of the collision detection primitives
*/
void RigidBody::updateWorldCDPs(){
	for (uint i=0;i<cdps.size();i++)
		cdps[i]->updateToWorldPrimitive();
}

/**
	This method renders the rigid body in its current state as a set of vertices 
	and faces that will be appended to the passed OBJ file.

	vertexIdxOffset indicates the index of the first vertex for this object, this makes it possible to render
	multiple different meshes to the same OBJ file
	 
	Returns the number of vertices written to the file
*/
uint RigidBody::renderToObjFile(FILE* fp, uint vertexIdxOffset) {
	int retVal = 0;
//	for (uint i=0;i<meshes.size();i++)
//		retVal = meshes[i]->renderToObjFile( fp, vertexIdxOffset, this->toWorld );
	return retVal;
}

/**
	This method is used to compute the correct toWorld coordinates matrix based on the state of the rigid body
*/
//void RigidBody::updateToWorldTransformation(){
//	this->state.orientation.getRotationMatrix(&toWorld);
//	toWorld.setTranslation(this->state.position);
//}

/**
	This method loads all the pertinent information regarding the rigid body from a file.
*/
void RigidBody::loadFromFile(FILE* f){

    //have a temporary buffer used to read the file line by line...
    char buffer[200];
    char meshName[200];

    //temporary variables that we may end up populating
    double r, g, b, a;
    Point3d p1, p2;
    Vector3d n;
    double t1, t2, t3;
    double t;

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
        case RB_MASS:
            if (sscanf(line, "%lf", &t)!=1)
                return;
            this->props.setMass(t);
            break;
        case RB_MOI:
            if (sscanf(line, "%lf %lf %lf", &t1, &t2, &t3)!=3)
                return;
            if (t1<=0 || t2<=0 || t3<=0)
                return;
            this->props.setMOI(t1, t2, t3);
            break;
        case RB_END_RB:
            return;//and... done
            break;
        case RB_SPHERE:
            if (sscanf(line, "%lf %lf %lf %lf", &p1.x, &p1.y, &p1.z, &r)!=4)
                return;
            cdps.push_back(new SphereCDP(p1, r, this));
            break;
        case RB_CAPSULE:
            if (sscanf(line, "%lf %lf %lf %lf %lf %lf %lf", &p1.x, &p1.y, &p1.z, &p2.x, &p2.y, &p2.z, &r)!=7)
                return;
            cdps.push_back(new CapsuleCDP(p1, p2, r, this));
            break;
        case RB_BOX:
            if (sscanf(line, "%lf %lf %lf %lf %lf %lf", &p1.x, &p1.y, &p1.z, &p2.x, &p2.y, &p2.z)!=6)
               return;
            cdps.push_back(new BoxCDP(p1, p2, this));
            break;
        case RB_PLANE:
            if (sscanf(line, "%lf %lf %lf %lf %lf %lf", &n.x, &n.y, &n.z, &p1.x, &p1.y, &p1.z)!=6)
                return;
            cdps.push_back(new PlaneCDP(n, p1, this));
            break;
        case RB_NOT_IMPORTANT:
            if (strlen(line)!=0 && line[0] != '#')
                printf("Ignoring input line: \'%s\'\n", line);
            break;
        case RB_LOCKED:
            this->props.lockBody();
            break;
        case RB_POSITION:
            if (sscanf(line, "%lf %lf %lf", &state.position.x, &state.position.y, &state.position.z)!=3)
                return;
            break;
        case RB_ORIENTATION:
            if (sscanf(line, "%lf %lf %lf %lf", &t, &t1, &t2, &t3)!=4)
                return;
            state.orientation = Quaternion::getRotationQuaternion(t, Vector3d(t1, t2, t3).toUnit()) * state.orientation;
            break;
        case RB_VELOCITY:
            if (sscanf(line, "%lf %lf %lf", &state.velocity.x, &state.velocity.y, &state.velocity.z)!=3)
                return;
            break;
        case RB_ANGULAR_VELOCITY:
            if (sscanf(line, "%lf %lf %lf", &state.angularVelocity.x, &state.angularVelocity.y, &state.angularVelocity.z)!=3)
               return;
            break;
        case RB_FRICTION_COEFF:
            if (sscanf(line, "%lf", &props.mu)!=1)
                return;
            if (props.mu<0)
                return;
            break;
        case RB_RESTITUTION_COEFF:
            if (sscanf(line, "%lf", &props.epsilon)!=1)
                return;
            if (props.epsilon<0 || props.epsilon>1)
                return;
            break;
        case RB_ODE_GROUND_COEFFS:
            if (sscanf(line, "%lf %lf", &t1, &t2)!=2)
                return;
            props.groundSoftness = t1;
            props.groundPenalty = t2;
            break;
        case RB_PLANAR:
            props.isPlanar = true;
            break;
        default:
            return;
            break;
        }
    }
    return;
}