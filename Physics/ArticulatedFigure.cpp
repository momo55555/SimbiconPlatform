#include "stdafx.h"

#include "ArticulatedFigure.h"
#include "World.h"
#include "RBUtils.h"
#include <PUtils.h>
/**
	Default constructor
*/
ArticulatedFigure::ArticulatedFigure(void){
	root = NULL;
	name[0] = '\0';
	mass = 0;
}

ArticulatedFigure::~ArticulatedFigure(void){
	//delete all the joints
	if (root != NULL)
		delete root;
	for (uint i=0;i<arbs.size();i++)
		delete arbs[i];
	for (uint i=0;i<joints.size();i++)
		delete joints[i];
	arbs.clear();
	joints.clear();
}

void ArticulatedFigure::loadIntoWorld() {
	World& world = World::instance();

	if( root == NULL ) return;
	world.addRigidBody(root);
	for (uint i=0;i<arbs.size();i++)
		world.addRigidBody(arbs[i]);
}

/**
	Sets the root
*/
void ArticulatedFigure::setRoot( ArticulatedRigidBody* root ) {
	if (this->root != NULL) return;
    root->setAFParent( this );
	this->root = root;
}

/**
	Sets the root
*/
void ArticulatedFigure::addArticulatedRigidBody( ArticulatedRigidBody* arb ) {
    arb->setAFParent( this );
	arbs.push_back( arb );
}

/**
	This method is used to automatically fix the errors in the joints (i.e. drift errors caused by numercial integration). At some future
	point it can be changed into a proper stabilization technique.
*/
void ArticulatedFigure::fixJointConstraints(bool fixOrientations, bool fixVelocities){
	if (!root)
		return;

	for (uint i=0;i<root->cJoints.size();i++)
		root->cJoints[i]->fixJointConstraints(fixOrientations, fixVelocities, true);
}


/**
	This method is used to compute the total mass of the articulated figure.
*/
void ArticulatedFigure::computeMass(){
	double curMass = root->getMass();
	double totalMass = curMass;

	for (uint i=0; i < joints.size(); i++){
		curMass = joints[i]->child->getMass();
		totalMass += curMass;
	}

	mass = totalMass;
}

void ArticulatedFigure::addJointsToList(DynamicArray<Joint*> *joints)
{
    if (!root)
        return;
    DynamicArray<ArticulatedRigidBody*> bodies;
    bodies.push_back(root);

    int currentBody = 0;

    while ((uint)currentBody<bodies.size()){
        //add all the children joints to the list
        for (uint i=0;i<bodies[currentBody]->cJoints.size();i++){
            joints->push_back(bodies[currentBody]->cJoints[i]);
            bodies.push_back(bodies[currentBody]->cJoints[i]->child);
        }
        currentBody++;
    }
}

/**
	This method is used to get the total mass of the articulated figure.
*/
double ArticulatedFigure::getMass(){
	return mass;
}

/**
	This method is used to load the details of an articulated figure from file. The PhysicalWorld parameter points to the world in which the objects
	that need to be linked live in.
*/
void ArticulatedFigure::loadFromFile(FILE* f, World* world){
    //have a temporary buffer used to read the file line by line...
    char buffer[200];
    char tempName[100];
    Joint* tempJoint;

    //this is where it happens.
    while (!feof(f)){
        //get a line from the file...
        fgets(buffer, 200, f);

        char *line = lTrim(buffer);
        int lineType = getRBLineType(line);
        switch (lineType) {
        case RB_ROOT:
            sscanf(line, "%s", tempName);
            root = world->getARBByName(tempName);

            break;
        case RB_JOINT_TYPE_UNIVERSAL:
            tempJoint = new UniversalJoint();
            tempJoint->readAxes( line );
            tempJoint->loadFromFile(f, world);
            tempJoint->child->AFParent = this;
            tempJoint->parent->AFParent = this;
            break;
        case RB_JOINT_TYPE_HINGE:
            tempJoint = new HingeJoint();
            tempJoint->readAxes( line );
            tempJoint->loadFromFile(f, world);
            tempJoint->child->AFParent = this;
            tempJoint->parent->AFParent = this;
            break;
        case RB_JOINT_TYPE_BALL_IN_SOCKET:
            tempJoint = new BallInSocketJoint();
            tempJoint->readAxes( line );
            tempJoint->loadFromFile(f, world);
            tempJoint->child->AFParent = this;
            tempJoint->parent->AFParent = this;
            break;
        case RB_END_ARTICULATED_FIGURE:
            //make sure that the root does not have a parent, otherwise we'll end up with loops in the articulated figure]
            return;//and... done
            break;
        case RB_NOT_IMPORTANT:
            break;
        default:
            break;
        }
    }
}