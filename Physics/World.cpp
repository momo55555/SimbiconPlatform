//#include <Physics/PhysXWorld.h>
//#define NOMINMAX
#include "stdafx.h"

#include <cstdlib>
#include <assert.h>

#include "World.h"
#include "RBUtils.h"
#include <PUtils.h>
#include <string>
#include <fstream>
#include <iostream>
#include "NullWorld.h"

#include "PhysX3World.h"

#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )

// Singleton stuff
World* World::_instance = NULL;
void World::create() {
	bool test = false;
	//World::testmode = false;
	assert( _instance == NULL );
	
	std::string line;
	int engine;
	std::ifstream myfile("..\\controllerconfig.txt");
	if (myfile.is_open())
	{
		std::getline (myfile,line);
		char *a=new char[line.size()+1];
		memcpy(a,line.c_str(),line.size());
		if(a[0] == 't'){
			test = true;
		}
		a[line.size()]=0;
		std::getline (myfile,line);
		a=new char[line.size()+1];
		a[line.size()]=0;
		memcpy(a,line.c_str(),line.size());
		engine = atoi(a);
		//delete [] a;
		//printf("simulation engine : %c\n",a[0]);
		if(a[0] == 'o' || a[0] == 'O' || a[0] == 'q' || a[0] == 'Q'){
#ifdef ODE
			_instance = new ODEWorld();
#else
			printf("ODE needs to be defined in compileconfig.h before it is instantiated... exiting...\n");
			exit(0);
#endif
			/*if(a[0] == 'q' || a[0] == 'Q'){
				quick = true;
			} else {
				quick = false;
			}*/
		} else if(a[0] == 'p' || a[0] == 'P'){
			if(a[1] == '3'){
			_instance = new PhysX3World();

			}else{
#ifdef PhysX
			_instance = new PhysXWorld();
#else
			printf("PhysX needs to be defined in compileconfig.h before it is instantiated... exiting...\n");
			exit(0);
#endif
			}
		} else if(a[0] == 'v' || a[0] == 'V'){
#ifdef Vortex
			_instance = new VortexWorld();
#else
			printf("Vortex needs to be defined in compileconfig.h before it is instantiated... exiting...\n");
			exit(0);
#endif
		} else if(a[0] == 'b' || a[0] == 'B'){
#ifdef Bullet
			_instance = new BulletWorld();
#else
			printf("Bullet needs to be defined in compileconfig.h before it is instantiated... exiting...\n");
			exit(0);
#endif
		} else{
			printf("Please specify a valid simulation engine\n");
			printf("'o' for ODE (or 'oq' for ODE Quick)\n");
			printf("'p' for PhysX\n");
			printf("'b' for Bullet\n");
			printf("'v' for Vortex\n");
			exit(0);
		}
		_instance->testmode = test;
		_instance->testmode = false;
		_instance->nperturb = 0;
	}
	myfile.close();

	std::atexit( World::destroy );
}

void World::destroy() {
	assert( _instance != NULL );
	delete _instance;
}



World::World(void){
	this->objects = DynamicArray<RigidBody*>(300);
	this->objects.clear();

	//for performance analysis
	nbFrames = 0;
	stepCounter = 0;
    testmode = false;
	//this->frame = DynamicArray<Vector3d>(20);
	//this->frame.clear();
}

World::~World(void){
	destroyWorld();
}

void World::destroyWorld() {

	//delete all the rigid bodies in this world
	for (uint i=0;i<objects.size();i++) {
		if( objects[i]->isArticulated() ) {
			if ( ((ArticulatedRigidBody*)objects[i])->getAFParent() != NULL )
				continue; // Articulated figure is responsible of deleting this object
		}
		delete objects[i];
	}
	objects.clear();

	ABs.clear();

	//delete the references to the articulated figures that we hold as well
	for (uint i=0;i<AFs.size();i++)
		delete AFs[i];
	AFs.clear();

	jts.clear();

	contactPoints.clear();
}

void World::destroyAllObjects() {
	destroyWorld();
}


/**
	This method returns the reference to the first articulated rigid body with 
	its name and its articulared figure name, or NULL if it is not found
*/
ArticulatedRigidBody* World::getARBByName(char* name, char* articulatedFigureName){
	if (name == NULL)
		return NULL;
	for (uint i=0;i<ABs.size();i++)
		if (strcmp(name, ABs[i]->name) == 0)
			if( articulatedFigureName == NULL ||
				strcmp( articulatedFigureName, ABs[i]->getAFParent()->getName() ) == 0 )
			return ABs[i];
	return NULL;
}

/**
	This method returns the reference to the first articulated rigid body with 
	its name and its articulared figure name, or NULL if it is not found
*/
ArticulatedRigidBody* World::getARBByName(char* name, const ArticulatedFigure* articulatedFigure){
	if (name == NULL)
		return NULL;
	for (uint i=0;i<ABs.size();i++)
		if (strcmp(name, ABs[i]->name) == 0 && articulatedFigure == ABs[i]->getAFParent() )
			return ABs[i];
	return NULL;
}

/**
	This method returns the reference to the rigid body with the given name, or NULL if it is not found
*/
RigidBody* World::getRBByName(char* name){
	if (name == NULL)
		return NULL;
	for (uint i=0;i<this->objects.size();i++)
		if (strcmp(name, objects[i]->name) == 0)
			return objects[i];
	return NULL;
}

/**
	This method reads a list of rigid bodies from the specified file.
*/
void World::loadRBsFromFile(char* fName){

	FILE *f = fopen(fName, "r");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	RigidBody* newBody = NULL;
	ArticulatedFigure* newFigure = NULL;
	//this is where it happens.
	while (fgets(buffer, 200, f)){
		//get a line from the file...
		char *line = lTrim(buffer);
		int lineType = getRBLineType(line);
		switch (lineType) {
			case RB_RB:
				//create a new rigid body and have it load its own info...
				newBody = new RigidBody();
				newBody->loadFromFile(f);
				objects.push_back(newBody);
				break;
			case RB_ARB:
				//create a new articulated rigid body and have it load its own info...
				newBody = new ArticulatedRigidBody();
				newBody->loadFromFile(f);
				objects.push_back(newBody);
				//remember it as an articulated rigid body to be able to link it with other ABs later on
				ABs.push_back((ArticulatedRigidBody*)newBody);
				break;
			case RB_ARTICULATED_FIGURE:
				//we have an articulated figure to worry about...
                newFigure = new ArticulatedFigure();
				AFs.push_back(newFigure);
				newFigure->loadFromFile(f, this);
				newFigure->addJointsToList(&jts);
				break;
			case RB_NOT_IMPORTANT:
				break;
			default:
                break;
		}
	}
    fclose(f);

	//now we'll make sure that the joint constraints are satisfied
	for (uint i=0;i<AFs.size();i++)
		AFs[i]->fixJointConstraints();

	//and now make sure that each rigid body's toWorld transformation is updated
//	for (uint i=0;i<objects.size();i++){
//		objects[i]->updateToWorldTransformation();
//	}
}

/**
	This method adds one rigid body (not articulated).
*/
void World::addRigidBody(RigidBody* rigidBody){
	objects.push_back(rigidBody);
	if( rigidBody->isArticulated() )
		ABs.push_back((ArticulatedRigidBody*)rigidBody);
}


int nchar = 0;
/**
	This method adds one rigid body (not articulated).
*/
void World::addArticulatedFigure(ArticulatedFigure* articulatedFigure){
	nchar++;
	printf("Creating character %d\n",nchar);
	articulatedFigure->loadIntoWorld();
	AFs.push_back(articulatedFigure);
	articulatedFigure->addJointsToList(&jts);
	articulatedFigure->fixJointConstraints();
}

/**
	This method is used to get the state of all the rigid body in this collection.
*/
void World::getState(DynamicArray<double>* state){
	for (uint i=0;i<this->objects.size();i++){
		state->push_back(objects[i]->state.position.x);
		state->push_back(objects[i]->state.position.y);
		state->push_back(objects[i]->state.position.z);

		state->push_back(objects[i]->state.orientation.s);
		state->push_back(objects[i]->state.orientation.v.x);
		state->push_back(objects[i]->state.orientation.v.y);
		state->push_back(objects[i]->state.orientation.v.z);

		state->push_back(objects[i]->state.velocity.x);
		state->push_back(objects[i]->state.velocity.y);
		state->push_back(objects[i]->state.velocity.z);

		state->push_back(objects[i]->state.angularVelocity.x);
		state->push_back(objects[i]->state.angularVelocity.y);
		state->push_back(objects[i]->state.angularVelocity.z);
	}
}

/**
	This method is used to set the state of all the rigid body in this collection.
*/
void World::setState(DynamicArray<double>* state, int start){
	int i = start;
	for (uint j=0;j<this->objects.size();j++){
		objects[j]->state.position = Point3d((*state)[i+0], (*state)[i+1], (*state)[i+2]);
		i+=3;
		objects[j]->state.orientation = Quaternion((*state)[i+0], (*state)[i+1], (*state)[i+2], (*state)[i+3]);
		i+=4;
		objects[j]->state.velocity = Vector3d((*state)[i+0], (*state)[i+1], (*state)[i+2]);
		i+=3;
		objects[j]->state.angularVelocity = Vector3d((*state)[i+0], (*state)[i+1], (*state)[i+2]);
		i+=3;
//		objects[j]->updateToWorldTransformation();
	}
}

bool World::addStepCounter(){
	stepCounter++;
	if(stepCounter>=10 && stepCounter < 32)
		return true;
	else 
		return false;
	//printf("Number of steps : %d\n",stepCounter);
}

bool World::countFrames(){
	return (stepCounter >= 10);
}

void World::getCOMPositions(){
	//printf("number of objects : %d\n",objects.size());
	RigidBody * root;
	for (uint j=0;j<objects.size();j++){
		if(objects[j]->name[0] == 'p' && objects[j]->name[1] == 'e' && objects[j]->name[2] == 'l' && objects[j]->name[3] == 'v'){
			root = objects[j];
		}
	}
	uint iter = 0;
	for (uint j=0;j<objects.size();j++){
		if( objects[j]->isLocked() ) 
			continue;
		walkcycle[nbFrames][iter] = root->getLocalCoordinates(objects[j]->getCMPosition());
		//printf("Local CM position : %f,%f,%f\n",localcom.x,localcom.y,localcom.z);
		iter++;
	}
}

void World::addNbFrames(){
	if(countFrames()){
		getCOMPositions();
		nbFrames++;
	}
}

void World::resetNbFrames(){
	if(stepCounter % 2 == 0 && stepCounter >= 12 && stepCounter <= 40){
		if(stepCounter >= 12 && stepCounter <= 16){
		std::ofstream myfile;
		myfile.open ("example.txt",std::ios::app);
		myfile<<nbFrames<<std::endl;
		printf("Number of steps : %d\n",stepCounter);
		//printf("Number of frames in this walk cycle : %d\n",nbFrames);
		for(int i = 0; i < nbFrames;i++){
			for(int j = 0;j < 16;j++){
				myfile<<walkcycle[i][j].x<<" "<<walkcycle[i][j].y<<" "<<walkcycle[i][j].z<<std::endl;
				//printf("local com pos : %f,%f,%f\n",walkcycle[i][j].x,walkcycle[i][j].y,walkcycle[i][j].z);
			}
		}
		myfile.close();
		} else if(stepCounter >= 18 && stepCounter <= 22){
			std::ofstream myfile;
			myfile.open ("example1.txt",std::ios::app);
			myfile<<nbFrames<<std::endl;
			printf("Number of steps : %d\n",stepCounter);
			//printf("Number of frames in this walk cycle : %d\n",nbFrames);
			for(int i = 0; i < nbFrames;i++){
				for(int j = 0;j < 16;j++){
					myfile<<walkcycle[i][j].x<<" "<<walkcycle[i][j].y<<" "<<walkcycle[i][j].z<<std::endl;
					//printf("local com pos : %f,%f,%f\n",walkcycle[i][j].x,walkcycle[i][j].y,walkcycle[i][j].z);
				}
			}
		myfile.close();
		} else if(stepCounter >= 24 && stepCounter <= 28){
			std::ofstream myfile;
			myfile.open ("example2.txt",std::ios::app);
			myfile<<nbFrames<<std::endl;
			printf("Number of steps : %d\n",stepCounter);
			//printf("Number of frames in this walk cycle : %d\n",nbFrames);
			for(int i = 0; i < nbFrames;i++){
				for(int j = 0;j < 16;j++){
					myfile<<walkcycle[i][j].x<<" "<<walkcycle[i][j].y<<" "<<walkcycle[i][j].z<<std::endl;
					//printf("local com pos : %f,%f,%f\n",walkcycle[i][j].x,walkcycle[i][j].y,walkcycle[i][j].z);
				}
			}
		myfile.close();
		}

		nbFrames = 0;
		if(stepCounter == 16){
			this->perturb = true;
		}
		//char c;
		//scanf("%c",&c);
	}
}

double World::updateCharacterHeight(){
	RigidBody * head;
	for (uint j=0;j<objects.size();j++){
		if(objects[j]->name[0] == 'h' && objects[j]->name[1] == 'e' && objects[j]->name[2] == 'a' && objects[j]->name[3] == 'd'){
			head = objects[j];
		}
	}

	for (uint j=0;j<head->cdps.size();j++){
		int cdpType = head->cdps[j]->getType();
		if(cdpType == SPHERE_CDP){
			height = head->state.position.y + ((SphereCDP*)head->cdps[j])->getRadius();
			//printf("Character height : %f(head y pos), %f(head radius), %f(total)\n", head->state.position.y,((SphereCDP*)head->cdps[j])->getRadius(),height);
		}
	}
	return height;
}



