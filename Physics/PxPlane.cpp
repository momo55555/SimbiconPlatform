 #include "stdafx.h"
// 
// #include "PxPlane.h"
// 
// using namespace Physics;
// using namespace Math3D;
// 
// void CPlane::createPlane(Vector3f pos, Vector3f rotation, PxReal staticFriction, PxReal dynamicFriction, PxReal restitution)
// {
// 
//     PxPhysics* gPhysicsSDK = mPhysicsContext->getSDK();
//     PxScene* gScene = mPhysicsContext->getScene();
//     //Creating PhysX material (staticFriction, dynamicFriction, restitution)
//     PxMaterial* material = gPhysicsSDK->createMaterial(staticFriction, dynamicFriction, restitution);
// 
//     //---------Creating actors-----------]
// 
//     //1-Creating static plane that will act as ground
//     PxTransform planePos =	PxTransform(PxVec3(0.0), PxQuat(PxHalfPi, PxVec3(0.0, 0.0, 1.0)));	//Position and orientation(transform) for plane actor  
//     mPlane =  gPhysicsSDK->createRigidStatic(planePos);    //Creating rigid static actor	
//     mPlane->createShape(PxPlaneGeometry(), *material);	   //Defining geometry for plane actor
//     gScene->addActor(*mPlane);	 //Adding plane actor to PhysX scene
// }
// 