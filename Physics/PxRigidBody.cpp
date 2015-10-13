 #include "stdafx.h"
// 
// #include "PxRigidBody.h"
// 
// using namespace Physics;
// 
// void CRigidBody::createRigidBody(Vector3f pos,  Vector3f rotation, Vector3f geometry, PxReal staticFriction, PxReal dynamicFriction, PxReal restitution)
// {
//     PxPhysics* gPhysicsSDK = mPhysicsContext->getSDK();
//     PxScene* gScene = mPhysicsContext->getScene();
// 
//     Matrix4f m;
//     m.InitRotateTransform(rotation.x, rotation.y, rotation.z);
//     Quaternions q(0,0,0,0);
//     Matrix4f2Quat(m, q);
//     q.Normalize();
//     
//     if(abs(q.x) < eps && abs(q.y) < eps && abs(q.z) < eps) q.z = 1.0f;
// 
//     PxMaterial* mat = gPhysicsSDK->createMaterial(staticFriction, dynamicFriction, restitution);
//     PxTransform boxPos(PxVec3(pos.x, pos.y, pos.z), PxQuat(q.w*2, PxVec3(q.x, q.y, q.z)));
//     PxBoxGeometry boxGeometry(geometry.x, geometry.y, geometry.z);
//     gBox = PxCreateDynamic(*gPhysicsSDK, boxPos, boxGeometry, *mat, 1.0f);
//     
//     //gScene->addActor(*gBox);
// }
// 
// void CRigidBody::createRigidBody(PxTransform transform_, Vector3f geometry, PxReal staticFriction, PxReal dynamicFriction, PxReal restitution, bool isStatic)
// {
//     PxPhysics* gPhysicsSDK = mPhysicsContext->getSDK();
//     PxScene* gScene = mPhysicsContext->getScene();
// 
//     PxMaterial* mat = gPhysicsSDK->createMaterial(staticFriction, dynamicFriction, restitution);
//     PxBoxGeometry boxGeometry(geometry.x, geometry.y, geometry.z);
//     //if(!isStatic)
//     gBox = PxCreateDynamic(*gPhysicsSDK, transform_, boxGeometry, *mat, 1.0f);
//     //else gBox = PxCreateStatic(*gPhysicsSDK, transform_, boxGeometry, *mat);
//     //gScene->addActor(*gBox);
// }