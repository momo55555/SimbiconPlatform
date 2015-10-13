// #pragma once
// 
// #include "PxSetupEnv.h"
// #include "math_3d.h"
// 
// using namespace Math3D;
// 
// namespace Physics
// {
//     class CRigidBody
//     {
//     public:
//         CRigidBody(PhysicsContext* _PhysicsContext)
//         {
//             mPhysicsContext = _PhysicsContext;
//         }
// 
//         void createRigidBody(Vector3f pos, Vector3f rotation, Vector3f geometry, PxReal staticFriction, PxReal dynamicFriction, PxReal restitution);
//         void createRigidBody(PxTransform transform_, Vector3f geometry, PxReal staticFriction, PxReal dynamicFriction, PxReal restitution, bool isStatic);
// 
//         PxRigidActor* getRigidBodyEntity()
//         {
//             return gBox;
//         }
// 
//     private:
//         PhysicsContext* mPhysicsContext;
//         PxRigidActor* gBox;
//         
//     };
// }