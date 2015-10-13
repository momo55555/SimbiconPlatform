// #pragma once
// 
// #include "PxSetupEnv.h"
// #include "math_3d.h"
// 
// using namespace Math3D;
// 
// namespace Physics
// {
//     class CPlane
//     {
//     public:
//         CPlane(PhysicsContext* PhysicsContext_)
//         {
//             mPhysicsContext = PhysicsContext_;
//         }
// 
//         void createPlane(Vector3f pos, Vector3f rotation, PxReal staticFriction, PxReal dynamicFriction, PxReal restitution);
// 
//     private:
//         PhysicsContext* mPhysicsContext;
//         PxRigidStatic* mPlane;
//     };
// }