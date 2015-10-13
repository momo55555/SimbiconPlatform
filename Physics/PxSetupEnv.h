// #pragma once
// 
// //#include <PxPhysicsAPI.h>
// 
// using namespace physx;
// 
// namespace Physics
// {
//     class PhysicsContext
//     {
//     public:
//         PhysicsContext();
// 
//         void init();
//         
//         void shutdown();
// 
//         PxPhysics* getSDK()
//         {
//             return gPhysicsSDK;
//         }
// 
//         PxScene* getScene()
//         {
//             return gScene;
//         }
//     
//         PxFoundation* getFoundation()
//         {
//             return gFoundation;
//         }
//         
//         PxReal gTimeStep;
// 
//     private:
//         PxPhysics* gPhysicsSDK;
//         PxFoundation* gFoundation;
//         //PxDefaultErrorCallback gDefaultErrorCallback;
//         //PxDefaultAllocator gDefaultAllocatorCallback;
//         PxScene* gScene;
//         //PxVisualDebuggerConnection* theConnection;
//     };
// }